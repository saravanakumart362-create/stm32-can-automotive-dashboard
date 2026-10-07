/* Virtual 5-node CAN bus + ECU-01 UART link, in the browser.
 *
 * JavaScript port of dashboard/config.py and dashboard/virtual_ecu.py. The bus
 * and the dashboard still talk in the firmware's 6-byte UART frames
 * (SOF 0xA5 | CMD | 4-byte big-endian payload); only the TCP socket that stood
 * in for the ST-Link virtual COM port is replaced by an in-page VirtualPort.
 */
(function () {
  "use strict";

  const SOF = 0xA5;
  const FRAME_LEN = 6;

  const CMD = {
    LED_ON: 0x01,
    LED_OFF: 0x02,
    READ_SENSOR: 0x10,
    TRIGGER_ABS: 0x11,      // PC button -> triggers the ABS over CAN
    PRESSURE: 0x20,
    SPEED: 0x21,
    TEMPERATURE: 0x22,
    DEBIT: 0x23,
    BATTERY_VOLTAGE: 0x24,
    EXTERNAL_LIGHT: 0x25,
    WINDOW_LEFT: 0x31,
    WINDOW_RIGHT: 0x32,
    PORTE_DROIT: 0x33,
    ACK: 0x50,              // LED ack (payload = 0) OR ABS event (payload[0] = 1)
  };

  const RX_NAMES = {
    [CMD.PRESSURE]: "Pressure",
    [CMD.SPEED]: "Speed Motor",
    [CMD.TEMPERATURE]: "Temperature",
    [CMD.DEBIT]: "Débit Fluide",
    [CMD.BATTERY_VOLTAGE]: "Battery Voltage",
    [CMD.EXTERNAL_LIGHT]: "External Light",
    [CMD.WINDOW_LEFT]: "Window Left",
    [CMD.WINDOW_RIGHT]: "Window Right",
    [CMD.PORTE_DROIT]: "Porte Droit",
    [CMD.ACK]: "ACK / ABS",
  };

  const THRESH_OVERHEAT = 110.0;
  const THRESH_HIGH_PRESS = 200.0;
  const THRESH_LOW_FLOW = 20.0;

  // Palette, shared with style.css
  const C = {
    BG: "#05080D", PANEL: "#0A0F18", CARD: "#0F1620", CARD2: "#161F2E",
    BORDER2: "#243348", AMBER: "#F0A500", AMBER_D: "#5A3800", AMBER_L: "#FFD060",
    GREEN: "#00E676", RED: "#FF3D3D", TEAL: "#00E5FF", TEXT: "#D0DCE8",
    TEXT2: "#8FAABE", MUTED: "#2E3F52", DIAL_BG: "#060A10", NEEDLE: "#FF1744",
    TICK: "#3A4A5E", TICK_LIT: "#C7D3DE",
    PRESSURE: "#F0A500", SPEED: "#00E5FF", TEMP: "#CE93D8", DEBIT: "#2979FF",
    BATTERY: "#FFCA28",
    ABS: "#FF3D3D", WIN_L: "#00E5FF", WIN_R: "#CE93D8", DOOR: "#2979FF",
    EXT_LGT: "#EC407A",
    OVHT: "#FF7043", PRES_W: "#F0A500", FLOW_W: "#2979FF",
  };

  const SENSOR_PERIOD = 0.5;    // s, same cadence as the firmware main loop
  const SCENARIO_LEN = 60.0;    // s, one full auto sweep through every warning threshold
  const ABS_EVENT = 0x01000000; // CMD_ACK with payload[0] = 1

  function node(ecu, owner, sensor, unit, vMax, cmd, canId, dlc, def,
                evName, evCmd, evCanId, color, evColor) {
    return { ecu, owner, sensor, unit, vMax, cmd, canId, dlc, def,
             evName, evCmd, evCanId, color, evColor };
  }

  const NODES = [
    node("ECU-01", "Amine", "Pressure", "bar", 255, CMD.PRESSURE, 0x55C, 4, 120,
         "ABS", CMD.ACK, 0x679, C.PRESSURE, C.ABS),
    node("ECU-02", "Yassine", "Speed Motor", "rpm", 5000, CMD.SPEED, 0x406, 2, 2500,
         "Window Left", CMD.WINDOW_LEFT, 0x401, C.SPEED, C.WIN_L),
    node("ECU-03", "Souha", "Temperature", "°C", 150, CMD.TEMPERATURE, 0x456, 4, 85,
         "Window Right", CMD.WINDOW_RIGHT, 0x402, C.TEMP, C.WIN_R),
    node("ECU-04", "Eya", "Débit Fluide", "L/s", 255, CMD.DEBIT, 0x390, 1, 90,
         "Door Right", CMD.PORTE_DROIT, 0x790, C.DEBIT, C.DOOR),
    node("ECU-05", "Nour", "Battery Voltage", "mV", 15000, CMD.BATTERY_VOLTAGE, 0x222, 2, 12600,
         "External Light", CMD.EXTERNAL_LIGHT, 0x221, C.BATTERY, C.EXT_LGT),
  ];

  // cmd -> [centre, amplitude, phase] of the auto-sweep waveform
  const SWEEP = {
    [CMD.PRESSURE]: [120, 100, 0.0],            // 20..220 bar   -> PRES warning
    [CMD.SPEED]: [2500, 2000, 1.0],             // 500..4500 rpm
    [CMD.TEMPERATURE]: [85, 35, 2.0],           // 50..120 °C    -> OVHT warning
    [CMD.DEBIT]: [90, 80, 3.5],                 // 10..170 L/s   -> FLOW warning
    [CMD.BATTERY_VOLTAGE]: [12600, 1200, 5.0],  // 11.4..13.8 V
  };

  // ── UART frames ───────────────────────────────────────────
  function encodeFrame(cmd, value) {
    const frame = new Uint8Array(FRAME_LEN);
    frame[0] = SOF;
    frame[1] = cmd & 0xFF;
    new DataView(frame.buffer).setUint32(2, value >>> 0);
    return frame;
  }

  /* Byte-stream parser: resynchronises on SOF, as protocol.py and the
   * firmware's UART state machine do. */
  class FrameParser {
    constructor(onFrame) {
      this._buf = [];
      this._onFrame = onFrame;
    }

    feed(bytes) {
      for (const b of bytes) this._buf.push(b);
      while (this._buf.length >= FRAME_LEN) {
        const idx = this._buf.indexOf(SOF);
        if (idx === -1) { this._buf.length = 0; return; }
        if (idx > 0) this._buf.splice(0, idx);
        if (this._buf.length < FRAME_LEN) return;
        const frame = Uint8Array.from(this._buf.splice(0, FRAME_LEN));
        const value = new DataView(frame.buffer).getUint32(2);
        this._onFrame(frame[1], value, frame);
      }
    }
  }

  /* The PC end of ECU-01's UART, standing in for the ST-Link virtual COM port. */
  class VirtualPort {
    constructor(bus) {
      this._bus = bus;
      this.isOpen = true;
      this.onbytes = null;      // ECU -> PC
    }

    write(bytes) {
      if (this.isOpen) this._bus._uartRx(bytes);
    }

    close() {
      if (!this.isOpen) return;
      this.isOpen = false;
      this._bus._closed(this);
    }
  }

  /* The shared CAN bus with five nodes on it, seen from ECU-01.
   *
   * Every frame put on the bus is reported to the listeners and, while a
   * dashboard is connected, relayed over UART exactly as ECU-01's pass-through
   * CAN filter does. Listeners get (kind, a, b, c):
   *
   *   "can"      canId, cmd, value     a frame went onto the bus
   *   "uart_tx"  cmd, value, 0         ECU-01 -> dashboard
   *   "uart_rx"  cmd, value, 0         dashboard -> ECU-01
   *   "led"      state, 0, 0           ECU-01 LD1 switched by the dashboard
   *   "link"     state, 0, 0           dashboard connected / disconnected
   *   "value"    cmd, value, 0         a sensor input was set by hand
   *   "auto"     cmd, state, 0         a sensor entered / left auto sweep
   */
  class VirtualBus {
    constructor(auto = true, randomEvents = false) {
      this.values = {};
      this.auto = {};
      for (const n of NODES) {
        this.values[n.cmd] = n.def;
        this.auto[n.cmd] = auto;
      }
      this.randomEvents = randomEvents;
      this.led = false;
      this._listeners = [];
      this._port = null;
      this._parser = new FrameParser((cmd, value) => this._onCommand(cmd, value));
      this._t0 = performance.now() / 1000;
      this._turn = 0;
      this._nextEvent = this._t0 + 3 + Math.random() * 3;
    }

    listen(callback) {
      this._listeners.push(callback);
    }

    notify(kind, a, b, c) {
      for (const cb of this._listeners) cb(kind, a, b, c);
    }

    // ── inputs ──────────────────────────────────────────────
    setValue(cmd, value) {
      this.values[cmd] = Math.trunc(value);
      this.notify("value", cmd, this.values[cmd], 0);
    }

    setAuto(cmd, state) {
      if (this.auto[cmd] === state) return;
      this.auto[cmd] = state;
      this.notify("auto", cmd, state ? 1 : 0, 0);
    }

    setAutoAll(state) {
      for (const n of NODES) this.setAuto(n.cmd, state);
    }

    get allAuto() {
      return NODES.every((n) => this.auto[n.cmd]);
    }

    /* A node's push button: one asynchronous event frame on the bus. */
    trigger(evCmd) {
      const n = NODES.find((x) => x.evCmd === evCmd);
      this._canTx(n.evCanId, evCmd, evCmd === CMD.ACK ? ABS_EVENT : 1);
    }

    // ── bus ─────────────────────────────────────────────────
    start() {
      setInterval(() => this._busTick(), SENSOR_PERIOD * 1000 / NODES.length);
    }

    _sweep(cmd) {
      const [centre, amp, phase] = SWEEP[cmd];
      const t = (performance.now() / 1000 - this._t0) / SCENARIO_LEN * 2 * Math.PI;
      const noise = (Math.random() * 0.04 - 0.02) * amp;
      return Math.max(0, Math.trunc(centre + amp * Math.sin(t + phase) + noise));
    }

    /* One node transmits per tick, so all five are heard every SENSOR_PERIOD. */
    _busTick() {
      const n = NODES[this._turn];
      this._turn = (this._turn + 1) % NODES.length;
      if (this.auto[n.cmd]) this.values[n.cmd] = this._sweep(n.cmd);
      this._canTx(n.canId, n.cmd, this.values[n.cmd]);

      const now = performance.now() / 1000;
      if (this._turn === 0 && this.randomEvents && now >= this._nextEvent) {
        this.trigger(NODES[Math.floor(Math.random() * NODES.length)].evCmd);
        this._nextEvent = now + 3 + Math.random() * 3;
      }
    }

    _canTx(canId, cmd, value) {
      this.notify("can", canId, cmd, value);
      this._uartTx(cmd, value);
    }

    // ── ECU-01 UART side ────────────────────────────────────
    open() {
      if (this._port) this._port.close();
      this._port = new VirtualPort(this);
      this.notify("link", 1, 0, 0);
      return this._port;
    }

    _closed(port) {
      if (this._port !== port) return;
      this._port = null;
      this.notify("link", 0, 0, 0);
    }

    _uartTx(cmd, value) {
      const port = this._port;
      if (!port) return;
      if (port.onbytes) port.onbytes(encodeFrame(cmd, value));
      this.notify("uart_tx", cmd, value, 0);
    }

    _uartRx(bytes) {
      this._parser.feed(bytes);
    }

    /* Mirror of NewFrameReceivedCallback() in firmware/Core/Src/main.c. */
    _onCommand(cmd, value) {
      this.notify("uart_rx", cmd, value, 0);
      if (cmd === CMD.LED_ON || cmd === CMD.LED_OFF) {
        this.led = cmd === CMD.LED_ON;
        this.notify("led", this.led ? 1 : 0, 0, 0);
        this._uartTx(CMD.ACK, 0);
      } else if (cmd === CMD.READ_SENSOR) {
        this._uartTx(CMD.PRESSURE, this.values[CMD.PRESSURE]);
      } else if (cmd === CMD.TRIGGER_ABS) {
        // Goes out on the CAN bus and comes back through the pass-through filter
        this.trigger(CMD.ACK);
      }
    }
  }

  // ── helpers shared by the three views ─────────────────────
  function hex(value, width) {
    return (value >>> 0).toString(16).toUpperCase().padStart(width, "0");
  }

  function hexBytes(bytes) {
    return Array.from(bytes, (b) => hex(b, 2)).join(" ");
  }

  /* Big-endian CAN data field of ``dlc`` bytes. */
  function canData(value, dlc) {
    const out = [];
    for (let i = dlc - 1; i >= 0; i--) out.push(hex(Math.floor(value / 2 ** (8 * i)) % 256, 2));
    return out.join(" ");
  }

  function timestamp() {
    const d = new Date();
    const p = (v, n) => String(v).padStart(n, "0");
    return `${p(d.getHours(), 2)}:${p(d.getMinutes(), 2)}:${p(d.getSeconds(), 2)}.${p(d.getMilliseconds(), 3)}`;
  }

  /* Text of the frame a node just put on the bus, as shown by the bench and the car. */
  function describeCan(n, isEvent, canId, value) {
    const data = isEvent ? "01" : canData(value, n.dlc);
    const what = isEvent ? `${n.evName} button` : `${n.sensor} = ${value} ${n.unit}`;
    return `0x${hex(canId, 3)}   [${data}]   ${n.ecu} · ${what}`;
  }

  /* canId -> [node index, node, isEvent] */
  function canIndex() {
    const map = {};
    NODES.forEach((n, i) => {
      map[n.canId] = [i, n, false];
      map[n.evCanId] = [i, n, true];
    });
    return map;
  }

  function appendLog(box, tag, color, html, maxLines = 300) {
    const line = document.createElement("div");
    line.className = "ln";
    line.innerHTML =
      `<span class="ts">${timestamp()}</span>` +
      `<span class="tag" style="color:${color}">${tag}</span>${html}`;
    const pinned = box.scrollHeight - box.scrollTop - box.clientHeight < 40;
    box.appendChild(line);
    while (box.childElementCount > maxLines) box.removeChild(box.firstChild);
    if (pinned) box.scrollTop = box.scrollHeight;
  }

  /* One sensor input: slider, value label and "auto sweep" checkbox, kept in
   * step with the bus so the same sensor can be driven from any view. */
  function bindSensor(bus, n, slider, label, chkAuto, onUser) {
    const show = (v) => { label.textContent = `${v} ${n.unit}`; };
    slider.min = 0;
    slider.max = n.vMax;
    slider.value = bus.values[n.cmd];
    show(bus.values[n.cmd]);
    chkAuto.checked = bus.auto[n.cmd];

    // Moved by hand: take the sensor out of auto sweep and apply the value
    slider.addEventListener("input", () => {
      bus.setAuto(n.cmd, false);
      bus.setValue(n.cmd, Number(slider.value));
      if (onUser) onUser();
    });
    chkAuto.addEventListener("change", () => bus.setAuto(n.cmd, chkAuto.checked));

    bus.listen((kind, a, b, c) => {
      if (kind === "can" && a === n.canId) { slider.value = c; show(c); }
      else if (kind === "value" && a === n.cmd) { slider.value = b; show(b); }
      else if (kind === "auto" && a === n.cmd) chkAuto.checked = !!b;
    });
  }

  /* "Auto sweep all sensors" checkbox. */
  function bindAutoAll(bus, chk) {
    chk.checked = bus.allAuto;
    chk.addEventListener("change", () => bus.setAutoAll(chk.checked));
    bus.listen((kind) => { if (kind === "auto") chk.checked = bus.allAuto; });
  }

  function el(tag, className, html) {
    const e = document.createElement(tag);
    if (className) e.className = className;
    if (html !== undefined) e.innerHTML = html;
    return e;
  }

  /* Size a canvas to its CSS box at device resolution; returns [ctx, w, h] in CSS px. */
  function fitCanvas(canvas) {
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return [ctx, w, h];
  }

  function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  function font(size, bold) {
    return `${bold ? "bold " : ""}${size}px Consolas, "Courier New", monospace`;
  }

  window.CanSim = {
    SOF, FRAME_LEN, CMD, RX_NAMES, C, NODES, SWEEP, SENSOR_PERIOD, ABS_EVENT,
    THRESH_OVERHEAT, THRESH_HIGH_PRESS, THRESH_LOW_FLOW,
    encodeFrame, FrameParser, VirtualBus,
    hex, hexBytes, canData, timestamp, describeCan, canIndex, appendLog,
    bindSensor, bindAutoAll, el, fitCanvas, roundRect, font,
  };
})();
