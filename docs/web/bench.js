/* Virtual test bench: the five ECUs and their shared CAN bus, driven by hand.
 * Port of dashboard/virtual_bench.py.
 *
 * Each board has a slider for its sensor input and a push button for its manual
 * trigger. Whatever is set here goes onto the simulated bus and reaches the
 * dashboard through ECU-01's UART link, as it would on the real hardware.
 */
(function () {
  "use strict";

  const S = window.CanSim;
  const { C, NODES } = S;

  const CARD_SPACING = 10;
  const FLASH_MS = 220;

  /* CANH/CANL pair with a stub per node, terminators, and ECU-01's UART link. */
  class BusView {
    constructor(canvas) {
      this.canvas = canvas;
      this._active = null;      // index of the node currently transmitting
      this._text = "";
      this._uart = null;        // color of the current UART flash
      this._linked = false;
      this._canTimer = null;
      this._uartTimer = null;
      new ResizeObserver(() => this.paint()).observe(canvas);
    }

    setLinked(state) {
      this._linked = state;
      this.paint();
    }

    flashCan(index, text) {
      this._active = index;
      this._text = text;
      clearTimeout(this._canTimer);
      this._canTimer = setTimeout(() => { this._active = null; this.paint(); }, FLASH_MS);
      this.paint();
    }

    flashUart(color) {
      this._uart = color;
      clearTimeout(this._uartTimer);
      this._uartTimer = setTimeout(() => { this._uart = null; this.paint(); }, FLASH_MS);
      this.paint();
    }

    _centre(i, w) {
      const cardW = (w - CARD_SPACING * (NODES.length - 1)) / NODES.length;
      return i * (cardW + CARD_SPACING) + cardW / 2;
    }

    paint() {
      if (!this.canvas.clientWidth) return;
      const [p, w, h] = S.fitCanvas(this.canvas);
      p.clearRect(0, 0, w, h);
      const yH = 46, yL = 60;
      const x0 = 34, x1 = w - 34;
      const idle = C.BORDER2;
      const live = this._active !== null ? NODES[this._active].color : null;
      const line = (xa, ya, xb, yb) => {
        p.beginPath();
        p.moveTo(xa, ya);
        p.lineTo(xb, yb);
        p.stroke();
      };
      p.textAlign = "center";
      p.textBaseline = "middle";

      // stubs: transceiver box on each board, then down to the pair
      NODES.forEach((n, i) => {
        const cx = this._centre(i, w);
        const on = i === this._active;
        p.strokeStyle = on ? n.color : idle;
        p.lineWidth = on ? 3 : 1.5;
        line(cx - 4, 22, cx - 4, yH);
        line(cx + 4, 22, cx + 4, yL);
        p.strokeStyle = n.color;
        p.lineWidth = 1;
        p.fillStyle = C.CARD;
        S.roundRect(p, cx - 46, 2, 92, 20, 4);
        p.fill();
        p.stroke();
        p.font = S.font(9, true);
        p.fillStyle = n.color;
        p.fillText("CAN TRANSCEIVER", cx, 12.5);
      });

      // the differential pair
      p.strokeStyle = live || idle;
      p.lineWidth = live ? 4 : 2.5;
      line(x0, yH, x1, yH);
      line(x0, yL, x1, yL);
      p.font = S.font(9, false);
      p.fillStyle = C.TEXT2;
      p.textAlign = "right";
      p.fillText("CAN_H", x1 - 16, yH - 8);
      p.fillText("CAN_L", x1 - 16, yL + 9);
      p.textAlign = "center";

      // 120 Ω termination at both ends
      for (const x of [x0, x1]) {
        p.strokeStyle = C.TEXT2;
        p.lineWidth = 1;
        p.fillStyle = C.CARD2;
        p.fillRect(x - 6, yH - 4, 12, yL - yH + 8);
        p.strokeRect(x - 6, yH - 4, 12, yL - yH + 8);
        p.fillStyle = C.TEXT2;
        p.fillText("120 Ω", x, yL + 22);
      }

      // frame currently on the bus
      if (live) {
        p.fillStyle = live;
        p.font = S.font(13, true);
        p.fillText(this._text, w / 2, yL + 31);
      }

      // ECU-01's UART link down to the PC
      const ux = Math.max(14, this._centre(0, w) - 60);
      const linkCol = this._uart || (this._linked ? C.TEAL : C.MUTED);
      p.strokeStyle = linkCol;
      p.lineWidth = this._uart ? 3 : 1.5;
      p.setLineDash([6, 4]);
      line(ux, 0, ux, 126);
      p.setLineDash([]);
      p.lineWidth = 1.5;
      p.fillStyle = C.CARD;
      S.roundRect(p, 4, 126, 270, 38, 6);
      p.fill();
      p.stroke();
      p.font = S.font(10, true);
      p.fillStyle = linkCol;
      p.fillText("PC DASHBOARD  ·  UART 115200", 139, 138);
      p.fillText(`ST-Link VCP  ·  ${this._linked ? "CONNECTED" : "NOT CONNECTED"}`, 139, 152);
    }
  }

  /* One NUCLEO board: status LEDs, a sensor input and a push button. */
  class NodeCard {
    constructor(n, bus) {
      this.node = n;
      const user = n.ecu === "ECU-01"
        ? `<span class="led user"></span><span class="small">LD1</span>` : "";
      this.el = S.el("div", "node-card",
        `<div class="head">${n.ecu}  ·  ${n.owner.toUpperCase()}</div>` +
        `<div class="small board">NUCLEO-F429ZI</div>` +
        `<div class="leds"><span class="led beat"></span><span class="small">LD2</span>` +
        `<span class="led tx"></span><span class="small">CAN TX</span>${user}</div>` +
        `<div class="section">SENSOR  ·  ${n.sensor.toUpperCase()}</div>` +
        `<div class="value"></div>` +
        `<input type="range" aria-label="${n.sensor}">` +
        `<div class="range"><span class="small">0</span>` +
        `<label><input type="checkbox"> Auto sweep</label>` +
        `<span class="small">${n.vMax}</span></div>` +
        `<div class="small">CAN 0x${S.hex(n.canId, 3)}  →  UART 0x${S.hex(n.cmd, 2)}</div>` +
        `<div class="section">PUSH BUTTON</div>` +
        `<button type="button">●  ${n.evName.toUpperCase()}</button>` +
        `<div class="small">CAN 0x${S.hex(n.evCanId, 3)}  →  UART 0x${S.hex(n.evCmd, 2)}</div>`);
      this.el.style.setProperty("--c", n.color);
      this.ledBeat = this.el.querySelector(".led.beat");
      this.ledTx = this.el.querySelector(".led.tx");
      this.ledUser = this.el.querySelector(".led.user");
      this._txTimer = null;
      S.bindSensor(bus, n, this.el.querySelector("input[type=range]"),
                   this.el.querySelector(".value"), this.el.querySelector("input[type=checkbox]"));
      this.el.querySelector("button").addEventListener("click", () => bus.trigger(n.evCmd));
    }

    flashTx(ms = 120) {
      this.ledTx.classList.add("on");
      clearTimeout(this._txTimer);
      this._txTimer = setTimeout(() => this.ledTx.classList.remove("on"), ms);
    }
  }

  class VirtualBench {
    constructor(root, bus) {
      this._linked = false;
      this._byCan = S.canIndex();

      const cards = root.querySelector("#b-cards");
      this._cards = NODES.map((n) => {
        const card = new NodeCard(n, bus);
        cards.appendChild(card.el);
        return card;
      });
      this._busView = new BusView(root.querySelector("#b-bus"));
      this._log = root.querySelector("#b-log");
      this._chkSensors = root.querySelector("#b-show-sensors");
      S.bindAutoAll(bus, root.querySelector("#b-auto-all"));

      let beat = false;
      setInterval(() => {
        beat = !beat;
        for (const c of this._cards) c.ledBeat.classList.toggle("on", beat);
      }, 500);

      bus.listen((kind, a, b, c) => this._onBus(kind, a, b, c));
    }

    _onBus(kind, a, b, c) {
      if (kind === "can") {
        const [index, n, isEvent] = this._byCan[a];
        if (!this._linked) return;          // dashboard disconnected: no transfer shown
        this._cards[index].flashTx();
        const text = S.describeCan(n, isEvent, a, c);
        this._busView.flashCan(index, text);
        if (isEvent || this._chkSensors.checked) this._append("CAN", n.color, text);
      } else if (kind === "uart_tx") {
        this._busView.flashUart(C.TEAL);
      } else if (kind === "uart_rx") {
        this._busView.flashUart(C.AMBER);
        this._append("UART", C.AMBER_L,
          `Dashboard → ECU-01   CMD 0x${S.hex(a, 2)}   DATA 0x${S.hex(b, 8)}`);
      } else if (kind === "led") {
        this._cards[0].ledUser.classList.toggle("on", !!a);
        this._append("GPIO", C.TEAL, `ECU-01 LD1 ${a ? "ON" : "OFF"}`);
      } else if (kind === "link") {
        this._linked = !!a;
        this._busView.setLinked(!!a);
        this._append("UART", C.AMBER_L, a ? "Dashboard connected" : "Dashboard disconnected");
      }
    }

    _append(tag, color, msg) {
      S.appendLog(this._log, tag, color, `<span class="msg">${msg}</span>`);
    }
  }

  S.VirtualBench = VirtualBench;
})();
