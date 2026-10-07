/* The CAN automotive dashboard: gauges, warnings, node status, events and the
 * frame log. Port of dashboard/dashboard_window.py and dashboard/widgets.py.
 */
(function () {
  "use strict";

  const S = window.CanSim;
  const { CMD, C, NODES, RX_NAMES } = S;

  const PT = 1.33;    // Qt point sizes -> CSS pixels

  class ClassicGauge {
    constructor(canvas, n) {
      this.canvas = canvas;
      this.label = n.sensor;
      this.unit = n.unit;
      this.vMin = 0;
      this.vMax = n.vMax;
      this.color = n.color;
      this._value = 0;
      this._anim = 0;
      this._online = false;
      this._lastRx = null;
      this._stale = false;
      this._dirty = true;
      this._w = 0;
      this._h = 0;
    }

    setValue(v) {
      this._value = Math.max(this.vMin, Math.min(this.vMax, v));
      this._online = true;
      this._lastRx = performance.now();
      if (this._stale) this._dirty = true;
      this._stale = false;
    }

    tick(now) {
      if (this._online && this._lastRx !== null) {
        const stale = now - this._lastRx > 5000;
        if (stale !== this._stale) { this._stale = stale; this._dirty = true; }
      }
      const diff = this._value - this._anim;
      if (Math.abs(diff) > 0.15) {
        this._anim += diff * 0.16;
        this._dirty = true;
      } else if (this._anim !== this._value) {
        this._anim = this._value;
        this._dirty = true;
      }
      const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
      if (w !== this._w || h !== this._h) { this._w = w; this._h = h; this._dirty = true; }
      if (this._dirty && w && h) { this._dirty = false; this.paint(); }
    }

    paint() {
      const [p, W, H] = S.fitCanvas(this.canvas);
      p.clearRect(0, 0, W, H);
      p.globalAlpha = this._stale ? 0.4 : 1.0;

      const size = Math.min(W, H - 46);
      const cx = W / 2, cy = (H - 46) / 2 + 8;
      const r = size / 2 - 8;
      if (r < 20) return;
      const live = this._online && !this._stale;

      const dial = p.createRadialGradient(cx, cy - r * 0.15, 0, cx, cy - r * 0.15, r * 1.1);
      dial.addColorStop(0, "#12181F");
      dial.addColorStop(1, C.DIAL_BG);
      p.fillStyle = dial;
      p.strokeStyle = C.BORDER2;
      p.lineWidth = 1.4;
      p.beginPath();
      p.arc(cx, cy, r, 0, 2 * Math.PI);
      p.fill();
      p.stroke();

      // The scale runs clockwise from 225° over 270°; canvas angles are clockwise too
      const start = 225, sweep = 270;
      const rad = (deg) => deg * Math.PI / 180;
      let pct = (this._anim - this.vMin) / Math.max(this.vMax - this.vMin, 1e-9);
      pct = Math.max(0, Math.min(1, pct));
      const arcR = r * 0.955;
      p.lineWidth = Math.max(5.0, r * 0.085);
      p.lineCap = "round";
      p.strokeStyle = C.TICK;
      p.beginPath();
      p.arc(cx, cy, arcR, rad(-start), rad(-start + sweep));
      p.stroke();
      if (pct > 0) {
        p.strokeStyle = this.color;
        p.beginPath();
        p.arc(cx, cy, arcR, rad(-start), rad(-start + sweep * pct));
        p.stroke();
      }

      p.lineCap = "butt";
      p.textAlign = "center";
      p.textBaseline = "middle";
      const nMinor = 25;
      for (let i = 0; i <= nMinor; i++) {
        const frac = i / nMinor;
        const a = rad(start - frac * sweep);
        const major = i % 5 === 0;
        const rOut = r * 0.90, rIn = r * (major ? 0.74 : 0.82);
        p.strokeStyle = major ? C.TICK_LIT : C.TICK;
        p.lineWidth = major ? 2.0 : 1.0;
        p.beginPath();
        p.moveTo(cx + rOut * Math.cos(a), cy - rOut * Math.sin(a));
        p.lineTo(cx + rIn * Math.cos(a), cy - rIn * Math.sin(a));
        p.stroke();
        if (major) {
          const txt = (this.vMin + frac * (this.vMax - this.vMin)).toFixed(0);
          const pt = txt.length <= 3 ? Math.max(7, Math.floor(r * 0.10))
                                     : Math.max(6, Math.floor(r * 0.085));
          p.font = S.font(pt * PT, true);
          p.fillStyle = C.TICK_LIT;
          p.fillText(txt, cx + r * 0.56 * Math.cos(a), cy - r * 0.56 * Math.sin(a));
        }
      }

      const na = rad(start - pct * sweep);
      const nx = cx + r * 0.68 * Math.cos(na), ny = cy - r * 0.68 * Math.sin(na);
      const bx = cx - r * 0.12 * Math.cos(na), by = cy + r * 0.12 * Math.sin(na);
      p.lineCap = "round";
      p.strokeStyle = "rgba(0,0,0,0.43)";
      p.lineWidth = 4;
      p.beginPath();
      p.moveTo(bx + 1.2, by + 1.2);
      p.lineTo(nx + 1.2, ny + 1.2);
      p.stroke();
      p.strokeStyle = C.NEEDLE;
      p.lineWidth = 2.4;
      p.beginPath();
      p.moveTo(bx, by);
      p.lineTo(nx, ny);
      p.stroke();

      const hub = p.createRadialGradient(cx, cy, 0, cx, cy, r * 0.09);
      hub.addColorStop(0, "#9AAFC4");
      hub.addColorStop(1, "#28313F");
      p.fillStyle = hub;
      p.strokeStyle = C.BORDER2;
      p.lineWidth = 1;
      p.beginPath();
      p.arc(cx, cy, r * 0.07, 0, 2 * Math.PI);
      p.fill();
      p.stroke();

      p.font = S.font(Math.max(16, Math.floor(r * 0.20)) * PT, true);
      p.fillStyle = live ? this.color : C.MUTED;
      p.fillText(this._anim.toFixed(0), cx, cy + r * 0.30);

      p.font = S.font(Math.max(8, Math.floor(r * 0.10)) * PT, false);
      p.fillStyle = C.TEXT2;
      p.fillText(this.unit, cx, cy + r * 0.59);

      p.font = S.font(Math.max(10, Math.floor(r * 0.13)) * PT, true);
      p.fillStyle = live ? this.color : C.MUTED;
      p.fillText(this.label.toUpperCase(), cx, H - 17);

      p.fillStyle = live ? C.GREEN : C.MUTED;
      p.beginPath();
      p.arc(W - 14, 12, 4.5, 0, 2 * Math.PI);
      p.fill();
      p.globalAlpha = 1.0;
    }
  }

  /* A badge or warning lamp that can be lit for a while. */
  class Lamp {
    constructor(element) {
      this.el = element;
      this._timer = null;
    }

    setActive(state) {
      this.el.classList.toggle("active", state);
      const status = this.el.querySelector(".st");
      if (status) status.textContent = state ? "ACTIVE" : "IDLE";
    }

    pulse(ms) {
      clearTimeout(this._timer);
      this.setActive(true);
      this._timer = setTimeout(() => this.setActive(false), ms);
    }
  }

  class Dashboard {
    constructor(root, bus) {
      this._root = root;
      this._bus = bus;
      this._port = null;
      this._parser = new S.FrameParser((cmd, value, frame) => {
        this._onRaw(frame);
        this._onFrame(cmd, value);
      });
      this._frameCount = 0;
      this._frameCountPrev = 0;
      this._statusTimer = null;
      this._idleStatus = "Disconnected  •  OpCode Labs CAN Automotive Network";

      this._build();
      this._applyConnected(false);
      setInterval(() => this._updateClock(), 1000);
      this._updateClock();

      const loop = (now) => {
        for (const g of Object.values(this._gauges)) g.tick(now);
        requestAnimationFrame(loop);
      };
      requestAnimationFrame(loop);
    }

    _q(sel) {
      return this._root.querySelector(sel);
    }

    _build() {
      // header: one chip per node
      const hdr = this._q("#d-hdr-nodes");
      this._hdr = {};
      for (const n of NODES) {
        const chip = S.el("div", "hdr-node",
          `<div class="nm">● ${n.owner.toUpperCase()}</div><div class="val">OFFLINE</div>`);
        chip.style.setProperty("--c", n.color);
        hdr.appendChild(chip);
        this._hdr[n.cmd] = chip;
      }

      // warning indicators
      const warns = this._q("#d-warns");
      const mkWarn = (label, sub, color) => {
        const w = S.el("div", "warn", `<div class="main">${label}</div><div class="sub">${sub}</div>`);
        w.style.setProperty("--c", color);
        warns.appendChild(w);
        return new Lamp(w);
      };
      this._warnAbs = mkWarn("ABS", "Anti-lock", C.ABS);
      this._warnOvht = mkWarn("OVHT", "Overheat", C.OVHT);
      this._warnPres = mkWarn("PRES", "High Press", C.PRES_W);
      this._warnFlow = mkWarn("FLOW", "Low Flow", C.FLOW_W);

      // CAN node status
      const status = this._q("#d-node-status");
      this._badges = {};
      for (const n of NODES) {
        const b = S.el("div", "node-badge",
          `<span class="dot">●</span>` +
          `<div class="info"><div><span class="nm">${n.owner.toUpperCase()}</span>` +
          `<span class="role">${n.sensor}</span></div>` +
          `<div class="ids">CAN 0x${S.hex(n.canId, 3)}&nbsp;&nbsp;&nbsp;CMD 0x${S.hex(n.cmd, 2)}</div></div>` +
          `<span class="val">—</span>`);
        b.style.setProperty("--c", n.color);
        status.appendChild(b);
        this._badges[n.cmd] = b;
      }

      // gauges: speed / pressure / temperature on top, battery / flow below
      this._gauges = {};
      const place = (row, order) => {
        for (const i of order) {
          const canvas = S.el("canvas", "gauge");
          this._q(row).appendChild(canvas);
          this._gauges[NODES[i].cmd] = new ClassicGauge(canvas, NODES[i]);
        }
      };
      place("#d-gauges-top", [1, 0, 2]);
      place("#d-gauges-bot", [4, 3]);

      // network events
      const events = this._q("#d-events");
      this._notif = {};
      for (const n of NODES) {
        const b = S.el("div", "notif",
          `<span class="dot">◉</span>` +
          `<div class="info"><div class="main">${n.evName.toUpperCase()}</div>` +
          `<div class="sub">${n.owner} · 0x${S.hex(n.evCanId, 3)}</div></div>` +
          `<span class="st">IDLE</span>`);
        b.style.setProperty("--c", n.evColor);
        events.appendChild(b);
        this._notif[n.evCmd] = new Lamp(b);
      }

      // simulator inputs: what the real sensors and push buttons would provide
      const inputs = this._q("#d-sim-inputs");
      for (const n of NODES) {
        const card = S.el("div", "sim-input",
          `<div class="head"><span class="nm">${n.sensor.toUpperCase()}</span><span class="val"></span></div>` +
          `<input type="range" aria-label="${n.sensor}">` +
          `<div class="row"><button type="button">●  ${n.evName.toUpperCase()}</button>` +
          `<label><input type="checkbox"> Auto</label></div>`);
        card.style.setProperty("--c", n.color);
        inputs.appendChild(card);
        S.bindSensor(this._bus, n, card.querySelector("input[type=range]"),
                     card.querySelector(".val"), card.querySelector("input[type=checkbox]"));
        card.querySelector("button").addEventListener("click", () => this._bus.trigger(n.evCmd));
      }
      S.bindAutoAll(this._bus, this._q("#d-auto-all"));
      const chkRandom = this._q("#d-random");
      chkRandom.checked = this._bus.randomEvents;
      chkRandom.addEventListener("change", () => { this._bus.randomEvents = chkRandom.checked; });

      // connection and commands
      this._btnConn = this._q("#d-connect");
      this._btnDisc = this._q("#d-disconnect");
      this._btnConn.addEventListener("click", () => this.connect());
      this._btnDisc.addEventListener("click", () => this.disconnect());
      this._cmdWidgets = Array.from(this._root.querySelectorAll("[data-cmd], #d-send"));
      for (const btn of this._root.querySelectorAll("[data-cmd]")) {
        btn.addEventListener("click", () => this._send(Number(btn.dataset.cmd), 0));
      }
      this._q("#d-send").addEventListener("click", () => this._sendManual());

      this._log = this._q("#d-log");
      this._chkTx = this._q("#d-log-tx");
      this._chkRx = this._q("#d-log-rx");
      this._q("#d-log-clear").addEventListener("click", () => { this._log.textContent = ""; });
    }

    // ── link ────────────────────────────────────────────────
    connect() {
      if (this._port) return;
      const baud = this._q("#d-baud").value;
      this._port = this._bus.open();
      this._port.onbytes = (bytes) => this._parser.feed(bytes);
      this._applyConnected(true);
      this._idleStatus = `Connected  •  Virtual ECU (simulator)  •  ${baud} baud`;
      this._showStatus(this._idleStatus);
      this._sysLog(`Port opened: Virtual ECU (simulator) @ ${baud}`);
    }

    disconnect() {
      if (!this._port) return;
      this._port.close();
      this._port = null;
      this._applyConnected(false);
      this._idleStatus = "Disconnected  •  OpCode Labs CAN Network";
      this._showStatus(this._idleStatus);
      this._sysLog("Port closed.");
    }

    _applyConnected(state) {
      this._btnConn.disabled = state;
      this._btnDisc.disabled = !state;
      this._q("#d-port").disabled = state;
      this._q("#d-baud").disabled = state;
      for (const w of this._cmdWidgets) w.disabled = !state;
      const lbl = this._q("#d-status");
      lbl.textContent = state ? "● ONLINE" : "● OFFLINE";
      lbl.classList.toggle("on", state);
    }

    _send(cmd, data) {
      if (!this._port) return;
      const frame = S.encodeFrame(cmd, data);
      if (this._chkTx.checked) {
        S.appendLog(this._log, "TX", C.GREEN,
          `<span class="hex">${S.hexBytes(frame)}</span><b style="color:${C.AMBER_L}">CMD 0x${S.hex(cmd, 2)}</b>`);
      }
      this._port.write(frame);
    }

    _sendManual() {
      const cmd = Number.parseInt(this._q("#d-man-cmd").value, 16);
      const data = Number.parseInt(this._q("#d-man-data").value, 16);
      if (Number.isNaN(cmd) || Number.isNaN(data) || cmd < 0 || cmd > 0xFF || data < 0 || data > 0xFFFFFFFF) {
        this._sysLog("Invalid format — use hex, e.g. 0x20");
        return;
      }
      this._send(cmd, data);
    }

    // ── reception ───────────────────────────────────────────
    _onFrame(cmd, value) {
      this._frameCount += 1;
      this._q("#d-frames-total").textContent = String(this._frameCount);

      const gauge = this._gauges[cmd];
      if (gauge) {
        const n = NODES.find((x) => x.cmd === cmd);
        gauge.setValue(value);
        const badge = this._badges[cmd];
        badge.querySelector(".val").textContent = `${value} ${n.unit}`;
        badge.classList.add("online");
        const chip = this._hdr[cmd];
        chip.querySelector(".val").textContent = `${value} ${n.unit}`;
        chip.classList.add("online");

        if (cmd === CMD.TEMPERATURE) this._warnOvht.setActive(value >= S.THRESH_OVERHEAT);
        else if (cmd === CMD.PRESSURE) this._warnPres.setActive(value >= S.THRESH_HIGH_PRESS);
        else if (cmd === CMD.DEBIT) this._warnFlow.setActive(value <= S.THRESH_LOW_FLOW);
      } else if (cmd === CMD.WINDOW_LEFT) {
        this._notif[cmd].pulse(2000);
        this._showStatus("WINDOW LEFT — Yassine", 2000);
      } else if (cmd === CMD.WINDOW_RIGHT) {
        this._notif[cmd].pulse(2000);
        this._showStatus("WINDOW RIGHT — Souha", 2000);
      } else if (cmd === CMD.PORTE_DROIT) {
        this._notif[cmd].pulse(3000);
        this._showStatus("DOOR RIGHT — Eya", 3000);
      } else if (cmd === CMD.EXTERNAL_LIGHT) {
        this._notif[cmd].pulse(2000);
        this._showStatus("EXTERNAL LIGHT — Nour", 2000);
      } else if (cmd === CMD.ACK) {
        if ((value >>> 24 & 0xFF) === 0x01) {
          this._notif[cmd].pulse(2500);
          this._warnAbs.pulse(2500);
          this._showStatus("ABS — Amine", 2500);
        } else {
          this._showStatus("ACK received from STM32 (LED)", 1500);
        }
      }
    }

    _onRaw(frame) {
      if (!this._chkRx.checked) return;
      const name = RX_NAMES[frame[1]] || `0x${S.hex(frame[1], 2)}`;
      S.appendLog(this._log, "RX", C.TEAL,
        `<span class="hex">${S.hexBytes(frame)}</span><b>${name}</b>`);
    }

    _sysLog(msg) {
      S.appendLog(this._log, "SYS", C.AMBER_L, `<span>${msg}</span>`);
    }

    _showStatus(msg, ms) {
      const bar = this._q("#d-statusbar");
      clearTimeout(this._statusTimer);
      bar.textContent = msg;
      if (ms) this._statusTimer = setTimeout(() => { bar.textContent = this._idleStatus; }, ms);
    }

    _updateClock() {
      this._q("#d-clock").textContent = S.timestamp().slice(0, 8);
      const delta = this._frameCount - this._frameCountPrev;
      this._frameCountPrev = this._frameCount;
      this._q("#d-frames-hz").textContent = `${delta.toFixed(1)} Hz`;
    }
  }

  S.Dashboard = Dashboard;
})();
