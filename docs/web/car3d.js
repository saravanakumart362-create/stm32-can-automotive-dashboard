/* Virtual car in 3D: the five ECUs where they sit in a vehicle, and their wiring.
 * Port of dashboard/virtual_car3d.py.
 *
 * The CAN twisted pair runs the length of the car between the two terminated end
 * nodes, every other ECU hangs off it on a stub, and ECU-01's UART leaves for the
 * instrument cluster, which stands in for the PC dashboard. Scroll over an ECU (or
 * use its slider) to change the sensor value at that location: the frame travels
 * down the harness to ECU-01 and on to the dashboard, as on the real hardware.
 *
 * Drawn on a 2D canvas with a depth sort, so it needs no WebGL.
 */
(function () {
  "use strict";

  const S = window.CanSim;
  const { C, NODES } = S;

  // World axes: x towards the front of the car, y to its left, z up. Units are metres.
  const FOV = 34.0;
  const NEAR = 0.3;
  const PULSE_SPEED = 5.0;    // m/s, slowed down enormously so a frame can be followed by eye
  const TX_FLASH = 0.18;      // s

  const COL_CAN_H = "#FFD54F";
  const COL_CAN_L = "#66BB6A";

  // ── where things are ────────────────────────────────────────
  const ECU_SIZE = [0.30, 0.22, 0.12];
  const ECU_POS = [
    [1.10, 0.50, 0.68],     // ECU-01  engine bay left, next to the brake unit
    [1.30, -0.50, 0.70],    // ECU-02  engine bay right, next to the motor
    [1.90, -0.55, 0.50],    // ECU-03  behind the radiator   (bus end, 120 Ω)
    [-0.55, -0.62, 0.36],   // ECU-04  floor, by the right door
    [-1.75, 0.45, 0.50],    // ECU-05  boot                   (bus end, 120 Ω)
  ];

  // The sensed component of each node: [label, centre, size]
  const PARTS = [
    ["Brake pressure unit", [1.65, 0.58, 0.55], [0.20, 0.20, 0.20]],
    ["Motor", [1.55, 0.00, 0.52], [0.55, 0.50, 0.36]],
    ["Radiator", [2.00, 0.00, 0.50], [0.06, 0.80, 0.36]],
    ["Fluid pump", [-0.95, -0.30, 0.30], [0.45, 0.50, 0.12]],
    ["Battery", [-1.85, -0.40, 0.52], [0.30, 0.40, 0.25]],
  ];

  const SENSOR_WIRES = [
    [[1.55, 0.58, 0.60], [1.40, 0.58, 0.68], [1.25, 0.50, 0.68]],
    [[1.55, -0.25, 0.62], [1.50, -0.45, 0.70], [1.45, -0.50, 0.70]],
    [[1.98, -0.38, 0.62], [1.92, -0.50, 0.62], [1.90, -0.52, 0.56]],
    [[-0.85, -0.45, 0.34], [-0.75, -0.62, 0.36], [-0.70, -0.62, 0.36]],
    [[-1.85, -0.20, 0.56], [-1.85, 0.30, 0.56], [-1.75, 0.34, 0.52]],
  ];

  // CAN trunk, from one terminated end node to the other, along the centre tunnel
  const TRUNK = [
    [1.90, -0.55, 0.44], [1.90, -0.55, 0.27], [1.90, 0.00, 0.27],
    [1.30, 0.00, 0.27], [1.10, 0.00, 0.27], [-0.55, 0.00, 0.27],
    [-1.50, 0.00, 0.27], [-1.50, 0.45, 0.27], [-1.75, 0.45, 0.27], [-1.75, 0.45, 0.44],
  ];
  const TAPS = [4, 3, 0, 5, 9];     // trunk vertex each node joins at
  const STUBS = [                   // from the ECU down to its tap; end nodes need none
    [[1.10, 0.50, 0.62], [1.10, 0.50, 0.27], [1.10, 0.00, 0.27]],
    [[1.30, -0.50, 0.64], [1.30, -0.50, 0.27], [1.30, 0.00, 0.27]],
    [],
    [[-0.55, -0.62, 0.30], [-0.55, -0.62, 0.27], [-0.55, 0.00, 0.27]],
    [],
  ];
  const TERMINATORS = [[1.90, -0.55, 0.35], [-1.75, 0.45, 0.35]];

  // ECU-01's UART, through the bulkhead to the instrument cluster
  const UART = [[1.10, 0.50, 0.74], [1.10, 0.50, 0.84], [0.92, 0.50, 0.84],
                [0.75, 0.40, 0.93], [0.69, 0.40, 0.96]];
  const CLUSTER = [0.66, 0.40, 1.00];

  const WHEELS = [[1.38, 0.86], [1.38, -0.86], [-1.38, 0.86], [-1.38, -0.86]];
  const WHEEL_R = 0.33;
  const DOOR_HINGE = [0.95, -0.88];
  const DOOR_OPEN = 0.95;           // rad

  // Body cross-sections: x, floor half-width, floor z, side half-width, side z,
  // beltline z, roof half-width, roof z
  const STATIONS = [
    [2.28, 0.62, 0.34, 0.70, 0.46, 0.60, 0.58, 0.64],
    [2.05, 0.80, 0.24, 0.88, 0.48, 0.72, 0.72, 0.78],
    [0.95, 0.84, 0.22, 0.90, 0.52, 0.88, 0.74, 0.93],   // windscreen base, door hinge
    [0.35, 0.84, 0.22, 0.90, 0.52, 0.92, 0.60, 1.40],
    [-0.55, 0.84, 0.22, 0.90, 0.52, 0.92, 0.62, 1.42],  // door rear edge
    [-1.15, 0.84, 0.22, 0.90, 0.52, 0.92, 0.60, 1.38],
    [-1.75, 0.84, 0.22, 0.90, 0.52, 0.90, 0.72, 0.96],
    [-2.10, 0.80, 0.26, 0.88, 0.50, 0.84, 0.72, 0.90],
    [-2.28, 0.62, 0.36, 0.70, 0.50, 0.68, 0.58, 0.74],
  ];

  const BODY_RGB = [64, 104, 148];
  const GLASS_RGB = [120, 200, 232];
  const SEAT_RGB = [46, 56, 72];
  const TYRE_RGB = [24, 26, 31];
  const RIM_RGB = [172, 180, 192];
  const WARN_RGB = [255, 61, 61];

  const VIEWS = {       // name -> [yaw, pitch, distance]
    ISO: [0.75, 0.42, 8.2],
    SIDE: [Math.PI / 2, 0.06, 7.6],
    TOP: [Math.PI / 2, 1.45, 8.6],
    FRONT: [0.0, 0.14, 7.0],
  };

  // ── small vector kit ────────────────────────────────────────
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const mul = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const len = (a) => Math.hypot(a[0], a[1], a[2]);
  const unit = (a) => mul(a, 1 / len(a));
  const dist = (a, b) => len(sub(a, b));
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  function rgb(color) {
    const v = parseInt(color.slice(1), 16);
    return [v >> 16 & 255, v >> 8 & 255, v & 255];
  }

  function css(color, alpha = 1) {
    return `rgba(${color[0] | 0},${color[1] | 0},${color[2] | 0},${alpha})`;
  }

  function mix(a, b, t) {
    t = clamp(t, 0, 1);
    return a.map((x, i) => x + (b[i] - x) * t);
  }

  /* Polyline with extra points so no segment is longer than ``step``. */
  function resample(pts, step) {
    const out = [pts[0].slice()];
    for (let i = 0; i + 1 < pts.length; i++) {
      const a = pts[i], b = pts[i + 1];
      const n = Math.max(1, Math.ceil(dist(a, b) / step));
      for (let k = 1; k <= n; k++) out.push(add(a, mul(sub(b, a), k / n)));
    }
    return out;
  }

  /* The two conductors of a twisted pair following the polyline ``pts``. */
  function twistedPair(pts, radius = 0.02, pitch = 0.30, step = 0.05) {
    const c = resample(pts, step);
    const high = [], low = [];
    let s = 0;
    for (let i = 0; i < c.length; i++) {
      if (i > 0) s += dist(c[i], c[i - 1]);
      const t = unit(sub(c[Math.min(i + 1, c.length - 1)], c[Math.max(i - 1, 0)]));
      const ref = Math.abs(t[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
      const a = unit(cross(t, ref));
      const b = cross(t, a);
      const phi = 2 * Math.PI * s / pitch;
      const off = add(mul(a, radius * Math.cos(phi)), mul(b, radius * Math.sin(phi)));
      high.push(add(c[i], off));
      low.push(sub(c[i], off));
    }
    return [high, low];
  }

  function pathLength(pts) {
    let total = 0;
    for (let i = 0; i + 1 < pts.length; i++) total += dist(pts[i], pts[i + 1]);
    return total;
  }

  /* Flat-shaded polygons. Each face owns its vertices, so a named span of
   * them can be moved every frame (wheels, door) without touching the rest. */
  class Mesh {
    constructor() {
      this.verts = [];
      this.faces = [];    // {idx, color, kind, tag}
      this.spans = {};
      this._open = null;
    }

    begin(name) {
      this._open = [name, this.verts.length];
    }

    end() {
      const [name, start] = this._open;
      this.spans[name] = [start, this.verts.length];
    }

    poly(pts, color, kind = "solid", tag = null) {
      const start = this.verts.length;
      for (const p of pts) this.verts.push(p.slice());
      this.faces.push({ idx: pts.map((_, i) => start + i), color, kind, tag });
    }

    box(centre, size, color, kind = "solid", tag = null) {
      const [cx, cy, cz] = centre, [sx, sy, sz] = size;
      const x0 = cx - sx / 2, x1 = cx + sx / 2;
      const y0 = cy - sy / 2, y1 = cy + sy / 2;
      const z0 = cz - sz / 2, z1 = cz + sz / 2;
      for (const z of [z0, z1]) {
        this.poly([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]], color, kind, tag);
      }
      for (const x of [x0, x1]) {
        this.poly([[x, y0, z0], [x, y1, z0], [x, y1, z1], [x, y0, z1]], color, kind, tag);
      }
      for (const y of [y0, y1]) {
        this.poly([[x0, y, z0], [x1, y, z0], [x1, y, z1], [x0, y, z1]], color, kind, tag);
      }
    }

    /* Tyre with a spoked rim and a brake disc on its outer face. */
    wheel(cx, cy, radius, width, sides = 10) {
      const cz = radius;
      const out = cy > 0 ? 1 : -1;
      const yo = cy + out * width / 2, yi = cy - out * width / 2;
      const pt = (angle, r, y) => [cx + r * Math.cos(angle), y, cz + r * Math.sin(angle)];

      for (let i = 0; i < sides; i++) {
        const a0 = 2 * Math.PI * i / sides, a1 = 2 * Math.PI * (i + 1) / sides;
        this.poly([pt(a0, radius, yi), pt(a1, radius, yi),
                   pt(a1, radius, yo), pt(a0, radius, yo)], TYRE_RGB);
        this.poly([pt(a0, radius * 0.5, yo), pt(a1, radius * 0.5, yo),
                   pt(a1, radius * 0.9, yo), pt(a0, radius * 0.9, yo)],
                  i % 2 === 0 ? RIM_RGB : [52, 57, 66]);
        this.poly([[cx, yo, cz], pt(a0, radius * 0.5, yo), pt(a1, radius * 0.5, yo)],
                  [98, 104, 116], "solid", "brake");
      }
      const cap = [];
      for (let i = 0; i < sides; i++) cap.push(pt(2 * Math.PI * i / sides, radius, yi));
      this.poly(cap, TYRE_RGB);
    }
  }

  /* Cross-section outline: up the left side, across the roof, down the right. */
  function bodyRing(station) {
    const [x, wb, zb, ws, zm, zs, wr, zr] = station;
    const left = [[x, wb, zb], [x, ws, zm], [x, ws * 0.97, zs], [x, wr, zr]];
    return left.concat(left.slice().reverse().map(([px, py, pz]) => [px, -py, pz]));
  }

  function buildCar() {
    const m = new Mesh();
    const rings = STATIONS.map(bodyRing);
    const door = [];

    for (let s = 0; s + 1 < rings.length; s++) {
      const a = rings[s], b = rings[s + 1];
      const cabin = s === 2 || s === 3 || s === 4;
      for (let k = 0; k < 8; k++) {
        const k1 = (k + 1) % 8;
        const quad = [a[k], a[k1], b[k1], b[k]];
        let kind = "body", tag = null;
        if (k === 3 && (s === 2 || s === 5)) {              // windscreen, rear window
          kind = "glass";
        } else if ((k === 2 || k === 4) && cabin) {         // side windows
          kind = "glass";
          tag = k === 2 ? "winL" : "winR";
        } else if (k === 7) {
          kind = "floor";
        }
        if ((s === 2 || s === 3) && k >= 4 && k <= 6) {     // right door, hinged separately
          door.push([quad, kind, tag || "door"]);
        } else {
          m.poly(quad, kind === "glass" ? GLASS_RGB : BODY_RGB, kind, tag);
        }
      }
    }
    m.poly(rings[0], BODY_RGB, "body");
    m.poly(rings[rings.length - 1], BODY_RGB, "body");

    m.begin("door");
    for (const [quad, kind, tag] of door) {
      m.poly(quad, kind === "glass" ? GLASS_RGB : BODY_RGB, kind, tag);
    }
    m.end();

    WHEELS.forEach(([x, y], i) => {
      m.begin(`wheel${i}`);
      m.wheel(x, y, WHEEL_R, 0.22);
      m.end();
    });

    for (const y of [0.42, -0.42]) {                        // front seats
      m.box([0.00, y, 0.48], [0.50, 0.50, 0.14], SEAT_RGB, "ghost");
      m.box([-0.28, y, 0.80], [0.12, 0.50, 0.56], SEAT_RGB, "ghost");
    }
    m.box([-0.95, 0.0, 0.48], [0.45, 1.30, 0.14], SEAT_RGB, "ghost");
    m.box([-1.20, 0.0, 0.78], [0.12, 1.30, 0.50], SEAT_RGB, "ghost");

    for (const y of [0.50, -0.50]) {
      m.box([2.19, y, 0.56], [0.08, 0.26, 0.09], [120, 118, 96], "solid", "head");
      m.box([-2.20, y, 0.64], [0.08, 0.26, 0.08], [110, 30, 34], "solid", "tail");
    }

    PARTS.forEach(([, centre, size], i) => m.box(centre, size, [70, 80, 96], "solid", `part${i}`));
    for (const pos of TERMINATORS) m.box(pos, [0.07, 0.11, 0.06], [206, 176, 128]);

    NODES.forEach((n, i) => {
      m.box(ECU_POS[i], ECU_SIZE, mix(rgb(n.color), [0, 0, 0], 0.2), "solid", `ecu${i}`);
    });
    const [x, y, z] = ECU_POS[0];
    m.box([x - 0.09, y, z + 0.075], [0.05, 0.05, 0.03], [30, 50, 60], "solid", "led");

    m.box(CLUSTER, [0.05, 0.50, 0.20], [12, 22, 30], "solid", "screen");
    return m;
  }

  /* Orbit camera looking at ``target``; projects world points to the canvas. */
  class Camera {
    constructor(yaw, pitch, distance, target, w, h) {
      const cp = Math.cos(pitch);
      const back = [cp * Math.cos(yaw), cp * Math.sin(yaw), Math.sin(pitch)];
      this.pos = add(target, mul(back, distance));
      this.fwd = mul(back, -1);
      this.right = unit(cross(this.fwd, [0, 0, 1]));
      this.up = cross(this.right, this.fwd);
      this.scale = 0.5 * Math.min(h, w * 0.72) / Math.tan(FOV * Math.PI / 360);
      this.cx = w / 2;
      this.cy = h / 2;
    }

    view(p) {
      const rel = sub(p, this.pos);
      return [dot(rel, this.right), dot(rel, this.up), dot(rel, this.fwd)];
    }

    /* [screen x, screen y, depth] */
    project(p) {
      const [x, y, z] = this.view(p);
      const zs = Math.max(z, 1e-3);
      return [this.cx + this.scale * x / zs, this.cy - this.scale * y / zs, z];
    }

    /* Screen end points of a world segment, clipped to the near plane. */
    segment(a, b) {
      let [xa, ya, za] = this.view(a);
      let [xb, yb, zb] = this.view(b);
      if (za < NEAR && zb < NEAR) return null;
      if (za < NEAR || zb < NEAR) {
        const t = (NEAR - za) / (zb - za);
        const xc = xa + (xb - xa) * t, yc = ya + (yb - ya) * t;
        if (za < NEAR) { xa = xc; ya = yc; za = NEAR; }
        else { xb = xc; yb = yc; zb = NEAR; }
      }
      const s = this.scale;
      return [this.cx + s * xa / za, this.cy - s * ya / za,
              this.cx + s * xb / zb, this.cy - s * yb / zb];
    }
  }

  /* One frame travelling along a wire. Past ``split`` metres it has left the
   * CAN bus and is on ECU-01's UART, so it takes the second color. */
  class Pulse {
    constructor(pts, color, color2 = null, split = null, big = false) {
      this.pts = pts;
      this.cum = [0];
      for (let i = 1; i < pts.length; i++) this.cum.push(this.cum[i - 1] + dist(pts[i], pts[i - 1]));
      this.total = this.cum[this.cum.length - 1];
      this.color = color;
      this.color2 = color2 || color;
      this.split = split === null ? this.total : split;
      this.big = big;
      this.t0 = performance.now() / 1000;
    }

    at(d) {
      const cum = this.cum;
      let i = 1;
      while (i < cum.length - 1 && cum[i] < d) i++;
      const span = cum[i] - cum[i - 1];
      const t = span > 0 ? clamp((d - cum[i - 1]) / span, 0, 1) : 0;
      return add(this.pts[i - 1], mul(sub(this.pts[i], this.pts[i - 1]), t));
    }
  }

  /* The 3D scene. Left-drag orbits, right-drag pans, the wheel zooms, or,
   * over an ECU, changes that node's sensor value. */
  class CarView {
    constructor(canvas) {
      this.canvas = canvas;
      this.onSelect = () => {};     // node index, -1 for none
      this.onAdjust = () => {};     // node index, steps
      this.onTrigger = () => {};    // node index

      [this.yaw, this.pitch, this.dist] = VIEWS.ISO;
      this.target = [0, 0, 0.55];
      this._goal = null;

      this.values = NODES.map((n) => n.def);
      this.hud = {};            // cmd -> last value the dashboard was sent
      this.linked = false;
      this.led = false;
      this.xray = true;
      this.labels = true;
      this.rotate = false;
      this.selected = -1;
      this.lastFrame = "";

      this._hover = -1;
      this._press = null;
      this._last = null;
      this._dragged = false;
      this._wheelAcc = 0;
      this._spin = 0;
      this._door = 0;
      this._fx = {};            // effect name -> time it ends
      this._txUntil = NODES.map(() => 0);
      this._pulses = [];
      this._ecuPx = [];         // [x, y, radius, depth] of each ECU on screen
      this._tPrev = performance.now() / 1000;

      this._buildMesh();
      this._buildWires();
      this._buildGround();
      this._bindInput();

      const loop = () => {
        this._step();
        requestAnimationFrame(loop);
      };
      requestAnimationFrame(loop);
    }

    // ── scene ───────────────────────────────────────────────
    _buildMesh() {
      const m = buildCar();
      this._v0 = m.verts;
      this._faces = m.faces;
      this._wheelSpans = WHEELS.map(([x], i) => [m.spans[`wheel${i}`], x]);
      this._doorSpan = m.spans.door;
      this._light = unit([0.35, -0.45, 0.82]);
    }

    _buildWires() {
      this._wirePts = [];
      this._wireStyle = [];     // [rgb, width in metres, dashed, key]
      this._chunks = [];        // [first point, one past last, style index]

      const addWire = (pts, color, width, dashed = false, key = null) => {
        const base = this._wirePts.length, style = this._wireStyle.length;
        for (const p of pts) this._wirePts.push(p);
        this._wireStyle.push([rgb(color), width, dashed, key]);
        for (let a = 0; a < pts.length - 1; a += 5) {
          this._chunks.push([base + a, base + Math.min(a + 6, pts.length), style]);
        }
      };

      for (const path of [TRUNK].concat(STUBS.filter((s) => s.length))) {
        const [high, low] = twistedPair(path);
        addWire(high, COL_CAN_H, 0.016);
        addWire(low, COL_CAN_L, 0.016);
      }
      NODES.forEach((n, i) => addWire(resample(SENSOR_WIRES[i], 0.08), n.color, 0.011));
      addWire(resample(UART, 0.05), C.TEAL, 0.018, true, "uart");

      const rim = [];                                   // steering wheel and column
      for (let i = 0; i < 25; i++) {
        const a = 2 * Math.PI * i / 24;
        rim.push([0.42 - 0.05 * Math.sin(a), 0.40 + 0.17 * Math.cos(a), 0.95 + 0.17 * Math.sin(a)]);
      }
      addWire(rim, "#7C8CA0", 0.022);
      addWire(resample([[0.42, 0.40, 0.95], [0.64, 0.40, 0.98]], 0.08), "#7C8CA0", 0.022);
    }

    _buildGround() {
      this._grid = [];
      for (let x = -4.0; x <= 4.001; x += 0.5) this._grid.push([[x, -2.5, 0], [x, 2.5, 0]]);
      for (let y = -2.5; y <= 2.501; y += 0.5) this._grid.push([[-4.0, y, 0], [4.0, y, 0]]);
      this._shadow = [];
      for (let i = 0; i < 28; i++) {
        const a = 2 * Math.PI * i / 28;
        this._shadow.push([2.45 * Math.cos(a), 1.05 * Math.sin(a), 0.002]);
      }
    }

    _worldVerts() {
      const v0 = this._v0;
      const v = v0.slice();
      let c = Math.cos(this._spin), s = Math.sin(this._spin);
      for (const [[a, b], cx] of this._wheelSpans) {
        for (let i = a; i < b; i++) {
          const dx = v0[i][0] - cx, dz = v0[i][2] - WHEEL_R;
          v[i] = [cx + dx * c + dz * s, v0[i][1], WHEEL_R - dx * s + dz * c];
        }
      }
      if (this._door > 1e-3) {
        const [a, b] = this._doorSpan;
        c = Math.cos(this._door);
        s = Math.sin(this._door);
        for (let i = a; i < b; i++) {
          const dx = v0[i][0] - DOOR_HINGE[0], dy = v0[i][1] - DOOR_HINGE[1];
          v[i] = [DOOR_HINGE[0] + dx * c - dy * s, DOOR_HINGE[1] + dx * s + dy * c, v0[i][2]];
        }
      }
      return v;
    }

    // ── inputs from the bus ─────────────────────────────────
    /* A frame from node ``index`` went onto the bus. */
    onCan(index, isEvent, value, text) {
      const now = performance.now() / 1000;
      const n = NODES[index];
      this.lastFrame = text;
      if (!isEvent) this.values[index] = value;
      if (!this.linked) return;         // dashboard disconnected: the scene stays still
      this._txUntil[index] = now + TX_FLASH;
      if (isEvent) this._fx[`event${index}`] = now + (index === 3 ? 3.0 : 2.2);

      // Pulses are only retired while drawing; drop the finished ones here too,
      // so they do not pile up while another tab of the page is showing
      this._pulses = this._pulses.filter((p) => (now - p.t0) * PULSE_SPEED <= p.total + 0.3);

      if (index === 0) {
        // ECU-01 is the gateway itself: onto the bus and straight out on UART
        this._pulses.push(new Pulse(STUBS[0], n.color, null, null, isEvent));
        this._pulses.push(new Pulse(UART, C.TEAL, null, null, isEvent));
        return;
      }
      const a = TAPS[index], b = TAPS[0];
      const trunk = a <= b ? TRUNK.slice(a, b + 1) : TRUNK.slice(b, a + 1).reverse();
      const path = STUBS[index].slice(0, -1).concat(trunk, STUBS[0].slice().reverse().slice(1));
      const split = pathLength(path);
      this._pulses.push(new Pulse(path.concat(UART), n.color, C.TEAL, split, isEvent));
    }

    onUartTx(cmd, value) {
      this.hud[cmd] = value;
    }

    onUartRx() {
      this._pulses.push(new Pulse(UART.slice().reverse(), C.AMBER, null, null, true));
    }

    /* Dashboard connected or disconnected; without it every animation stops. */
    setLinked(state) {
      this.linked = state;
      if (!state) {
        this.hud = {};
        this._pulses = [];
        this._fx = {};
        this._txUntil = NODES.map(() => 0);
      }
    }

    setView(name) {
      this._goal = VIEWS[name];
      this.target = [0, 0, 0.55];
    }

    _active(name, now) {
      return now < (this._fx[name] || 0);
    }

    // ── animation ───────────────────────────────────────────
    _step() {
      const now = performance.now() / 1000;
      const dt = Math.min(0.1, now - this._tPrev);
      this._tPrev = now;
      if (this.linked) {
        this._spin = (this._spin + this.values[1] / NODES[1].vMax * 12.0 * dt) % (2 * Math.PI);
      }
      const doorGoal = this._active("event3", now) ? DOOR_OPEN : 0;
      this._door += (doorGoal - this._door) * Math.min(1, dt * 5);
      if (this._goal) {
        const k = Math.min(1, dt * 6);
        const [gy, gp, gd] = this._goal;
        const twoPi = 2 * Math.PI;
        const dyaw = (((gy - this.yaw + Math.PI) % twoPi) + twoPi) % twoPi - Math.PI;
        this.yaw += dyaw * k;
        this.pitch += (gp - this.pitch) * k;
        this.dist += (gd - this.dist) * k;
        if (Math.abs(dyaw) + Math.abs(gp - this.pitch) + Math.abs(gd - this.dist) < 0.01) {
          this._goal = null;
        }
      } else if (this.rotate && this.linked && this._press === null) {
        this.yaw += 0.25 * dt;
      }
      if (this.canvas.clientWidth && this.canvas.clientHeight) this._paint(now);
    }

    // ── drawing ─────────────────────────────────────────────
    /* Colors that follow the live state, keyed by face tag: [rgb, alpha or null]. */
    _tagColors(now) {
      const blink = !this.linked || Math.floor(now * 8) % 2 === 0;
      const out = {};
      NODES.forEach((n, i) => {
        const base = rgb(n.color);
        if (now < this._txUntil[i]) out[`ecu${i}`] = [mix(base, [255, 255, 255], 0.65), null];
        const level = this.values[i] / n.vMax;
        let part = mix(mix(base, [0, 0, 0], 0.75), base, level);
        if (this._warning(i) && blink) part = WARN_RGB;
        out[`part${i}`] = [part, null];
      });
      if (this._active("event0", now)) out.brake = [blink ? WARN_RGB : [90, 20, 20], null];
      if (this._active("event1", now)) out.winL = [rgb(NODES[1].color), 190];
      if (this._active("event2", now)) out.winR = [rgb(NODES[2].color), 190];
      if (this._active("event3", now)) out.door = [rgb(NODES[3].color), 210];
      if (this._active("event4", now)) {
        out.head = [[255, 246, 190], null];
        out.tail = [[255, 45, 45], null];
      }
      if (this.led) out.led = [rgb(C.TEAL), null];
      if (this.linked) out.screen = [[0, 104, 120], null];
      return out;
    }

    _warning(index) {
      const v = this.values[index];
      return (index === 0 && v >= S.THRESH_HIGH_PRESS) ||
             (index === 2 && v >= S.THRESH_OVERHEAT) ||
             (index === 3 && v <= S.THRESH_LOW_FLOW);
    }

    _paint(now) {
      const [p, w, h] = S.fitCanvas(this.canvas);
      const sky = p.createLinearGradient(0, 0, 0, h);
      sky.addColorStop(0, "#0B1420");
      sky.addColorStop(1, C.BG);
      p.fillStyle = sky;
      p.fillRect(0, 0, w, h);
      p.lineJoin = "round";

      const cam = new Camera(this.yaw, this.pitch, this.dist, this.target, w, h);
      this._drawGround(p, cam);
      this._drawScene(p, cam, now);
      this._drawLamps(p, cam, now);
      this._drawPulses(p, cam, now);
      this._drawEcuRings(p, cam);
      this._drawCallouts(p, cam, now, w);
      this._drawOverlay(p, w, h);
    }

    _drawGround(p, cam) {
      p.strokeStyle = "rgba(36,51,72,0.59)";
      p.lineWidth = 1;
      p.beginPath();
      for (const [a, b] of this._grid) {
        const seg = cam.segment(a, b);
        if (seg) {
          p.moveTo(seg[0], seg[1]);
          p.lineTo(seg[2], seg[3]);
        }
      }
      p.stroke();
      const pts = this._shadow.map((q) => cam.project(q));
      if (pts.every((q) => q[2] > NEAR)) {
        p.fillStyle = "rgba(0,0,0,0.43)";
        p.beginPath();
        pts.forEach((q, i) => (i ? p.lineTo(q[0], q[1]) : p.moveTo(q[0], q[1])));
        p.closePath();
        p.fill();
      }
    }

    /* Faces and wires together, far to near. */
    _drawScene(p, cam, now) {
      const v = this._worldVerts();
      const P = v.map((q) => cam.project(q));
      const W = this._wirePts.map((q) => cam.project(q));
      const faces = this._faces;
      const nf = faces.length;

      const depth = new Float64Array(nf + this._chunks.length);
      const hidden = new Uint8Array(nf);
      for (let k = 0; k < nf; k++) {
        const idx = faces[k].idx;
        let sum = 0, zmin = Infinity;
        for (const i of idx) {
          sum += P[i][2];
          if (P[i][2] < zmin) zmin = P[i][2];
        }
        depth[k] = sum / idx.length;
        hidden[k] = zmin < NEAR ? 1 : 0;
      }
      this._chunks.forEach(([a, b], k) => { depth[nf + k] = W[(a + b) >> 1][2]; });
      const order = Array.from(depth.keys()).sort((i, j) => depth[j] - depth[i]);

      const tagged = this._tagColors(now);
      const styles = this.xray ? {
        body: [46, "rgba(140,186,226,0.39)"],
        glass: [30, "rgba(150,220,240,0.35)"],
        floor: [150, "rgba(90,120,150,0.31)"],
        ghost: [70, "rgba(100,120,144,0.31)"],
        solid: [255, "rgba(6,10,16,0.67)"],
      } : {
        body: [242, "rgba(20,34,52,0.78)"],
        glass: [120, "rgba(150,220,240,0.47)"],
        floor: [255, "rgba(20,34,52,0.78)"],
        ghost: [255, "rgba(20,26,34,0.78)"],
        solid: [255, "rgba(6,10,16,0.67)"],
      };
      const uartRgb = rgb(this.linked ? C.TEAL : C.MUTED);

      for (const k of order) {
        if (k < nf) {
          if (hidden[k]) continue;
          const face = faces[k];
          let [alpha, pen] = styles[face.kind];
          let color = face.color;
          const live = face.tag && tagged[face.tag];
          if (live) {
            color = live[0];
            alpha = live[1] || alpha;
          }
          const [i0, i1, i2] = face.idx;
          const normal = cross(sub(v[i1], v[i0]), sub(v[i2], v[i0]));
          const s = 0.5 + 0.5 * Math.abs(dot(normal, this._light)) / Math.max(len(normal), 1e-9);
          p.setLineDash([]);
          p.lineWidth = 1;
          p.strokeStyle = pen;
          p.fillStyle = css(mul(color, s), alpha / 255);
          p.beginPath();
          face.idx.forEach((i, n) => (n ? p.lineTo(P[i][0], P[i][1]) : p.moveTo(P[i][0], P[i][1])));
          p.closePath();
          p.fill();
          p.stroke();
        } else {
          const [a, b, style] = this._chunks[k - nf];
          let near = false;
          for (let i = a; i < b; i++) if (W[i][2] < NEAR) near = true;
          if (near) continue;
          let [color, width, dashed, key] = this._wireStyle[style];
          if (key === "uart") color = uartRgb;
          p.strokeStyle = css(color);
          p.lineWidth = Math.max(1.2, width * cam.scale / W[(a + b) >> 1][2]);
          p.lineCap = "round";
          p.setLineDash(dashed ? [p.lineWidth * 2.5, p.lineWidth * 2] : []);
          p.beginPath();
          for (let i = a; i < b; i++) (i > a ? p.lineTo(W[i][0], W[i][1]) : p.moveTo(W[i][0], W[i][1]));
          p.stroke();
        }
      }
      p.setLineDash([]);
      p.lineCap = "butt";
    }

    _glow(p, x, y, radius, color, core = true) {
      const c = rgb(color);
      const g = p.createRadialGradient(x, y, 0, x, y, radius);
      if (core) {
        g.addColorStop(0, "rgba(255,255,255,0.94)");
        g.addColorStop(0.3, css(c, 0.82));
      } else {
        g.addColorStop(0, css(c, 0.67));
      }
      g.addColorStop(1, css(c, 0));
      p.fillStyle = g;
      p.beginPath();
      p.arc(x, y, radius, 0, 2 * Math.PI);
      p.fill();
    }

    _drawLamps(p, cam, now) {
      if (!this._active("event4", now)) return;
      const lamps = [];
      for (const y of [0.5, -0.5]) lamps.push([[2.24, y, 0.56], "#FFF6BE"]);
      for (const y of [0.5, -0.5]) lamps.push([[-2.25, y, 0.64], "#FF2D2D"]);
      for (const [pos, color] of lamps) {
        const [x, y, z] = cam.project(pos);
        if (z > NEAR) this._glow(p, x, y, 0.34 * cam.scale / z, color, false);
      }
    }

    _drawPulses(p, cam, now) {
      const alive = [];
      for (const pulse of this._pulses) {
        const d = (now - pulse.t0) * PULSE_SPEED;
        if (d > pulse.total + 0.3) continue;
        alive.push(pulse);
        for (let k = 0; k < 4; k++) {                   // head and a fading tail
          const dk = d - k * 0.07;
          if (dk < 0 || dk > pulse.total) continue;
          const color = dk <= pulse.split ? pulse.color : pulse.color2;
          const size = (pulse.big ? 0.085 : 0.05) * (1.0 - 0.2 * k);
          const [x, y, z] = cam.project(pulse.at(dk));
          if (z > NEAR) this._glow(p, x, y, Math.max(3.0, size * cam.scale / z), color);
        }
      }
      this._pulses = alive;
    }

    _drawEcuRings(p, cam) {
      this._ecuPx = ECU_POS.map((pos) => {
        const [x, y, z] = cam.project(pos);
        return [x, y, Math.max(14.0, 0.21 * cam.scale / Math.max(z, NEAR)), z];
      });
      this._ecuPx.forEach(([px, py, radius, depth], i) => {
        if (depth < NEAR || (i !== this.selected && i !== this._hover)) return;
        p.strokeStyle = NODES[i].color;
        p.lineWidth = i === this.selected ? 2.5 : 1.5;
        p.setLineDash(i === this.selected ? [] : [2, 3]);
        p.beginPath();
        p.arc(px, py, radius, 0, 2 * Math.PI);
        p.stroke();
      });
      p.setLineDash([]);
    }

    /* Size and painter of a small label; ``lines`` is [[text, size, bold, color]]. */
    _textBox(p, lines, border, strong = false, plain = false) {
      const pad = plain ? 3 : 7;
      const heights = lines.map(([, size]) => Math.ceil(size * 1.25));
      let w = 0;
      for (const [text, size, bold] of lines) {
        p.font = S.font(size, bold);
        w = Math.max(w, p.measureText(text).width);
      }
      w += 2 * pad + 4;
      const h = heights.reduce((a, b) => a + b, 0) + 2 * pad;

      const draw = (rect) => {
        p.fillStyle = css(rgb(C.PANEL), plain ? 0.59 : 0.89);
        p.strokeStyle = css(rgb(border), plain ? 0.43 : 1);
        p.lineWidth = strong ? 2 : 1;
        S.roundRect(p, rect.x, rect.y, rect.w, rect.h, 5);
        p.fill();
        p.stroke();
        let y = rect.y + pad;
        p.textAlign = "center";
        p.textBaseline = "middle";
        lines.forEach(([text, size, bold, color], i) => {
          p.font = S.font(size, bold);
          p.fillStyle = color;
          p.fillText(text, rect.x + rect.w / 2, y + heights[i] / 2 + 1);
          y += heights[i];
        });
      };
      return [w, h, draw];
    }

    /* What the dashboard has been sent over UART, shown at the cluster. */
    _hudBox(p) {
      const w = 214, h = 104;
      const draw = (rect) => {
        const edge = this.linked ? C.TEAL : C.MUTED;
        p.fillStyle = css(rgb(C.PANEL), 0.92);
        p.strokeStyle = edge;
        p.lineWidth = 1.5;
        S.roundRect(p, rect.x, rect.y, w, h, 6);
        p.fill();
        p.stroke();
        p.textAlign = "center";
        p.textBaseline = "middle";
        p.font = S.font(10, true);
        p.fillStyle = edge;
        p.fillText("PC DASHBOARD  ·  UART 115200", rect.x + w / 2, rect.y + 12);
        p.font = S.font(9, false);
        p.fillStyle = C.TEXT2;
        p.fillText(this.linked ? "ST-Link VCP  ·  CONNECTED" : "NOT CONNECTED", rect.x + w / 2, rect.y + 25);
        const col = (w - 16) / NODES.length;
        NODES.forEach((n, i) => {
          const x = rect.x + 8 + i * col;
          const value = this.hud[n.cmd];
          const known = value !== undefined;
          p.fillStyle = known ? n.color : C.MUTED;
          p.fillText(known ? String(value) : "--", x + col / 2, rect.y + 40);
          const bx = x + 6, by = rect.y + 48, bw = col - 12, bh = 34;
          p.fillStyle = C.CARD2;
          S.roundRect(p, bx, by, bw, bh, 2);
          p.fill();
          if (known) {
            const fill = bh * Math.min(1, value / n.vMax);
            if (fill > 0.5) {
              p.fillStyle = n.color;
              S.roundRect(p, bx, by + bh - fill, bw, fill, Math.min(2, fill / 2));
              p.fill();
            }
          }
          p.fillStyle = C.TEXT2;
          p.fillText(n.unit, x + col / 2, rect.y + 92);
        });
      };
      return [w, h, draw];
    }

    _drawCallouts(p, cam, now, w) {
      const anchors = [];     // [world position, lift in px, [w, h, draw], leader color]
      const blink = Math.floor(now * 6) % 2 === 0;

      NODES.forEach((n, i) => {
        const warn = this._warning(i);
        const lines = [[`${n.ecu}  ·  ${n.sensor.toUpperCase()}`, 10, true, n.color],
                       [`${this.values[i]} ${n.unit}`, 16, true, warn ? C.RED : C.TEXT]];
        if (this._active(`event${i}`, now)) {
          lines.push([`●  ${n.evName.toUpperCase()}`, 11, true, blink ? n.color : C.TEXT]);
        } else if (i === this._hover) {
          lines.push([`scroll: value   dbl-click: ${n.evName}`, 9, false, C.TEXT2]);
        }
        const [x, y, z] = ECU_POS[i];
        const box = this._textBox(p, lines, n.color, i === this.selected || i === this._hover);
        anchors.push([[x, y, z + 0.07], 38, box, n.color]);
      });

      anchors.push([CLUSTER, 60, this._hudBox(p), this.linked ? C.TEAL : C.MUTED]);

      if (this.labels) {
        const small = PARTS.map(([name, c, s]) => [[c[0], c[1], c[2] + s[2] / 2], name]);
        for (const pos of TERMINATORS) small.push([pos, "120 Ω"]);
        small.push([[0.25, 0.0, 0.27], "CAN_H / CAN_L  ·  twisted pair"]);
        small.push([UART[2], "UART  ·  ST-Link VCP"]);
        for (const [pos, text] of small) {
          const box = this._textBox(p, [[text, 9, false, C.TEXT2]], C.BORDER2, false, true);
          anchors.push([pos, 16, box, C.BORDER2]);
        }
      }

      const proj = anchors.map((a) => cam.project(a[0]));
      const order = anchors.map((_, i) => i).sort((i, j) => proj[i][2] - proj[j][2]);
      const hit = (a, b) => a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
      const placed = [];
      // nearest first, so the labels in front keep the spot next to their anchor
      for (const i of order) {
        const [x, y, z] = proj[i];
        if (z < NEAR) continue;
        const [, lift, [bw, bh, draw], color] = anchors[i];
        const rect = { x: x - bw / 2, y: y - lift - bh, w: bw, h: bh };
        rect.x = Math.min(Math.max(rect.x, 4), Math.max(4, w - bw - 4));
        for (let n = 0; n < 70; n++) {
          const grown = { x: rect.x - 3, y: rect.y - 3, w: rect.w + 6, h: rect.h + 6 };
          if (!placed.some((other) => hit(grown, other.rect))) break;
          rect.y -= 6;
        }
        placed.push({ rect, ax: x, ay: y, draw, color });
      }

      for (const { rect, ax, ay, color } of placed) {
        const c = css(rgb(color), 0.67);
        p.strokeStyle = c;
        p.lineWidth = 1;
        p.beginPath();
        p.moveTo(ax, ay);
        p.lineTo(rect.x + rect.w / 2, rect.y + rect.h);
        p.stroke();
        p.fillStyle = c;
        p.beginPath();
        p.arc(ax, ay, 2.5, 0, 2 * Math.PI);
        p.fill();
      }
      for (let i = placed.length - 1; i >= 0; i--) placed[i].draw(placed[i].rect);
    }

    _drawOverlay(p, w, h) {
      p.textBaseline = "middle";
      p.textAlign = "left";
      p.font = S.font(13, true);
      p.fillStyle = C.AMBER;
      p.fillText("VIRTUAL  CAR   ·   CAN  NETWORK  IN  3D", 14, 19);
      p.font = S.font(11, false);
      p.fillStyle = C.TEXT;
      p.fillText(this.lastFrame, 14, 38);
      if (!this.linked) {
        p.font = S.font(12, true);
        p.fillStyle = C.RED;
        p.textAlign = "center";
        p.fillText("DASHBOARD  DISCONNECTED   ·   ANIMATION  PAUSED", w / 2, 63);
        p.textAlign = "left";
      }

      p.font = S.font(10, false);
      p.fillStyle = C.TEXT2;
      p.fillText(w > 760
        ? "drag: orbit    right-drag: pan    wheel: zoom    " +
          "wheel over an ECU: change its value    double-click an ECU: press its button"
        : "drag: orbit   wheel: zoom   double-click an ECU: press its button", 14, h - 15);

      const legend = [[COL_CAN_H, "CAN_H", false], [COL_CAN_L, "CAN_L", false],
                      [C.TEAL, "UART to dashboard", true], [C.TEXT, "sensor wire", false]];
      let y = h - 46 - 16 * legend.length;
      for (const [color, text, dashed] of legend) {
        p.strokeStyle = color;
        p.lineWidth = 3;
        p.setLineDash(dashed ? [6, 4] : []);
        p.beginPath();
        p.moveTo(w - 170, y + 7);
        p.lineTo(w - 140, y + 7);
        p.stroke();
        p.fillStyle = C.TEXT2;
        p.fillText(text, w - 132, y + 7);
        y += 16;
      }
      p.setLineDash([]);
    }

    // ── mouse and keyboard ──────────────────────────────────
    _pick(x, y) {
      let best = -1, bestD = 1e9;
      this._ecuPx.forEach(([ex, ey, radius, depth], i) => {
        const d = Math.hypot(x - ex, y - ey);
        if (depth > NEAR && d <= radius && d < bestD) {
          best = i;
          bestD = d;
        }
      });
      return best;
    }

    _bindInput() {
      const cv = this.canvas;
      const at = (e) => {
        const box = cv.getBoundingClientRect();
        return [e.clientX - box.left, e.clientY - box.top];
      };

      cv.addEventListener("pointerdown", (e) => {
        cv.setPointerCapture(e.pointerId);
        cv.focus({ preventScroll: true });
        this._press = this._last = at(e);
        this._dragged = false;
      });

      cv.addEventListener("pointermove", (e) => {
        const pos = at(e);
        if (this._press === null) {
          const hover = this._pick(pos[0], pos[1]);
          if (hover !== this._hover) {
            this._hover = hover;
            cv.style.cursor = hover >= 0 ? "pointer" : "grab";
          }
          return;
        }
        const dx = pos[0] - this._last[0], dy = pos[1] - this._last[1];
        this._last = pos;
        if (Math.abs(pos[0] - this._press[0]) + Math.abs(pos[1] - this._press[1]) > 4) this._dragged = true;
        if (!this._dragged) return;
        this._goal = null;
        if ((e.buttons & 1) && !e.shiftKey) {
          this.yaw -= dx * 0.008;
          this.pitch = clamp(this.pitch + dy * 0.006, 0.03, 1.5);
        } else {
          const cam = new Camera(this.yaw, this.pitch, this.dist, this.target,
                                 cv.clientWidth, cv.clientHeight);
          const k = this.dist / cam.scale;
          const t = add(sub(this.target, mul(cam.right, dx * k)), mul(cam.up, dy * k));
          this.target = [clamp(t[0], -2.5, 2.5), clamp(t[1], -1.5, 1.5), clamp(t[2], 0, 1.6)];
        }
      });

      const release = (e) => {
        if (this._press !== null && !this._dragged && e.button === 0 && e.type === "pointerup") {
          const pos = at(e);
          this.onSelect(this._pick(pos[0], pos[1]));
        }
        this._press = null;
      };
      cv.addEventListener("pointerup", release);
      cv.addEventListener("pointercancel", release);
      cv.addEventListener("contextmenu", (e) => e.preventDefault());

      cv.addEventListener("dblclick", (e) => {
        const pos = at(e);
        const index = this._pick(pos[0], pos[1]);
        if (index >= 0) this.onTrigger(index);
      });

      cv.addEventListener("wheel", (e) => {
        e.preventDefault();
        const notches = e.deltaMode === 1 ? -e.deltaY / 3 : -e.deltaY / 100;
        this._wheelAcc += notches;
        if (this._hover < 0) {
          this._goal = null;
          this.dist = clamp(this.dist * 0.9 ** this._wheelAcc, 2.2, 16.0);
          this._wheelAcc = 0;
          return;
        }
        const steps = Math.trunc(this._wheelAcc);
        if (steps) {
          this._wheelAcc -= steps;
          this.onAdjust(this._hover, steps);
        }
      }, { passive: false });

      cv.addEventListener("keydown", (e) => {
        if (this.selected < 0) return;
        const steps = { ArrowUp: 1, ArrowRight: 1, ArrowDown: -1, ArrowLeft: -1,
                        PageUp: 5, PageDown: -5 }[e.key];
        if (steps) {
          e.preventDefault();
          this.onAdjust(this.selected, steps);
        } else if (e.key === " ") {
          e.preventDefault();
          this.onTrigger(this.selected);
        }
      });
    }
  }

  /* One ECU's inputs: the sensor value at its location and its push button. */
  class EcuControl {
    constructor(index, n, bus, onClick) {
      this.index = index;
      this.node = n;
      this._bus = bus;
      this.el = S.el("div", "ecu-control",
        `<div class="head"><span class="nm">${n.ecu}  ·  ${n.owner.toUpperCase()}</span>` +
        `<span class="val"></span></div>` +
        `<div class="small">${n.sensor}  ·  CAN 0x${S.hex(n.canId, 3)}  →  UART 0x${S.hex(n.cmd, 2)}</div>` +
        `<input type="range" aria-label="${n.sensor}">` +
        `<div class="row"><button type="button">●  ${n.evName.toUpperCase()}</button>` +
        `<label><input type="checkbox"> Auto</label></div>`);
      this.el.style.setProperty("--c", n.color);
      this._slider = this.el.querySelector("input[type=range]");
      S.bindSensor(bus, n, this._slider, this.el.querySelector(".val"),
                   this.el.querySelector("input[type=checkbox]"), () => onClick(index));
      this.el.querySelector("button").addEventListener("click", () => bus.trigger(n.evCmd));
      this.el.addEventListener("pointerdown", () => onClick(index));
    }

    /* Wheel or arrow keys over the ECU in the scene: a hand-set value, like the slider. */
    nudge(steps) {
      const n = this.node;
      const step = Math.max(1, Math.floor(n.vMax / 50));
      const value = clamp(this._bus.values[n.cmd] + steps * step, 0, n.vMax);
      this._bus.setAuto(n.cmd, false);
      this._bus.setValue(n.cmd, value);
    }

    setSelected(state) {
      this.el.classList.toggle("selected", state);
    }
  }

  class Car3D {
    constructor(root, bus) {
      this._byCan = S.canIndex();
      this._view = new CarView(root.querySelector("#c-canvas"));
      this._view.onSelect = (i) => this._select(i);
      this._view.onAdjust = (i, steps) => { this._select(i); this._cards[i].nudge(steps); };
      this._view.onTrigger = (i) => bus.trigger(NODES[i].evCmd);

      const views = root.querySelector("#c-views");
      for (const name of Object.keys(VIEWS)) {
        const btn = S.el("button", "", name);
        btn.type = "button";
        btn.addEventListener("click", () => this._view.setView(name));
        views.appendChild(btn);
      }
      for (const chk of root.querySelectorAll("[data-opt]")) {
        chk.checked = this._view[chk.dataset.opt];
        chk.addEventListener("change", () => { this._view[chk.dataset.opt] = chk.checked; });
      }

      const cards = root.querySelector("#c-cards");
      this._cards = NODES.map((n, i) => {
        const card = new EcuControl(i, n, bus, (index) => this._select(index));
        cards.appendChild(card.el);
        return card;
      });
      S.bindAutoAll(bus, root.querySelector("#c-auto-all"));

      bus.listen((kind, a, b, c) => this._onBus(kind, a, b, c));
    }

    _select(index) {
      if (index === this._view.selected) return;
      this._view.selected = index;
      for (const card of this._cards) card.setSelected(card.index === index);
      if (index >= 0) this._cards[index].el.scrollIntoView({ block: "nearest" });
    }

    _onBus(kind, a, b, c) {
      if (kind === "can") {
        const [index, n, isEvent] = this._byCan[a];
        this._view.onCan(index, isEvent, c, S.describeCan(n, isEvent, a, c));
      } else if (kind === "value") {
        this._view.values[NODES.findIndex((n) => n.cmd === a)] = b;
      } else if (kind === "uart_tx") {
        this._view.onUartTx(a, b);
      } else if (kind === "uart_rx") {
        this._view.onUartRx();
      } else if (kind === "led") {
        this._view.led = !!a;
      } else if (kind === "link") {
        this._view.setLinked(!!a);
      }
    }
  }

  S.Car3D = Car3D;
})();
