"""Virtual car in 3D: the five ECUs where they sit in a vehicle, and their wiring.

The CAN twisted pair runs the length of the car between the two terminated end
nodes, every other ECU hangs off it on a stub, and ECU-01's UART leaves for the
instrument cluster, which stands in for the PC dashboard. Scroll over an ECU (or
use its slider) to change the sensor value at that location: the frame travels
down the harness to ECU-01 and on to the dashboard, as on the real hardware.

Run with ``python dashboard/stm32_dashboard.py --3d`` (car + dashboard) or
``python dashboard/virtual_car3d.py`` (car only, then connect a dashboard to
the "Virtual ECU (simulator)" port).

Drawn with QPainter and a depth sort, so it needs no OpenGL.
"""

import math
import sys
import time
from functools import lru_cache

import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QCheckBox, QSlider, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QColor, QPainter, QPen, QPolygonF, QFontMetrics, QLinearGradient, QRadialGradient
)

if __package__:
    from .config import *
    from .virtual_bench import BENCH_QSS, px_font, tint
    from .virtual_ecu import NODES, URL, VirtualBus
else:
    from config import *
    from virtual_bench import BENCH_QSS, px_font, tint
    from virtual_ecu import NODES, URL, VirtualBus

# World axes: x towards the front of the car, y to its left, z up. Units are metres.
FOV         = 34.0
NEAR        = 0.3
FRAME_MS    = 33
PULSE_SPEED = 5.0      # m/s, slowed down enormously so a frame can be followed by eye
TX_FLASH    = 0.18     # s

COL_CAN_H = "#FFD54F"
COL_CAN_L = "#66BB6A"

# ── where things are ────────────────────────────────────────
ECU_SIZE = (0.30, 0.22, 0.12)
ECU_POS = [
    (1.10,  0.50, 0.68),    # ECU-01  engine bay left, next to the brake unit
    (1.30, -0.50, 0.70),    # ECU-02  engine bay right, next to the motor
    (1.90, -0.55, 0.50),    # ECU-03  behind the radiator   (bus end, 120 Ω)
    (-0.55, -0.62, 0.36),   # ECU-04  floor, by the right door
    (-1.75, 0.45, 0.50),    # ECU-05  boot                   (bus end, 120 Ω)
]

# The sensed component of each node: (label, centre, size)
PARTS = [
    ("Brake pressure unit", (1.65,  0.58, 0.55), (0.20, 0.20, 0.20)),
    ("Motor",               (1.55,  0.00, 0.52), (0.55, 0.50, 0.36)),
    ("Radiator",            (2.00,  0.00, 0.50), (0.06, 0.80, 0.36)),
    ("Fluid pump",          (-0.95, -0.30, 0.30), (0.45, 0.50, 0.12)),
    ("Battery",             (-1.85, -0.40, 0.52), (0.30, 0.40, 0.25)),
]

SENSOR_WIRES = [
    [(1.55, 0.58, 0.60), (1.40, 0.58, 0.68), (1.25, 0.50, 0.68)],
    [(1.55, -0.25, 0.62), (1.50, -0.45, 0.70), (1.45, -0.50, 0.70)],
    [(1.98, -0.38, 0.62), (1.92, -0.50, 0.62), (1.90, -0.52, 0.56)],
    [(-0.85, -0.45, 0.34), (-0.75, -0.62, 0.36), (-0.70, -0.62, 0.36)],
    [(-1.85, -0.20, 0.56), (-1.85, 0.30, 0.56), (-1.75, 0.34, 0.52)],
]

# CAN trunk, from one terminated end node to the other, along the centre tunnel
TRUNK = [
    (1.90, -0.55, 0.44), (1.90, -0.55, 0.27), (1.90, 0.00, 0.27),
    (1.30, 0.00, 0.27), (1.10, 0.00, 0.27), (-0.55, 0.00, 0.27),
    (-1.50, 0.00, 0.27), (-1.50, 0.45, 0.27), (-1.75, 0.45, 0.27), (-1.75, 0.45, 0.44),
]
TAPS = [4, 3, 0, 5, 9]      # trunk vertex each node joins at
STUBS = [                   # from the ECU down to its tap; end nodes need none
    [(1.10, 0.50, 0.62), (1.10, 0.50, 0.27), (1.10, 0.00, 0.27)],
    [(1.30, -0.50, 0.64), (1.30, -0.50, 0.27), (1.30, 0.00, 0.27)],
    [],
    [(-0.55, -0.62, 0.30), (-0.55, -0.62, 0.27), (-0.55, 0.00, 0.27)],
    [],
]
TERMINATORS = [(1.90, -0.55, 0.35), (-1.75, 0.45, 0.35)]

# ECU-01's UART, through the bulkhead to the instrument cluster
UART = [(1.10, 0.50, 0.74), (1.10, 0.50, 0.84), (0.92, 0.50, 0.84),
        (0.75, 0.40, 0.93), (0.69, 0.40, 0.96)]
CLUSTER = (0.66, 0.40, 1.00)

WHEELS     = [(1.38, 0.86), (1.38, -0.86), (-1.38, 0.86), (-1.38, -0.86)]
WHEEL_R    = 0.33
DOOR_HINGE = (0.95, -0.88)
DOOR_OPEN  = 0.95           # rad

# Body cross-sections: x, floor half-width, floor z, side half-width, side z,
# beltline z, roof half-width, roof z
STATIONS = [
    (2.28,  0.62, 0.34, 0.70, 0.46, 0.60, 0.58, 0.64),
    (2.05,  0.80, 0.24, 0.88, 0.48, 0.72, 0.72, 0.78),
    (0.95,  0.84, 0.22, 0.90, 0.52, 0.88, 0.74, 0.93),   # windscreen base, door hinge
    (0.35,  0.84, 0.22, 0.90, 0.52, 0.92, 0.60, 1.40),
    (-0.55, 0.84, 0.22, 0.90, 0.52, 0.92, 0.62, 1.42),   # door rear edge
    (-1.15, 0.84, 0.22, 0.90, 0.52, 0.92, 0.60, 1.38),
    (-1.75, 0.84, 0.22, 0.90, 0.52, 0.90, 0.72, 0.96),
    (-2.10, 0.80, 0.26, 0.88, 0.50, 0.84, 0.72, 0.90),
    (-2.28, 0.62, 0.36, 0.70, 0.50, 0.68, 0.58, 0.74),
]

BODY_RGB  = (64, 104, 148)
GLASS_RGB = (120, 200, 232)
SEAT_RGB  = (46, 56, 72)
TYRE_RGB  = (24, 26, 31)
RIM_RGB   = (172, 180, 192)
WARN_RGB  = (255, 61, 61)

VIEWS = {       # name -> (yaw, pitch, distance)
    "ISO":   (0.75, 0.42, 8.2),
    "SIDE":  (math.pi / 2, 0.06, 7.6),
    "TOP":   (math.pi / 2, 1.45, 8.6),
    "FRONT": (0.0, 0.14, 7.0),
}


def rgb(color):
    c = QColor(color)
    return (c.red(), c.green(), c.blue())


@lru_cache(maxsize=None)
def label_font(size, bold=False):
    """Font and its metrics, kept so the labels are not rebuilt every frame."""
    font = px_font(size, bold)
    return font, QFontMetrics(font)


def mix(a, b, t):
    t = min(1.0, max(0.0, t))
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def resample(pts, step):
    """Polyline with extra points so no segment is longer than ``step``."""
    out = [np.array(pts[0], float)]
    for a, b in zip(pts, pts[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step)))
        out.extend(a + (b - a) * k / n for k in range(1, n + 1))
    return np.array(out)


def twisted_pair(pts, radius=0.02, pitch=0.30, step=0.05):
    """The two conductors of a twisted pair following the polyline ``pts``."""
    c = resample(pts, step)
    t = np.gradient(c, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    ref = np.where(np.abs(t[:, 2:3]) > 0.9, [[1.0, 0.0, 0.0]], [[0.0, 0.0, 1.0]])
    a = np.cross(t, ref)
    a /= np.linalg.norm(a, axis=1, keepdims=True)
    b = np.cross(t, a)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
    phi = (2 * math.pi * s / pitch)[:, None]
    off = radius * (np.cos(phi) * a + np.sin(phi) * b)
    return c + off, c - off


class Mesh:
    """Flat-shaded polygons. Each face owns its vertices, so a named span of
    them can be moved every frame (wheels, door) without touching the rest."""

    def __init__(self):
        self.verts = []
        self.faces = []     # (vertex indices, rgb, kind, tag)
        self.spans = {}
        self._open = None

    def begin(self, name):
        self._open = (name, len(self.verts))

    def end(self):
        name, start = self._open
        self.spans[name] = (start, len(self.verts))

    def poly(self, pts, color, kind="solid", tag=None):
        start = len(self.verts)
        self.verts.extend(pts)
        self.faces.append((tuple(range(start, start + len(pts))), color, kind, tag))

    def box(self, centre, size, color, kind="solid", tag=None):
        (cx, cy, cz), (sx, sy, sz) = centre, size
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        z0, z1 = cz - sz / 2, cz + sz / 2
        for z in (z0, z1):
            self.poly([(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)], color, kind, tag)
        for x in (x0, x1):
            self.poly([(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)], color, kind, tag)
        for y in (y0, y1):
            self.poly([(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)], color, kind, tag)

    def wheel(self, cx, cy, radius, width, sides=10):
        """Tyre with a spoked rim and a brake disc on its outer face."""
        cz = radius
        out = 1 if cy > 0 else -1
        yo, yi = cy + out * width / 2, cy - out * width / 2

        def pt(angle, r, y):
            return (cx + r * math.cos(angle), y, cz + r * math.sin(angle))

        for i in range(sides):
            a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
            self.poly([pt(a0, radius, yi), pt(a1, radius, yi),
                       pt(a1, radius, yo), pt(a0, radius, yo)], TYRE_RGB)
            self.poly([pt(a0, radius * 0.5, yo), pt(a1, radius * 0.5, yo),
                       pt(a1, radius * 0.9, yo), pt(a0, radius * 0.9, yo)],
                      RIM_RGB if i % 2 == 0 else (52, 57, 66))
            self.poly([(cx, yo, cz), pt(a0, radius * 0.5, yo), pt(a1, radius * 0.5, yo)],
                      (98, 104, 116), tag="brake")
        self.poly([pt(2 * math.pi * i / sides, radius, yi) for i in range(sides)], TYRE_RGB)


def body_ring(station):
    """Cross-section outline: up the left side, across the roof, down the right."""
    x, wb, zb, ws, zm, zs, wr, zr = station
    left = [(x, wb, zb), (x, ws, zm), (x, ws * 0.97, zs), (x, wr, zr)]
    return left + [(px, -py, pz) for px, py, pz in reversed(left)]


def build_car():
    m = Mesh()
    rings = [body_ring(s) for s in STATIONS]
    door = []

    for s in range(len(rings) - 1):
        a, b = rings[s], rings[s + 1]
        cabin = s in (2, 3, 4)
        for k in range(8):
            k1 = (k + 1) % 8
            quad = [a[k], a[k1], b[k1], b[k]]
            kind, tag = "body", None
            if k == 3 and s in (2, 5):                  # windscreen, rear window
                kind = "glass"
            elif k in (2, 4) and cabin:                 # side windows
                kind, tag = "glass", "winL" if k == 2 else "winR"
            elif k == 7:
                kind = "floor"
            if s in (2, 3) and k in (4, 5, 6):          # right door, hinged separately
                door.append((quad, kind, tag or "door"))
            else:
                m.poly(quad, GLASS_RGB if kind == "glass" else BODY_RGB, kind, tag)
    m.poly(rings[0], BODY_RGB, "body")
    m.poly(rings[-1], BODY_RGB, "body")

    m.begin("door")
    for quad, kind, tag in door:
        m.poly(quad, GLASS_RGB if kind == "glass" else BODY_RGB, kind, tag)
    m.end()

    for i, (x, y) in enumerate(WHEELS):
        m.begin(f"wheel{i}")
        m.wheel(x, y, WHEEL_R, 0.22)
        m.end()

    for y in (0.42, -0.42):                             # front seats
        m.box((0.00, y, 0.48), (0.50, 0.50, 0.14), SEAT_RGB, "ghost")
        m.box((-0.28, y, 0.80), (0.12, 0.50, 0.56), SEAT_RGB, "ghost")
    m.box((-0.95, 0.0, 0.48), (0.45, 1.30, 0.14), SEAT_RGB, "ghost")
    m.box((-1.20, 0.0, 0.78), (0.12, 1.30, 0.50), SEAT_RGB, "ghost")

    for y in (0.50, -0.50):
        m.box((2.19, y, 0.56), (0.08, 0.26, 0.09), (120, 118, 96), tag="head")
        m.box((-2.20, y, 0.64), (0.08, 0.26, 0.08), (110, 30, 34), tag="tail")

    for i, (_, centre, size) in enumerate(PARTS):
        m.box(centre, size, (70, 80, 96), tag=f"part{i}")
    for pos in TERMINATORS:
        m.box(pos, (0.07, 0.11, 0.06), (206, 176, 128))

    for i, node in enumerate(NODES):
        m.box(ECU_POS[i], ECU_SIZE, mix(rgb(node.color), (0, 0, 0), 0.2), tag=f"ecu{i}")
    x, y, z = ECU_POS[0]
    m.box((x - 0.09, y, z + 0.075), (0.05, 0.05, 0.03), (30, 50, 60), tag="led")

    m.box(CLUSTER, (0.05, 0.50, 0.20), (12, 22, 30), tag="screen")
    return m


class Camera:
    """Orbit camera looking at ``target``; projects world points to the widget."""

    def __init__(self, yaw, pitch, dist, target, w, h):
        cp = math.cos(pitch)
        back = np.array([cp * math.cos(yaw), cp * math.sin(yaw), math.sin(pitch)])
        self.pos = target + dist * back
        self.fwd = -back
        self.right = np.cross(self.fwd, [0.0, 0.0, 1.0])
        self.right /= np.linalg.norm(self.right)
        self.up = np.cross(self.right, self.fwd)
        self.scale = 0.5 * min(h, w * 0.72) / math.tan(math.radians(FOV) / 2)
        self.cx, self.cy = w / 2, h / 2

    def view(self, pts):
        rel = pts - self.pos
        return rel @ self.right, rel @ self.up, rel @ self.fwd

    def project(self, pts):
        x, y, z = self.view(np.asarray(pts, float))
        zs = np.maximum(z, 1e-3)
        return self.cx + self.scale * x / zs, self.cy - self.scale * y / zs, z

    def segment(self, a, b):
        """Screen end points of a world segment, clipped to the near plane."""
        (xa, xb), (ya, yb), (za, zb) = self.view(np.array([a, b], float))
        if za < NEAR and zb < NEAR:
            return None
        if za < NEAR or zb < NEAR:
            t = (NEAR - za) / (zb - za)
            xc, yc = xa + (xb - xa) * t, ya + (yb - ya) * t
            if za < NEAR:
                xa, ya, za = xc, yc, NEAR
            else:
                xb, yb, zb = xc, yc, NEAR
        s = self.scale
        return (QPointF(self.cx + s * xa / za, self.cy - s * ya / za),
                QPointF(self.cx + s * xb / zb, self.cy - s * yb / zb))


class Pulse:
    """One frame travelling along a wire. Past ``split`` metres it has left the
    CAN bus and is on ECU-01's UART, so it takes the second color."""

    def __init__(self, pts, color, color2=None, split=None, big=False):
        self.pts = np.array(pts, float)
        self.cum = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(self.pts, axis=0), axis=1))])
        self.total = float(self.cum[-1])
        self.color, self.color2 = color, color2 or color
        self.split = self.total if split is None else split
        self.big = big
        self.t0 = time.monotonic()

    def at(self, d):
        return [float(np.interp(d, self.cum, self.pts[:, k])) for k in range(3)]


def path_length(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


class CarView(QWidget):
    """The 3D scene. Left-drag orbits, right-drag pans, the wheel zooms, or,
    over an ECU, changes that node's sensor value."""

    sig_select  = pyqtSignal(int)        # node index, -1 for none
    sig_adjust  = pyqtSignal(int, int)   # node index, steps
    sig_trigger = pyqtSignal(int)        # node index

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(560, 420)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        self.yaw, self.pitch, self.dist = VIEWS["ISO"]
        self.target = np.array([0.0, 0.0, 0.55])
        self._goal = None

        self.values   = [n.default for n in NODES]
        self.hud      = {}          # cmd -> last value the dashboard was sent
        self.linked   = False
        self.led      = False
        self.xray     = True
        self.labels   = True
        self.rotate   = False
        self.selected = -1
        self.last_frame = ""

        self._hover     = -1
        self._press     = None
        self._last      = None
        self._dragged   = False
        self._wheel_acc = 0.0
        self._spin      = 0.0
        self._door      = 0.0
        self._fx        = {}        # effect name -> time it ends
        self._tx_until  = [0.0] * len(NODES)
        self._pulses    = []
        self._ecu_px    = []        # (x, y, radius, depth) of each ECU on screen
        self._t_prev    = time.monotonic()

        self._build_mesh()
        self._build_wires()
        self._build_ground()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._step)
        self._timer.start(FRAME_MS)

    # ── scene ───────────────────────────────────────────────
    def _build_mesh(self):
        m = build_car()
        self._v0    = np.array(m.verts, float)
        self._fidx  = [f[0] for f in m.faces]
        self._fbase = [f[1] for f in m.faces]
        self._fkind = [f[2] for f in m.faces]
        self._ftag  = [f[3] for f in m.faces]
        width = max(len(i) for i in self._fidx)
        self._fpad = np.array([i + (i[-1],) * (width - len(i)) for i in self._fidx])
        self._f0, self._f1, self._f2 = (self._fpad[:, k] for k in range(3))
        self._wheel_spans = [(m.spans[f"wheel{i}"], x) for i, (x, _) in enumerate(WHEELS)]
        self._door_span = m.spans["door"]
        self._light = np.array([0.35, -0.45, 0.82])
        self._light /= np.linalg.norm(self._light)

    def _build_wires(self):
        self._wire_pts   = []
        self._wire_style = []       # (rgb, width in metres, dashed, key)
        self._chunks     = []       # (first point, one past last, style index)

        def add(pts, color, width, dashed=False, key=None):
            pts = np.asarray(pts, float)
            base, style = len(self._wire_pts), len(self._wire_style)
            self._wire_pts.extend(pts)
            self._wire_style.append((rgb(color), width, dashed, key))
            for a in range(0, len(pts) - 1, 5):
                self._chunks.append((base + a, base + min(a + 6, len(pts)), style))

        for path in [TRUNK] + [s for s in STUBS if s]:
            high, low = twisted_pair(path)
            add(high, COL_CAN_H, 0.016)
            add(low, COL_CAN_L, 0.016)
        for node, path in zip(NODES, SENSOR_WIRES):
            add(resample(path, 0.08), node.color, 0.011)
        add(resample(UART, 0.05), C_TEAL, 0.018, dashed=True, key="uart")

        ang = np.linspace(0, 2 * math.pi, 25)           # steering wheel and column
        rim = np.stack([0.42 - 0.05 * np.sin(ang), 0.40 + 0.17 * np.cos(ang),
                        0.95 + 0.17 * np.sin(ang)], axis=1)
        add(rim, "#7C8CA0", 0.022)
        add(resample([(0.42, 0.40, 0.95), (0.64, 0.40, 0.98)], 0.08), "#7C8CA0", 0.022)

        self._wire_pts  = np.array(self._wire_pts)
        self._chunk_mid = np.array([(a + b) // 2 for a, b, _ in self._chunks])

    def _build_ground(self):
        lines = [((x, -2.5, 0.0), (x, 2.5, 0.0)) for x in np.arange(-4.0, 4.01, 0.5)]
        lines += [((-4.0, y, 0.0), (4.0, y, 0.0)) for y in np.arange(-2.5, 2.51, 0.5)]
        self._grid = lines
        ang = np.linspace(0, 2 * math.pi, 28, endpoint=False)
        self._shadow = np.stack([2.45 * np.cos(ang), 1.05 * np.sin(ang),
                                 np.full(ang.shape, 0.002)], axis=1)

    def _world_verts(self):
        v = self._v0.copy()
        c, s = math.cos(self._spin), math.sin(self._spin)
        for (a, b), cx in self._wheel_spans:
            dx, dz = self._v0[a:b, 0] - cx, self._v0[a:b, 2] - WHEEL_R
            v[a:b, 0] = cx + dx * c + dz * s
            v[a:b, 2] = WHEEL_R - dx * s + dz * c
        if self._door > 1e-3:
            a, b = self._door_span
            c, s = math.cos(self._door), math.sin(self._door)
            dx, dy = self._v0[a:b, 0] - DOOR_HINGE[0], self._v0[a:b, 1] - DOOR_HINGE[1]
            v[a:b, 0] = DOOR_HINGE[0] + dx * c - dy * s
            v[a:b, 1] = DOOR_HINGE[1] + dx * s + dy * c
        return v

    # ── inputs from the bus ─────────────────────────────────
    def on_can(self, index, is_event, value, text):
        """A frame from node ``index`` went onto the bus."""
        now = time.monotonic()
        node = NODES[index]
        self.last_frame = text
        if not is_event:
            self.values[index] = value
        if not self.linked:
            return                  # dashboard disconnected: the scene stays still
        self._tx_until[index] = now + TX_FLASH
        if is_event:
            self._fx[f"event{index}"] = now + (3.0 if index == 3 else 2.2)

        if index == 0:
            # ECU-01 is the gateway itself: onto the bus and straight out on UART
            self._pulses.append(Pulse(STUBS[0], node.color, big=is_event))
            self._pulses.append(Pulse(UART, C_TEAL, big=is_event))
            return
        a, b = TAPS[index], TAPS[0]
        trunk = TRUNK[a:b + 1] if a <= b else TRUNK[b:a + 1][::-1]
        path = STUBS[index][:-1] + trunk + STUBS[0][::-1][1:]
        split = path_length(path)
        self._pulses.append(Pulse(path + UART, node.color, C_TEAL, split, big=is_event))

    def on_uart_tx(self, cmd, value):
        self.hud[cmd] = value

    def on_uart_rx(self):
        self._pulses.append(Pulse(UART[::-1], C_AMBER, big=True))

    def set_linked(self, state):
        """Dashboard connected or disconnected; without it every animation stops."""
        self.linked = state
        if not state:
            self.hud.clear()
            self._pulses.clear()
            self._fx.clear()
            self._tx_until = [0.0] * len(NODES)

    def set_led(self, state):
        self.led = state

    def set_selected(self, index):
        self.selected = index
        self.update()

    def set_view(self, name):
        self._goal = VIEWS[name]
        self.target = np.array([0.0, 0.0, 0.55])

    def _active(self, name, now):
        return now < self._fx.get(name, 0.0)

    # ── animation ───────────────────────────────────────────
    def _step(self):
        now = time.monotonic()
        dt, self._t_prev = min(0.1, now - self._t_prev), now
        if self.linked:
            self._spin = (self._spin + self.values[1] / NODES[1].v_max * 12.0 * dt) % (2 * math.pi)
        door_goal = DOOR_OPEN if self._active("event3", now) else 0.0
        self._door += (door_goal - self._door) * min(1.0, dt * 5)
        if self._goal:
            k = min(1.0, dt * 6)
            gy, gp, gd = self._goal
            dyaw = (gy - self.yaw + math.pi) % (2 * math.pi) - math.pi
            self.yaw += dyaw * k
            self.pitch += (gp - self.pitch) * k
            self.dist += (gd - self.dist) * k
            if abs(dyaw) + abs(gp - self.pitch) + abs(gd - self.dist) < 0.01:
                self._goal = None
        elif self.rotate and self.linked and self._press is None:
            self.yaw += 0.25 * dt
        self.update()

    # ── drawing ─────────────────────────────────────────────
    def _tag_colors(self, now):
        """Colors that follow the live state, keyed by face tag: (rgb, alpha or None)."""
        blink = not self.linked or int(now * 8) % 2 == 0
        out = {}
        for i, node in enumerate(NODES):
            base = rgb(node.color)
            if now < self._tx_until[i]:
                out[f"ecu{i}"] = (mix(base, (255, 255, 255), 0.65), None)
            level = self.values[i] / node.v_max
            part = mix(mix(base, (0, 0, 0), 0.75), base, level)
            if self._warning(i) and blink:
                part = WARN_RGB
            out[f"part{i}"] = (part, None)
        if self._active("event0", now):
            out["brake"] = (WARN_RGB if blink else (90, 20, 20), None)
        if self._active("event1", now):
            out["winL"] = (rgb(NODES[1].color), 190)
        if self._active("event2", now):
            out["winR"] = (rgb(NODES[2].color), 190)
        if self._active("event3", now):
            out["door"] = (rgb(NODES[3].color), 210)
        if self._active("event4", now):
            out["head"] = ((255, 246, 190), None)
            out["tail"] = ((255, 45, 45), None)
        if self.led:
            out["led"] = (rgb(C_TEAL), None)
        if self.linked:
            out["screen"] = ((0, 104, 120), None)
        return out

    def _warning(self, index):
        v = self.values[index]
        return ((index == 0 and v >= THRESH_HIGH_PRESS) or
                (index == 2 and v >= THRESH_OVERHEAT) or
                (index == 3 and v <= THRESH_LOW_FLOW))

    def paintEvent(self, _):
        now = time.monotonic()
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        sky = QLinearGradient(0, 0, 0, h)
        sky.setColorAt(0.0, QColor("#0B1420"))
        sky.setColorAt(1.0, QColor(C_BG))
        p.fillRect(self.rect(), sky)

        cam = Camera(self.yaw, self.pitch, self.dist, self.target, w, h)
        self._draw_ground(p, cam)
        self._draw_scene(p, cam, now)
        self._draw_lamps(p, cam, now)
        self._draw_pulses(p, cam, now)
        self._draw_ecu_rings(p, cam)
        self._draw_callouts(p, cam, now)
        self._draw_overlay(p, w, h)

    def _draw_ground(self, p, cam):
        p.setPen(QPen(QColor(36, 51, 72, 150), 1))
        for a, b in self._grid:
            seg = cam.segment(a, b)
            if seg:
                p.drawLine(*seg)
        x, y, z = cam.project(self._shadow)
        if z.min() > NEAR:
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(0, 0, 0, 110))
            p.drawPolygon(QPolygonF([QPointF(a, b) for a, b in zip(x.tolist(), y.tolist())]))

    def _draw_scene(self, p, cam, now):
        """Faces and wires together, far to near."""
        v = self._world_verts()
        sx, sy, sz = cam.project(v)
        fz = sz[self._fpad]
        normal = np.cross(v[self._f1] - v[self._f0], v[self._f2] - v[self._f0])
        length = np.linalg.norm(normal, axis=1)
        shade = 0.5 + 0.5 * np.abs(normal @ self._light) / np.maximum(length, 1e-9)

        wx, wy, wz = cam.project(self._wire_pts)
        depth = np.concatenate([fz.mean(axis=1), wz[self._chunk_mid]])
        order = np.argsort(-depth).tolist()

        X, Y = sx.tolist(), sy.tolist()
        WX, WY, WZ = wx.tolist(), wy.tolist(), wz.tolist()
        hidden = (fz.min(axis=1) < NEAR).tolist()
        shade = shade.tolist()
        tagged = self._tag_colors(now)
        if self.xray:
            styles = {
                "body":  (46,  QPen(QColor(140, 186, 226, 100), 1)),
                "glass": (30,  QPen(QColor(150, 220, 240, 90), 1)),
                "floor": (150, QPen(QColor(90, 120, 150, 80), 1)),
                "ghost": (70,  QPen(QColor(100, 120, 144, 80), 1)),
                "solid": (255, QPen(QColor(6, 10, 16, 170), 1)),
            }
        else:
            styles = {
                "body":  (242, QPen(QColor(20, 34, 52, 200), 1)),
                "glass": (120, QPen(QColor(150, 220, 240, 120), 1)),
                "floor": (255, QPen(QColor(20, 34, 52, 200), 1)),
                "ghost": (255, QPen(QColor(20, 26, 34, 200), 1)),
                "solid": (255, QPen(QColor(6, 10, 16, 170), 1)),
            }
        nf = len(self._fidx)
        uart_rgb = rgb(C_TEAL if self.linked else C_MUTED)

        for k in order:
            if k < nf:
                if hidden[k]:
                    continue
                alpha, pen = styles[self._fkind[k]]
                color = self._fbase[k]
                live = tagged.get(self._ftag[k])
                if live:
                    color = live[0]
                    alpha = live[1] or alpha
                s = shade[k]
                p.setPen(pen)
                p.setBrush(QColor(int(color[0] * s), int(color[1] * s), int(color[2] * s), alpha))
                p.drawPolygon(QPolygonF([QPointF(X[i], Y[i]) for i in self._fidx[k]]))
            else:
                a, b, style = self._chunks[k - nf]
                if min(WZ[a:b]) < NEAR:
                    continue
                color, width, dashed, key = self._wire_style[style]
                if key == "uart":
                    color = uart_rgb
                pen = QPen(QColor(*color), max(1.2, width * cam.scale / WZ[(a + b) // 2]))
                pen.setCapStyle(Qt.RoundCap)
                if dashed:
                    pen.setStyle(Qt.DashLine)
                p.setPen(pen)
                p.drawPolyline(QPolygonF([QPointF(WX[i], WY[i]) for i in range(a, b)]))

    def _glow(self, p, x, y, radius, color, core=True):
        g = QRadialGradient(QPointF(x, y), radius)
        c = QColor(color)
        if core:
            g.setColorAt(0.0, QColor(255, 255, 255, 240))
            c.setAlpha(210)
            g.setColorAt(0.3, c)
        else:
            c.setAlpha(170)
            g.setColorAt(0.0, c)
        edge = QColor(color)
        edge.setAlpha(0)
        g.setColorAt(1.0, edge)
        p.setPen(Qt.NoPen)
        p.setBrush(g)
        p.drawEllipse(QPointF(x, y), radius, radius)

    def _draw_lamps(self, p, cam, now):
        if not self._active("event4", now):
            return
        lamps = [((2.24, y, 0.56), "#FFF6BE") for y in (0.5, -0.5)]
        lamps += [((-2.25, y, 0.64), "#FF2D2D") for y in (0.5, -0.5)]
        x, y, z = cam.project(np.array([pos for pos, _ in lamps]))
        for i, (_, color) in enumerate(lamps):
            if z[i] > NEAR:
                self._glow(p, x[i], y[i], 0.34 * cam.scale / z[i], color, core=False)

    def _draw_pulses(self, p, cam, now):
        dots = []       # (position, color, relative size)
        alive = []
        for pulse in self._pulses:
            d = (now - pulse.t0) * PULSE_SPEED
            if d > pulse.total + 0.3:
                continue
            alive.append(pulse)
            for k in range(4):                          # head and a fading tail
                dk = d - k * 0.07
                if 0.0 <= dk <= pulse.total:
                    color = pulse.color if dk <= pulse.split else pulse.color2
                    size = (0.085 if pulse.big else 0.05) * (1.0 - 0.2 * k)
                    dots.append((pulse.at(dk), color, size))
        self._pulses = alive
        if not dots:
            return
        x, y, z = cam.project(np.array([d[0] for d in dots]))
        for i, (_, color, size) in enumerate(dots):
            if z[i] > NEAR:
                self._glow(p, x[i], y[i], max(3.0, size * cam.scale / z[i]), color)

    def _draw_ecu_rings(self, p, cam):
        x, y, z = cam.project(np.array(ECU_POS))
        self._ecu_px = [(x[i], y[i], max(14.0, 0.21 * cam.scale / max(z[i], NEAR)), z[i])
                        for i in range(len(NODES))]
        p.setBrush(Qt.NoBrush)
        for i, (px, py, radius, depth) in enumerate(self._ecu_px):
            if depth < NEAR or i not in (self.selected, self._hover):
                continue
            pen = QPen(QColor(NODES[i].color), 2.5 if i == self.selected else 1.5)
            if i != self.selected:
                pen.setStyle(Qt.DotLine)
            p.setPen(pen)
            p.drawEllipse(QPointF(px, py), radius, radius)

    @staticmethod
    def _text_box(lines, border, strong=False, plain=False):
        """Size and painter of a small label; ``lines`` is [(text, (font, metrics), color)]."""
        pad = 3 if plain else 7
        metrics = [fm for _, (_, fm), _ in lines]
        lines = [(text, font, color) for text, (font, _), color in lines]
        w = max(fm.horizontalAdvance(text) for fm, (text, _, _) in zip(metrics, lines)) + 2 * pad + 4
        h = sum(fm.height() for fm in metrics) + 2 * pad

        def draw(p, rect):
            bg = QColor(C_PANEL)
            bg.setAlpha(150 if plain else 228)
            edge = QColor(border)
            if plain:
                edge.setAlpha(110)
            p.setPen(QPen(edge, 2 if strong else 1))
            p.setBrush(bg)
            p.drawRoundedRect(rect, 5, 5)
            y = rect.top() + pad
            for fm, (text, font, color) in zip(metrics, lines):
                p.setFont(font)
                p.setPen(QColor(color))
                p.drawText(QRectF(rect.left(), y, rect.width(), fm.height()), Qt.AlignCenter, text)
                y += fm.height()
        return w, h, draw

    def _hud_box(self):
        """What the dashboard has been sent over UART, shown at the cluster."""
        w, h = 214, 104

        def draw(p, rect):
            edge = QColor(C_TEAL if self.linked else C_MUTED)
            bg = QColor(C_PANEL)
            bg.setAlpha(235)
            p.setPen(QPen(edge, 1.5))
            p.setBrush(bg)
            p.drawRoundedRect(rect, 6, 6)
            p.setFont(px_font(10, True))
            p.setPen(edge)
            p.drawText(QRectF(rect.left(), rect.top() + 5, w, 13), Qt.AlignCenter,
                       "PC DASHBOARD  ·  UART 115200")
            p.setFont(px_font(9))
            p.setPen(QColor(C_TEXT2))
            state = "ST-Link VCP  ·  CONNECTED" if self.linked else "NOT CONNECTED"
            p.drawText(QRectF(rect.left(), rect.top() + 19, w, 12), Qt.AlignCenter, state)
            col = (w - 16) / len(NODES)
            for i, node in enumerate(NODES):
                x = rect.left() + 8 + i * col
                value = self.hud.get(node.cmd)
                p.setPen(QColor(node.color if value is not None else C_MUTED))
                p.drawText(QRectF(x, rect.top() + 34, col, 12), Qt.AlignCenter,
                           "--" if value is None else str(value))
                bar = QRectF(x + 6, rect.top() + 48, col - 12, 34)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor(C_CARD2))
                p.drawRoundedRect(bar, 2, 2)
                if value is not None:
                    fill = bar.height() * min(1.0, value / node.v_max)
                    p.setBrush(QColor(node.color))
                    p.drawRoundedRect(QRectF(bar.left(), bar.bottom() - fill, bar.width(), fill), 2, 2)
                p.setPen(QColor(C_TEXT2))
                p.drawText(QRectF(x, rect.top() + 86, col, 12), Qt.AlignCenter, node.unit)
        return w, h, draw

    def _draw_callouts(self, p, cam, now):
        anchors = []    # (world position, lift in px, (w, h, draw), leader color)
        blink = int(now * 6) % 2 == 0

        for i, node in enumerate(NODES):
            warn = self._warning(i)
            lines = [(f"{node.ecu}  ·  {node.sensor.upper()}", label_font(10, True), node.color),
                     (f"{self.values[i]} {node.unit}", label_font(16, True),
                      C_RED if warn else C_TEXT)]
            if self._active(f"event{i}", now):
                lines.append((f"●  {node.ev_name.upper()}", label_font(11, True),
                              node.color if blink else C_TEXT))
            elif i == self._hover:
                lines.append((f"scroll: value   dbl-click: {node.ev_name}", label_font(9), C_TEXT2))
            x, y, z = ECU_POS[i]
            box = self._text_box(lines, node.color, strong=i in (self.selected, self._hover))
            anchors.append(((x, y, z + 0.07), 38, box, node.color))

        anchors.append((CLUSTER, 60, self._hud_box(), C_TEAL if self.linked else C_MUTED))

        if self.labels:
            small = [((c[0], c[1], c[2] + s[2] / 2), name) for name, c, s in PARTS]
            small += [(pos, "120 Ω") for pos in TERMINATORS]
            small += [((0.25, 0.0, 0.27), "CAN_H / CAN_L  ·  twisted pair"),
                      (UART[2], "UART  ·  ST-Link VCP")]
            for pos, text in small:
                box = self._text_box([(text, label_font(9), C_TEXT2)], C_BORDER2, plain=True)
                anchors.append((pos, 16, box, C_BORDER2))

        x, y, z = cam.project(np.array([a[0] for a in anchors]))
        w = self.width()
        placed = []
        # nearest first, so the labels in front keep the spot next to their anchor
        for i in sorted(range(len(anchors)), key=lambda k: z[k]):
            if z[i] < NEAR:
                continue
            _, lift, (bw, bh, draw), color = anchors[i]
            rect = QRectF(x[i] - bw / 2, y[i] - lift - bh, bw, bh)
            rect.moveLeft(min(max(rect.left(), 4.0), max(4.0, w - bw - 4.0)))
            for _ in range(70):
                grown = rect.adjusted(-3, -3, 3, 3)
                if not any(grown.intersects(other) for other, *_ in placed):
                    break
                rect.translate(0, -6)
            placed.append((rect, QPointF(x[i], y[i]), draw, color))

        for rect, anchor, _, color in placed:
            c = QColor(color)
            c.setAlpha(170)
            p.setPen(QPen(c, 1))
            p.drawLine(anchor, QPointF(rect.center().x(), rect.bottom()))
            p.setBrush(c)
            p.drawEllipse(anchor, 2.5, 2.5)
        for rect, _, draw, _ in reversed(placed):
            draw(p, rect)

    def _draw_overlay(self, p, w, h):
        p.setFont(px_font(13, True))
        p.setPen(QColor(C_AMBER))
        p.drawText(QRectF(14, 10, w, 18), Qt.AlignLeft, "VIRTUAL  CAR   ·   CAN  NETWORK  IN  3D")
        p.setFont(px_font(11))
        p.setPen(QColor(C_TEXT))
        p.drawText(QRectF(14, 30, w, 16), Qt.AlignLeft, self.last_frame)
        if not self.linked:
            p.setFont(px_font(12, True))
            p.setPen(QColor(C_RED))
            p.drawText(QRectF(0, 54, w, 18), Qt.AlignHCenter,
                       "DASHBOARD  DISCONNECTED   ·   ANIMATION  PAUSED")

        p.setFont(px_font(10))
        p.setPen(QColor(C_TEXT2))
        p.drawText(QRectF(14, h - 22, w, 14), Qt.AlignLeft,
                   "drag: orbit    right-drag: pan    wheel: zoom    "
                   "wheel over an ECU: change its value    double-click an ECU: press its button")

        legend = [(COL_CAN_H, "CAN_H", False), (COL_CAN_L, "CAN_L", False),
                  (C_TEAL, "UART to dashboard", True), (C_TEXT, "sensor wire", False)]
        y = h - 46 - 16 * len(legend)
        for color, text, dashed in legend:
            pen = QPen(QColor(color), 3)
            if dashed:
                pen.setStyle(Qt.DashLine)
            p.setPen(pen)
            p.drawLine(QPointF(w - 170, y + 7), QPointF(w - 140, y + 7))
            p.setPen(QColor(C_TEXT2))
            p.drawText(QRectF(w - 132, y, 130, 14), Qt.AlignLeft, text)
            y += 16

    # ── mouse and keyboard ──────────────────────────────────
    def _pick(self, pos):
        best, best_d = -1, 1e9
        for i, (x, y, radius, depth) in enumerate(self._ecu_px):
            d = math.hypot(pos.x() - x, pos.y() - y)
            if depth > NEAR and d <= radius and d < best_d:
                best, best_d = i, d
        return best

    def mousePressEvent(self, e):
        self._press = self._last = e.pos()
        self._dragged = False

    def mouseMoveEvent(self, e):
        if self._press is None:
            hover = self._pick(e.pos())
            if hover != self._hover:
                self._hover = hover
                self.setCursor(Qt.PointingHandCursor if hover >= 0 else Qt.ArrowCursor)
            return
        d = e.pos() - self._last
        self._last = e.pos()
        if (e.pos() - self._press).manhattanLength() > 4:
            self._dragged = True
        if not self._dragged:
            return
        self._goal = None
        if e.buttons() & Qt.LeftButton and not e.modifiers() & Qt.ShiftModifier:
            self.yaw -= d.x() * 0.008
            self.pitch = min(1.5, max(0.03, self.pitch + d.y() * 0.006))
        else:
            cam = Camera(self.yaw, self.pitch, self.dist, self.target, self.width(), self.height())
            k = self.dist / cam.scale
            self.target = np.clip(self.target - cam.right * d.x() * k + cam.up * d.y() * k,
                                  [-2.5, -1.5, 0.0], [2.5, 1.5, 1.6])

    def mouseReleaseEvent(self, e):
        if not self._dragged and e.button() == Qt.LeftButton:
            self.sig_select.emit(self._pick(e.pos()))
        self._press = None

    def mouseDoubleClickEvent(self, e):
        index = self._pick(e.pos())
        if index >= 0:
            self.sig_trigger.emit(index)

    def wheelEvent(self, e):
        self._wheel_acc += e.angleDelta().y() / 120.0
        if self._hover < 0:
            self._goal = None
            self.dist = min(16.0, max(2.2, self.dist * 0.9 ** self._wheel_acc))
            self._wheel_acc = 0.0
            return
        steps = int(self._wheel_acc)
        if steps:
            self._wheel_acc -= steps
            self.sig_adjust.emit(self._hover, steps)

    def keyPressEvent(self, e):
        steps = {Qt.Key_Up: 1, Qt.Key_Right: 1, Qt.Key_Down: -1, Qt.Key_Left: -1,
                 Qt.Key_PageUp: 5, Qt.Key_PageDown: -5}.get(e.key())
        if self.selected < 0:
            super().keyPressEvent(e)
        elif steps:
            self.sig_adjust.emit(self.selected, steps)
        elif e.key() == Qt.Key_Space:
            self.sig_trigger.emit(self.selected)
        else:
            super().keyPressEvent(e)


class EcuControl(QFrame):
    """One ECU's inputs: the sensor value at its location and its push button."""

    sig_clicked = pyqtSignal(int)

    def __init__(self, index, node, bus, parent=None):
        super().__init__(parent)
        self.index = index
        self.node  = node
        self._bus  = bus
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 8, 10, 9)
        lay.setSpacing(4)

        head = QHBoxLayout()
        name = QLabel(f"{node.ecu}  ·  {node.owner.upper()}")
        name.setStyleSheet(f"color:{node.color}; font-size:11px; font-weight:bold; letter-spacing:1px;")
        self._lbl_value = QLabel()
        self._lbl_value.setStyleSheet(f"color:{node.color}; font-size:15px; font-weight:bold;")
        head.addWidget(name)
        head.addStretch()
        head.addWidget(self._lbl_value)
        lay.addLayout(head)

        lay.addWidget(self._small(f"{node.sensor}  ·  CAN 0x{node.can_id:03X}  →  UART 0x{node.cmd:02X}"))

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, node.v_max)
        self._slider.setSingleStep(max(1, node.v_max // 50))
        self._slider.setPageStep(max(1, node.v_max // 10))
        self._slider.setValue(node.default)
        self._slider.valueChanged.connect(self._on_slider)
        lay.addWidget(self._slider)

        row = QHBoxLayout()
        btn = QPushButton(f"●  {node.ev_name.upper()}")
        btn.setFixedHeight(28)
        btn.setStyleSheet(
            f"QPushButton {{ background:{C_CARD}; border:1px solid {tint(node.color, '88')}; "
            f"border-radius:6px; color:{node.color}; font-weight:bold; font-size:9px; "
            f"padding:0 6px; min-height:0; }}"
            f"QPushButton:hover {{ background:{tint(node.color, '22')}; border-color:{node.color}; }}"
            f"QPushButton:pressed {{ background:{node.color}; color:#000; }}"
        )
        btn.clicked.connect(lambda: bus.trigger(node.ev_cmd))
        self._chk_auto = QCheckBox("Auto")
        self._chk_auto.toggled.connect(lambda s: bus.set_auto(node.cmd, s))
        row.addWidget(btn, 1)
        row.addWidget(self._chk_auto)
        lay.addLayout(row)

        self.set_selected(False)
        self._show(node.default)

    @staticmethod
    def _small(text):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"color:{C_TEXT2}; font-size:9px;")
        return lbl

    def _show(self, value):
        self._lbl_value.setText(f"{value} {self.node.unit}")

    def _on_slider(self, value):
        """Moved by hand: take the sensor out of auto sweep and apply the value."""
        self._chk_auto.setChecked(False)
        self._bus.set_value(self.node.cmd, value)
        self._show(value)
        self.sig_clicked.emit(self.index)

    def nudge(self, steps):
        self._slider.setValue(self._slider.value() + steps * self._slider.singleStep())

    def set_auto(self, state):
        self._chk_auto.setChecked(state)

    def set_selected(self, state):
        edge = self.node.color if state else C_BORDER2
        self.setStyleSheet(
            f"EcuControl {{ background:{C_CARD if state else C_PANEL}; border:1px solid {edge}; "
            f"border-left:3px solid {self.node.color}; border-radius:8px; }}"
        )

    def on_sensor_frame(self, value):
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)
        self._show(value)

    def mousePressEvent(self, e):
        self.sig_clicked.emit(self.index)
        super().mousePressEvent(e)


class Car3DWindow(QMainWindow):
    sig_bus = pyqtSignal(str, object, object, object)

    def __init__(self, bus):
        super().__init__()
        self.setWindowTitle("Virtual Car 3D  •  ECUs, CAN harness and UART link")
        self.resize(1320, 800)
        self.setStyleSheet(BENCH_QSS)
        self._bus = bus
        self._by_can = {}

        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QHBoxLayout(root_w)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        self._view = CarView()
        self._view.sig_select.connect(self._select)
        self._view.sig_adjust.connect(lambda i, steps: self._cards[i].nudge(steps))
        self._view.sig_trigger.connect(lambda i: bus.trigger(NODES[i].ev_cmd))
        root.addWidget(self._view, stretch=1)

        side = QWidget()
        side.setFixedWidth(300)
        sv = QVBoxLayout(side)
        sv.setContentsMargins(0, 0, 0, 0)
        sv.setSpacing(8)

        views = QHBoxLayout()
        views.setSpacing(5)
        for name in VIEWS:
            btn = QPushButton(name)
            btn.setFixedHeight(28)
            btn.clicked.connect(lambda _, n=name: self._view.set_view(n))
            views.addWidget(btn)
        sv.addLayout(views)

        opts = QHBoxLayout()
        for text, attr, on in [("X-ray body", "xray", True), ("Labels", "labels", True),
                               ("Rotate", "rotate", False)]:
            chk = QCheckBox(text)
            chk.setChecked(on)
            chk.toggled.connect(lambda s, a=attr: setattr(self._view, a, s))
            opts.addWidget(chk)
        opts.addStretch()
        sv.addLayout(opts)

        cards_w = QWidget()
        cv = QVBoxLayout(cards_w)
        cv.setContentsMargins(0, 0, 4, 0)
        cv.setSpacing(8)
        self._cards = []
        for i, node in enumerate(NODES):
            card = EcuControl(i, node, bus)
            card.sig_clicked.connect(self._select)
            cv.addWidget(card)
            self._cards.append(card)
            self._by_can[node.can_id]    = (i, node, False)
            self._by_can[node.ev_can_id] = (i, node, True)
        self._chk_all = QCheckBox("Auto sweep all sensors")
        self._chk_all.toggled.connect(lambda s: [c.set_auto(s) for c in self._cards])
        cv.addWidget(self._chk_all)
        cv.addStretch()

        self._scroll = QScrollArea()
        self._scroll.setWidget(cards_w)
        self._scroll.setWidgetResizable(True)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._scroll.setFrameShape(QFrame.NoFrame)
        sv.addWidget(self._scroll, stretch=1)
        root.addWidget(side)

        # Bus callbacks arrive on worker threads; the signal hops them to the GUI thread
        self.sig_bus.connect(self._on_bus)
        bus.listen(self.sig_bus.emit)

    def _select(self, index):
        self._view.set_selected(index)
        for card in self._cards:
            card.set_selected(card.index == index)
        if index >= 0:
            self._scroll.ensureWidgetVisible(self._cards[index])

    def _on_bus(self, kind, a, b, c):
        if kind == "can":
            index, node, is_event = self._by_can[a]
            if is_event:
                data, what = "01", f"{node.ev_name} button"
            else:
                self._cards[index].on_sensor_frame(c)
                data = " ".join(f"{x:02X}" for x in c.to_bytes(node.dlc, "big"))
                what = f"{node.sensor} = {c} {node.unit}"
            self._view.on_can(index, is_event, c,
                              f"0x{a:03X}   [{data}]   {node.ecu} · {what}")
        elif kind == "uart_tx":
            self._view.on_uart_tx(a, b)
        elif kind == "uart_rx":
            self._view.on_uart_rx()
        elif kind == "led":
            self._view.set_led(bool(a))
        elif kind == "link":
            self._view.set_linked(bool(a))


def open_car3d(bus=None):
    """Show the 3D car on ``bus``, or on a virtual bus of its own."""
    own = bus is None
    if own:
        bus = VirtualBus(auto=False)
    win = Car3DWindow(bus)
    if own:
        bus.start()
    win.show()
    return win


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    car = open_car3d()
    print(f"Virtual car running, connect the dashboard to {URL}")
    sys.exit(app.exec_())
