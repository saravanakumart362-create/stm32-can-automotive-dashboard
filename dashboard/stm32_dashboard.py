"""
STM32 Automotive Dashboard — OpCode Labs

Dashboard PyQt5 de supervision d'un réseau CAN à 5 nœuds, relié au
firmware STM32 via une liaison UART (protocole SOF + CMD + PAYLOAD,
voir main.c).

Architecture par nœud :
  Amine   -> Pressure        (CAN 0x55C / CMD 0x20)  + ABS            (CAN 0x679 / CMD 0x50)
  Yassine -> Speed Motor     (CAN 0x406 / CMD 0x21)  + Window Left    (CAN 0x401 / CMD 0x31)
  Souha   -> Temperature     (CAN 0x456 / CMD 0x22)  + Window Right   (CAN 0x402 / CMD 0x32)
  Eya     -> Débit Fluide    (CAN 0x390 / CMD 0x23)  + Door Right     (CAN 0x790 / CMD 0x33)
  Nour    -> Battery Voltage (CAN 0x222 / CMD 0x24)  + External Light (CAN 0x221 / CMD 0x25)

ABS est déclenché par un bouton sur le dashboard (Amine) ; les autres
événements par nœud (Window Left/Right, Door Right, External Light)
sont déclenchés par un bouton physique sur leur propre carte et
n'apparaissent que comme notifications passives côté dashboard.
"""

import sys, struct, serial, serial.tools.list_ports, math
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox,
    QGroupBox, QTextEdit, QFrame,
    QStatusBar, QCheckBox, QSizePolicy,
    QScrollArea
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QRectF, QPointF
from PyQt5.QtGui import (
    QFont, QColor, QPainter, QPen, QBrush, QRadialGradient
)

# ═══════════════════════════════════════════
#  PROTOCOLE (conforme à main.c)
# ═══════════════════════════════════════════
SOF       = 0xA5
FRAME_LEN = 6

CMD_LED_ON            = 0x01
CMD_LED_OFF           = 0x02
CMD_READ_SENSOR       = 0x10
CMD_TRIGGER_ABS       = 0x11   # bouton PC (Amine) -> déclenche l'ABS via CAN
CMD_PRESSURE          = 0x20
CMD_SPEED             = 0x21
CMD_TEMPERATURE       = 0x22
CMD_DEBIT             = 0x23
CMD_BATTERY_VOLTAGE   = 0x24
CMD_EXTERNAL_LIGHT    = 0x25
CMD_WINDOW_LEFT       = 0x31
CMD_WINDOW_RIGHT      = 0x32
CMD_PORTE_DROIT       = 0x33
CMD_ACK               = 0x50   # LED ack (payload=0) OU événement ABS (payload[0]=1)

RX_NAMES = {
    CMD_PRESSURE        : "Pressure",
    CMD_SPEED           : "Speed Motor",
    CMD_TEMPERATURE     : "Temperature",
    CMD_DEBIT           : "Débit Fluide",
    CMD_BATTERY_VOLTAGE : "Battery Voltage",
    CMD_EXTERNAL_LIGHT  : "External Light",
    CMD_WINDOW_LEFT     : "Window Left",
    CMD_WINDOW_RIGHT    : "Window Right",
    CMD_PORTE_DROIT     : "Porte Droit",
    CMD_ACK             : "ACK / ABS",
}

THRESH_OVERHEAT   = 110.0
THRESH_HIGH_PRESS = 200.0
THRESH_LOW_FLOW   = 20.0

# ═══════════════════════════════════════════
#  PALETTE
# ═══════════════════════════════════════════
C_BG       = "#05080D"
C_PANEL    = "#0A0F18"
C_CARD     = "#0F1620"
C_CARD2    = "#161F2E"
C_BORDER2  = "#243348"
C_AMBER    = "#F0A500"
C_AMBER_D  = "#5A3800"
C_AMBER_L  = "#FFD060"
C_GREEN    = "#00E676"
C_RED      = "#FF3D3D"
C_TEAL     = "#00E5FF"
C_TEXT     = "#D0DCE8"
C_TEXT2    = "#8FAABE"
C_MUTED    = "#2E3F52"
C_DIAL_BG  = "#060A10"
C_NEEDLE   = "#FF1744"
C_TICK     = "#3A4A5E"
C_TICK_LIT = "#C7D3DE"

COL_PRESSURE = "#F0A500"
COL_SPEED    = "#00E5FF"
COL_TEMP     = "#CE93D8"
COL_DEBIT    = "#2979FF"
COL_BATTERY  = "#FFCA28"

COL_ABS      = "#FF3D3D"
COL_WIN_L    = "#00E5FF"
COL_WIN_R    = "#CE93D8"
COL_DOOR     = "#2979FF"
COL_EXT_LGT  = "#EC407A"

COL_OVHT     = "#FF7043"
COL_PRES_W   = "#F0A500"
COL_FLOW_W   = "#2979FF"

QSS = f"""
* {{
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
    color: {C_TEXT};
}}
QMainWindow, QWidget {{ background-color: {C_BG}; }}
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{ background: {C_PANEL}; width: 6px; border-radius: 3px; }}
QScrollBar::handle:vertical {{ background: {C_BORDER2}; border-radius: 3px; min-height: 20px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QGroupBox {{
    background-color: {C_PANEL}; border: 1px solid {C_BORDER2};
    border-radius: 10px; margin-top: 22px; padding-top: 16px;
}}
QGroupBox::title {{
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 12px; top: 2px; padding: 2px 8px;
    background-color: {C_BG}; border-radius: 4px;
    font-size: 9px; font-weight: bold; letter-spacing: 3px;
    text-transform: uppercase;
}}
QPushButton {{
    background-color: {C_CARD}; border: 1px solid {C_BORDER2};
    border-radius: 6px; padding: 6px 10px; min-height: 28px;
    font-weight: bold; font-size: 10px; color: {C_TEXT};
}}
QPushButton:hover   {{ background-color: {C_AMBER_D}; border-color: {C_AMBER}; color: {C_AMBER_L}; }}
QPushButton:pressed {{ background-color: {C_AMBER};   color: #111; }}
QPushButton:disabled {{ background-color: {C_PANEL};  color: {C_MUTED}; border-color: {C_BORDER2}; }}
QPushButton#btn_connect    {{ border-color:#00C853; color:#00E676; background:#021A0A; font-size:11px; }}
QPushButton#btn_connect:hover {{ background:#00C853; color:#000; }}
QPushButton#btn_disconnect {{ border-color:{C_RED}; color:{C_RED}; background:#1A0206; font-size:11px; }}
QPushButton#btn_disconnect:hover {{ background:{C_RED}; color:#fff; }}
QComboBox {{
    background-color: {C_CARD}; border: 1px solid {C_BORDER2};
    border-radius: 5px; padding: 4px 8px; min-height: 26px;
    color: {C_TEXT}; font-size: 11px;
}}
QComboBox::drop-down {{ border: none; width: 20px; }}
QComboBox QAbstractItemView {{
    background: {C_CARD2}; border: 1px solid {C_AMBER};
    selection-background-color: {C_AMBER_D}; color: {C_TEXT};
}}
QTextEdit {{
    background-color: #020508; border: 1px solid {C_BORDER2};
    border-radius: 6px; color: #4CAF50; font-size: 12px;
    padding: 6px; line-height: 1.9;
}}
QCheckBox {{ spacing: 6px; color: {C_TEXT2}; font-size: 10px; }}
QCheckBox::indicator {{
    width: 12px; height: 12px; border: 1px solid {C_BORDER2};
    border-radius: 3px; background: {C_CARD};
}}
QCheckBox::indicator:checked {{ background: {C_AMBER}; border-color: {C_AMBER}; }}
QStatusBar {{
    background: {C_PANEL}; border-top: 1px solid {C_BORDER2};
    color: {C_TEXT2}; font-size: 10px; padding: 2px 10px; letter-spacing: 1px;
}}
QLabel {{ background: transparent; }}
"""


def section_label(text, color):
    lbl = QLabel(f"  {text}")
    lbl.setStyleSheet(
        f"color: {color}; font-size: 8px; font-weight: bold; "
        f"letter-spacing: 3px; background: {color}18; "
        f"border-left: 2px solid {color}; border-radius: 2px; "
        f"padding: 3px 8px; margin-bottom: 2px;"
    )
    lbl.setFixedHeight(20)
    return lbl


class SerialReader(QThread):
    sig_frame = pyqtSignal(int, float)
    sig_raw   = pyqtSignal(bytes)
    sig_error = pyqtSignal(str)

    def __init__(self, port, parent=None):
        super().__init__(parent)
        self._port   = port
        self._active = True
        self._buf    = bytearray()

    def stop(self):
        self._active = False

    def run(self):
        while self._active:
            try:
                waiting = self._port.in_waiting
                if waiting > 0:
                    self._buf.extend(self._port.read(waiting))
                    self._parse()
                else:
                    self.msleep(5)
            except serial.SerialException as exc:
                self.sig_error.emit(str(exc))
                break

    def _parse(self):
        while len(self._buf) >= FRAME_LEN:
            idx = self._buf.find(SOF)
            if idx == -1:
                self._buf.clear(); return
            if idx > 0:
                del self._buf[:idx]
            if len(self._buf) < FRAME_LEN:
                return
            frame = bytes(self._buf[:FRAME_LEN])
            del self._buf[:FRAME_LEN]
            cmd   = frame[1]
            value = struct.unpack('>I', frame[2:6])[0]
            self.sig_raw.emit(frame)
            self.sig_frame.emit(cmd, float(value))


class ClassicGauge(QWidget):
    def __init__(self, label="", unit="", v_min=0, v_max=255,
                 color="#F0A500", node_name="", parent=None):
        super().__init__(parent)
        self.label     = label
        self.unit      = unit
        self.v_min     = v_min
        self.v_max     = v_max
        self.color     = QColor(color)
        self.node_name = node_name
        self._value    = 0.0
        self._anim_val = 0.0
        self._online   = False
        self._last_rx  = None
        self._stale    = False

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._animate)
        self._timer.start(16)

        self._stale_timer = QTimer(self)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start(1000)

        self.setMinimumSize(190, 210)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v):
        self._value   = max(self.v_min, min(self.v_max, v))
        self._online  = True
        self._last_rx = datetime.now().timestamp()
        self._stale   = False

    def _check_stale(self):
        if self._online and self._last_rx is not None:
            elapsed = datetime.now().timestamp() - self._last_rx
            self._stale = elapsed > 5.0
            self.update()

    def _animate(self):
        diff = self._value - self._anim_val
        if abs(diff) > 0.15:
            self._anim_val += diff * 0.16
            self.update()
        elif self._anim_val != self._value:
            self._anim_val = self._value
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        p.setOpacity(0.4 if self._stale else 1.0)

        W, H = self.width(), self.height()
        size = min(W, H - 46)
        cx, cy = W / 2, (H - 46) / 2 + 8
        r = size / 2 - 8

        dial_grad = QRadialGradient(cx, cy - r * 0.15, r * 1.1)
        dial_grad.setColorAt(0.0, QColor("#12181F"))
        dial_grad.setColorAt(1.0, QColor(C_DIAL_BG))
        p.setBrush(QBrush(dial_grad))
        p.setPen(QPen(QColor(C_BORDER2), 1.4))
        p.drawEllipse(QPointF(cx, cy), r, r)

        start_angle, sweep_angle = 225, 270
        n_minor = 25
        for i in range(n_minor + 1):
            frac = i / n_minor
            angle = math.radians(start_angle - frac * sweep_angle)
            is_major = (i % 5 == 0)
            r_out = r * 0.90
            r_in  = r * (0.74 if is_major else 0.82)
            width = 2.0 if is_major else 1.0
            col = QColor(C_TICK_LIT) if is_major else QColor(C_TICK)
            p.setPen(QPen(col, width))
            p.drawLine(
                QPointF(cx + r_out*math.cos(angle), cy - r_out*math.sin(angle)),
                QPointF(cx + r_in *math.cos(angle), cy - r_in *math.sin(angle))
            )
            if is_major:
                val_at = self.v_min + frac * (self.v_max - self.v_min)
                val_txt  = f"{val_at:.0f}"
                n_digits = len(val_txt)
                lbl_r    = r * 0.56
                box_w    = max(44, n_digits * 13)
                box_h    = 18
                lx = cx + lbl_r*math.cos(angle) - box_w/2
                ly = cy - lbl_r*math.sin(angle) - box_h/2
                font_sz = max(7, int(r*0.10)) if n_digits <= 3 else max(6, int(r*0.085))
                p.setFont(QFont("Consolas", font_sz, QFont.Bold))
                p.setPen(QColor(C_TICK_LIT))
                p.drawText(QRectF(lx, ly, box_w, box_h), Qt.AlignCenter, val_txt)

        pct = (self._anim_val - self.v_min) / max(self.v_max - self.v_min, 1e-9)
        pct = max(0.0, min(1.0, pct))
        needle_angle = math.radians(start_angle - pct * sweep_angle)
        nx = cx + r*0.68*math.cos(needle_angle)
        ny = cy - r*0.68*math.sin(needle_angle)
        bx = cx + r*0.12*math.cos(needle_angle + math.pi)
        by = cy - r*0.12*math.sin(needle_angle + math.pi)

        p.setPen(QPen(QColor(0, 0, 0, 110), 4, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(bx+1.2, by+1.2), QPointF(nx+1.2, ny+1.2))
        p.setPen(QPen(QColor(C_NEEDLE), 2.4, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(bx, by), QPointF(nx, ny))

        hub_g = QRadialGradient(cx, cy, r*0.09)
        hub_g.setColorAt(0.0, QColor("#9AAFC4"))
        hub_g.setColorAt(1.0, QColor("#28313F"))
        p.setBrush(QBrush(hub_g))
        p.setPen(QPen(QColor(C_BORDER2), 1))
        p.drawEllipse(QPointF(cx, cy), r*0.07, r*0.07)

        val_str = f"{self._anim_val:.0f}"
        val_rect = QRectF(cx - r*0.5, cy + r*0.14, r, r*0.36)
        p.setFont(QFont("Consolas", max(16, int(r*0.24)), QFont.Bold))
        p.setPen(QColor("#F2F6FA"))
        p.drawText(val_rect, Qt.AlignCenter, val_str)

        p.setFont(QFont("Consolas", max(8, int(r*0.10))))
        p.setPen(QColor(C_TEXT2))
        p.drawText(QRectF(cx - r, cy + r*0.48, r*2, r*0.22), Qt.AlignCenter, self.unit)

        p.setFont(QFont("Consolas", max(10, int(r*0.13)), QFont.Bold))
        p.setPen(self.color if (self._online and not self._stale) else QColor(C_MUTED))
        p.drawText(QRectF(0, H-28, W, 22), Qt.AlignCenter, self.label.upper())

        dot_c = QColor(C_GREEN) if (self._online and not self._stale) else QColor(C_MUTED)
        p.setBrush(dot_c); p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(W-14, 12), 4.5, 4.5)


class WarningIndicator(QFrame):
    def __init__(self, label="", sublabel="", color="#2979FF", parent=None):
        super().__init__(parent)
        self._color = color
        self._active = False
        self.setFixedSize(92, 52)

        self._base_style = (
            f"QFrame {{ background:{C_CARD}; border:1px solid {C_BORDER2}; border-radius:7px; }}"
        )
        self._active_style = (
            f"QFrame {{ background:{color}22; border:1px solid {color}; border-radius:7px; }}"
        )
        self.setStyleSheet(self._base_style)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 5, 6, 5)
        lay.setSpacing(1)

        self._lbl_main = QLabel(label.upper())
        self._lbl_main.setAlignment(Qt.AlignCenter)
        self._lbl_main.setStyleSheet(
            f"color:{C_MUTED}; font-size:11px; font-weight:bold; letter-spacing:1px; "
            f"border:none; background:transparent;"
        )
        self._lbl_sub = QLabel(sublabel)
        self._lbl_sub.setAlignment(Qt.AlignCenter)
        self._lbl_sub.setStyleSheet(
            f"color:{C_MUTED}; font-size:8px; border:none; background:transparent;"
        )
        lay.addWidget(self._lbl_main)
        lay.addWidget(self._lbl_sub)

    def set_active(self, state):
        self._active = state
        if state:
            self.setStyleSheet(self._active_style)
            self._lbl_main.setStyleSheet(
                f"color:{self._color}; font-size:11px; font-weight:bold; letter-spacing:1px; "
                f"border:none; background:transparent;"
            )
            self._lbl_sub.setStyleSheet(
                f"color:{C_TEXT2}; font-size:8px; border:none; background:transparent;"
            )
        else:
            self.setStyleSheet(self._base_style)
            self._lbl_main.setStyleSheet(
                f"color:{C_MUTED}; font-size:11px; font-weight:bold; letter-spacing:1px; "
                f"border:none; background:transparent;"
            )
            self._lbl_sub.setStyleSheet(
                f"color:{C_MUTED}; font-size:8px; border:none; background:transparent;"
            )


class NotificationBadge(QFrame):
    def __init__(self, label="", sublabel="", color="#2979FF", parent=None):
        super().__init__(parent)
        self._color  = color
        self._phase  = 0.0
        self._timer  = QTimer(self)
        self._timer.timeout.connect(self._tick)

        self.setFixedHeight(42)
        self._base_style = (
            f"QFrame {{ background:{C_CARD}; border:1px solid {C_BORDER2}; "
            f"border-radius:7px; border-left:3px solid {color}33; }}"
        )
        self._active_style = (
            f"QFrame {{ background:{color}15; border:1px solid {color}BB; "
            f"border-radius:7px; border-left:3px solid {color}; }}"
        )
        self.setStyleSheet(self._base_style)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 5, 8, 5)
        lay.setSpacing(7)

        self._dot = QLabel("◉")
        self._dot.setStyleSheet(f"color:{C_MUTED}; font-size:13px; font-weight:bold;")
        self._dot.setFixedWidth(18)
        lay.addWidget(self._dot)

        txt_lay = QVBoxLayout(); txt_lay.setSpacing(1)
        self._lbl_main = QLabel(label.upper())
        self._lbl_main.setStyleSheet(
            f"color:{C_TEXT2}; font-size:9px; font-weight:bold; letter-spacing:1px;"
        )
        self._lbl_sub = QLabel(sublabel)
        self._lbl_sub.setStyleSheet(f"color:{C_MUTED}; font-size:8px;")
        txt_lay.addWidget(self._lbl_main)
        txt_lay.addWidget(self._lbl_sub)
        lay.addLayout(txt_lay)
        lay.addStretch()

        self._lbl_status = QLabel("IDLE")
        self._lbl_status.setStyleSheet(
            f"color:{C_MUTED}; font-size:8px; font-weight:bold; letter-spacing:1px;"
        )
        lay.addWidget(self._lbl_status)

    def set_active(self, state):
        if state:
            self._phase = 0.0
            self._timer.start(30)
            self._lbl_status.setText("ACTIVE")
            self._lbl_status.setStyleSheet(
                f"color:{self._color}; font-size:8px; font-weight:bold; letter-spacing:1px;"
            )
            self._lbl_main.setStyleSheet(
                f"color:{self._color}; font-size:9px; font-weight:bold; letter-spacing:1px;"
            )
            self._lbl_sub.setStyleSheet(f"color:{C_TEXT2}; font-size:8px;")
            self.setStyleSheet(self._active_style)
        else:
            self._timer.stop()
            self._dot.setStyleSheet(f"color:{C_MUTED}; font-size:13px; font-weight:bold;")
            self._lbl_status.setText("IDLE")
            self._lbl_status.setStyleSheet(
                f"color:{C_MUTED}; font-size:8px; font-weight:bold; letter-spacing:1px;"
            )
            self._lbl_main.setStyleSheet(
                f"color:{C_TEXT2}; font-size:9px; font-weight:bold; letter-spacing:1px;"
            )
            self._lbl_sub.setStyleSheet(f"color:{C_MUTED}; font-size:8px;")
            self.setStyleSheet(self._base_style)

    def _tick(self):
        self._phase = (self._phase + 0.06) % 1.0
        pulse = abs(math.sin(self._phase * math.pi))
        alpha = int(80 + 175 * pulse)
        col = QColor(self._color)
        col.setAlpha(alpha)
        self._dot.setStyleSheet(
            f"color:rgba({col.red()},{col.green()},{col.blue()},{alpha}); "
            f"font-size:13px; font-weight:bold;"
        )


class NodeBadge(QFrame):
    def __init__(self, name, role, can_id, cmd_id, color, parent=None):
        super().__init__(parent)
        self._color = color
        self.setFixedHeight(50)
        self.setStyleSheet(
            f"QFrame {{ background:{C_CARD}; border:1px solid {C_BORDER2}; "
            f"border-radius:7px; border-left:3px solid {color}44; }}"
        )
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 5, 10, 5)
        lay.setSpacing(8)

        self._dot = QLabel("●")
        self._dot.setStyleSheet(f"color:{C_MUTED}; font-size:10px;")
        self._dot.setFixedWidth(14)
        lay.addWidget(self._dot)

        info = QVBoxLayout(); info.setSpacing(1)
        r1 = QHBoxLayout()
        lbl_name = QLabel(name.upper())
        lbl_name.setStyleSheet(f"color:{color}; font-size:9px; font-weight:bold; letter-spacing:2px;")
        lbl_role = QLabel(role)
        lbl_role.setStyleSheet(f"color:{C_TEXT}; font-size:8px;")
        r1.addWidget(lbl_name); r1.addWidget(lbl_role); r1.addStretch()
        info.addLayout(r1)

        r2 = QHBoxLayout()
        lbl_can = QLabel(f"CAN {can_id}")
        lbl_can.setStyleSheet(f"color:{C_TEXT2}; font-size:8px;")
        lbl_cmd = QLabel(f"CMD {cmd_id}")
        lbl_cmd.setStyleSheet(f"color:{C_TEXT2}; font-size:8px;")
        r2.addWidget(lbl_can); r2.addSpacing(8); r2.addWidget(lbl_cmd); r2.addStretch()
        info.addLayout(r2)
        lay.addLayout(info)

        self._lbl_val = QLabel("—")
        self._lbl_val.setStyleSheet(f"color:{C_TEXT}; font-size:13px; font-weight:bold;")
        self._lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self._lbl_val)

    def update_value(self, val, unit=""):
        self._lbl_val.setText(f"{val:.0f} {unit}")
        self._dot.setStyleSheet(f"color:{self._color}; font-size:10px;")
        self.setStyleSheet(
            f"QFrame {{ background:{C_CARD}; border:1px solid {self._color}55; "
            f"border-radius:7px; border-left:3px solid {self._color}; }}"
        )


class STM32Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpCode Labs  •  CAN Automotive Dashboard  •  5-Node Network")
        self.resize(1440, 880)
        self.setMinimumSize(1150, 700)
        self.setStyleSheet(QSS)

        self._serial = None
        self._reader = None
        self._cmd_widgets = []
        self._connected = False

        self._frame_count      = 0
        self._frame_count_prev = 0

        self._build_ui()
        self._refresh_ports()
        self._apply_connected(False)

        self._tick = QTimer(self)
        self._tick.timeout.connect(self._update_clock)
        self._tick.start(1000)

    def _build_ui(self):
        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(12, 10, 12, 6)
        root.setSpacing(8)
        root.addWidget(self._mk_header())
        root.addWidget(self._mk_warning_bar())

        body = QHBoxLayout()
        body.setSpacing(10)

        left_inner = QWidget()
        left_inner.setMinimumWidth(230)
        left_inner.setMaximumWidth(270)
        lv = QVBoxLayout(left_inner)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(8)
        lv.addWidget(self._mk_connection())
        lv.addWidget(self._mk_commands())
        lv.addWidget(self._mk_node_status())
        lv.addStretch()

        scroll = QScrollArea()
        scroll.setWidget(left_inner)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setMinimumWidth(238)
        scroll.setMaximumWidth(278)
        body.addWidget(scroll)

        center = QWidget()
        cv = QVBoxLayout(center)
        cv.setContentsMargins(0, 0, 0, 0)
        cv.setSpacing(8)
        cv.addWidget(self._mk_gauges())
        body.addWidget(center, stretch=1)

        right = QWidget()
        right.setMinimumWidth(280)
        right.setMaximumWidth(360)
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(8)
        rv.addWidget(self._mk_events())
        rv.addWidget(self._mk_log(), stretch=1)
        body.addWidget(right)

        root.addLayout(body, stretch=1)
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("Disconnected  •  OpCode Labs CAN Automotive Network")

    def _mk_header(self):
        w = QFrame()
        w.setStyleSheet(
            f"QFrame {{ background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
            f"stop:0 #0A1422, stop:0.5 #0E1A28, stop:1 #0A1422); "
            f"border: 1px solid {C_BORDER2}; border-radius:10px; }}"
        )
        w.setFixedHeight(56)
        h = QHBoxLayout(w)
        h.setContentsMargins(20, 0, 20, 0)

        dot = QLabel("◈")
        dot.setStyleSheet("color:#F0A500; font-size:22px;")
        h.addWidget(dot)
        h.addSpacing(10)
        title = QLabel("OPCODE  LABS")
        title.setStyleSheet("color:#F0A500; font-size:15px; font-weight:bold; letter-spacing:6px;")
        h.addWidget(title)
        h.addSpacing(8)
        subtitle = QLabel("CAN AUTOMOTIVE DASHBOARD")
        subtitle.setStyleSheet(f"color:{C_TEXT}; font-size:11px; letter-spacing:2px;")
        h.addWidget(subtitle)
        h.addStretch()

        nodes = [
            ("AMINE",   COL_PRESSURE, "_hdr_amine"),
            ("YASSINE", COL_SPEED,    "_hdr_yassine"),
            ("SOUHA",   COL_TEMP,     "_hdr_souha"),
            ("EYA",     COL_DEBIT,    "_hdr_eya"),
            ("NOUR",    COL_BATTERY,  "_hdr_nour"),
        ]
        for name, color, attr in nodes:
            frame = QFrame()
            frame.setStyleSheet(
                f"QFrame {{ background:{C_CARD}; border:1px solid {color}44; "
                f"border-radius:5px; border-left: 2px solid {color}; }}"
            )
            frame.setFixedSize(78, 34)
            fl = QVBoxLayout(frame)
            fl.setContentsMargins(6, 3, 6, 3)
            fl.setSpacing(1)
            dot2 = QLabel(f"● {name}")
            dot2.setStyleSheet(f"color:{color}; font-size:8px; font-weight:bold; letter-spacing:1px;")
            val_lbl = QLabel("OFFLINE")
            val_lbl.setStyleSheet(f"color:{C_TEXT2}; font-size:8px;")
            fl.addWidget(dot2)
            fl.addWidget(val_lbl)
            setattr(self, attr, dot2)
            setattr(self, attr + "_val", val_lbl)
            h.addWidget(frame)
            h.addSpacing(5)

        h.addSpacing(10)
        self._lbl_clock = QLabel("--:--:--")
        self._lbl_clock.setStyleSheet(f"color:{C_TEXT}; font-size:12px; font-family:Consolas;")
        h.addWidget(self._lbl_clock)
        h.addSpacing(16)

        self._lbl_status = QLabel("● OFFLINE")
        self._lbl_status.setStyleSheet(f"color:{C_RED}; font-weight:bold; font-size:12px;")
        h.addWidget(self._lbl_status)
        return w

    def _mk_warning_bar(self):
        w = QFrame()
        w.setStyleSheet(
            f"QFrame {{ background:{C_PANEL}; border:1px solid {C_BORDER2}; border-radius:10px; }}"
        )
        w.setFixedHeight(72)
        h = QHBoxLayout(w)
        h.setContentsMargins(14, 8, 14, 8)
        h.setSpacing(10)

        lbl = QLabel("WARNING  INDICATORS")
        lbl.setStyleSheet(
            f"color:{C_TEAL}; font-size:9px; font-weight:bold; letter-spacing:2px; "
            f"border:none; background:transparent;"
        )
        h.addWidget(lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"background:{C_BORDER2}; max-width:1px;")
        h.addWidget(sep)
        h.addSpacing(4)

        self._warn_abs  = WarningIndicator("ABS",  "Anti-lock",  COL_ABS)
        self._warn_ovht = WarningIndicator("OVHT", "Overheat",   COL_OVHT)
        self._warn_pres = WarningIndicator("PRES", "High Press", COL_PRES_W)
        self._warn_flow = WarningIndicator("FLOW", "Low Flow",   COL_FLOW_W)

        for wi in [self._warn_abs, self._warn_ovht, self._warn_pres, self._warn_flow]:
            h.addWidget(wi)

        h.addStretch()

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.VLine)
        sep2.setStyleSheet(f"background:{C_BORDER2}; max-width:1px;")
        h.addWidget(sep2)
        h.addSpacing(4)

        frames_box = QVBoxLayout()
        frames_box.setSpacing(2)
        lbl_fr = QLabel("FRAMES RX")
        lbl_fr.setStyleSheet(
            f"color:{C_TEXT2}; font-size:9px; font-weight:bold; letter-spacing:1px; "
            f"border:none; background:transparent;"
        )
        self._lbl_frames_hz = QLabel("0.0 Hz")
        self._lbl_frames_hz.setStyleSheet(
            f"color:{C_GREEN}; font-size:11px; font-weight:bold; "
            f"border:none; background:transparent;"
        )
        frames_box.addWidget(lbl_fr)
        frames_box.addWidget(self._lbl_frames_hz)
        h.addLayout(frames_box)

        self._lbl_frames_total = QLabel("0")
        self._lbl_frames_total.setStyleSheet(
            f"color:{C_TEXT}; font-size:16px; font-weight:bold; "
            f"border:none; background:transparent;"
        )
        h.addWidget(self._lbl_frames_total)
        return w

    def _mk_connection(self):
        grp = QGroupBox("CONNECTION")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_TEAL}; }}")
        lay = QVBoxLayout(grp)
        lay.setSpacing(6)
        lay.setContentsMargins(8, 14, 8, 10)

        r1 = QHBoxLayout()
        lbl_p = QLabel("Port:")
        lbl_p.setStyleSheet(f"color:{C_TEXT}; font-size:10px; font-weight:bold;")
        self._combo_port = QComboBox()
        self._btn_refresh = QPushButton("⟳")
        self._btn_refresh.setFixedWidth(30)
        self._btn_refresh.setFixedHeight(26)
        self._btn_refresh.clicked.connect(self._refresh_ports)
        r1.addWidget(lbl_p)
        r1.addWidget(self._combo_port, 1)
        r1.addWidget(self._btn_refresh)
        lay.addLayout(r1)

        r2 = QHBoxLayout()
        lbl_b = QLabel("Baud:")
        lbl_b.setStyleSheet(f"color:{C_TEXT}; font-size:10px; font-weight:bold;")
        self._combo_baud = QComboBox()
        for b in ["9600", "19200", "38400", "57600", "115200", "230400"]:
            self._combo_baud.addItem(b)
        self._combo_baud.setCurrentText("115200")
        r2.addWidget(lbl_b)
        r2.addWidget(self._combo_baud, 1)
        lay.addLayout(r2)

        br = QHBoxLayout()
        br.setSpacing(6)
        self._btn_conn = QPushButton("CONNECT")
        self._btn_conn.setObjectName("btn_connect")
        self._btn_disc = QPushButton("DISCONNECT")
        self._btn_disc.setObjectName("btn_disconnect")
        self._btn_conn.setFixedHeight(32)
        self._btn_disc.setFixedHeight(32)
        self._btn_conn.clicked.connect(self._connect)
        self._btn_disc.clicked.connect(self._disconnect)
        br.addWidget(self._btn_conn)
        br.addWidget(self._btn_disc)
        lay.addLayout(br)
        return grp

    def _mk_commands(self):
        grp = QGroupBox("COMMANDS  →  STM32")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: #FF6D00; }}")
        outer = QVBoxLayout(grp)
        outer.setContentsMargins(8, 18, 8, 10)
        outer.setSpacing(6)

        outer.addWidget(section_label("LIGHTS", C_GREEN))
        led_row = QHBoxLayout()
        led_row.setSpacing(6)
        self._btn_led_on  = QPushButton("● LIGHTS ON")
        self._btn_led_off = QPushButton("○ LIGHTS OFF")
        for btn, color in [(self._btn_led_on, C_GREEN), (self._btn_led_off, C_RED)]:
            btn.setFixedHeight(30)
            btn.setStyleSheet(
                f"QPushButton {{ background:{C_CARD}; border:1px solid {color}55; "
                f"border-radius:7px; color:{color}; font-weight:bold; font-size:9px; }}"
                f"QPushButton:hover {{ background:{color}22; border-color:{color}; }}"
                f"QPushButton:pressed {{ background:{color}; color:#000; }}"
                f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER2}; }}"
            )
            led_row.addWidget(btn)
        outer.addLayout(led_row)

        self._btn_read_sens = QPushButton("⟳   READ SENSOR")
        self._btn_read_sens.setFixedHeight(30)
        self._btn_read_sens.setStyleSheet(
            f"QPushButton {{ background:{C_CARD}; border:1px solid {C_AMBER}55; "
            f"border-radius:7px; color:{C_AMBER}; font-weight:bold; font-size:9px; letter-spacing:1px; }}"
            f"QPushButton:hover {{ background:{C_AMBER}22; border-color:{C_AMBER}; color:{C_AMBER_L}; }}"
            f"QPushButton:pressed {{ background:{C_AMBER}; color:#000; }}"
            f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER2}; }}"
        )
        outer.addWidget(self._btn_read_sens)

        self._btn_trigger_abs = QPushButton("⚠   TRIGGER ABS")
        self._btn_trigger_abs.setFixedHeight(30)
        self._btn_trigger_abs.setStyleSheet(
            f"QPushButton {{ background:{C_CARD}; border:1px solid {COL_ABS}55; "
            f"border-radius:7px; color:{COL_ABS}; font-weight:bold; font-size:9px; letter-spacing:1px; }}"
            f"QPushButton:hover {{ background:{COL_ABS}22; border-color:{COL_ABS}; }}"
            f"QPushButton:pressed {{ background:{COL_ABS}; color:#000; }}"
            f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER2}; }}"
        )
        outer.addWidget(self._btn_trigger_abs)

        self._btn_led_on.clicked.connect(lambda: self._send(CMD_LED_ON, 0))
        self._btn_led_off.clicked.connect(lambda: self._send(CMD_LED_OFF, 0))
        self._btn_read_sens.clicked.connect(lambda: self._send(CMD_READ_SENSOR, 0))
        self._btn_trigger_abs.clicked.connect(lambda: self._send(CMD_TRIGGER_ABS, 0))
        self._cmd_widgets.extend([
            self._btn_led_on, self._btn_led_off,
            self._btn_read_sens, self._btn_trigger_abs
        ])

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background:{C_BORDER2}; max-height:1px; margin:4px 0px;")
        outer.addWidget(sep)

        outer.addWidget(section_label("MANUAL FRAME", C_AMBER))
        self._edit_cmd = QComboBox()
        self._edit_cmd.setEditable(True)
        for c in ["0x01", "0x02", "0x10", "0x11", "0x20", "0x21", "0x22", "0x23", "0x24"]:
            self._edit_cmd.addItem(c)
        self._edit_cmd.setCurrentText("0x10")

        self._edit_data = QComboBox()
        self._edit_data.setEditable(True)
        for d in ["0x00000000", "0x00000001", "0x0000007F", "0x000000FF"]:
            self._edit_data.addItem(d)
        self._edit_data.setCurrentText("0x00000000")

        r_cmd = QHBoxLayout()
        lbl_c = QLabel("CMD")
        lbl_c.setStyleSheet(f"color:{C_TEXT}; font-size:9px; font-weight:bold; min-width:36px;")
        r_cmd.addWidget(lbl_c)
        r_cmd.addWidget(self._edit_cmd, 1)
        outer.addLayout(r_cmd)

        r_dat = QHBoxLayout()
        lbl_d = QLabel("DATA")
        lbl_d.setStyleSheet(f"color:{C_TEXT}; font-size:9px; font-weight:bold; min-width:36px;")
        r_dat.addWidget(lbl_d)
        r_dat.addWidget(self._edit_data, 1)
        outer.addLayout(r_dat)

        btn_send = QPushButton("▶   SEND FRAME")
        btn_send.setFixedHeight(32)
        btn_send.setStyleSheet(
            f"QPushButton {{ background:{C_AMBER_D}; border:1px solid {C_AMBER}; "
            f"border-radius:7px; color:{C_AMBER_L}; font-weight:bold; font-size:10px; letter-spacing:1px; }}"
            f"QPushButton:hover {{ background:{C_AMBER}; color:#000; }}"
            f"QPushButton:pressed {{ background:{C_AMBER}; color:#000; }}"
            f"QPushButton:disabled {{ background:{C_PANEL}; color:{C_MUTED}; border-color:{C_BORDER2}; }}"
        )
        btn_send.clicked.connect(self._send_manual)
        outer.addWidget(btn_send)
        self._cmd_widgets.append(btn_send)
        return grp

    def _mk_node_status(self):
        grp = QGroupBox("CAN  NODE  STATUS")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_GREEN}; }}")
        lay = QVBoxLayout(grp)
        lay.setSpacing(5)
        lay.setContentsMargins(8, 16, 8, 8)

        self._badge_amine   = NodeBadge("Amine",   "Pressure",       "0x55C", "0x20", COL_PRESSURE)
        self._badge_yassine = NodeBadge("Yassine", "Speed Motor",    "0x406", "0x21", COL_SPEED)
        self._badge_souha   = NodeBadge("Souha",   "Temperature",    "0x456", "0x22", COL_TEMP)
        self._badge_eya     = NodeBadge("Eya",     "Débit Fluide",   "0x390", "0x23", COL_DEBIT)
        self._badge_nour    = NodeBadge("Nour",    "Battery Voltage","0x222", "0x24", COL_BATTERY)

        for bg in [self._badge_amine, self._badge_yassine, self._badge_souha,
                   self._badge_eya, self._badge_nour]:
            lay.addWidget(bg)
        return grp

    def _mk_gauges(self):
        grp = QGroupBox("SENSOR  DASHBOARD  ←  CAN  NETWORK")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_AMBER}; }}")
        v = QVBoxLayout(grp)
        v.setContentsMargins(10, 22, 10, 10)
        v.setSpacing(10)

        self._gauge_pressure = ClassicGauge("Pressure", "bar", 0, 255, COL_PRESSURE, "Amine · 0x55C")
        self._gauge_speed    = ClassicGauge("Speed Motor", "rpm", 0, 5000, COL_SPEED, "Yassine · 0x406")
        self._gauge_temp     = ClassicGauge("Temperature", "°C", 0, 150, COL_TEMP, "Souha · 0x456")
        self._gauge_debit    = ClassicGauge("Débit Fluide", "L/s", 0, 255, COL_DEBIT, "Eya · 0x390")
        self._gauge_battery  = ClassicGauge("Battery Voltage", "mV", 0, 15000, COL_BATTERY, "Nour · 0x222")

        row_top = QHBoxLayout()
        row_top.setSpacing(8)
        row_top.addWidget(self._gauge_speed,    stretch=9)
        row_top.addWidget(self._gauge_pressure, stretch=11)
        row_top.addWidget(self._gauge_temp,     stretch=9)
        v.addLayout(row_top, stretch=1)

        row_bot = QHBoxLayout()
        row_bot.setSpacing(8)
        row_bot.addWidget(self._gauge_battery, stretch=1)
        row_bot.addWidget(self._gauge_debit,   stretch=1)
        v.addLayout(row_bot, stretch=1)

        self._gauges = {
            CMD_PRESSURE       : self._gauge_pressure,
            CMD_SPEED          : self._gauge_speed,
            CMD_TEMPERATURE    : self._gauge_temp,
            CMD_DEBIT          : self._gauge_debit,
            CMD_BATTERY_VOLTAGE: self._gauge_battery,
        }
        return grp

    def _mk_events(self):
        grp = QGroupBox("NETWORK  EVENTS  —  NOTIFICATIONS  PASSIVES")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_TEAL}; }}")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(8, 18, 8, 10)
        lay.setSpacing(6)

        self._notif_abs   = NotificationBadge("ABS",           "Amine · 0x679",   COL_ABS)
        self._notif_winL  = NotificationBadge("Window Left",   "Yassine · 0x401", COL_WIN_L)
        self._notif_winR  = NotificationBadge("Window Right",  "Souha · 0x402",   COL_WIN_R)
        self._notif_door  = NotificationBadge("Door Right",    "Eya · 0x790",     COL_DOOR)
        self._notif_extlt = NotificationBadge("External Light","Nour · 0x221",    COL_EXT_LGT)

        for nb in [self._notif_abs, self._notif_winL, self._notif_winR,
                   self._notif_door, self._notif_extlt]:
            lay.addWidget(nb)
        return grp

    def _mk_log(self):
        grp = QGroupBox("FRAME  LOG  —  SERIAL  MONITOR")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_TEAL}; }}")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(8, 18, 8, 8)
        lay.setSpacing(5)

        self._log = QTextEdit()
        self._log.setReadOnly(True)
        lay.addWidget(self._log)

        row = QHBoxLayout()
        self._chk_log_tx = QCheckBox("Log TX")
        self._chk_log_rx = QCheckBox("Log RX")
        self._chk_log_rx.setChecked(True)
        btn_clr = QPushButton("Clear")
        btn_clr.setFixedWidth(60)
        btn_clr.setFixedHeight(26)
        btn_clr.clicked.connect(self._log.clear)
        row.addWidget(self._chk_log_tx)
        row.addWidget(self._chk_log_rx)
        row.addStretch()
        row.addWidget(btn_clr)
        lay.addLayout(row)
        return grp

    def _refresh_ports(self):
        self._combo_port.clear()
        ports = serial.tools.list_ports.comports()
        default_index = 0
        if ports:
            for i, p in enumerate(ports):
                desc = f"{p.device}  –  {p.description[:22]}" if p.description not in ("n/a", "") else p.device
                self._combo_port.addItem(desc, userData=p.device)
                if p.device == "/dev/ttyACM0":
                    default_index = i
            self._combo_port.setCurrentIndex(default_index)
        else:
            self._combo_port.addItem("No ports detected", userData="")

    def _connect(self):
        port = self._combo_port.currentData()
        if not port:
            self._sys_log("No valid port selected.")
            return
        baud = int(self._combo_baud.currentText())
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self._reader = SerialReader(self._serial, self)
            self._reader.sig_frame.connect(self._on_frame)
            self._reader.sig_raw.connect(self._on_raw)
            self._reader.sig_error.connect(self._on_serial_error)
            self._reader.start()
            self._apply_connected(True)
            self._status.showMessage(f"Connected  •  {port}  •  {baud} baud")
            self._sys_log(f"Port ouvert : {port} @ {baud}")
        except serial.SerialException as exc:
            self._sys_log(f"ERREUR : {exc}")

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader.wait(600)
            self._reader = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._apply_connected(False)
        self._status.showMessage("Disconnected  •  OpCode Labs CAN Network")
        self._sys_log("Port fermé.")

    def _apply_connected(self, state):
        self._connected = state
        self._btn_conn.setEnabled(not state)
        self._btn_disc.setEnabled(state)
        self._combo_port.setEnabled(not state)
        self._combo_baud.setEnabled(not state)
        for w in self._cmd_widgets:
            w.setEnabled(state)
        if state:
            self._lbl_status.setText("● ONLINE")
            self._lbl_status.setStyleSheet(f"color:{C_GREEN}; font-weight:bold; font-size:12px;")
        else:
            self._lbl_status.setText("● OFFLINE")
            self._lbl_status.setStyleSheet(f"color:{C_RED}; font-weight:bold; font-size:12px;")

    def _send(self, cmd, data=0):
        if not (self._serial and self._serial.is_open):
            return
        frame = struct.pack('>BBI', SOF, cmd, data)
        self._serial.write(frame)
        if self._chk_log_tx.isChecked():
            self._tx_log(cmd, frame)

    def _send_manual(self):
        try:
            cmd  = int(self._edit_cmd.currentText(),  16)
            data = int(self._edit_data.currentText(), 16)
        except ValueError:
            self._sys_log("Format invalide — utiliser hex ex: 0x20")
            return
        self._send(cmd, data)

    def _on_frame(self, cmd, value):
        self._frame_count += 1
        self._lbl_frames_total.setText(str(self._frame_count))

        if cmd in self._gauges:
            self._gauges[cmd].set_value(value)
            unit_map = {
                CMD_PRESSURE: "bar", CMD_SPEED: "rpm", CMD_TEMPERATURE: "°C",
                CMD_DEBIT: "L/s", CMD_BATTERY_VOLTAGE: "mV"
            }
            badge_map = {
                CMD_PRESSURE: self._badge_amine, CMD_SPEED: self._badge_yassine,
                CMD_TEMPERATURE: self._badge_souha, CMD_DEBIT: self._badge_eya,
                CMD_BATTERY_VOLTAGE: self._badge_nour
            }
            badge_map[cmd].update_value(value, unit_map[cmd])
            hdr_map = {
                CMD_PRESSURE: "amine", CMD_SPEED: "yassine", CMD_TEMPERATURE: "souha",
                CMD_DEBIT: "eya", CMD_BATTERY_VOLTAGE: "nour"
            }
            self._set_header_online(hdr_map[cmd], value, unit_map[cmd])

            if cmd == CMD_TEMPERATURE:
                self._warn_ovht.set_active(value >= THRESH_OVERHEAT)
            elif cmd == CMD_PRESSURE:
                self._warn_pres.set_active(value >= THRESH_HIGH_PRESS)
            elif cmd == CMD_DEBIT:
                self._warn_flow.set_active(value <= THRESH_LOW_FLOW)

        elif cmd == CMD_WINDOW_LEFT:
            self._notif_winL.set_active(True)
            QTimer.singleShot(2000, lambda: self._notif_winL.set_active(False))
            self._status.showMessage("WINDOW LEFT — Yassine", 2000)

        elif cmd == CMD_WINDOW_RIGHT:
            self._notif_winR.set_active(True)
            QTimer.singleShot(2000, lambda: self._notif_winR.set_active(False))
            self._status.showMessage("WINDOW RIGHT — Souha", 2000)

        elif cmd == CMD_PORTE_DROIT:
            self._notif_door.set_active(True)
            QTimer.singleShot(3000, lambda: self._notif_door.set_active(False))
            self._status.showMessage("DOOR RIGHT — Eya", 3000)

        elif cmd == CMD_EXTERNAL_LIGHT:
            self._notif_extlt.set_active(True)
            QTimer.singleShot(2000, lambda: self._notif_extlt.set_active(False))
            self._status.showMessage("EXTERNAL LIGHT — Nour", 2000)

        elif cmd == CMD_ACK:
            payload0 = int(value) >> 24 & 0xFF
            if payload0 == 0x01:
                self._notif_abs.set_active(True)
                self._warn_abs.set_active(True)
                QTimer.singleShot(2500, lambda: self._notif_abs.set_active(False))
                QTimer.singleShot(2500, lambda: self._warn_abs.set_active(False))
                self._status.showMessage("ABS — Amine", 2500)
            else:
                self._status.showMessage("ACK reçu du STM32 (LED)", 1500)

    def _set_header_online(self, name, val, unit):
        colors = {
            "amine": COL_PRESSURE, "yassine": COL_SPEED, "souha": COL_TEMP,
            "eya": COL_DEBIT, "nour": COL_BATTERY
        }
        color = colors.get(name, C_TEXT)
        dot_lbl = getattr(self, f"_hdr_{name}", None)
        val_lbl = getattr(self, f"_hdr_{name}_val", None)
        if dot_lbl:
            dot_lbl.setStyleSheet(f"color:{color}; font-size:8px; font-weight:bold; letter-spacing:1px;")
        if val_lbl:
            val_lbl.setStyleSheet(f"color:{C_GREEN}; font-size:8px; font-weight:bold;")
            val_lbl.setText(f"{val:.0f} {unit}")

    def _on_raw(self, frame):
        if not self._chk_log_rx.isChecked():
            return
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = " ".join(f"{b:02X}" for b in frame)
        cmd = frame[1]
        name = RX_NAMES.get(cmd, f"0x{cmd:02X}")
        self._log.append(
            f'<span style="color:{C_TEXT2}; font-size:12px;">{ts}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_TEAL}; font-size:12px; font-weight:bold;">RX</span>&nbsp;&nbsp;'
            f'<span style="color:#4CAF50; font-size:13px; font-family:Consolas;">{hex_str}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_TEXT}; font-size:12px; font-weight:bold;">{name}</span>'
        )

    def _tx_log(self, cmd, frame):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        hex_str = " ".join(f"{b:02X}" for b in frame)
        self._log.append(
            f'<span style="color:{C_TEXT2}; font-size:12px;">{ts}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_GREEN}; font-size:12px; font-weight:bold;">TX</span>&nbsp;&nbsp;'
            f'<span style="color:#4CAF50; font-size:13px; font-family:Consolas;">{hex_str}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_AMBER_L}; font-size:12px; font-weight:bold;">CMD 0x{cmd:02X}</span>'
        )

    def _sys_log(self, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.append(
            f'<span style="color:{C_TEXT2}; font-size:12px;">{ts}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_AMBER_L}; font-size:12px; font-weight:bold;">SYS</span>&nbsp;&nbsp;'
            f'<span style="color:{C_TEXT}; font-size:12px;">{msg}</span>'
        )

    def _on_serial_error(self, msg):
        self._sys_log(f"Liaison perdue : {msg}")
        self._disconnect()

    def _update_clock(self):
        self._lbl_clock.setText(datetime.now().strftime("%H:%M:%S"))
        delta = self._frame_count - self._frame_count_prev
        self._frame_count_prev = self._frame_count
        self._lbl_frames_hz.setText(f"{delta:.1f} Hz")

    def closeEvent(self, event):
        self._disconnect()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = STM32Dashboard()
    win.show()
    sys.exit(app.exec_())