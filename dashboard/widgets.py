"""Reusable Qt widgets used by the STM32 dashboard."""

import math
from datetime import datetime

from PyQt5.QtWidgets import (
    QLabel, QFrame, QVBoxLayout, QHBoxLayout, QSizePolicy, QWidget
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF
from PyQt5.QtGui import QFont, QColor, QPainter, QPen, QBrush, QRadialGradient

if __package__:
    from .config import *
else:
    from config import *

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

        pct_arc = (self._anim_val - self.v_min) / max(self.v_max - self.v_min, 1e-9)
        pct_arc = max(0.0, min(1.0, pct_arc))
        arc_r = r * 0.955
        arc_rect = QRectF(cx - arc_r, cy - arc_r, arc_r * 2, arc_r * 2)
        pen_w = max(5.0, r * 0.085)

        bg_pen = QPen(QColor(C_TICK), pen_w, Qt.SolidLine, Qt.RoundCap)
        p.setPen(bg_pen)
        p.drawArc(arc_rect, int(start_angle * 16), int(-sweep_angle * 16))

        if pct_arc > 0.0:
            lit_pen = QPen(self.color, pen_w, Qt.SolidLine, Qt.RoundCap)
            p.setPen(lit_pen)
            p.drawArc(arc_rect, int(start_angle * 16), int(-sweep_angle * pct_arc * 16))
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

        pct = pct_arc
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
        p.setPen(self.color if (self._online and not self._stale) else QColor(C_MUTED))
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
