"""Virtual test bench: the five ECUs and their shared CAN bus, driven by hand.

Each board has a slider for its sensor input and a push button for its manual
trigger. Whatever is set here goes onto the simulated bus and reaches the
dashboard through ECU-01's UART link, as it would on the real hardware.

Run with ``python dashboard/stm32_dashboard.py --sim`` (bench + dashboard) or
``python dashboard/virtual_bench.py`` (bench only, then connect a dashboard to
the "Virtual ECU (simulator)" port).
"""

import sys
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QTextEdit, QFrame, QCheckBox, QSlider
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QFont

if __package__:
    from .config import *
    from .virtual_ecu import NODES, URL, VirtualBus
else:
    from config import *
    from virtual_ecu import NODES, URL, VirtualBus

CARD_SPACING = 10
FLASH_MS     = 220

BENCH_QSS = QSS + f"""
QSlider::groove:horizontal {{ height: 6px; background: {C_CARD2}; border-radius: 3px; }}
QSlider::handle:horizontal {{
    width: 16px; margin: -6px 0; border-radius: 8px;
    background: {C_TEXT}; border: 1px solid {C_BORDER2};
}}
QSlider::sub-page:horizontal {{ background: {C_AMBER}; border-radius: 3px; }}
"""


class Led(QWidget):
    def __init__(self, color, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._on    = False
        self.setFixedSize(14, 14)

    def set_on(self, state):
        self._on = bool(state)
        self.update()

    def flash(self, ms=120):
        self.set_on(True)
        QTimer.singleShot(ms, lambda: self.set_on(False))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(C_BORDER2), 1))
        p.setBrush(self._color if self._on else QColor(C_MUTED))
        p.drawEllipse(QRectF(1.5, 1.5, 11, 11))


class NodeCard(QFrame):
    """One NUCLEO board: status LEDs, a sensor input and a push button."""

    def __init__(self, node, bus, parent=None):
        super().__init__(parent)
        self.node = node
        self._bus = bus
        self.setStyleSheet(
            f"NodeCard {{ background:{C_PANEL}; border:1px solid {C_BORDER2}; "
            f"border-top:3px solid {node.color}; border-radius:10px; }}"
        )
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 12)
        lay.setSpacing(6)

        head = QLabel(f"{node.ecu}  ·  {node.owner.upper()}")
        head.setStyleSheet(f"color:{node.color}; font-size:12px; font-weight:bold; letter-spacing:2px;")
        board = QLabel("NUCLEO-F429ZI")
        board.setStyleSheet(f"color:{C_TEXT2}; font-size:9px; letter-spacing:1px;")
        lay.addWidget(head)
        lay.addWidget(board)

        leds = QHBoxLayout()
        leds.setSpacing(5)
        self.led_heartbeat = Led(C_GREEN)
        self.led_tx        = Led(node.color)
        for led, text in [(self.led_heartbeat, "LD2"), (self.led_tx, "CAN TX")]:
            leds.addWidget(led)
            leds.addWidget(self._small(text))
            leds.addSpacing(6)
        self.led_user = None
        if node.ecu == "ECU-01":
            self.led_user = Led(C_TEAL)
            leds.addWidget(self.led_user)
            leds.addWidget(self._small("LD1"))
        leds.addStretch()
        lay.addLayout(leds)

        lay.addSpacing(4)
        lay.addWidget(section_title(f"SENSOR  ·  {node.sensor.upper()}", node.color))
        self._lbl_value = QLabel()
        self._lbl_value.setAlignment(Qt.AlignCenter)
        self._lbl_value.setStyleSheet(f"color:{node.color}; font-size:26px; font-weight:bold;")
        lay.addWidget(self._lbl_value)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, node.v_max)
        self._slider.setPageStep(max(1, node.v_max // 20))
        self._slider.setValue(node.default)
        self._slider.valueChanged.connect(self._on_slider)
        lay.addWidget(self._slider)

        rng = QHBoxLayout()
        rng.addWidget(self._small("0"))
        rng.addStretch()
        self._chk_auto = QCheckBox("Auto sweep")
        self._chk_auto.toggled.connect(lambda s: bus.set_auto(node.cmd, s))
        rng.addWidget(self._chk_auto)
        rng.addStretch()
        rng.addWidget(self._small(str(node.v_max)))
        lay.addLayout(rng)

        lay.addWidget(self._small(f"CAN 0x{node.can_id:03X}  →  UART 0x{node.cmd:02X}"))

        lay.addSpacing(4)
        lay.addWidget(section_title("PUSH BUTTON", node.color))
        btn = QPushButton(f"●  {node.ev_name.upper()}")
        btn.setFixedHeight(34)
        btn.setStyleSheet(
            f"QPushButton {{ background:{C_CARD}; border:1px solid {tint(node.color, '88')}; "
            f"border-radius:7px; color:{node.color}; font-weight:bold; font-size:10px; }}"
            f"QPushButton:hover {{ background:{tint(node.color, '22')}; border-color:{node.color}; }}"
            f"QPushButton:pressed {{ background:{node.color}; color:#000; }}"
        )
        btn.clicked.connect(lambda: bus.trigger(node.ev_cmd))
        self.button = btn
        lay.addWidget(btn)
        lay.addWidget(self._small(f"CAN 0x{node.ev_can_id:03X}  →  UART 0x{node.ev_cmd:02X}"))
        lay.addStretch()

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

    def set_auto(self, state):
        self._chk_auto.setChecked(state)

    def set_value(self, value):
        self._slider.setValue(value)

    def on_sensor_frame(self, value):
        self._slider.blockSignals(True)
        self._slider.setValue(value)
        self._slider.blockSignals(False)
        self._show(value)


def tint(color, alpha):
    """Translucent version of a #RRGGBB color (Qt style sheets expect #AARRGGBB)."""
    return f"#{alpha}{color[1:]}"


def px_font(size, bold=False):
    f = QFont("Consolas")
    f.setPixelSize(size)
    f.setBold(bold)
    return f


def section_title(text, color):
    lbl = QLabel(text)
    lbl.setStyleSheet(
        f"color:{color}; font-size:8px; font-weight:bold; letter-spacing:2px; "
        f"background:{tint(color, '18')}; border-left:2px solid {color}; padding:3px 6px;"
    )
    return lbl


class BusView(QWidget):
    """CANH/CANL pair with a stub per node, terminators, and ECU-01's UART link."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(170)
        self._active   = None      # index of the node currently transmitting
        self._text     = ""
        self._uart     = None      # color of the current UART flash
        self._linked   = False
        self._token    = 0
        self._u_token  = 0

    def set_linked(self, state):
        self._linked = state
        self.update()

    def flash_can(self, index, text):
        self._active, self._text = index, text
        self._token += 1
        token = self._token
        QTimer.singleShot(FLASH_MS, lambda: self._clear_can(token))
        self.update()

    def _clear_can(self, token):
        if token == self._token:
            self._active = None
            self.update()

    def flash_uart(self, color):
        self._uart = color
        self._u_token += 1
        token = self._u_token
        QTimer.singleShot(FLASH_MS, lambda: self._clear_uart(token))
        self.update()

    def _clear_uart(self, token):
        if token == self._u_token:
            self._uart = None
            self.update()

    def _centre(self, i):
        card_w = (self.width() - CARD_SPACING * (len(NODES) - 1)) / len(NODES)
        return i * (card_w + CARD_SPACING) + card_w / 2

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        y_h, y_l = 46, 60
        x0, x1 = 34, w - 34
        idle = QColor(C_BORDER2)
        live = QColor(NODES[self._active].color) if self._active is not None else None

        # stubs: transceiver box on each board, then down to the pair
        for i, node in enumerate(NODES):
            cx = self._centre(i)
            col = QColor(node.color) if i == self._active else idle
            p.setPen(QPen(col, 3 if i == self._active else 1.5))
            p.drawLine(QPointF(cx - 4, 22), QPointF(cx - 4, y_h))
            p.drawLine(QPointF(cx + 4, 22), QPointF(cx + 4, y_l))
            p.setPen(QPen(QColor(node.color), 1))
            p.setBrush(QColor(C_CARD))
            p.drawRoundedRect(QRectF(cx - 46, 2, 92, 20), 4, 4)
            p.setFont(px_font(9, True))
            p.drawText(QRectF(cx - 46, 2, 92, 20), Qt.AlignCenter, "CAN TRANSCEIVER")

        # the differential pair
        p.setPen(QPen(live or idle, 4 if live else 2.5))
        p.drawLine(QPointF(x0, y_h), QPointF(x1, y_h))
        p.drawLine(QPointF(x0, y_l), QPointF(x1, y_l))
        p.setFont(px_font(9))
        p.setPen(QColor(C_TEXT2))
        p.drawText(QRectF(x1 - 60, y_h - 14, 50, 12), Qt.AlignRight, "CAN_H")
        p.drawText(QRectF(x1 - 60, y_l + 3, 50, 12), Qt.AlignRight, "CAN_L")

        # 120 Ω termination at both ends
        for x in (x0, x1):
            p.setPen(QPen(QColor(C_TEXT2), 1))
            p.setBrush(QColor(C_CARD2))
            p.drawRect(QRectF(x - 6, y_h - 4, 12, y_l - y_h + 8))
            p.drawText(QRectF(x - 30, y_l + 8, 60, 12), Qt.AlignCenter, "120 Ω")

        # frame currently on the bus
        if live:
            p.setPen(live)
            p.setFont(px_font(13, True))
            p.drawText(QRectF(0, y_l + 22, w, 18), Qt.AlignCenter, self._text)

        # ECU-01's UART link down to the PC
        ux = self._centre(0) - 60
        if self._uart:
            link_col, width = QColor(self._uart), 3
        else:
            link_col, width = QColor(C_TEAL if self._linked else C_MUTED), 1.5
        pen = QPen(link_col, width)
        pen.setStyle(Qt.DashLine)
        p.setPen(pen)
        p.drawLine(QPointF(ux, 0), QPointF(ux, 126))
        box = QRectF(4, 126, 270, 38)
        p.setPen(QPen(link_col, 1.5))
        p.setBrush(QColor(C_CARD))
        p.drawRoundedRect(box, 6, 6)
        p.setFont(px_font(10, True))
        state = "CONNECTED" if self._linked else "NOT CONNECTED"
        p.drawText(box, Qt.AlignCenter, f"PC DASHBOARD  ·  UART 115200\nST-Link VCP  ·  {state}")


class VirtualBench(QMainWindow):
    sig_bus = pyqtSignal(str, object, object, object)

    def __init__(self, bus):
        super().__init__()
        self.setWindowTitle("Virtual Bench  •  5 ECUs on a shared CAN bus")
        self.resize(1180, 820)
        self.setStyleSheet(BENCH_QSS)
        self._bus = bus
        self._by_can = {}
        self._linked = False

        root_w = QWidget()
        self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        head = QHBoxLayout()
        title = QLabel("VIRTUAL  BENCH")
        title.setStyleSheet(f"color:{C_AMBER}; font-size:15px; font-weight:bold; letter-spacing:6px;")
        hint = QLabel("Move a slider or press a button — the frame goes onto the bus and into the dashboard")
        hint.setStyleSheet(f"color:{C_TEXT2}; font-size:10px;")
        self._chk_all = QCheckBox("Auto sweep all sensors")
        self._chk_all.toggled.connect(lambda s: [c.set_auto(s) for c in self._cards])
        head.addWidget(title)
        head.addSpacing(14)
        head.addWidget(hint)
        head.addStretch()
        head.addWidget(self._chk_all)
        root.addLayout(head)

        cards = QHBoxLayout()
        cards.setSpacing(CARD_SPACING)
        self._cards = []
        for i, node in enumerate(NODES):
            card = NodeCard(node, bus)
            cards.addWidget(card, stretch=1)
            self._cards.append(card)
            self._by_can[node.can_id]    = (i, node, False)
            self._by_can[node.ev_can_id] = (i, node, True)
        root.addLayout(cards)

        self._bus_view = BusView()
        root.addWidget(self._bus_view)

        grp = QGroupBox("CAN  BUS  MONITOR")
        grp.setStyleSheet(grp.styleSheet() + f"QGroupBox::title {{ color: {C_TEAL}; }}")
        gl = QVBoxLayout(grp)
        gl.setContentsMargins(8, 18, 8, 8)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.document().setMaximumBlockCount(300)
        gl.addWidget(self._log)
        row = QHBoxLayout()
        self._chk_sensors = QCheckBox("Show periodic sensor frames")
        row.addWidget(self._chk_sensors)
        row.addStretch()
        gl.addLayout(row)
        root.addWidget(grp, stretch=1)

        self._beat = QTimer(self)
        self._beat.timeout.connect(self._heartbeat)
        self._beat.start(500)
        self._beat_on = False

        # Bus callbacks arrive on worker threads; the signal hops them to the GUI thread
        self.sig_bus.connect(self._on_bus)
        bus.listen(self.sig_bus.emit)

    def card(self, index):
        return self._cards[index]

    def _heartbeat(self):
        self._beat_on = not self._beat_on
        for c in self._cards:
            c.led_heartbeat.set_on(self._beat_on)

    def _on_bus(self, kind, a, b, c):
        if kind == "can":
            index, node, is_event = self._by_can[a]
            card = self._cards[index]
            if is_event:
                data, what = "01", f"{node.ev_name} button"
            else:
                card.on_sensor_frame(c)
                data = " ".join(f"{x:02X}" for x in c.to_bytes(node.dlc, "big"))
                what = f"{node.sensor} = {c} {node.unit}"
            if not self._linked:
                return              # dashboard disconnected: no transfer shown
            card.led_tx.flash()
            text = f"0x{a:03X}   [{data}]   {node.ecu} · {what}"
            self._bus_view.flash_can(index, text)
            if is_event or self._chk_sensors.isChecked():
                self._append("CAN", node.color, text)
        elif kind == "uart_tx":
            self._bus_view.flash_uart(C_TEAL)
        elif kind == "uart_rx":
            self._bus_view.flash_uart(C_AMBER)
            self._append("UART", C_AMBER_L, f"Dashboard → ECU-01   CMD 0x{a:02X}   DATA 0x{b:08X}")
        elif kind == "led":
            self._cards[0].led_user.set_on(a)
            self._append("GPIO", C_TEAL, f"ECU-01 LD1 {'ON' if a else 'OFF'}")
        elif kind == "link":
            self._linked = bool(a)
            self._bus_view.set_linked(bool(a))
            self._append("UART", C_AMBER_L, "Dashboard connected" if a else "Dashboard disconnected")

    def _append(self, tag, color, msg):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.append(
            f'<span style="color:{C_TEXT2};">{ts}</span>&nbsp;&nbsp;'
            f'<span style="color:{color}; font-weight:bold;">{tag}</span>&nbsp;&nbsp;'
            f'<span style="color:{C_TEXT};">{msg}</span>'
        )


def open_bench(bus=None):
    """Show the bench on ``bus``, or on a virtual bus of its own."""
    own = bus is None
    if own:
        bus = VirtualBus(auto=False)
    win = VirtualBench(bus)
    if own:
        bus.start()
    win.show()
    return win


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    bench = open_bench()
    print(f"Virtual bench running, connect the dashboard to {URL}")
    sys.exit(app.exec_())
