"""Main application window for the STM32 automotive dashboard."""

import struct
import serial
import serial.tools.list_ports
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QTextEdit, QFrame,
    QStatusBar, QCheckBox, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer

if __package__:
    from .config import *
    from .protocol import SerialReader
    from .virtual_ecu import URL as VIRTUAL_ECU_URL
    from .widgets import *
else:
    from config import *
    from protocol import SerialReader
    from virtual_ecu import URL as VIRTUAL_ECU_URL
    from widgets import *

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
        self._combo_port.addItem("Virtual ECU (simulator)", userData=VIRTUAL_ECU_URL)

    def connect_virtual(self):
        self._combo_port.setCurrentIndex(self._combo_port.findData(VIRTUAL_ECU_URL))
        self._connect()

    def _connect(self):
        port = self._combo_port.currentData()
        if not port:
            self._sys_log("No valid port selected.")
            return
        baud = int(self._combo_baud.currentText())
        try:
            self._serial = serial.serial_for_url(port, baudrate=baud, timeout=0.1)
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
