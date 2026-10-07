"""Shared dashboard protocol settings, thresholds, and visual theme."""

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


