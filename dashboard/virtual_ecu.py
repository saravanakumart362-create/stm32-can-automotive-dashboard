"""Virtual 5-node CAN bus + ECU-01 UART link, for running the dashboard without hardware.

Stands in for the NUCLEO board's ST-Link virtual COM port: it serves the same
6-byte UART frames (SOF + CMD + 4-byte payload) over a local TCP socket, which
the dashboard opens through pyserial's ``socket://`` URL handler.

This module is the headless model. ``virtual_bench.py`` puts a window on top of
it so sensor inputs and buttons can be driven by hand.
"""

import math
import random
import socket
import struct
import threading
import time
from collections import namedtuple

if __package__:
    from .config import *
else:
    from config import *

HOST = "127.0.0.1"
PORT = 5555
URL  = f"socket://{HOST}:{PORT}"

SENSOR_PERIOD = 0.5     # s, same cadence as the firmware main loop
SCENARIO_LEN  = 60.0    # s, one full auto sweep through every warning threshold

ABS_EVENT = 0x01000000  # CMD_ACK with payload[0] = 1

Node = namedtuple("Node", "ecu owner sensor unit v_max cmd can_id dlc default "
                          "ev_name ev_cmd ev_can_id color")

NODES = [
    Node("ECU-01", "Amine",   "Pressure",        "bar", 255,   CMD_PRESSURE,        0x55C, 4, 120,
         "ABS",            CMD_ACK,            0x679, COL_PRESSURE),
    Node("ECU-02", "Yassine", "Speed Motor",     "rpm", 5000,  CMD_SPEED,           0x406, 2, 2500,
         "Window Left",    CMD_WINDOW_LEFT,    0x401, COL_SPEED),
    Node("ECU-03", "Souha",   "Temperature",     "°C",  150,   CMD_TEMPERATURE,     0x456, 4, 85,
         "Window Right",   CMD_WINDOW_RIGHT,   0x402, COL_TEMP),
    Node("ECU-04", "Eya",     "Débit Fluide",    "L/s", 255,   CMD_DEBIT,           0x390, 1, 90,
         "Door Right",     CMD_PORTE_DROIT,    0x790, COL_DEBIT),
    Node("ECU-05", "Nour",    "Battery Voltage", "mV",  15000, CMD_BATTERY_VOLTAGE, 0x222, 2, 12600,
         "External Light", CMD_EXTERNAL_LIGHT, 0x221, COL_BATTERY),
]

# cmd -> (centre, amplitude, phase) of the auto-sweep waveform
SWEEP = {
    CMD_PRESSURE        : (120,   100,  0.0),   # 20..220 bar   -> PRES warning
    CMD_SPEED           : (2500,  2000, 1.0),   # 500..4500 rpm
    CMD_TEMPERATURE     : (85,    35,   2.0),   # 50..120 °C    -> OVHT warning
    CMD_DEBIT           : (90,    80,   3.5),   # 10..170 L/s   -> FLOW warning
    CMD_BATTERY_VOLTAGE : (12600, 1200, 5.0),   # 11.4..13.8 V
}


class VirtualBus:
    """The shared CAN bus with five nodes on it, seen from ECU-01.

    Every frame put on the bus is reported through ``notify`` and, while a
    dashboard is connected, relayed over UART exactly as ECU-01's pass-through
    CAN filter does. ``notify(kind, a, b, c)`` is called from worker threads:

        "can"      can_id, cmd, value     a frame went onto the bus
        "uart_tx"  cmd, value, 0          ECU-01 -> dashboard
        "uart_rx"  cmd, value, 0          dashboard -> ECU-01
        "led"      state, 0, 0            ECU-01 LD1 switched by the dashboard
        "link"     state, 0, 0            dashboard connected / disconnected
    """

    def __init__(self, auto=True, random_events=False):
        self.values        = {n.cmd: n.default for n in NODES}
        self.auto          = {n.cmd: auto for n in NODES}
        self.random_events = random_events
        self.led           = False
        self.notify        = lambda kind, a, b, c: None
        self._conn         = None
        self._lock         = threading.Lock()
        self._t0           = time.monotonic()

    def listen(self, callback):
        """Add a listener next to whatever ``notify`` already calls."""
        previous = self.notify

        def both(kind, a, b, c):
            previous(kind, a, b, c)
            callback(kind, a, b, c)
        self.notify = both

    # ── inputs ──────────────────────────────────────────────
    def set_value(self, cmd, value):
        self.values[cmd] = int(value)

    def set_auto(self, cmd, state):
        self.auto[cmd] = state

    def trigger(self, ev_cmd):
        """A node's push button: one asynchronous event frame on the bus."""
        node = next(n for n in NODES if n.ev_cmd == ev_cmd)
        self._can_tx(node.ev_can_id, ev_cmd, ABS_EVENT if ev_cmd == CMD_ACK else 1)

    # ── bus ─────────────────────────────────────────────────
    def start(self, host=HOST, port=PORT):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.bind((host, port))
        srv.listen(1)
        threading.Thread(target=self._serve, args=(srv,), daemon=True).start()
        threading.Thread(target=self._bus_loop, daemon=True).start()

    def _sweep(self, cmd):
        centre, amp, phase = SWEEP[cmd]
        t = (time.monotonic() - self._t0) / SCENARIO_LEN * 2 * math.pi
        noise = random.uniform(-0.02, 0.02) * amp
        return max(0, int(centre + amp * math.sin(t + phase) + noise))

    def _bus_loop(self):
        next_event = time.monotonic() + random.uniform(3, 6)
        while True:
            for node in NODES:
                if self.auto[node.cmd]:
                    self.values[node.cmd] = self._sweep(node.cmd)
                self._can_tx(node.can_id, node.cmd, self.values[node.cmd])
                time.sleep(SENSOR_PERIOD / len(NODES))
            if self.random_events and time.monotonic() >= next_event:
                self.trigger(random.choice(NODES).ev_cmd)
                next_event = time.monotonic() + random.uniform(3, 6)

    def _can_tx(self, can_id, cmd, value):
        self.notify("can", can_id, cmd, value)
        self._uart_tx(cmd, value)

    # ── ECU-01 UART side ────────────────────────────────────
    def _uart_tx(self, cmd, value):
        with self._lock:
            if self._conn is None:
                return
            try:
                self._conn.sendall(struct.pack('>BBI', SOF, cmd, value))
            except OSError:
                return
        self.notify("uart_tx", cmd, value, 0)

    def _on_command(self, cmd, value):
        """Mirror of NewFrameReceivedCallback() in firmware/Core/Src/main.c."""
        self.notify("uart_rx", cmd, value, 0)
        if cmd in (CMD_LED_ON, CMD_LED_OFF):
            self.led = cmd == CMD_LED_ON
            self.notify("led", int(self.led), 0, 0)
            self._uart_tx(CMD_ACK, 0)
        elif cmd == CMD_READ_SENSOR:
            self._uart_tx(CMD_PRESSURE, self.values[CMD_PRESSURE])
        elif cmd == CMD_TRIGGER_ABS:
            # Goes out on the CAN bus and comes back through the pass-through filter
            self.trigger(CMD_ACK)

    def _rx_loop(self, conn):
        buf = bytearray()
        while True:
            data = conn.recv(64)
            if not data:
                return
            buf.extend(data)
            while len(buf) >= FRAME_LEN:
                idx = buf.find(SOF)
                if idx == -1:
                    buf.clear(); break
                del buf[:idx]
                if len(buf) < FRAME_LEN:
                    break
                cmd, value = struct.unpack('>BI', buf[1:FRAME_LEN])
                del buf[:FRAME_LEN]
                self._on_command(cmd, value)

    def _serve(self, srv):
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._conn = conn
            self.notify("link", 1, 0, 0)
            try:
                self._rx_loop(conn)
            except OSError:
                pass
            with self._lock:
                self._conn = None
            conn.close()
            self.notify("link", 0, 0, 0)


if __name__ == "__main__":
    def _print(kind, a, b, c):
        if kind in ("uart_tx", "uart_rx"):
            arrow = "ECU -> PC " if kind == "uart_tx" else "PC  -> ECU"
            print(f"  {arrow}  {RX_NAMES.get(a, f'CMD 0x{a:02X}'):<16} {b}")
        elif kind == "link":
            print("Dashboard connected" if a else "Dashboard disconnected")

    bus = VirtualBus(auto=True, random_events=True)
    bus.notify = _print
    bus.start()
    print(f"Virtual CAN bus running, ECU-01 UART on {URL}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
