"""UART frame reader for the STM32 dashboard protocol."""

import struct
import serial
from PyQt5.QtCore import QThread, pyqtSignal

if __package__:
    from .config import FRAME_LEN, SOF
else:
    from config import FRAME_LEN, SOF

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
