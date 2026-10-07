"""Compatibility entry point for the STM32 automotive dashboard.

The implementation lives in focused modules so this file remains a small,
easy-to-find launcher while preserving ``python dashboard/stm32_dashboard.py``.
"""

import sys

from PyQt5.QtWidgets import QApplication

if __package__:
    from .config import *
    from .dashboard_window import STM32Dashboard
    from .protocol import SerialReader
    from .virtual_bench import open_bench
    from .virtual_car3d import open_car3d
    from .virtual_ecu import VirtualBus
    from .widgets import *
else:
    from config import *
    from dashboard_window import STM32Dashboard
    from protocol import SerialReader
    from virtual_bench import open_bench
    from virtual_car3d import open_car3d
    from virtual_ecu import VirtualBus
    from widgets import *

__all__ = ["STM32Dashboard", "SerialReader"]


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = STM32Dashboard()
    win.show()
    # No board needed: --sim runs against the virtual bench, --3d against the
    # virtual car. Both together share one bus.
    sim, car = "--sim" in sys.argv, "--3d" in sys.argv
    if sim or car:
        bus = VirtualBus(auto=False)
        views = []
        if sim:
            views.append(open_bench(bus))
        if car:
            views.append(open_car3d(bus))
        bus.start()
        win.connect_virtual()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
