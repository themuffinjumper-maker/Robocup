"""Repeated non-motion Bluetooth reconnect test using the production backend."""

import os
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from BluetoothSerial import BluetoothSerial


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    cycles = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    app = QApplication([])
    link = BluetoothSerial()
    errors = []
    received = [0] * cycles
    cycle = 0

    link.error_received.connect(errors.append)

    def on_message(_message):
        if 0 <= cycle < cycles:
            received[cycle] += 1

    link.message_received.connect(on_message)

    def connect_next():
        nonlocal cycle
        if cycle >= cycles:
            print({"port": port, "messages_by_cycle": received,
                   "errors": errors})
            app.quit()
            return
        link.connect_port(port, 115200)
        QTimer.singleShot(1600, disconnect_current)

    def disconnect_current():
        nonlocal cycle
        link.disconnect_port()
        cycle += 1
        QTimer.singleShot(350, connect_next)

    QTimer.singleShot(0, connect_next)
    app.exec()
    if errors or any(count == 0 for count in received):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
