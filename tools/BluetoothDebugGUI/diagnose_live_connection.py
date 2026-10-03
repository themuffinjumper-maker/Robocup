"""Manual, read-only COM-port/reconnect diagnostic for the debug GUI.

Run with the GUI environment and pass a port, e.g. COM10. This opens the port,
sends only the normal hello handshake, then disconnects and reconnects once.
No robot motion commands are sent.
"""

import os
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from DebugGUI import RobotDebugGUI


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QApplication([])
    window = RobotDebugGUI()
    index = window.port_combo.findData(port)
    if index < 0:
        raise SystemExit(f"{port} is not in the app's port list")
    window.port_combo.setCurrentIndex(index)
    errors = []
    messages = []
    window.bluetooth.error_received.connect(errors.append)
    window.bluetooth.message_received.connect(lambda message: messages.append(message.get("type")))

    def snapshot(label):
        arena = window.arena_view.model
        free_cells = sum(1 for value in arena.cells.values() if value < 0)
        obstacle_cells = sum(1 for value in arena.cells.values() if value > 0)
        weight_cells = sum(1 for value in arena.weight_votes.values() if value >= 2)
        print(label, "status=", window.connection_status.text(),
              "rows=", window.telemetry_table.rowCount(),
              "frames=", len(messages), "telemetry_in_last_second=",
              len(window.telemetry_event_times),
              "arena=", window.arena_view.status.text()[:100],
              "grid=", {"free": free_cells, "obstacle": obstacle_cells,
                         "possible_weight": weight_cells},
              "errors=", errors[-3:], flush=True)

    QTimer.singleShot(0, window.toggle_connection)
    QTimer.singleShot(3200, lambda: (snapshot("first"), window.toggle_connection()))
    QTimer.singleShot(3900, lambda: (snapshot("disconnected"), window.toggle_connection()))
    QTimer.singleShot(7100, lambda: (snapshot("reconnected"), window.close(), app.quit()))
    app.exec()


if __name__ == "__main__":
    main()
