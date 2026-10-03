"""Smoke-test non-motion robot commands over the normal GUI serial backend."""

import os
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from BluetoothSerial import BluetoothSerial


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    app = QApplication([])
    link = BluetoothSerial()
    logs = []
    errors = []
    parameter_values = []
    telemetry_samples = 0

    def on_log(level, message):
        logs.append((level, message))

    def on_telemetry(_name, _value, _timestamp):
        nonlocal telemetry_samples
        telemetry_samples += 1

    link.log_received.connect(on_log)
    link.error_received.connect(errors.append)
    link.telemetry_received.connect(on_telemetry)
    link.parameter_value_received.connect(
        lambda name, value: parameter_values.append((name, value)))
    link.connect_port(port, 115200)
    QTimer.singleShot(800, lambda: link.send_command("ping"))
    QTimer.singleShot(1400, lambda: link.send_command("read_all_tof"))
    QTimer.singleShot(1900, lambda: link.set_parameter(
        "debug.telemetry_interval_ms", 500))
    QTimer.singleShot(2200, lambda: link.request_parameter(
        "debug.telemetry_interval_ms"))
    # Safe to exercise even on a live robot: it commands neutral/torque-off.
    QTimer.singleShot(2500, lambda: link.send_command("stop"))

    def finish():
        link.disconnect_port()
        print({"port": port, "telemetry_samples": telemetry_samples,
               "parameter_values": parameter_values,
               "logs": logs, "errors": errors})
        app.quit()

    QTimer.singleShot(3000, finish)
    app.exec()
    if (errors or not any(message == "Pong from Teensy" for _, message in logs)
            or ("debug.telemetry_interval_ms", 500.0) not in parameter_values
            or not any("STOP received" in message for _, message in logs)):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
