"""Read-only live telemetry summary; sends no actuator commands."""

import os
import sys
from collections import Counter

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from BluetoothSerial import BluetoothSerial


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    duration_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 8000
    app = QApplication([])
    link = BluetoothSerial()
    latest = {}
    counts = Counter()
    matrices = []
    errors = []
    heading_samples = []
    encoder_samples = {"encoder.1.count": [], "encoder.2.count": []}
    numeric_samples = {}

    def telemetry(name, value, _timestamp):
        latest[name] = value
        counts[name] += 1
        if name == "imu.heading_deg" and isinstance(value, (int, float)):
            heading_samples.append(float(value))
        if name in encoder_samples and isinstance(value, (int, float)):
            encoder_samples[name].append(float(value))
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            numeric_samples.setdefault(name, []).append(float(value))

    def matrix(message):
        matrices.append(message)

    link.telemetry_received.connect(telemetry)
    link.tof_8x8_received.connect(matrix)
    link.error_received.connect(errors.append)

    def finish():
        link.disconnect_port()
        tof_names = [
            "top_mid_left", "top_mid_right", "bottom_mid_left",
            "bottom_mid_right", "bottom_right_right", "bottom_left_left",
        ]
        print(f"port={port} duration_s={duration_ms / 1000:.1f} errors={errors}")
        print("system:", {key: latest.get(key) for key in (
            "system.uptime_s", "system.debug_mode", "system.stopped")})
        print("imu:", {key: latest.get(key) for key in (
            "imu.available", "imu.valid", "imu.bus", "imu.address",
            "imu.fusion_running", "imu.system_status", "imu.system_error",
            "imu.system_error_active", "imu.operation_mode",
            "imu.self_test_result", "imu.self_test_passed",
            "imu.heading_deg", "imu.roll_deg", "imu.pitch_deg",
            "imu.calibration.system", "imu.calibration.gyro",
            "imu.calibration.accel", "imu.calibration.mag",
            "imu.quaternion.w", "imu.quaternion.x", "imu.quaternion.y",
            "imu.quaternion.z", "imu.linear_accel.x_mps2",
            "imu.linear_accel.y_mps2", "imu.linear_accel.z_mps2",
            "imu.gravity.x_mps2", "imu.gravity.y_mps2",
            "imu.gravity.z_mps2", "imu.temperature_c")})
        print("encoders:", {key: latest.get(key) for key in (
            "encoder.1.available", "encoder.1.count", "encoder.1.delta",
            "encoder.1.counts_per_s", "encoder.2.available",
            "encoder.2.count", "encoder.2.delta", "encoder.2.counts_per_s")})
        if heading_samples:
            unwrapped = [heading_samples[0]]
            for value in heading_samples[1:]:
                delta = (value - unwrapped[-1] + 180) % 360 - 180
                unwrapped.append(unwrapped[-1] + delta)
            print("motion span:", {
                "heading_change_deg": round(unwrapped[-1] - unwrapped[0], 3),
                "heading_range_deg": round(max(unwrapped) - min(unwrapped), 3),
                "encoder_1_change": (encoder_samples["encoder.1.count"][-1] -
                                     encoder_samples["encoder.1.count"][0])
                                    if encoder_samples["encoder.1.count"] else None,
                "encoder_2_change": (encoder_samples["encoder.2.count"][-1] -
                                     encoder_samples["encoder.2.count"][0])
                                    if encoder_samples["encoder.2.count"] else None,
            })
        for name in tof_names:
            print(f"tof.{name}:", {suffix: latest.get(f"tof.{name}.{suffix}")
                  for suffix in ("available", "timed_out", "distance_mm")})
        print("point TOF spans:", {
            name: {
                "min": min(numeric_samples.get(f"tof.{name}.distance_mm", [0])),
                "max": max(numeric_samples.get(f"tof.{name}.distance_mm", [0])),
            } for name in tof_names
        })
        for name in ("a", "b"):
            prefix = f"ultrasound.{name}."
            print(f"ultrasound.{name}:", {
                suffix: latest.get(prefix + suffix) for suffix in (
                    "valid", "timed_out", "distance_mm", "echo_us",
                    "echo_high", "echo_adc", "echo_voltage_v",
                    "ping_min_adc", "ping_max_adc", "ping_min_voltage_v",
                    "ping_max_voltage_v", "trigger_low_adc",
                    "trigger_high_adc", "trigger_high_voltage_v",
                    "trigger_count", "rise_count", "fall_count",
                )
            })
        if matrices:
            last = matrices[-1]
            data = last.get("data") if isinstance(last.get("data"), list) else []
            valid_values = [value for value in data if isinstance(value, (int, float)) and value > 0]
            print("8x8:", {
                "messages": len(matrices), "available": last.get("available"),
                "valid": last.get("valid"), "frame": last.get("frame"),
                "address": last.get("address"), "zones": len(data),
                "minimum_mm": min(valid_values) if valid_values else None,
                "maximum_mm": max(valid_values) if valid_values else None,
            })
        else:
            print("8x8: no messages")
        sampled = [value for value in counts.values() if value]
        print("telemetry:", {"signals": len(latest),
              "minimum_samples_per_signal": min(sampled) if sampled else 0,
              "maximum_samples_per_signal": max(sampled) if sampled else 0})
        app.quit()

    link.connect_port(port, 115200)
    QTimer.singleShot(duration_ms, finish)
    app.exec()


if __name__ == "__main__":
    main()
