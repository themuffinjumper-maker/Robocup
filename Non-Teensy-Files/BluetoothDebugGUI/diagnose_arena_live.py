"""Live Arena View diagnostic using the production GUI/model code."""

import os
import sys
from pathlib import Path

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

from DebugGUI import RobotDebugGUI


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
    duration_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 20000
    output = Path(sys.argv[3]) if len(sys.argv) > 3 else Path("tmp/arena_live.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    window = RobotDebugGUI()
    index = window.port_combo.findData(port)
    if index < 0:
        raise SystemExit(f"{port} is not available")
    window.port_combo.setCurrentIndex(index)
    timeline = []

    def sample():
        model = window.arena_view.model
        timeline.append({
            "x": round(model.x, 1), "y": round(model.y, 1),
            "heading": round(__import__("math").degrees(model.theta), 1),
            "encoder_1": window.telemetry.get("encoder.1.count"),
            "encoder_2": window.telemetry.get("encoder.2.count"),
            "imu_heading": window.telemetry.get("imu.heading_deg"),
            "cells": len(model.cells), "rays": len(model.current_rays),
        })

    def finish():
        model = window.arena_view.model
        model.finish_frame()
        sample()
        cells = list(model.cells)
        bounds = None
        if cells:
            bounds = {
                "column": [min(c[0] for c in cells), max(c[0] for c in cells)],
                "row": [min(c[1] for c in cells), max(c[1] for c in cells)],
            }
        view = window.arena_view
        view.resize(1500, 900)
        view.grab().save(str(output))
        print("status:", view.status.text())
        print("calibration:", {
            "encoder_1_mm_per_count": model.encoder_1_mm_per_count,
            "encoder_2_mm_per_count": model.encoder_2_mm_per_count,
            "invert_1": model.invert_left, "invert_2": model.invert_right,
            "wheel_track_mm": model.track_width_mm,
            "cell_size_mm": model.cell_size_mm,
        })
        print("start:", timeline[0] if timeline else None)
        print("end:", timeline[-1] if timeline else None)
        print("cell_bounds:", bounds)
        print("sensors:", [
            {"name": spec["name"], "x": spec["x"], "y": spec["y"],
             "angle": spec["angle"], "layer": spec.get("layer")}
            for spec in model.sensor_specs
        ])
        print("matrix:", model.matrix_spec)
        print("image:", output.resolve())
        window.close()
        app.quit()

    QTimer.singleShot(0, window.toggle_connection)
    timer = QTimer()
    timer.timeout.connect(sample)
    timer.start(500)
    QTimer.singleShot(duration_ms, finish)
    app.exec()


if __name__ == "__main__":
    main()
