"""Checks that recordings contain sufficient structured data for full replay."""

import json
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
VISUALISER = HERE.parent / "DataVisualisationAfter"
for path in (HERE, VISUALISER):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from DataRecorder import DataRecorder
from DataVisualiser import DataVisualiser
from PyQt6.QtWidgets import QApplication


class RecordingReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_schema_records_matrix_and_gui_snapshot(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "complete.rdbg"
            recorder = DataRecorder()
            recorder.start(path, metadata={"wiring_guide_json": "[]"})
            recorder.record_telemetry("imu.heading_deg", 42.5, 1000)
            recorder.record_matrix({
                "type": "tof_8x8", "time": 1000, "frame": 7,
                "available": True, "valid": True, "data": [500] * 64,
            })
            recorder.record_ui_snapshot({
                "active_tab": "Arena View", "telemetry": {"imu.heading_deg": 42.5},
            })
            recorder.stop()

            conn = sqlite3.connect(path)
            try:
                self.assertEqual(conn.execute(
                    "SELECT value FROM metadata WHERE key='format_version'"
                ).fetchone()[0], "4")
                matrix = json.loads(conn.execute(
                    "SELECT frame_json FROM matrix_frames"
                ).fetchone()[0])
                snapshot = json.loads(conn.execute(
                    "SELECT snapshot_json FROM ui_snapshots"
                ).fetchone()[0])
                self.assertEqual(matrix["data"], [500] * 64)
                self.assertEqual(snapshot["active_tab"], "Arena View")
            finally:
                conn.close()
            visualiser = DataVisualiser()
            try:
                visualiser.load_recording(path)
                self.assertTrue(any(event[1] == "matrix"
                                    for event in visualiser.replay_events))
                visualiser.replay_slider.setValue(visualiser.replay_slider.maximum())
                self.assertEqual(visualiser.replay_matrix_cells[0][0].text(), "500")
            finally:
                visualiser.close()

    def test_navigation_has_fast_sensing_and_non_stopping_diagnostics(self):
        source = (HERE.parents[1] / "PlatformIO" / "lib" /
                  "BluetoothDebugWorkflow" / "BluetoothDebugWorkflow.cpp").read_text()
        self.assertIn("now - lastRangePollMs_ >= 100U", source)
        self.assertIn("FRONT_AVOID_MM = 300", source)
        self.assertIn('strncmp(tofSensorNames_[i], "top_", 4)', source)
        self.assertIn("Ultrasound A faces robot-left and B faces robot-right", source)
        self.assertIn("NAV_FOLLOW_WALL", source)
        self.assertIn("NAV_LANE_SHIFT", source)
        self.assertIn("WALL_FOLLOW_TARGET_MM = 200", source)
        self.assertIn("LANE_SPACING_MM = 200.0f", source)
        self.assertIn("navigation.remaining_width_mm", source)
        self.assertIn("MATRIX_BROAD_WALL_MM = 550", source)
        self.assertIn("matrixUsableCentreCount >= 20", source)
        self.assertIn("value >= 10 && value <= 3500", source)
        self.assertIn("navigation.matrix_broad_wall", source)
        self.assertIn("navigationTurnSettledSinceMs_", source)
        self.assertIn("pulsePhase < 90U", source)
        self.assertIn("absoluteError > 55.0f", source)
        self.assertIn('system.navigation_controller_version', source)
        self.assertIn('system.navigation_controller_version"] = 6', source)
        self.assertIn('system.max_loop_gap_ms', source)
        workflow_header = (HERE.parents[1] / "PlatformIO" / "lib" /
                           "BluetoothDebugWorkflow" /
                           "BluetoothDebugWorkflow.h").read_text()
        transport = (HERE.parents[1] / "PlatformIO" / "lib" /
                     "RobotDebug" / "RobotDebug.cpp").read_text()
        self.assertIn("bluetoothTxBuffer_[8192]", workflow_header)
        self.assertIn("addMemoryForWrite", source)
        self.assertNotIn("delay(3)", transport)
        self.assertNotIn("stream_.flush()", transport)
        self.assertIn("NAV_ESCAPE_REVERSE", source)
        self.assertIn("U-shaped enclosure detected", source)
        self.assertIn("uTrapLeftClose", source)
        self.assertIn("left >= 350", source)
        self.assertIn("NAV_CLEARANCE_TURN", source)
        self.assertIn("Turn ended facing a wall", source)
        self.assertIn("driveOnHeading(navigationHeadingReferenceDeg_, 100)", source)
        self.assertNotIn("SWEEP_LANE_COUNT", source)
        self.assertIn("navigationHeadingReferenceDeg_ +", source)
        self.assertIn("beginForwardLeg(navigationTargetHeadingDeg_)", source)
        self.assertIn('data["navigation.motion_consistent"]', source)
        self.assertNotIn("tripNavigationWatchdog", source)
        self.assertNotIn("navigationWatchdogFault_", source)

    def test_arena_canvas_has_no_assumed_outline(self):
        source = (HERE / "ArenaView.py").read_text(encoding="utf-8")
        self.assertIn("DISCOVERED MAP", source)
        self.assertNotIn("p.drawRect(bounds)", source)

    def test_visualiser_has_full_run_replay_and_end_view(self):
        visualiser = DataVisualiser()
        try:
            top_tabs = [visualiser.tabs.tabText(i) for i in range(visualiser.tabs.count())]
            replay_tabs = [visualiser.replay_tabs.tabText(i)
                           for i in range(visualiser.replay_tabs.count())]
            self.assertIn("Run Replay", top_tabs)
            for expected in ("Dashboard", "8x8 TOF", "Arena View", "Plots",
                             "Parameters", "Commands", "Wiring Guide", "Logs",
                             "Raw Serial", "End View"):
                self.assertIn(expected, replay_tabs)
        finally:
            visualiser.close()


if __name__ == "__main__":
    unittest.main()
