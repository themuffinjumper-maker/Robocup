"""Synthetic checks for local mapping; no robot or COM port is required."""

import math
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from PyQt6.QtCore import QPointF, QSettings, Qt
from PyQt6.QtWidgets import QApplication, QPushButton

from ArenaView import ArenaModel, ArenaView
from DebugGUI import CommandWidget, RobotDebugGUI


class ArenaModelTests(unittest.TestCase):
    def setUp(self):
        self.map = ArenaModel()
        self.map.mm_per_count = 1
        self.map.track_width_mm = 100

    def feed(self, timestamp, left, right, heading=0, distance=None):
        frame = {
            "encoder.1.count": left,
            "encoder.2.count": right,
            "imu.available": True,
            "imu.valid": True,
            "imu.heading_deg": heading,
            "imu.calibration.system": 3,
            "tof.front.available": distance is not None,
            "tof.front.timed_out": False,
        }
        if distance is not None:
            frame["tof.front.distance_mm"] = distance
        for key, value in frame.items():
            self.map.receive_telemetry(key, value, timestamp)
        self.map.finish_frame()

    def test_forward_and_front_obstacle(self):
        self.feed(0, 0, 0)
        self.feed(200, 100, 100, distance=500)
        self.assertAlmostEqual(self.map.x, 0, places=5)
        self.assertAlmostEqual(self.map.y, 100, places=5)
        self.assertAlmostEqual(self.map.points[-1][1], 600, places=5)

    def test_separate_encoder_scales_and_forward_polarity(self):
        self.map.encoder_1_mm_per_count = 0.09
        self.map.encoder_2_mm_per_count = 0.10
        self.map.invert_left = True
        self.feed(0, 0, 0)
        self.feed(200, -100, 100)
        self.assertAlmostEqual(self.map.x, 0, places=5)
        self.assertAlmostEqual(self.map.y, 9.5, places=5)
        self.assertAlmostEqual(self.map.distance_travelled_mm, 9.5, places=5)

    def test_heading_wrap(self):
        self.feed(0, 0, 0)
        self.feed(200, 0, 0, heading=359)
        self.feed(400, 100, 100, heading=1)
        self.assertAlmostEqual(math.degrees(self.map.theta), 88, places=5)
        self.assertGreater(self.map.y, 99)

    def test_sparse_turn_uses_exact_arc_integration(self):
        self.feed(0, 0, 0, heading=0)
        self.feed(200, 0, 0, heading=0)
        self.feed(400, 100, 0, heading=90)
        expected = 100.0 / math.pi
        self.assertAlmostEqual(self.map.x, expected, places=4)
        self.assertAlmostEqual(self.map.y, expected, places=4)

    def test_commanded_point_turn_does_not_create_map_translation(self):
        self.map._integrate_pose({
            "encoder.1.count": 0, "encoder.2.count": 0,
            "imu.available": True, "imu.valid": True,
            "imu.heading_deg": 0,
        })
        self.map._integrate_pose({
            "encoder.1.count": 0, "encoder.2.count": 0,
            "imu.available": True, "imu.valid": True,
            "imu.heading_deg": 0,
        })
        self.map._integrate_pose({
            "encoder.1.count": 120, "encoder.2.count": -80,
            "imu.available": True, "imu.valid": True,
            "imu.heading_deg": 25,
            "dc_motor_203_second.channel_a_percent": -80,
            "dc_motor_203_second.channel_b_percent": 80,
        })
        self.assertAlmostEqual(self.map.x, 0)
        self.assertAlmostEqual(self.map.y, 0)
        self.assertTrue(self.map.wheel_slip_detected)
        self.assertNotAlmostEqual(self.map.theta, math.pi / 2)

    def test_forward_wheelspin_at_close_wall_does_not_move_map(self):
        self.map._integrate_pose({
            "encoder.1.count": 0, "encoder.2.count": 0,
            "imu.available": True, "imu.valid": True,
            "imu.heading_deg": 0,
        })
        self.map._integrate_pose({
            "encoder.1.count": 100, "encoder.2.count": 100,
            "imu.available": True, "imu.valid": True,
            "imu.heading_deg": 0,
            "dc_motor_203_second.channel_a_percent": -90,
            "dc_motor_203_second.channel_b_percent": -90,
            "navigation.front_mm": 180,
        })
        self.assertAlmostEqual(self.map.x, 0)
        self.assertAlmostEqual(self.map.y, 0)
        self.assertEqual(self.map.distance_travelled_mm, 0)
        self.assertTrue(self.map.wheel_slip_detected)

    def test_running_fusion_is_usable_when_system_calibration_is_zero(self):
        # The first encoder frame establishes the odometry baseline; the next
        # two establish and then change the absolute IMU heading.
        for timestamp, heading, count in ((0, 330, 0), (200, 350, 0), (400, 10, 10)):
            frame = {
                "encoder.1.count": count,
                "encoder.2.count": count,
                "imu.available": True,
                "imu.valid": True,
                "imu.fusion_running": True,
                "imu.system_status": 5,
                "imu.system_error": 0,
                "imu.heading_deg": heading,
                "imu.calibration.system": 0,
            }
            for key, value in frame.items():
                self.map.receive_telemetry(key, value, timestamp)
            self.map.finish_frame()
        self.assertAlmostEqual(math.degrees(self.map.theta), 70, places=5)
        self.assertEqual(self.map.last_source, "IMU heading + encoder distance")

    def test_stationary_encoders_suppress_imu_yaw_drift(self):
        self.feed(0, 0, 0, heading=10)
        self.feed(200, 0, 0, heading=20)
        self.feed(400, 0, 0, heading=35)
        self.assertAlmostEqual(math.degrees(self.map.theta), 90, places=5)
        self.assertIn("Stationary", self.map.last_source)

    def test_single_tof_range_jump_is_not_painted(self):
        self.feed(0, 0, 0, distance=500)
        self.feed(200, 0, 0, distance=510)
        previous_points = len(self.map.points)
        self.feed(400, 0, 0, distance=1800)
        self.assertEqual(len(self.map.points), previous_points)

    def test_invalid_point_tof_sentinels_do_not_poison_filter(self):
        self.feed(0, 0, 0, distance=500)
        self.feed(200, 0, 0, distance=510)
        previous_points = len(self.map.points)
        self.feed(400, 0, 0, distance=8191)
        self.feed(600, 0, 0, distance=0)
        self.assertEqual(len(self.map.points), previous_points)
        self.feed(800, 0, 0, distance=520)
        self.assertEqual(len(self.map.points), previous_points + 1)

    def test_invalid_matrix_is_not_plotted(self):
        self.map.receive_matrix({"available": True, "valid": True,
                                 "frame": 1, "data": [0] * 64})
        self.assertEqual(len(self.map.points), 0)
        values = [0] * 64
        values[2 * 8 + 3] = 800
        values[3 * 8 + 3] = 810
        values[4 * 8 + 3] = 820
        for frame in (2, 3):
            self.map.receive_matrix({"available": True, "valid": True,
                                     "frame": frame, "data": values})
        self.assertEqual(len(self.map.points), 1)
        self.assertEqual(len(self.map.current_rays), 3)

    def test_full_matrix_projects_all_64_zones_as_a_cone(self):
        for frame in (1, 2):
            self.map.receive_matrix({"available": True, "valid": True,
                                     "frame": frame, "data": [1000] * 64})
        self.assertEqual(len(self.map.points), 8)
        self.assertEqual(len(self.map.current_rays), 24)
        horizontal = [point[0] for point in self.map.points]
        forward = [point[1] for point in self.map.points]
        self.assertGreater(max(horizontal) - min(horizontal), 800)
        self.assertLess(max(forward) - min(forward), 1e-6)
        self.assertAlmostEqual(forward[0], 1150.0, places=5)
        self.assertGreater(len(self.map.cells), 3000)
        self.assertLess(self.map.cells.get(self.map._cell(0, 700), 0), 0)
        self.assertNotIn(self.map._cell(800, 700), self.map.cells)

    def test_uncalibrated_heading_uses_encoder_fallback(self):
        self.feed(0, 0, 0)
        self.map._integrate_pose({"encoder.1.count": 0, "encoder.2.count": 100,
                                  "imu.available": False, "imu.valid": False})
        self.assertAlmostEqual(self.map.theta, math.pi / 2 + 1)

    def test_default_matrix_orientation_is_robot_perspective(self):
        values = [0] * 64
        for row in (2, 3, 4):
            values[row * 8] = 500  # raw column zero appears on robot-right
        for frame in (1, 2):
            self.map.receive_matrix({"available": True, "valid": True,
                                     "frame": frame, "data": values})
        x, _y, source = self.map.points[-1]
        self.assertEqual(source, "8x8")
        self.assertGreater(x, 0)

    def test_occupied_endpoint_and_free_squares(self):
        self.feed(0, 0, 0)
        self.feed(200, 0, 0, distance=500)
        endpoint = self.map._cell(0, 500)
        self.assertGreater(self.map.cells[endpoint], 0)
        self.assertLess(self.map.cells[self.map._cell(0, 200)], 0)

    def test_repeated_top_returns_form_one_persistent_wall(self):
        self.map.sensor_specs[0]["layer"] = "top"
        for index, distance in enumerate((500, 505, 495, 502), start=1):
            self.feed(index * 200, 0, 0, distance=distance)
        confirmed = [wall for wall in self.map.wall_tracks
                     if wall["observations"] >= 3]
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0]["orientation"], "horizontal")
        self.assertAlmostEqual(confirmed[0]["coordinate"], 500, delta=10)

    def test_three_independent_frames_lock_wall_permanently(self):
        for frame in (1, 2, 3):
            self.map._merge_internal_wall_observation(
                "horizontal", 500 + frame, frame * 20, "8x8",
                observations=2,
                verification_tokens={("matrix", frame)},
            )
        self.assertEqual(len(self.map.internal_wall_tracks), 1)
        wall = self.map.internal_wall_tracks[0]
        self.assertTrue(wall["locked"])
        fixed_coordinate = wall["coordinate"]
        self.map._merge_internal_wall_observation(
            "horizontal", fixed_coordinate + 100, 300, "8x8",
            observations=2,
            verification_tokens={("matrix", 4)},
        )
        self.assertEqual(wall["coordinate"], fixed_coordinate)
        self.assertGreaterEqual(wall["maximum"], 300)

    def test_locked_verified_wall_survives_false_odometry_crossing(self):
        wall = self.map._make_wall_track(
            "horizontal", 500, 0, observations=8,
            sources={"8x8"}, kind="internal",
            verification_tokens={("matrix", 1), ("matrix", 2), ("matrix", 3)},
        )
        wall["minimum"] = -800
        wall["maximum"] = 800
        self.assertTrue(wall["locked"])
        self.map.internal_wall_tracks = [wall]
        self.map._clear_traversed_wall_geometry(0, 0, 0, 1000)
        self.assertEqual(self.map.internal_wall_tracks, [wall])
        self.assertEqual((wall["minimum"], wall["maximum"]), (-800, 800))

    def test_provisional_wall_can_still_be_carved_by_confirmed_traversal(self):
        wall = self.map._make_wall_track(
            "horizontal", 500, 0, observations=4,
            sources={"top"}, kind="internal",
            verification_tokens={("telemetry", 1), ("telemetry", 2)},
        )
        wall["minimum"] = -800
        wall["maximum"] = 800
        self.assertFalse(wall["locked"])
        self.map.internal_wall_tracks = [wall]
        self.map._clear_traversed_wall_geometry(0, 0, 0, 1000)
        self.assertEqual(len(self.map.internal_wall_tracks), 2)
        self.assertTrue(all(piece.get("boundary_blocked")
                            for piece in self.map.internal_wall_tracks))

    def test_bottom_sensor_never_becomes_a_wall(self):
        self.map.sensor_specs = [
            {"name": "lower", "angle": 0, "x": 0, "y": 0,
             "port": "XSHUT5", "layer": "bottom"},
        ]
        for index in range(4):
            self.map._frame = {
                "encoder.1.count": 0, "encoder.2.count": 0,
                "tof.lower.available": True,
                "tof.lower.timed_out": False,
                "tof.lower.distance_mm": 500,
            }
            self.map.finish_frame()
        self.assertFalse(self.map.wall_tracks)

    def test_known_boundary_cannot_spawn_parallel_duplicate_walls(self):
        self.map.theta = math.pi / 2
        for distance in (500, 505, 495, 900, 1300):
            self.map._add_observation(
                distance, 0, "top", wall_candidate=True
            )
        self.assertEqual(len(self.map.wall_tracks), 1)
        self.assertEqual(self.map.wall_tracks[0]["boundary"], "north")
        self.assertAlmostEqual(self.map.wall_tracks[0]["coordinate"], 500,
                               delta=10)

    def test_four_confirmed_walls_join_at_discovered_corners(self):
        self.map.wall_tracks = [
            {"orientation": "horizontal", "coordinate": 1200,
             "minimum": -100, "maximum": 100, "observations": 3},
            {"orientation": "horizontal", "coordinate": -1200,
             "minimum": -100, "maximum": 100, "observations": 3},
            {"orientation": "vertical", "coordinate": -2450,
             "minimum": -100, "maximum": 100, "observations": 3},
            {"orientation": "vertical", "coordinate": 2450,
             "minimum": -100, "maximum": 100, "observations": 3},
        ]
        self.assertEqual(self.map.wall_segment(self.map.wall_tracks[0]),
                         (-2450, 2450))
        self.assertEqual(self.map.wall_segment(self.map.wall_tracks[2]),
                         (-1200, 1200))

    def test_every_configured_point_sensor_is_used(self):
        self.map.sensor_specs = [
            {"name": "s1", "angle": 0, "x": -100, "y": 100},
            {"name": "s2", "angle": 0, "x": 100, "y": 100},
            {"name": "s3", "angle": 0, "x": -100, "y": -100},
            {"name": "s4", "angle": 0, "x": 100, "y": -100},
            {"name": "s5", "angle": -45, "x": 180, "y": -100},
        ]
        frame = {"encoder.1.count": 0, "encoder.2.count": 0}
        for spec in self.map.sensor_specs:
            name = spec["name"]
            frame[f"tof.{name}.available"] = True
            frame[f"tof.{name}.timed_out"] = False
            frame[f"tof.{name}.distance_mm"] = 500
        self.map._frame = frame
        self.map.finish_frame()
        self.assertEqual({p[2] for p in self.map.points}, {f"s{i}" for i in range(1, 6)})

    def test_unavailable_sensor_does_not_claim_free_space(self):
        self.map._frame = {"encoder.1.count": 0, "encoder.2.count": 0,
                           "tof.front.available": False,
                           "tof.front.distance_mm": 500}
        self.map.finish_frame()
        self.assertFalse(self.map.cells)

    def test_firmware_contains_all_six_wiring_ports(self):
        config = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        sensors = json.loads(config.read_text(encoding="utf-8"))["tof_sensors"]
        self.assertEqual({sensor["port"]: sensor["type"] for sensor in sensors}, {
            "XSHUT1": "long", "XSHUT2": "long", "XSHUT5": "short",
            "XSHUT0": "short", "XSHUT3": "short", "XSHUT4": "short",
        })

    def test_centimetre_cells(self):
        self.assertEqual(self.map.cell_size_mm, 10)
        self.assertEqual(self.map._cell(99, 99), (9, 9))

    def test_purple_candidate_needs_repeated_paired_difference(self):
        self.map.sensor_specs = [
            {"name": "upper", "angle": 0, "x": 0, "y": 0, "layer": "top"},
            {"name": "lower", "angle": 0, "x": 0, "y": 0, "layer": "bottom"},
        ]
        frame = {"encoder.1.count": 0, "encoder.2.count": 0}
        for name, distance in (("upper", 1000), ("lower", 500)):
            frame[f"tof.{name}.available"] = True
            frame[f"tof.{name}.timed_out"] = False
            frame[f"tof.{name}.distance_mm"] = distance
        self.map._frame = frame.copy()
        self.map.finish_frame()
        cell = self.map._cell(0, 500)
        self.assertEqual(self.map.weight_votes[cell], 1)
        self.map._frame = frame.copy()
        self.map.finish_frame()
        self.assertEqual(self.map.weight_votes[cell], 2)
        frame["tof.upper.distance_mm"] = 520
        self.map._frame = frame.copy()
        self.map.finish_frame()
        # A large range step must repeat before replacing the accepted value.
        self.map._frame = frame.copy()
        self.map.finish_frame()
        self.assertNotIn(cell, self.map.weight_votes)

    def test_no_purple_without_valid_top_comparison(self):
        self.map.sensor_specs = [
            {"name": "upper", "angle": 0, "x": 0, "y": 0, "layer": "top"},
            {"name": "lower", "angle": 0, "x": 0, "y": 0, "layer": "bottom"},
        ]
        self.map._frame = {"encoder.1.count": 0, "encoder.2.count": 0,
                           "tof.upper.available": False,
                           "tof.lower.available": True,
                           "tof.lower.timed_out": False,
                           "tof.lower.distance_mm": 500}
        self.map.finish_frame()
        self.assertFalse(self.map.weight_votes)

    def test_recent_top_matrix_can_compare_unpaired_bottom_sensor(self):
        angle = -self.map.matrix_fov_deg * 0.5 / 7
        self.map.sensor_specs = [
            {"name": "lower", "angle": angle, "x": 0, "y": 150, "layer": "bottom"},
        ]
        values = [0] * 64
        values[2 * 8 + 3] = 1000
        values[3 * 8 + 3] = 1000
        values[4 * 8 + 3] = 1000
        for frame_number in (1, 2, 3):
            self.map.receive_matrix({"available": True, "valid": True,
                                     "frame": frame_number, "time": frame_number * 200,
                                     "data": values})
            telemetry_time = frame_number * 200 + 100
            for name, value in {
                "encoder.1.count": 0, "encoder.2.count": 0,
                "tof.lower.available": True, "tof.lower.timed_out": False,
                "tof.lower.distance_mm": 500,
            }.items():
                self.map.receive_telemetry(name, value, telemetry_time)
            self.map.finish_frame()
        self.assertTrue(any(v >= 2 for v in self.map.weight_votes.values()))


class SensorLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_graphical_position_and_layer_persist(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "layout.ini"), QSettings.Format.IniFormat)
            view = ArenaView(settings)
            self.assertEqual(view.grid_size.value(), 10)
            self.assertEqual(len(view.sensor_canvas.specs), 7)
            view.sensor_canvas.resize(340, 340)
            spec = next(s for s in view.model.sensor_specs if s["name"] == "bottom_mid_left")
            view.sensor_controls[spec["name"]]["layer"].setCurrentText("top")
            view.sensor_canvas.geometry_changed.emit(spec["name"], "x", -135.0)
            settings.sync()
            self.assertEqual(spec["layer"], "top")
            self.assertEqual(spec["x"], -135.0)
            restored = ArenaView(settings)
            again = next(s for s in restored.model.sensor_specs if s["name"] == spec["name"])
            self.assertEqual((again["layer"], again["x"]), ("top", -135.0))
            view.close()
            restored.close()

    def test_arena_controls_emit_existing_robot_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "controls.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            sent = []
            view.command_requested.connect(
                lambda name, arguments: sent.append((name, arguments))
            )
            view.run_motors_button.click()
            view.run_navigation_button.click()
            view.stop_navigation_button.click()
            view.stop_motors_button.click()
            self.assertEqual(sent, [
                ("run", {}),
                ("autonomous_navigation", {"enabled": True}),
                ("autonomous_navigation", {"enabled": False}),
                ("stop", {}),
            ])
            view.close()

    def test_raw_tof_panel_shows_point_and_complete_matrix_values(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "raw.ini"), QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.receive_telemetry("tof.top_mid_left.available", True, 100)
            view.receive_telemetry("tof.top_mid_left.distance_mm", 8191, 100)
            self.assertIn("8191 mm", view.raw_point_labels["top_mid_left"].text())
            self.assertIn("invalid", view.raw_point_labels["top_mid_left"].text())
            values = list(range(200, 264))
            view.receive_matrix({"available": True, "valid": True, "frame": 7,
                                 "bus": "I2C1", "address": 0x33, "data": values})
            self.assertEqual(view.raw_matrix_cells[0][0].text(), "200")
            self.assertEqual(view.raw_matrix_cells[7][7].text(), "263")
            self.assertIn("0x33", view.raw_matrix_status.text())
            view.close()

    def test_arrow_tip_drag_changes_pointing_angle(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "layout.ini"), QSettings.Format.IniFormat)
            view = ArenaView(settings)
            canvas = view.sensor_canvas
            canvas.resize(340, 340)
            spec = view.model.sensor_specs[0]
            canvas.selected_name = spec["name"]
            tip = canvas._tip(spec)
            marker = canvas._marker(spec)

            class Event:
                def __init__(self, position):
                    self._position = position

                def position(self):
                    return self._position

                def button(self):
                    return Qt.MouseButton.LeftButton

            canvas.mousePressEvent(Event(tip))
            canvas.mouseMoveEvent(Event(QPointF(marker.x() - 40, marker.y())))
            canvas.mouseReleaseEvent(Event(tip))
            self.assertAlmostEqual(spec["angle"], 90, places=1)
            view.close()

    def test_old_robot_telemetry_is_identified_as_firmware_mismatch(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "layout.ini"), QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest = {"tof.front.available": True, "tof.left.available": True}
            view._refresh_status()
            self.assertIn("Firmware mismatch", view.status.text())
            view.close()


class ConnectionStatusTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_open_port_is_not_reported_as_live_robot(self):
        with patch("DebugGUI.BluetoothSerial.available_ports", return_value=[]):
            window = RobotDebugGUI()
        try:
            with patch.object(window.bluetooth, "is_connected", return_value=True):
                window.bluetooth.port = "COM10"
                window.on_connection_changed(True, "COM10")
                self.assertIn("WAITING FOR ROBOT", window.connection_status.text())
                window.connection_started_monotonic = time.monotonic() - 4
                window.update_link_health()
                self.assertIn("NO ROBOT DATA", window.connection_status.text())

                window.on_telemetry("system.uptime_s", 10, 10)
                self.assertIn("CONNECTED", window.connection_status.text())
                window.on_telemetry("tof.front.available", True, 10)
                window.on_telemetry("tof.left.available", True, 10)
                self.assertTrue(window.firmware_warning_label.isVisibleTo(window))
                window.on_telemetry("imu.available", True, 10)
                self.assertFalse(window.firmware_warning_label.isVisibleTo(window))

                window.last_telemetry_monotonic = time.monotonic() - 4
                window.update_link_health()
                self.assertIn("ROBOT SILENT", window.connection_status.text())
        finally:
            window.close()

    def test_keyboard_drive_uses_bank_two_robot_direction_mapping(self):
        with patch("DebugGUI.BluetoothSerial.available_ports", return_value=[]):
            window = RobotDebugGUI()
        try:
            window.keyboard_drive_speed.setValue(30)
            cases = {
                Qt.Key.Key_Up: (-30, -30),
                Qt.Key.Key_Down: (30, 30),
                Qt.Key.Key_Left: (30, -30),
                Qt.Key.Key_Right: (-30, 30),
            }
            for key, expected in cases.items():
                window.keyboard_drive_keys = {key}
                self.assertEqual(window._keyboard_drive_values(), expected)
            window.keyboard_drive_keys = {Qt.Key.Key_Up, Qt.Key.Key_Left}
            self.assertEqual(window._keyboard_drive_values(), (0, -30))

            sent = []
            window.robot_debug_mode = True
            window.robot_stopped = False
            window.keyboard_drive_last_output = (0, 0)
            window.keyboard_drive_keys = {Qt.Key.Key_Up}
            with patch.object(window.bluetooth, "is_connected", return_value=True), \
                    patch.object(window.bluetooth, "send_message",
                                 side_effect=lambda packet: sent.append(packet)):
                window._update_keyboard_drive()
                window.keyboard_drive_keys.clear()
                window._update_keyboard_drive()
            self.assertEqual(sent[0], {"type": "drive", "a": -30, "b": -30})
            self.assertEqual(sent[-1], {"type": "drive", "a": 0, "b": 0})
        finally:
            window.close()

    def test_bumper_command_card_has_on_and_off_in_one_block(self):
        config_path = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        commands = json.loads(config_path.read_text(encoding="utf-8"))["commands"]
        definition = next(item for item in commands if item["name"] == "set_bumper_servos")
        sent = []
        widget = CommandWidget(definition, lambda name, arguments: sent.append((name, arguments)))
        buttons = {button.text(): button for button in widget.findChildren(QPushButton)}
        buttons["BUMPERS ON"].click()
        buttons["BUMPERS OFF"].click()
        self.assertEqual(sent, [
            ("set_bumper_servos", {"enabled": True}),
            ("set_bumper_servos", {"enabled": False}),
        ])
        firmware = (Path(__file__).resolve().parents[2] / "PlatformIO" / "lib" /
                    "BluetoothDebugWorkflow" / "BluetoothDebugWorkflow.cpp").read_text(encoding="utf-8")
        self.assertIn('strcmp(action, "hx12k_bumpers")', firmware)
        self.assertIn('enabled ? 0.0f : 130.0f', firmware)
        self.assertIn('enabled ? 130.0f : 0.0f', firmware)
        widget.close()


if __name__ == "__main__":
    unittest.main()
