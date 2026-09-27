"""Focused, hardware-free checks for pre-laid mission upload and detours."""

import json
import math
import tempfile
import unittest
from pathlib import Path

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

from ArenaView import ArenaView, MissionLayout
from DebugGUI import CommandWidget
from PyQt6.QtWidgets import QPushButton


class MissionUploadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_live_navigation_phase_is_visible_above_arena(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            for phase, expected in (
                ("following_route", "FOLLOWING PATH"),
                ("aligning_in_home", "ALIGNING IN HOME"),
                ("leaving_home_straight", "DRIVING STRAIGHT OUT OF HOME"),
                ("aligning_to_weight", "CENTERING ON WEIGHT"),
                ("reversing_to_center_weight", "BACKING UP TO CENTER WEIGHT"),
                ("checking_outer_weight_cue", "TURNING TO VERIFY OUTER WEIGHT CUE"),
                ("waiting_for_point_tofs", "WAITING FOR POINT TOF RECOVERY"),
                ("reversing_for_weight_retry", "REVERSING FOR WEIGHT RETRY"),
                ("unloading_at_home", "UNLOADING AT HOME"),
            ):
                view.model.latest.update({"mission.active": True,
                                          "mission.phase": phase})
                view._refresh_status()
                self.assertIn(expected, view.mission_phase_label.text())
            view.close()

    def test_point_tof_bank_failure_is_visible_on_arena(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "system.uptime_s": 30.0,
                "system.navigation_controller_version": 25,
                "tof.expander_available": False,
                "tof.recovery_attempts": 2,
            })
            view._refresh_status()
            self.assertIn("POINT TOFS OFFLINE", view.status.text())
            self.assertIn("I2C0 expander", view.status.text())
            self.assertIn("Recovery attempts: 2", view.status.text())
            view.close()

    def test_arena_gate_buttons_send_open_and_close(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view.open_gate_button.click()
            view.close_gate_button.click()
            self.assertEqual(sent, [
                ("storage_release_gate", {"open": True}),
                ("storage_release_gate", {"open": False}),
            ])
            view.close()

    def test_gate_command_card_has_open_and_close(self):
        config = (Path(__file__).resolve().parents[2] / "PlatformIO" /
                  "debug_config.json")
        commands = json.loads(config.read_text(encoding="utf-8"))["commands"]
        definition = next(item for item in commands
                          if item["name"] == "storage_release_gate")
        sent = []
        widget = CommandWidget(definition,
                               lambda name, args: sent.append((name, args)))
        buttons = {button.text(): button for button in widget.findChildren(QPushButton)}
        buttons["OPEN GATE"].click()
        buttons["CLOSE GATE"].click()
        self.assertEqual(sent, [
            ("storage_release_gate", {"open": True}),
            ("storage_release_gate", {"open": False}),
        ])
        widget.close()

    def test_rotated_wall_geometry_is_saved_and_uploaded(self):
        layout = MissionLayout()
        wall = layout.add_obstacle("wall", 2200, 1100)
        self.assertEqual((wall["w"], wall["h"]), (600, 130))
        self.assertTrue(layout.set_obstacle_rotation(wall["id"], 35))
        self.assertTrue(layout.blocked(2200, 1100))
        self.assertFalse(layout.blocked(3000, 1100))
        payload = layout.map_payload()
        self.assertEqual(payload["rotated_walls"], [1, 2200, 1100, 35])
        saved = layout.to_dict()
        restored = MissionLayout()
        restored.load_dict(saved)
        self.assertEqual(restored.obstacles[0]["rotation"], 35)
        self.assertEqual(restored.map_payload(), payload)
        route = layout._astar((1000, 1100), (3400, 1100))
        self.assertIsNotNone(route)
        self.assertTrue(all(layout._segment_clear(a, b)
                            for a, b in zip(route, route[1:])))

    def test_recorded_wall_gap_has_clear_leg_after_missed_transit_point(self):
        # Geometry and poses from the 27 Sep 14:10 run: the robot was near
        # (1066,1261), still chasing (800,1400) after passing it, although
        # the next leg to (1600,1000) went through the wall gap.
        layout = MissionLayout()
        layout.my_home = "blue"
        for x, y in ((1207, 311), (1204, 2089)):
            wall = layout.add_obstacle("wall", x, y)
            layout.set_obstacle_rotation(wall["id"], 90)
        current = (800, 1400)
        following = (1600, 1000)
        pose = (1066, 1261)
        length = math.dist(current, following)
        along = sum((pose[i] - current[i]) * (following[i] - current[i])
                    for i in (0, 1)) / length
        lateral = abs((pose[0] - current[0]) * (following[1] - current[1]) -
                      (pose[1] - current[1]) * (following[0] - current[0])) / length
        self.assertGreater(along, 130)
        self.assertLess(lateral, 240)
        self.assertTrue(layout._segment_clear(pose, following))

    def test_recorded_home_layout_has_diagonal_docking_corridor(self):
        # The 15:21 run had a confirmed pickup but stopped because every
        # horizontal home corridor crossed this upper wall's inflated shape.
        layout = MissionLayout()
        for x, y in ((1207, 311), (1204, 2089)):
            wall = layout.add_obstacle("wall", x, y)
            layout.set_obstacle_rotation(wall["id"], 90)
        route = layout.docking_path((2408, 1491), (325, 325))
        self.assertIsNotNone(route)
        self.assertTrue(all(layout._segment_clear(a, b)
                            for a, b in zip(route, route[1:])))
        self.assertGreater(route[-2][1], 800)

    def test_sensed_weight_marker_persists_after_detection_clears(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            canvas = view.mission_canvas
            canvas.observe_weight_estimate({
                "weight.vector_waypoint_index": 2,
                "weight.vector_hit_x_mm": 2120,
                "weight.vector_hit_y_mm": 840,
                "mission.pose_x_mm": 2000,
                "mission.pose_y_mm": 800,
                "mission.heading_deg": 0,
            })
            canvas.observe_weight_estimate({"weight.detected": False})
            self.assertEqual(canvas.detected_weight_hits[2], (2120, 840, True))
            self.assertIn(2, canvas.detected_weight_local_hits)
            canvas.observe_weight_estimate({
                "weight.detected": True,
                "weight.bearing_fresh": True,
                "weight.bearing_waypoint_index": 3,
                "mission.waypoint_index": 3,
                "weight.bearing_hit_x_mm": 2800,
                "weight.bearing_hit_y_mm": 940,
            })
            self.assertEqual(canvas.detected_weight_hits[3], (2800, 940, False))
            self.assertIn(2, canvas.detected_weight_hits)
            view.close()

    def test_map_contains_forbidden_home_and_physical_landmarks(self):
        layout = MissionLayout()
        layout.add_obstacle("wall", 1800, 1000)
        layout.add_weight(2600, 1200, dummy=True)
        features = layout.map_payload()["features"]
        self.assertEqual(len(features), 15)
        self.assertEqual(features[:5], [0, 1750, 650, 2400, 0])
        self.assertEqual(features[9], 1)
        self.assertEqual(features[14], 2)
        self.assertEqual(layout.map_payload()["dummy_extra_mm"], 150)
        self.assertTrue(layout.blocked(3150, 1200))
        self.assertFalse(layout.blocked(3300, 1200))
        layout.dummy_extra_mm = 50
        self.assertFalse(layout.blocked(3150, 1200))
        restored = MissionLayout()
        restored.load_dict(layout.to_dict())
        self.assertEqual(restored.dummy_extra_mm, 50)

    def test_weight_route_has_straight_staging_and_run_through(self):
        layout = MissionLayout()
        layout.add_weight(2200, 1200)
        self.assertTrue(layout.plan(), layout.route_error)
        self.assertTrue(layout.route[-1]["target"])
        stage = layout.route[-2]
        target = layout.route[-1]
        dx, dy = target["x"] - stage["x"], target["y"] - stage["y"]
        distance = math.hypot(dx, dy)
        self.assertGreaterEqual(distance, 300)
        end = (target["x"] + 200 * dx / distance,
               target["y"] + 200 * dy / distance)
        self.assertTrue(layout._segment_clear(
            (target["x"], target["y"]), end))

    def test_weight_approach_prefers_clear_side_over_short_side(self):
        layout = MissionLayout()
        layout.start.update(x=1700.0, y=1200.0)
        layout.add_weight(2200, 1200)
        # The direct approach from the start points through the weight at a
        # nearby wall. The opposite/side approach has room for collection.
        layout.add_obstacle("wall", 3000, 1200)
        self.assertTrue(layout.plan(), layout.route_error)
        stage, target = layout.route[-2:]
        dx = target["x"] - stage["x"]
        dy = target["y"] - stage["y"]
        distance = math.hypot(dx, dy)
        self.assertGreaterEqual(distance, 300)
        self.assertTrue(layout._segment_clear(
            (target["x"], target["y"]),
            (target["x"] + 500 * dx / distance,
             target["y"] + 500 * dy / distance)))

    def test_run_uploads_map_then_route_then_start(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.navigation_strategy.setCurrentIndex(3)
            view.mission_layout.add_weight(1500, 900)
            self.assertTrue(view.mission_layout.plan())
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._run_navigation_with_tuning()
            self.assertEqual([name for name, _ in sent],
                             ["mission_map_set", "mission_plan_set",
                              "autonomous_navigation"])
            self.assertEqual(sent[1][1]["home_x_mm"], 325)
            self.assertEqual(sent[1][1]["home_y_mm"], 325)
            for name, data in sent[:2]:
                encoded = json.dumps({"type": "command", "command": name, **data},
                                     separators=(",", ":")).encode("utf-8")
                self.assertLess(len(encoded), 760)
            view.close()

    def test_wall_consensus_tuning_is_sent_to_firmware(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            requested = []
            view.parameter_requested.connect(
                lambda name, value: requested.append((name, value)))
            view.navigation_tuning_controls["matrix_wall_mm"].setValue(375)
            view._apply_navigation_tuning()
            self.assertIn(("navigation.matrix_wall_mm", 375), requested)
            view.close()

    def test_return_home_sends_clear_route_and_stop_at_home_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "mission.pose_x_mm": 1000.0,
                "mission.pose_y_mm": 1000.0,
                "mission.heading_deg": 0.0,
                "imu.valid": True,
                "system.stopped": False,
            })
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._return_home()
            self.assertEqual([name for name, _ in sent], [
                "autonomous_navigation", "mission_map_set",
                "mission_plan_set", "autonomous_navigation"])
            self.assertFalse(sent[0][1]["enabled"])
            self.assertTrue(sent[2][1]["return_home"])
            self.assertTrue(sent[-1][1]["enabled"])
            self.assertEqual(sent[2][1]["points"][-1], 5)
            points = sent[2][1]["points"]
            self.assertGreaterEqual(len(points), 6)
            stage = (points[-6], points[-5])
            home = (points[-3], points[-2])
            self.assertTrue(view.mission_layout._segment_clear(stage, home))
            self.assertEqual(len(sent[1][1]["bottom_sensors"]), 12)
            self.assertEqual(len(sent[1][1]["ultrasound_offsets"]), 4)
            view.close()

    def test_start_run_orders_sorting_bank_one_and_navigation_and_stops_timer(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.mission_layout.add_weight(1500, 900)
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._start_run()
            self.assertEqual([name for name, _ in sent][-4:], [
                "run", "arm_sorting", "set_dc_motor_203_speed",
                "autonomous_navigation"])
            self.assertEqual(sent[-2][1], {
                "channel_a_percent": 100, "channel_b_percent": 100})
            self.assertTrue(view.run_timer_active)
            view._stop_navigation()
            self.assertFalse(view.run_timer_active)
            self.assertEqual(sent[-1],
                             ("autonomous_navigation", {"enabled": False}))
            view._stop_run()
            self.assertEqual(sent[-1], ("stop", {}))
            view.close()

    def test_travel_legs_remain_collision_free(self):
        layout = MissionLayout()
        layout.add_obstacle("wall", 2450, 1200)
        path = layout._astar((900, 1200), (4000, 1200))
        self.assertIsNotNone(path)
        self.assertGreaterEqual(len(path), 3)
        for first, second in zip(path, path[1:]):
            self.assertTrue(layout._segment_clear(first, second))

    def test_realistic_mission_route_cannot_cut_diagonal_wall_corners(self):
        layout = MissionLayout()
        layout.SAFETY_MARGIN_MM = 40
        layout.start.update(x=288, y=328, heading_deg=0)
        layout.add_obstacle("wall", 933, 729)
        layout.add_obstacle("wall", 318, 723)
        layout.add_obstacle("tube", 1835, 1574)
        layout.add_weight(1851, 1167, dummy=True)
        layout.add_weight(1827, 1996)
        layout.add_weight(624, 1184)
        self.assertTrue(layout.plan(), layout.route_error)
        positions = [(layout.start["x"], layout.start["y"])] + [
            (point["x"], point["y"]) for point in layout.route]
        self.assertTrue(all(layout._segment_clear(a, b)
                            for a, b in zip(positions, positions[1:])))
        command = json.dumps({"type": "command", "command": "mission_plan_set",
                              **layout.command_payload()},
                             separators=(",", ":")).encode("utf-8")
        self.assertLess(len(command), 760)

    def test_return_home_can_use_marked_actual_position_when_odometry_drifted(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "mission.pose_x_mm": 863.0,
                "mission.pose_y_mm": 663.0,
                "mission.heading_deg": 205.0,
                "imu.valid": True,
                "system.stopped": False,
            })
            view.mission_layout.start["x"] = 315.0
            view.mission_layout.start["y"] = 878.0
            view.mission_manual_home_pose.setChecked(True)
            sent = []
            view.command_requested.connect(lambda name, data: sent.append((name, data)))
            view._return_home()
            plans = [data for name, data in sent if name == "mission_plan_set"]
            self.assertEqual(len(plans), 1)
            self.assertEqual((plans[0]["start_x_mm"], plans[0]["start_y_mm"]),
                             (315, 878))
            self.assertIn("marked actual position", view.mission_status.text())
            view.close()

    def test_oversized_map_does_not_start_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.navigation_strategy.setCurrentIndex(3)
            view.mission_layout.add_weight(1500, 900)
            self.assertTrue(view.mission_layout.plan())
            for index in range(24):
                view.mission_layout.add_obstacle("tube", 800 + index * 140, 1800)
            # Keep the already-planned route to exercise upload validation.
            view.mission_layout.route = [{"x": 1500, "y": 900, "target": True}]
            sent = []
            view.command_requested.connect(lambda name, data: sent.append(name))
            view._run_navigation_with_tuning()
            self.assertNotIn("autonomous_navigation", sent)
            self.assertIn("exceeds", view.mission_status.text())
            view.close()

    def test_weight_visibility_distinguishes_target_and_height_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            settings = QSettings(str(Path(directory) / "mission.ini"),
                                 QSettings.Format.IniFormat)
            view = ArenaView(settings)
            view.model.latest.update({
                "weight.detected": False,
                "weight.geometry_valid": True,
                "weight.valid_mask": 1,
                "weight.gap_mask": 1,
                "weight.target_mask": 0,
                "weight.map_wall_mask": 0,
            })
            view._refresh_mission_weight_visibility()
            self.assertIn("does not land near", view.mission_weight_visibility.text())
            view.model.latest.update({"weight.gap_mask": 0, "weight.target_mask": 1})
            view._refresh_mission_weight_visibility()
            self.assertIn("height gap", view.mission_weight_visibility.text())
            view.model.latest.update({
                "weight.detected": True,
                "weight.sector_mask": 4,
                "weight.nearest_mm": 99,
                "weight.direction": 1,
                "weight.bearing_active": True,
            })
            view._refresh_mission_weight_visibility()
            self.assertIn("Steering toward measured hit",
                          view.mission_weight_visibility.text())
            view.model.latest.update({
                "weight.target_commit_active": True,
                "weight.front_target_visible": True,
                "navigation.front_mm": 330,
            })
            view._refresh_mission_weight_visibility()
            self.assertIn("APPROACHING KNOWN WEIGHT",
                          view.mission_weight_visibility.text())
            self.assertIn("100 mm wall limit",
                          view.mission_weight_visibility.text())
            view.model.latest.update({
                "mission.active": True,
                "mission.pose_x_mm": 4135.0,
                "mission.pose_y_mm": 345.0,
                "mission.heading_deg": 347.0,
                "mission.current_waypoint_x_mm": 4590.0,
                "mission.current_waypoint_y_mm": 355.0,
                "weight.bearing_hit_x_mm": 4472.0,
                "weight.bearing_hit_y_mm": 446.0,
            })
            self.assertFalse(view.mission_canvas.grab().isNull())
            view.close()


if __name__ == "__main__":
    unittest.main()
