"""Live, local robot view from encoders, BNO055, TOF and ultrasound telemetry.

This is a visualisation, not a navigation or collision-avoidance controller.
The first received pose defines the origin; no absolute arena localisation exists.
"""

from __future__ import annotations

import math
import json
import re
import heapq
from collections import deque
from pathlib import Path

from PyQt6.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QScrollArea, QTabWidget, QVBoxLayout, QWidget,
)


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


class ArenaModel:
    """Local-frame odometry and an expanding occupancy-evidence map."""

    def __init__(self):
        self.mm_per_count = 0.0
        self.encoder_1_mm_per_count = 0.0
        self.encoder_2_mm_per_count = 0.0
        self.track_width_mm = 0.0
        self.invert_left = False
        self.invert_right = False
        self.front_angle_deg = 0.0
        self.left_angle_deg = 90.0
        self.matrix_fov_deg = 60.0  # user-adjustable estimate, not a sensor spec
        self.matrix_floor_rows = 2
        self.matrix_mirrored = True
        self.sensor_specs = [
            {"name": "front", "angle": 0.0, "x": 0.0, "y": 0.0, "port": "XSHUT1"},
            {"name": "left", "angle": 90.0, "x": 0.0, "y": 0.0, "port": "XSHUT2"},
        ]
        self.ultrasound_specs = []
        self.matrix_spec = {"name": "matrix", "port": "I2C1", "label": "8x8 TOF",
                            "angle": 0.0, "x": 0.0, "y": 150.0, "layer": "top"}
        self.cell_size_mm = 10.0
        self.weight_gap_mm = 150.0
        self.min_range_mm = 10
        self.max_range_mm = 3500.0
        self.reset()

    def reset(self):
        self.grid_generation = getattr(self, "grid_generation", 0) + 1
        self.x = 0.0
        self.y = 0.0
        self.theta = math.pi / 2  # forward is up in the initial local frame
        self._last_counts = None
        self._last_heading = None
        self._last_imu_motion_event_count = None
        self._imu_motion_latched = False
        self.encoder_imu_disagreement = False
        self.wheel_slip_detected = False
        self._frame_time = None
        self._frame = {}
        self.latest = {}
        self.points = deque(maxlen=5000)
        self.cells = {}  # (column,row) -> log-odds-like evidence, negative free
        self.weight_votes = {}  # purple only after repeated paired readings
        self.trail = deque([(0.0, 0.0)], maxlen=1500)
        self.current_rays = []
        self.last_matrix_frame = None
        self.last_matrix_available = None
        self.last_matrix_valid = None
        self.matrix_top_samples = []
        self.matrix_sample_time = None
        self.last_source = "Awaiting telemetry"
        self.last_imu_calibration = None
        self.last_imu_fusion_running = None
        self.last_imu_system_status = None
        self.last_imu_system_error = None
        self.last_imu_system_error_active = None
        self.last_imu_self_test_passed = None
        self.distance_travelled_mm = 0.0
        self.linear_speed_mm_s = 0.0
        self.range_filter_state = {}
        # Stable arena geometry, separate from the noisy occupancy evidence.
        # Each track is an axis-aligned wall repeatedly observed by the top
        # ranging sensors. Tracks are retained for the whole run and extended
        # as the robot sees more of the same wall.
        self.wall_tracks = []
        # Internal geometry is separate from the four outer arena boundaries.
        # Repeated observations can make it persistent, while confirmed chassis
        # motion can later shorten/delete geometry proven to be traversable.
        self.internal_wall_tracks = []
        # Wall geometry is now created from fused evidence rather than directly
        # from a single range endpoint. Pending clusters are deliberately not
        # rendered as walls. Free-space rays and confirmed chassis traversal can
        # weaken/delete them before they ever become persistent geometry.
        self.wall_evidence_clusters = []
        self.wall_evidence_serial = 0
        self.wall_promotions = 0

    def _filtered_distance(self, name, distance, confirm_initial=False):
        """Reject one-frame range jumps, then lightly median-filter accepted data."""
        state = self.range_filter_state.setdefault(name, {
            "accepted": None, "candidate": None, "candidate_count": 0,
            "recent": deque(maxlen=5),
        })
        accepted = state["accepted"]
        if accepted is None and confirm_initial:
            candidate = state["candidate"]
            candidate_limit = max(100.0, abs(distance) * 0.20)
            if candidate is not None and abs(distance - candidate) <= candidate_limit:
                state["candidate_count"] += 1
            else:
                state["candidate"] = distance
                state["candidate_count"] = 1
            if state["candidate_count"] < 3:
                return None
        if accepted is not None:
            jump_limit = max(150.0, abs(accepted) * 0.35)
            if abs(distance - accepted) > jump_limit:
                candidate = state["candidate"]
                candidate_limit = max(100.0, abs(distance) * 0.20)
                if candidate is not None and abs(distance - candidate) <= candidate_limit:
                    state["candidate_count"] += 1
                else:
                    state["candidate"] = distance
                    state["candidate_count"] = 1
                if state["candidate_count"] < 2:
                    return None
                state["recent"].clear()
            else:
                state["candidate"] = None
                state["candidate_count"] = 0
        state["accepted"] = distance
        state["candidate"] = None
        state["candidate_count"] = 0
        state["recent"].append(distance)
        ordered = sorted(state["recent"])
        return ordered[len(ordered) // 2]

    def receive_telemetry(self, name, value, timestamp):
        if timestamp is None:
            return False
        if self._frame_time is not None and timestamp != self._frame_time:
            self.finish_frame()
        self._frame_time = timestamp
        self._frame[name] = value
        return True

    def finish_frame(self):
        if not self._frame:
            return
        frame = self._frame
        self._frame = {}
        self.latest = frame.copy()
        self._integrate_pose(frame)
        self.current_rays = []
        valid_distances = {}
        for spec in self.sensor_specs:
            label = spec["name"]
            if frame.get(f"tof.{label}.available") is not True or frame.get(f"tof.{label}.timed_out") is True:
                continue
            distance = _number(frame.get(f"tof.{label}.distance_mm"))
            # Zero and 8191 are common invalid/sentinel returns from these
            # modules. Reject every out-of-map-range value before it can
            # become the range filter's accepted baseline or a weight vote.
            if (distance is not None and
                    self.min_range_mm <= distance <= self.max_range_mm):
                filtered = self._filtered_distance(
                    label, distance, confirm_initial=spec.get("layer") != "bottom"
                )
                if filtered is not None:
                    valid_distances[label] = filtered
                    # Bottom point TOFs are weight detectors only. Painting
                    # their floor/chassis returns into occupancy creates false
                    # walls even when wall_candidate is false.
                    if spec.get("layer") != "bottom":
                        self._add_observation(
                            filtered, spec["angle"], label, spec["x"], spec["y"],
                            wall_candidate=True
                        )

        # Ultrasound contributes ordinary free/obstacle centreline evidence.
        # Keep it out of the point-TOF top/bottom weight comparison because
        # its acoustic beam is broad and has different geometry.
        for spec in self.ultrasound_specs:
            telemetry_name = spec["telemetry_name"]
            if (frame.get(f"ultrasound.{telemetry_name}.valid") is not True or
                    frame.get(f"ultrasound.{telemetry_name}.timed_out") is True):
                continue
            distance = _number(frame.get(f"ultrasound.{telemetry_name}.distance_mm"))
            if (distance is not None and
                    self.min_range_mm <= distance <= self.max_range_mm):
                filtered = self._filtered_distance(
                    f"ultrasound.{telemetry_name}", distance, confirm_initial=True
                )
                if filtered is not None:
                    self._add_observation(
                        filtered,
                        spec["angle"],
                        f"ultrasound.{telemetry_name}",
                        spec["x"],
                        spec["y"],
                        wall_candidate=True,
                    )

        for bottom in self.sensor_specs:
            if bottom.get("layer") != "bottom" or bottom["name"] not in valid_distances:
                continue
            possible_tops = [top for top in self.sensor_specs
                             if top.get("layer") == "top" and top["name"] in valid_distances
                             and abs(top["x"] - bottom["x"]) <= 150
                             and abs(top["y"] - bottom["y"]) <= 150
                             and abs((top["angle"] - bottom["angle"] + 180) % 360 - 180) <= 15]
            bottom_distance = valid_distances[bottom["name"]]
            if not self.min_range_mm <= bottom_distance <= self.max_range_mm:
                continue
            top_distance = None
            if possible_tops:
                top = min(possible_tops, key=lambda item: abs(item["x"] - bottom["x"]) +
                          abs(item["y"] - bottom["y"]))
                top_distance = valid_distances[top["name"]]
            elif (self.matrix_spec.get("layer") == "top" and
                  abs(self.matrix_spec["x"] - bottom["x"]) <= 0 and
                  self.matrix_top_samples and
                  _number(self._frame_time) is not None and
                  _number(self.matrix_sample_time) is not None and
                  abs(float(self._frame_time) - float(self.matrix_sample_time)) <= 750):
                matching = [(abs((angle - bottom["angle"] + 180) % 360 - 180), distance)
                            for angle, distance in self.matrix_top_samples]
                matching = [item for item in matching if item[0] <= 10]
                if matching:
                    top_distance = min(matching)[1]
            if top_distance is None:
                continue
            endpoint = self._endpoint(bottom_distance, bottom["angle"], bottom["x"], bottom["y"])
            cell = self._cell(*endpoint)
            if top_distance >= bottom_distance + self.weight_gap_mm:
                self.weight_votes[cell] = min(5, self.weight_votes.get(cell, 0) + 1)
            else:
                self.weight_votes.pop(cell, None)

    def _integrate_pose(self, frame):
        left = _number(frame.get("encoder.1.count"))
        right = _number(frame.get("encoder.2.count"))
        if left is None or right is None:
            return
        counts = (left, right)

        motion_event = _number(frame.get("imu.motion_event_count"))
        motion_event = int(motion_event) if motion_event is not None else None
        motion_fields_present = (
            "imu.motion_active" in frame and
            "imu.motion_event_count" in frame
        )

        if self._last_counts is None:
            self._last_counts = counts
            self._last_imu_motion_event_count = motion_event
            self._imu_motion_latched = (
                frame.get("imu.available") is True and
                frame.get("imu.valid") is True and
                frame.get("imu.motion_active") is True
            )
            self.encoder_imu_disagreement = False
            self.last_source = "Encoder baseline acquired"
            return

        dleft = left - self._last_counts[0]
        dright = right - self._last_counts[1]
        # Always consume the encoder sample, including rejected wheel-only
        # movement, so false motion is never back-filled into the map later.
        self._last_counts = counts

        # An encoder-zero command can cause a huge count discontinuity.
        if abs(dleft) > 100000 or abs(dright) > 100000:
            self._imu_motion_latched = False
            self.encoder_imu_disagreement = False
            self.last_source = "Encoder reset detected; pose held"
            return
        if self.invert_left:
            dleft = -dleft
        if self.invert_right:
            dright = -dright

        left_scale = self.encoder_1_mm_per_count or self.mm_per_count
        right_scale = self.encoder_2_mm_per_count or self.mm_per_count
        distance_scale_ready = left_scale > 0 and right_scale > 0
        dleft_mm = dleft * left_scale
        dright_mm = dright * right_scale
        ds = (dleft_mm + dright_mm) * 0.5
        encoder_motion = abs(dleft) + abs(dright) >= 4

        motor_a = _number(frame.get("dc_motor_203_second.channel_a_percent"))
        motor_b = _number(frame.get("dc_motor_203_second.channel_b_percent"))
        commanded_turn_in_place = (
            motor_a is not None and motor_b is not None and
            motor_a * motor_b < 0 and abs(motor_a) >= 50 and abs(motor_b) >= 50
        )
        front_mm = _number(frame.get("navigation.front_mm"))
        pushing_into_wall = (
            motor_a is not None and motor_b is not None and
            motor_a <= -50 and motor_b <= -50 and
            front_mm is not None and 0 < front_mm <= 300
        )

        valid_imu = frame.get("imu.available") is True and frame.get("imu.valid") is True
        heading = _number(frame.get("imu.heading_deg")) if valid_imu else None
        cal = _number(frame.get("imu.calibration.system"))
        self.last_imu_calibration = int(cal) if cal is not None else None
        self.last_imu_fusion_running = frame.get("imu.fusion_running")
        system_status = _number(frame.get("imu.system_status"))
        system_error = _number(frame.get("imu.system_error"))
        self.last_imu_system_status = int(system_status) if system_status is not None else None
        self.last_imu_system_error = int(system_error) if system_error is not None else None
        self.last_imu_system_error_active = frame.get("imu.system_error_active")
        self.last_imu_self_test_passed = frame.get("imu.self_test_passed")
        fusion_field_present = "imu.fusion_running" in frame
        heading_ready = (
            valid_imu and heading is not None and
            (self.last_imu_fusion_running is True or
             (not fusion_field_present and (cal is None or cal >= 1)))
        )

        previous_theta = self.theta
        previous_x, previous_y = self.x, self.y
        clockwise_delta = None
        if heading_ready and self._last_heading is not None:
            # Wrap through 0/360 without a spurious full revolution.
            clockwise_delta = (heading - self._last_heading + 180) % 360 - 180

        event_changed = (
            motion_event is not None and
            self._last_imu_motion_event_count is not None and
            motion_event != self._last_imu_motion_event_count
        )
        if motion_event is not None:
            self._last_imu_motion_event_count = motion_event

        # A heading change is independent IMU evidence of real chassis motion
        # during a turn. Straight translation is confirmed by the firmware's
        # short-lived linear-acceleration event counter. Once a movement bout
        # is confirmed, keep it latched through constant-speed travel because
        # zero acceleration does not mean zero velocity. The latch resets when
        # encoder motion stops, so the next bout must be confirmed again.
        imu_heading_motion = (
            clockwise_delta is not None and abs(clockwise_delta) >= 0.5
        )
        if motion_fields_present:
            if not encoder_motion:
                self._imu_motion_latched = False
            elif valid_imu and (
                    frame.get("imu.motion_active") is True or
                    event_changed or imu_heading_motion):
                self._imu_motion_latched = True
            # Current firmware provides imu.motion_* specifically so encoder
            # translation can be cross-checked. Do not advance the virtual
            # chassis unless the IMU itself is currently valid and has
            # confirmed this encoder-motion bout.
            translation_confirmed = (
                encoder_motion and valid_imu and self._imu_motion_latched
            )
            self.encoder_imu_disagreement = (
                encoder_motion and not translation_confirmed
            )
        else:
            # Backward compatibility for recordings and firmware made before
            # imu.motion_* telemetry existed.
            translation_confirmed = True
            self.encoder_imu_disagreement = False

        # Unequal wheel counts during a commanded point turn are rotation, not
        # translation. Likewise, spinning both wheels forward against a wall
        # cannot move the chassis through the wall even if vibration trips the
        # IMU acceleration latch. Keep BNO055 yaw, but reject map translation.
        self.wheel_slip_detected = bool(
            encoder_motion and (commanded_turn_in_place or pushing_into_wall)
        )
        if self.wheel_slip_detected:
            translation_confirmed = False
            self.encoder_imu_disagreement = True

        if heading_ready:
            if clockwise_delta is not None:
                # The BNO055 magnetometer drifts around the powered chassis.
                # If neither wheel moved, update its baseline but hold map yaw.
                if encoder_motion:
                    self.theta -= math.radians(clockwise_delta)
            self._last_heading = heading
            if self.encoder_imu_disagreement:
                self.last_source = "Encoder movement held: IMU did not confirm chassis motion"
            else:
                self.last_source = (
                    "IMU heading + encoder distance" if encoder_motion
                    else "Stationary: encoder lock suppressing IMU yaw drift"
                )
        elif self.track_width_mm > 0 and distance_scale_ready:
            self._last_heading = None
            if translation_confirmed:
                self.theta += (dright_mm - dleft_mm) / self.track_width_mm
                self.last_source = "Encoder-only heading (IMU uncalibrated/unavailable)"
            else:
                self.last_source = "Encoder movement held: IMU did not confirm chassis motion"
        else:
            self._last_heading = None
            self.last_source = (
                "Encoder movement held: IMU did not confirm chassis motion"
                if self.encoder_imu_disagreement
                else "IMU fusion unavailable; turn mapping paused"
            )

        if distance_scale_ready:
            left_rate = _number(frame.get("encoder.1.counts_per_s"))
            right_rate = _number(frame.get("encoder.2.counts_per_s"))
            if left_rate is not None and right_rate is not None:
                if self.invert_left:
                    left_rate = -left_rate
                if self.invert_right:
                    right_rate = -right_rate
                encoder_speed = (
                    left_rate * left_scale + right_rate * right_scale
                ) * 0.5
                self.linear_speed_mm_s = encoder_speed if translation_confirmed else 0.0
            elif not translation_confirmed:
                self.linear_speed_mm_s = 0.0

            if not translation_confirmed:
                return

            self.distance_travelled_mm += abs(ds)
            # Integrate the exact constant-curvature arc between telemetry
            # frames. The old midpoint approximation badly displaced the map
            # during turns when Bluetooth delivered sparse pose samples.
            dtheta = self.theta - previous_theta
            if abs(dtheta) < 1e-6:
                self.x += ds * math.cos(self.theta)
                self.y += ds * math.sin(self.theta)
            else:
                radius = ds / dtheta
                self.x += radius * (math.sin(self.theta) - math.sin(previous_theta))
                self.y -= radius * (math.cos(self.theta) - math.cos(previous_theta))
            # This runs only after the encoder/IMU checks accepted the
            # translation. Physical chassis occupancy therefore outranks old
            # range hits that claimed a wall occupied the same corridor.
            self._clear_traversed_wall_geometry(
                previous_x, previous_y, self.x, self.y
            )
            if (not self.trail or
                    math.hypot(self.x - self.trail[-1][0],
                               self.y - self.trail[-1][1]) >= 15):
                self.trail.append((self.x, self.y))

    def _cell(self, x, y):
        return math.floor(x / self.cell_size_mm), math.floor(y / self.cell_size_mm)

    def _endpoint(self, distance_mm, angle_deg, lateral_mm=0.0, forward_mm=0.0):
        angle = self.theta + math.radians(angle_deg)
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        target = (origin_x + distance_mm * math.cos(angle),
                  origin_y + distance_mm * math.sin(angle))
        return target

    @staticmethod
    def _wall_is_more_outward(boundary, candidate, current):
        if boundary in ("north", "east"):
            return candidate > current
        return candidate < current

    @staticmethod
    def _make_wall_track(orientation, coordinate, tangent, observations=0,
                         sources=None, boundary=None, kind="internal",
                         boundary_blocked=False):
        return {
            "boundary": boundary,
            "kind": kind,
            "orientation": orientation,
            "coordinate": coordinate,
            "coordinates": deque([coordinate], maxlen=41),
            "minimum": tangent,
            "maximum": tangent,
            "observations": observations,
            "sources": set() if sources is None else set(sources),
            # A line physically crossed by the chassis must not immediately be
            # promoted back into an arena boundary simply because its surviving
            # pieces are long.
            "boundary_blocked": bool(boundary_blocked),
        }

    @staticmethod
    def _wall_geometry(target, world_angle):
        ray_x = math.cos(world_angle)
        ray_y = math.sin(world_angle)
        horizontal = abs(ray_y) >= abs(ray_x)
        orientation = "horizontal" if horizontal else "vertical"
        coordinate = target[1] if horizontal else target[0]
        tangent = target[0] if horizontal else target[1]
        return orientation, coordinate, tangent

    @staticmethod
    def _wall_source_strength(source):
        """Relative trust used only for promoting range hits into wall geometry."""
        if source == "8x8":
            return 2.5
        if source.startswith("ultrasound."):
            # Broad acoustic beam: useful corroboration, poor precise geometry.
            return 0.75
        return 2.0  # point TOF after temporal confirmation

    def _weaken_pending_wall_evidence(self, free_cells, preserve_point=None,
                                      preserve_radius_mm=100.0):
        """Free-space rays contradict unconfirmed wall endpoints."""
        if not free_cells or not self.wall_evidence_clusters:
            return
        # Do not use the final part of the current ray as contradiction evidence.
        # A perfectly stable wall can quantise into the cell immediately before
        # the next endpoint as the distance changes by only a few millimetres.
        expanded = set(free_cells)
        if preserve_point is not None:
            px, py = preserve_point
            expanded = {
                cell for cell in expanded
                if math.hypot(
                    (cell[0] + 0.5) * self.cell_size_mm - px,
                    (cell[1] + 0.5) * self.cell_size_mm - py,
                ) > preserve_radius_mm
            }
        kept = []
        for cluster in self.wall_evidence_clusters:
            if cluster.get("promoted"):
                kept.append(cluster)
                continue
            if cluster["cells"] & expanded:
                cluster["score"] = max(0.0, cluster["score"] - 2.0)
            if cluster["score"] >= 0.75:
                kept.append(cluster)
        self.wall_evidence_clusters = kept

    def _accumulate_wall_evidence(self, target, ray_angle_deg, source,
                                  strength=None):
        """Fuse repeated range endpoints before allowing them to become walls."""
        self.wall_evidence_serial += 1
        world_angle = self.theta + math.radians(ray_angle_deg)
        orientation, coordinate, tangent = self._wall_geometry(target, world_angle)
        strength = (self._wall_source_strength(source)
                    if strength is None else float(strength))
        cell = self._cell(*target)

        # Expire weak one-off clusters. A real wall will be refreshed and/or
        # promoted long before this; stale speckle should not accumulate forever.
        serial = self.wall_evidence_serial
        refreshed = []
        for item in self.wall_evidence_clusters:
            age = serial - item["last_seen"]
            if not item.get("promoted") and age > 120:
                item["score"] *= 0.45
            if item.get("promoted") or item["score"] >= 0.75:
                refreshed.append(item)
        self.wall_evidence_clusters = refreshed

        nearby = [
            item for item in self.wall_evidence_clusters
            if not item.get("promoted")
            and item["orientation"] == orientation
            and abs(item["coordinate"] - coordinate) <= 120.0
            and tangent >= item["minimum"] - 220.0
            and tangent <= item["maximum"] + 220.0
        ]
        if nearby:
            cluster = min(
                nearby,
                key=lambda item: (
                    abs(item["coordinate"] - coordinate) +
                    max(0.0, item["minimum"] - tangent,
                        tangent - item["maximum"])
                )
            )
        else:
            cluster = {
                "orientation": orientation,
                "coordinate": coordinate,
                "coordinates": deque([coordinate], maxlen=31),
                "minimum": tangent,
                "maximum": tangent,
                "score": 0.0,
                "hits": 0,
                "sources": set(),
                "cells": set(),
                "last_seen": serial,
                "promoted": False,
            }
            self.wall_evidence_clusters.append(cluster)

        cluster["coordinates"].append(coordinate)
        ordered = sorted(cluster["coordinates"])
        cluster["coordinate"] = ordered[len(ordered) // 2]
        cluster["minimum"] = min(cluster["minimum"], tangent)
        cluster["maximum"] = max(cluster["maximum"], tangent)
        cluster["score"] = min(30.0, cluster["score"] + strength)
        cluster["hits"] += 1
        cluster["sources"].add(source)
        cluster["cells"].add(cell)
        cluster["last_seen"] = serial

        # A point TOF needs several temporally-confirmed observations. A coherent
        # 8x8 surface can accumulate the same score faster because adjacent matrix
        # columns independently support the same plane. Ultrasound alone takes
        # much longer and therefore mainly corroborates existing geometry.
        if cluster["score"] < 4.0 or cluster["hits"] < 2:
            return

        cluster["promoted"] = True
        target = ((cluster["minimum"] + cluster["maximum"]) * 0.5,
                  cluster["coordinate"])
        if orientation == "vertical":
            target = (cluster["coordinate"],
                      (cluster["minimum"] + cluster["maximum"]) * 0.5)
        self._record_wall_observation(
            target, ray_angle_deg, source,
            observations=max(3, int(round(cluster["score"]))),
            sources=cluster["sources"],
            minimum=cluster["minimum"], maximum=cluster["maximum"],
        )
        # Persistent geometry now owns this evidence; discard the temporary
        # cluster so it cannot grow without bound or be counted twice.
        if cluster in self.wall_evidence_clusters:
            self.wall_evidence_clusters.remove(cluster)

    def _merge_internal_wall_observation(self, orientation, coordinate, tangent,
                                         source, observations=1, sources=None,
                                         minimum=None, maximum=None,
                                         boundary_blocked=False):
        """Merge confirmed non-boundary evidence into stable internal segments."""
        minimum = tangent if minimum is None else minimum
        maximum = tangent if maximum is None else maximum

        # Once a long line has been promoted to an arena boundary, new confirmed
        # collinear chunks should extend that boundary instead of appearing as a
        # fresh row of orange internal segments beside it.
        if not boundary_blocked:
            boundary_matches = [
                wall for wall in self.wall_tracks
                if wall["orientation"] == orientation
                and abs(wall["coordinate"] - coordinate) <= 200.0
                and maximum >= wall["minimum"] - 350.0
                and minimum <= wall["maximum"] + 350.0
            ]
            if boundary_matches:
                wall = min(
                    boundary_matches,
                    key=lambda item: abs(item["coordinate"] - coordinate)
                )
                wall["coordinates"].append(coordinate)
                ordered = sorted(wall["coordinates"])
                wall["coordinate"] = ordered[len(ordered) // 2]
                wall["minimum"] = min(wall["minimum"], minimum)
                wall["maximum"] = max(wall["maximum"], maximum)
                wall["observations"] += observations
                wall["sources"].add(source)
                if sources:
                    wall["sources"].update(sources)
                return wall

        nearby = [
            track for track in self.internal_wall_tracks
            if track["orientation"] == orientation
            and abs(track["coordinate"] - coordinate) <= 150.0
            and maximum >= track["minimum"] - 300.0
            and minimum <= track["maximum"] + 300.0
        ]
        if nearby:
            track = min(
                nearby,
                key=lambda item: (
                    abs(item["coordinate"] - coordinate) +
                    max(0.0, item["minimum"] - maximum,
                        minimum - item["maximum"])
                )
            )
        else:
            track = self._make_wall_track(
                orientation, coordinate, tangent, kind="internal",
                boundary_blocked=boundary_blocked,
            )
            self.internal_wall_tracks.append(track)

        track["coordinates"].append(coordinate)
        ordered = sorted(track["coordinates"])
        track["coordinate"] = ordered[len(ordered) // 2]
        track["minimum"] = min(track["minimum"], minimum)
        track["maximum"] = max(track["maximum"], maximum)
        track["observations"] += observations
        track["sources"].add(source)
        if sources:
            track["sources"].update(sources)
        track["boundary_blocked"] = (
            track.get("boundary_blocked", False) or boundary_blocked
        )
        self._coalesce_internal_walls()
        self._promote_external_walls()
        return track

    def _coalesce_internal_walls(self):
        """Join collinear nearby chunks before deciding whether they are outer walls."""
        changed = True
        while changed:
            changed = False
            for i, first in enumerate(self.internal_wall_tracks):
                for j in range(i + 1, len(self.internal_wall_tracks)):
                    second = self.internal_wall_tracks[j]
                    if first["orientation"] != second["orientation"]:
                        continue
                    if abs(first["coordinate"] - second["coordinate"]) > 220.0:
                        continue
                    gap = max(
                        0.0,
                        second["minimum"] - first["maximum"],
                        first["minimum"] - second["maximum"],
                    )
                    if gap > 350.0:
                        continue
                    first["coordinates"].extend(second["coordinates"])
                    ordered = sorted(first["coordinates"])
                    first["coordinate"] = ordered[len(ordered) // 2]
                    first["minimum"] = min(first["minimum"], second["minimum"])
                    first["maximum"] = max(first["maximum"], second["maximum"])
                    first["observations"] += second["observations"]
                    first["sources"].update(second["sources"])
                    first["boundary_blocked"] = (
                        first.get("boundary_blocked", False) or
                        second.get("boundary_blocked", False)
                    )
                    del self.internal_wall_tracks[j]
                    changed = True
                    break
                if changed:
                    break

    def _track_free_space_side(self, track):
        """Return the explored side of a candidate wall, or None if ambiguous."""
        positive = 0
        negative = 0
        margin = 180.0
        tangent_pad = 350.0
        for x, y in self.trail:
            tangent = x if track["orientation"] == "horizontal" else y
            if not (track["minimum"] - tangent_pad <= tangent <=
                    track["maximum"] + tangent_pad):
                continue
            normal = ((y - track["coordinate"])
                      if track["orientation"] == "horizontal"
                      else (x - track["coordinate"]))
            if normal >= margin:
                positive += 1
            elif normal <= -margin:
                negative += 1
        if positive >= 5 and negative <= 1:
            return 1
        if negative >= 5 and positive <= 1:
            return -1
        return None

    def _promote_external_walls(self):
        """Promote long one-sided collinear internal geometry to arena boundary."""
        for track in list(self.internal_wall_tracks):
            if track.get("boundary_blocked"):
                continue
            span = track["maximum"] - track["minimum"]
            if span < 800.0 or track["observations"] < 6:
                continue
            free_side = self._track_free_space_side(track)
            if free_side is None:
                continue

            if track["orientation"] == "horizontal":
                boundary = "south" if free_side > 0 else "north"
            else:
                boundary = "west" if free_side > 0 else "east"

            existing = next(
                (wall for wall in self.wall_tracks
                 if wall.get("boundary") == boundary), None
            )
            if existing is not None:
                if abs(existing["coordinate"] - track["coordinate"]) <= 180.0:
                    existing["minimum"] = min(existing["minimum"], track["minimum"])
                    existing["maximum"] = max(existing["maximum"], track["maximum"])
                    existing["observations"] += track["observations"]
                    existing["sources"].update(track["sources"])
                    self.internal_wall_tracks.remove(track)
                    continue
                if not self._wall_is_more_outward(
                        boundary, track["coordinate"], existing["coordinate"]):
                    continue
                # A stronger/farther long wall replaces the earlier boundary.
                # Demote the old line directly and block immediate re-promotion;
                # otherwise recursive reclassification can recreate the boundary
                # that was just superseded.
                self.wall_tracks.remove(existing)
                existing["kind"] = "internal"
                existing["boundary"] = None
                existing["boundary_blocked"] = True
                existing["sources"].add("demoted-boundary")
                self.internal_wall_tracks.append(existing)
                self._coalesce_internal_walls()

            if track not in self.internal_wall_tracks:
                continue
            self.internal_wall_tracks.remove(track)
            track["kind"] = "boundary"
            track["boundary"] = boundary
            self.wall_tracks.append(track)
            self.wall_promotions += 1

    @staticmethod
    def _carve_wall_track(track, crossing_tangent, half_width=220.0):
        """Return wall pieces left after the chassis proves a corridor is open."""
        clear_min = crossing_tangent - half_width
        clear_max = crossing_tangent + half_width
        pieces = []
        if track["minimum"] < clear_min - 60.0:
            pieces.append(
                (track["minimum"], min(track["maximum"], clear_min))
            )
        if track["maximum"] > clear_max + 60.0:
            pieces.append(
                (max(track["minimum"], clear_max), track["maximum"])
            )
        return pieces

    def _clear_traversed_wall_geometry(self, x0, y0, x1, y1):
        """Use confirmed chassis motion to erase contradictory wall evidence."""
        distance = math.hypot(x1 - x0, y1 - y0)
        if distance < 2.0:
            return

        free_radius_mm = 210.0
        step_mm = max(5.0, self.cell_size_mm)
        samples = max(1, math.ceil(distance / step_mm))
        cell_radius = max(1, math.ceil(free_radius_mm / self.cell_size_mm))
        cleared_cells = set()
        for index in range(samples + 1):
            fraction = index / samples
            px = x0 + (x1 - x0) * fraction
            py = y0 + (y1 - y0) * fraction
            centre_col, centre_row = self._cell(px, py)
            for dc in range(-cell_radius, cell_radius + 1):
                for dr in range(-cell_radius, cell_radius + 1):
                    cx = (centre_col + dc + 0.5) * self.cell_size_mm
                    cy = (centre_row + dr + 0.5) * self.cell_size_mm
                    if math.hypot(cx - px, cy - py) > free_radius_mm:
                        continue
                    cell = (centre_col + dc, centre_row + dr)
                    cleared_cells.add(cell)
                    self.cells[cell] = min(-8, self.cells.get(cell, 0) - 4)
                    self.weight_votes.pop(cell, None)

        # Pending endpoints that the chassis physically occupied are impossible
        # walls; erase them before they can later accumulate enough confidence.
        if cleared_cells:
            self.wall_evidence_clusters = [
                item for item in self.wall_evidence_clusters
                if item.get("promoted") or not (item["cells"] & cleared_cells)
            ]

        def crossing_tangent(track):
            coordinate = track["coordinate"]
            if track["orientation"] == "horizontal":
                delta = y1 - y0
                if abs(delta) < 1e-6:
                    return None
                if (y0 - coordinate) * (y1 - coordinate) > 0:
                    return None
                fraction = (coordinate - y0) / delta
                if not 0.0 <= fraction <= 1.0:
                    return None
                return x0 + (x1 - x0) * fraction

            delta = x1 - x0
            if abs(delta) < 1e-6:
                return None
            if (x0 - coordinate) * (x1 - coordinate) > 0:
                return None
            fraction = (coordinate - x0) / delta
            if not 0.0 <= fraction <= 1.0:
                return None
            return y0 + (y1 - y0) * fraction

        rebuilt_internal = []
        for track in self.internal_wall_tracks:
            tangent = crossing_tangent(track)
            if (tangent is None or
                    tangent < track["minimum"] - 220.0 or
                    tangent > track["maximum"] + 220.0):
                rebuilt_internal.append(track)
                continue
            for minimum, maximum in self._carve_wall_track(track, tangent):
                piece = self._make_wall_track(
                    track["orientation"], track["coordinate"],
                    (minimum + maximum) * 0.5,
                    observations=track["observations"],
                    sources=track["sources"], kind="internal",
                    boundary_blocked=True,
                )
                piece["minimum"] = minimum
                piece["maximum"] = maximum
                piece["coordinates"] = deque(track["coordinates"], maxlen=41)
                rebuilt_internal.append(piece)
        self.internal_wall_tracks = rebuilt_internal

        rebuilt_boundaries = []
        for track in self.wall_tracks:
            tangent = crossing_tangent(track)
            if (tangent is None or
                    tangent < track["minimum"] - 220.0 or
                    tangent > track["maximum"] + 220.0):
                rebuilt_boundaries.append(track)
                continue
            for minimum, maximum in self._carve_wall_track(track, tangent):
                self._merge_internal_wall_observation(
                    track["orientation"], track["coordinate"],
                    (minimum + maximum) * 0.5, "traversed-boundary",
                    observations=max(3, track["observations"]),
                    sources=track["sources"],
                    minimum=minimum, maximum=maximum,
                    boundary_blocked=True,
                )
        self.wall_tracks = rebuilt_boundaries

    def _record_wall_observation(self, target, ray_angle_deg, source,
                                 observations=1, sources=None,
                                 minimum=None, maximum=None):
        """Store only fused wall evidence; external classification happens later."""
        world_angle = self.theta + math.radians(ray_angle_deg)
        orientation, coordinate, tangent = self._wall_geometry(target, world_angle)
        self._merge_internal_wall_observation(
            orientation, coordinate, tangent, source,
            observations=observations, sources=sources,
            minimum=minimum, maximum=maximum,
        )

    def _add_observation(self, distance_mm, angle_deg, source, lateral_mm=0.0,
                         forward_mm=0.0, wall_candidate=False):
        if not self.min_range_mm <= distance_mm <= self.max_range_mm:
            return
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        target = self._endpoint(distance_mm, angle_deg, lateral_mm, forward_mm)
        self.current_rays.append((origin_x, origin_y, *target, source))
        # Without a distance scale, previous readings cannot be registered.
        if self.mm_per_count > 0:
            self.points.append((*target, source))
            endpoint = self._cell(*target)
            traversed = set()
            count = max(1, math.ceil(distance_mm / (self.cell_size_mm / 3)))
            for step in range(count):
                fraction = step / count
                cell = self._cell(origin_x + fraction * (target[0] - origin_x),
                                  origin_y + fraction * (target[1] - origin_y))
                if cell == endpoint or cell in traversed:
                    continue
                traversed.add(cell)
                self.cells[cell] = max(-8, self.cells.get(cell, 0) - 1)
            self._weaken_pending_wall_evidence(traversed, target)

            if source.startswith("ultrasound."):
                endpoint_strength = 1
            else:
                endpoint_strength = 2
            self.cells[endpoint] = min(
                8, self.cells.get(endpoint, 0) + endpoint_strength
            )
            if wall_candidate:
                self._accumulate_wall_evidence(target, angle_deg, source)

    def wall_segment(self, wall):
        """Return a visible segment for outer or internal wall geometry."""
        if wall.get("kind") == "internal":
            minimum, maximum = wall["minimum"], wall["maximum"]
            if maximum - minimum < 120.0:
                centre = (minimum + maximum) * 0.5
                return centre - 60.0, centre + 60.0
            return minimum, maximum

        if wall["orientation"] == "horizontal":
            perpendicular = sorted(
                item["coordinate"] for item in self.wall_tracks
                if item["orientation"] == "vertical" and
                item["observations"] >= 3
            )
        else:
            perpendicular = sorted(
                item["coordinate"] for item in self.wall_tracks
                if item["orientation"] == "horizontal" and
                item["observations"] >= 3
            )
        if len(perpendicular) >= 2:
            return perpendicular[0], perpendicular[-1]
        minimum, maximum = wall["minimum"], wall["maximum"]
        if maximum - minimum < 300.0:
            centre = (minimum + maximum) * 0.5
            minimum, maximum = centre - 150.0, centre + 150.0
        return minimum, maximum

    def _add_wedge_observation(self, distance_mm, centre_angle_deg, width_deg,
                               source, lateral_mm=0.0, forward_mm=0.0,
                               reference_angle_deg=0.0, wall_candidate=False):
        """Paint a matrix sector whose distance is forward depth, not radius."""
        if not self.min_range_mm <= distance_mm <= self.max_range_mm:
            return
        origin_x = self.x + forward_mm * math.cos(self.theta) + lateral_mm * math.sin(self.theta)
        origin_y = self.y + forward_mm * math.sin(self.theta) - lateral_mm * math.cos(self.theta)
        half_width = width_deg * 0.5

        def plane_endpoint(angle_deg):
            # Equal matrix depths lie on a plane perpendicular to the module's
            # centreline. Dividing by cos converts axial depth to ray length.
            relative = math.radians(angle_deg - reference_angle_deg)
            cosine = max(0.05, math.cos(relative))
            return self._endpoint(distance_mm / cosine, angle_deg,
                                  lateral_mm, forward_mm)

        for offset in (-half_width, 0.0, half_width):
            target = plane_endpoint(centre_angle_deg + offset)
            self.current_rays.append((origin_x, origin_y, *target, source))
        if self.mm_per_count <= 0:
            return

        centre_target = plane_endpoint(centre_angle_deg)
        self.points.append((*centre_target, source))
        free_cells = set()
        left_relative = math.radians(centre_angle_deg - half_width - reference_angle_deg)
        right_relative = math.radians(centre_angle_deg + half_width - reference_angle_deg)
        cap_width = abs(distance_mm * (math.tan(right_relative) - math.tan(left_relative)))
        angular_samples = max(2, math.ceil(cap_width / self.cell_size_mm))
        endpoint_cells = set()
        for index in range(angular_samples + 1):
            fraction = index / angular_samples
            angle_deg = centre_angle_deg - half_width + width_deg * fraction
            relative = math.radians(angle_deg - reference_angle_deg)
            ray_length = distance_mm / max(0.05, math.cos(relative))
            world_angle = self.theta + math.radians(angle_deg)
            free_limit = max(0.0, ray_length - self.cell_size_mm * 1.5)
            radius = self.cell_size_mm * 0.5
            while radius <= free_limit:
                free_cells.add(self._cell(origin_x + radius * math.cos(world_angle),
                                          origin_y + radius * math.sin(world_angle)))
                radius += self.cell_size_mm
            endpoint_cells.add(self._cell(
                origin_x + ray_length * math.cos(world_angle),
                origin_y + ray_length * math.sin(world_angle)
            ))
        for cell in free_cells:
            self.cells[cell] = max(-8, self.cells.get(cell, 0) - 1)
        self._weaken_pending_wall_evidence(
            free_cells, centre_target, preserve_radius_mm=140.0
        )
        endpoint_strength = 3 if wall_candidate else 1
        for cell in endpoint_cells:
            self.cells[cell] = min(
                8, self.cells.get(cell, 0) + endpoint_strength
            )
        if wall_candidate:
            self._accumulate_wall_evidence(
                centre_target, centre_angle_deg, source,
                strength=self._wall_source_strength(source),
            )

    def receive_matrix(self, message):
        self.last_matrix_available = bool(message.get("available"))
        self.last_matrix_valid = bool(message.get("valid"))
        if not message.get("available") or not message.get("valid"):
            self.matrix_top_samples = []
            return
        frame_id = message.get("frame")
        if frame_id == self.last_matrix_frame:
            return
        values = message.get("data")
        if not isinstance(values, list) or len(values) != 64:
            return
        self.last_matrix_frame = frame_id
        self.matrix_sample_time = message.get("time")
        self.matrix_top_samples = []

        # First reduce each physical horizontal column using vertical consensus.
        # Four inlier pixels are required; a few flying/noisy pixels cannot define
        # a whole floor-plan sector. Then require an adjacent column at a similar
        # depth before the matrix is allowed to create persistent wall geometry.
        side = -1 if self.matrix_mirrored else 1
        sector_width = self.matrix_fov_deg / 8.0
        candidates = []
        for col in range(8):
            floor_rows = max(0, min(4, int(getattr(self, "matrix_floor_rows", 2))))
            raw = sorted(
                distance for row in range(8 - floor_rows)
                if (distance := _number(values[row * 8 + col])) is not None
                and self.min_range_mm <= distance <= self.max_range_mm
            )
            if len(raw) < 4:
                continue
            median = raw[len(raw) // 2]
            inlier_limit = max(80.0, median * 0.12)
            inliers = [value for value in raw
                       if abs(value - median) <= inlier_limit]
            if len(inliers) < 4:
                continue
            distance = inliers[len(inliers) // 2]
            distance = self._filtered_distance(
                f"matrix.column.{col}", distance, confirm_initial=True
            )
            if distance is None:
                continue
            angle = (self.matrix_spec["angle"] +
                     side * ((3.5 - col) / 8.0) * self.matrix_fov_deg)
            candidates.append((col, angle, distance))

        coherent = set()
        by_col = {col: (angle, distance) for col, angle, distance in candidates}
        for col, _angle, distance in candidates:
            for neighbour in (col - 1, col + 1):
                if neighbour not in by_col:
                    continue
                other_distance = by_col[neighbour][1]
                agreement = max(100.0, min(distance, other_distance) * 0.15)
                if abs(distance - other_distance) <= agreement:
                    coherent.add(col)
                    coherent.add(neighbour)

        for col, angle, distance in candidates:
            self.matrix_top_samples.append((angle, distance))
            self._add_wedge_observation(
                distance, angle, sector_width, "8x8",
                self.matrix_spec["x"], self.matrix_spec["y"],
                self.matrix_spec["angle"],
                wall_candidate=col in coherent,
            )



class MissionLayout:
    """Editable pre-laid arena and grid/A* mission planner."""

    WIDTH_MM = 4900.0
    HEIGHT_MM = 2400.0
    HOME_WIDTH_MM = 650.0
    HOME_HEIGHT_MM = 650.0
    ROBOT_RADIUS_MM = 215.0
    SAFETY_MARGIN_MM = 90.0
    GRID_MM = 100.0
    MAX_ROUTE_POINTS = 64
    REFERENCE_LINE_COUNT = 7

    DEFAULT_SIZES = {
        "wall": (700.0, 110.0),
        "ramp": (700.0, 380.0),
        "tube": (320.0, 320.0),
    }

    def __init__(self):
        self.my_home = "green"
        self.fallback_strategy = 0
        self.start = {"x": 325.0, "y": 325.0, "heading_deg": 0.0}
        self.weights = []
        self.obstacles = []
        self.route = []
        self.route_error = ""
        self._next_id = 1

    def _id(self):
        value = self._next_id
        self._next_id += 1
        return value

    def home_rect(self, colour):
        if colour == "green":
            return (0.0, 0.0, self.HOME_WIDTH_MM, self.HOME_HEIGHT_MM)
        return (0.0, self.HEIGHT_MM - self.HOME_HEIGHT_MM,
                self.HOME_WIDTH_MM, self.HEIGHT_MM)

    def no_go_home(self):
        return self.home_rect("blue" if self.my_home == "green" else "green")

    def add_weight(self, x, y, dummy=False):
        item = {"id": self._id(), "x": float(x), "y": float(y),
                "dummy": bool(dummy)}
        self.weights.append(item)
        self.route = []
        return item

    def add_obstacle(self, kind, x, y):
        if kind not in self.DEFAULT_SIZES:
            raise ValueError(kind)
        width, height = self.DEFAULT_SIZES[kind]
        item = {"id": self._id(), "kind": kind, "x": float(x), "y": float(y),
                "w": width, "h": height, "rotation": 0}
        self.obstacles.append(item)
        self.route = []
        return item

    def move_item(self, item_id, x, y):
        if item_id == "start":
            self.start["x"] = max(0.0, min(self.WIDTH_MM, float(x)))
            self.start["y"] = max(0.0, min(self.HEIGHT_MM, float(y)))
            self.route = []
            return True
        for collection in (self.weights, self.obstacles):
            for item in collection:
                if item["id"] == item_id:
                    item["x"] = max(0.0, min(self.WIDTH_MM, float(x)))
                    item["y"] = max(0.0, min(self.HEIGHT_MM, float(y)))
                    self.route = []
                    return True
        return False

    def delete_item(self, item_id):
        for collection in (self.weights, self.obstacles):
            for index, item in enumerate(collection):
                if item["id"] == item_id:
                    del collection[index]
                    self.route = []
                    return True
        return False

    def rotate_item(self, item_id):
        for item in self.obstacles:
            if item["id"] == item_id and item["kind"] in ("wall", "ramp"):
                item["rotation"] = (int(item.get("rotation", 0)) + 90) % 180
                self.route = []
                return True
        return False

    def set_obstacle_size(self, item_id, width_mm, height_mm):
        for item in self.obstacles:
            if item["id"] != item_id:
                continue
            width = max(50.0, min(4000.0, float(width_mm)))
            height = max(50.0, min(2400.0, float(height_mm)))
            item["w"] = width
            item["h"] = width if item["kind"] == "tube" else height
            self.route = []
            return True
        return False

    @staticmethod
    def _inflate_rect(rect, amount):
        x0, y0, x1, y1 = rect
        return (x0 - amount, y0 - amount, x1 + amount, y1 + amount)

    @staticmethod
    def _inside_rect(x, y, rect):
        x0, y0, x1, y1 = rect
        return x0 <= x <= x1 and y0 <= y <= y1

    def _obstacle_rect(self, item):
        width, height = item["w"], item["h"]
        if int(item.get("rotation", 0)) % 180 == 90:
            width, height = height, width
        return (item["x"] - width / 2, item["y"] - height / 2,
                item["x"] + width / 2, item["y"] + height / 2)

    def blocked(self, x, y, extra=0.0):
        clearance = self.ROBOT_RADIUS_MM + self.SAFETY_MARGIN_MM + extra
        if (x < clearance or y < clearance or
                x > self.WIDTH_MM - clearance or
                y > self.HEIGHT_MM - clearance):
            return True
        if self._inside_rect(x, y, self._inflate_rect(self.no_go_home(), clearance)):
            return True
        for item in self.obstacles:
            if item["kind"] == "tube":
                radius = item["w"] / 2 + clearance
                if math.hypot(x - item["x"], y - item["y"]) <= radius:
                    return True
            elif self._inside_rect(x, y, self._inflate_rect(
                    self._obstacle_rect(item), clearance)):
                return True
        for item in self.weights:
            if not item["dummy"]:
                continue
            if math.hypot(x - item["x"], y - item["y"]) <= 120.0 + clearance:
                return True
        return False

    def _segment_clear(self, first, second):
        x0, y0 = first
        x1, y1 = second
        distance = math.hypot(x1 - x0, y1 - y0)
        samples = max(1, math.ceil(distance / 45.0))
        for index in range(samples + 1):
            t = index / samples
            if self.blocked(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True

    def _nearest_free_grid(self, point):
        spacing = self.GRID_MM
        gx = int(round(point[0] / spacing))
        gy = int(round(point[1] / spacing))
        if not self.blocked(gx * spacing, gy * spacing):
            return gx, gy
        for radius in range(1, 8):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidates.append((gx + dx, gy + dy))
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidates.append((gx + dx, gy + dy))
            candidates.sort(key=lambda p: math.hypot(
                p[0] * spacing - point[0], p[1] * spacing - point[1]))
            for candidate in candidates:
                x, y = candidate[0] * spacing, candidate[1] * spacing
                if 0 <= x <= self.WIDTH_MM and 0 <= y <= self.HEIGHT_MM and not self.blocked(x, y):
                    return candidate
        return None

    def _astar(self, start, goal):
        start_cell = self._nearest_free_grid(start)
        goal_cell = self._nearest_free_grid(goal)
        if start_cell is None or goal_cell is None:
            return None
        spacing = self.GRID_MM
        neighbours = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
        ]
        queue = [(0.0, start_cell)]
        g_score = {start_cell: 0.0}
        parent = {}
        closed = set()
        while queue:
            _priority, current = heapq.heappop(queue)
            if current in closed:
                continue
            if current == goal_cell:
                break
            closed.add(current)
            for dx, dy, cost in neighbours:
                nxt = (current[0] + dx, current[1] + dy)
                x, y = nxt[0] * spacing, nxt[1] * spacing
                if not (0 <= x <= self.WIDTH_MM and 0 <= y <= self.HEIGHT_MM):
                    continue
                if self.blocked(x, y):
                    continue
                tentative = g_score[current] + cost
                if tentative >= g_score.get(nxt, float("inf")):
                    continue
                g_score[nxt] = tentative
                parent[nxt] = current
                heuristic = math.hypot(goal_cell[0] - nxt[0], goal_cell[1] - nxt[1])
                heapq.heappush(queue, (tentative + heuristic, nxt))
        if goal_cell not in g_score:
            return None
        cells = [goal_cell]
        while cells[-1] != start_cell:
            cells.append(parent[cells[-1]])
        cells.reverse()
        path = [(cell[0] * spacing, cell[1] * spacing) for cell in cells]
        if self._segment_clear(start, path[0]):
            path[0] = (float(start[0]), float(start[1]))
        if self._segment_clear(path[-1], goal):
            path[-1] = (float(goal[0]), float(goal[1]))
        return self._smooth(path)

    def _smooth(self, path):
        if not path:
            return []
        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            furthest = anchor + 1
            for candidate in range(len(path) - 1, anchor, -1):
                if self._segment_clear(path[anchor], path[candidate]):
                    furthest = candidate
                    break
            result.append(path[furthest])
            anchor = furthest
        return result

    @staticmethod
    def _path_length(path):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(path, path[1:]))

    def plan(self):
        self.route = []
        self.route_error = ""
        real_weights = [item for item in self.weights if not item["dummy"]]
        if not real_weights:
            self.route_error = "Add at least one real weight location."
            return False
        current = (self.start["x"], self.start["y"])
        if self.blocked(*current):
            self.route_error = "Start position is inside a border/no-go/obstacle clearance."
            return False
        remaining = list(real_weights)
        route = []
        while remaining:
            options = []
            for item in remaining:
                weight = (item["x"], item["y"])
                visit = weight
                if self.blocked(*visit):
                    free_cell = self._nearest_free_grid(weight)
                    if free_cell is None:
                        continue
                    visit = (free_cell[0] * self.GRID_MM,
                             free_cell[1] * self.GRID_MM)
                    # Firmware treats a real target as visited at <=100 mm. A
                    # farther approach point would falsely mark an unreachable
                    # weight as collected.
                    if math.hypot(visit[0] - weight[0],
                                  visit[1] - weight[1]) > 80.0:
                        continue
                path = self._astar(current, visit)
                if path:
                    options.append((self._path_length(path), item, path, visit))
            if not options:
                self.route_error = "At least one real weight is unreachable with the current obstacles/no-go zone."
                return False
            _cost, chosen, path, visit = min(options, key=lambda value: value[0])
            for point in path[1:]:
                route.append({"x": point[0], "y": point[1], "target": False})
            if not route or math.hypot(route[-1]["x"] - visit[0],
                                       route[-1]["y"] - visit[1]) > 1.0:
                route.append({"x": visit[0], "y": visit[1], "target": True})
            else:
                route[-1]["target"] = True
                route[-1]["x"], route[-1]["y"] = visit
            current = visit
            remaining.remove(chosen)
        if len(route) > self.MAX_ROUTE_POINTS:
            self.route_error = f"Planned route has {len(route)} points; simplify the layout to <= {self.MAX_ROUTE_POINTS}."
            return False
        self.route = route
        return True

    def command_payload(self):
        points = []
        for point in self.route:
            points.extend([int(round(point["x"])), int(round(point["y"])),
                           1 if point.get("target") else 0])
        return {
            "start_x_mm": int(round(self.start["x"])),
            "start_y_mm": int(round(self.start["y"])),
            "start_heading_deg": int(round(self.start["heading_deg"])) % 360,
            "fallback_strategy": int(self.fallback_strategy),
            "points": points,
        }

    def to_dict(self):
        return {
            "my_home": self.my_home,
            "fallback_strategy": self.fallback_strategy,
            "start": self.start,
            "weights": self.weights,
            "obstacles": self.obstacles,
            "next_id": self._next_id,
        }

    def load_dict(self, data):
        if not isinstance(data, dict):
            return
        if data.get("my_home") in ("green", "blue"):
            self.my_home = data["my_home"]
        self.fallback_strategy = max(0, min(2, int(data.get("fallback_strategy", 0))))
        start = data.get("start", {})
        self.start = {
            "x": float(start.get("x", 325.0)),
            "y": float(start.get("y", 325.0)),
            "heading_deg": float(start.get("heading_deg", 0.0)) % 360.0,
        }
        self.weights = [dict(item) for item in data.get("weights", []) if isinstance(item, dict)]
        self.obstacles = [dict(item) for item in data.get("obstacles", []) if isinstance(item, dict)]
        self._next_id = max(int(data.get("next_id", 1)),
                            1 + max([0] + [int(item.get("id", 0)) for item in self.weights + self.obstacles]))
        self.route = []


class MissionLayoutCanvas(QWidget):
    layout_changed = pyqtSignal()
    selection_changed = pyqtSignal(object)
    cursor_moved = pyqtSignal(float, float)

    def __init__(self, layout_model, live_model, parent=None):
        super().__init__(parent)
        self.layout_model = layout_model
        self.live_model = live_model
        self.setMinimumSize(760, 430)
        self.setMouseTracking(True)
        self.add_mode = None
        self.selected_id = "start"
        self._dragging = False
        self._panning = False
        self._pan_at = None
        self.zoom = 1.0
        self.pan = QPointF(0.0, 0.0)

    def reset_view(self):
        self.zoom = 1.0
        self.pan = QPointF(0.0, 0.0)
        self.update()

    def set_add_mode(self, mode):
        self.add_mode = mode
        self.update()

    def _transform(self):
        margin = 48.0
        base_scale = min(
            max(0.001, (self.width() - 2 * margin) / self.layout_model.WIDTH_MM),
            max(0.001, (self.height() - 2 * margin) / self.layout_model.HEIGHT_MM),
        )
        scale = base_scale * self.zoom
        ox = (self.width() - self.layout_model.WIDTH_MM * scale) / 2 + self.pan.x()
        oy = (self.height() - self.layout_model.HEIGHT_MM * scale) / 2 + self.pan.y()
        return scale, ox, oy

    def _screen(self, x, y):
        scale, ox, oy = self._transform()
        return QPointF(ox + x * scale, oy + y * scale)

    def _world(self, point):
        scale, ox, oy = self._transform()
        return ((point.x() - ox) / scale, (point.y() - oy) / scale)

    def wheelEvent(self, event):
        before = self._world(event.position())
        old_zoom = self.zoom
        self.zoom = min(8.0, max(0.75, old_zoom *
                        (1.20 if event.angleDelta().y() > 0 else 1 / 1.20)))
        if abs(self.zoom - old_zoom) < 1e-9:
            return
        after = self._screen(*before)
        self.pan += event.position() - after
        self.update()

    def _nearest_item(self, position):
        candidates = [("start", self.layout_model.start["x"], self.layout_model.start["y"])]
        candidates += [(item["id"], item["x"], item["y"])
                       for item in self.layout_model.weights]
        candidates += [(item["id"], item["x"], item["y"])
                       for item in self.layout_model.obstacles]
        if not candidates:
            return None
        ident, x, y = min(
            candidates,
            key=lambda item: (position - self._screen(item[1], item[2])).manhattanLength(),
        )
        return ident if (position - self._screen(x, y)).manhattanLength() <= 40 else None

    def mousePressEvent(self, event):
        if event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._panning = True
            self._pan_at = event.position()
            return
        if event.button() != Qt.MouseButton.LeftButton:
            return
        x, y = self._world(event.position())
        x = max(0.0, min(self.layout_model.WIDTH_MM, x))
        y = max(0.0, min(self.layout_model.HEIGHT_MM, y))
        if self.add_mode:
            if self.add_mode == "start":
                self.layout_model.move_item("start", x, y)
                self.selected_id = "start"
            elif self.add_mode == "weight":
                self.selected_id = self.layout_model.add_weight(x, y, False)["id"]
            elif self.add_mode == "dummy":
                self.selected_id = self.layout_model.add_weight(x, y, True)["id"]
            else:
                self.selected_id = self.layout_model.add_obstacle(self.add_mode, x, y)["id"]
            self.add_mode = None
            self.selection_changed.emit(self.selected_id)
            self.layout_changed.emit()
            self.update()
            return
        selected = self._nearest_item(event.position())
        if selected is not None:
            self.selected_id = selected
            self._dragging = True
            self.selection_changed.emit(selected)
            self.update()

    def mouseMoveEvent(self, event):
        x, y = self._world(event.position())
        self.cursor_moved.emit(x, y)
        if self._panning and self._pan_at is not None:
            self.pan += event.position() - self._pan_at
            self._pan_at = event.position()
            self.update()
            return
        if not self._dragging or self.selected_id is None:
            return
        x = max(0.0, min(self.layout_model.WIDTH_MM, x))
        y = max(0.0, min(self.layout_model.HEIGHT_MM, y))
        self.layout_model.move_item(self.selected_id, x, y)
        self.layout_changed.emit()
        self.update()

    def mouseReleaseEvent(self, _event):
        self._dragging = False
        self._panning = False
        self._pan_at = None

    def delete_selected(self):
        if self.selected_id == "start":
            return
        if self.layout_model.delete_item(self.selected_id):
            self.selected_id = None
            self.selection_changed.emit(None)
            self.layout_changed.emit()
            self.update()

    def rotate_selected(self):
        if self.layout_model.rotate_item(self.selected_id):
            self.layout_changed.emit()
            self.selection_changed.emit(self.selected_id)
            self.update()

    @staticmethod
    def _signed_angle_delta(target_deg, current_deg):
        return (target_deg - current_deg + 180.0) % 360.0 - 180.0

    def _likely_visible_planned_weight(self):
        if self.live_model.latest.get("weight.detected") is not True:
            return None
        robot_x = _number(self.live_model.latest.get("mission.pose_x_mm"))
        robot_y = _number(self.live_model.latest.get("mission.pose_y_mm"))
        robot_heading = _number(self.live_model.latest.get("mission.heading_deg"))
        nearest_mm = _number(self.live_model.latest.get("weight.nearest_mm"))
        direction = int(_number(self.live_model.latest.get("weight.direction")) or 0)
        if None in (robot_x, robot_y, robot_heading):
            return None
        sector_centre = {-2: -38.0, -1: -13.0, 1: 13.0, 2: 38.0}.get(direction, 0.0)
        best = None
        for item in self.layout_model.weights:
            if item.get("dummy"):
                continue
            dx = item["x"] - robot_x
            dy = item["y"] - robot_y
            distance = math.hypot(dx, dy)
            if distance > 1450.0:
                continue
            bearing = math.degrees(math.atan2(dy, dx)) % 360.0
            error = abs(self._signed_angle_delta(bearing, robot_heading) - sector_centre)
            if error > 28.0:
                continue
            range_error = abs(distance - nearest_mm) if nearest_mm not in (None, 0) else 0.0
            score = error * 12.0 + range_error
            if best is None or score < best[0]:
                best = (score, item)
        return None if best is None else best[1]

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#111827"))
        model = self.layout_model
        scale, ox, oy = self._transform()
        arena = QRectF(ox, oy, model.WIDTH_MM * scale, model.HEIGHT_MM * scale)
        p.setPen(QPen(QColor("#f8fafc"), 4))
        p.setBrush(QColor("#182231"))
        p.drawRect(arena)

        def rect_world(rect):
            x0, y0, x1, y1 = rect
            return QRectF(self._screen(x0, y0), self._screen(x1, y1)).normalized()

        # Seven placement references split 4.9 m into eight equal bays.
        p.setPen(QPen(QColor(148, 163, 184, 120), 1, Qt.PenStyle.DashLine))
        for index in range(1, model.REFERENCE_LINE_COUNT + 1):
            x = model.WIDTH_MM * index / (model.REFERENCE_LINE_COUNT + 1)
            top = self._screen(x, 0.0)
            bottom = self._screen(x, model.HEIGHT_MM)
            p.drawLine(top, bottom)
            p.setPen(QColor("#94a3b8"))
            p.drawText(QRectF(top.x() - 48, top.y() + 3, 96, 20),
                       Qt.AlignmentFlag.AlignCenter,
                       f"L{index}  {x:.0f} mm")
            p.setPen(QPen(QColor(148, 163, 184, 120), 1, Qt.PenStyle.DashLine))

        for colour, fill, text in (("green", "#166534", "GREEN HOME"),
                                   ("blue", "#1d4ed8", "BLUE HOME")):
            rect = rect_world(model.home_rect(colour))
            p.setPen(QPen(QColor("#86efac" if colour == "green" else "#93c5fd"), 2))
            p.setBrush(QColor(fill))
            p.drawRect(rect)
            p.setPen(QColor("#f8fafc"))
            p.drawText(rect, Qt.AlignmentFlag.AlignCenter, text)
            if colour != model.my_home:
                p.setPen(QPen(QColor("#ef4444"), 3, Qt.PenStyle.DashLine))
                p.setBrush(QColor(127, 29, 29, 90))
                p.drawRect(rect)
                p.drawText(rect.adjusted(0, 22, 0, 0),
                           Qt.AlignmentFlag.AlignCenter, "NO-GO ZONE")

        if model.route:
            points = ([self._screen(model.start["x"], model.start["y"])] +
                      [self._screen(point["x"], point["y"])
                       for point in model.route])
            p.setPen(QPen(QColor("#22d3ee"), 4, Qt.PenStyle.SolidLine,
                          Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            p.drawPolyline(QPolygonF(points))
            visit_number = 0
            for point in model.route:
                if not point.get("target"):
                    continue
                visit_number += 1
                centre = self._screen(point["x"], point["y"])
                p.setPen(QPen(QColor("#f8fafc"), 2))
                p.setBrush(QColor("#0891b2"))
                p.drawEllipse(centre, 9, 9)
                p.drawText(QRectF(centre.x() - 12, centre.y() - 25, 24, 18),
                           Qt.AlignmentFlag.AlignCenter, str(visit_number))

        for item in model.obstacles:
            selected = item["id"] == self.selected_id
            colour = {"wall": "#9ca3af", "ramp": "#f59e0b", "tube": "#a78bfa"}[item["kind"]]
            p.setPen(QPen(QColor("#ffffff" if selected else colour), 3 if selected else 2))
            p.setBrush(QColor(colour))
            if item["kind"] == "tube":
                radius = item["w"] / 2 * scale
                p.drawEllipse(self._screen(item["x"], item["y"]), radius, radius)
            else:
                x0, y0, x1, y1 = model._obstacle_rect(item)
                p.drawRect(rect_world((x0, y0, x1, y1)))
            centre = self._screen(item["x"], item["y"])
            p.setPen(QColor("#111827"))
            size_text = (f"Ø{item['w']:.0f}" if item["kind"] == "tube" else
                         f"{item['w']:.0f}×{item['h']:.0f}")
            p.drawText(QRectF(centre.x() - 80, centre.y() - 17, 160, 34),
                       Qt.AlignmentFlag.AlignCenter,
                       f"{item['kind'].upper()}\n{size_text} mm")

        likely_weight = self._likely_visible_planned_weight()
        for index, item in enumerate(model.weights, 1):
            centre = self._screen(item["x"], item["y"])
            selected = item["id"] == self.selected_id
            colour = QColor("#6b7280" if item["dummy"] else "#facc15")
            if item is likely_weight:
                p.setPen(QPen(QColor("#22c55e"), 6))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawEllipse(centre, 20, 20)
            p.setPen(QPen(QColor("#ffffff") if selected else colour, 3 if selected else 2))
            p.setBrush(colour)
            radius = 14 if item["dummy"] else 12
            p.drawEllipse(centre, radius, radius)
            p.setPen(QColor("#111827"))
            p.drawText(QRectF(centre.x() - 16, centre.y() - 9, 32, 18),
                       Qt.AlignmentFlag.AlignCenter,
                       "D" if item["dummy"] else str(index))

        start = self._screen(model.start["x"], model.start["y"])
        heading = math.radians(model.start["heading_deg"])
        nose = QPointF(start.x() + 24 * math.cos(heading),
                       start.y() + 24 * math.sin(heading))
        p.setPen(QPen(QColor("#67e8f9"), 3))
        p.setBrush(QColor("#0891b2"))
        p.drawEllipse(start, 11, 11)
        p.drawLine(start, nose)
        p.drawText(QRectF(start.x() + 13, start.y() - 18, 70, 22),
                   Qt.AlignmentFlag.AlignLeft, "START")

        robot_x = _number(self.live_model.latest.get("mission.pose_x_mm"))
        robot_y = _number(self.live_model.latest.get("mission.pose_y_mm"))
        robot_heading = _number(self.live_model.latest.get("mission.heading_deg"))
        weight_detected = self.live_model.latest.get("weight.detected") is True
        weight_mask = int(_number(self.live_model.latest.get("weight.sector_mask")) or 0)
        if robot_x is not None and robot_y is not None:
            point = self._screen(robot_x, robot_y)
            p.setPen(QPen(QColor("#ffffff"), 2))
            p.setBrush(QColor("#0ea5e9"))
            p.drawEllipse(point, 9, 9)
            if robot_heading is not None:
                angle = math.radians(robot_heading)
                p.drawLine(point, QPointF(point.x() + 24 * math.cos(angle),
                                          point.y() + 24 * math.sin(angle)))
                sector_offsets = (-38.0, -13.0, 13.0, 38.0)
                for sector, offset in enumerate(sector_offsets):
                    active = bool(weight_mask & (1 << sector))
                    sector_angle = math.radians(robot_heading + offset)
                    reach = 125 if active else 70
                    p.setPen(QPen(QColor("#22c55e" if active else "#475569"),
                                  4 if active else 1,
                                  Qt.PenStyle.SolidLine if active else Qt.PenStyle.DashLine))
                    p.drawLine(point, QPointF(point.x() + reach * math.cos(sector_angle),
                                              point.y() + reach * math.sin(sector_angle)))

        panel = QRectF(max(8.0, self.width() - 285.0), 10.0, 275.0, 54.0)
        p.setPen(QPen(QColor("#22c55e" if weight_detected else "#64748b"), 2))
        p.setBrush(QColor(20, 83, 45, 220) if weight_detected else QColor(30, 41, 59, 220))
        p.drawRoundedRect(panel, 7, 7)
        p.setPen(QColor("#f8fafc"))
        nearest = int(_number(self.live_model.latest.get("weight.nearest_mm")) or 0)
        direction = int(_number(self.live_model.latest.get("weight.direction")) or 0)
        direction_text = {-2: "far left", -1: "left", 1: "right", 2: "far right"}.get(direction, "—")
        if weight_detected:
            text = f"WEIGHT VISIBLE  ·  {nearest} mm  ·  {direction_text}"
        elif "weight.detected" in self.live_model.latest:
            text = "NO WEIGHT CURRENTLY VISIBLE"
        else:
            text = "WAITING FOR WEIGHT-VISIBILITY TELEMETRY"
        p.drawText(panel, Qt.AlignmentFlag.AlignCenter | Qt.TextFlag.TextWordWrap, text)

        p.setPen(QColor("#cbd5e1"))
        mode = (f"Click arena to add: {self.add_mode}" if self.add_mode else
                "Left-drag items · middle/right-drag to pan · wheel to zoom")
        p.drawText(12, self.height() - 10, mode)
        p.end()

class ArenaCanvas(QWidget):
    def __init__(self, model, owner):
        super().__init__(owner)
        self.model = model
        self.owner = owner
        self.setMinimumSize(550, 450)
        self.zoom = 1.0
        self.pan = QPointF(0, 0)
        self._drag_at = None
        self._grid_key = None
        self._grid_image = None
        self._painted = {}

    def wheelEvent(self, event):
        old = self.zoom
        self.zoom = min(30.0, max(1.0, old * (1.25 if event.angleDelta().y() > 0 else 0.8)))
        ratio = self.zoom / old
        cursor = event.position() - QPointF(self.width() / 2, self.height() / 2)
        self.pan = cursor - (cursor - self.pan) * ratio
        self.update()

    def mousePressEvent(self, event):
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.MiddleButton):
            self._drag_at = event.position()

    def mouseMoveEvent(self, event):
        if self._drag_at is not None:
            delta = event.position() - self._drag_at
            self.pan += delta
            self._drag_at = event.position()
            self.update()

    def mouseReleaseEvent(self, _event):
        self._drag_at = None

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#151b25"))

        # The map has no assumed arena rectangle. Fit the view around the robot
        # and accumulated evidence; detected obstacle cells eventually form the
        # wall boundary themselves.
        cell_size = self.model.cell_size_mm
        xs = [self.model.x]
        ys = [self.model.y]
        if self.model.trail:
            xs.extend(point[0] for point in self.model.trail)
            ys.extend(point[1] for point in self.model.trail)
        if self.model.cells:
            xs.extend(column * cell_size for column, _row in self.model.cells)
            ys.extend(row * cell_size for _column, row in self.model.cells)
        for wall in (
                self.model.wall_tracks + self.model.internal_wall_tracks):
            if wall["observations"] < 3:
                continue
            minimum, maximum = self.model.wall_segment(wall)
            if wall["orientation"] == "horizontal":
                xs.extend((minimum, maximum))
                ys.append(wall["coordinate"])
            else:
                xs.append(wall["coordinate"])
                ys.extend((minimum, maximum))
        for x0, y0, x1, y1, _source in self.model.current_rays:
            xs.extend((x0, x1))
            ys.extend((y0, y1))
        padding = 350.0
        min_x, max_x = min(xs) - padding, max(xs) + padding
        min_y, max_y = min(ys) - padding, max(ys) + padding
        if max_x - min_x < 1500.0:
            extra = (1500.0 - (max_x - min_x)) * 0.5
            min_x -= extra
            max_x += extra
        if max_y - min_y < 1500.0:
            extra = (1500.0 - (max_y - min_y)) * 0.5
            min_y -= extra
            max_y += extra
        world_width = max_x - min_x
        world_height = max_y - min_y
        scale = self.zoom * min((self.width() - 60) / world_width,
                                (self.height() - 60) / world_height)
        scale = max(scale, 0.01)

        def screen(x, y):
            return QPointF(30 + (x - min_x) * scale + self.pan.x(),
                           self.height() - 30 - (y - min_y) * scale + self.pan.y())

        bounds = QRectF(screen(min_x, max_y), screen(max_x, min_y)).normalized()
        col_min = math.floor(min_x / cell_size)
        col_max = math.ceil(max_x / cell_size)
        row_min = math.floor(min_y / cell_size)
        row_max = math.ceil(max_y / cell_size)
        cols = col_max - col_min
        rows = row_max - row_min
        p.setClipRect(bounds)
        key = (self.model.grid_generation, col_min, col_max, row_min, row_max)
        if cols * rows <= 1_000_000:
            if self._grid_key != key:
                self._grid_key = key
                self._grid_image = QImage(cols, rows, QImage.Format.Format_ARGB32)
                self._grid_image.fill(QColor(0, 0, 0, 0))
                self._painted = {}
            for cell, evidence in self.model.cells.items():
                votes = self.model.weight_votes.get(cell, 0)
                colour = "#a855f7" if votes >= 2 else (
                    "#dc2626" if evidence >= 4 else
                    "#16a34a" if evidence <= -3 else "#151b25")
                if self._painted.get(cell) == colour:
                    continue
                self._painted[cell] = colour
                col, row = cell
                if col_min <= col < col_max and row_min <= row < row_max:
                    self._grid_image.setPixelColor(col - col_min, row_max - 1 - row, QColor(colour))
            image_bounds = QRectF(screen(col_min * cell_size, row_max * cell_size),
                                  screen(col_max * cell_size, row_min * cell_size)).normalized()
            p.drawImage(image_bounds, self._grid_image)
        else:
            p.setPen(Qt.PenStyle.NoPen)
            for (column, row), evidence in self.model.cells.items():
                colour = "#a855f7" if self.model.weight_votes.get((column, row), 0) >= 2 else (
                    "#dc2626" if evidence >= 4 else
                    "#16a34a" if evidence <= -3 else "#151b25")
                p.setBrush(QColor(colour))
                x = column * cell_size
                y = row * cell_size
                p.drawRect(QRectF(screen(x, y + cell_size), screen(x + cell_size, y)).normalized())
        p.setClipping(False)
        p.setPen(QColor("#667489"))
        p.drawText(
            20, 23,
            "DISCOVERED MAP — fused: solid red = arena boundary; "
            "dashed orange = confirmed internal wall"
        )

        # Confirmed walls are drawn above the raw occupancy squares. Unlike
        # individual ray endpoints these landmarks remain fixed and extend as
        # later observations agree with the same physical wall.
        p.setPen(QPen(QColor("#ef4444"), 5, Qt.PenStyle.SolidLine,
                      Qt.PenCapStyle.RoundCap))
        for wall in self.model.wall_tracks:
            if wall["observations"] < 3:
                continue
            minimum, maximum = self.model.wall_segment(wall)
            if wall["orientation"] == "horizontal":
                start = screen(minimum, wall["coordinate"])
                end = screen(maximum, wall["coordinate"])
            else:
                start = screen(wall["coordinate"], minimum)
                end = screen(wall["coordinate"], maximum)
            p.drawLine(start, end)

        p.setPen(QPen(QColor("#f97316"), 4, Qt.PenStyle.DashLine,
                      Qt.PenCapStyle.RoundCap))
        for wall in self.model.internal_wall_tracks:
            if wall["observations"] < 3:
                continue
            minimum, maximum = self.model.wall_segment(wall)
            if wall["orientation"] == "horizontal":
                start = screen(minimum, wall["coordinate"])
                end = screen(maximum, wall["coordinate"])
            else:
                start = screen(wall["coordinate"], minimum)
                end = screen(wall["coordinate"], maximum)
            p.drawLine(start, end)

        if len(self.model.trail) > 1:
            p.setPen(QPen(QColor("#60a5fa"), 2))
            trail = QPolygonF([screen(x, y) for x, y in self.model.trail])
            p.drawPolyline(trail)
        p.setPen(QPen(QColor("#6b7280"), 1))
        for x0, y0, x1, y1, _source in self.model.current_rays:
            p.drawLine(screen(x0, y0), screen(x1, y1))

        robot = screen(self.model.x, self.model.y)
        theta = self.model.theta
        size = 19.0
        shape = QPolygonF([
            QPointF(robot.x() + size * math.cos(theta), robot.y() - size * math.sin(theta)),
            QPointF(robot.x() + size * 0.7 * math.cos(theta + 2.45),
                    robot.y() - size * 0.7 * math.sin(theta + 2.45)),
            QPointF(robot.x() + size * 0.7 * math.cos(theta - 2.45),
                    robot.y() - size * 0.7 * math.sin(theta - 2.45)),
        ])
        p.setPen(QPen(QColor("#e0f2fe"), 2))
        p.setBrush(QColor("#0ea5e9"))
        p.drawPolygon(shape)
        p.end()


class SensorLayoutCanvas(QWidget):
    """Drag sensor markers in robot coordinates; drag arrow tips to rotate."""

    selected = pyqtSignal(str)
    geometry_changed = pyqtSignal(str, str, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)
        self.specs = []
        self.aux_specs = []
        self.selected_name = None
        self.drag_mode = None

    def _all_specs(self):
        return self.specs + self.aux_specs

    def set_specs(self, specs):
        self.specs = specs
        all_specs = self._all_specs()
        if self.selected_name not in {item["name"] for item in all_specs}:
            self.selected_name = all_specs[0]["name"] if all_specs else None
        self.update()

    def set_aux_specs(self, specs):
        """Add extra draggable sensors without changing the primary TOF spec list."""
        self.aux_specs = specs
        all_specs = self._all_specs()
        if self.selected_name not in {item["name"] for item in all_specs}:
            self.selected_name = all_specs[0]["name"] if all_specs else None
        self.update()

    def _scale(self):
        return min((self.width() - 55) / 700.0, (self.height() - 60) / 800.0)

    def _marker(self, spec):
        if spec.get("kind") == "ultrasound":
            layer_shift = 0
        else:
            layer_shift = -6 if spec.get("layer") == "top" else 6
        return QPointF(self.width() / 2 + spec["x"] * self._scale() + layer_shift,
                       self.height() / 2 - spec["y"] * self._scale())

    def _tip(self, spec):
        point = self._marker(spec)
        radians = math.radians(spec["angle"])
        return QPointF(point.x() - 42 * math.sin(radians),
                       point.y() - 42 * math.cos(radians))

    def paintEvent(self, _event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor("#17202b"))
        centre = QPointF(self.width() / 2, self.height() / 2)
        scale = self._scale()
        body = QRectF(centre.x() - 190 * scale, centre.y() - 250 * scale,
                      380 * scale, 500 * scale)
        p.setPen(QPen(QColor("#94a3b8"), 2))
        p.setBrush(QColor("#253244"))
        p.drawRoundedRect(body, 15, 15)
        p.setPen(QPen(QColor("#93c5fd"), 2))
        p.drawLine(QPointF(centre.x(), body.top() + 24),
                   QPointF(centre.x(), body.top() - 14))
        p.drawText(int(centre.x() - 24), int(body.top() - 19), "FRONT")
        p.setPen(QColor("#94a3b8"))
        p.drawText(8, self.height() - 8, "Robot left ←                         → Robot right")
        for spec in self._all_specs():
            marker = self._marker(spec)
            tip = self._tip(spec)
            is_ultrasound = spec.get("kind") == "ultrasound"
            colour = QColor(
                "#facc15" if is_ultrasound
                else "#38bdf8" if spec.get("layer") == "top"
                else "#fb923c"
            )
            selected = spec["name"] == self.selected_name
            p.setPen(QPen(colour, 3 if selected else 2))
            p.drawLine(marker, tip)
            p.setBrush(colour)
            if is_ultrasound:
                size = 9 if selected else 7
                diamond = QPolygonF([
                    QPointF(marker.x(), marker.y() - size),
                    QPointF(marker.x() + size, marker.y()),
                    QPointF(marker.x(), marker.y() + size),
                    QPointF(marker.x() - size, marker.y()),
                ])
                p.drawPolygon(diamond)
            elif spec.get("layer") == "top":
                p.drawEllipse(marker, 8 if selected else 6, 8 if selected else 6)
            else:
                size = 8 if selected else 6
                p.drawRect(QRectF(marker.x() - size, marker.y() - size, size * 2, size * 2))
            if selected:
                p.setBrush(QColor("#f8fafc"))
                p.drawEllipse(tip, 5, 5)
            p.setPen(QColor("#e2e8f0"))
            p.drawText(int(marker.x() + 9), int(marker.y() - 10), spec["port"])
        p.end()

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton:
            return
        position = event.position()
        current = next((s for s in self._all_specs() if s["name"] == self.selected_name), None)
        if current and (position - self._tip(current)).manhattanLength() <= 17:
            self.drag_mode = "angle"
            return
        nearest = min(self._all_specs(),
                      key=lambda s: (position - self._marker(s)).manhattanLength(),
                      default=None)
        if nearest and (position - self._marker(nearest)).manhattanLength() <= 24:
            self.selected_name = nearest["name"]
            self.selected.emit(self.selected_name)
            self.drag_mode = "position"
            self.update()

    def mouseMoveEvent(self, event):
        if not self.drag_mode:
            return
        spec = next((s for s in self._all_specs() if s["name"] == self.selected_name), None)
        if spec is None:
            return
        position = event.position()
        if self.drag_mode == "angle":
            marker = self._marker(spec)
            angle = math.degrees(math.atan2(-(position.x() - marker.x()),
                                            -(position.y() - marker.y())))
            self.geometry_changed.emit(spec["name"], "angle", max(-180.0, min(180.0, angle)))
        else:
            if spec.get("kind") == "ultrasound":
                shift = 0
            else:
                shift = -6 if spec.get("layer") == "top" else 6
            lateral = (position.x() - self.width() / 2 - shift) / self._scale()
            forward = (self.height() / 2 - position.y()) / self._scale()
            self.geometry_changed.emit(spec["name"], "x", max(-500.0, min(500.0, lateral)))
            self.geometry_changed.emit(spec["name"], "y", max(-500.0, min(500.0, forward)))

    def mouseReleaseEvent(self, _event):
        self.drag_mode = None


class ArenaView(QWidget):
    command_requested = pyqtSignal(str, dict)
    parameter_requested = pyqtSignal(str, object)

    def __init__(self, settings, parent=None, show_controls=True):
        super().__init__(parent)
        self.settings = settings
        self.model = ArenaModel()
        self.sensor_unmatched = []
        self.model.matrix_mirrored = str(settings.value("sensor/mirror_8x8", "true")).lower() == "true"
        layout = QHBoxLayout(self)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        if show_controls:
            controls = QHBoxLayout()
            controls.setSpacing(8)
            controls.addWidget(QLabel("Arena controls:"))
            self.run_navigation_button = QPushButton("RUN NAVIGATION")
            self.stop_navigation_button = QPushButton("STOP NAVIGATION")
            self.run_motors_button = QPushButton("RUN MOTORS")
            self.stop_motors_button = QPushButton("STOP MOTORS")
            for button in (self.run_navigation_button, self.run_motors_button):
                button.setStyleSheet(
                    "QPushButton { background: #15803d; color: white; "
                    "font-weight: bold; padding: 7px 14px; }"
                )
            for button in (self.stop_navigation_button, self.stop_motors_button):
                button.setStyleSheet(
                    "QPushButton { background: #b91c1c; color: white; "
                    "font-weight: bold; padding: 7px 14px; }"
                )
            self.run_navigation_button.clicked.connect(
                self._run_navigation_with_tuning
            )
            self.stop_navigation_button.clicked.connect(
                lambda: self.command_requested.emit(
                    "autonomous_navigation", {"enabled": False})
            )
            self.run_motors_button.clicked.connect(
                lambda: self.command_requested.emit("run", {})
            )
            self.stop_motors_button.clicked.connect(
                lambda: self.command_requested.emit("stop", {})
            )
            controls.addWidget(self.run_navigation_button)
            controls.addWidget(self.stop_navigation_button)
            controls.addSpacing(16)
            controls.addWidget(self.run_motors_button)
            controls.addWidget(self.stop_motors_button)
            controls.addStretch()
            left_layout.addLayout(controls)
        self.canvas = ArenaCanvas(self.model, self)
        self._build_mission_editor()
        self.map_tabs = QTabWidget()
        self.map_tabs.addTab(self.canvas, "Live Arena")
        self.map_tabs.addTab(self.mission_page, "Mission Planner")
        left_layout.addWidget(self.map_tabs, 1)
        self._build_navigation_tuning_panel()
        left_layout.addWidget(self.navigation_tuning_tabs)
        self._build_raw_tof_panel()
        left_layout.addWidget(self.raw_tof_group)
        layout.addWidget(left, 1)
        side = QWidget()
        side.setMaximumWidth(330)
        form = QFormLayout(side)
        self.status = QLabel("Waiting for encoder, IMU, TOF and ultrasound telemetry")
        self.status.setWordWrap(True)
        form.addRow(self.status)

        def spin(key, label, default, maximum, decimals=0):
            control = QDoubleSpinBox()
            control.setRange(0, maximum)
            control.setDecimals(decimals)
            control.setSuffix(" mm" if "deg" not in key and "count" not in key else "")
            try:
                control.setValue(float(settings.value("arena/" + key, default)))
            except (TypeError, ValueError):
                control.setValue(default)
            control.valueChanged.connect(lambda value, k=key: self._changed(k, value))
            form.addRow(label, control)
            return control

        self.arena_width = spin("width", "Arena width", 2400, 50000)
        self.arena_height = spin("height", "Arena length", 4900, 50000)
        # Retain these values for old recording compatibility, but they no
        # longer constrain or draw the live discovered map.
        for control in (self.arena_width, self.arena_height):
            label = form.labelForField(control)
            if label is not None:
                label.hide()
            control.hide()
        # Powered straight-run calibration. The first two consistent trials
        # covered 775 mm over 8522/8080 counts. A third trial was rejected as
        # an approximately 12% distance outlier.
        if not settings.value("arena/dual_encoder_calibration_20260920", False, type=bool):
            settings.setValue("arena/encoder_1_mm_per_count", 775.0 / 8522.0)
            settings.setValue("arena/encoder_2_mm_per_count", 775.0 / 8080.0)
            settings.setValue("arena/invert_left", True)
            settings.setValue("arena/invert_right", False)
            settings.setValue("arena/dual_encoder_calibration_20260920", True)
        self.encoder_1_scale = spin(
            "encoder_1_mm_per_count", "Encoder 1 mm / count", 775.0 / 8522.0, 100, 5)
        self.encoder_2_scale = spin(
            "encoder_2_mm_per_count", "Encoder 2 mm / count", 775.0 / 8080.0, 100, 5)
        self.track_width = spin("track_width", "Wheel track", 0, 2000)
        if not settings.value("arena/wheel_track_40mm_migrated", False, type=bool):
            if self.track_width.value() == 0:
                self.track_width.blockSignals(True)
                self.track_width.setValue(40)
                self.track_width.blockSignals(False)
                settings.setValue("arena/track_width", 40)
            settings.setValue("arena/wheel_track_40mm_migrated", True)
        if not settings.value("arena/centimetre_grid_migrated", False, type=bool):
            old_size = _number(settings.value("arena/cell_size_mm"))
            if old_size is None or old_size == 100:
                settings.setValue("arena/cell_size_mm", 10)
            settings.setValue("arena/centimetre_grid_migrated", True)
        self.grid_size = spin("cell_size_mm", "Grid square size", 10, 500)
        self.grid_size.setMinimum(5)
        self.weight_gap = spin("weight_gap_mm", "Purple distance gap", 150, 1000)
        self.weight_gap.setMinimum(20)
        self.matrix_fov = spin("matrix_fov_deg", "8x8 horizontal FOV (°)", 60, 180, 1)
        self.sensor_canvas = SensorLayoutCanvas()
        self.sensor_canvas.geometry_changed.connect(self._sensor_canvas_changed)
        form.addRow(self.sensor_canvas)
        self.sensor_controls = {}
        self.sensor_group = None
        self.sensor_form_parent = form
        self.reload_sensor_layout()
        reload_button = QPushButton("Reload sensor ports from Wiring Guide")
        reload_button.clicked.connect(self.reload_sensor_layout)
        form.addRow(reload_button)
        self.invert_left = QCheckBox("Invert encoder 1")
        self.invert_right = QCheckBox("Invert encoder 2")
        for key, check in (("invert_left", self.invert_left), ("invert_right", self.invert_right)):
            check.setChecked(str(settings.value("arena/" + key, "false")).lower() == "true")
            check.toggled.connect(lambda value, k=key: self._changed(k, value))
            form.addRow(check)
        reset = QPushButton("Reset local map / set current pose as origin")
        reset.clicked.connect(self.reset_map)
        form.addRow(reset)
        reset_view = QPushButton("Reset map zoom / pan")
        reset_view.clicked.connect(self.reset_view)
        form.addRow(reset_view)
        note = QLabel("Grey: unknown · green: measured free · red: obstacle · purple: possible weight (bottom return, paired top sees farther). "
                      "Wheel over map to zoom; drag map to pan. In the robot diagram, drag a sensor to place it and drag its white arrow tip to aim it. "
                      "Cyan circles are top point TOFs; orange squares are bottom point TOFs; yellow diamonds are ultrasound. "
                      "All sensor markers can be dragged to place them and their arrow tips can be dragged to aim them. "
                      "No arena outline is assumed; detected red wall cells build the boundary. "
                      "Initial wheel scale assumes ~80 mm diameter, 663 PPR and 4× decoding; verify with a measured push. "
                      "Blue is the robot trail. Free/obstacle colours are sensor evidence, not confirmed walls. "
                      "Set wheel scale and directions before trusting the history. "
                      "The robot starts at the map centre, not a surveyed arena location.")
        note.setWordWrap(True)
        form.addRow(note)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(side)
        scroll.setMaximumWidth(410)
        self.side_scroll = scroll
        layout.addWidget(scroll)
        self.map_tabs.currentChanged.connect(self._arena_subtab_changed)
        self._arena_subtab_changed(self.map_tabs.currentIndex())
        self._sync()

    def _build_mission_editor(self):
        self.mission_layout = MissionLayout()
        try:
            saved = self.settings.value("navigation/mission_layout", "")
            if saved:
                self.mission_layout.load_dict(json.loads(str(saved)))
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

        self.mission_page = QWidget()
        page_layout = QVBoxLayout(self.mission_page)
        page_layout.setContentsMargins(5, 5, 5, 5)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(5)
        self.mission_home_combo = QComboBox()
        self.mission_home_combo.addItems(["Green home", "Blue home"])
        self.mission_home_combo.setCurrentIndex(
            0 if self.mission_layout.my_home == "green" else 1)
        self.mission_home_combo.currentIndexChanged.connect(self._mission_home_changed)
        toolbar.addWidget(QLabel("My home:"))
        toolbar.addWidget(self.mission_home_combo)

        self.mission_heading_combo = QComboBox()
        self.mission_heading_combo.addItems(["East →", "South ↓", "West ←", "North ↑"])
        self.mission_heading_combo.setCurrentIndex(
            int(round(self.mission_layout.start["heading_deg"] / 90.0)) % 4)
        self.mission_heading_combo.currentIndexChanged.connect(self._mission_heading_changed)
        toolbar.addWidget(QLabel("Start heading:"))
        toolbar.addWidget(self.mission_heading_combo)

        self.mission_fallback_combo = QComboBox()
        self.mission_fallback_combo.addItems(["Balanced", "Gap explorer", "Conservative"])
        self.mission_fallback_combo.setCurrentIndex(self.mission_layout.fallback_strategy)
        self.mission_fallback_combo.currentIndexChanged.connect(self._mission_fallback_changed)
        toolbar.addWidget(QLabel("After weights:"))
        toolbar.addWidget(self.mission_fallback_combo)
        toolbar.addStretch()
        page_layout.addLayout(toolbar)

        tools = QHBoxLayout()
        for label, mode in (("Set start", "start"), ("+ Weight", "weight"),
                            ("+ Dummy weight", "dummy"), ("+ Wall", "wall"),
                            ("+ Ramp", "ramp"), ("+ Tube", "tube")):
            button = QPushButton(label)
            button.clicked.connect(
                lambda _checked=False, m=mode: self.mission_canvas.set_add_mode(m))
            tools.addWidget(button)
        rotate = QPushButton("Rotate selected 90°")
        rotate.clicked.connect(lambda: self.mission_canvas.rotate_selected())
        tools.addWidget(rotate)
        delete = QPushButton("Delete selected")
        delete.clicked.connect(lambda: self.mission_canvas.delete_selected())
        tools.addWidget(delete)
        reset_view = QPushButton("Reset view")
        reset_view.clicked.connect(lambda: self.mission_canvas.reset_view())
        tools.addWidget(reset_view)
        plan = QPushButton("PLAN ROUTE")
        plan.setStyleSheet(
            "QPushButton { background:#0369a1; color:white; font-weight:bold; padding:6px 12px; }")
        plan.clicked.connect(self._plan_mission_route)
        tools.addWidget(plan)
        page_layout.addLayout(tools)

        body = QHBoxLayout()
        self.mission_canvas = MissionLayoutCanvas(
            self.mission_layout, self.model, self.mission_page)
        self.mission_canvas.layout_changed.connect(self._mission_layout_changed)
        self.mission_canvas.selection_changed.connect(self._mission_selection_changed)
        self.mission_canvas.cursor_moved.connect(self._mission_cursor_moved)
        body.addWidget(self.mission_canvas, 1)

        inspector = QWidget()
        inspector.setMaximumWidth(310)
        inspector_layout = QVBoxLayout(inspector)
        inspector_layout.setContentsMargins(4, 4, 4, 4)

        selection_group = QGroupBox("Selected obstacle dimensions")
        selection_form = QFormLayout(selection_group)
        self.mission_selected_label = QLabel("START")
        selection_form.addRow("Selected", self.mission_selected_label)
        self.mission_obstacle_width = QDoubleSpinBox()
        self.mission_obstacle_width.setRange(50, 4000)
        self.mission_obstacle_width.setDecimals(0)
        self.mission_obstacle_width.setSuffix(" mm")
        self.mission_obstacle_height = QDoubleSpinBox()
        self.mission_obstacle_height.setRange(50, 2400)
        self.mission_obstacle_height.setDecimals(0)
        self.mission_obstacle_height.setSuffix(" mm")
        self.mission_obstacle_width.valueChanged.connect(
            self._mission_obstacle_dimension_changed)
        self.mission_obstacle_height.valueChanged.connect(
            self._mission_obstacle_dimension_changed)
        selection_form.addRow("Length / diameter", self.mission_obstacle_width)
        selection_form.addRow("Thickness / depth", self.mission_obstacle_height)
        inspector_layout.addWidget(selection_group)

        visibility_group = QGroupBox("Live weight visibility")
        visibility_layout = QVBoxLayout(visibility_group)
        self.mission_weight_visibility = QLabel(
            "Waiting for weight-visibility telemetry")
        self.mission_weight_visibility.setWordWrap(True)
        self.mission_weight_visibility.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mission_weight_visibility.setMinimumHeight(74)
        visibility_layout.addWidget(self.mission_weight_visibility)
        inspector_layout.addWidget(visibility_group)

        self.mission_cursor_label = QLabel("Cursor: —")
        self.mission_cursor_label.setWordWrap(True)
        inspector_layout.addWidget(self.mission_cursor_label)
        guide = QLabel(
            "Seven dashed vertical placement lines split the 4.9 m arena into eight equal bays. "
            "Walls/ramps/tubes and dummy weights are inflated by robot radius + safety margin before A*. "
            "Changing a wall dimension invalidates the old route; Plan Route recalculates around it.")
        guide.setWordWrap(True)
        inspector_layout.addWidget(guide)
        inspector_layout.addStretch()
        body.addWidget(inspector)
        page_layout.addLayout(body, 1)

        self.mission_status = QLabel(
            "Pre-lay the arena: fixed border; green home top-left; blue home bottom-left. "
            "The other home is NO-GO. Add real/dummy weights and obstacles, set START, then Plan Route.")
        self.mission_status.setWordWrap(True)
        page_layout.addWidget(self.mission_status)
        self._mission_selection_changed("start")
        self._refresh_mission_weight_visibility()

    def _arena_subtab_changed(self, _index):
        mission_visible = (
            hasattr(self, "map_tabs") and hasattr(self, "mission_page") and
            self.map_tabs.currentWidget() is self.mission_page
        )
        # Mission Planner gets the full Arena View working area while editing.
        if hasattr(self, "navigation_tuning_tabs"):
            self.navigation_tuning_tabs.setVisible(not mission_visible)
        if hasattr(self, "raw_tof_group"):
            self.raw_tof_group.setVisible(not mission_visible)
        if hasattr(self, "side_scroll"):
            self.side_scroll.setVisible(not mission_visible)
        if mission_visible and hasattr(self, "mission_canvas"):
            self.mission_canvas.update()

    def _mission_cursor_moved(self, x, y):
        if not hasattr(self, "mission_cursor_label"):
            return
        if (0.0 <= x <= self.mission_layout.WIDTH_MM and
                0.0 <= y <= self.mission_layout.HEIGHT_MM):
            self.mission_cursor_label.setText(
                f"Cursor: x={x:.0f} mm, y={y:.0f} mm")
        else:
            self.mission_cursor_label.setText("Cursor: outside arena")

    def _mission_selection_changed(self, ident):
        if not hasattr(self, "mission_obstacle_width"):
            return
        obstacle = next(
            (item for item in self.mission_layout.obstacles
             if item.get("id") == ident), None)
        self.mission_obstacle_width.blockSignals(True)
        self.mission_obstacle_height.blockSignals(True)
        try:
            if obstacle is None:
                if ident == "start":
                    label = "START"
                else:
                    weight = next(
                        (item for item in self.mission_layout.weights
                         if item.get("id") == ident), None)
                    label = ("DUMMY WEIGHT" if weight and weight.get("dummy") else
                             "WEIGHT" if weight else "Nothing selected")
                self.mission_selected_label.setText(label)
                self.mission_obstacle_width.setEnabled(False)
                self.mission_obstacle_height.setEnabled(False)
                return
            self.mission_selected_label.setText(
                f"{obstacle['kind'].upper()} #{obstacle['id']}")
            self.mission_obstacle_width.setEnabled(True)
            self.mission_obstacle_width.setValue(float(obstacle.get("w", 100.0)))
            is_tube = obstacle.get("kind") == "tube"
            self.mission_obstacle_height.setEnabled(not is_tube)
            self.mission_obstacle_height.setValue(
                float(obstacle.get("h", obstacle.get("w", 100.0))))
        finally:
            self.mission_obstacle_width.blockSignals(False)
            self.mission_obstacle_height.blockSignals(False)

    def _mission_obstacle_dimension_changed(self, _value=None):
        if not hasattr(self, "mission_canvas"):
            return
        ident = self.mission_canvas.selected_id
        obstacle = next(
            (item for item in self.mission_layout.obstacles
             if item.get("id") == ident), None)
        if obstacle is None:
            return
        had_route = bool(self.mission_layout.route)
        width = self.mission_obstacle_width.value()
        height = (width if obstacle.get("kind") == "tube"
                  else self.mission_obstacle_height.value())
        if not self.mission_layout.set_obstacle_size(ident, width, height):
            return
        self._save_mission_layout()
        self.mission_canvas.update()
        self.mission_status.setText(
            f"{obstacle['kind'].title()} resized to {width:.0f} × {height:.0f} mm. "
            "Route geometry updated." if obstacle.get("kind") != "tube" else
            f"Tube resized to Ø{width:.0f} mm. Route geometry updated.")
        if had_route:
            self._plan_mission_route()

    def _refresh_mission_weight_visibility(self):
        if not hasattr(self, "mission_weight_visibility"):
            return
        latest = self.model.latest
        if "weight.detected" not in latest:
            self.mission_weight_visibility.setText(
                "WAITING FOR V10 WEIGHT TELEMETRY\nNo live confirmation yet")
            self.mission_weight_visibility.setStyleSheet(
                "QLabel { background:#334155; color:white; padding:10px; font-weight:bold; }")
            return
        detected = latest.get("weight.detected") is True
        mask = int(_number(latest.get("weight.sector_mask")) or 0)
        nearest = int(_number(latest.get("weight.nearest_mm")) or 0)
        direction = int(_number(latest.get("weight.direction")) or 0)
        direction_text = {-2: "far left", -1: "left", 1: "right", 2: "far right"}.get(direction, "unknown")
        if detected:
            active = [name for bit, name in enumerate(
                ("far-left", "mid-left", "mid-right", "far-right"))
                if mask & (1 << bit)]
            self.mission_weight_visibility.setText(
                "WEIGHT VISIBLE\n"
                f"{nearest} mm · {direction_text} · sectors: {', '.join(active) or '—'}")
            self.mission_weight_visibility.setStyleSheet(
                "QLabel { background:#14532d; color:#dcfce7; padding:10px; font-weight:bold; }")
        else:
            self.mission_weight_visibility.setText(
                "NO WEIGHT CURRENTLY VISIBLE\n4-sector TOF confirmation is clear")
            self.mission_weight_visibility.setStyleSheet(
                "QLabel { background:#1e293b; color:#cbd5e1; padding:10px; font-weight:bold; }")

    def _save_mission_layout(self):
        self.settings.setValue(
            "navigation/mission_layout",
            json.dumps(self.mission_layout.to_dict(), separators=(",", ":")),
        )

    def _mission_layout_changed(self):
        self.mission_layout.route = []
        self._save_mission_layout()
        self.mission_status.setText("Layout changed — press PLAN ROUTE to recalculate around the new geometry.")
        self.mission_canvas.update()

    def _mission_home_changed(self, index):
        self.mission_layout.my_home = "green" if index == 0 else "blue"
        self._mission_layout_changed()

    def _mission_heading_changed(self, index):
        self.mission_layout.start["heading_deg"] = float((index % 4) * 90)
        self._mission_layout_changed()

    def _mission_fallback_changed(self, index):
        self.mission_layout.fallback_strategy = max(0, min(2, int(index)))
        self._save_mission_layout()

    def _navigation_strategy_changed(self, index):
        if (hasattr(self, "map_tabs") and index == 3 and
                self.map_tabs.indexOf(self.mission_page) >= 0):
            self.map_tabs.setCurrentWidget(self.mission_page)

    def _plan_mission_route(self):
        # Use the same physical footprint/safety margin that will be sent to
        # firmware so the graphical route and the real clearance agree.
        self.mission_layout.ROBOT_RADIUS_MM = (
            self.navigation_tuning_controls["robot_width_mm"].value() * 0.5
        )
        self.mission_layout.SAFETY_MARGIN_MM = max(
            40.0, self.navigation_tuning_controls["gap_margin_mm"].value()
        )
        if self.mission_layout.plan():
            target_count = sum(1 for item in self.mission_layout.weights if not item["dummy"])
            length = MissionLayout._path_length(
                [(self.mission_layout.start["x"], self.mission_layout.start["y"])] +
                [(point["x"], point["y"]) for point in self.mission_layout.route]
            )
            self.mission_status.setText(
                f"Route ready: {target_count} real weights, {len(self.mission_layout.route)} waypoints, "
                f"approximately {length/1000.0:.1f} m. RUN NAVIGATION will upload this route first."
            )
        else:
            self.mission_status.setText("ROUTE ERROR: " + self.mission_layout.route_error)
        self.mission_canvas.update()
        return bool(self.mission_layout.route)

    def _build_navigation_tuning_panel(self):
        self.navigation_tuning_tabs = QTabWidget()
        self.navigation_tuning_tabs.setMaximumHeight(245)
        page = QWidget()
        grid = QGridLayout(page)
        self.navigation_strategy = QComboBox()
        self.navigation_strategy.addItems([
            "Balanced coverage", "Gap explorer", "Conservative wall-safe",
            "Pre-laid mission"
        ])
        self.navigation_strategy.setCurrentIndex(max(0, min(
            3, int(self.settings.value("navigation/strategy", 0)))))
        self.navigation_strategy.currentIndexChanged.connect(
            self._navigation_strategy_changed
        )
        grid.addWidget(QLabel("Algorithm"), 0, 0)
        grid.addWidget(self.navigation_strategy, 0, 1)
        definitions = [
            ("front_avoid_mm", "Front stop", 300, 100, 1000),
            ("side_avoid_mm", "Side clearance", 200, 80, 800),
            ("wall_follow_mm", "Wall-follow distance", 200, 80, 1000),
            ("lane_spacing_mm", "Sweep spacing", 200, 80, 1000),
            ("robot_width_mm", "Robot width", 430, 100, 1200),
            ("gap_margin_mm", "Gap safety margin", 80, 0, 500),
            ("gap_depth_mm", "Open-gap depth", 550, 250, 3000),
            ("matrix_floor_rows", "Ignore bottom 8×8 rows", 2, 0, 4),
            ("matrix_fov_deg", "8×8 horizontal FOV", 40, 20, 90),
            ("gap_confirm_frames", "Gap confirmation frames", 2, 1, 5),
        ]
        self.navigation_tuning_controls = {}
        for index, (name, label, default, minimum, maximum) in enumerate(definitions):
            control = QDoubleSpinBox()
            control.setDecimals(0)
            control.setRange(minimum, maximum)
            control.setValue(float(self.settings.value("navigation/" + name, default)))
            control.setSuffix(" mm" if name.endswith("_mm") else "")
            row = 1 + index // 3
            col = (index % 3) * 2
            grid.addWidget(QLabel(label), row, col)
            grid.addWidget(control, row, col + 1)
            self.navigation_tuning_controls[name] = control
        apply_button = QPushButton("APPLY NAVIGATION TUNING")
        apply_button.clicked.connect(self._apply_navigation_tuning)
        grid.addWidget(apply_button, 5, 0, 1, 6)
        self.navigation_tuning_tabs.addTab(page, "Navigation tuning")

    def _run_navigation_with_tuning(self):
        # RUN NAVIGATION is transactional: first push the visible tuning. The
        # pre-laid strategy additionally computes/uploads its path before the
        # normal autonomous_navigation command is sent.
        self._apply_navigation_tuning()
        if self.navigation_strategy.currentIndex() == 3:
            if not self.mission_layout.route and not self._plan_mission_route():
                self.map_tabs.setCurrentWidget(self.mission_page)
                return
            payload = self.mission_layout.command_payload()
            if len(payload["points"]) // 3 > MissionLayout.MAX_ROUTE_POINTS:
                self.mission_status.setText("ROUTE ERROR: too many firmware waypoints.")
                self.map_tabs.setCurrentWidget(self.mission_page)
                return
            self.command_requested.emit("mission_plan_set", payload)
        self.command_requested.emit(
            "autonomous_navigation", {"enabled": True}
        )

    def _apply_navigation_tuning(self):
        strategy = self.navigation_strategy.currentIndex()
        self.settings.setValue("navigation/strategy", strategy)
        self.parameter_requested.emit("navigation.strategy", strategy)
        for name, control in self.navigation_tuning_controls.items():
            value = int(control.value())
            self.settings.setValue("navigation/" + name, value)
            self.parameter_requested.emit("navigation." + name, value)
        self.model.matrix_floor_rows = int(
            self.navigation_tuning_controls["matrix_floor_rows"].value())
        self.model.matrix_fov_deg = float(
            self.navigation_tuning_controls["matrix_fov_deg"].value())

    def _build_raw_tof_panel(self):
        """Build the always-visible raw range-sensor readout."""
        self.raw_tof_group = QGroupBox("Raw range readings — values before mapping filters")
        self.raw_tof_group.setMaximumHeight(285)
        panel = QHBoxLayout(self.raw_tof_group)

        point_box = QGroupBox("Point TOF sensors")
        self.raw_point_grid = QGridLayout(point_box)
        self.raw_point_grid.setColumnStretch(0, 1)
        self.raw_point_labels = {}
        self.raw_point_state = {}
        panel.addWidget(point_box, 2)

        ultrasound_box = QGroupBox("Ultrasound sensors")
        self.raw_ultrasound_grid = QGridLayout(ultrasound_box)
        self.raw_ultrasound_grid.setColumnStretch(0, 1)
        self.raw_ultrasound_labels = {}
        self.raw_ultrasound_state = {}
        panel.addWidget(ultrasound_box, 2)

        matrix_box = QGroupBox("8×8 TOF — raw millimetres")
        matrix_layout = QVBoxLayout(matrix_box)
        self.raw_matrix_status = QLabel("Waiting for an 8×8 frame")
        matrix_layout.addWidget(self.raw_matrix_status)
        matrix_grid = QGridLayout()
        matrix_grid.setSpacing(2)
        corner = QLabel("Y\\X")
        corner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        matrix_grid.addWidget(corner, 0, 0)
        for col in range(8):
            header = QLabel(f"X{col}")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            matrix_grid.addWidget(header, 0, col + 1)
        self.raw_matrix_cells = []
        for row in range(8):
            header = QLabel(f"Y{row}")
            header.setAlignment(Qt.AlignmentFlag.AlignCenter)
            matrix_grid.addWidget(header, row + 1, 0)
            row_cells = []
            for col in range(8):
                cell = QLabel("—")
                cell.setAlignment(Qt.AlignmentFlag.AlignCenter)
                cell.setMinimumWidth(37)
                cell.setStyleSheet(
                    "QLabel { background: #374151; color: white; "
                    "border: 1px solid #111827; padding: 1px; }"
                )
                matrix_grid.addWidget(cell, row + 1, col + 1)
                row_cells.append(cell)
            self.raw_matrix_cells.append(row_cells)
        matrix_layout.addLayout(matrix_grid)
        panel.addWidget(matrix_box, 3)

    def _rebuild_raw_point_rows(self):
        while self.raw_point_grid.count():
            item = self.raw_point_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self.raw_point_labels = {}
        for row, spec in enumerate(self.model.sensor_specs):
            name = spec["name"]
            title = QLabel(f"{spec.get('label', name)} ({spec.get('port', '?')})")
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.raw_point_grid.addWidget(title, row, 0)
            self.raw_point_grid.addWidget(value, row, 1)
            self.raw_point_labels[name] = value
            self._refresh_raw_point(name)

    def _refresh_raw_point(self, name):
        label = self.raw_point_labels.get(name)
        if label is None:
            return
        state = self.raw_point_state.get(name, {})
        available = state.get("available")
        timed_out = state.get("timed_out")
        distance = _number(state.get("distance_mm"))
        if available is False:
            text, colour = "OFFLINE", "#ef4444"
        elif timed_out is True:
            text, colour = "TIMEOUT", "#f59e0b"
        elif distance is None:
            text, colour = "—", "#9ca3af"
        elif not self.model.min_range_mm <= distance <= self.model.max_range_mm:
            text, colour = f"{distance:g} mm · invalid", "#f59e0b"
        else:
            text, colour = f"{distance:g} mm", "#22c55e"
        label.setText(text)
        label.setStyleSheet(f"QLabel {{ color: {colour}; font-weight: bold; }}")

    def _rebuild_raw_ultrasound_rows(self):
        while self.raw_ultrasound_grid.count():
            item = self.raw_ultrasound_grid.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self.raw_ultrasound_labels = {}
        for row, spec in enumerate(self.model.ultrasound_specs):
            telemetry_name = spec["telemetry_name"]
            title = QLabel(f"{spec.get('label', telemetry_name)} ({spec.get('port', '?')})")
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.raw_ultrasound_grid.addWidget(title, row, 0)
            self.raw_ultrasound_grid.addWidget(value, row, 1)
            self.raw_ultrasound_labels[telemetry_name] = value
            self._refresh_raw_ultrasound(telemetry_name)

    def _refresh_raw_ultrasound(self, name):
        label = self.raw_ultrasound_labels.get(name)
        if label is None:
            return
        state = self.raw_ultrasound_state.get(name, {})
        valid = state.get("valid")
        timed_out = state.get("timed_out")
        distance = _number(state.get("distance_mm"))
        echo_us = _number(state.get("echo_us"))
        rise_count = _number(state.get("rise_count"))
        fall_count = _number(state.get("fall_count"))

        if timed_out is True:
            text, colour = "TIMEOUT", "#f59e0b"
        elif valid is True and distance is not None:
            details = [f"{distance:g} mm"]
            if echo_us is not None:
                details.append(f"{echo_us:g} µs")
            if rise_count is not None and fall_count is not None:
                details.append(f"edges {int(rise_count)}/{int(fall_count)}")
            if not self.model.min_range_mm <= distance <= self.model.max_range_mm:
                details.append("outside map")
                colour = "#f59e0b"
            else:
                colour = "#22c55e"
            text = " · ".join(details)
        elif valid is False:
            text, colour = "NO VALID RANGE", "#9ca3af"
        else:
            text, colour = "—", "#9ca3af"

        label.setText(text)
        label.setStyleSheet(f"QLabel {{ color: {colour}; font-weight: bold; }}")

    def _refresh_raw_matrix(self, message):
        available = bool(message.get("available"))
        valid = bool(message.get("valid"))
        values = message.get("data")
        if not available or not valid or not isinstance(values, list) or len(values) != 64:
            self.raw_matrix_status.setText(
                "Unavailable" if not available else "Latest frame is invalid"
            )
            for row_cells in self.raw_matrix_cells:
                for cell in row_cells:
                    cell.setText("—")
            return
        address = _number(message.get("address"))
        address_text = f"0x{int(address):02X}" if address is not None else "?"
        self.raw_matrix_status.setText(
            f"Frame {message.get('frame', '?')} · {message.get('bus', 'I2C')} · "
            f"address {address_text}"
        )
        for row in range(8):
            for col in range(8):
                value = _number(values[row * 8 + col])
                self.raw_matrix_cells[row][col].setText("—" if value is None else f"{value:g}")

    def reload_sensor_layout(self):
        """Join firmware telemetry names to the user's persisted port guide."""
        config_path = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            config = {}
        configured = config.get("tof_sensors", [])
        configured_ultrasound = config.get("ultrasound_sensors", [])
        try:
            guide = json.loads(str(self.settings.value("wiring_guide/entries_v1", "[]")))
        except (TypeError, ValueError):
            guide = []
        by_port = {}
        ultrasound_labels = {}
        for entry in guide if isinstance(guide, list) else []:
            if not isinstance(entry, dict) or str(entry.get("kind", "")).lower() != "sensor":
                continue
            port = str(entry.get("connector", "")).strip().upper()
            device = str(entry.get("device", "")).strip()
            pins = str(entry.get("pins", "")).upper()
            if re.fullmatch(r"XSHUT[0-7]", port) and "tof" in device.lower():
                by_port[port] = device
            if "ultrasound" in device.lower():
                lower_device = device.lower()
                display_device = device
                if lower_device == "ultrasound a":
                    display_device = "Ultrasound Left (A)"
                elif lower_device == "ultrasound b":
                    display_device = "Ultrasound Right (B)"
                if ("(a)" in lower_device or lower_device.endswith(" a") or
                        ("D14" in pins and "D24" in pins)):
                    ultrasound_labels["a"] = display_device
                elif ("(b)" in lower_device or lower_device.endswith(" b") or
                      ("D22" in pins and "D20" in pins)):
                    ultrasound_labels["b"] = display_device

        if self.sensor_group is not None:
            self.sensor_form_parent.removeRow(self.sensor_group)
        self.sensor_group = QGroupBox("Range sensors — robot point of view")
        sensor_form = QFormLayout(self.sensor_group)
        self.sensor_controls = {}
        specs = []
        configured_ports = set()
        for sensor in configured if isinstance(configured, list) else []:
            if not isinstance(sensor, dict):
                continue
            port = str(sensor.get("port", "")).strip().upper()
            name = str(sensor.get("name", "")).strip()
            if not name or not re.fullmatch(r"XSHUT[0-7]", port):
                continue
            configured_ports.add(port)
            label = by_port.get(port, str(sensor.get("label", name)))
            lower = label.lower()
            lateral = -100 if "left" in lower else (180 if "right right" in lower else 100 if "right" in lower else 0)
            forward = 120  # top/bottom describes height, not forward/back position
            layer_default = "bottom" if "bottom" in lower else "top"
            layer = str(self.settings.value(f"arena/sensors/{name}/layer", layer_default)).lower()
            if layer not in ("top", "bottom"):
                layer = layer_default
            spec = {"name": name, "port": port, "label": label,
                    "angle": 0.0, "x": float(lateral), "y": float(forward), "layer": layer}
            sensor_form.addRow(QLabel(f"{label} ({port}, {sensor.get('type', '?')})"))
            controls = {}
            for field, caption, default in (
                ("angle", "Angle (°)", 0),
                ("x", "Right offset (mm)", lateral),
                ("y", "Forward offset (mm)", forward),
            ):
                control = QDoubleSpinBox()
                control.setRange(-180 if field == "angle" else -1000,
                                 180 if field == "angle" else 1000)
                control.setDecimals(1)
                key = f"arena/sensors/{name}/{field}"
                try:
                    value = float(self.settings.value(key, default))
                except (TypeError, ValueError):
                    value = float(default)
                control.setValue(value)
                spec[field] = control.value()
                control.valueChanged.connect(
                    lambda new_value, s=spec, f=field, k=key: self._sensor_geometry_changed(s, f, k, new_value)
                )
                sensor_form.addRow(caption, control)
                controls[field] = control
            layer_combo = QComboBox()
            layer_combo.addItems(["top", "bottom"])
            layer_combo.setCurrentText(layer)
            layer_combo.currentTextChanged.connect(
                lambda value, s=spec: self._sensor_layer_changed(s, value)
            )
            sensor_form.addRow("Height layer", layer_combo)
            controls["layer"] = layer_combo
            self.sensor_controls[name] = controls
            specs.append(spec)

        # Ultrasound channels are ordinary draggable range sensors in the same
        # robot-layout canvas. They retain their own telemetry names (a/b), while
        # the user-facing labels come from the Wiring Guide when available.
        ultrasound_specs = []
        for index, sensor in enumerate(
                configured_ultrasound if isinstance(configured_ultrasound, list) else []):
            if not isinstance(sensor, dict):
                continue
            telemetry_name = str(sensor.get("name", "")).strip()
            if not telemetry_name:
                continue
            control_name = f"ultrasound.{telemetry_name}"
            trigger_pin = sensor.get("trigger_pin", "?")
            echo_pin = sensor.get("echo_pin", "?")
            default_label = (
                "Ultrasound Left (A)" if telemetry_name.lower() == "a"
                else "Ultrasound Right (B)" if telemetry_name.lower() == "b"
                else str(sensor.get("label", f"Ultrasound {telemetry_name.upper()}"))
            )
            label = ultrasound_labels.get(telemetry_name.lower(), default_label)
            lateral_default = -90 if index == 0 else 90
            forward_default = 180
            spec = {
                "name": control_name,
                "telemetry_name": telemetry_name,
                "kind": "ultrasound",
                "port": f"US-{telemetry_name.upper()}",
                "label": label,
                "angle": 0.0,
                "x": float(lateral_default),
                "y": float(forward_default),
                "layer": "top",
            }
            sensor_form.addRow(QLabel(
                f"{label} (TRIG D{trigger_pin}, ECHO D{echo_pin})"
            ))
            controls = {}
            for field, caption, default in (
                ("angle", "Angle (°)", 0),
                ("x", "Right offset (mm)", lateral_default),
                ("y", "Forward offset (mm)", forward_default),
            ):
                control = QDoubleSpinBox()
                control.setRange(-180 if field == "angle" else -1000,
                                 180 if field == "angle" else 1000)
                control.setDecimals(1)
                key = f"arena/sensors/{control_name}/{field}"
                try:
                    value = float(self.settings.value(key, default))
                except (TypeError, ValueError):
                    value = float(default)
                control.setValue(value)
                spec[field] = control.value()
                control.valueChanged.connect(
                    lambda new_value, s=spec, f=field, k=key:
                    self._sensor_geometry_changed(s, f, k, new_value)
                )
                sensor_form.addRow(caption, control)
                controls[field] = control
            self.sensor_controls[control_name] = controls
            ultrasound_specs.append(spec)

        matrix_entry = next((entry for entry in guide if isinstance(entry, dict) and
                             "8x8" in str(entry.get("device", "")).lower()), None)
        matrix_label = str(matrix_entry.get("device", "SEN0628 8x8 TOF")) if matrix_entry else "SEN0628 8x8 TOF"
        matrix = {"name": "matrix", "port": "I2C1", "label": matrix_label,
                  "angle": 0.0, "x": 0.0, "y": 150.0, "layer": "top"}
        sensor_form.addRow(QLabel(f"{matrix_label} (I2C1)"))
        matrix_controls = {}
        for field, caption, default in (("angle", "Angle (°)", 0),
                                        ("x", "Right offset (mm)", 0),
                                        ("y", "Forward offset (mm)", 150)):
            control = QDoubleSpinBox()
            control.setRange(-180 if field == "angle" else -1000,
                             180 if field == "angle" else 1000)
            control.setDecimals(1)
            key = f"arena/sensors/matrix/{field}"
            try:
                control.setValue(float(self.settings.value(key, default)))
            except (TypeError, ValueError):
                control.setValue(default)
            matrix[field] = control.value()
            control.valueChanged.connect(
                lambda value, s=matrix, f=field, k=key: self._sensor_geometry_changed(s, f, k, value)
            )
            sensor_form.addRow(caption, control)
            matrix_controls[field] = control
        matrix_layer = QComboBox()
        matrix_layer.addItems(["top", "bottom"])
        matrix_layer.setCurrentText(str(self.settings.value("arena/sensors/matrix/layer", "top")))
        matrix["layer"] = matrix_layer.currentText()
        matrix_layer.currentTextChanged.connect(lambda value, s=matrix: self._sensor_layer_changed(s, value))
        sensor_form.addRow("Height layer", matrix_layer)
        matrix_controls["layer"] = matrix_layer
        self.sensor_controls["matrix"] = matrix_controls
        self.sensor_form_parent.addRow(self.sensor_group)
        self.model.sensor_specs = specs
        self.model.ultrasound_specs = ultrasound_specs
        self.model.matrix_spec = matrix
        self.sensor_canvas.set_specs(specs + [matrix])
        self.sensor_canvas.set_aux_specs(ultrasound_specs)
        self._rebuild_raw_point_rows()
        self._rebuild_raw_ultrasound_rows()
        self.sensor_unmatched = sorted(set(by_port) - configured_ports)
        self.reset_map()

    def _sensor_geometry_changed(self, spec, field, key, value):
        spec[field] = value
        self.settings.setValue(key, value)
        self.sensor_canvas.update()
        self.reset_map()

    def _sensor_layer_changed(self, spec, layer):
        spec["layer"] = layer
        self.settings.setValue(f"arena/sensors/{spec['name']}/layer", layer)
        self.sensor_canvas.update()
        self.reset_map()

    def _sensor_canvas_changed(self, name, field, value):
        control = self.sensor_controls.get(name, {}).get(field)
        if control is not None:
            control.setValue(value)

    def reset_view(self):
        self.canvas.zoom = 1.0
        self.canvas.pan = QPointF(0, 0)
        self.canvas.update()

    def _sync(self):
        m = self.model
        m.encoder_1_mm_per_count = self.encoder_1_scale.value()
        m.encoder_2_mm_per_count = self.encoder_2_scale.value()
        m.mm_per_count = (
            m.encoder_1_mm_per_count + m.encoder_2_mm_per_count
        ) * 0.5
        m.track_width_mm = self.track_width.value()
        m.cell_size_mm = self.grid_size.value()
        m.weight_gap_mm = self.weight_gap.value()
        m.matrix_fov_deg = self.matrix_fov.value()
        m.invert_left = self.invert_left.isChecked()
        m.invert_right = self.invert_right.isChecked()
        self._refresh_status()

    def _changed(self, key, value):
        self.settings.setValue("arena/" + key, value)
        # Changing geometric calibration invalidates registration of old points.
        if key not in ("width", "height"):
            self.reset_map()
        self._sync()
        self.canvas.update()

    def reset_map(self):
        self.model.reset()
        self.canvas.update()
        self._refresh_status()

    def receive_telemetry(self, name, value, timestamp):
        match = re.fullmatch(r"tof\.([^.]+)\.(available|timed_out|distance_mm)", name)
        if match:
            sensor_name, field = match.groups()
            self.raw_point_state.setdefault(sensor_name, {})[field] = value
            self._refresh_raw_point(sensor_name)

        ultrasound_match = re.fullmatch(
            r"ultrasound\.([^.]+)\.(valid|timed_out|distance_mm|echo_us|"
            r"trigger_count|rise_count|fall_count|echo_high)",
            name,
        )
        if ultrasound_match:
            sensor_name, field = ultrasound_match.groups()
            self.raw_ultrasound_state.setdefault(sensor_name, {})[field] = value
            self._refresh_raw_ultrasound(sensor_name)

        old_time = self.model._frame_time
        self.model.receive_telemetry(name, value, timestamp)
        if old_time is not None and old_time != timestamp:
            self._refresh_status()
            self.canvas.update()
            if hasattr(self, "mission_canvas"):
                self.mission_canvas.update()

    def receive_matrix(self, message):
        self._refresh_raw_matrix(message)
        self.model.receive_matrix(message)
        self._refresh_status()
        self.canvas.update()
        if hasattr(self, "mission_canvas"):
            self.mission_canvas.update()

    def set_matrix_mirrored(self, mirrored):
        if self.model.matrix_mirrored != mirrored:
            self.model.matrix_mirrored = mirrored
            self.reset_map()

    def _refresh_status(self):
        m = self.model
        calibration = "?" if m.last_imu_calibration is None else str(m.last_imu_calibration)
        accuracy = ("History disabled until both encoder scales are set. "
                    if m.encoder_1_mm_per_count <= 0 or m.encoder_2_mm_per_count <= 0 else "")
        old_firmware = (any(key.startswith("tof.front.") or key.startswith("tof.left.")
                            for key in m.latest) and "imu.available" not in m.latest)
        firmware_warning = (
            "Firmware mismatch: robot still sends old front/left TOF names and no IMU; "
            "upload the current PlatformIO firmware. " if old_firmware else ""
        )
        controller_version = _number(m.latest.get("system.navigation_controller_version"))
        if ("system.uptime_s" in m.latest and controller_version != 10):
            firmware_warning += (
                "Navigation controller v10 is not running; upload the current clean build. "
            )
        online = sum(m.latest.get(f"tof.{spec['name']}.available") is True
                     for spec in m.sensor_specs)
        ultrasound_valid = sum(
            m.latest.get(f"ultrasound.{spec['telemetry_name']}.valid") is True
            for spec in m.ultrasound_specs
        )
        missing = (f" Wiring Guide ports not active in firmware: {', '.join(self.sensor_unmatched)}."
                   if self.sensor_unmatched else "")
        motion_warning = (
            "WHEEL SLIP / POINT TURN: map translation held. "
            if m.wheel_slip_detected else
            ("ENCODER/IMU DISAGREEMENT: map translation held. "
             if m.encoder_imu_disagreement else "")
        )
        self._refresh_mission_weight_visibility()
        if hasattr(self, "mission_status") and m.latest.get("mission.active") is True:
            visited = int(_number(m.latest.get("mission.targets_visited")) or 0)
            total = int(_number(m.latest.get("mission.target_count")) or 0)
            waypoint = int(_number(m.latest.get("mission.waypoint_index")) or 0)
            waypoint_count = int(_number(m.latest.get("mission.waypoint_count")) or 0)
            blocked = m.latest.get("mission.blocked") is True
            self.mission_status.setText(
                f"MISSION LIVE — weights {visited}/{total}; waypoint "
                f"{waypoint + 1 if waypoint < waypoint_count else waypoint_count}/{waypoint_count}. "
                + ("Live ranging is blocking the route; robot is holding position."
                   if blocked else "Robot is correcting toward the uploaded route.")
            )
        self.status.setText(
            f"{firmware_warning}{accuracy}{motion_warning}Pose: {m.x:.0f}, {m.y:.0f} mm; heading {math.degrees(m.theta):.1f}°. "
            f"Travel: {m.distance_travelled_mm:.0f} mm; speed: {m.linear_speed_mm_s:.0f} mm/s. "
            f"IMU fusion {'running' if m.last_imu_fusion_running is True else 'not ready'}; "
            f"system calibration {calibration}/3; status/error "
            f"{m.last_imu_system_status if m.last_imu_system_status is not None else '?'}/"
            f"{m.last_imu_system_error if m.last_imu_system_error_active is True else 'inactive'}; "
            f"self-test {'pass' if m.last_imu_self_test_passed is True else 'not confirmed'}. "
            f"{m.last_source}. "
            f"Point TOF online: {online}/{len(m.sensor_specs)}; "
            f"ultrasound valid: {ultrasound_valid}/{len(m.ultrasound_specs)}; "
            f"8x8: {'ready' if m.last_matrix_available and m.last_matrix_valid else 'waiting/invalid'}. "
            f"Known grid squares: {len(m.cells)}; boundaries/internal: "
            f"{sum(w['observations'] >= 3 for w in m.wall_tracks)}/"
            f"{sum(w['observations'] >= 3 for w in m.internal_wall_tracks)}; "
            f"pending wall clusters: "
            f"{sum(not w.get('promoted') for w in m.wall_evidence_clusters)}; "
            f"possible-weight squares: "
            f"{sum(v >= 2 for v in m.weight_votes.values())}.{missing}"
        )
