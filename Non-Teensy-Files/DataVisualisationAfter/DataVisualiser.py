"""
DataVisualiser.py

Offline robot recording visualiser for *.rdbg files.

Features:
- searchable telemetry plotting
- safe autoscale, including sensible all-zero handling
- temporary blue highlight for selected plotted signal
- click-to-inspect cursor with values for every plotted signal
- draggable measurement region with min/max/mean/std/delta/sample count
- fault / command / parameter / state event markers
- double-click an event to jump the graph to it
- derived signals using expressions
- compare a second recording, including align-to-first-fault
- basic anomaly detection overlays
- raw serial, metadata, parameter snapshots, CSV export

Dependencies:
    pip install PyQt6 pyqtgraph numpy
"""

from __future__ import annotations

import csv
import json
import math
import sqlite3
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QEvent, QSettings, Qt, QTimer
from PyQt6.QtGui import QBrush, QColor, QPen
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QInputDialog,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

HERE = Path(__file__).resolve().parent
DATA_DIR = HERE / "Data"
DATA_DIR.mkdir(parents=True, exist_ok=True)
GUI_DIR = HERE.parent / "BluetoothDebugGUI"
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))
from ArenaView import ArenaView, MissionLayout, MissionLayoutCanvas


@dataclass
class SignalSeries:
    elapsed: np.ndarray
    robot: np.ndarray
    values: np.ndarray


class DataVisualiser(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Data Visualiser")
        self.resize(1600, 950)

        self.db_path: Path | None = None
        self.conn: sqlite3.Connection | None = None

        self.compare_path: Path | None = None
        self.compare_conn: sqlite3.Connection | None = None
        self.compare_offset = 0.0

        self.plot_curves: dict[str, pg.PlotDataItem] = {}
        self.compare_curves: dict[str, pg.PlotDataItem] = {}
        self.signal_cache: dict[tuple[str, str], SignalSeries] = {}
        self.derived: dict[str, str] = {}

        self.highlighted_signal: str | None = None
        self.highlighted_original_pen = None
        self.highlighted_original_visible = None
        self.highlighted_original_z = None

        self.marker_items = []
        self.anomaly_items = []
        self.replay_events = []
        self.replay_index = 0
        self.replay_time = 0.0
        self.replay_duration = 0.0
        self.replay_latest = {}
        self._replay_wall_clock = None
        self._replay_advancing = False
        self._replay_dashboard_at = -1.0

        self.cursor_line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen(width=1))
        self.cursor_line.setZValue(1000)
        self.cursor_line.hide()

        self.measure_region = pg.LinearRegionItem(values=(0, 1), movable=True)
        self.measure_region.setZValue(900)
        self.measure_region.hide()

        self._build_ui()
        self.replay_timer = QTimer(self)
        self.replay_timer.timeout.connect(self.advance_replay)
        QApplication.instance().installEventFilter(self)
        self.refresh_recordings()

    # ------------------------------------------------------------------
    # DB helpers
    # ------------------------------------------------------------------

    @staticmethod
    def table_exists(conn: sqlite3.Connection | None, name: str) -> bool:
        if conn is None:
            return False
        return conn.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' AND name=?",
            (name,),
        ).fetchone() is not None

    @staticmethod
    def get_metadata(conn: sqlite3.Connection | None) -> dict[str, str]:
        if conn is None or not DataVisualiser.table_exists(conn, "metadata"):
            return {}
        return {str(k): str(v) for k, v in conn.execute("SELECT key,value FROM metadata")}

    def _series(self, signal: str, source: str = "main") -> SignalSeries:
        key = (source, signal)
        if key in self.signal_cache:
            return self.signal_cache[key]

        if source == "main" and signal in self.derived:
            series = self._evaluate_derived(self.derived[signal])
            self.signal_cache[key] = series
            return series

        conn = self.conn if source == "main" else self.compare_conn
        if conn is None:
            return SignalSeries(np.array([]), np.array([]), np.array([]))

        rows = conn.execute(
            """SELECT elapsed_s, robot_time, value_num
               FROM telemetry
               WHERE signal=? AND value_num IS NOT NULL
               ORDER BY elapsed_s""",
            (signal,),
        ).fetchall()

        if not rows:
            result = SignalSeries(np.array([]), np.array([]), np.array([]))
        else:
            e = np.asarray([r[0] for r in rows], dtype=float)
            r = np.asarray([np.nan if r[1] is None else r[1] for r in rows], dtype=float)
            v = np.asarray([r[2] for r in rows], dtype=float)
            result = SignalSeries(e, r, v)

        self.signal_cache[key] = result
        return result

    def _evaluate_derived(self, expression: str) -> SignalSeries:
        # Expression syntax: sig("drive.left_rpm") - sig("drive.right_rpm")
        import re

        names = re.findall(r"""sig\(\s*["']([^"']+)["']\s*\)""", expression)
        if not names:
            raise ValueError('Expression must contain at least one sig("name") reference.')

        base = self._series(names[0], "main")
        if len(base.elapsed) == 0:
            raise ValueError(f"No numeric data for {names[0]}")

        x = base.elapsed.copy()

        def sig(name: str):
            s = self._series(name, "main")
            if len(s.elapsed) == 0:
                raise ValueError(f"No numeric data for {name}")
            return np.interp(x, s.elapsed, s.values, left=np.nan, right=np.nan)

        safe = {
            "sig": sig,
            "np": np,
            "abs": np.abs,
            "sqrt": np.sqrt,
            "sin": np.sin,
            "cos": np.cos,
            "tan": np.tan,
            "clip": np.clip,
            "minimum": np.minimum,
            "maximum": np.maximum,
        }
        values = eval(expression, {"__builtins__": {}}, safe)
        values = np.asarray(values, dtype=float)
        return SignalSeries(x, np.full_like(x, np.nan), values)

    def _x_values(self, s: SignalSeries, source: str = "main") -> np.ndarray:
        if self.x_axis_combo.currentData() == "robot":
            robot = s.robot.copy()
            finite = np.isfinite(robot)
            if finite.any():
                first = robot[finite][0]
                robot[finite] -= first
                if np.nanmax(np.abs(robot[finite])) > 1000:
                    robot[finite] /= 1000.0
                return robot
        x = s.elapsed.copy()
        if source == "compare":
            x = x + self.compare_offset
        return x

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        top = QHBoxLayout()
        top.addWidget(QLabel("Recording:"))
        self.file_combo = QComboBox()
        self.file_combo.setMinimumWidth(380)
        top.addWidget(self.file_combo)

        self.refresh_button = QPushButton("Refresh")
        self.open_button = QPushButton("Open")
        self.browse_button = QPushButton("Browse...")
        self.compare_button = QPushButton("Compare...")
        self.align_fault_button = QPushButton("Align Compare to First Fault")
        self.clear_compare_button = QPushButton("Clear Compare")
        self.export_button = QPushButton("Export Selected CSV")

        for w in (
            self.refresh_button, self.open_button, self.browse_button,
            self.compare_button, self.align_fault_button, self.clear_compare_button,
            self.export_button,
        ):
            top.addWidget(w)
        top.addStretch()
        root.addLayout(top)

        self.summary_label = QLabel("No recording loaded")
        self.summary_label.setWordWrap(True)
        root.addWidget(self.summary_label)

        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)

        self._build_plot_tab()
        self._build_replay_tab()
        self._build_events_tab()
        self._build_raw_tab()
        self._build_metadata_tab()

        self.refresh_button.clicked.connect(self.refresh_recordings)
        self.open_button.clicked.connect(self.open_selected_recording)
        self.browse_button.clicked.connect(self.browse_recording)
        self.compare_button.clicked.connect(self.browse_compare_recording)
        self.align_fault_button.clicked.connect(self.align_compare_to_first_fault)
        self.clear_compare_button.clicked.connect(self.clear_compare)
        self.export_button.clicked.connect(self.export_selected_csv)

    def _build_replay_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        controls = QHBoxLayout()
        self.replay_play = QPushButton("Play")
        self.replay_end = QPushButton("Go to End")
        self.replay_speed = QComboBox()
        for label, value in (("0.25×", 0.25), ("0.5×", 0.5), ("1×", 1.0),
                             ("2×", 2.0), ("5×", 5.0)):
            self.replay_speed.addItem(label, value)
        self.replay_speed.setCurrentText("1×")
        self.replay_clock = QLabel("0.00 / 0.00 s")
        self.replay_recorded_tab = QLabel("Recorded tab: —")
        self.replay_slider = QSlider(Qt.Orientation.Horizontal)
        self.replay_slider.setRange(0, 0)
        controls.addWidget(self.replay_play)
        controls.addWidget(self.replay_end)
        controls.addWidget(QLabel("Speed:"))
        controls.addWidget(self.replay_speed)
        controls.addWidget(self.replay_clock)
        controls.addWidget(self.replay_recorded_tab)
        controls.addWidget(self.replay_slider, 1)
        layout.addLayout(controls)

        self.replay_tabs = QTabWidget()
        layout.addWidget(self.replay_tabs, 1)

        self.replay_dashboard = QTableWidget(0, 2)
        self.replay_dashboard.setHorizontalHeaderLabels(["Signal", "Value at replay time"])
        self.replay_dashboard.horizontalHeader().setStretchLastSection(True)
        self.replay_tabs.addTab(self.replay_dashboard, "Dashboard")

        matrix_page = QWidget(); matrix_layout = QVBoxLayout(matrix_page)
        self.replay_matrix_status = QLabel("No matrix frame at this time")
        matrix_layout.addWidget(self.replay_matrix_status)
        grid = QGridLayout(); self.replay_matrix_cells = []
        grid.addWidget(QLabel("Y\\X"), 0, 0)
        for col in range(8): grid.addWidget(QLabel(f"X{col}"), 0, col + 1)
        for row in range(8):
            grid.addWidget(QLabel(f"Y{row}"), row + 1, 0)
            cells = []
            for col in range(8):
                cell = QLabel("—"); cell.setAlignment(Qt.AlignmentFlag.AlignCenter)
                cell.setStyleSheet("QLabel{background:#374151;color:white;border:1px solid #111827;padding:5px;}")
                grid.addWidget(cell, row + 1, col + 1); cells.append(cell)
            self.replay_matrix_cells.append(cells)
        matrix_layout.addLayout(grid); matrix_layout.addStretch()
        self.replay_tabs.addTab(matrix_page, "8x8 TOF")

        self.replay_arena = ArenaView(
            QSettings("RobotProject", "DataVisualiserReplay"),
            show_controls=False,
        )
        self.replay_tabs.addTab(self.replay_arena, "Arena View")

        planned_page = QWidget()
        planned_layout = QVBoxLayout(planned_page)
        self.replay_mission_note = QLabel(
            "Pre-laid arena · cyan planned route · orange recorded path · "
            "blue recorded robot pose · yellow live waypoint · purple verified weight bearing")
        planned_layout.addWidget(self.replay_mission_note)
        self.replay_mission_comparison = QLabel(
            "Waiting for recorded pose and forward range")
        planned_layout.addWidget(self.replay_mission_comparison)
        self.replay_mission_layout = MissionLayout()
        self.replay_mission_telemetry = SimpleNamespace(latest={})
        self.replay_mission_canvas = MissionLayoutCanvas(
            self.replay_mission_layout, self.replay_mission_telemetry, planned_page)
        self.replay_mission_canvas.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        planned_layout.addWidget(self.replay_mission_canvas, 1)
        self.replay_tabs.addTab(planned_page, "Planned Arena")

        plot_note = QLabel("The Plots tab is synchronized to this replay clock. "
                           "Its blue cursor follows playback and every recorded numeric signal remains selectable.")
        plot_note.setWordWrap(True); plot_page = QWidget(); plot_layout = QVBoxLayout(plot_page)
        plot_layout.addWidget(plot_note); plot_layout.addStretch()
        self.replay_tabs.addTab(plot_page, "Plots")

        self.replay_parameters = QTableWidget(0, 2)
        self.replay_parameters.setHorizontalHeaderLabels(["Parameter", "Value at replay time"])
        self.replay_parameters.horizontalHeader().setStretchLastSection(True)
        self.replay_tabs.addTab(self.replay_parameters, "Parameters")
        self.replay_commands = QTextEdit(); self.replay_commands.setReadOnly(True)
        self.replay_tabs.addTab(self.replay_commands, "Commands")
        self.replay_wiring = QTextEdit(); self.replay_wiring.setReadOnly(True)
        self.replay_tabs.addTab(self.replay_wiring, "Wiring Guide")
        self.replay_logs = QTextEdit(); self.replay_logs.setReadOnly(True)
        self.replay_tabs.addTab(self.replay_logs, "Logs")
        self.replay_raw = QTextEdit(); self.replay_raw.setReadOnly(True)
        self.replay_tabs.addTab(self.replay_raw, "Raw Serial")
        self.replay_end_arena = ArenaView(
            QSettings("RobotProject", "DataVisualiserEndView"),
            show_controls=False,
        )
        self.replay_tabs.addTab(self.replay_end_arena, "End View")

        self.tabs.addTab(page, "Run Replay")
        self.replay_play.clicked.connect(self.toggle_replay)
        self.replay_end.clicked.connect(lambda: self.replay_slider.setValue(self.replay_slider.maximum()))
        self.replay_slider.valueChanged.connect(self.seek_replay)

    def _build_plot_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        controls.addWidget(QLabel("Search:"))
        self.signal_filter = QLineEdit()
        self.signal_filter.setPlaceholderText("drive, imu, battery...")
        controls.addWidget(self.signal_filter)

        self.add_button = QPushButton("Add Selected")
        self.remove_button = QPushButton("Remove Plot")
        self.clear_button = QPushButton("Clear Plots")
        self.autoscale_button = QPushButton("Auto Scale")
        self.derived_button = QPushButton("Add Derived Signal")
        self.measure_button = QPushButton("Measure Region")
        self.anomaly_button = QPushButton("Find Anomalies")
        self.clear_anomaly_button = QPushButton("Clear Anomalies")

        for w in (
            self.add_button, self.remove_button, self.clear_button, self.autoscale_button,
            self.derived_button, self.measure_button, self.anomaly_button, self.clear_anomaly_button,
        ):
            controls.addWidget(w)

        self.show_faults = QCheckBox("Faults")
        self.show_faults.setChecked(True)
        self.show_commands = QCheckBox("Commands")
        self.show_parameters = QCheckBox("Parameters")
        self.show_states = QCheckBox("States")
        for w in (self.show_faults, self.show_commands, self.show_parameters, self.show_states):
            controls.addWidget(w)

        controls.addWidget(QLabel("X:"))
        self.x_axis_combo = QComboBox()
        self.x_axis_combo.addItem("Elapsed time", "elapsed")
        self.x_axis_combo.addItem("Robot time", "robot")
        controls.addWidget(self.x_axis_combo)
        layout.addLayout(controls)

        split = QSplitter(Qt.Orientation.Horizontal)

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.addWidget(QLabel("Available numeric signals"))
        self.signal_list = QListWidget()
        self.signal_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        ll.addWidget(self.signal_list)

        ll.addWidget(QLabel("Currently plotted"))
        self.plotted_list = QListWidget()
        ll.addWidget(self.plotted_list)

        ll.addWidget(QLabel("Cursor values"))
        self.cursor_table = QTableWidget(0, 2)
        self.cursor_table.setHorizontalHeaderLabels(["Signal", "Value"])
        self.cursor_table.horizontalHeader().setStretchLastSection(True)
        self.cursor_table.setMaximumHeight(220)
        ll.addWidget(self.cursor_table)

        self.measure_label = QLabel("Measurement region: off")
        self.measure_label.setWordWrap(True)
        ll.addWidget(self.measure_label)
        left.setMaximumWidth(410)
        split.addWidget(left)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel("bottom", "Elapsed time", units="s")
        self.plot_widget.setLabel("left", "Value")
        self.plot_widget.addLegend()
        self.plot_widget.addItem(self.cursor_line)
        self.plot_widget.addItem(self.measure_region)
        split.addWidget(self.plot_widget)
        split.setStretchFactor(1, 1)

        layout.addWidget(split, 1)
        self.tabs.addTab(page, "Plots")

        self.signal_filter.textChanged.connect(self.apply_signal_filter)
        self.add_button.clicked.connect(self.add_selected_signals)
        self.remove_button.clicked.connect(self.remove_selected_plot)
        self.clear_button.clicked.connect(self.clear_plots)
        self.autoscale_button.clicked.connect(self.auto_scale_plots)
        self.derived_button.clicked.connect(self.add_derived_signal)
        self.measure_button.clicked.connect(self.toggle_measure_region)
        self.anomaly_button.clicked.connect(self.find_anomalies)
        self.clear_anomaly_button.clicked.connect(self.clear_anomalies)
        self.x_axis_combo.currentIndexChanged.connect(self.reload_plots)
        self.plotted_list.itemSelectionChanged.connect(self.highlight_selected_curve)
        self.show_faults.stateChanged.connect(self.refresh_markers)
        self.show_commands.stateChanged.connect(self.refresh_markers)
        self.show_parameters.stateChanged.connect(self.refresh_markers)
        self.show_states.stateChanged.connect(self.refresh_markers)
        self.measure_region.sigRegionChanged.connect(self.update_measurement)
        self.plot_widget.scene().sigMouseClicked.connect(self.plot_clicked)

    def _build_events_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        self.ev_fault = QCheckBox("Faults"); self.ev_fault.setChecked(True)
        self.ev_logs = QCheckBox("Logs"); self.ev_logs.setChecked(True)
        self.ev_commands = QCheckBox("Commands"); self.ev_commands.setChecked(True)
        self.ev_parameters = QCheckBox("Parameters"); self.ev_parameters.setChecked(True)
        self.ev_states = QCheckBox("States"); self.ev_states.setChecked(True)
        self.ev_annotations = QCheckBox("Annotations"); self.ev_annotations.setChecked(True)
        for w in (self.ev_fault, self.ev_logs, self.ev_commands, self.ev_parameters, self.ev_states, self.ev_annotations):
            controls.addWidget(w)
            w.stateChanged.connect(self.load_events)
        controls.addStretch()
        layout.addLayout(controls)

        self.events_table = QTableWidget(0, 4)
        self.events_table.setHorizontalHeaderLabels(["Elapsed (s)", "Type", "Name / Level", "Details"])
        self.events_table.horizontalHeader().setStretchLastSection(True)
        self.events_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.events_table.cellDoubleClicked.connect(self.jump_to_event)
        layout.addWidget(self.events_table)

        hint = QLabel("Tip: double-click any event to centre the plot around it.")
        layout.addWidget(hint)
        self.tabs.addTab(page, "Events")

    def _build_raw_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.raw_table = QTableWidget(0, 3)
        self.raw_table.setHorizontalHeaderLabels(["Elapsed (s)", "Wall Time", "Serial Line"])
        self.raw_table.horizontalHeader().setStretchLastSection(True)
        layout.addWidget(self.raw_table)
        self.tabs.addTab(page, "Raw Serial")

    def _build_metadata_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        self.metadata_text = QTextEdit()
        self.metadata_text.setReadOnly(True)
        layout.addWidget(self.metadata_text)
        self.tabs.addTab(page, "Session Info")

    # ------------------------------------------------------------------
    # Recording loading
    # ------------------------------------------------------------------

    def refresh_recordings(self):
        current = self.file_combo.currentData()
        self.file_combo.clear()
        files = sorted(DATA_DIR.glob("*.rdbg"), key=lambda p: p.stat().st_mtime, reverse=True)
        for f in files:
            self.file_combo.addItem(f.name, str(f))
        if current:
            idx = self.file_combo.findData(current)
            if idx >= 0:
                self.file_combo.setCurrentIndex(idx)

    def open_selected_recording(self):
        path = self.file_combo.currentData()
        if path:
            self.load_recording(Path(path))

    def browse_recording(self):
        name, _ = QFileDialog.getOpenFileName(self, "Open Robot Recording", str(DATA_DIR), "Robot Debug Recording (*.rdbg)")
        if name:
            self.load_recording(Path(name))

    def load_recording(self, path: Path):
        try:
            if self.conn is not None:
                self.conn.close()
            self.conn = sqlite3.connect(path)
            self.db_path = path
            self.signal_cache.clear()
            self.derived.clear()
            self.clear_plots()
            self.load_signal_names()
            self.load_events()
            self.load_raw()
            self.load_metadata()
            self.load_replay()
            self.update_summary()
            self.refresh_markers()
            self.statusBar().showMessage(f"Loaded {path}")
        except Exception as exc:
            QMessageBox.critical(self, "Could not open recording", str(exc))

    @staticmethod
    def _recorded_value(value_num, value_text, value_type):
        if value_type == "bool":
            return bool(value_num)
        if value_type == "number":
            return value_num
        if value_type == "null":
            return None
        return value_text

    def load_replay(self):
        self.replay_timer.stop()
        self.replay_play.setText("Play")
        self.replay_events = []
        if self.conn is None:
            return
        for row in self.conn.execute(
            "SELECT elapsed_s,robot_time,signal,value_num,value_text,value_type "
            "FROM telemetry ORDER BY elapsed_s"
        ):
            elapsed, robot, signal, number, text_value, value_type = row
            self.replay_events.append((float(elapsed), "telemetry", (
                str(signal), self._recorded_value(number, text_value, value_type), robot)))
        if self.table_exists(self.conn, "matrix_frames"):
            for elapsed, frame_json in self.conn.execute(
                    "SELECT elapsed_s,frame_json FROM matrix_frames ORDER BY elapsed_s"):
                try:
                    self.replay_events.append((float(elapsed), "matrix", json.loads(frame_json)))
                except Exception:
                    pass
        else:
            # Version 3 recordings still retain complete inbound JSON in raw_serial.
            for elapsed, line in self.conn.execute(
                    "SELECT elapsed_s,line FROM raw_serial ORDER BY elapsed_s"):
                try:
                    start = str(line).find("{")
                    message = json.loads(str(line)[start:])
                    if message.get("type") == "tof_8x8":
                        self.replay_events.append((float(elapsed), "matrix", message))
                except Exception:
                    pass
        for table, kind, columns in (
            ("logs", "log", "elapsed_s,level,message"),
            ("commands", "command", "elapsed_s,command,arguments_json"),
            ("parameters", "parameter", "elapsed_s,name,value_json"),
            ("states", "state", "elapsed_s,state_json"),
            ("raw_serial", "raw", "elapsed_s,line"),
            ("ui_snapshots", "snapshot", "elapsed_s,snapshot_json"),
        ):
            if not self.table_exists(self.conn, table):
                continue
            for row in self.conn.execute(f"SELECT {columns} FROM {table} ORDER BY elapsed_s"):
                elapsed, *payload = row
                if kind in ("state", "snapshot"):
                    try: payload = [json.loads(payload[0])]
                    except Exception: pass
                self.replay_events.append((float(elapsed), kind, payload))
        self.replay_events.sort(key=lambda event: event[0])
        self.replay_duration = max((event[0] for event in self.replay_events), default=0.0)
        self.replay_slider.blockSignals(True)
        self.replay_slider.setRange(0, max(0, int(math.ceil(self.replay_duration * 1000))))
        self.replay_slider.setValue(0)
        self.replay_slider.blockSignals(False)
        self._apply_recorded_arena_configuration(self.replay_arena)
        self._apply_recorded_arena_configuration(self.replay_end_arena)
        self._load_recorded_mission()
        self._load_replay_wiring()
        self._reset_replay()
        self._build_end_view()
        self.seek_replay(0)

    def _apply_recorded_arena_configuration(self, view):
        meta = self.get_metadata(self.conn)
        try: layout = json.loads(meta.get("arena_layout_json", "{}"))
        except Exception: layout = {}
        try: settings = json.loads(meta.get("arena_settings_json", "{}"))
        except Exception: settings = {}
        if isinstance(layout, dict):
            point = layout.get("point")
            ultrasound = layout.get("ultrasound")
            matrix = layout.get("matrix")
            if isinstance(point, list): view.model.sensor_specs = point
            if isinstance(ultrasound, list): view.model.ultrasound_specs = ultrasound
            if isinstance(matrix, dict): view.model.matrix_spec = matrix
            view.sensor_canvas.set_specs(view.model.sensor_specs + [view.model.matrix_spec])
            view.sensor_canvas.set_aux_specs(view.model.ultrasound_specs)
        controls = {
            "width": view.arena_width, "height": view.arena_height,
            "encoder_1_mm_per_count": view.encoder_1_scale,
            "encoder_2_mm_per_count": view.encoder_2_scale,
            "track_width_mm": view.track_width, "cell_size_mm": view.grid_size,
            "weight_gap_mm": view.weight_gap, "matrix_fov_deg": view.matrix_fov,
        }
        for key, control in controls.items():
            if key in settings:
                control.blockSignals(True); control.setValue(float(settings[key])); control.blockSignals(False)
        if "matrix_mirrored" in settings:
            view.model.matrix_mirrored = bool(settings["matrix_mirrored"])
        view._sync()
        view.reset_map()

    def _load_recorded_mission(self):
        """Prefer the exact saved layout; recover older runs from sent commands."""
        meta = self.get_metadata(self.conn)
        try:
            recorded = json.loads(meta.get("mission_layout_json", "{}"))
        except (TypeError, ValueError):
            recorded = {}
        exact = isinstance(recorded, dict) and isinstance(
            recorded.get("weights"), list)
        self.replay_mission_layout = MissionLayout()
        if exact:
            self.replay_mission_layout.load_dict(recorded)
            self.replay_mission_layout.route = list(recorded.get("route", []))
            self.replay_mission_note.setText(
                "Exact saved pre-laid arena · cyan planned route · orange "
                "recorded path · blue current pose")
        else:
            plan = None
            map_data = None
            for name, payload in self.conn.execute(
                    "SELECT command,arguments_json FROM commands ORDER BY elapsed_s"):
                try:
                    arguments = json.loads(payload)
                except (TypeError, ValueError):
                    continue
                if name == "mission_map_set":
                    map_data = arguments
                elif name == "mission_plan_set":
                    plan = arguments
            if isinstance(plan, dict):
                layout = self.replay_mission_layout
                layout.start = {
                    "x": float(plan.get("start_x_mm", 325)),
                    "y": float(plan.get("start_y_mm", 325)),
                    "heading_deg": float(plan.get("start_heading_deg", 0)),
                }
                points = plan.get("points", [])
                for index in range(0, len(points) - 2, 3):
                    x, y, target = points[index:index + 3]
                    layout.route.append({"x": float(x), "y": float(y),
                                         "target": bool(target)})
                    if target:
                        layout.add_weight(float(x), float(y))
                # add_weight clears the route while editing; restore the
                # recorded command route once the target markers are made.
                layout.route = [
                    {"x": float(points[index]), "y": float(points[index + 1]),
                     "target": bool(points[index + 2])}
                    for index in range(0, len(points) - 2, 3)]
                if isinstance(map_data, dict):
                    features = map_data.get("features", [])
                    for index in range(0, len(features) - 4, 5):
                        x0, y0, x1, y1, kind = features[index:index + 5]
                        if kind != 1:
                            continue
                        obstacle = layout.add_obstacle(
                            "wall", (x0 + x1) / 2, (y0 + y1) / 2)
                        obstacle["w"] = float(x1 - x0)
                        obstacle["h"] = float(y1 - y0)
                    layout.route = [
                        {"x": float(points[index]), "y": float(points[index + 1]),
                         "target": bool(points[index + 2])}
                        for index in range(0, len(points) - 2, 3)]
                self.replay_mission_note.setText(
                    "Reconstructed from recorded mission commands: target "
                    "waypoints approximate weight positions; original editor "
                    "layout was not saved in this older recording. "
                    "Cyan = planned route; orange = recorded path.")
            else:
                self.replay_mission_note.setText(
                    "No pre-laid mission layout or route in this recording")
        self.replay_mission_canvas.layout_model = self.replay_mission_layout
        self.replay_mission_canvas.update()

    def _load_replay_wiring(self):
        meta = self.get_metadata(self.conn)
        try:
            entries = json.loads(meta.get("wiring_guide_json", "[]"))
        except Exception:
            entries = []
        lines = ["CPU WIRING RECORDED WITH THIS RUN", "================================="]
        for entry in entries if isinstance(entries, list) else []:
            if isinstance(entry, dict):
                lines.append(
                    f"{entry.get('device','?')}: {entry.get('connector','?')} — "
                    f"{entry.get('pins','?')}  {entry.get('notes','')}"
                )
        if not entries:
            lines.append("This older recording has no embedded wiring snapshot.")
        self.replay_wiring.setPlainText("\n".join(lines))

    def _reset_replay(self):
        self.replay_index = 0
        self.replay_time = 0.0
        self._replay_dashboard_at = -1.0
        self.replay_latest = {}
        self.replay_mission_telemetry.latest.clear()
        self.replay_mission_canvas.replay_path.clear()
        self.replay_mission_canvas.replay_observed_ray = None
        self.replay_mission_canvas.replay_expected_ray = None
        self.replay_mission_comparison.setText(
            "Waiting for recorded pose and forward range")
        self.replay_dashboard.setRowCount(0)
        self.replay_parameters.setRowCount(0)
        self.replay_commands.clear(); self.replay_logs.clear(); self.replay_raw.clear()
        self.replay_recorded_tab.setText("Recorded tab: —")
        self.replay_arena.reset_map()
        if self.conn is not None:
            self._load_recorded_mission()
        for row in self.replay_matrix_cells:
            for cell in row: cell.setText("—")

    def _build_end_view(self):
        self.replay_end_arena.reset_map()
        for _elapsed, kind, payload in self.replay_events:
            if kind == "telemetry":
                self.replay_end_arena.receive_telemetry(*payload)
            elif kind == "matrix" and isinstance(payload, dict):
                self.replay_end_arena.receive_matrix(payload)

    def toggle_replay(self):
        if self.replay_timer.isActive():
            self.replay_timer.stop(); self.replay_play.setText("Play")
            self._replay_wall_clock = None
        else:
            if self.replay_time >= self.replay_duration:
                self.replay_slider.setValue(0)
            self._replay_wall_clock = time.monotonic()
            self.replay_timer.start(50); self.replay_play.setText("Pause")

    def advance_replay(self):
        speed = float(self.replay_speed.currentData() or 1.0)
        now = time.monotonic()
        elapsed = 0.05 if self._replay_wall_clock is None else max(
            0.0, now - self._replay_wall_clock)
        self._replay_wall_clock = now
        value = self.replay_slider.value() + max(1, int(elapsed * 1000 * speed))
        if value >= self.replay_slider.maximum():
            value = self.replay_slider.maximum()
            self.replay_timer.stop(); self.replay_play.setText("Play")
            self._replay_wall_clock = None
        self._replay_advancing = True
        try:
            self.replay_slider.setValue(value)
        finally:
            self._replay_advancing = False

    def seek_replay(self, milliseconds: int):
        target = milliseconds / 1000.0
        if target < self.replay_time:
            self._reset_replay()
        while self.replay_index < len(self.replay_events) and \
                self.replay_events[self.replay_index][0] <= target:
            elapsed, kind, payload = self.replay_events[self.replay_index]
            self._apply_replay_event(elapsed, kind, payload)
            self.replay_index += 1
        self.replay_time = target
        self.replay_clock.setText(f"{target:.2f} / {self.replay_duration:.2f} s")
        if (not self._replay_advancing or target - self._replay_dashboard_at >= 0.2
                or target >= self.replay_duration):
            self._refresh_replay_dashboard()
            self._replay_dashboard_at = target
        self._update_planned_comparison()
        self.replay_mission_canvas.update()
        self.cursor_line.setPos(target); self.cursor_line.show(); self.update_cursor_values()

    def _update_planned_comparison(self):
        """Contrast the forward range with physical walls in the saved map."""
        latest = self.replay_mission_telemetry.latest
        keys = ("mission.pose_x_mm", "mission.pose_y_mm",
                "mission.heading_deg", "navigation.front_mm")
        values = [latest.get(key) for key in keys]
        canvas = self.replay_mission_canvas
        canvas.replay_observed_ray = None
        canvas.replay_expected_ray = None
        if not all(isinstance(value, (int, float)) for value in values):
            self.replay_mission_comparison.setText(
                "Waiting for recorded pose and forward range")
            return
        x, y, heading, observed = (float(value) for value in values)
        if observed < 30 or observed > 3500:
            self.replay_mission_comparison.setText(
                "No valid forward range at this replay time")
            return
        matrix = self.replay_arena.model.matrix_spec
        lateral = float(matrix.get("x", 0))
        forward = float(matrix.get("y", 0))
        body_angle = math.radians(heading)
        sensor_x = x + math.cos(body_angle) * forward - math.sin(body_angle) * lateral
        sensor_y = y + math.sin(body_angle) * forward + math.cos(body_angle) * lateral
        ray_angle = math.radians(heading + float(matrix.get("angle", 0)))
        ux, uy = math.cos(ray_angle), math.sin(ray_angle)

        def intersect(rect):
            x0, y0, x1, y1 = rect
            near, far = -5000.0, 5000.0
            for origin, direction, low, high in (
                    (sensor_x, ux, x0, x1), (sensor_y, uy, y0, y1)):
                if abs(direction) < 1e-5:
                    if origin < low or origin > high:
                        return None
                    continue
                a, b = (low - origin) / direction, (high - origin) / direction
                near, far = max(near, min(a, b)), min(far, max(a, b))
            hit = near if near > 0 else far
            return hit if far >= near and hit > 0 else None

        layout = self.replay_mission_layout
        width, height = layout.WIDTH_MM, layout.HEIGHT_MM
        physical = [(-1, -1, 0, height + 1),
                    (width, -1, width + 1, height + 1),
                    (0, -1, width, 0),
                    (0, height, width, height + 1)]
        physical.extend(layout._obstacle_rect(item)
                        for item in layout.obstacles)
        hits = [distance for rect in physical
                if (distance := intersect(rect)) is not None]
        expected = min(hits) if hits else None
        origin = (sensor_x, sensor_y)
        canvas.replay_observed_ray = (
            origin, (sensor_x + observed * ux, sensor_y + observed * uy))
        if expected is None:
            self.replay_mission_comparison.setText(
                f"Forward TOF saw {observed:.0f} mm · no mapped wall on "
                "this ray (red line = observed)")
            return
        canvas.replay_expected_ray = (
            origin, (sensor_x + expected * ux, sensor_y + expected * uy))
        difference = expected - observed
        if difference > 250:
            verdict = "CLOSER THAN MAP — obstacle or pose/map mismatch"
        elif difference < -250:
            verdict = "FARTHER THAN MAP — wall may be missed or pose shifted"
        else:
            verdict = "roughly consistent with mapped wall"
        self.replay_mission_comparison.setText(
            f"Forward TOF saw {observed:.0f} mm (red); mapped wall along "
            f"8×8 axis ≈{expected:.0f} mm (cyan) · {verdict}")

    def _apply_replay_event(self, elapsed, kind, payload):
        if kind == "telemetry":
            name, value, robot_time = payload
            self.replay_latest[name] = value
            self.replay_mission_telemetry.latest[name] = value
            if name == "mission.pose_y_mm":
                x = self.replay_mission_telemetry.latest.get("mission.pose_x_mm")
                y = value
                if isinstance(x, (int, float)) and isinstance(y, (int, float)):
                    path = self.replay_mission_canvas.replay_path
                    if not path or math.hypot(x - path[-1][0],
                                              y - path[-1][1]) >= 10.0:
                        path.append((float(x), float(y)))
            self.replay_arena.receive_telemetry(name, value, robot_time)
        elif kind == "matrix" and isinstance(payload, dict):
            self.replay_arena.receive_matrix(payload)
            values = payload.get("data", [])
            self.replay_matrix_status.setText(
                f"t={elapsed:.2f}s · frame {payload.get('frame','?')} · "
                f"valid={payload.get('valid',False)}")
            if isinstance(values, list) and len(values) == 64:
                for row in range(8):
                    for col in range(8):
                        self.replay_matrix_cells[row][col].setText(str(values[row * 8 + col]))
        elif kind == "log":
            self.replay_logs.append(f"[{elapsed:8.3f}] [{payload[0]}] {payload[1]}")
        elif kind == "command":
            self.replay_commands.append(f"[{elapsed:8.3f}] {payload[0]} {payload[1]}")
            if payload[0] == "mission_plan_set":
                try:
                    plan = json.loads(payload[1])
                    points = plan.get("points", [])
                    self.replay_mission_layout.route = [
                        {"x": float(points[index]), "y": float(points[index + 1]),
                         "target": bool(points[index + 2])}
                        for index in range(0, len(points) - 2, 3)]
                except (TypeError, ValueError, KeyError):
                    pass
        elif kind == "parameter":
            try: value = json.loads(payload[1])
            except Exception: value = payload[1]
            self.replay_latest[f"parameter:{payload[0]}"] = value
        elif kind == "state":
            state = payload[0] if isinstance(payload, list) else payload
            if isinstance(state, dict):
                for key, value in state.items(): self.replay_latest[f"state:{key}"] = value
        elif kind == "raw":
            self.replay_raw.append(f"[{elapsed:8.3f}] {payload[0]}")
        elif kind == "snapshot":
            snap = payload[0] if isinstance(payload, list) else payload
            if isinstance(snap, dict):
                self.replay_recorded_tab.setText(f"Recorded tab: {snap.get('active_tab','—')}")
                layout = snap.get("mission_layout")
                if isinstance(layout, dict) and isinstance(layout.get("weights"), list):
                    self.replay_mission_layout.load_dict(layout)
                    self.replay_mission_layout.route = list(layout.get("route", []))
                for name, value in snap.get("parameters", {}).items():
                    self.replay_latest.setdefault(f"parameter:{name}", value)

    def _refresh_replay_dashboard(self):
        telemetry = sorted((k, v) for k, v in self.replay_latest.items()
                           if not k.startswith("parameter:"))
        self.replay_dashboard.setRowCount(len(telemetry))
        for row, (name, value) in enumerate(telemetry):
            self.replay_dashboard.setItem(row, 0, QTableWidgetItem(name))
            self.replay_dashboard.setItem(row, 1, QTableWidgetItem(str(value)))
        parameters = sorted((k[10:], v) for k, v in self.replay_latest.items()
                            if k.startswith("parameter:"))
        self.replay_parameters.setRowCount(len(parameters))
        for row, (name, value) in enumerate(parameters):
            self.replay_parameters.setItem(row, 0, QTableWidgetItem(name))
            self.replay_parameters.setItem(row, 1, QTableWidgetItem(str(value)))

    def browse_compare_recording(self):
        name, _ = QFileDialog.getOpenFileName(self, "Open comparison recording", str(DATA_DIR), "Robot Debug Recording (*.rdbg)")
        if not name:
            return
        try:
            if self.compare_conn is not None:
                self.compare_conn.close()
            self.compare_path = Path(name)
            self.compare_conn = sqlite3.connect(self.compare_path)
            self.compare_offset = 0.0
            self.signal_cache = {k: v for k, v in self.signal_cache.items() if k[0] != "compare"}
            self.reload_plots()
            self.statusBar().showMessage(f"Comparison loaded: {self.compare_path.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Could not load comparison", str(exc))

    def clear_compare(self):
        if self.compare_conn is not None:
            self.compare_conn.close()
        self.compare_conn = None
        self.compare_path = None
        self.compare_offset = 0.0
        for curve in self.compare_curves.values():
            self.plot_widget.removeItem(curve)
        self.compare_curves.clear()
        self.statusBar().showMessage("Comparison cleared")

    def _first_fault(self, conn):
        if conn is None or not self.table_exists(conn, "faults"):
            return None
        row = conn.execute("SELECT elapsed_s FROM faults ORDER BY elapsed_s LIMIT 1").fetchone()
        return None if row is None else float(row[0])

    def align_compare_to_first_fault(self):
        if self.conn is None or self.compare_conn is None:
            QMessageBox.information(self, "Compare", "Load both a main and comparison recording first.")
            return
        a = self._first_fault(self.conn)
        b = self._first_fault(self.compare_conn)
        if a is None or b is None:
            QMessageBox.information(self, "Compare", "Both recordings need at least one fault marker.")
            return
        self.compare_offset = a - b
        self.reload_plots()
        self.statusBar().showMessage(f"Comparison aligned: first fault at {a:.3f} s")

    def update_summary(self):
        if self.conn is None:
            return
        meta = self.get_metadata(self.conn)
        sigs = self.conn.execute("SELECT COUNT(DISTINCT signal) FROM telemetry").fetchone()[0]
        samples = self.conn.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        duration = self.conn.execute("SELECT COALESCE(MAX(elapsed_s),0) FROM telemetry").fetchone()[0]
        faults = 0
        if self.table_exists(self.conn, "faults"):
            faults = self.conn.execute("SELECT COUNT(*) FROM faults").fetchone()[0]
        compare = f" | Compare: {self.compare_path.name}" if self.compare_path else ""
        self.summary_label.setText(
            f"{self.db_path.name} | {duration:.2f} s | {sigs} signals | {samples} telemetry rows | "
            f"{faults} faults | Test: {meta.get('test_name','—')}{compare}"
        )

    def load_signal_names(self):
        self.signal_list.clear()
        if self.conn is None:
            return
        rows = self.conn.execute(
            "SELECT DISTINCT signal FROM telemetry WHERE value_num IS NOT NULL ORDER BY signal"
        )
        for (name,) in rows:
            self.signal_list.addItem(str(name))

    def apply_signal_filter(self):
        needle = self.signal_filter.text().strip().lower()
        for i in range(self.signal_list.count()):
            item = self.signal_list.item(i)
            item.setHidden(needle not in item.text().lower())

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def add_selected_signals(self):
        for item in self.signal_list.selectedItems():
            self._add_signal(item.text())
        self.ensure_sensible_zero_only_view()
        self.refresh_markers()

    def _add_signal(self, name: str):
        if name in self.plot_curves:
            return
        s = self._series(name, "main")
        x = self._x_values(s, "main")
        finite = np.isfinite(x) & np.isfinite(s.values)
        curve = self.plot_widget.plot(x[finite], s.values[finite], name=name)
        self.plot_curves[name] = curve
        self.plotted_list.addItem(name)

        if self.compare_conn is not None:
            cs = self._series(name, "compare")
            cx = self._x_values(cs, "compare")
            cfinite = np.isfinite(cx) & np.isfinite(cs.values)
            if cfinite.any():
                pen = pg.mkPen(style=Qt.PenStyle.DashLine, width=2)
                ccurve = self.plot_widget.plot(cx[cfinite], cs.values[cfinite], pen=pen, name=f"{name} [compare]")
                self.compare_curves[name] = ccurve

    def reload_plots(self):
        names = list(self.plot_curves)
        self.clear_plot_items_only()
        self.plotted_list.clear()
        for name in names:
            self._add_signal(name)
        self.refresh_markers()
        self.update_cursor_values()
        self.update_measurement()

    def clear_plot_items_only(self):
        self.restore_highlight()
        for curve in list(self.plot_curves.values()) + list(self.compare_curves.values()):
            self.plot_widget.removeItem(curve)
        self.plot_curves.clear()
        self.compare_curves.clear()

    def remove_selected_plot(self):
        for item in list(self.plotted_list.selectedItems()):
            name = item.text()
            self.restore_highlight()
            curve = self.plot_curves.pop(name, None)
            if curve is not None:
                self.plot_widget.removeItem(curve)
            ccurve = self.compare_curves.pop(name, None)
            if ccurve is not None:
                self.plot_widget.removeItem(ccurve)
            self.plotted_list.takeItem(self.plotted_list.row(item))
        self.update_cursor_values()

    def clear_plots(self):
        self.clear_plot_items_only()
        self.plotted_list.clear()
        self.cursor_table.setRowCount(0)
        self.cursor_line.hide()
        self.refresh_markers()

    def auto_scale_plots(self):
        visible = []
        for name, curve in self.plot_curves.items():
            if not curve.isVisible():
                continue
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                visible.append((x[mask], s.values[mask]))

        for name, curve in self.compare_curves.items():
            if not curve.isVisible():
                continue
            s = self._series(name, "compare")
            x = self._x_values(s, "compare")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                visible.append((x[mask], s.values[mask]))

        if not visible:
            return

        all_x = np.concatenate([x for x, _ in visible])
        all_y = np.concatenate([y for _, y in visible])

        if np.all(all_y == 0):
            xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
            if xmin == xmax:
                xmin, xmax = xmin - 0.5, xmax + 0.5
            else:
                pad = max((xmax - xmin) * 0.05, 0.05)
                xmin, xmax = xmin - pad, xmax + pad
            self.plot_widget.setXRange(xmin, xmax, padding=0)
            self.plot_widget.setYRange(-1.0, 1.0, padding=0)
            return

        xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
        ymin, ymax = float(np.min(all_y)), float(np.max(all_y))

        if xmin == xmax:
            xmin, xmax = xmin - 0.5, xmax + 0.5
        else:
            px = max((xmax - xmin) * 0.05, 0.05)
            xmin, xmax = xmin - px, xmax + px

        if ymin == ymax:
            py = max(abs(ymin) * 0.1, 1.0)
        else:
            py = (ymax - ymin) * 0.08

        self.plot_widget.setXRange(xmin, xmax, padding=0)
        self.plot_widget.setYRange(ymin - py, ymax + py, padding=0)

    def ensure_sensible_zero_only_view(self):
        ys = []
        xs = []
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if mask.any():
                xs.append(x[mask]); ys.append(s.values[mask])
        if ys and all(np.all(y == 0) for y in ys):
            x = np.concatenate(xs)
            xmin, xmax = float(np.min(x)), float(np.max(x))
            if xmin == xmax:
                xmin, xmax = xmin - 0.5, xmax + 0.5
            self.plot_widget.setXRange(xmin, xmax, padding=0)
            self.plot_widget.setYRange(-1, 1, padding=0)
            return True
        return False

    # ------------------------------------------------------------------
    # Temporary blue highlight
    # ------------------------------------------------------------------

    def highlight_selected_curve(self):
        self.restore_highlight()
        items = self.plotted_list.selectedItems()
        if not items:
            return
        name = items[0].text()
        curve = self.plot_curves.get(name)
        if curve is None:
            return
        self.highlighted_signal = name
        self.highlighted_original_pen = curve.opts.get("pen")
        self.highlighted_original_visible = curve.isVisible()
        self.highlighted_original_z = curve.zValue()
        curve.setVisible(True)
        curve.setPen(pg.mkPen((0, 120, 255), width=4))
        curve.setZValue(500)

    def restore_highlight(self):
        if self.highlighted_signal is None:
            return
        curve = self.plot_curves.get(self.highlighted_signal)
        if curve is not None:
            if self.highlighted_original_pen is not None:
                curve.setPen(self.highlighted_original_pen)
            curve.setVisible(bool(self.highlighted_original_visible))
            curve.setZValue(self.highlighted_original_z or 0)
        self.highlighted_signal = None

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Type.MouseButtonPress and self.highlighted_signal is not None:
            pos = event.globalPosition().toPoint() if hasattr(event, "globalPosition") else None
            if pos is not None:
                local = self.plotted_list.mapFromGlobal(pos)
                if not self.plotted_list.rect().contains(local):
                    self.restore_highlight()
                    self.plotted_list.clearSelection()
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------------
    # Cursor
    # ------------------------------------------------------------------

    def plot_clicked(self, ev):
        if ev.button() != Qt.MouseButton.LeftButton:
            return
        if not self.plot_widget.getPlotItem().sceneBoundingRect().contains(ev.scenePos()):
            return
        mouse = self.plot_widget.getPlotItem().vb.mapSceneToView(ev.scenePos())
        self.cursor_line.setPos(mouse.x())
        self.cursor_line.show()
        self.update_cursor_values()

    def update_cursor_values(self):
        if not self.cursor_line.isVisible():
            return
        x0 = float(self.cursor_line.value())
        rows = []

        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if not mask.any():
                continue
            xf, yf = x[mask], s.values[mask]
            idx = int(np.argmin(np.abs(xf - x0)))
            rows.append((name, yf[idx]))

        for name in self.compare_curves:
            s = self._series(name, "compare")
            x = self._x_values(s, "compare")
            mask = np.isfinite(x) & np.isfinite(s.values)
            if not mask.any():
                continue
            xf, yf = x[mask], s.values[mask]
            idx = int(np.argmin(np.abs(xf - x0)))
            rows.append((f"{name} [compare]", yf[idx]))

        self.cursor_table.setRowCount(len(rows))
        for r, (name, value) in enumerate(rows):
            self.cursor_table.setItem(r, 0, QTableWidgetItem(name))
            self.cursor_table.setItem(r, 1, QTableWidgetItem(f"{value:.8g}"))
        self.statusBar().showMessage(f"Cursor: {x0:.4f} s")

    # ------------------------------------------------------------------
    # Measurement region
    # ------------------------------------------------------------------

    def toggle_measure_region(self):
        if self.measure_region.isVisible():
            self.measure_region.hide()
            self.measure_label.setText("Measurement region: off")
            self.measure_button.setText("Measure Region")
            return

        xr = self.plot_widget.getPlotItem().vb.viewRange()[0]
        a = xr[0] + 0.35 * (xr[1] - xr[0])
        b = xr[0] + 0.65 * (xr[1] - xr[0])
        self.measure_region.setRegion((a, b))
        self.measure_region.show()
        self.measure_button.setText("Hide Measure Region")
        self.update_measurement()

    def update_measurement(self):
        if not self.measure_region.isVisible():
            return
        a, b = sorted(self.measure_region.getRegion())
        lines = [f"{a:.3f}–{b:.3f} s  (Δt={b-a:.3f}s)"]
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values) & (x >= a) & (x <= b)
            vals = s.values[mask]
            if len(vals):
                std = float(np.std(vals)) if len(vals) > 1 else 0.0
                lines.append(
                    f"{name}: n={len(vals)} min={np.min(vals):.5g} max={np.max(vals):.5g} "
                    f"mean={np.mean(vals):.5g} std={std:.5g} Δ={vals[-1]-vals[0]:.5g}"
                )
        self.measure_label.setText("\n".join(lines))

    # ------------------------------------------------------------------
    # Derived signals
    # ------------------------------------------------------------------

    def add_derived_signal(self):
        name, ok = QInputDialog.getText(self, "Derived signal", "New signal name:")
        if not ok or not name.strip():
            return
        expr, ok = QInputDialog.getText(
            self,
            "Derived signal expression",
            'Expression, e.g. sig("drive.left_rpm") - sig("drive.right_rpm"):',
        )
        if not ok or not expr.strip():
            return

        name = name.strip()
        try:
            self.derived[name] = expr.strip()
            self.signal_cache.pop(("main", name), None)
            self._series(name, "main")  # validate now
        except Exception as exc:
            self.derived.pop(name, None)
            QMessageBox.warning(self, "Derived signal error", str(exc))
            return

        self.signal_list.addItem(name)
        self._add_signal(name)
        self.statusBar().showMessage(f"Derived signal added: {name}")

    # ------------------------------------------------------------------
    # Event markers
    # ------------------------------------------------------------------

    def _remove_markers(self):
        for item in self.marker_items:
            try:
                self.plot_widget.removeItem(item)
            except Exception:
                pass
        self.marker_items.clear()

    def _add_marker(self, x: float, label: str, color, width=2, style=Qt.PenStyle.SolidLine):
        line = pg.InfiniteLine(pos=x, angle=90, movable=False, pen=pg.mkPen(color=color, width=width, style=style))
        line.setZValue(800)
        text = pg.InfLineLabel(line, text=label, position=0.92, rotateAxis=(1, 0), anchor=(1, 1))
        self.plot_widget.addItem(line)
        self.marker_items.extend([line, text])

    def refresh_markers(self):
        self._remove_markers()
        if self.conn is None or self.x_axis_combo.currentData() != "elapsed":
            return

        if self.show_faults.isChecked() and self.table_exists(self.conn, "faults"):
            for elapsed, label in self.conn.execute("SELECT elapsed_s,label FROM faults ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), str(label), (255, 0, 0), width=3)

        if self.show_commands.isChecked() and self.table_exists(self.conn, "commands"):
            for elapsed, name in self.conn.execute("SELECT elapsed_s,command FROM commands ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), f"CMD {name}", (0, 170, 255), style=Qt.PenStyle.DashLine)

        if self.show_parameters.isChecked() and self.table_exists(self.conn, "parameters"):
            for elapsed, name in self.conn.execute("SELECT elapsed_s,name FROM parameters ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), f"PARAM {name}", (255, 170, 0), style=Qt.PenStyle.DotLine)

        if self.show_states.isChecked() and self.table_exists(self.conn, "states"):
            for elapsed, in self.conn.execute("SELECT elapsed_s FROM states ORDER BY elapsed_s"):
                self._add_marker(float(elapsed), "STATE", (170, 0, 255), style=Qt.PenStyle.DashDotLine)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def load_events(self):
        self.events_table.setRowCount(0)
        if self.conn is None:
            return
        events = []

        if self.ev_fault.isChecked() and self.table_exists(self.conn, "faults"):
            for t, label in self.conn.execute("SELECT elapsed_s,label FROM faults"):
                events.append((t, "FAULT", label, "Manual fault marker"))

        if self.ev_logs.isChecked() and self.table_exists(self.conn, "logs"):
            for t, level, msg in self.conn.execute("SELECT elapsed_s,level,message FROM logs"):
                events.append((t, "LOG", level, msg))

        if self.ev_commands.isChecked() and self.table_exists(self.conn, "commands"):
            for t, cmd, args in self.conn.execute("SELECT elapsed_s,command,arguments_json FROM commands"):
                events.append((t, "COMMAND", cmd, args))

        if self.ev_parameters.isChecked() and self.table_exists(self.conn, "parameters"):
            for t, name, value in self.conn.execute("SELECT elapsed_s,name,value_json FROM parameters"):
                events.append((t, "PARAMETER", name, value))

        if self.ev_states.isChecked() and self.table_exists(self.conn, "states"):
            for t, state in self.conn.execute("SELECT elapsed_s,state_json FROM states"):
                events.append((t, "STATE", "Robot state", state))

        if self.ev_annotations.isChecked() and self.table_exists(self.conn, "annotations"):
            for t, label, note in self.conn.execute("SELECT elapsed_s,label,note FROM annotations"):
                events.append((t, "ANNOTATION", label, note))

        events.sort(key=lambda x: x[0])
        self.events_table.setRowCount(len(events))
        for row, event in enumerate(events):
            for col, value in enumerate(event):
                self.events_table.setItem(row, col, QTableWidgetItem(f"{value:.3f}" if col == 0 else str(value)))

    def jump_to_event(self, row: int, _col: int):
        item = self.events_table.item(row, 0)
        if item is None:
            return
        try:
            t = float(item.text())
        except ValueError:
            return
        self.tabs.setCurrentIndex(0)
        self.plot_widget.setXRange(t - 3.0, t + 3.0, padding=0)
        self.cursor_line.setPos(t)
        self.cursor_line.show()
        self.update_cursor_values()

    # ------------------------------------------------------------------
    # Anomalies
    # ------------------------------------------------------------------

    def clear_anomalies(self):
        for item in self.anomaly_items:
            try:
                self.plot_widget.removeItem(item)
            except Exception:
                pass
        self.anomaly_items.clear()

    def find_anomalies(self):
        self.clear_anomalies()
        if not self.plot_curves:
            return

        total = 0
        for name in self.plot_curves:
            s = self._series(name, "main")
            x = self._x_values(s, "main")
            mask = np.isfinite(x) & np.isfinite(s.values)
            x, y = x[mask], s.values[mask]
            if len(y) < 8:
                continue

            median = np.median(y)
            mad = np.median(np.abs(y - median))
            if mad > 0:
                robust_z = 0.6745 * (y - median) / mad
                idx = np.where(np.abs(robust_z) > 6.0)[0]
            else:
                idx = np.array([], dtype=int)

            dy = np.diff(y)
            if len(dy) >= 5:
                dmed = np.median(dy)
                dmad = np.median(np.abs(dy - dmed))
                didx = np.where(np.abs(0.6745 * (dy - dmed) / dmad) > 8.0)[0] + 1 if dmad > 0 else np.array([], dtype=int)
                idx = np.unique(np.concatenate([idx, didx]))

            # Collapse markers closer than 50 ms.
            last = -1e99
            for i in idx:
                if x[i] - last < 0.05:
                    continue
                line = pg.InfiniteLine(pos=float(x[i]), angle=90, movable=False, pen=pg.mkPen((255, 0, 255), width=1))
                line.setZValue(700)
                self.plot_widget.addItem(line)
                self.anomaly_items.append(line)
                last = x[i]
                total += 1

        self.statusBar().showMessage(f"Anomaly scan: {total} candidate points marked in magenta")

    # ------------------------------------------------------------------
    # Raw / metadata
    # ------------------------------------------------------------------

    def load_raw(self):
        self.raw_table.setRowCount(0)
        if self.conn is None or not self.table_exists(self.conn, "raw_serial"):
            return
        rows = self.conn.execute("SELECT elapsed_s,wall_time,line FROM raw_serial ORDER BY elapsed_s").fetchall()
        self.raw_table.setRowCount(len(rows))
        for r, values in enumerate(rows):
            for c, value in enumerate(values):
                self.raw_table.setItem(r, c, QTableWidgetItem(f"{value:.3f}" if c == 0 else str(value)))

    def load_metadata(self):
        if self.conn is None:
            self.metadata_text.clear()
            return
        meta = self.get_metadata(self.conn)
        lines = ["SESSION METADATA", "================"]
        for key in sorted(meta):
            lines.append(f"{key}: {meta[key]}")

        if self.table_exists(self.conn, "parameter_snapshots"):
            row = self.conn.execute(
                "SELECT elapsed_s,snapshot_json FROM parameter_snapshots ORDER BY elapsed_s LIMIT 1"
            ).fetchone()
            if row:
                lines.extend(["", "PARAMETER SNAPSHOT", "==================", f"at {row[0]:.3f} s"])
                try:
                    snap = json.loads(row[1])
                    for key in sorted(snap):
                        lines.append(f"{key}: {snap[key]}")
                except Exception:
                    lines.append(str(row[1]))

        self.metadata_text.setPlainText("\n".join(lines))

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def export_selected_csv(self):
        if self.conn is None:
            return
        names = [i.text() for i in self.plotted_list.selectedItems()]
        if not names:
            names = list(self.plot_curves)
        if not names:
            QMessageBox.information(self, "Export", "Plot or select at least one signal first.")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self, "Export telemetry CSV", str(DATA_DIR / "telemetry_export.csv"), "CSV (*.csv)"
        )
        if not filename:
            return

        rows = []
        for name in names:
            s = self._series(name, "main")
            for t, v in zip(s.elapsed, s.values):
                if np.isfinite(t) and np.isfinite(v):
                    rows.append((float(t), name, float(v)))
        rows.sort()

        with open(filename, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["elapsed_s", "signal", "value"])
            w.writerows(rows)

        self.statusBar().showMessage(f"Exported {len(rows)} rows to {filename}")

    def closeEvent(self, event):
        if self.conn is not None:
            self.conn.close()
        if self.compare_conn is not None:
            self.compare_conn.close()
        event.accept()


def main():
    app = QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = DataVisualiser()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
