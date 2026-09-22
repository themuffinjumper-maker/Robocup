"""
DebugGUI.py

Live robot debug console with:
- COM port selection
- dynamic telemetry
- dashboard commands
- live plots
- dynamic parameters and commands
- logs/raw serial
- full session recording to .rdbg files
- manual "Log Fault" event markers

Dependencies:
    pip install PyQt6 pyserial pyqtgraph

Required beside this file:
    BluetoothSerial.py
    DataRecorder.py
"""

from __future__ import annotations

import json
import sys
import time
import subprocess
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any

from PyQt6.QtCore import QEvent, Qt, QSettings, QTimer
from PyQt6.QtGui import QColor, QFont, QImage, QPixmap
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import pyqtgraph as pg

from BluetoothSerial import BluetoothSerial
from DataRecorder import DataRecorder
from WiringGuide import WiringGuide
from ArenaView import ArenaView


class ValueEditor(QWidget):
    def __init__(self, definition: dict, parent=None):
        super().__init__(parent)

        datatype = str(
            definition.get(
                "datatype",
                definition.get("type", "float"),
            )
        ).lower()

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if datatype in ("bool", "boolean"):
            self.editor = QCheckBox()
            self.editor.setChecked(
                bool(
                    definition.get(
                        "value",
                        definition.get("default", False),
                    )
                )
            )

        elif datatype in ("int", "integer"):
            self.editor = QSpinBox()
            self.editor.setRange(
                int(definition.get("min", -1_000_000_000)),
                int(definition.get("max", 1_000_000_000)),
            )
            self.editor.setSingleStep(
                max(int(definition.get("step", 1)), 1)
            )
            self.editor.setValue(
                int(
                    definition.get(
                        "value",
                        definition.get("default", 0),
                    )
                )
            )

        elif datatype in ("enum", "choice", "select"):
            self.editor = QComboBox()

            for option in definition.get("options", []):
                if isinstance(option, dict):
                    label = str(
                        option.get(
                            "label",
                            option.get("value", ""),
                        )
                    )
                    value = option.get("value", label)
                    self.editor.addItem(label, value)
                else:
                    self.editor.addItem(str(option), option)

            current = definition.get(
                "value",
                definition.get("default"),
            )

            for index in range(self.editor.count()):
                if self.editor.itemData(index) == current:
                    self.editor.setCurrentIndex(index)
                    break

        elif datatype in ("str", "string", "text"):
            self.editor = QLineEdit()
            self.editor.setText(
                str(
                    definition.get(
                        "value",
                        definition.get("default", ""),
                    )
                )
            )

        else:
            self.editor = QDoubleSpinBox()
            self.editor.setDecimals(
                int(definition.get("decimals", 4))
            )
            self.editor.setRange(
                float(definition.get("min", -1e12)),
                float(definition.get("max", 1e12)),
            )
            self.editor.setSingleStep(
                float(definition.get("step", 0.01))
            )
            self.editor.setValue(
                float(
                    definition.get(
                        "value",
                        definition.get("default", 0.0),
                    )
                )
            )

        layout.addWidget(self.editor)

    def value(self):
        if isinstance(self.editor, QCheckBox):
            return self.editor.isChecked()

        if isinstance(self.editor, QComboBox):
            return self.editor.currentData()

        if isinstance(self.editor, QLineEdit):
            return self.editor.text()

        return self.editor.value()

    def set_value(self, value: Any):
        try:
            if isinstance(self.editor, QCheckBox):
                self.editor.setChecked(bool(value))

            elif isinstance(self.editor, QComboBox):
                for index in range(self.editor.count()):
                    if self.editor.itemData(index) == value:
                        self.editor.setCurrentIndex(index)
                        break

            elif isinstance(self.editor, QLineEdit):
                self.editor.setText(str(value))

            else:
                self.editor.setValue(value)

        except Exception:
            pass


class CommandWidget(QGroupBox):
    def __init__(
        self,
        definition: dict,
        send_callback,
        favourite_callback=None,
        is_favourite=False,
        parent=None,
    ):
        name = str(definition.get("name", "command"))
        label = str(definition.get("label", name))

        super().__init__(label, parent)

        self.name = name
        self.send_callback = send_callback
        self.argument_editors = {}

        layout = QFormLayout(self)

        category = str(definition.get("category", "General"))
        category_label = QLabel(category)
        category_label.setStyleSheet(
            "QLabel {"
            " background: #334155; color: white; border-radius: 7px;"
            " padding: 3px 8px; font-weight: bold;"
            "}"
        )
        category_label.setMaximumWidth(150)
        layout.addRow(category_label)

        self.favourite_button = QPushButton()
        self.favourite_button.setCheckable(True)
        self.favourite_button.setChecked(bool(is_favourite))
        self.favourite_button.setMaximumWidth(42)
        self.favourite_button.setToolTip(
            "Show this command on the Dashboard"
        )
        self._update_favourite_icon(bool(is_favourite))
        self.favourite_button.toggled.connect(
            self._update_favourite_icon
        )
        if favourite_callback is not None:
            self.favourite_button.toggled.connect(
                lambda checked: favourite_callback(self.name, checked)
            )
        layout.addRow("Dashboard", self.favourite_button)

        description = definition.get("description")

        if description:
            description_label = QLabel(str(description))
            description_label.setWordWrap(True)
            layout.addRow(description_label)

        arguments = definition.get(
            "args",
            definition.get("arguments", []),
        )

        if isinstance(arguments, dict):
            converted = []

            for arg_name, arg_definition in arguments.items():
                if isinstance(arg_definition, dict):
                    item = dict(arg_definition)
                    item["name"] = arg_name
                else:
                    item = {
                        "name": arg_name,
                        "type": str(arg_definition),
                    }

                converted.append(item)

            arguments = converted

        for argument in arguments:
            arg_name = str(argument.get("name", "value"))
            arg_label = str(argument.get("label", arg_name))

            editor = ValueEditor(argument)

            self.argument_editors[arg_name] = editor

            layout.addRow(
                arg_label,
                editor,
            )

        buttons = definition.get("buttons")
        if isinstance(buttons, list) and buttons:
            button_row = QWidget()
            button_layout = QHBoxLayout(button_row)
            button_layout.setContentsMargins(0, 0, 0, 0)
            for button_definition in buttons:
                if not isinstance(button_definition, dict):
                    continue
                button = QPushButton(str(button_definition.get("label", "Run")))
                fixed_arguments = button_definition.get("arguments", {})
                if not isinstance(fixed_arguments, dict):
                    fixed_arguments = {}
                fixed_arguments = dict(fixed_arguments)
                button.clicked.connect(
                    lambda _checked=False, args=fixed_arguments: self._execute(args)
                )
                if fixed_arguments.get("enabled") is True:
                    button.setStyleSheet(
                        "QPushButton { background: #15803d; color: white; font-weight: bold; }"
                    )
                elif fixed_arguments.get("enabled") is False:
                    button.setStyleSheet(
                        "QPushButton { background: #b91c1c; color: white; font-weight: bold; }"
                    )
                button_layout.addWidget(button)
            layout.addRow(button_row)
        else:
            run_button = QPushButton(f"Run {label}")
            run_button.clicked.connect(self._execute)
            layout.addRow(run_button)

    def _update_favourite_icon(self, favourite):
        self.favourite_button.setText("★" if favourite else "☆")

    def set_favourite(self, favourite):
        self.favourite_button.blockSignals(True)
        self.favourite_button.setChecked(bool(favourite))
        self._update_favourite_icon(bool(favourite))
        self.favourite_button.blockSignals(False)

    def _execute(self, fixed_arguments=None):
        arguments = {
            name: editor.value()
            for name, editor in self.argument_editors.items()
        }
        if isinstance(fixed_arguments, dict):
            arguments.update(fixed_arguments)

        self.send_callback(
            self.name,
            arguments,
        )


class RobotDebugGUI(QMainWindow):
    MAX_HISTORY_POINTS = 10000

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Robot Debug Console")
        self.resize(1500, 900)

        self.settings = QSettings(
            "RobotProject",
            "DebugGUI",
        )

        self.bluetooth = BluetoothSerial()
        self.recorder = DataRecorder()

        self.telemetry = {}
        self.telemetry_rows = {}
        self.telemetry_history = {}
        self.parameter_editors = {}
        self.command_widgets = {}
        self.command_definitions = {}
        self.dashboard_command_widgets = {}
        try:
            saved_favourites = json.loads(
                str(self.settings.value("favourite_commands", "[]"))
            )
        except (TypeError, json.JSONDecodeError):
            saved_favourites = []
        self.favourite_commands = set(saved_favourites)
        self.plot_curves = {}

        # Recording metadata / parameter snapshot support.
        self.parameter_definitions = {}
        self.parameter_values = {}
        self.local_config = {}
        self.last_ui_snapshot_second = -1

        # Link-health counters.
        self.raw_line_times = deque(maxlen=5000)
        self.telemetry_event_times = deque(maxlen=10000)
        self.last_telemetry_monotonic = None
        self.connection_started_monotonic = None
        self.session_signal_names = set()
        self.protocol_error_count = 0
        self.robot_debug_mode = False
        self.robot_stopped = True
        self.keyboard_drive_keys = set()
        self.keyboard_drive_last_output = (0, 0)

        self.start_time = time.monotonic()

        self._build_ui()
        self._connect_signals()
        QApplication.instance().installEventFilter(self)
        self.load_local_debug_config()

        self.port_refresh_timer = QTimer(self)
        self.port_refresh_timer.timeout.connect(
            self.refresh_ports
        )
        self.port_refresh_timer.start(2000)

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(
            self.update_plot
        )
        self.plot_timer.start(50)

        self.recording_timer = QTimer(self)
        self.recording_timer.timeout.connect(
            self.update_recording_clock
        )
        self.recording_timer.start(250)

        self.health_timer = QTimer(self)
        self.health_timer.timeout.connect(
            self.update_link_health
        )
        self.health_timer.start(500)

        self.keyboard_drive_timer = QTimer(self)
        self.keyboard_drive_timer.timeout.connect(
            self._keyboard_drive_heartbeat
        )
        self.keyboard_drive_timer.start(150)

        self.refresh_ports()

    def load_local_debug_config(self):
        """Populate controls even when wireless definition frames are lost."""
        config_path = (
            Path(__file__).resolve().parents[2]
            / "PlatformIO"
            / "debug_config.json"
        )
        try:
            config = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            self.add_log(
                "WARNING",
                f"Could not load local debug controls: {error}",
            )
            return
        self.local_config = config

        for parameter in config.get("parameters", []):
            if isinstance(parameter, dict):
                self.on_parameter_definition(dict(parameter))

        for command in config.get("commands", []):
            if isinstance(command, dict):
                definition = dict(command)
                definition.pop("action", None)
                definition.pop("debug_mode_required", None)
                self.on_command_definition(definition)

    # =================================================================
    # Recording
    # =================================================================

    def default_data_directory(self) -> Path:
        debug_gui_dir = Path(__file__).resolve().parent

        data_dir = (
            debug_gui_dir.parent
            / "DataVisualisationAfter"
            / "Data"
        )

        data_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        return data_dir

    def git_metadata(self) -> dict:
        """Best-effort Git branch/commit capture for reproducible tests."""
        try:
            project_root = Path(__file__).resolve().parents[2]
            branch = subprocess.check_output(
                ["git", "branch", "--show-current"],
                cwd=project_root,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            commit = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=project_root,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            dirty = bool(
                subprocess.check_output(
                    ["git", "status", "--porcelain"],
                    cwd=project_root,
                    text=True,
                    stderr=subprocess.DEVNULL,
                ).strip()
            )
            return {
                "git_branch": branch,
                "git_commit": commit,
                "git_dirty": str(dirty).lower(),
            }
        except Exception:
            return {
                "git_branch": "",
                "git_commit": "",
                "git_dirty": "",
            }

    def toggle_recording(self):
        if self.recorder.is_recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        if not self.bluetooth.is_connected():
            QMessageBox.warning(
                self,
                "Robot not connected",
                "Connect to the robot before starting a recording.",
            )
            return

        now = datetime.now()

        suggested = (
            f"RobotRun_"
            f"{now:%Y-%m-%d_%H-%M-%S}"
            f".rdbg"
        )

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Robot Recording As",
            str(
                self.default_data_directory()
                / suggested
            ),
            "Robot Debug Recording (*.rdbg)",
        )

        if not filename:
            return

        path = Path(filename)

        if path.suffix.lower() != ".rdbg":
            path = path.with_suffix(".rdbg")

        session_metadata = {
            "test_name": self.test_name_edit.text().strip(),
            "test_notes": self.test_notes_edit.text().strip(),
            "debug_config_json": json.dumps(self.local_config, separators=(",", ":")),
            "wiring_guide_json": json.dumps(self.wiring_guide.entries, separators=(",", ":")),
            "arena_layout_json": json.dumps({
                "point": self.arena_view.model.sensor_specs,
                "ultrasound": self.arena_view.model.ultrasound_specs,
                "matrix": self.arena_view.model.matrix_spec,
            }, separators=(",", ":")),
            "arena_settings_json": json.dumps(self._arena_recording_settings(), separators=(",", ":")),
            **self.git_metadata(),
        }

        self.recorder.start(
            path,
            session_name=path.stem,
            port=str(
                self.port_combo.currentData()
                or ""
            ),
            baudrate=int(
                self.baud_combo.currentData()
            ),
            metadata=session_metadata,
        )

        # Snapshot every known parameter at recording start.
        snapshot = dict(self.parameter_values)
        for name, editor in self.parameter_editors.items():
            try:
                snapshot[name] = editor.value()
            except Exception:
                pass
        self.recorder.record_parameter_snapshot(snapshot)
        self.last_ui_snapshot_second = -1
        self._record_ui_snapshot()

        self.record_button.setText(
            "Stop Recording"
        )

        self.recording_status.setText(
            f"● RECORDING — {path.name}"
        )

        self.log_fault_button.setEnabled(
            True
        )

        self.statusBar().showMessage(
            f"Recording to {path}"
        )

        self.add_log(
            "SYSTEM",
            f"Recording started: {path.name}",
        )

    def stop_recording(self):
        if not self.recorder.is_recording:
            return

        path = self.recorder.path

        self.recorder.record_log(
            "SYSTEM",
            "Recording stopped",
        )

        self.recorder.stop()

        self.record_button.setText(
            "Start Recording"
        )

        self.recording_status.setText(
            "○ NOT RECORDING"
        )

        self.recording_time_label.setText(
            "00:00:00"
        )

        self.log_fault_button.setEnabled(
            False
        )

        if path is not None:
            self.statusBar().showMessage(
                f"Recording saved: {path}"
            )

            self.add_log(
                "SYSTEM",
                f"Recording saved: {path.name}",
            )

    def update_recording_clock(self):
        if not self.recorder.is_recording:
            return

        seconds = int(
            self.recorder.elapsed
        )

        if seconds != self.last_ui_snapshot_second:
            self.last_ui_snapshot_second = seconds
            self._record_ui_snapshot()

        hours, remainder = divmod(
            seconds,
            3600,
        )

        minutes, seconds = divmod(
            remainder,
            60,
        )

        self.recording_time_label.setText(
            f"{hours:02d}:"
            f"{minutes:02d}:"
            f"{seconds:02d}"
        )

    def _record_ui_snapshot(self):
        if not self.recorder.is_recording:
            return
        self.recorder.record_ui_snapshot({
            "active_tab": self.tabs.tabText(self.tabs.currentIndex()),
            "telemetry": dict(self.telemetry),
            "parameters": dict(self.parameter_values),
            "robot_state": {
                "debug_mode": self.robot_debug_mode,
                "stopped": self.robot_stopped,
            },
            "tof8x8": dict(getattr(self, "tof8x8_latest_message", {}) or {}),
            "tof8x8_view": self.tof8x8_view_mode.currentText(),
            "favourite_commands": sorted(self.favourite_commands),
            "active_commands": sorted(self.command_definitions),
            "wiring_guide": list(self.wiring_guide.entries),
            "arena_layout": {
                "point": self.arena_view.model.sensor_specs,
                "ultrasound": self.arena_view.model.ultrasound_specs,
                "matrix": self.arena_view.model.matrix_spec,
            },
            "arena_settings": self._arena_recording_settings(),
        })

    def _arena_recording_settings(self):
        view = self.arena_view
        return {
            "width": view.arena_width.value(),
            "height": view.arena_height.value(),
            "encoder_1_mm_per_count": view.encoder_1_scale.value(),
            "encoder_2_mm_per_count": view.encoder_2_scale.value(),
            "track_width_mm": view.track_width.value(),
            "cell_size_mm": view.grid_size.value(),
            "weight_gap_mm": view.weight_gap.value(),
            "matrix_fov_deg": view.matrix_fov.value(),
            "matrix_mirrored": view.model.matrix_mirrored,
        }

    def log_fault(self):
        """
        Immediately mark the current recording time as a fault.

        No dialog is shown because the point of this button is to mark
        something abnormal as quickly as possible while watching the robot.
        """
        if not self.recorder.is_recording:
            return

        elapsed = self.recorder.record_fault(
            "MANUAL FAULT MARKER"
        )

        if elapsed is None:
            return

        self.add_log(
            "FAULT",
            f"Manual fault marker logged at {elapsed:.3f} s",
        )

        self.statusBar().showMessage(
            f"FAULT MARKED at {elapsed:.3f} s"
        )

        # Give immediate visual acknowledgement without blocking the GUI.
        self.log_fault_button.setText(
            f"FAULT LOGGED @ {elapsed:.1f}s"
        )

        QTimer.singleShot(
            900,
            lambda: self.log_fault_button.setText(
                "LOG FAULT"
            ),
        )

    # =================================================================
    # UI
    # =================================================================

    def _build_ui(self):
        central = QWidget()

        self.setCentralWidget(
            central
        )

        main_layout = QVBoxLayout(
            central
        )

        # --------------------------------------------------------------
        # Connection / recording bar
        # --------------------------------------------------------------

        connection_group = QGroupBox(
            "Robot Connection"
        )

        connection_layout = QHBoxLayout(
            connection_group
        )

        connection_layout.addWidget(
            QLabel("Port:")
        )

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(250)

        connection_layout.addWidget(
            self.port_combo
        )

        self.refresh_button = QPushButton(
            "Refresh"
        )

        connection_layout.addWidget(
            self.refresh_button
        )

        connection_layout.addSpacing(10)

        connection_layout.addWidget(
            QLabel("Baud:")
        )

        self.baud_combo = QComboBox()

        for baud in (
            9600,
            19200,
            38400,
            57600,
            115200,
            230400,
            460800,
            921600,
        ):
            self.baud_combo.addItem(
                str(baud),
                baud,
            )

        saved_baud = int(
            self.settings.value(
                "baud",
                115200,
            )
        )

        baud_index = (
            self.baud_combo.findData(
                saved_baud
            )
        )

        if baud_index >= 0:
            self.baud_combo.setCurrentIndex(
                baud_index
            )

        connection_layout.addWidget(
            self.baud_combo
        )

        self.connect_button = QPushButton(
            "Connect"
        )

        self.connect_button.setMinimumWidth(
            95
        )

        connection_layout.addWidget(
            self.connect_button
        )

        self.connection_status = QLabel(
            "● DISCONNECTED"
        )

        self.connection_status.setMinimumWidth(
            145
        )

        connection_layout.addWidget(
            self.connection_status
        )

        connection_layout.addSpacing(10)

        self.record_button = QPushButton(
            "Start Recording"
        )

        self.record_button.setMinimumWidth(
            120
        )

        self.record_button.setEnabled(
            False
        )

        connection_layout.addWidget(
            self.record_button
        )

        self.log_fault_button = QPushButton(
            "LOG FAULT"
        )

        self.log_fault_button.setMinimumWidth(
            120
        )

        # It is deliberately disabled/grey until recording begins.
        self.log_fault_button.setEnabled(
            False
        )

        connection_layout.addWidget(
            self.log_fault_button
        )

        self.recording_status = QLabel(
            "○ NOT RECORDING"
        )

        self.recording_status.setMinimumWidth(
            175
        )

        connection_layout.addWidget(
            self.recording_status
        )

        self.recording_time_label = QLabel(
            "00:00:00"
        )

        self.recording_time_label.setMinimumWidth(
            65
        )

        connection_layout.addWidget(
            self.recording_time_label
        )

        connection_layout.addSpacing(8)
        connection_layout.addWidget(QLabel("Test:"))
        self.test_name_edit = QLineEdit()
        self.test_name_edit.setPlaceholderText("Straight drive PID test")
        self.test_name_edit.setMaximumWidth(190)
        connection_layout.addWidget(self.test_name_edit)

        connection_layout.addWidget(QLabel("Notes:"))
        self.test_notes_edit = QLineEdit()
        self.test_notes_edit.setPlaceholderText("Optional")
        self.test_notes_edit.setMaximumWidth(190)
        connection_layout.addWidget(self.test_notes_edit)

        connection_layout.addStretch()

        self.enter_debug_button = QPushButton(
            "Enter Debug Mode"
        )

        self.enter_debug_button.setEnabled(
            False
        )

        connection_layout.addWidget(
            self.enter_debug_button
        )

        self.exit_debug_button = QPushButton(
            "Exit Debug Mode"
        )

        self.exit_debug_button.setEnabled(
            False
        )

        connection_layout.addWidget(
            self.exit_debug_button
        )

        main_layout.addWidget(
            connection_group
        )

        health_row = QHBoxLayout()
        health_row.addWidget(QLabel("Link health:"))
        self.link_health_label = QLabel(
            "Frames/s: 0 | Signals/s: 0 | Last telemetry: — | Link/robot errors: 0"
        )
        health_row.addWidget(self.link_health_label)
        health_row.addStretch()
        main_layout.addLayout(health_row)

        self.firmware_warning_label = QLabel()
        self.firmware_warning_label.setWordWrap(True)
        self.firmware_warning_label.setStyleSheet(
            "QLabel { color: #fbbf24; background: #423315; padding: 6px; }"
        )
        self.firmware_warning_label.hide()
        main_layout.addWidget(self.firmware_warning_label)

        self.tabs = QTabWidget()

        main_layout.addWidget(
            self.tabs,
            1,
        )

        self._build_dashboard_tab()
        self._build_tof8x8_tab()
        self._build_arena_tab()
        self._build_plot_tab()
        self._build_parameter_tab()
        self._build_command_tab()
        self._build_wiring_guide_tab()
        self._build_log_tab()
        self._build_raw_tab()

        self.statusBar().showMessage(
            "Ready"
        )

    # =================================================================
    # Dashboard
    # =================================================================

    def _build_wiring_guide_tab(self):
        self.wiring_guide = WiringGuide(self.settings)
        self.wiring_guide.entries_changed.connect(self.arena_view.reload_sensor_layout)
        self.tabs.addTab(self.wiring_guide, "Wiring Guide")

    def _build_arena_tab(self):
        self.arena_view = ArenaView(self.settings)
        self.arena_view.command_requested.connect(self.execute_command)
        self.arena_view.parameter_requested.connect(self.apply_arena_parameter)
        self.tabs.addTab(self.arena_view, "Arena View")

    def apply_arena_parameter(self, name, value):
        self.parameter_values[name] = value
        self.recorder.record_parameter(name, value)
        self.bluetooth.set_parameter(name, value)
        self.add_log("TX", f"{name} = {value}")


    def _build_dashboard_tab(self):
        page = QWidget()

        main_layout = QVBoxLayout(
            page
        )

        splitter = QSplitter(
            Qt.Orientation.Horizontal
        )

        main_layout.addWidget(
            splitter,
            1,
        )

        # --------------------------------------------------------------
        # Telemetry
        # --------------------------------------------------------------

        telemetry_panel = QWidget()

        telemetry_layout = QVBoxLayout(
            telemetry_panel
        )

        top_layout = QHBoxLayout()

        title = QLabel(
            "Live Telemetry"
        )

        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)

        title.setFont(
            title_font
        )

        top_layout.addWidget(
            title
        )

        top_layout.addStretch()

        self.clear_telemetry_button = QPushButton(
            "Clear"
        )

        top_layout.addWidget(
            self.clear_telemetry_button
        )

        telemetry_layout.addLayout(
            top_layout
        )

        self.telemetry_table = QTableWidget(
            0,
            3,
        )

        self.telemetry_table.setHorizontalHeaderLabels(
            [
                "Signal",
                "Value",
                "Last Update",
            ]
        )

        self.telemetry_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )

        self.telemetry_table.setSelectionBehavior(
            QTableWidget.SelectionBehavior.SelectRows
        )

        header = (
            self.telemetry_table.horizontalHeader()
        )

        header.setStretchLastSection(
            False
        )

        header.setSectionResizeMode(
            0,
            header.ResizeMode.Stretch,
        )

        header.setSectionResizeMode(
            1,
            header.ResizeMode.ResizeToContents,
        )

        header.setSectionResizeMode(
            2,
            header.ResizeMode.ResizeToContents,
        )

        telemetry_layout.addWidget(
            self.telemetry_table,
            1,
        )

        splitter.addWidget(
            telemetry_panel
        )

        # --------------------------------------------------------------
        # Commands
        # --------------------------------------------------------------

        command_panel = QWidget()

        command_panel_layout = QVBoxLayout(
            command_panel
        )

        command_header = QHBoxLayout()

        command_title = QLabel(
            "Commands"
        )

        command_title.setFont(
            title_font
        )

        command_header.addWidget(
            command_title
        )

        command_header.addStretch()

        command_panel_layout.addLayout(
            command_header
        )

        stop_group = QGroupBox(
            "Emergency / General"
        )

        stop_layout = QHBoxLayout(
            stop_group
        )

        self.dashboard_stop_button = QPushButton(
            "STOP ROBOT"
        )

        self.dashboard_stop_button.setMinimumHeight(
            45
        )

        self.dashboard_stop_button.setEnabled(
            False
        )

        self.dashboard_stop_button.setStyleSheet(
            "QPushButton { background: #b91c1c; color: white; font-weight: bold; }"
        )

        self.dashboard_run_button = QPushButton(
            "RUN ROBOT"
        )

        self.dashboard_run_button.setMinimumHeight(
            45
        )

        self.dashboard_run_button.setEnabled(
            False
        )

        self.dashboard_run_button.setStyleSheet(
            "QPushButton { background: #15803d; color: white; font-weight: bold; }"
        )

        stop_layout.addWidget(
            self.dashboard_stop_button
        )

        stop_layout.addWidget(
            self.dashboard_run_button
        )

        command_panel_layout.addWidget(
            stop_group
        )

        keyboard_group = QGroupBox("Keyboard Drive — motor bank 2")
        keyboard_layout = QGridLayout(keyboard_group)
        self.keyboard_drive_enabled = QCheckBox("Enable arrow-key driving")
        self.keyboard_drive_enabled.setEnabled(False)
        keyboard_layout.addWidget(self.keyboard_drive_enabled, 0, 0, 1, 2)
        keyboard_layout.addWidget(QLabel("Drive power"), 1, 0)
        self.keyboard_drive_speed = QSpinBox()
        self.keyboard_drive_speed.setRange(5, 100)
        self.keyboard_drive_speed.setSingleStep(5)
        self.keyboard_drive_speed.setSuffix(" %")
        try:
            self.keyboard_drive_speed.setValue(int(
                self.settings.value("keyboard_drive/speed_percent", 30)))
        except (TypeError, ValueError):
            self.keyboard_drive_speed.setValue(30)
        keyboard_layout.addWidget(self.keyboard_drive_speed, 1, 1)
        self.keyboard_drive_status = QLabel(
            "Disabled. Enable, enter Debug Mode, then press Run Robot.\n"
            "↑ forward · ↓ reverse · ←/→ turn · release stops"
        )
        self.keyboard_drive_status.setWordWrap(True)
        keyboard_layout.addWidget(self.keyboard_drive_status, 2, 0, 1, 2)
        command_panel_layout.addWidget(keyboard_group)

        self.dashboard_command_scroll = QScrollArea()

        self.dashboard_command_scroll.setWidgetResizable(
            True
        )

        self.dashboard_command_container = QWidget()

        self.dashboard_command_layout = QVBoxLayout(
            self.dashboard_command_container
        )

        self.dashboard_command_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )

        self.dashboard_command_scroll.setWidget(
            self.dashboard_command_container
        )

        command_panel_layout.addWidget(
            self.dashboard_command_scroll,
            1,
        )

        splitter.addWidget(
            command_panel
        )

        splitter.setSizes(
            [
                950,
                500,
            ]
        )

        splitter.setStretchFactor(
            0,
            3,
        )

        splitter.setStretchFactor(
            1,
            2,
        )

        self.tabs.addTab(
            page,
            "Dashboard",
        )

    # =================================================================
    # 8x8 TOF heatmap
    # =================================================================

    def _build_tof8x8_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        controls = QHBoxLayout()
        title = QLabel("SEN0628 8x8 TOF distance map")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        controls.addWidget(title)
        controls.addStretch()

        controls.addWidget(QLabel("View:"))
        self.tof8x8_view_mode = QComboBox()
        self.tof8x8_view_mode.addItems(
            ["Raw grid", "Raw v2", "Smoothed image", "Stable detail", "Dark surface safe"]
        )
        self.tof8x8_view_mode.currentIndexChanged.connect(self._refresh_tof8x8_view)
        controls.addWidget(self.tof8x8_view_mode)

        self.tof8x8_mirror = QCheckBox("Flip left/right for robot POV")
        mirrored = str(self.settings.value("sensor/mirror_8x8", "false")).lower() == "true"
        # The installed robot's sensor is the opposite orientation to the
        # original assumption. Invert the saved choice once so existing users
        # immediately see robot-left on the left without losing the manual
        # override checkbox.
        if int(self.settings.value("sensor/8x8_orientation_version", 0)) < 2:
            mirrored = (not mirrored) if self.settings.contains("sensor/mirror_8x8") else False
            self.settings.setValue("sensor/mirror_8x8", mirrored)
            self.settings.setValue("sensor/8x8_orientation_version", 2)
        self.tof8x8_mirror.setChecked(mirrored)
        self.tof8x8_mirror.toggled.connect(self._set_tof8x8_mirror)
        if hasattr(self, "arena_view"):
            self.arena_view.set_matrix_mirrored(mirrored)
        controls.addWidget(self.tof8x8_mirror)

        self.tof8x8_auto_range = QCheckBox("Auto colour range")
        controls.addWidget(self.tof8x8_auto_range)
        controls.addWidget(QLabel("Maximum (mm):"))
        self.tof8x8_max_distance = QSpinBox()
        self.tof8x8_max_distance.setRange(100, 10000)
        self.tof8x8_max_distance.setSingleStep(100)
        self.tof8x8_max_distance.setValue(3500)
        controls.addWidget(self.tof8x8_max_distance)
        layout.addLayout(controls)

        self.tof8x8_status = QLabel("Waiting for an 8x8 TOF frame")
        self.tof8x8_latest_message = None
        self.tof8x8_frame_history = deque(maxlen=5)
        self.tof8x8_stable_values = None
        self.tof8x8_filtered_frame_number = None
        self.tof8x8_dark_safe_values = None
        self.tof8x8_far_jump_counts = [0] * 64
        layout.addWidget(self.tof8x8_status)

        grid_group = QGroupBox("Distance by zone (mm) - robot POV: left ← 8x8 → right")
        grid = QGridLayout(grid_group)
        grid.setSpacing(4)
        self.tof8x8_cells = []

        sides = QHBoxLayout()
        sides.addWidget(QLabel("ROBOT LEFT"))
        sides.addStretch()
        sides.addWidget(QLabel("ROBOT RIGHT"))
        layout.addLayout(sides)

        grid.addWidget(QLabel("Y \\ X"), 0, 0, alignment=Qt.AlignmentFlag.AlignCenter)
        for column in range(8):
            raw_column = 7 - column if self.tof8x8_mirror.isChecked() else column
            label = QLabel(f"X{raw_column}")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(label, 0, column + 1)
            if not hasattr(self, "tof8x8_column_labels"):
                self.tof8x8_column_labels = []
            self.tof8x8_column_labels.append(label)

        for row in range(8):
            label = QLabel(f"Y{row}")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            grid.addWidget(label, row + 1, 0)
            row_cells = []
            for column in range(8):
                cell = QLabel("-")
                cell.setAlignment(Qt.AlignmentFlag.AlignCenter)
                cell.setMinimumSize(62, 48)
                cell.setStyleSheet(
                    "QLabel { background: #374151; color: white; "
                    "border: 1px solid #111827; font-weight: bold; }"
                )
                grid.addWidget(cell, row + 1, column + 1)
                row_cells.append(cell)
            self.tof8x8_cells.append(row_cells)

        self.tof8x8_grid_group = grid_group
        layout.addWidget(grid_group, 1)
        self.tof8x8_image = QLabel()
        self.tof8x8_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.tof8x8_image.setMinimumSize(640, 480)
        self.tof8x8_image.setStyleSheet("QLabel { background: #111827; border: 1px solid #374151; }")
        self.tof8x8_image.hide()
        layout.addWidget(self.tof8x8_image, 1)
        legend = QLabel("Near  red    →    yellow/green    →    blue  far")
        legend.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(legend)
        self.tabs.addTab(page, "8x8 TOF")

    def on_tof8x8_frame(self, message: dict):
        self.recorder.record_matrix(message)
        self.arena_view.receive_matrix(message)
        self.tof8x8_latest_message = message
        bus_name = str(message.get("bus", "I2C"))
        available = bool(message.get("available", False))
        valid = bool(message.get("valid", False))
        frame_number = int(message.get("frame", 0))
        values = message.get("data", [])

        if not available:
            all_addresses = message.get("i2c_addresses", [])
            ack_mask = int(message.get("address_ack_mask", 0))
            address_52_ack = bool(message.get("address_52_ack", False))
            acknowledged = [
                f"0x{0x30 + index:02X}"
                for index in range(4)
                if ack_mask & (1 << index)
            ]
            if address_52_ack:
                acknowledged.append("0x52")
            detail = (
                "responding addresses: " + ", ".join(acknowledged)
                if acknowledged
                else "nothing responded at 0x30-0x33 or 0x52"
            )
            bus_detail = (
                ", ".join(f"0x{int(address):02X}" for address in all_addresses)
                if isinstance(all_addresses, list) and all_addresses
                else "none"
            )
            self.tof8x8_status.setText(
                f"SEN0628 unavailable on {bus_name} ({detail}); all bus devices: {bus_detail}"
            )
            return
        if not valid or not isinstance(values, list) or len(values) != 64:
            self.tof8x8_status.setText("SEN0628 connected, but the latest frame is invalid")
            return

        self._update_tof8x8_stable(values, frame_number)
        self._update_tof8x8_dark_safe(values, frame_number)
        view_mode = self.tof8x8_view_mode.currentText()
        if view_mode == "Raw v2":
            values = self._filter_tof8x8_isolated_pixels(values)
        elif view_mode == "Stable detail":
            values = self.tof8x8_stable_values
        elif view_mode == "Dark surface safe":
            values = self.tof8x8_dark_safe_values

        numeric = [float(value) for value in values if isinstance(value, (int, float)) and value > 0]
        if not numeric:
            self.tof8x8_status.setText("Frame contains no valid distance values")
            return

        colour_max = (
            max(numeric)
            if self.tof8x8_auto_range.isChecked()
            else float(self.tof8x8_max_distance.value())
        )
        colour_max = max(colour_max, 1.0)

        smooth_view = view_mode in (
            "Raw v2", "Smoothed image", "Stable detail", "Dark surface safe"
        )
        self.tof8x8_grid_group.setVisible(not smooth_view)
        self.tof8x8_image.setVisible(smooth_view)
        if smooth_view:
            self._render_tof8x8_image(
                values,
                colour_max,
                grey_below_200=(view_mode in ("Raw v2", "Dark surface safe")),
            )

        for index, raw_value in enumerate(values):
            display_col = 7 - index % 8 if self.tof8x8_mirror.isChecked() else index % 8
            cell = self.tof8x8_cells[index // 8][display_col]
            if not isinstance(raw_value, (int, float)) or raw_value <= 0:
                cell.setText("-")
                cell.setStyleSheet(
                    "QLabel { background: #374151; color: white; "
                    "border: 1px solid #111827; font-weight: bold; }"
                )
                continue

            ratio = min(max(float(raw_value) / colour_max, 0.0), 1.0)
            colour = QColor.fromHsvF(0.66 * ratio, 0.82, 0.88)
            text_colour = "black" if 0.12 < ratio < 0.62 else "white"
            cell.setText(str(int(raw_value)))
            cell.setStyleSheet(
                f"QLabel {{ background: {colour.name()}; color: {text_colour}; "
                "border: 1px solid #111827; font-weight: bold; }"
            )

        address = int(message.get("address", 0))
        self.tof8x8_status.setText(
            f"{self.tof8x8_view_mode.currentText()} | {bus_name} 0x{address:02X} | "
            f"frame {frame_number} | min {int(min(numeric))} mm | "
            f"max {int(max(numeric))} mm | updated {time.strftime('%H:%M:%S')}"
        )

    def _refresh_tof8x8_view(self):
        if isinstance(self.tof8x8_latest_message, dict):
            self.on_tof8x8_frame(self.tof8x8_latest_message)

    def _set_tof8x8_mirror(self, mirrored):
        self.settings.setValue("sensor/mirror_8x8", mirrored)
        for column, label in enumerate(self.tof8x8_column_labels):
            label.setText(f"X{7 - column if mirrored else column}")
        self.arena_view.set_matrix_mirrored(mirrored)
        self._refresh_tof8x8_view()

    def _render_tof8x8_image(self, values: list, colour_max: float, grey_below_200: bool = False):
        image = QImage(8, 8, QImage.Format.Format_RGB32)
        for index, raw_value in enumerate(values):
            if not isinstance(raw_value, (int, float)) or raw_value <= 0:
                colour = QColor("#374151")
            elif grey_below_200 and raw_value < 200:
                colour = QColor("#808080")
            else:
                ratio = min(max(float(raw_value) / colour_max, 0.0), 1.0)
                colour = QColor.fromHsvF(0.66 * ratio, 0.82, 0.88)
            display_col = 7 - index % 8 if self.tof8x8_mirror.isChecked() else index % 8
            image.setPixelColor(display_col, index // 8, colour)

        target = self.tof8x8_image.size()
        pixmap = QPixmap.fromImage(image).scaled(
            max(target.width() - 8, 64),
            max(target.height() - 8, 64),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.tof8x8_image.setPixmap(pixmap)

    def _filter_tof8x8_isolated_pixels(self, values: list) -> list:
        """Replace isolated spikes while retaining coherent objects and edges."""
        filtered = list(values)
        for index, current in enumerate(values):
            if not isinstance(current, (int, float)) or current <= 0:
                continue
            row, column = divmod(index, 8)
            neighbours = []
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    y, x = row + dy, column + dx
                    if 0 <= y < 8 and 0 <= x < 8:
                        value = values[y * 8 + x]
                        if isinstance(value, (int, float)) and value > 0:
                            neighbours.append(float(value))
            if len(neighbours) < 4:
                continue
            neighbours.sort()
            median = neighbours[len(neighbours) // 2]
            agreeing = sum(abs(value - median) <= 250 for value in neighbours)
            if agreeing >= 4 and abs(float(current) - median) > 600:
                filtered[index] = int(round(median))
        return filtered

    def _update_tof8x8_stable(self, values: list, frame_number: int):
        if self.tof8x8_filtered_frame_number == frame_number:
            return
        self.tof8x8_filtered_frame_number = frame_number
        self.tof8x8_frame_history.append(list(values))

        stable = []
        previous = self.tof8x8_stable_values
        for index, current in enumerate(values):
            samples = sorted(
                float(frame[index])
                for frame in self.tof8x8_frame_history
                if isinstance(frame[index], (int, float)) and frame[index] > 0
            )
            if not samples:
                stable.append(0)
                continue

            median = samples[len(samples) // 2]
            current_valid = isinstance(current, (int, float)) and current > 0
            old = previous[index] if isinstance(previous, list) and len(previous) == 64 else None

            # A substantially nearer reading may be a newly appearing obstacle;
            # accept it immediately. Farther jumps remain filtered so one noisy
            # sample cannot suddenly make an obstacle disappear.
            if current_valid and old is not None and old > 0 and float(current) < float(old) - 250:
                result = float(current)
            elif old is not None and old > 0:
                result = 0.65 * float(old) + 0.35 * median
            else:
                result = median
            stable.append(int(round(result)))

        self.tof8x8_stable_values = stable

    def _update_tof8x8_dark_safe(self, values: list, frame_number: int):
        if getattr(self, "tof8x8_dark_safe_frame_number", None) == frame_number:
            return
        self.tof8x8_dark_safe_frame_number = frame_number
        previous = self.tof8x8_dark_safe_values
        safe = list(values)

        # Nearer changes are safety-relevant and pass immediately. A large
        # farther jump must persist for three frames before replacing a known
        # nearer surface; invalid samples are held for the same short interval.
        for index, current in enumerate(values):
            old = previous[index] if isinstance(previous, list) and len(previous) == 64 else None
            current_valid = isinstance(current, (int, float)) and current > 0
            suspicious_far = (
                old is not None and old > 0 and
                (not current_valid or float(current) > float(old) + 400)
            )
            if suspicious_far:
                self.tof8x8_far_jump_counts[index] += 1
                if self.tof8x8_far_jump_counts[index] < 3:
                    safe[index] = old
            else:
                self.tof8x8_far_jump_counts[index] = 0

        # Two conservative passes repair holes at the boundary of a coherent
        # nearby surface. Iteration lets the correction reach a small cluster,
        # but the neighbour agreement requirement protects genuine openings.
        for _ in range(2):
            source = list(safe)
            for index, current in enumerate(source):
                if not isinstance(current, (int, float)) or current <= 0:
                    continue
                row, column = divmod(index, 8)
                neighbours = []
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        y, x = row + dy, column + dx
                        if 0 <= y < 8 and 0 <= x < 8:
                            value = source[y * 8 + x]
                            if isinstance(value, (int, float)) and value > 0:
                                neighbours.append(float(value))
                if len(neighbours) < 3:
                    continue
                neighbours.sort()
                median = neighbours[len(neighbours) // 2]
                agreeing = sum(abs(value - median) <= 220 for value in neighbours)
                if agreeing >= 3 and float(current) > median + 450:
                    safe[index] = int(round(median))

        self.tof8x8_dark_safe_values = safe

    # =================================================================
    # Plots
    # =================================================================

    def _build_plot_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addWidget(
            QLabel("Signal:")
        )

        self.plot_signal_combo = QComboBox()

        self.plot_signal_combo.setMinimumWidth(
            300
        )

        controls.addWidget(
            self.plot_signal_combo
        )

        self.add_plot_button = QPushButton(
            "Add to Plot"
        )

        self.remove_plot_button = QPushButton(
            "Remove Selected"
        )

        self.clear_plot_button = QPushButton(
            "Clear Plot"
        )

        controls.addWidget(
            self.add_plot_button
        )

        controls.addWidget(
            self.remove_plot_button
        )

        controls.addWidget(
            self.clear_plot_button
        )

        controls.addSpacing(20)

        controls.addWidget(
            QLabel("Time Window:")
        )

        self.time_window_combo = QComboBox()

        for text, seconds in (
            ("5 seconds", 5),
            ("10 seconds", 10),
            ("30 seconds", 30),
            ("1 minute", 60),
            ("2 minutes", 120),
            ("5 minutes", 300),
        ):
            self.time_window_combo.addItem(
                text,
                seconds,
            )

        self.time_window_combo.setCurrentIndex(
            1
        )

        controls.addWidget(
            self.time_window_combo
        )

        controls.addStretch()

        layout.addLayout(
            controls
        )

        splitter = QSplitter(
            Qt.Orientation.Horizontal
        )

        signal_container = QWidget()

        signal_layout = QVBoxLayout(
            signal_container
        )

        signal_layout.addWidget(
            QLabel("Currently plotted:")
        )

        self.active_plot_list = QListWidget()

        signal_layout.addWidget(
            self.active_plot_list
        )

        signal_container.setMaximumWidth(
            280
        )

        splitter.addWidget(
            signal_container
        )

        self.plot_widget = pg.PlotWidget()

        self.plot_widget.showGrid(
            x=True,
            y=True,
            alpha=0.3,
        )

        self.plot_widget.setLabel(
            "bottom",
            "Time",
            units="s",
        )

        self.plot_widget.setLabel(
            "left",
            "Value",
        )

        self.plot_widget.addLegend()

        splitter.addWidget(
            self.plot_widget
        )

        splitter.setStretchFactor(
            1,
            1,
        )

        layout.addWidget(
            splitter,
            1,
        )

        self.tabs.addTab(
            page,
            "Plots",
        )

    # =================================================================
    # Parameters
    # =================================================================

    def _build_parameter_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        top = QHBoxLayout()

        title = QLabel(
            "Robot Parameters"
        )

        font = QFont()
        font.setPointSize(14)
        font.setBold(True)

        title.setFont(
            font
        )

        top.addWidget(
            title
        )

        top.addStretch()

        self.refresh_definitions_button = QPushButton(
            "Refresh From Robot"
        )

        top.addWidget(
            self.refresh_definitions_button
        )

        layout.addLayout(
            top
        )

        info = QLabel(
            "Parameters advertised by the robot appear here automatically."
        )

        layout.addWidget(
            info
        )

        self.parameter_scroll = QScrollArea()

        self.parameter_scroll.setWidgetResizable(
            True
        )

        self.parameter_container = QWidget()

        self.parameter_layout = QVBoxLayout(
            self.parameter_container
        )

        self.parameter_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )

        self.parameter_scroll.setWidget(
            self.parameter_container
        )

        layout.addWidget(
            self.parameter_scroll
        )

        self.tabs.addTab(
            page,
            "Parameters",
        )

    # =================================================================
    # Commands
    # =================================================================

    def _build_command_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        top = QHBoxLayout()

        title = QLabel(
            "Robot Commands"
        )

        font = QFont()
        font.setPointSize(14)
        font.setBold(True)

        title.setFont(
            font
        )

        top.addWidget(
            title
        )

        top.addStretch()

        self.stop_button = QPushButton(
            "STOP ROBOT"
        )

        self.stop_button.setEnabled(
            False
        )

        top.addWidget(
            self.stop_button
        )

        layout.addLayout(
            top
        )

        info = QLabel(
            "Commands are arranged in a grid. Star a command to show it on the Dashboard."
        )

        layout.addWidget(
            info
        )

        self.command_scroll = QScrollArea()

        self.command_scroll.setWidgetResizable(
            True
        )

        self.command_container = QWidget()

        self.command_layout = QGridLayout(
            self.command_container
        )

        self.command_layout.setAlignment(
            Qt.AlignmentFlag.AlignTop
        )
        self.command_layout.setHorizontalSpacing(12)
        self.command_layout.setVerticalSpacing(12)
        for column in range(3):
            self.command_layout.setColumnStretch(column, 1)

        self.command_scroll.setWidget(
            self.command_container
        )

        layout.addWidget(
            self.command_scroll
        )

        self.tabs.addTab(
            page,
            "Commands",
        )

    # =================================================================
    # Logs
    # =================================================================

    def _build_log_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        controls.addStretch()

        self.clear_log_button = QPushButton(
            "Clear"
        )

        controls.addWidget(
            self.clear_log_button
        )

        layout.addLayout(
            controls
        )

        self.log_console = QTextEdit()

        self.log_console.setReadOnly(
            True
        )

        font = QFont(
            "Consolas"
        )

        font.setStyleHint(
            QFont.StyleHint.Monospace
        )

        self.log_console.setFont(
            font
        )

        layout.addWidget(
            self.log_console
        )

        self.tabs.addTab(
            page,
            "Logs",
        )

    # =================================================================
    # Raw Serial
    # =================================================================

    def _build_raw_tab(self):
        page = QWidget()

        layout = QVBoxLayout(
            page
        )

        controls = QHBoxLayout()

        self.raw_pause_checkbox = QCheckBox(
            "Pause display"
        )

        controls.addWidget(
            self.raw_pause_checkbox
        )

        controls.addStretch()

        self.clear_raw_button = QPushButton(
            "Clear"
        )

        controls.addWidget(
            self.clear_raw_button
        )

        layout.addLayout(
            controls
        )

        self.raw_console = QTextEdit()

        self.raw_console.setReadOnly(
            True
        )

        font = QFont(
            "Consolas"
        )

        font.setStyleHint(
            QFont.StyleHint.Monospace
        )

        self.raw_console.setFont(
            font
        )

        layout.addWidget(
            self.raw_console
        )

        self.tabs.addTab(
            page,
            "Raw Serial",
        )

    # =================================================================
    # Signals
    # =================================================================

    def _connect_signals(self):
        self.refresh_button.clicked.connect(
            self.refresh_ports
        )

        self.connect_button.clicked.connect(
            self.toggle_connection
        )

        self.record_button.clicked.connect(
            self.toggle_recording
        )

        self.log_fault_button.clicked.connect(
            self.log_fault
        )

        self.enter_debug_button.clicked.connect(
            lambda: self.execute_command(
                "set_debug_mode",
                {
                    "enabled": True,
                },
            )
        )

        self.exit_debug_button.clicked.connect(
            lambda: self.execute_command(
                "set_debug_mode",
                {
                    "enabled": False,
                },
            )
        )

        self.stop_button.clicked.connect(
            lambda: self.execute_command(
                "stop",
                {},
            )
        )

        self.dashboard_stop_button.clicked.connect(
            lambda: self.execute_command(
                "stop",
                {},
            )
        )

        self.dashboard_run_button.clicked.connect(
            lambda: self.execute_command(
                "run",
                {},
            )
        )

        self.keyboard_drive_enabled.toggled.connect(
            self.set_keyboard_drive_enabled
        )
        self.keyboard_drive_speed.valueChanged.connect(
            lambda value: self.settings.setValue(
                "keyboard_drive/speed_percent", value)
        )

        self.refresh_definitions_button.clicked.connect(
            self.bluetooth.request_definitions
        )

        self.clear_telemetry_button.clicked.connect(
            self.clear_telemetry
        )

        self.add_plot_button.clicked.connect(
            self.add_selected_plot
        )

        self.remove_plot_button.clicked.connect(
            self.remove_selected_plot
        )

        self.clear_plot_button.clicked.connect(
            self.clear_plots
        )

        self.clear_log_button.clicked.connect(
            self.log_console.clear
        )

        self.clear_raw_button.clicked.connect(
            self.raw_console.clear
        )

        self.bluetooth.connection_changed.connect(
            self.on_connection_changed
        )

        self.bluetooth.telemetry_received.connect(
            self.on_telemetry
        )

        self.bluetooth.tof_8x8_received.connect(
            self.on_tof8x8_frame
        )

        self.bluetooth.parameter_definition_received.connect(
            self.on_parameter_definition
        )

        self.bluetooth.parameter_value_received.connect(
            self.on_parameter_value
        )

        self.bluetooth.command_definition_received.connect(
            self.on_command_definition
        )

        self.bluetooth.log_received.connect(
            self.on_log
        )

        self.bluetooth.raw_received.connect(
            self.on_raw
        )

        self.bluetooth.error_received.connect(
            self.on_error
        )

        self.bluetooth.robot_state_received.connect(
            self.on_robot_state
        )

    def update_link_health(self):
        now = time.monotonic()
        cutoff = now - 1.0

        while self.raw_line_times and self.raw_line_times[0] < cutoff:
            self.raw_line_times.popleft()

        while (
            self.telemetry_event_times
            and self.telemetry_event_times[0] < cutoff
        ):
            self.telemetry_event_times.popleft()

        if self.last_telemetry_monotonic is None:
            age_text = "—"
        else:
            age_text = (
                f"{now - self.last_telemetry_monotonic:.2f} s"
            )

        alert = ""
        if self.bluetooth.is_connected():
            if self.last_telemetry_monotonic is None and self.connection_started_monotonic is not None:
                if now - self.connection_started_monotonic > 3:
                    alert = " | Port open, no telemetry; check CH9143 link and robot power"
                    self.connection_status.setText(
                        f"● PORT OPEN — NO ROBOT DATA — {self.bluetooth.port or ''}"
                    )
            elif self.last_telemetry_monotonic is not None and now - self.last_telemetry_monotonic > 3:
                alert = " | Telemetry stalled; check Raw Serial"
                self.connection_status.setText(
                    f"● PORT OPEN — ROBOT SILENT — {self.bluetooth.port or ''}"
                )
        self.link_health_label.setText(
            f"Frames/s: {len(self.raw_line_times)} | "
            f"Signals/s: {len(self.telemetry_event_times)} | "
            f"Last telemetry: {age_text} | "
            f"Link/robot errors: {self.protocol_error_count}{alert}"
        )

    # =================================================================
    # Ports / connection
    # =================================================================

    def refresh_ports(self):
        ports = self.bluetooth.available_ports()

        current_device = (
            self.port_combo.currentData()
        )

        saved_device = self.settings.value(
            "port",
            "",
        )

        target_device = (
            current_device
            or saved_device
        )

        # This project's matched receiver identifies itself as CH9143.
        # Prefer it over the Teensy's USB programming port, including when an
        # old COM-port choice was saved in QSettings.
        ch9143_devices = {
            port["device"]
            for port in ports
            if "CH9143" in port["description"].upper()
        }
        if ch9143_devices and target_device not in ch9143_devices:
            target_device = sorted(ch9143_devices)[0]

        existing_devices = {
            self.port_combo.itemData(i)
            for i in range(
                self.port_combo.count()
            )
        }

        new_devices = {
            port["device"]
            for port in ports
        }

        if existing_devices == new_devices:
            return

        self.port_combo.blockSignals(
            True
        )

        self.port_combo.clear()

        for port in ports:
            device = port["device"]
            description = port["description"]

            label = (
                f"{device} — {description}"
                if description
                else device
            )

            self.port_combo.addItem(
                label,
                device,
            )

        if target_device:
            index = (
                self.port_combo.findData(
                    target_device
                )
            )

            if index >= 0:
                self.port_combo.setCurrentIndex(
                    index
                )

        self.port_combo.blockSignals(
            False
        )

    def toggle_connection(self):
        if self.bluetooth.is_connected():
            self.bluetooth.disconnect_port()
            return

        port = self.port_combo.currentData()

        if not port:
            QMessageBox.warning(
                self,
                "No Serial Port",
                "No serial port is available.\n\n"
                "Connect the adapter and press Refresh.",
            )
            return

        baudrate = int(
            self.baud_combo.currentData()
        )

        self.settings.setValue(
            "port",
            port,
        )

        self.settings.setValue(
            "baud",
            baudrate,
        )

        self.statusBar().showMessage(
            f"Connecting to {port}..."
        )

        self.connect_button.setEnabled(
            False
        )

        self.bluetooth.connect_port(
            port,
            baudrate,
        )

    def on_connection_changed(
        self,
        connected: bool,
        port: str,
    ):
        self.connection_started_monotonic = time.monotonic() if connected else None
        self.last_telemetry_monotonic = None
        self.raw_line_times.clear()
        self.telemetry_event_times.clear()
        self.session_signal_names.clear()
        self.firmware_warning_label.hide()
        self.connect_button.setEnabled(
            True
        )

        if connected:
            self.connect_button.setText(
                "Disconnect"
            )

            self.connection_status.setText(
                f"● PORT OPEN — WAITING FOR ROBOT — {port}"
            )

            self.port_combo.setEnabled(
                False
            )

            self.baud_combo.setEnabled(
                False
            )

            self.refresh_button.setEnabled(
                False
            )

            self.enter_debug_button.setEnabled(
                True
            )

            self.exit_debug_button.setEnabled(
                True
            )

            self.stop_button.setEnabled(
                True
            )

            self.dashboard_stop_button.setEnabled(
                True
            )

            self.dashboard_run_button.setEnabled(
                True
            )

            self.record_button.setEnabled(
                True
            )

            self.keyboard_drive_enabled.setEnabled(True)

            # Still disabled until the user starts recording.
            self.log_fault_button.setEnabled(
                False
            )

            self.statusBar().showMessage(
                f"Serial port open on {port}; waiting for robot telemetry"
            )

            self.add_log(
                "SYSTEM",
                f"Serial port open on {port}; waiting for robot telemetry",
            )

        else:
            self.set_keyboard_drive_enabled(False)
            self.keyboard_drive_enabled.blockSignals(True)
            self.keyboard_drive_enabled.setChecked(False)
            self.keyboard_drive_enabled.blockSignals(False)
            self.keyboard_drive_enabled.setEnabled(False)
            if self.recorder.is_recording:
                self.stop_recording()

            self.connect_button.setText(
                "Connect"
            )

            self.connection_status.setText(
                "● DISCONNECTED"
            )

            self.port_combo.setEnabled(
                True
            )

            self.baud_combo.setEnabled(
                True
            )

            self.refresh_button.setEnabled(
                True
            )

            self.enter_debug_button.setEnabled(
                False
            )

            self.exit_debug_button.setEnabled(
                False
            )

            self.stop_button.setEnabled(
                False
            )

            self.dashboard_stop_button.setEnabled(
                False
            )

            self.dashboard_run_button.setEnabled(
                False
            )

            self.record_button.setEnabled(
                False
            )

            self.log_fault_button.setEnabled(
                False
            )

            self.statusBar().showMessage(
                "Disconnected"
            )

            self.add_log(
                "SYSTEM",
                "Disconnected",
            )

    # =================================================================
    # Telemetry
    # =================================================================

    def on_telemetry(
        self,
        name: str,
        value: Any,
        robot_timestamp: Any,
    ):
        self.arena_view.receive_telemetry(name, value, robot_timestamp)
        now = time.monotonic()
        self.session_signal_names.add(name)
        if self.bluetooth.is_connected() and not self.connection_status.text().startswith("● CONNECTED"):
            self.connection_status.setText(f"● CONNECTED — {self.bluetooth.port or ''}")
        if name == "imu.available":
            self.firmware_warning_label.hide()
        elif (name == "tof.left.available" and
              "tof.front.available" in self.session_signal_names and
              "imu.available" not in self.session_signal_names):
            self.firmware_warning_label.setText(
                "Robot telemetry is live, but the Teensy is running the older "
                "front/left TOF firmware. Upload the current PlatformIO build "
                "to enable the IMU and six-sensor Arena View."
            )
            self.firmware_warning_label.show()
        self.telemetry_event_times.append(now)
        self.last_telemetry_monotonic = now

        self.recorder.record_telemetry(
            name,
            value,
            robot_timestamp,
        )

        self.telemetry[
            name
        ] = value

        if (
            isinstance(
                value,
                (int, float),
            )
            and not isinstance(
                value,
                bool,
            )
        ):
            if (
                name
                not in self.telemetry_history
            ):
                self.telemetry_history[
                    name
                ] = deque(
                    maxlen=self.MAX_HISTORY_POINTS
                )

            elapsed = (
                now
                - self.start_time
            )

            self.telemetry_history[
                name
            ].append(
                (
                    elapsed,
                    float(value),
                )
            )

            if (
                self.plot_signal_combo.findText(
                    name
                )
                < 0
            ):
                self.plot_signal_combo.addItem(
                    name
                )

        if name not in self.telemetry_rows:
            row = (
                self.telemetry_table.rowCount()
            )

            self.telemetry_table.insertRow(
                row
            )

            self.telemetry_rows[
                name
            ] = row

            self.telemetry_table.setItem(
                row,
                0,
                QTableWidgetItem(name),
            )

        row = self.telemetry_rows[
            name
        ]

        self.telemetry_table.setItem(
            row,
            1,
            QTableWidgetItem(
                self.format_value(
                    value
                )
            ),
        )

        self.telemetry_table.setItem(
            row,
            2,
            QTableWidgetItem(
                time.strftime(
                    "%H:%M:%S"
                )
            ),
        )

    @staticmethod
    def format_value(
        value: Any,
    ) -> str:
        if isinstance(
            value,
            float,
        ):
            return f"{value:.6g}"

        if isinstance(
            value,
            bool,
        ):
            return (
                "TRUE"
                if value
                else "FALSE"
            )

        return str(value)

    def clear_telemetry(self):
        self.telemetry.clear()
        self.telemetry_rows.clear()
        self.telemetry_history.clear()

        self.telemetry_table.setRowCount(
            0
        )

        self.plot_signal_combo.clear()

        self.clear_plots()

        self.start_time = time.monotonic()

    # =================================================================
    # Live plotting
    # =================================================================

    def add_selected_plot(self):
        name = (
            self.plot_signal_combo.currentText()
        )

        if not name:
            return

        if name in self.plot_curves:
            return

        curve = self.plot_widget.plot(
            [],
            [],
            name=name,
        )

        self.plot_curves[
            name
        ] = curve

        self.active_plot_list.addItem(
            name
        )

    def remove_selected_plot(self):
        selected = (
            self.active_plot_list.selectedItems()
        )

        for item in selected:
            name = item.text()

            curve = self.plot_curves.pop(
                name,
                None,
            )

            if curve is not None:
                self.plot_widget.removeItem(
                    curve
                )

            self.active_plot_list.takeItem(
                self.active_plot_list.row(
                    item
                )
            )

    def clear_plots(self):
        for curve in (
            self.plot_curves.values()
        ):
            self.plot_widget.removeItem(
                curve
            )

        self.plot_curves.clear()

        self.active_plot_list.clear()

    def update_plot(self):
        if not self.plot_curves:
            return

        now = (
            time.monotonic()
            - self.start_time
        )

        window = float(
            self.time_window_combo.currentData()
        )

        minimum_time = (
            now
            - window
        )

        for name, curve in (
            self.plot_curves.items()
        ):
            history = (
                self.telemetry_history.get(
                    name
                )
            )

            if not history:
                continue

            x = []
            y = []

            for timestamp, value in history:
                if timestamp >= minimum_time:
                    x.append(
                        timestamp
                        - now
                    )

                    y.append(
                        value
                    )

            curve.setData(
                x,
                y,
            )

        self.plot_widget.setXRange(
            -window,
            0,
            padding=0,
        )

    # =================================================================
    # Parameters
    # =================================================================

    def on_parameter_definition(
        self,
        definition: dict,
    ):
        name = str(
            definition.get(
                "name",
                "",
            )
        )

        if not name:
            return

        self.parameter_definitions[name] = dict(definition)
        if "value" in definition:
            self.parameter_values[name] = definition["value"]
        elif (
            "default" in definition
            and name not in self.parameter_values
        ):
            self.parameter_values[name] = definition["default"]

        if name in self.parameter_editors:
            if "value" in definition:
                self.parameter_editors[
                    name
                ].set_value(
                    definition[
                        "value"
                    ]
                )
            return

        group = QGroupBox(
            str(
                definition.get(
                    "label",
                    name,
                )
            )
        )

        layout = QVBoxLayout(
            group
        )

        description = definition.get(
            "description"
        )

        if description:
            text = QLabel(
                str(description)
            )

            text.setWordWrap(
                True
            )

            layout.addWidget(
                text
            )

        editor = ValueEditor(
            definition
        )

        self.parameter_editors[
            name
        ] = editor

        layout.addWidget(
            editor
        )

        unit = definition.get(
            "unit"
        )

        if unit:
            layout.addWidget(
                QLabel(
                    f"Unit: {unit}"
                )
            )

        apply_button = QPushButton(
            "Apply"
        )

        apply_button.clicked.connect(
            lambda checked=False,
            parameter_name=name,
            parameter_editor=editor:
            self.apply_parameter(
                parameter_name,
                parameter_editor,
            )
        )

        layout.addWidget(
            apply_button
        )

        self.parameter_layout.addWidget(
            group
        )

    def apply_parameter(
        self,
        name: str,
        editor: ValueEditor,
    ):
        value = editor.value()
        self.parameter_values[name] = value

        self.recorder.record_parameter(
            name,
            value,
        )

        self.bluetooth.set_parameter(
            name,
            value,
        )

        self.add_log(
            "TX",
            f"{name} = {value}",
        )

    def on_parameter_value(
        self,
        name: str,
        value: Any,
    ):
        self.parameter_values[name] = value

        editor = (
            self.parameter_editors.get(
                name
            )
        )

        if editor is not None:
            editor.set_value(
                value
            )

    # =================================================================
    # Commands
    # =================================================================

    def on_command_definition(
        self,
        definition: dict,
    ):
        name = str(
            definition.get(
                "name",
                "",
            )
        )

        if not name:
            return

        self.command_definitions[name] = dict(definition)

        if name not in self.command_widgets:
            command_widget = CommandWidget(
                definition,
                self.execute_command,
                self.set_command_favourite,
                name in self.favourite_commands,
            )

            self.command_widgets[
                name
            ] = command_widget

            index = len(self.command_widgets) - 1
            self.command_layout.addWidget(command_widget, index // 3, index % 3)

        if name in self.favourite_commands:
            self.add_dashboard_command(name)

    def add_dashboard_command(self, name):
        if name in self.dashboard_command_widgets:
            return
        definition = self.command_definitions.get(name)
        if definition is None:
            return

        dashboard_widget = CommandWidget(
            definition,
            self.execute_command,
            self.set_command_favourite,
            True,
        )

        self.dashboard_command_widgets[name] = dashboard_widget
        self.dashboard_command_layout.addWidget(dashboard_widget)

    def set_command_favourite(self, name, favourite):
        if favourite:
            self.favourite_commands.add(name)
        else:
            self.favourite_commands.discard(name)

        self.settings.setValue(
            "favourite_commands",
            json.dumps(sorted(self.favourite_commands)),
        )

        command_widget = self.command_widgets.get(name)
        if command_widget is not None:
            command_widget.set_favourite(favourite)

        if favourite:
            self.add_dashboard_command(name)
        else:
            dashboard_widget = self.dashboard_command_widgets.pop(name, None)
            if dashboard_widget is not None:
                self.dashboard_command_layout.removeWidget(dashboard_widget)
                dashboard_widget.setParent(None)
                dashboard_widget.deleteLater()

    def execute_command(
        self,
        name: str,
        arguments: dict,
    ):
        self.recorder.record_command(
            name,
            arguments,
        )

        self.bluetooth.send_command(name, **arguments)

        if arguments:
            argument_text = ", ".join(
                f"{key}={value}"
                for key, value
                in arguments.items()
            )

            text = (
                f"{name}("
                f"{argument_text}"
                f")"
            )
        else:
            text = (
                f"{name}()"
            )

        self.add_log(
            "TX",
            text,
        )

    # =================================================================
    # Keyboard drive (second 203 motor bank)
    # =================================================================

    def set_keyboard_drive_enabled(self, enabled: bool):
        if not enabled:
            self.keyboard_drive_keys.clear()
            self._send_keyboard_drive_output(0, 0, force=True)
            self.keyboard_drive_status.setText(
                "Disabled. Enable, enter Debug Mode, then press Run Robot.\n"
                "↑ forward · ↓ reverse · ←/→ turn · release stops"
            )
            return
        self.keyboard_drive_status.setText(
            "Armed. Waiting for arrow key.\n"
            "↑ forward · ↓ reverse · ←/→ turn · release stops"
        )

    def _keyboard_drive_ready(self):
        return (
            self.bluetooth.is_connected()
            and self.robot_debug_mode is True
            and self.robot_stopped is False
        )

    def _keyboard_drive_values(self):
        speed = self.keyboard_drive_speed.value()
        forward = (-speed if Qt.Key.Key_Up in self.keyboard_drive_keys else 0)
        forward += speed if Qt.Key.Key_Down in self.keyboard_drive_keys else 0
        turn = speed if Qt.Key.Key_Left in self.keyboard_drive_keys else 0
        turn -= speed if Qt.Key.Key_Right in self.keyboard_drive_keys else 0
        channel_a = max(-speed, min(speed, forward + turn))
        channel_b = max(-speed, min(speed, forward - turn))
        return channel_a, channel_b

    def _send_keyboard_drive_output(self, channel_a, channel_b, force=False, quiet=False):
        output = (int(channel_a), int(channel_b))
        if output == self.keyboard_drive_last_output and not force:
            return
        self.keyboard_drive_last_output = output
        if not self.bluetooth.is_connected():
            return
        packet = {"type": "drive", "a": output[0], "b": output[1]}
        if quiet:
            self.bluetooth.send_message(packet)
        else:
            self.recorder.record_command("keyboard_drive", {
                "channel_a_percent": output[0],
                "channel_b_percent": output[1],
            })
            self.bluetooth.send_message(packet)
            self.add_log(
                "TX", f"keyboard_drive(a={output[0]}, b={output[1]})")
        self.keyboard_drive_status.setText(
            f"Bank 2 output: left A {output[0]:+d}% · right B {output[1]:+d}%\n"
            "Release all arrows to stop"
        )

    def _update_keyboard_drive(self):
        if not self._keyboard_drive_ready():
            self.keyboard_drive_keys.clear()
            self._send_keyboard_drive_output(0, 0)
            self.keyboard_drive_status.setText(
                "Not ready: connect, enter Debug Mode, then press Run Robot."
            )
            return
        self._send_keyboard_drive_output(*self._keyboard_drive_values())

    def _keyboard_drive_heartbeat(self):
        if (self.keyboard_drive_enabled.isChecked()
                and self.keyboard_drive_keys
                and self._keyboard_drive_ready()):
            self._send_keyboard_drive_output(
                *self._keyboard_drive_values(), force=True, quiet=True)

    def eventFilter(self, watched, event):
        if hasattr(self, "keyboard_drive_enabled"):
            if event.type() in (
                QEvent.Type.ApplicationDeactivate,
                QEvent.Type.WindowDeactivate,
            ):
                if self.keyboard_drive_enabled.isChecked():
                    self.keyboard_drive_keys.clear()
                    self._send_keyboard_drive_output(0, 0)
            elif (self.keyboard_drive_enabled.isChecked()
                  and event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease)
                  and event.key() in (
                      Qt.Key.Key_Up, Qt.Key.Key_Down,
                      Qt.Key.Key_Left, Qt.Key.Key_Right,
                  )):
                if event.isAutoRepeat():
                    return True
                if event.type() == QEvent.Type.KeyPress:
                    self.keyboard_drive_keys.add(event.key())
                else:
                    self.keyboard_drive_keys.discard(event.key())
                self._update_keyboard_drive()
                return True
        return super().eventFilter(watched, event)


    # =================================================================
    # Logs / state / raw
    # =================================================================

    def on_log(
        self,
        level: str,
        message: str,
    ):
        self.recorder.record_log(
            level,
            message,
        )

        self.add_log(
            level,
            message,
        )

    def add_log(
        self,
        level: str,
        message: str,
    ):
        timestamp = time.strftime(
            "%H:%M:%S"
        )

        self.log_console.append(
            f"[{timestamp}] "
            f"[{level}] "
            f"{message}"
        )

    def on_raw(
        self,
        text: str,
    ):
        self.raw_line_times.append(
            time.monotonic()
        )

        # Recording is independent of whether the Raw Serial tab is paused.
        self.recorder.record_raw(
            text
        )

        if self.raw_pause_checkbox.isChecked():
            return

        self.raw_console.append(
            text
        )

    def on_robot_state(
        self,
        state: dict,
    ):
        self.recorder.record_state(
            state
        )

        debug_enabled = state.get(
            "debug_mode"
        )
        stopped = state.get("stopped")
        if debug_enabled is not None:
            self.robot_debug_mode = bool(debug_enabled)
        if stopped is not None:
            self.robot_stopped = bool(stopped)
        if (self.robot_debug_mode is not True or self.robot_stopped is True):
            self.keyboard_drive_keys.clear()
            self._send_keyboard_drive_output(0, 0)
            if self.keyboard_drive_enabled.isChecked():
                self.keyboard_drive_status.setText(
                    "Not ready: enter Debug Mode, then press Run Robot."
                )

        if debug_enabled is True:
            self.statusBar().showMessage(
                "Robot is in DEBUG mode"
            )

        elif debug_enabled is False:
            self.statusBar().showMessage(
                "Robot is in NORMAL mode"
            )

    def on_error(
        self,
        message: str,
    ):
        self.protocol_error_count += 1

        self.add_log(
            "ERROR",
            message,
        )

        self.statusBar().showMessage(
            message
        )

    # =================================================================
    # Shutdown
    # =================================================================

    def closeEvent(
        self,
        event,
    ):
        if self.recorder.is_recording:
            self.stop_recording()

        self.set_keyboard_drive_enabled(False)
        self.bluetooth.disconnect_port()

        event.accept()


def main():
    app = QApplication(
        sys.argv
    )

    app.setApplicationName(
        "Robot Debug Console"
    )

    window = RobotDebugGUI()

    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()
