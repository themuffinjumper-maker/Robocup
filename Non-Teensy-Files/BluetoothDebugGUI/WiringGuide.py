"""Editable, locally persisted CPU-side wiring guide for the debug console.

This is a documentation view. Editing it never changes Teensy pin assignments.
"""

from __future__ import annotations

import json

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QPen
from PyQt6.QtWidgets import (
    QComboBox,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)


DEFAULT_ENTRIES = [
    {"kind": "Comms", "device": "Bluetooth CH9143", "connector": "SERIAL1", "pins": "RX1 D0 / TX1 D1", "notes": "Wireless debug link"},
    {"kind": "Servo", "device": "Herkulex bus", "connector": "SERIAL2", "pins": "Serial2 TX/RX", "notes": "Smart servos by ID"},
    {"kind": "Motor driver", "device": "203 driver 1 — A", "connector": "Digital", "pins": "D27", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 1 — B", "connector": "Digital", "pins": "D26", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 2 — A", "connector": "Digital", "pins": "D25", "notes": "Servo pulse; 1500 µs neutral"},
    {"kind": "Motor driver", "device": "203 driver 2 — B", "connector": "Digital", "pins": "D15 / A1", "notes": "Conflicts with planned IR_RIGHT input"},
    {"kind": "Servo", "device": "HX12K A", "connector": "Digital Raw 1 / level shift", "pins": "D33", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K B", "connector": "Digital Raw 1 / level shift", "pins": "D32", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K C", "connector": "Digital Raw 1 / level shift", "pins": "D31", "notes": "Level-shifted servo output"},
    {"kind": "Servo", "device": "HX12K D", "connector": "Digital Raw 1 / level shift", "pins": "D30", "notes": "Level-shifted servo output"},
    {"kind": "Sensor", "device": "Dual encoder board", "connector": "Digital Raw 2", "pins": "E1 A:D2 B:D3 / E2 A:D4 B:D5", "notes": "Quadrature; 3.3 V signals only"},
    {"kind": "Sensor", "device": "Inductive proximity sensor", "connector": "Digital", "pins": "D21", "notes": "Active-low digital input; INPUT_PULLUP; 3.3 V maximum"},
    {"kind": "Sensor", "device": "Ultrasound Left (A)", "connector": "Ultrasound interface / 8-pin", "pins": "TRIG D14 / ECHO D24", "notes": "Left ultrasonic range sensor; channel A; 3.3 V maximum"},
    {"kind": "Sensor", "device": "Ultrasound Right (B)", "connector": "Ultrasound interface / 8-pin", "pins": "TRIG D22 / ECHO D20", "notes": "Right ultrasonic range sensor; channel B; 3.3 V maximum"},
    {"kind": "Sensor", "device": "TOF Long Top Mid Left", "connector": "XSHUT1", "pins": "I2C0 / XSHUT1", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Long Top Mid Right", "connector": "XSHUT2", "pins": "I2C0 / XSHUT2", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Mid Left", "connector": "XSHUT5", "pins": "I2C0 / XSHUT5", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Mid Right", "connector": "XSHUT0", "pins": "I2C0 / XSHUT0", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Right Right", "connector": "XSHUT3", "pins": "I2C0 / XSHUT3", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Short Bottom Left Left", "connector": "XSHUT4", "pins": "I2C0 / XSHUT4", "notes": "Configured in debug_config.json"},
    {"kind": "Sensor", "device": "TOF Long Top Right Right", "connector": "XSHUT6", "pins": "I2C0 / XSHUT6", "notes": "Long-range angled navigation sensor"},
    {"kind": "Sensor", "device": "TOF Long Top Left Left", "connector": "XSHUT7", "pins": "I2C0 / XSHUT7", "notes": "Long-range angled navigation sensor"},
    {"kind": "Sensor", "device": "SEN0628 8×8 TOF", "connector": "Raw I2C1", "pins": "3V / G / SC / SD", "notes": "Wire1, I2C address 0x33"},
    {"kind": "Sensor", "device": "BNO055 IMU", "connector": "Raw I2C1", "pins": "3V / G / SC / SD", "notes": "Wire1, I2C address 0x28/0x29"},
]

COLUMNS = ("Type", "Device", "CPU connector", "CPU pin(s)", "Notes")
FIELDS = ("kind", "device", "connector", "pins", "notes")
COLORS = {
    "Comms": QColor("#67b7f7"),
    "Motor": QColor("#f6a768"),
    "Motor driver": QColor("#f6a768"),
    "Servo": QColor("#b39df5"),
    "Sensor": QColor("#7dd6ab"),
}


class WiringGuide(QWidget):
    SETTINGS_KEY = "wiring_guide/entries_v1"
    entries_changed = pyqtSignal()

    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.entries = self._load_entries()
        self._populating = False

        layout = QVBoxLayout(self)
        heading = QLabel("CPU wiring guide")
        heading.setStyleSheet("font-size: 20px; font-weight: 700;")
        layout.addWidget(heading)
        layout.addWidget(QLabel(
            "Edit the table or add a device below. Changes save automatically on this computer. "
            "This guide does not change Teensy firmware pin assignments."
        ))

        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter, 1)

        self.scene = QGraphicsScene(self)
        self.diagram = QGraphicsView(self.scene)
        self.diagram.setRenderHints(self.diagram.renderHints())
        self.diagram.setMinimumHeight(250)
        splitter.addWidget(self.diagram)

        table_panel = QWidget()
        table_layout = QVBoxLayout(table_panel)
        self.table = QTableWidget(0, len(COLUMNS))
        self.table.setHorizontalHeaderLabels(COLUMNS)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.itemChanged.connect(self._table_changed)
        table_layout.addWidget(self.table, 1)

        form = QHBoxLayout()
        self.kind = QComboBox()
        self.kind.setEditable(True)
        self.kind.addItems(("Sensor", "Motor", "Motor driver", "Servo", "Comms", "Other"))
        self.device = QLineEdit()
        self.device.setPlaceholderText("Device name")
        self.connector = QLineEdit()
        self.connector.setPlaceholderText("CPU connector")
        self.pins = QLineEdit()
        self.pins.setPlaceholderText("CPU pin(s)")
        self.notes = QLineEdit()
        self.notes.setPlaceholderText("Notes / address")
        for widget in (self.kind, self.device, self.connector, self.pins, self.notes):
            form.addWidget(widget)
        table_layout.addLayout(form)

        buttons = QHBoxLayout()
        self.add_button = QPushButton("Add device")
        self.remove_button = QPushButton("Remove selected")
        self.add_button.clicked.connect(self.add_entry)
        self.remove_button.clicked.connect(self.remove_selected)
        buttons.addWidget(self.add_button)
        buttons.addWidget(self.remove_button)
        buttons.addStretch()
        table_layout.addLayout(buttons)
        splitter.addWidget(table_panel)
        splitter.setSizes((420, 360))

        self._refresh()

    def _load_entries(self):
        raw = self.settings.value(self.SETTINGS_KEY)
        if raw is None:
            return [entry.copy() for entry in DEFAULT_ENTRIES]
        try:
            entries = json.loads(str(raw))
            if not isinstance(entries, list):
                raise ValueError("Not a list")
            cleaned = [
                {field: str(entry.get(field, "")) for field in FIELDS}
                for entry in entries if isinstance(entry, dict)
            ]
            # Existing installations already have a saved guide; add the newly
            # confirmed encoder connection once, without replacing user edits.
            if not self.settings.value("wiring_guide/encoder_raw2_migrated", False, type=bool):
                if not any("Digital Raw 2" in entry["connector"] or
                           "encoder" in entry["device"].lower() for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["device"] == "Dual encoder board"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/encoder_raw2_migrated", True)
                self.settings.sync()

            # Add the confirmed XSHUT4 sensor once while retaining any custom
            # XSHUT4 entry/label already created in the live Wiring Guide.
            if not self.settings.value("wiring_guide/xshut4_migrated", False, type=bool):
                if not any(entry["connector"].strip().upper() == "XSHUT4"
                           for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["connector"] == "XSHUT4"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/xshut4_migrated", True)
                self.settings.sync()

            # Add the confirmed D21 proximity input once without replacing any
            # custom wiring rows or labels.
            if not self.settings.value("wiring_guide/inductive_d21_migrated", False, type=bool):
                if not any("D21" in entry["pins"].upper() or
                           "inductive" in entry["device"].lower() for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["device"] == "Inductive proximity sensor"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/inductive_d21_migrated", True)
                self.settings.sync()

            if not self.settings.value("wiring_guide/ultrasound_d24_d14_migrated", False, type=bool):
                if not any("D24" in entry["pins"].upper() and
                           ("D14" in entry["pins"].upper() or "D22" in entry["pins"].upper())
                           for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["device"] == "Ultrasound Left (A)"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/ultrasound_d24_d14_migrated", True)
                self.settings.sync()

            if not self.settings.value("wiring_guide/ultrasound_d20_d22_migrated", False, type=bool):
                if not any("D20" in entry["pins"].upper() and
                           ("D22" in entry["pins"].upper() or "D14" in entry["pins"].upper())
                           for entry in cleaned):
                    cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                        if entry["device"] == "Ultrasound Right (B)"))
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/ultrasound_d20_d22_migrated", True)
                self.settings.sync()

            if not self.settings.value("wiring_guide/ultrasound_echo_swap_migrated", False, type=bool):
                changed = False
                for entry in cleaned:
                    pins = entry["pins"].strip().upper()
                    if pins == "TRIG D24 / ECHO D14":
                        entry["pins"] = "TRIG D24 / ECHO D22"
                        entry["notes"] = "Test mapping after swapping the two echo lines; 3.3 V maximum"
                        changed = True
                    elif pins == "TRIG D20 / ECHO D22":
                        entry["pins"] = "TRIG D20 / ECHO D14"
                        entry["notes"] = "Test mapping after swapping the two echo lines; 3.3 V maximum"
                        changed = True
                if changed:
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/ultrasound_echo_swap_migrated", True)
                self.settings.sync()

            if not self.settings.value("wiring_guide/ultrasound_direction_swap_migrated", False, type=bool):
                changed = False
                for entry in cleaned:
                    pins = entry["pins"].strip().upper()
                    if pins in ("TRIG D24 / ECHO D14", "TRIG D24 / ECHO D22"):
                        entry["pins"] = "TRIG D14 / ECHO D24"
                        entry["notes"] = "Original D24/D14 pair with signal directions reversed; 3.3 V maximum"
                        changed = True
                    elif pins in ("TRIG D20 / ECHO D22", "TRIG D20 / ECHO D14"):
                        entry["pins"] = "TRIG D22 / ECHO D20"
                        entry["notes"] = "Original D20/D22 pair with signal directions reversed; 3.3 V maximum"
                        changed = True
                if changed:
                    self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/ultrasound_direction_swap_migrated", True)
                self.settings.sync()

            # Rename the old generic A/B rows once so existing installations get
            # the physical left/right naming used by Arena View. User-created
            # custom ultrasound names are left alone.
            if not self.settings.value(
                    "wiring_guide/ultrasound_left_right_names_migrated", False, type=bool):
                changed = False
                for entry in cleaned:
                    old_name = entry["device"].strip().lower()
                    if old_name == "ultrasound a":
                        entry["device"] = "Ultrasound Left (A)"
                        entry["notes"] = "Left ultrasonic range sensor; channel A; 3.3 V maximum"
                        changed = True
                    elif old_name == "ultrasound b":
                        entry["device"] = "Ultrasound Right (B)"
                        entry["notes"] = "Right ultrasonic range sensor; channel B; 3.3 V maximum"
                        changed = True
                if changed:
                    self.settings.setValue(
                        self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False)
                    )
                self.settings.setValue(
                    "wiring_guide/ultrasound_left_right_names_migrated", True
                )
                self.settings.sync()

            # Add the two outer top long-range sensors after the established
            # hardware migrations, preserving the stable order of older rows.
            if not self.settings.value("wiring_guide/xshut6_xshut7_migrated", False, type=bool):
                existing_ports = {entry["connector"].strip().upper() for entry in cleaned}
                for port in ("XSHUT6", "XSHUT7"):
                    if port not in existing_ports:
                        cleaned.append(next(entry.copy() for entry in DEFAULT_ENTRIES
                                            if entry["connector"] == port))
                self.settings.setValue(self.SETTINGS_KEY, json.dumps(cleaned, ensure_ascii=False))
                self.settings.setValue("wiring_guide/xshut6_xshut7_migrated", True)
                self.settings.sync()

            return cleaned
        except (TypeError, ValueError):
            return [entry.copy() for entry in DEFAULT_ENTRIES]

    def _save(self):
        self.settings.setValue(self.SETTINGS_KEY, json.dumps(self.entries, ensure_ascii=False))
        self.settings.sync()
        self.entries_changed.emit()

    def _refresh(self):
        self._populating = True
        self.table.setRowCount(len(self.entries))
        for row, entry in enumerate(self.entries):
            for column, field in enumerate(FIELDS):
                self.table.setItem(row, column, QTableWidgetItem(entry[field]))
        self._populating = False
        self.table.resizeColumnsToContents()
        self._draw_diagram()

    def _table_changed(self, item):
        if self._populating or item.row() >= len(self.entries):
            return
        self.entries[item.row()][FIELDS[item.column()]] = item.text().strip()
        self._save()
        self._draw_diagram()

    def add_entry(self):
        entry = {
            "kind": self.kind.currentText().strip() or "Other",
            "device": self.device.text().strip(),
            "connector": self.connector.text().strip(),
            "pins": self.pins.text().strip(),
            "notes": self.notes.text().strip(),
        }
        if not entry["device"] or not entry["pins"]:
            self.device.setFocus() if not entry["device"] else self.pins.setFocus()
            return
        self.entries.append(entry)
        self._save()
        self._refresh()
        self.table.selectRow(len(self.entries) - 1)
        for edit in (self.device, self.connector, self.pins, self.notes):
            edit.clear()

    def remove_selected(self):
        row = self.table.currentRow()
        if row < 0 or row >= len(self.entries):
            return
        del self.entries[row]
        self._save()
        self._refresh()

    def _draw_diagram(self):
        self.scene.clear()
        row_height = 72
        top = 80
        height = max(280, top + len(self.entries) * row_height + 30)
        self.scene.setSceneRect(0, 0, 1100, height)
        self.scene.setBackgroundBrush(QBrush(QColor("#202327")))

        cpu_brush = QBrush(QColor("#35465c"))
        outline = QPen(QColor("#8093a7"), 2)
        self.scene.addRect(35, 24, 360, 48, outline, cpu_brush)
        self._label("TEENSY / CPU BOARD", 52, 36, QColor("#ffffff"), bold=True)
        self._label("CPU CONNECTOR / PIN", 52, 84, QColor("#bfcbd8"), bold=True)
        self._label("CONNECTED DEVICE", 595, 84, QColor("#bfcbd8"), bold=True)

        if not self.entries:
            self._label("No wiring entries yet — add one below.", 52, 145, QColor("#bfcbd8"))
            return

        for index, entry in enumerate(self.entries):
            y = top + 30 + index * row_height
            color = COLORS.get(entry["kind"], QColor("#d6dce4"))
            connector = entry["connector"] or "Unspecified connector"
            pins = entry["pins"] or "Unspecified pins"
            device = entry["device"] or "Unnamed device"
            self.scene.addRect(35, y, 360, 54, outline, cpu_brush)
            self._label(connector, 50, y + 3, QColor("#ffffff"), bold=True)
            self._label(pins, 50, y + 27, QColor("#d5e2f0"))
            self.scene.addLine(395, y + 27, 585, y + 27, QPen(color, 3))
            self.scene.addEllipse(486, y + 22, 10, 10, QPen(color), QBrush(color))
            self.scene.addRect(585, y, 480, 54, QPen(color, 2), QBrush(QColor("#2d3238")))
            self._label(f"{device}  ·  {entry['kind']}", 600, y + 3, color, bold=True)
            self._label(entry["notes"], 600, y + 27, QColor("#d6dce4"))

    def _label(self, text, x, y, color, bold=False):
        item = self.scene.addText(text)
        item.setDefaultTextColor(color)
        font = item.font()
        font.setBold(bold)
        item.setFont(font)
        item.setPos(x, y)
