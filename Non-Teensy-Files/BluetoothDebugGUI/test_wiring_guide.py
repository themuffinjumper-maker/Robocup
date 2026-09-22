"""Headless checks for the editable CPU wiring guide."""

import os
import json
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import QSettings
from PyQt6.QtWidgets import QApplication

from DebugGUI import RobotDebugGUI
from WiringGuide import DEFAULT_ENTRIES, WiringGuide


class WiringGuideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_app_contains_wiring_tab(self):
        window = RobotDebugGUI()
        labels = [window.tabs.tabText(i) for i in range(window.tabs.count())]
        self.assertIn("Wiring Guide", labels)
        self.assertGreater(len(window.wiring_guide.entries), 0)
        window.close()

    def test_changes_persist_across_new_widget(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings_file = str(Path(temporary) / "wiring.ini")
            settings = QSettings(settings_file, QSettings.Format.IniFormat)
            first = WiringGuide(settings)
            first.kind.setCurrentText("Sensor")
            first.device.setText("Test encoder")
            first.connector.setText("Digital Raw 2")
            first.pins.setText("D40")
            first.notes.setText("Test note")
            first.add_entry()
            self.assertEqual(first.entries[-1]["device"], "Test encoder")

            first.table.item(0, 3).setText("D99")
            first.table.selectRow(len(first.entries) - 1)
            first.remove_selected()
            first.close()

            second = WiringGuide(QSettings(settings_file, QSettings.Format.IniFormat))
            self.assertEqual(second.entries[0]["pins"], "D99")
            self.assertEqual(len(second.entries), len(DEFAULT_ENTRIES))
            self.assertNotIn("Test encoder", [entry["device"] for entry in second.entries])
            second.close()

    def test_existing_guide_gets_new_hardware_without_losing_edits(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings_file = str(Path(temporary) / "existing.ini")
            settings = QSettings(settings_file, QSettings.Format.IniFormat)
            settings.setValue(WiringGuide.SETTINGS_KEY, json.dumps([
                {"kind": "Other", "device": "Custom device", "connector": "Custom port",
                 "pins": "D99", "notes": "Keep this"}
            ]))
            guide = WiringGuide(settings)
            self.assertEqual(guide.entries[0]["device"], "Custom device")
            self.assertEqual(guide.entries[1]["device"], "Dual encoder board")
            self.assertEqual(guide.entries[2]["connector"], "XSHUT4")
            self.assertEqual(guide.entries[3]["pins"], "D21")
            self.assertEqual(guide.entries[4]["pins"], "TRIG D14 / ECHO D24")
            self.assertEqual(guide.entries[5]["pins"], "TRIG D22 / ECHO D20")
            guide.close()

    def test_inductive_sensor_firmware_definition_is_d21_active_low(self):
        config_path = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        inputs = json.loads(config_path.read_text(encoding="utf-8"))["digital_inputs"]
        inductive = next(item for item in inputs if item["name"] == "inductive_proximity")
        self.assertEqual(inductive["pin"], 21)
        self.assertTrue(inductive["active_low"])
        self.assertTrue(inductive["pullup"])
        self.assertEqual(inductive["debounce_ms"], 20)

    def test_ultrasound_firmware_definition_reverses_each_original_pair(self):
        config_path = Path(__file__).resolve().parents[2] / "PlatformIO" / "debug_config.json"
        sensors = json.loads(config_path.read_text(encoding="utf-8"))["ultrasound_sensors"]
        first = next(item for item in sensors if item["name"] == "a")
        self.assertEqual(first["trigger_pin"], 14)
        self.assertEqual(first["echo_pin"], 24)
        self.assertEqual(first["timeout_us"], 30000)
        second = next(item for item in sensors if item["name"] == "b")
        self.assertEqual(second["trigger_pin"], 22)
        self.assertEqual(second["echo_pin"], 20)
        self.assertEqual(second["timeout_us"], 30000)

    def test_ultrasound_workflow_supports_two_sequential_channels(self):
        root = Path(__file__).resolve().parents[2] / "PlatformIO" / "lib" / "BluetoothDebugWorkflow"
        header = (root / "BluetoothDebugWorkflow.h").read_text(encoding="utf-8")
        source = (root / "BluetoothDebugWorkflow.cpp").read_text(encoding="utf-8")
        self.assertIn("MAX_ULTRASOUND_SENSORS = 2", header)
        self.assertIn("activeUltrasoundIndex_", source)
        self.assertIn("Only one transducer may transmit/listen at a time", source)

    def test_saved_ultrasound_rows_receive_direction_swap(self):
        with tempfile.TemporaryDirectory() as temporary:
            settings_file = str(Path(temporary) / "old_ultrasound.ini")
            settings = QSettings(settings_file, QSettings.Format.IniFormat)
            settings.setValue(WiringGuide.SETTINGS_KEY, json.dumps([
                {"kind": "Sensor", "device": "Ultrasound A", "connector": "Digital",
                 "pins": "TRIG D24 / ECHO D14", "notes": "Old A"},
                {"kind": "Sensor", "device": "Ultrasound B", "connector": "Digital",
                 "pins": "TRIG D20 / ECHO D22", "notes": "Old B"},
            ]))
            # Isolate this test to the echo-swap migration.
            settings.setValue("wiring_guide/encoder_raw2_migrated", True)
            settings.setValue("wiring_guide/xshut4_migrated", True)
            settings.setValue("wiring_guide/inductive_d21_migrated", True)
            settings.setValue("wiring_guide/ultrasound_d24_d14_migrated", True)
            settings.setValue("wiring_guide/ultrasound_d20_d22_migrated", True)
            guide = WiringGuide(settings)
            self.assertEqual(guide.entries[0]["pins"], "TRIG D14 / ECHO D24")
            self.assertEqual(guide.entries[1]["pins"], "TRIG D22 / ECHO D20")
            guide.close()


if __name__ == "__main__":
    unittest.main()
