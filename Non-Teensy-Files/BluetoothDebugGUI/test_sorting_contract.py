"""Regression checks for the app-to-firmware arm-sorting command contract."""

import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "PlatformIO" / "debug_config.json"
FIRMWARE = ROOT / "PlatformIO" / "lib" / "BluetoothDebugWorkflow" / "BluetoothDebugWorkflow.cpp"


class SortingContractTests(unittest.TestCase):
    def test_arm_buttons_have_a_firmware_handler(self):
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        arm = next(command for command in config["commands"]
                   if command["name"] == "arm_sorting")
        self.assertEqual(arm["action"], "arm_sorting_toggle")
        self.assertEqual([button["arguments"]["enabled"]
                          for button in arm["buttons"]], [True, False])
        code = FIRMWARE.read_text(encoding="utf-8")
        self.assertIn('strcmp(action, "arm_sorting_toggle")', code)
        self.assertIn("updateArmSorting(now);", code)

    def test_sorting_uses_detected_input_and_reports_state(self):
        code = FIRMWARE.read_text(encoding="utf-8")
        self.assertIn("digitalInputs_[sortingInputIndex_].detected()", code)
        for name in ("sorting.armed", "sorting.confirmed", "sorting.bumpers_on",
                     "sorting.gate_angle_deg", "sorting.idle_pulse"):
            self.assertIn(f'data["{name}"]', code)


if __name__ == "__main__":
    unittest.main()
