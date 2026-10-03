# Group 7 RoboCup Robot

An autonomous mobile robot project built around a Teensy 4.0. The repository
contains the robot firmware, a desktop operator and diagnostics GUI, and tools
for reviewing recorded telemetry.

## Project areas

- [`PlatformIO/`](PlatformIO/): Teensy firmware, device drivers, navigation,
  sensor integration, and PlatformIO configuration.
- [`tools/BluetoothDebugGUI/`](tools/BluetoothDebugGUI/): desktop control,
  configuration, arena planning, wiring reference, and live telemetry GUI.
- [`tools/recording-analysis/`](tools/recording-analysis/): offline
  recording visualiser and command-line analysis tools.
- [`docs/`](docs/): setup, navigation, and toolkit documentation.

The DFRobot Matrix Lidar source under `PlatformIO/lib/` is a third-party
dependency and retains its upstream licence and notices.

## Firmware

Open `PlatformIO/` in VS Code with PlatformIO installed, select the
`teensy40` environment, and build or upload. PlatformIO installs the declared
library dependencies. Board wiring and configuration notes are in
[`PlatformIO/`](PlatformIO/).

## Desktop tools

On Windows, create the GUI environment and launch it from PowerShell:

```powershell
cd tools\BluetoothDebugGUI
.\setup.ps1
.\run_gui.ps1
```

The desktop tools use Python and PyQt6. Install their dependencies with the
included `requirements.txt` files. `DataVisualiser.py` opens `.rdbg` recording
databases, and `DataAnalysisCLI.py` provides headless summaries and analysis.

## Documentation

- [Bluetooth bench setup](docs/BLUETOOTH_SETUP.md)
- [Navigation contract](docs/NAVIGATION_CONTRACT.md)
- [Robot debug toolkit guide](docs/ROBOT_DEBUG_TOOLKIT_GUIDE.md)
- [Recording and analysis workflow](tools/recording-analysis/ROBOT_DATA_WORKFLOW.md)

This repository is a student robotics project. Hardware-dependent behavior
must be validated on the robot before use.
