# Robot Debug Toolkit — Complete Reference Guide

**Project:** Group 7 RoboCup Robot  
**Purpose:** Live robot debugging, Bluetooth/serial communication, telemetry inspection, parameter tuning, command execution, recording, offline analysis, run comparison, fault investigation, and AI-assisted analysis  
**Primary Python tool folder:** `tools/`  
**Recommended location for this document:** the root of the Python-tools folder, alongside the tool subfolders  
**Recording format:** `.rdbg` SQLite database  
**Transport format:** newline-delimited JSON over serial/Bluetooth  
**Default serial settings:** `115200 baud, 8N1`

---

# 1. What This Toolkit Is

This toolkit is the complete PC-side debugging and data-analysis system for the robot.

It is designed around a simple principle:

> The robot should expose useful information and controls over a structured serial protocol, while the PC tools handle visualisation, recording, analysis, comparison, and operator interaction.

The system is intentionally split into several independent programs rather than trying to make one enormous application do everything.

The main components are:

| File | Role |
|---|---|
| `BluetoothSerial.py` | Serial/Bluetooth communications backend. Connects to the robot and translates newline-delimited JSON into Qt signals and outgoing messages. |
| `DebugGUI.py` | Live debugging application used while the robot is connected. Displays telemetry, sends commands, changes parameters, records test runs, logs faults, and shows communication health. |
| `DataRecorder.py` | Recording engine used by `DebugGUI.py`. Writes telemetry, commands, parameters, logs, states, raw serial, faults, metadata, and parameter snapshots into `.rdbg` SQLite files. |
| `DataVisualiser.py` | Offline graphical analysis tool for saved `.rdbg` recordings. Supports plotting, cursor inspection, measurement regions, faults/events, derived signals, anomalies, and comparing runs. |
| `DataAnalysisCLI.py` | Headless command-line analyser. Provides statistics, fault context, correlation, PID metrics, anomaly detection, JSON export, and Markdown reports without needing a GUI. |

Together, these programs create the following workflow:

```text
ROBOT / TEENSY
      │
      │ newline-delimited JSON
      │ over UART → CH9143 Bluetooth pair
      ▼
BluetoothSerial.py
      │
      ├──────────────► DebugGUI.py
      │                    │
      │                    ├─ Live telemetry
      │                    ├─ Commands
      │                    ├─ Parameter tuning
      │                    ├─ Logs / state / raw serial
      │                    ├─ Communication health
      │                    ├─ LOG FAULT
      │                    └─ Start / stop recording
      │
      ▼
DataRecorder.py
      │
      ▼
*.rdbg SQLite recording
      │
      ├──────────────► DataVisualiser.py
      │                    ├─ Plotting
      │                    ├─ Fault/event inspection
      │                    ├─ Cursor
      │                    ├─ Measurement regions
      │                    ├─ Derived signals
      │                    ├─ Run comparison
      │                    └─ Anomaly markers
      │
      └──────────────► DataAnalysisCLI.py
                           ├─ Statistics
                           ├─ Fault context
                           ├─ Correlation
                           ├─ PID analysis
                           ├─ Anomaly detection
                           ├─ JSON export
                           └─ AI-friendly Markdown report
```

The GUI and analysis software do **not** need to know every telemetry signal, parameter, or robot command in advance. The system is designed to support dynamic robot definitions, allowing the Teensy firmware to advertise parameters and commands at runtime.

---

# 2. Recommended Folder Layout

The current project structure is approximately:

```text
group7robocup/
│
├── .venv/
├── .vscode/
├── .gitignore
├── README.md
├── requirements.txt
├── setup_env.py
│
├── tools/
│   ├── BluetoothDebugGUI/
│   │   ├── BluetoothSerial.py
│   │   ├── DebugGUI.py
│   │   └── DataRecorder.py
│   │
│   └── recording-analysis/
│       ├── DataVisualiser.py
│       ├── DataAnalysisCLI.py
│       │
│       └── recordings/
│           ├── RobotRun_2026-09-05_20-15-10.rdbg
│           ├── StraightDriveTest_01.rdbg
│           └── ...
│
├── PlatformIO/
│   ├── platformio.ini
│   ├── include/
│   ├── lib/
│   ├── src/
│   └── test/
└── docs/
    └── ROBOT_DEBUG_TOOLKIT_GUIDE.md
```

The important relative path is:

```text
tools/BluetoothDebugGUI/
        │
        └── DebugGUI.py
              │
              └── ../recording-analysis/recordings/
```

`DebugGUI.py` automatically determines the data directory using its own file location, so recordings are stored under:

```text
tools/recording-analysis/recordings/
```

This means the parent folder being named `tools` is fine; the software does not rely on a hard-coded absolute Windows path.

---

# 3. Python Environment and Dependencies

The main GUI tools require:

```text
PyQt6
pyserial
pyqtgraph
numpy
```

The command-line analyser is deliberately much lighter:

```text
DataAnalysisCLI.py → Python standard library only
```

`DataRecorder.py` also uses only standard-library modules.

Typical installation inside the project's virtual environment:

```powershell
.\.venv\Scripts\Activate.ps1
pip install PyQt6 pyserial pyqtgraph numpy
```

A typical `requirements.txt` should therefore contain at least:

```text
PyQt6
pyserial
pyqtgraph
numpy
```

The intended workflow is to run the programs from the project virtual environment.

---

# 4. Communication Architecture

## 4.1 Physical transport

The current hardware communication design uses a matched **CH9143 Bluetooth serial pair**.

Conceptually:

```text
Teensy UART
    │
    ▼
CH9143 module
    )))))) Bluetooth link ((((((
CH9143 USB module
    │
    ▼
Windows COM port
```

The PC therefore sees the Bluetooth link as a normal serial COM port.

The Python software does not care that the link is Bluetooth. From its perspective it is simply a serial connection.

The same protocol can later be transported over:

- direct USB serial;
- another Bluetooth serial adapter;
- a wired UART-to-USB adapter;
- another transparent serial radio.

The communication protocol is deliberately independent of the physical transport.

---

# 5. Robot Communication Protocol

The protocol is based on **newline-delimited JSON**.

Every packet is a valid JSON object followed by:

```text
\n
```

For example:

```json
{"type":"telemetry","name":"battery.voltage","value":12.4}
```

followed by a newline.

This framing makes the protocol easy to:

- generate on the Teensy;
- inspect manually;
- log as raw text;
- parse in Python;
- extend later.

Every message should include a:

```json
"type"
```

field.

---

# 6. Protocol Message Types

## 6.1 `hello`

Used when the PC connects.

Example:

```json
{
  "type": "hello",
  "client": "RobotDebugGUI",
  "protocol": 1
}
```

Purpose:

- identifies the PC client;
- allows future protocol-version negotiation;
- gives the robot an opportunity to initialise a debug session.

---

## 6.2 `request_definitions`

Sent by the PC when it wants the robot to advertise its available commands and parameters.

Example:

```json
{
  "type": "request_definitions"
}
```

The robot should respond with `parameter_definition` and `command_definition` packets.

---

## 6.3 `telemetry`

Used for live measurements and state variables.

Single-value example:

```json
{
  "type": "telemetry",
  "name": "battery.voltage",
  "value": 12.4
}
```

A robot timestamp may also be included.

Grouped telemetry is also conceptually supported by the protocol design:

```json
{
  "type": "telemetry",
  "time": 12345,
  "data": {
    "drive.left_rpm": 420,
    "drive.right_rpm": 418
  }
}
```

Recommended telemetry names use dotted namespaces:

```text
drive.left_rpm
drive.right_rpm
drive.target_rpm
drive.left_output
drive.right_output

imu.heading
imu.pitch
imu.roll
imu.gyro_z

battery.voltage
battery.current

sorter.position
sorter.state

robot.loop_time_us
```

Dotted names make large signal lists much easier to search and understand.

---

## 6.4 `parameter_definition`

Advertises a tunable parameter.

The definition can describe:

- name;
- data type;
- default/current value;
- minimum;
- maximum;
- step size;
- enum options;
- description.

The PC GUI dynamically creates the appropriate editor.

Typical parameter examples:

```text
drive.pid.kp
drive.pid.ki
drive.pid.kd
drive.max_speed
reel.intake_speed
sorting.threshold
```

---

## 6.5 `parameter`

Sent from PC to robot to change a parameter.

Example:

```json
{
  "type": "parameter",
  "name": "drive.pid.kp",
  "value": 1.5
}
```

---

## 6.6 `parameter_request`

Requests a specific parameter value.

Example conceptually:

```json
{
  "type": "parameter_request",
  "name": "drive.pid.kp"
}
```

---

## 6.7 `parameter_value`

Robot response containing a current parameter value.

Example:

```json
{
  "type": "parameter_value",
  "name": "drive.pid.kp",
  "value": 1.2
}
```

---

## 6.8 `command_definition`

Advertises a command the robot supports.

The robot can describe:

- command name;
- human-readable description;
- arguments;
- argument types;
- limits/defaults.

The GUI dynamically creates command controls from these definitions.

Typical commands might include:

```text
drive_test
reel_test
sorter_test
home_servos
stop
set_debug_mode
reset_fault
```

---

## 6.9 `command`

Executes a command.

Example:

```json
{
  "type": "command",
  "command": "drive_test",
  "speed": 0.3,
  "duration_ms": 2000
}
```

Debug-mode example:

```json
{
  "type": "command",
  "command": "set_debug_mode",
  "enabled": true
}
```

The GUI's dedicated buttons ultimately use the same command mechanism.

---

## 6.10 `log`

Robot-generated textual log message.

Example:

```json
{
  "type": "log",
  "level": "INFO",
  "message": "Robot ready"
}
```

Recommended levels include:

```text
DEBUG
INFO
WARNING
ERROR
FAULT
```

---

## 6.11 `state`

Used for structured robot state.

Example:

```json
{
  "type": "state",
  "debug_mode": true,
  "fault": false
}
```

Unlike telemetry, which is best for continuously varying values, `state` is useful for discrete machine state.

---

## 6.12 `error`

Used when the robot or communications backend reports an error.

Errors are displayed in the GUI and counted by the communication-health display.

---

## 6.13 `heartbeat`

Reserved for connection-health / keepalive behaviour.

A future firmware implementation can periodically emit a heartbeat so the PC can explicitly distinguish:

- robot alive;
- transport connected but robot not responding;
- telemetry intentionally idle.

---

# 7. DebugGUI.py — Live Robot Debugging

`DebugGUI.py` is the main application used while the robot is physically connected.

Its purpose is to provide a single engineering dashboard for:

- connecting to the robot;
- seeing live telemetry;
- plotting live signals;
- tuning parameters;
- running defined commands;
- entering/exiting debug mode;
- stopping the robot;
- reading logs;
- inspecting raw protocol data;
- recording complete test sessions;
- manually marking faults;
- checking communication health.

---

# 8. DebugGUI Connection Controls

The top section of the GUI contains the connection controls.

## 8.1 COM port dropdown

The GUI automatically asks the serial backend for available serial ports.

It provides:

- automatic COM-port discovery;
- a dropdown rather than requiring manual typing;
- a Refresh button so newly connected devices can be detected.

This is particularly useful with the CH9143 receiver because Windows may assign different COM numbers on different machines or USB ports.

---

## 8.2 Baud-rate selection

Default:

```text
115200
```

This should match the Teensy UART and CH9143 configuration.

---

## 8.3 Connect / Disconnect

The connection button opens or closes the serial connection through `BluetoothSerial.py`.

On connection, the backend can:

- send the `hello` packet;
- request parameter/command definitions;
- begin receiving telemetry.

---

## 8.4 Persistent settings

The GUI uses Qt settings so commonly used connection information such as port and baud rate can be remembered between launches.

This avoids repeatedly reconfiguring the GUI.

---

# 9. Debug Mode Controls

The GUI provides dedicated:

```text
Enter Debug Mode
Exit Debug Mode
```

controls.

These send the normal robot command:

```json
{
  "type": "command",
  "command": "set_debug_mode",
  "enabled": true
}
```

or:

```json
{
  "type": "command",
  "command": "set_debug_mode",
  "enabled": false
}
```

Debug mode is intended to let the firmware distinguish ordinary robot operation from deliberate manual testing.

The Teensy firmware should use debug mode to protect dangerous operations.

For example, a motor-test command should ideally be rejected unless the robot is in debug mode.

---

# 10. STOP Robot

The GUI has a dedicated emergency-style software stop command.

It routes through:

```text
execute_command("stop", {})
```

This means the action is also recorded in the `.rdbg` command history when recording is active.

Important:

> This is a software command, not a substitute for a physical emergency stop or safe electrical shutdown system.

---

# 11. Dashboard Tab

The Dashboard is designed to put the most useful live information in one place.

It has a horizontal split arrangement:

```text
Live telemetry table     |     Command controls
```

## 11.1 Live telemetry table

Columns include:

```text
Signal
Value
Last Update
```

The table updates as telemetry is received.

Values are formatted automatically.

This is useful for signals that are easier to inspect numerically than graphically, such as:

```text
battery.voltage = 12.37
robot.state = READY
sorter.position = 43
drive.left_rpm = 427
```

---

# 12. Live Plot Tab in DebugGUI

Numeric telemetry can be plotted live.

The GUI stores recent numeric data in memory using bounded deques.

The design allows approximately:

```text
10,000 samples per signal
```

to be retained for plotting without allowing memory usage to grow indefinitely.

Users can:

- select signals;
- add them to the live plot;
- remove individual plotted signals;
- clear the plot.

The live plot is mainly intended for immediate debugging while the robot is running.

For deeper analysis, the saved `.rdbg` file should be opened in `DataVisualiser.py`.

---

# 13. Dynamic Parameter Tab

The Parameter tab is generated from the robot's `parameter_definition` packets.

The GUI does not need hard-coded knowledge of every tuning parameter.

The `ValueEditor` class creates controls based on parameter type.

Supported editor types include:

| Parameter type | GUI editor |
|---|---|
| Boolean | checkbox |
| Integer | spin box |
| Float | double spin box |
| Enum | combo box |
| String | text field |

Definitions can include limits and defaults.

This allows firmware parameters to be exposed cleanly without modifying the Python GUI whenever a new one is added.

---

# 14. Parameter Changes and Recording

When the user changes a parameter, the GUI:

1. reads the value from the dynamically created editor;
2. updates the locally tracked parameter state;
3. records the change if a recording is active;
4. sends the parameter to the robot.

This means a recording contains a complete history of tuning changes during the run.

Example:

```text
12.43 s  drive.pid.kp  1.20 → 1.35
37.91 s  drive.pid.ki  0.08 → 0.10
```

This is extremely important when analysing behaviour later.

---

# 15. Parameter Snapshot at Recording Start

In addition to recording parameter changes, the toolkit now captures a **complete parameter snapshot at the beginning of each recording**.

Why this matters:

Suppose a run uses:

```text
Kp = 1.4
Ki = 0.08
Kd = 0.02
```

but none of those values are changed during the run.

Without a snapshot, the recording would not know what values produced the behaviour.

The initial snapshot solves that problem.

The recording therefore contains:

- starting configuration;
- all subsequent parameter changes.

This makes tests much more reproducible.

---

# 16. Dynamic Command Controls

Commands are also generated dynamically.

When the robot sends a `command_definition`, the GUI creates a `CommandWidget`.

This can include editors for command arguments.

For example, a robot definition could describe:

```text
Command: drive_test

Arguments:
speed       float   -1.0 to +1.0
duration_ms integer 0 to 10000
```

The GUI can then automatically build the command interface.

The same commands appear in:

- Dashboard;
- Commands tab.

This provides both quick access and a dedicated command page.

---

# 17. Logs Tab

The Logs tab collects structured robot messages.

Typical messages:

```text
INFO     Robot ready
INFO     Entering debug mode
WARNING  Battery voltage low
ERROR    Sorter servo timeout
FAULT    Left drive encoder lost
```

Logs are also saved to recordings.

This makes it possible to correlate a textual firmware event with telemetry later.

---

# 18. Raw Serial Tab

The Raw Serial tab shows the incoming protocol stream.

This is useful when debugging:

- malformed JSON;
- unexpected robot responses;
- protocol mismatches;
- new firmware packets;
- connection problems.

Raw serial is recorded independently of whether the Raw Serial tab display is paused.

This is important because pausing visual display should not cause evidence to be lost from the saved test.

---

# 19. Communication / Link Health

The live GUI includes a link-health status display.

It shows approximately:

```text
Frames/s
Signals/s
Last telemetry age
Protocol errors
```

## 19.1 Frames per second

Counts recently received raw serial protocol lines.

Useful for detecting:

- complete communication loss;
- unusually low packet rate;
- robot firmware no longer transmitting.

## 19.2 Signals per second

Counts individual telemetry events.

This can be higher than the frame rate if a protocol frame contains multiple values.

## 19.3 Last telemetry age

Shows how long it has been since telemetry was last received.

For example:

```text
Last telemetry: 0.03 s
```

is healthy for a fast stream, whereas:

```text
Last telemetry: 8.47 s
```

would strongly suggest a communication or firmware problem.

## 19.4 Protocol error count

Counts errors reported to the GUI.

This can help distinguish:

```text
robot behaved strangely
```

from:

```text
the data link itself was unreliable
```

---

# 20. Recording Controls

Recording is integrated directly into the live GUI.

The controls include:

```text
Start Recording
LOG FAULT
recording status
recording duration
Test name
Notes
```

Recording can only be started when the robot is connected.

---

# 21. Recording File Naming

When recording begins, the GUI opens a Save As dialog.

The suggested name follows a timestamp format similar to:

```text
RobotRun_2026-09-05_23-42-17.rdbg
```

The user can replace this with a meaningful test name.

Examples:

```text
StraightDrive_Kp1p4.rdbg
SorterJam_Test03.rdbg
ReelIntake_HighSpeed.rdbg
BluetoothDropout_Test.rdbg
```

The `.rdbg` extension should be retained.

---

# 22. Test Name and Notes

The recording toolbar now supports test metadata.

Examples:

```text
Test:
Straight drive PID test

Notes:
New left track fitted; battery at 12.5 V
```

These values are stored in the recording metadata.

Meaningful notes are strongly recommended for serious testing.

A good test description should state what is intentionally being changed.

Example:

```text
Test:
Drive Kp sweep — run 4

Notes:
Kp 1.40, Ki 0.08, Kd 0.02. Same floor and battery as run 3.
```

---

# 23. Git Metadata

When recording starts, `DebugGUI.py` attempts to capture the project's Git state.

It records:

```text
git_branch
git_commit
git_dirty
```

Example:

```text
git_branch = testing
git_commit = 5c92d8a
git_dirty = false
```

`git_dirty=true` means there were uncommitted working-tree changes.

This is extremely valuable because a recording can later be associated with the exact firmware/software revision used during testing.

If Git information cannot be determined, recording continues; these metadata fields are simply left empty.

---

# 24. LOG FAULT

One of the most useful test features is the dedicated:

```text
LOG FAULT
```

button.

The button is:

```text
disabled when not recording
enabled while recording
```

This prevents fault markers being generated when there is no recording to store them in.

When something strange happens physically, the operator should click the button immediately.

Examples:

- robot suddenly veers left;
- wheel visibly stalls;
- sorter jams;
- servo twitches;
- reel throws an object;
- sensor reading appears wrong;
- Bluetooth appears to hesitate;
- robot vibrates;
- unexpected sound occurs.

The user does **not** need to stop and type a note.

Clicking the button records:

```text
elapsed time
wall-clock time
label = MANUAL FAULT MARKER
```

For example:

```text
42.637 s — MANUAL FAULT MARKER
```

The button briefly confirms that the fault was logged.

---

# 25. Why Manual Fault Markers Matter

The operator may notice physical behaviour that cannot be inferred from telemetry alone.

For example:

```text
42.637 s — operator sees left track visibly slip
```

Later, the visualiser can draw a vertical red line at exactly that time.

Then every signal can be inspected across the same instant:

```text
drive.left_rpm
drive.right_rpm
drive.left_output
drive.right_output
imu.gyro_z
battery.voltage
encoder counts
loop time
```

This transforms a vague observation into a precise analysis point.

---

# 26. Recording Lifecycle

A typical recording sequence is:

```text
Connect robot
    ↓
Enter debug mode if required
    ↓
Enter Test name and Notes
    ↓
Start Recording
    ↓
Run robot test
    ↓
Change parameters / issue commands as necessary
    ↓
Click LOG FAULT whenever something strange occurs
    ↓
Stop Recording
```

Recording also stops automatically if appropriate during application shutdown/disconnection, preventing the database from being left open unnecessarily.

---

# 27. DataRecorder.py

`DataRecorder.py` is the recording engine.

It is deliberately separated from the GUI.

The GUI sends recording events to the recorder; the recorder handles database persistence.

This separation makes the system easier to maintain and means recording functionality could later be reused by another interface.

---

# 28. Background Recording Architecture

The latest recorder uses a background writer thread.

The GUI thread does **not** directly perform a database commit for every telemetry sample.

Instead:

```text
GUI receives telemetry
        ↓
record_telemetry(...)
        ↓
thread-safe Queue
        ↓
background database writer
        ↓
batched SQLite writes / commits
```

This prevents disk I/O from blocking:

- Qt rendering;
- serial reception;
- user interaction;
- live plots.

---

# 29. Batched Database Commits

The writer commits roughly when either:

```text
100 queued writes have accumulated
```

or:

```text
about 0.25 seconds has passed with pending data
```

This greatly reduces the overhead compared with committing every individual row.

SQLite settings include:

```text
journal_mode = WAL
synchronous = NORMAL
temp_store = MEMORY
```

These settings provide a practical balance of recording performance and robustness.

---

# 30. The `.rdbg` File Format

An `.rdbg` file is actually an SQLite database.

Advantages:

- one self-contained file per test;
- efficient storage;
- supports very large recordings;
- indexed telemetry queries;
- no need to load the whole file into memory;
- easy extraction from Python;
- AI tools can analyse it programmatically;
- recording can include many data types without inventing a custom binary format.

The current internal format version is:

```text
3
```

---

# 31. `.rdbg` Database Schema

## 31.1 `metadata`

Columns:

```text
key
value
```

Stores session information such as:

```text
session_name
created_at
closed_at
port
baudrate
format_version
application
test_name
test_notes
git_branch
git_commit
git_dirty
```

---

## 31.2 `telemetry`

Columns:

```text
id
elapsed_s
wall_time
robot_time
signal
value_num
value_text
value_type
```

### `elapsed_s`

Time since recording started according to the PC's monotonic clock.

This is the primary analysis timebase.

### `wall_time`

Human-readable local timestamp.

Useful for knowing when the test physically occurred.

### `robot_time`

Optional timestamp supplied by the Teensy.

### `signal`

Telemetry name.

Example:

```text
drive.left_rpm
```

### `value_num`

Numeric value when available.

### `value_text`

Text representation for non-numeric telemetry.

### `value_type`

Stores the original value category, such as:

```text
number
bool
null
string
```

An index exists on:

```text
(signal, elapsed_s)
```

to speed up graph loading and time-window analysis.

---

# 32. `logs` Table

Stores:

```text
elapsed_s
wall_time
level
message
```

This preserves firmware logs inside the test recording.

---

# 33. `commands` Table

Stores:

```text
elapsed_s
wall_time
command
arguments_json
```

Example:

```text
18.250 s
drive_test
{"speed":0.3,"duration_ms":2000}
```

This provides an exact timeline of operator actions.

---

# 34. `parameters` Table

Stores parameter changes:

```text
elapsed_s
wall_time
name
value_json
```

This is a change log, not merely the final state.

---

# 35. `parameter_snapshots` Table

Stores complete parameter dictionaries.

Columns:

```text
elapsed_s
wall_time
snapshot_json
```

The initial snapshot is taken at recording start.

This provides the starting configuration for the test.

---

# 36. `states` Table

Stores robot state packets:

```text
elapsed_s
wall_time
state_json
```

This can hold structured data such as:

```json
{
  "debug_mode": true,
  "fault": false
}
```

---

# 37. `raw_serial` Table

Stores:

```text
elapsed_s
wall_time
line
```

This is the original raw line received by the PC.

It is invaluable when debugging protocol behaviour.

---

# 38. `faults` Table

Stores manual fault markers.

Columns:

```text
elapsed_s
wall_time
label
```

Index:

```text
elapsed_s
```

Default label:

```text
MANUAL FAULT MARKER
```

---

# 39. `annotations` Table

The recorder also supports general annotations:

```text
elapsed_s
wall_time
label
note
```

The live GUI currently focuses on the instant `LOG FAULT` workflow, but this table allows future tools to add named annotations without changing the database architecture.

---

# 40. Value Handling in DataRecorder

Telemetry is stored according to type.

## Numeric values

Example:

```text
427.6
```

stored in:

```text
value_num
```

## Boolean

Stored both numerically and textually where appropriate:

```text
true → 1.0
false → 0.0
```

## Text values

Stored in:

```text
value_text
```

## Null

Stored as an explicit null type.

This allows one recording system to preserve mixed telemetry types.

---

# 41. DataVisualiser.py

`DataVisualiser.py` is the main offline graphical analysis tool.

It is intended to be used after a run is complete.

Unlike the live GUI, it can take its time performing richer analysis because it is not responsible for maintaining a serial connection or controlling the robot.

---

# 42. Recording Browser

At the top of the visualiser is a recording picker.

It searches:

```text
tools/recording-analysis/recordings/
```

for:

```text
*.rdbg
```

files.

Controls include:

```text
Refresh
Open
Browse...
Compare...
Align Compare to First Fault
Clear Compare
Export Selected CSV
```

---

# 43. Recording Summary

When a recording is loaded, the visualiser shows summary information such as:

```text
filename
duration
number of signals
number of telemetry rows
number of fault markers
test name
comparison file, if active
```

This lets the user quickly verify they opened the correct test.

---

# 44. Signal Search

The available numeric telemetry signals are listed.

A search field allows filtering.

For example, entering:

```text
drive
```

can reduce a large list to:

```text
drive.left_rpm
drive.right_rpm
drive.target_rpm
drive.left_output
drive.right_output
```

This is particularly useful when a robot has dozens or hundreds of telemetry channels.

---

# 45. Adding Signals to the Plot

Select one or more numeric signals and click:

```text
Add Selected
```

Each signal becomes a plot curve.

Signals can then be:

- inspected;
- highlighted;
- measured;
- compared;
- exported.

---

# 46. Removing and Clearing Plots

Controls:

```text
Remove Plot
Clear Plots
```

`Remove Plot` removes selected currently plotted signals.

`Clear Plots` removes all telemetry curves.

Event-marker controls are handled separately.

---

# 47. Safe Auto Scale

The visualiser provides:

```text
Auto Scale
```

The autoscale system was specifically designed to avoid PyQtGraph's problematic behaviour with:

```text
NaN
Inf
all-zero signals
```

The implementation:

- ignores non-finite values;
- calculates plot bounds from finite visible samples;
- does not allow NaN ranges;
- creates non-zero ranges for constant signals.

---

# 48. All-Zero Signal Behaviour

A special case exists for signals where every visible value is:

```text
0
```

PyQtGraph's default automatic range can otherwise create absurd views such as:

```text
±1e-12
```

around zero.

The toolkit instead uses a sensible default vertical view:

```text
Y = -1 to +1
```

while fitting the X axis to the recording duration.

This correction also occurs when an all-zero signal is the **first signal added**, preventing PyQtGraph's initial automatic range from shrinking the view before the user even presses Auto Scale.

---

# 49. Currently Plotted Selection Highlight

The `Currently plotted` list doubles as a signal-inspection tool.

When a signal is selected there:

- its curve is forced visible;
- it becomes bright blue;
- it becomes thicker;
- it is moved visually above other traces.

This works even if the curve was previously hidden.

When the user clicks outside the `Currently plotted` box:

- the temporary highlight is removed;
- the original curve colour is restored;
- its previous visibility is restored;
- if it was hidden before selection, it becomes hidden again.

This gives a very fast way to identify one trace in a crowded graph.

---

# 50. Click-to-Inspect Cursor

Clicking the graph creates a vertical inspection cursor.

The visualiser then finds the nearest sample from every plotted signal to that X-coordinate.

A cursor table shows:

```text
Signal                     Value
drive.left_rpm             421.7
drive.right_rpm            418.9
imu.gyro_z                 -3.42
battery.voltage            12.17
```

If a comparison recording is loaded, comparison values are also shown.

This is ideal for answering:

> What was every relevant system variable doing at this exact instant?

---

# 51. Measurement Region

Click:

```text
Measure Region
```

to enable a draggable region across the plot.

For each plotted signal inside that time window, the visualiser calculates:

```text
sample count
minimum
maximum
mean
standard deviation
delta from first to last sample
```

It also displays:

```text
start time
end time
region duration
```

Example:

```text
31.200–34.800 s (Δt=3.600s)

drive.left_rpm:
n=361
min=410.2
max=429.8
mean=421.1
std=3.12
Δ=1.8
```

This is especially useful for:

- steady-state analysis;
- vibration windows;
- comparing before/after behaviour;
- quantifying oscillations;
- looking at a fault window.

---

# 52. X Axis Modes

The visualiser supports:

```text
Elapsed time
Robot time
```

## Elapsed time

PC monotonic time since recording began.

This is the recommended default.

## Robot time

Uses the robot-supplied timestamp where available.

The visualiser normalises robot time relative to its first sample.

If the timestamp appears to be in milliseconds rather than seconds, the visualiser attempts to normalise it accordingly.

Some event-marker functions are intentionally simplest in elapsed-time mode because manually logged faults use the PC recording clock.

---

# 53. Fault Markers on the Plot

Fault markers are displayed as:

```text
bright red vertical lines
```

with labels.

They cross the entire graph so every signal can be examined relative to the same event.

A manual fault therefore becomes visually obvious even when many curves are plotted.

---

# 54. Other Event Markers

The visualiser can also display:

## Commands

Shown with their own vertical marker style.

Example:

```text
CMD drive_test
```

## Parameter changes

Example:

```text
PARAM drive.pid.kp
```

## State messages

Example:

```text
STATE
```

Checkboxes allow these marker groups to be turned on or off.

This helps prevent an event-heavy recording from becoming visually cluttered.

---

# 55. Events Tab

The Events tab combines multiple event sources into one chronological table.

Supported event categories include:

```text
Faults
Logs
Commands
Parameter changes
Robot states
Annotations
```

Columns include:

```text
Elapsed time
Type
Name / Level
Details
```

Events are sorted by time.

---

# 56. Double-Click Event Navigation

Double-clicking an event jumps to it.

The visualiser:

1. switches to the Plot tab;
2. centres the X axis around the event;
3. uses approximately a ±3 second view;
4. places the inspection cursor at the event time;
5. updates cursor values.

This makes the workflow:

```text
Find FAULT in Events
        ↓
double-click
        ↓
immediately inspect telemetry around it
```

---

# 57. Derived Signals

The visualiser can create virtual signals from recorded telemetry.

Click:

```text
Add Derived Signal
```

Enter a name such as:

```text
left_right_rpm_error
```

Then an expression such as:

```python
sig("drive.left_rpm") - sig("drive.right_rpm")
```

The visualiser interpolates referenced signals against the first signal's timebase.

Useful examples:

## Left/right RPM difference

```python
sig("drive.left_rpm") - sig("drive.right_rpm")
```

## Control error

```python
sig("drive.target_rpm") - sig("drive.left_rpm")
```

## Electrical power

```python
sig("battery.voltage") * sig("battery.current")
```

## Gyroscope conversion from rad/s to deg/s

```python
sig("imu.gyro_z") * 57.2958
```

Available safe expression helpers include functions such as:

```text
abs
sqrt
sin
cos
tan
clip
minimum
maximum
```

Derived signals are calculated during analysis; they do not alter the original `.rdbg` file.

---

# 58. Comparing Two Recordings

The visualiser can load a second `.rdbg` file.

Click:

```text
Compare...
```

When a signal present in the main recording is plotted, the same signal from the comparison run is overlaid where available.

Comparison traces are visually differentiated, including dashed styling.

Use cases:

```text
Kp 1.2 vs Kp 1.5
old firmware vs new firmware
new track vs old track
fresh battery vs low battery
before repair vs after repair
```

---

# 59. Align Compare to First Fault

Click:

```text
Align Compare to First Fault
```

The visualiser finds the first manual fault in both recordings.

It time-shifts the comparison recording so the two first-fault timestamps coincide.

Conceptually:

```text
Run A fault = 42.6 s
Run B fault = 31.2 s
```

After alignment:

```text
fault A and fault B appear at the same X position
```

This is extremely useful for intermittent fault investigation because the relevant comparison becomes:

```text
5 seconds before fault
fault
5 seconds after fault
```

rather than comparing unrelated absolute test times.

Both recordings must contain at least one fault marker for this feature.

---

# 60. Automatic Anomaly Detection

Click:

```text
Find Anomalies
```

The visualiser performs robust outlier detection on currently plotted signals.

Candidate anomalies are marked with:

```text
magenta vertical lines
```

The method examines:

- unusually extreme signal values;
- unusually large sudden sample-to-sample changes.

The algorithm uses robust statistics based on median / median absolute deviation rather than ordinary mean and standard deviation.

This makes it less sensitive to a few existing extreme values.

The anomaly detector is intended to find **candidate regions worth inspecting**, not to declare with certainty that a robot fault occurred.

Always interpret anomalies in engineering context.

---

# 61. Clearing Anomalies

Click:

```text
Clear Anomalies
```

to remove the magenta candidate markers.

This does not modify the recording.

---

# 62. Raw Serial Offline Inspection

The visualiser includes a Raw Serial tab.

This allows an engineer to inspect exactly what was received during the historical test.

This can be useful if:

- telemetry appears incomplete;
- a firmware packet was malformed;
- a new packet type was being tested;
- communication failure is suspected.

---

# 63. Session Info Tab

The Session Info tab displays metadata.

It includes information such as:

```text
session name
creation time
serial port
baud rate
format version
test name
notes
Git branch
Git commit
Git dirty state
```

It also displays the initial parameter snapshot if present.

This makes the `.rdbg` file largely self-documenting.

---

# 64. CSV Export

The visualiser can export plotted/selected telemetry to CSV.

The CSV columns are:

```text
elapsed_s
signal
value
```

If specific signals are selected in `Currently plotted`, those are exported.

Otherwise, plotted signals are used.

The original `.rdbg` file remains the preferred archive format because CSV does not preserve:

- logs;
- commands;
- states;
- faults;
- parameter snapshots;
- metadata;
- raw serial.

CSV is best used for compatibility with:

- Excel;
- MATLAB;
- external plotting tools;
- quick manual processing.

---

# 65. DataAnalysisCLI.py

`DataAnalysisCLI.py` is the non-GUI analysis interface.

It is intended for:

- automated engineering analysis;
- AI agents;
- scripts;
- command-line use;
- remote/headless machines;
- extracting compact information from very large recordings.

It uses only the Python standard library.

Typical invocation:

```powershell
python DataAnalysisCLI.py <command> <recording.rdbg> [options]
```

---

# 66. CLI: `summary`

Usage:

```powershell
python DataAnalysisCLI.py summary Data\Test.rdbg
```

Returns JSON containing information such as:

```text
metadata
duration
telemetry row count
signal count
numeric signal list
fault count
fault details
parameter snapshots
```

This is usually the best first command for an AI or human exploring an unfamiliar recording.

---

# 67. CLI: `signals`

Usage:

```powershell
python DataAnalysisCLI.py signals Data\Test.rdbg
```

List numeric only:

```powershell
python DataAnalysisCLI.py signals Data\Test.rdbg --numeric-only
```

This is useful before requesting statistics for a specific telemetry channel.

---

# 68. CLI: `faults`

Usage:

```powershell
python DataAnalysisCLI.py faults Data\Test.rdbg
```

Filter by time:

```powershell
python DataAnalysisCLI.py faults Data\Test.rdbg --start 30 --end 50
```

Returns manual fault markers.

---

# 69. CLI: `events`

Usage:

```powershell
python DataAnalysisCLI.py events Data\Test.rdbg
```

Optional time range:

```powershell
python DataAnalysisCLI.py events Data\Test.rdbg --start 40 --end 45
```

Returns a combined chronological event stream containing items such as:

```text
faults
commands
parameter changes
logs
states
annotations
```

---

# 70. CLI: `context`

One of the most useful AI-analysis commands.

Usage:

```powershell
python DataAnalysisCLI.py context Data\Test.rdbg --time 42.637 --window 3
```

This asks:

> Give me the engineering context around 42.637 seconds, approximately three seconds either side.

The result includes:

- faults in the window;
- commands;
- logs;
- parameter changes;
- states;
- nearest value of every telemetry signal to the requested timestamp;
- statistics for every numeric signal in the window.

This is ideal for investigating a manually logged fault.

---

# 71. CLI: `stats`

Usage:

```powershell
python DataAnalysisCLI.py stats Data\Test.rdbg --signal drive.left_rpm
```

Optional range:

```powershell
python DataAnalysisCLI.py stats Data\Test.rdbg --signal drive.left_rpm --start 10 --end 20
```

Statistics include:

```text
sample count
minimum
maximum
mean
median
standard deviation
5th percentile
25th percentile
75th percentile
95th percentile
first value
last value
delta
duration
estimated sample rate
```

---

# 72. CLI: `range`

Extract raw values for one signal over a time range.

Usage:

```powershell
python DataAnalysisCLI.py range Data\Test.rdbg --signal imu.gyro_z --start 40 --end 45
```

Useful when another tool or AI needs the actual series rather than summary statistics.

---

# 73. CLI: `correlate`

Usage:

```powershell
python DataAnalysisCLI.py correlate Data\Test.rdbg --a drive.left_rpm --b drive.right_rpm
```

Optional time limits:

```powershell
python DataAnalysisCLI.py correlate Data\Test.rdbg --a drive.left_rpm --b drive.right_rpm --start 10 --end 30
```

The analyser interpolates one series onto the timestamps of the other and calculates correlation.

Examples of useful comparisons:

```text
left RPM vs right RPM
steering command vs gyro Z
motor output vs wheel speed
battery current vs motor load
```

Remember:

> Correlation describes statistical co-variation. It does not prove causation.

---

# 74. CLI: `pid`

This is specifically designed for control-system analysis.

Usage:

```powershell
python DataAnalysisCLI.py pid Data\Test.rdbg ^
  --setpoint drive.target_rpm ^
  --measured drive.left_rpm
```

PowerShell can also be entered on one line:

```powershell
python DataAnalysisCLI.py pid Data\Test.rdbg --setpoint drive.target_rpm --measured drive.left_rpm
```

Optional arguments:

```text
--start
--end
--tolerance
```

Default settling tolerance:

```text
2%
```

---

# 75. PID Metrics

The PID analyser calculates:

## 10–90% rise time

Approximate time taken for the response to move from 10% to 90% of the detected setpoint change.

## Overshoot percentage

How far the measured response exceeds the final target relative to step size.

## Settling time

Time until the signal enters and remains inside the tolerance band.

## Steady-state error

Mean error near the end of the analysed response.

## MAE

Mean absolute error.

## RMSE

Root mean square error.

## IAE

Integral of absolute error:

```text
∫ |e(t)| dt
```

## ISE

Integral of squared error:

```text
∫ e(t)^2 dt
```

These allow tuning choices to be compared quantitatively rather than purely by eye.

---

# 76. CLI: `anomalies`

Usage:

```powershell
python DataAnalysisCLI.py anomalies Data\Test.rdbg --signal imu.gyro_z
```

Optional range:

```powershell
python DataAnalysisCLI.py anomalies Data\Test.rdbg --signal drive.left_rpm --start 30 --end 50
```

Returns candidate:

```text
level_outlier
sudden_change
```

events.

Each result includes:

```text
time
value
kind
robust anomaly score
```

---

# 77. CLI: `export-json`

Usage:

```powershell
python DataAnalysisCLI.py export-json Data\Test.rdbg --output analysis.json
```

Produces a machine-readable analysis package containing:

```text
summary
statistics for all numeric signals
events
```

This is ideal for passing analysis data into another program or AI workflow.

---

# 78. CLI: `report`

Usage:

```powershell
python DataAnalysisCLI.py report Data\Test.rdbg --output analysis_report.md
```

Produces a Markdown report containing:

- recording duration;
- sample count;
- signal count;
- fault count;
- test name;
- Git branch/commit;
- fault list;
- statistics for numeric signals;
- candidate anomalies.

This gives an AI or engineer a compact first-pass understanding of the run without reading the raw database manually.

---

# 79. Recommended AI Analysis Workflow

For a large recording, do **not** immediately dump every telemetry sample into an AI conversation.

A better workflow is:

```text
1. summary
2. signals
3. faults
4. context around relevant fault
5. stats on suspicious signals
6. correlate related signals
7. PID analysis where applicable
8. raw range only if deeper inspection is required
```

Example:

```powershell
python DataAnalysisCLI.py summary Data\VeerLeftTest.rdbg
```

Then:

```powershell
python DataAnalysisCLI.py faults Data\VeerLeftTest.rdbg
```

Suppose it returns:

```text
42.637 s
```

Then:

```powershell
python DataAnalysisCLI.py context Data\VeerLeftTest.rdbg --time 42.637 --window 3
```

If the context suggests the drive system is responsible:

```powershell
python DataAnalysisCLI.py correlate Data\VeerLeftTest.rdbg --a drive.left_rpm --b drive.right_rpm --start 39 --end 46
```

and perhaps:

```powershell
python DataAnalysisCLI.py pid Data\VeerLeftTest.rdbg --setpoint drive.target_rpm --measured drive.left_rpm --start 39 --end 46
```

This keeps analysis focused and efficient.

---

# 80. Example Investigation — Robot Veers Left

Imagine the robot is commanded to drive straight.

At approximately:

```text
42.637 s
```

the operator notices a sudden left veer and hits:

```text
LOG FAULT
```

After the test:

## Step 1 — Open recording

Load it in `DataVisualiser.py`.

## Step 2 — Find the fault

The red vertical line appears.

## Step 3 — Plot relevant signals

Add:

```text
drive.target_rpm
drive.left_rpm
drive.right_rpm
drive.left_output
drive.right_output
imu.gyro_z
battery.voltage
```

## Step 4 — Click the fault

The cursor displays all values at that moment.

## Step 5 — Measure around it

Enable the measurement region for:

```text
40–45 s
```

## Step 6 — Look for anomaly markers

Click:

```text
Find Anomalies
```

## Step 7 — Inspect event history

Check whether:

- a parameter changed;
- a command was sent;
- firmware emitted a warning;
- state changed.

## Step 8 — Use CLI

```powershell
python DataAnalysisCLI.py context Data\VeerLeft.rdbg --time 42.637 --window 3
```

## Possible conclusions

### Left RPM falls but motor output rises

Likely mechanical load, wheel slip, drivetrain issue, or encoder problem.

### Left motor output falls unexpectedly

Likely control logic issue.

### Both RPM signals remain equal but gyro Z spikes

Possibly wheel slip, surface interaction, mechanical geometry, or inaccurate wheel-speed measurement.

### Battery voltage collapses

Possible power-system limitation.

### Raw serial has a gap at the same time

Communication issue may be affecting interpretation.

This is exactly why the toolkit records multiple information layers simultaneously.

---

# 81. Example PID Tuning Workflow

Suppose you are tuning wheel-speed control.

## Run A

```text
Kp = 1.20
Ki = 0.06
Kd = 0.00
```

Record:

```text
DrivePID_Kp1p20.rdbg
```

## Run B

```text
Kp = 1.40
Ki = 0.06
Kd = 0.00
```

Record:

```text
DrivePID_Kp1p40.rdbg
```

The initial parameter snapshot permanently records these values.

Open Run A in the visualiser.

Click:

```text
Compare...
```

and choose Run B.

Plot:

```text
drive.target_rpm
drive.left_rpm
```

Then use CLI:

```powershell
python DataAnalysisCLI.py pid Data\DrivePID_Kp1p20.rdbg --setpoint drive.target_rpm --measured drive.left_rpm
```

and:

```powershell
python DataAnalysisCLI.py pid Data\DrivePID_Kp1p40.rdbg --setpoint drive.target_rpm --measured drive.left_rpm
```

Compare:

```text
rise time
overshoot
settling time
steady-state error
IAE
ISE
RMSE
```

The best controller is not necessarily the one with the smallest rise time. It should satisfy the overall requirements without excessive overshoot, oscillation, control effort, or instability.

---

# 82. Recommended Naming Convention

## Telemetry

Use:

```text
subsystem.signal
```

Examples:

```text
drive.left_rpm
drive.right_rpm
drive.left_output
drive.right_output
drive.target_rpm

imu.heading
imu.gyro_x
imu.gyro_y
imu.gyro_z

battery.voltage
battery.current

reel.left_speed
reel.right_speed

sorter.gate_angle
sorter.kicker_angle
```

Avoid vague names such as:

```text
speed1
val2
temp
x
```

because they become confusing during analysis.

---

# 83. Telemetry Sampling Advice

Not every signal needs to be transmitted at the same rate.

Possible guideline:

```text
fast control data      50–200 Hz
wheel RPM              20–100 Hz
IMU                     50–200 Hz
battery voltage          5–20 Hz
temperatures             1–10 Hz
state changes            event driven
logs                     event driven
```

Sending excessive telemetry can:

- consume serial bandwidth;
- increase database size;
- affect firmware timing if implemented badly.

The robot should prioritise signals that are useful for debugging.

---

# 84. Robot Firmware Safety

The PC GUI can send commands to real actuators.

The firmware must remain responsible for safety.

Recommended protections include:

- require debug mode for direct actuator tests;
- enforce command limits;
- enforce maximum duration;
- reject invalid parameter ranges;
- maintain hardware-independent emergency stop mechanisms;
- timeout manual commands;
- stop actuators if communication is lost where appropriate;
- never assume that the PC GUI is a safety controller.

Example:

Even if the PC sends:

```json
{
  "command": "drive_test",
  "speed": 100
}
```

the robot firmware should validate it and reject/clamp it rather than blindly executing it.

---

# 85. Backward Compatibility

The analysis tools check whether optional database tables exist.

This allows older recordings without newer features such as:

```text
faults
parameter_snapshots
annotations
```

to remain usable.

Newer analysis features simply return no data for tables that do not exist.

This is one of the advantages of using SQLite rather than a rigid custom binary structure.

---

# 86. Old Recordings vs New Recordings

A recording created before format version 3 may not contain:

```text
initial parameter snapshot
Git metadata
annotations
```

but telemetry and older event data can still be analysed.

For best reproducibility, use the latest `DebugGUI.py` and `DataRecorder.py` for new tests.

---

# 87. What Data Is Preserved During a Recording

A modern recording can preserve all of the following:

```text
✓ numeric telemetry
✓ text telemetry
✓ PC elapsed time
✓ wall-clock timestamps
✓ robot timestamps
✓ robot logs
✓ commands sent from PC
✓ command arguments
✓ parameter changes
✓ initial parameter snapshot
✓ robot state messages
✓ raw serial lines
✓ manual fault markers
✓ optional annotations
✓ serial port
✓ baud rate
✓ test name
✓ notes
✓ Git branch
✓ Git commit
✓ whether Git tree was dirty
```

This is much richer than a normal CSV data logger.

---

# 88. What Is Not Automatically Recorded

Unless specifically added as telemetry or metadata, the system cannot know things such as:

- what surface the robot was driving on;
- which physical battery pack was installed;
- ambient lighting;
- whether a wheel visibly slipped;
- whether the robot made an unusual sound;
- external camera observations;
- exact mechanical modifications.

Use:

```text
Test name
Notes
LOG FAULT
```

to capture human context, and expose any repeatably important physical measurements as telemetry.

---

# 89. Recording Reliability

Because recording uses a queue, there can be a short delay between:

```text
record_telemetry(...)
```

and the corresponding SQLite write.

This is normal.

When `stop()` is called, the recorder sends an internal stop marker and waits for the writer thread to flush and close the database.

Do not forcibly terminate Python during an active high-value test unless necessary.

---

# 90. Why Elapsed Time Is Important

The toolkit uses:

```text
time.monotonic()
```

for recording elapsed time.

This is preferable to wall-clock time for measurements because it is not affected by:

- daylight saving changes;
- manual clock adjustments;
- network time synchronisation.

Wall-clock timestamps are still preserved separately for human reference.

---

# 91. Why Raw Serial Is Worth Recording

Structured tables preserve the useful decoded information.

Raw serial preserves the original evidence.

If a parser bug is later discovered, the raw stream may make it possible to understand what the robot actually transmitted.

This is the same reason robust engineering systems often preserve both:

```text
decoded data
raw source data
```

---

# 92. Troubleshooting — GUI Does Not See COM Port

Try:

1. verify CH9143 receiver is connected;
2. check Windows Device Manager;
3. press `Refresh` in DebugGUI;
4. confirm no other application has the COM port open;
5. unplug/replug the receiver;
6. confirm the correct CH9143 pair is being used.

---

# 93. Troubleshooting — Connects but No Telemetry

Check:

```text
Frames/s
Signals/s
Last telemetry
Raw Serial
```

Possible cases:

## Frames/s = 0

Likely transport problem.

Check:

- UART wiring;
- baud rate;
- CH9143 pairing;
- correct COM port;
- Teensy transmission.

## Frames/s > 0 but Signals/s = 0

Likely protocol/parsing issue.

Inspect Raw Serial.

Possible causes:

- malformed JSON;
- no newline framing;
- unsupported packet structure;
- wrong `type`.

## Signals/s > 0

Transport and telemetry parsing are likely functioning.

---

# 94. Troubleshooting — PyQtGraph `Cannot set range [nan, nan]`

The current visualiser's manual autoscaling ignores non-finite values.

If this error appears again:

- confirm the latest `DataVisualiser.py` is in use;
- inspect whether a newly added custom feature is calling PyQtGraph's raw `autoRange()` again;
- avoid directly auto-ranging objects that may contain `NaN`.

The intended implementation calculates finite ranges manually.

---

# 95. Troubleshooting — All-Zero Plot Is Microscopic

The latest visualiser explicitly handles this.

Expected result for an all-zero first signal:

```text
Y range ≈ -1 to +1
```

not:

```text
±1e-12
```

If the old behaviour appears, the running file is probably an earlier version.

---

# 96. Troubleshooting — Fault Button Greyed Out

This is intentional.

`LOG FAULT` is only enabled while a recording is active.

Sequence:

```text
Connect
Start Recording
LOG FAULT becomes available
```

---

# 97. Troubleshooting — No Fault Lines in Old Recording

Older recordings may not have a `faults` table or may simply contain no manual markers.

The visualiser is designed not to crash in this case.

---

# 98. Troubleshooting — Compare Run Shows No Trace

The second recording must contain the same signal name.

For example, if the main run contains:

```text
drive.left_rpm
```

but the comparison firmware renamed it to:

```text
drive.left_speed
```

they will not automatically match.

Maintaining stable telemetry names across firmware versions is therefore highly recommended.

---

# 99. Troubleshooting — Align to First Fault Does Nothing

Both recordings must contain at least one manual fault.

If only one contains a fault, there is no common alignment event.

---

# 100. Troubleshooting — Derived Signal Error

Derived expressions should reference signals using:

```python
sig("signal.name")
```

For example:

```python
sig("drive.left_rpm") - sig("drive.right_rpm")
```

Check that:

- the exact signal names exist;
- the signals contain numeric data;
- quotes and parentheses are balanced.

---

# 101. Troubleshooting — Git Metadata Is Blank

Git capture is best effort.

Possible causes:

- project is not inside a Git repository;
- Git is not installed/on PATH;
- script location differs from expected project hierarchy.

Recording still functions normally.

---

# 102. Recommended Test Discipline

For meaningful engineering data, use a repeatable process.

Before each test:

```text
1. Confirm robot mechanical configuration.
2. Confirm battery condition.
3. Confirm firmware branch/commit.
4. Connect DebugGUI.
5. Verify telemetry is flowing.
6. Verify link health.
7. Enter meaningful Test name.
8. Add useful Notes.
9. Start recording.
10. Perform only the intended test change.
11. LOG FAULT immediately on unusual behaviour.
12. Stop recording.
13. Do not overwrite the original .rdbg.
```

When tuning, change one important variable at a time where possible.

---

# 103. Good Test Metadata Examples

Poor:

```text
Test = test
Notes = trying stuff
```

Better:

```text
Test = Left drive PID Kp 1.40
Notes = Ki 0.08, Kd 0.02. Battery 12.4 V. Concrete floor. 3 m straight drive.
```

Excellent:

```text
Test = Drive PID sweep run 04 — Kp 1.40
Notes = Same robot config as run 03. Kp increased 1.20→1.40 only.
        Ki=0.08, Kd=0.02. New left track. Start battery=12.43 V.
        Robot commanded 0→400 RPM step at approximately 5 s.
```

---

# 104. Recommended Fault-Marker Discipline

Click `LOG FAULT` whenever your eyes or ears detect something important.

Do not wait to decide exactly what it was.

The timestamp is more valuable than a perfect explanation.

Examples:

```text
robot jolted
track slipped
unexpected noise
servo twitched
ball jammed
robot hesitated
Bluetooth pause suspected
```

The detailed interpretation can happen after the run.

---

# 105. Suggested Analysis Order After a Fault

Use this order:

```text
1. Red fault line
2. Cursor values
3. ±3 s event context
4. Measurement region
5. Relevant signal plots
6. Commands / parameter events
7. Logs
8. State transitions
9. Anomaly markers
10. Raw serial if communication suspected
11. CLI context
12. Correlation / PID analysis as appropriate
```

This avoids jumping immediately to a theory without checking the surrounding evidence.

---

# 106. Choosing Signals to Record

Useful signals usually fall into several categories.

## Commands / targets

```text
drive.target_rpm
steering.target
sorter.target_angle
```

## Measured response

```text
drive.left_rpm
drive.right_rpm
imu.heading
servo.position
```

## Controller output

```text
drive.left_output
drive.right_output
pid.error
pid.integral
```

## Physical health

```text
battery.voltage
battery.current
motor.current
temperature
```

## Software health

```text
robot.loop_time_us
queue_depth
serial_tx_rate
fault_flags
```

Recording both cause and response makes later diagnosis much easier.

---

# 107. High-Value Derived Signals

The following derived signals are often useful:

## Wheel mismatch

```python
sig("drive.left_rpm") - sig("drive.right_rpm")
```

## Average wheel speed

```python
(sig("drive.left_rpm") + sig("drive.right_rpm")) / 2
```

## Left speed control error

```python
sig("drive.target_rpm") - sig("drive.left_rpm")
```

## Right speed control error

```python
sig("drive.target_rpm") - sig("drive.right_rpm")
```

## Battery electrical power

```python
sig("battery.voltage") * sig("battery.current")
```

---

# 108. Human GUI Analysis vs CLI Analysis

Use `DataVisualiser.py` when:

- shape matters;
- timing relationships are visual;
- you want to browse;
- you want cursor inspection;
- you want to compare runs visually;
- you want to drag measurement windows.

Use `DataAnalysisCLI.py` when:

- you need exact statistics;
- automation is required;
- an AI is analysing the run;
- no desktop GUI is available;
- you need repeatable scripted analysis;
- you want reports or JSON.

The strongest workflow uses both.

---

# 109. Typical Daily Workflow

```text
Start development session
        ↓
git switch testing
        ↓
change firmware
        ↓
build / flash Teensy
        ↓
launch DebugGUI
        ↓
connect CH9143 serial link
        ↓
verify telemetry
        ↓
run recorded test
        ↓
LOG FAULT if anything unusual occurs
        ↓
open DataVisualiser
        ↓
inspect / compare
        ↓
use DataAnalysisCLI for quantitative analysis
        ↓
change code/tuning
        ↓
repeat
```

When behaviour is validated:

```text
merge tested work toward main
```

---

# 110. AI-Friendly Handoff Strategy

For another AI or coding agent, provide:

1. this document;
2. the four current Python files;
3. `BluetoothSerial.py`;
4. the communication protocol document if separately maintained;
5. one representative `.rdbg` file if analysis is required.

For a specific fault, also provide CLI output from:

```powershell
python DataAnalysisCLI.py context <file> --time <fault_time> --window 3
```

This gives the AI a compact but information-rich snapshot.

---

# 111. Extending the Protocol

The current protocol is intentionally extensible.

Future packet types could include:

```text
ack
subscription
file_transfer
event
diagnostic
firmware_info
test_definition
```

Recommended future addition:

## Command acknowledgement IDs

PC sends:

```json
{
  "type": "command",
  "id": 123,
  "command": "drive_test",
  "speed": 0.3
}
```

Robot responds:

```json
{
  "type": "ack",
  "id": 123,
  "ok": true
}
```

This would let the GUI distinguish:

```text
command transmitted
```

from:

```text
robot explicitly accepted command
```

---

# 112. Potential Future Telemetry Subscriptions

At present, the robot can stream telemetry continuously.

A more advanced system could let the PC request only selected signals or rates.

Example concept:

```json
{
  "type": "subscribe",
  "signals": [
    "drive.left_rpm",
    "drive.right_rpm",
    "imu.gyro_z"
  ],
  "rate_hz": 50
}
```

This could reduce bandwidth when the robot eventually exposes many channels.

---

# 113. Potential Future Named Fault Categories

The current `LOG FAULT` design is intentionally instant.

A later extension could optionally classify markers after the fact:

```text
mechanical
control
sensor
communications
power
unknown
```

The instant timestamp should remain the primary interaction so the operator is never forced to type during a live fault.

---

# 114. Potential Future Event-Centred Run Comparison

The current comparison aligns on the first fault.

A natural extension is to align on:

```text
specific fault number
specific command
state transition
parameter change
custom annotation
```

For example:

```text
align both runs to the moment drive_test begins
```

would be excellent for comparing controller step responses.

---

# 115. Potential Future Spectral Analysis

For vibration and oscillation investigation, a future analyser could provide:

```text
FFT
dominant frequency
power spectral density
frequency peaks
```

This would be particularly useful for:

- wheel vibration;
- drivetrain resonance;
- PID oscillation;
- mechanical chatter.

---

# 116. Potential Future Lagged Correlation

Current correlation compares signals at approximately the same time.

A future cross-correlation tool could estimate time delay.

Example question:

> Does motor command lead wheel speed by 80 ms?

This can be valuable in dynamic system identification and controller analysis.

---

# 117. Potential Future Automated Test Reports

The current CLI Markdown report can be expanded into a full automated engineering report containing:

```text
test metadata
parameter configuration
fault timeline
plots
PID metrics
anomalies
signal correlations
comparison against baseline
pass/fail limits
```

Because `.rdbg` already contains the required underlying data, the architecture is prepared for this.

---

# 118. File Responsibilities — Do Not Mix Them Up

## `BluetoothSerial.py`

Owns:

```text
serial port
JSON transport
incoming packet decoding
Qt communication signals
outgoing protocol messages
```

Should **not** become responsible for data visualisation.

## `DebugGUI.py`

Owns:

```text
live operator UI
live plot
commands
parameters
recording controls
fault button
communication health
```

Should **not** contain low-level SQLite details.

## `DataRecorder.py`

Owns:

```text
recording format
database schema
writer queue
threaded persistence
```

Should **not** control GUI widgets.

## `DataVisualiser.py`

Owns:

```text
offline human graphical analysis
```

Should not require the robot to be connected.

## `DataAnalysisCLI.py`

Owns:

```text
headless quantitative analysis
AI/script interface
```

Should not depend on Qt.

Keeping these responsibilities separate makes the toolkit maintainable.

---

# 119. Software Design Principles

The toolkit follows several useful principles.

## Dynamic discovery

Commands and parameters can be advertised by firmware.

## Transport independence

The protocol does not depend on Bluetooth specifically.

## One self-contained recording

All evidence for a test is kept in one `.rdbg`.

## Preserve raw evidence

Raw serial is stored as well as parsed data.

## Human observations matter

Manual fault markers preserve observations telemetry cannot capture.

## Reproducibility

Parameter snapshots + Git metadata allow runs to be recreated.

## Separate live operation from offline analysis

The live GUI remains responsive while deeper analysis happens later.

## AI-readable data

The CLI can extract concise structured information.

---

# 120. Quick Command Reference

## Launch live GUI

From the relevant environment/folder:

```powershell
python DebugGUI.py
```

## Launch offline visualiser

```powershell
python DataVisualiser.py
```

## Recording summary

```powershell
python DataAnalysisCLI.py summary Data\Test.rdbg
```

## List numeric signals

```powershell
python DataAnalysisCLI.py signals Data\Test.rdbg --numeric-only
```

## List faults

```powershell
python DataAnalysisCLI.py faults Data\Test.rdbg
```

## Fault context

```powershell
python DataAnalysisCLI.py context Data\Test.rdbg --time 42.637 --window 3
```

## Signal statistics

```powershell
python DataAnalysisCLI.py stats Data\Test.rdbg --signal drive.left_rpm
```

## Correlation

```powershell
python DataAnalysisCLI.py correlate Data\Test.rdbg --a drive.left_rpm --b drive.right_rpm
```

## PID analysis

```powershell
python DataAnalysisCLI.py pid Data\Test.rdbg --setpoint drive.target_rpm --measured drive.left_rpm
```

## Anomaly detection

```powershell
python DataAnalysisCLI.py anomalies Data\Test.rdbg --signal imu.gyro_z
```

## JSON export

```powershell
python DataAnalysisCLI.py export-json Data\Test.rdbg --output analysis.json
```

## Markdown report

```powershell
python DataAnalysisCLI.py report Data\Test.rdbg --output analysis_report.md
```

---

# 121. Quick Live-Test Checklist

```text
[ ] Correct firmware flashed
[ ] Correct Git branch
[ ] Debug GUI launched
[ ] Correct COM port selected
[ ] Baud = 115200
[ ] Connected
[ ] Frames/s healthy
[ ] Signals/s healthy
[ ] Telemetry updating
[ ] Debug mode enabled if required
[ ] Test name entered
[ ] Notes entered
[ ] Recording started
[ ] LOG FAULT available
[ ] Test performed
[ ] Recording stopped
[ ] .rdbg file verified
```

---

# 122. Quick Post-Test Checklist

```text
[ ] Open .rdbg in DataVisualiser
[ ] Check metadata
[ ] Check initial parameter snapshot
[ ] Plot relevant command/target signals
[ ] Plot measured responses
[ ] Inspect fault lines
[ ] Double-click relevant events
[ ] Use cursor
[ ] Use measurement region
[ ] Run anomaly scan where useful
[ ] Compare against reference run if available
[ ] Run CLI statistics/PID/correlation
[ ] Preserve original .rdbg
```

---

# 123. Summary

This toolkit provides a complete debugging pipeline from live robot operation through detailed post-test engineering analysis.

At the live stage, `DebugGUI.py` provides:

```text
serial connection
dynamic telemetry
live plots
parameters
commands
debug mode
software stop
logs
raw serial
recording
manual fault marking
test metadata
Git metadata
parameter snapshots
link-health monitoring
```

`DataRecorder.py` preserves this information in a high-performance `.rdbg` SQLite database using a queued background writer.

`DataVisualiser.py` provides:

```text
offline plots
safe autoscaling
all-zero handling
blue curve highlighting
cursor inspection
measurement regions
fault lines
command/parameter/state markers
event navigation
derived signals
recording comparison
fault alignment
anomaly detection
raw serial viewing
metadata
parameter snapshots
CSV export
```

`DataAnalysisCLI.py` provides:

```text
summary
signal discovery
fault extraction
event extraction
fault context
statistics
raw ranges
correlation
PID metrics
anomaly detection
JSON export
Markdown reporting
```

The intended engineering loop is:

```text
BUILD
  ↓
RUN
  ↓
RECORD
  ↓
MARK FAULTS
  ↓
VISUALISE
  ↓
MEASURE
  ↓
COMPARE
  ↓
ANALYSE
  ↓
CHANGE ONE THING
  ↓
RUN AGAIN
```

The most important thing to preserve is not merely telemetry—it is **context**.

A useful recording tells you:

```text
what the robot was commanded to do
what it actually did
what the controller was doing
what parameters were active
what code revision was running
what the robot reported internally
what the operator physically observed
when every event happened
```

That is what turns a debugging session into repeatable engineering evidence.

---

# 124. Current Core Files

Keep the current versions of these files together as one toolkit revision:

```text
BluetoothSerial.py
DebugGUI.py
DataRecorder.py
DataVisualiser.py
DataAnalysisCLI.py
ROBOT_DEBUG_TOOLKIT_GUIDE.md
```

If a feature is changed later, this guide should be updated alongside the code so it remains the authoritative reference for the PC-side robot debugging system.
