# Robot Debug Recording and Analysis Workflow

## Complete run recording and replay

New recordings use schema version 4. In addition to every telemetry value,
command, parameter change, robot-state packet, log, fault marker and raw serial
line, the recorder stores every complete 8x8 frame in `matrix_frames` and a
complete GUI snapshot once per second in `ui_snapshots`. Recording metadata
also embeds the debug configuration, editable wiring guide and sensor layout
that were active when the run began.

Open a recording in `DataVisualiser.py` and select **Run Replay**. The common
timeline, Play/Pause button, speed control and Go to End button drive replay
views for Dashboard, 8x8 TOF, Arena View, Plots, Parameters, Commands, Wiring
Guide, Logs and Raw Serial. **End View** is separately reconstructed to the end
of the run and remains fixed for inspection. Version 3 recordings remain
readable; their 8x8 frames are recovered from raw serial where possible, but
they cannot contain GUI snapshots or embedded wiring/layout data that were
never recorded.

**Planned Arena** replays the pre-laid mission layout separately from the
discovered Arena View. New recordings save the exact editor weights, obstacles,
start and route. The cyan line is the commanded route, the orange trace is the
recorded mission pose, and the blue marker is the pose at the current replay
time. For older recordings lacking an editor-layout snapshot, the view is
labelled as reconstructed from the uploaded mission map and plan commands;
those target waypoints only approximate the original weight centres.
At each replay time the red forward ray is the observed TOF distance and the
cyan dashed ray is the next wall predicted by the pre-laid physical arena
layout. A large difference is explicitly marked as an obstacle or possible
pose/map mismatch; the comparison is approximate because the navigation
forward value can come from either a point TOF or the 8x8 sensor.

## Purpose

This project has two distinct operating modes:

1. **Live debugging while the robot is running**
2. **Detailed analysis after the test is complete**

The live GUI is used to connect to the robot, watch telemetry, send commands, tune parameters, and record a complete test session.

The offline analysis tools are then used to inspect exactly what happened during the recorded run.

The recording is deliberately designed to be usable by both:

- a human through `DataVisualiser.py`
- an AI/code agent through `DataAnalysisCLI.py`

---

# Folder Layout

```text
Non-Teensy Files/
├── BluetoothDebugGUI/
│   ├── BluetoothSerial.py
│   ├── DebugGUI.py
│   └── DataRecorder.py
│
└── DataVisualisationAfter/
    ├── Data/
    │   ├── RobotRun_2026-09-05_22-00-00.rdbg
    │   ├── PID_Test_01.rdbg
    │   └── ...
    │
    ├── DataVisualiser.py
    └── DataAnalysisCLI.py
```

---

# 1. Live Test Workflow

Start the live GUI:

```bash
python DebugGUI.py
```

The normal workflow is:

```text
Start DebugGUI.py
       │
       ▼
Select COM port
       │
       ▼
Connect
       │
       ▼
Robot advertises telemetry,
parameters and commands
       │
       ▼
Click Start Recording
       │
       ▼
Choose / name recording
       │
       ▼
Run test
       │
       ├── telemetry recorded
       ├── logs recorded
       ├── commands recorded
       ├── parameter changes recorded
       ├── state packets recorded
       └── raw serial recorded
       │
       ▼
Click Stop Recording
       │
       ▼
.rdbg file remains in Data/
```

---

# 2. Start Recording Button

The recording controls are visible in the top connection bar:

```text
Port        Baud       Connect       Start Recording
[COM15 ▼]   [115200▼]  [Connect]     [Start Recording]

                                ○ NOT RECORDING
                                00:00:00
```

After connecting, the button becomes enabled.

Pressing it opens a normal **Save As** window.

The default directory is:

```text
DataVisualisationAfter/Data/
```

The suggested name is date/time stamped:

```text
RobotRun_2026-09-05_23-02-32.rdbg
```

You can replace that with something more meaningful:

```text
Drive_PID_Kp_Test_01_2026-09-05_23-02-32.rdbg
```

Once recording starts:

```text
[Stop Recording]   ● RECORDING — Drive_PID_Kp_Test_01_....rdbg   00:01:32
```

Disconnecting from the robot or closing the GUI automatically closes the active recording cleanly.

---

# 3. What Is Recorded

The `.rdbg` file is an SQLite database.

It contains multiple synchronized tables instead of flattening everything into one CSV.

## Telemetry

Every telemetry value received from the robot:

```text
drive.left_rpm
drive.right_rpm
drive.left_current
imu.heading
battery.voltage
...
```

Each sample includes:

```text
elapsed_s
wall_time
robot_time
signal
value
value type
```

---

## Commands

Every command sent from the GUI is recorded.

Example:

```text
18.251 s
drive_test
{
    "speed": 0.5,
    "duration_ms": 2000
}
```

---

## Parameter changes

Every tuning adjustment is recorded.

Example:

```text
31.482 s
drive.pid.kp
1.6
```

This is extremely important because later you can compare the robot response immediately before and after a tuning change.

---

## Robot states

State packets are recorded.

For example:

```json
{
    "debug_mode": true,
    "stopped": false,
    "fault": false
}
```

---

## Logs

Structured logs are recorded:

```text
INFO
WARNING
ERROR
```

---

## Raw serial

Every raw incoming serial line is also saved.

This remains useful if a structured JSON parser missed something or firmware generated malformed output.

---

# 4. Human Offline Analysis

Launch:

```bash
python DataVisualiser.py
```

The visualiser can open any `.rdbg` recording.

Current capabilities include:

```text
recording selection
searchable signal list
multiple plotted signals
elapsed-time plots
robot-time plots
event table
parameter-change history
command history
robot-state history
logs
raw serial
session metadata
CSV export
```

The idea is that the live GUI is for operating the robot.

The offline GUI is for investigating the test.

---

# 5. Headless / AI Analysis

The exact same recording can be analysed without a GUI.

This is what `DataAnalysisCLI.py` is for.

It only uses Python's standard library and does not require PyQt.

That means tools such as Claude Code can run commands against recordings directly from a terminal.

---

# 6. AI-Friendly Commands

## Get an overall run summary

```bash
python DataAnalysisCLI.py summary Data/PID_Test_01.rdbg
```

The output is JSON.

Example shape:

```json
{
    "duration_s": 92.4,
    "telemetry_samples": 18732,
    "signal_count": 17,
    "command_count": 4,
    "parameter_change_count": 3,
    "signals": [
        "drive.left_rpm",
        "drive.right_rpm",
        "drive.pid_output"
    ]
}
```

An AI can immediately inspect the structure of the run.

---

## List available signals

```bash
python DataAnalysisCLI.py signals Data/PID_Test_01.rdbg
```

Numeric-only:

```bash
python DataAnalysisCLI.py signals Data/PID_Test_01.rdbg --numeric-only
```

---

## Statistics for a signal

```bash
python DataAnalysisCLI.py stats Data/PID_Test_01.rdbg \
    --signal drive.left_rpm
```

For only part of the run:

```bash
python DataAnalysisCLI.py stats Data/PID_Test_01.rdbg \
    --signal drive.left_rpm \
    --start 20 \
    --end 35
```

The result includes:

```text
sample count
min
max
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
sample rate
```

---

# 7. Extract Raw Signal Data

```bash
python DataAnalysisCLI.py range Data/PID_Test_01.rdbg \
    --signal drive.left_current \
    --start 15 \
    --end 25
```

The result is JSON containing every selected sample.

This is useful when an AI wants to perform its own detailed calculations.

---

# 8. Inspect Events

```bash
python DataAnalysisCLI.py events Data/PID_Test_01.rdbg
```

Or a narrow time range:

```bash
python DataAnalysisCLI.py events Data/PID_Test_01.rdbg \
    --start 18 \
    --end 23
```

This combines:

```text
logs
commands
parameter changes
robot states
```

into one chronological list.

---

# 9. Ask "What Was Happening Around This Time?"

A particularly useful AI command is:

```bash
python DataAnalysisCLI.py context Data/PID_Test_01.rdbg \
    --time 31.5 \
    --window 2
```

This returns:

```text
events from 29.5 to 33.5 seconds
nearest value of every signal at 31.5 seconds
statistics for every numeric signal in that window
```

This lets an AI investigate questions like:

> Why did current spike at 31.5 seconds?

The AI can see what command was running, which parameters changed, and what all sensors were doing around that time.

---

# 10. Correlation Analysis

For several numeric signals:

```bash
python DataAnalysisCLI.py correlate Data/PID_Test_01.rdbg \
    --signals \
    drive.left_rpm \
    drive.right_rpm \
    drive.left_current \
    drive.pid_output
```

The output contains a correlation matrix.

It can also be limited to a period:

```bash
python DataAnalysisCLI.py correlate Data/PID_Test_01.rdbg \
    --signals drive.left_rpm drive.left_current \
    --start 20 \
    --end 30
```

This gives an AI a quick way to look for relationships between measurements.

---

# 11. Export a Whole Run to AI-Friendly JSON

```bash
python DataAnalysisCLI.py export-json \
    Data/PID_Test_01.rdbg \
    --output PID_Test_01_analysis.json
```

This produces a JSON summary containing:

```text
metadata
run duration
signal list
statistics for every numeric signal
all commands
all parameter changes
all logs
all robot states
```

This can be fed into another tool or inspected directly by an AI.

The raw telemetry remains in the `.rdbg` database and can be queried only where needed, avoiding unnecessarily huge context dumps.

---

# 12. Recommended Claude Code Workflow

A useful instruction to Claude Code could be:

```text
Analyse the robot recording Data/PID_Test_01.rdbg.

First run:
python DataAnalysisCLI.py summary ...

Then inspect the available signals.

Identify relevant commands and parameter changes with:
python DataAnalysisCLI.py events ...

Use stats/range/context/correlate as needed.

Do not load the entire raw recording into context unless necessary.
Give me:
- anomalies
- timing relationships
- likely causes
- control-loop behaviour
- recommendations for the next test
```

This is much more efficient than asking an AI to parse a giant CSV.

---

# 13. Why Headless Analysis Matters

A GUI is useful for humans.

An AI generally works better through:

```text
structured JSON
command-line tools
small targeted queries
statistics
event ranges
explicit signal names
```

So the architecture deliberately has two interfaces:

```text
                    .rdbg
                      │
          ┌───────────┴───────────┐
          ▼                       ▼
 DataVisualiser.py       DataAnalysisCLI.py
          │                       │
       Human GUI             AI / scripts
```

Both operate on exactly the same recorded data.

---

# 14. Full Engineering Workflow

The intended long-term workflow is:

```text
DEVELOP
   │
   ▼
Write / modify Teensy code
   │
   ▼
Flash robot
   │
   ▼
LIVE TEST
   │
   ├── watch telemetry
   ├── send commands
   ├── tune parameters
   ├── monitor faults
   └── record session
   │
   ▼
SAVE .rdbg
   │
   ├───────────────────────┐
   ▼                       ▼
Human analysis          AI analysis
DataVisualiser.py       DataAnalysisCLI.py
   │                       │
   ├── plots                ├── statistics
   ├── events               ├── correlations
   ├── zoom                 ├── event context
   └── inspect              └── anomaly analysis
   │                       │
   └───────────┬───────────┘
               ▼
          FIND PROBLEM
               │
               ▼
       Change firmware/tuning
               │
               ▼
           NEXT TEST
```

Every run therefore becomes a permanent, reproducible engineering record.

---

# 15. Example Analysis Scenario

Suppose during a drive test the robot suddenly veers left.

The run contains:

```text
21.30 s  command: drive_test(speed=0.60)
25.10 s  parameter: drive.pid.kp -> 1.80
27.42 s  left current increases
27.55 s  left RPM decreases
27.61 s  heading error increases
27.70 s  PID output saturates
28.02 s  WARNING: drive slip
```

You can inspect this visually in `DataVisualiser.py`.

Or an AI can run:

```bash
python DataAnalysisCLI.py context Test.rdbg \
    --time 27.6 \
    --window 3
```

Then:

```bash
python DataAnalysisCLI.py correlate Test.rdbg \
    --signals \
    drive.left_current \
    drive.left_rpm \
    drive.error \
    drive.pid_output \
    --start 24 \
    --end 30
```

The AI now has enough structured information to reason about whether the likely cause is:

```text
traction loss
mechanical resistance
motor loading
encoder error
controller tuning
current limiting
```

---

# 16. Design Rule

The key principle is:

> Record enough context that a test can be understood later without needing to remember what you were doing manually.

That is why the recording contains both measurements **and** actions.

A sensor trace by itself often does not explain a failure.

A trace plus:

```text
commands
parameter changes
logs
robot state
timestamps
```

usually does.

---

# 17. Files Added by This Suite

## `BluetoothDebugGUI/DataRecorder.py`

Responsible for creating and writing `.rdbg` recordings.

## `BluetoothDebugGUI/DebugGUI.py`

Updated live GUI containing the visible Start/Stop Recording control and automatic recording hooks.

## `DataVisualisationAfter/DataVisualiser.py`

Human-facing offline analysis GUI.

## `DataVisualisationAfter/DataAnalysisCLI.py`

Headless AI/script-facing analysis interface.

## `DataVisualisationAfter/Data/`

Default location for all recorded sessions.

---

# 18. Immediate Test

To test recording now:

```text
1. Put DataRecorder.py beside DebugGUI.py.
2. Replace DebugGUI.py with the updated supplied file.
3. Keep your current BluetoothSerial.py unchanged.
4. Run DebugGUI.py.
5. Connect to the ESP32 simulator.
6. The Start Recording button should enable.
7. Press Start Recording.
8. Choose a filename.
9. Let telemetry run for 10-20 seconds.
10. Send a few commands / change a parameter.
11. Press Stop Recording.
12. Open DataVisualiser.py.
13. Load the newly created .rdbg file.
14. Or analyse it from the terminal using DataAnalysisCLI.py.
```

At that point the entire live → record → human analysis → AI analysis pipeline is working.
