# Robot Debug GUI Communication Protocol

## Overview

This document describes how the robot debug system communicates between the robot and the computer, how data is formatted, how the Python GUI interprets it, and how the robot can use commands and tuning parameters sent back from the computer.

The system is designed around one simple idea:

> The Bluetooth link behaves like a normal serial cable.

The GUI does not need to know whether the serial connection is:

- a direct USB cable,
- a USB-to-UART adapter,
- the CH9143 Bluetooth pair,
- another transparent wireless serial bridge,
- or a future serial transport.

As long as both ends exchange the same newline-delimited JSON messages at the same baud rate, the higher-level protocol remains the same.

The current Python application consists of:

```text
DebugGUI.py
BluetoothSerial.py
```

The robot firmware implements the matching protocol.

The communication path is:

```text
┌──────────────────── COMPUTER ────────────────────┐
│                                                 │
│  DebugGUI.py                                    │
│      │                                          │
│      │ PyQt signals / function calls            │
│      ▼                                          │
│  BluetoothSerial.py                             │
│      │                                          │
│      │ pyserial                                 │
│      ▼                                          │
│  Selected COM port                              │
└──────┬──────────────────────────────────────────┘
       │
       │ newline-delimited JSON
       │
       │ 115200 baud by default
       │
       ▼
┌────────────── Bluetooth receiver ───────────────┐
│                    CH9143                       │
└───────────────────┬─────────────────────────────┘
                    )))
                    ))) Bluetooth
                    )))
┌───────────────────▼─────────────────────────────┐
│                    CH9143                       │
│             Robot-side transceiver              │
└───────────────────┬─────────────────────────────┘
                    │
                    │ UART
                    ▼
┌──────────────────── ROBOT ──────────────────────┐
│                                                │
│ Teensy 4.0                                     │
│                                                │
│ Debug serial interface                         │
│      │                                         │
│      ├── sends telemetry                       │
│      ├── sends parameter definitions           │
│      ├── sends command definitions             │
│      ├── sends logs                            │
│      ├── sends state                           │
│      │                                         │
│      └── receives commands and parameter edits │
│                                                │
└────────────────────────────────────────────────┘
```

---

# 1. Physical Communication Layer

## 1.1 Bluetooth serial link

The CH9143 pair behaves as a transparent serial connection.

One unit connects to the robot UART and the other appears on the computer as a COM port.

For the computer, this could appear as:

```text
COM4
COM7
COM11
COM15
```

The GUI automatically scans available serial ports and puts them in a dropdown.

The user does not need to edit Python code to change ports.

The selected port is passed to `BluetoothSerial.py`, which opens it using `pyserial`.

The default baud rate used by the current examples is:

```text
115200 baud
8 data bits
no parity
1 stop bit
```

or:

```text
115200 8N1
```

The baud rate can also be selected from the GUI.

---

# 2. Data Framing

The protocol uses:

```text
newline-delimited JSON
```

Every complete message is one JSON object followed by a newline character:

```text
\n
```

For example:

```json
{"type":"telemetry","name":"battery.voltage","value":12.42}
```

is actually transmitted as:

```text
{"type":"telemetry","name":"battery.voltage","value":12.42}\n
```

The newline is important because it tells the receiving side:

> This JSON packet is complete and may now be parsed.

The next packet starts after that newline.

For example, the actual serial stream may look like:

```text
{"type":"telemetry","name":"battery.voltage","value":12.42}\n
{"type":"telemetry","name":"imu.heading","value":91.5}\n
{"type":"log","level":"INFO","message":"Robot ready"}\n
```

The serial layer does not require fixed packet lengths.

Packets may contain different fields and different amounts of data.

---

# 3. Why JSON Is Used

JSON was chosen because it is:

- easy to read,
- easy to debug,
- flexible,
- easy to generate on embedded systems,
- built into Python,
- easy to inspect in the Raw Serial tab,
- self-describing,
- easy to extend later.

For a debug interface, readability and flexibility are more valuable than saving a small number of bytes.

A Teensy 4.0 has more than enough processing capability for this style of debugging protocol at ordinary telemetry rates.

If extremely high-rate telemetry is ever required, the transport could later be replaced by a binary protocol without redesigning the GUI concept.

---

# 4. The `type` Field

Every protocol message should contain a `type` field.

Example:

```json
{
    "type": "telemetry"
}
```

The `type` field tells the receiver how to interpret the rest of the packet.

The currently supported message types are:

```text
hello
request_definitions
telemetry
tof_8x8
parameter_definition
parameter
parameter_request
parameter_value
command_definition
command
log
state
error
heartbeat
```

Not every one has to be implemented immediately.

The important rule is:

> Never infer a packet's meaning only from the fields it contains. Use `type`.

This makes the protocol predictable and easy to extend.

---

# 5. Connection Sequence

When the user clicks **Connect**, the following process occurs.

```text
User selects COM port
        │
        ▼
DebugGUI.py
        │
        ▼
BluetoothSerial.connect_port()
        │
        ▼
SerialWorker thread starts
        │
        ▼
COM port opens
        │
        ▼
GUI receives connection_changed(True)
        │
        ▼
Computer sends "hello"
        │
        ▼
Computer sends "request_definitions"
        │
        ▼
Robot advertises parameters and commands
        │
        ▼
Robot begins sending telemetry
```

The first packet sent from the computer is currently:

```json
{
    "type": "hello",
    "client": "RobotDebugGUI",
    "protocol": 1
}
```

This allows the robot to know that a compatible debug client has connected.

The robot can respond with something such as:

```json
{
    "type": "log",
    "level": "INFO",
    "message": "Debug GUI connected"
}
```

It can also immediately send its state:

```json
{
    "type": "state",
    "debug_mode": true,
    "stopped": false,
    "fault": false,
    "uptime_ms": 12345
}
```

The computer then sends:

```json
{
    "type": "request_definitions"
}
```

This asks the robot to tell the GUI what parameters and commands are available.

That is what allows the GUI to remain generic.

---

# 6. Robot-to-Computer Telemetry

Telemetry is information generated by the robot and displayed by the GUI.

An SEN0628 matrix frame uses a dedicated packet so 64 zones do not clutter the
ordinary telemetry table:

```json
{
    "type": "tof_8x8",
    "time": 54321,
    "name": "matrix",
    "available": true,
    "valid": true,
    "frame": 42,
    "data": [320, 325, 330, 335, 340, 345, 350, 355]
}
```

`data` contains exactly 64 row-major millimetre values when `valid` is true.
The shortened array above illustrates the field only. The live application
renders the complete frame as eight Y rows by eight X columns.

Typical telemetry includes:

```text
battery voltage
motor RPM
motor current
encoder position
IMU heading
gyro rate
pitch
roll
servo position
distance sensor readings
PID error
PID output
temperature
state-machine state
fault flags
uptime
```

The robot may send a single signal:

```json
{
    "type": "telemetry",
    "name": "battery.voltage",
    "value": 12.41
}
```

The GUI interprets that as:

```text
Signal = battery.voltage
Value  = 12.41
```

The GUI then automatically adds it to the telemetry table.

If the value is numeric, it also becomes available in the plot selector.

---

# 7. Sending Multiple Telemetry Values

Sending every signal as a separate JSON message works, but it creates more serial overhead.

The preferred format is therefore a combined telemetry packet:

```json
{
    "type": "telemetry",
    "time": 123456,
    "data": {
        "power.battery_voltage": 12.41,
        "drive.left_rpm": 421.2,
        "drive.right_rpm": 419.7,
        "imu.heading": 83.4,
        "imu.pitch": 1.2,
        "imu.roll": -0.7
    }
}
```

`BluetoothSerial.py` loops through everything inside `data`.

It internally emits the equivalent of:

```text
telemetry_received("power.battery_voltage", 12.41, 123456)
telemetry_received("drive.left_rpm", 421.2, 123456)
telemetry_received("drive.right_rpm", 419.7, 123456)
telemetry_received("imu.heading", 83.4, 123456)
...
```

`DebugGUI.py` then treats every item as its own independent signal.

---

# 8. Telemetry Naming

A dotted naming system is recommended:

```text
power.battery_voltage
drive.left_rpm
drive.right_rpm
drive.left_current
drive.pid.error
drive.pid.output
imu.heading
imu.pitch
imu.roll
sorter.gate_angle
intake.rpm
distance.front
system.cpu_temperature
system.uptime
```

This is not required by the protocol.

The GUI simply treats the entire name as a string.

However, dotted names are useful because they naturally organise signals into groups.

For example:

```text
drive.left_rpm
drive.right_rpm
drive.left_current
drive.right_current
```

all obviously belong to the drivetrain.

A future GUI could use the text before the first dot to automatically group them under:

```text
Drive
```

---

# 9. How Telemetry Is Used by the GUI

When telemetry arrives, `BluetoothSerial.py` parses the JSON and emits:

```python
telemetry_received(name, value, timestamp)
```

`DebugGUI.py` receives that event in:

```python
on_telemetry(...)
```

The GUI then:

1. stores the latest value,
2. creates a row if the signal is new,
3. updates the displayed value,
4. updates its last-received time,
5. stores numeric samples in a history buffer,
6. makes numeric signals available for plotting.

The GUI therefore does not have to know beforehand what sensors exist.

If firmware begins sending:

```json
{
    "type": "telemetry",
    "name": "reel.motor_temperature",
    "value": 46.2
}
```

then `reel.motor_temperature` automatically appears.

No Python changes are required.

---

# 10. Telemetry History and Plotting

For numeric values, `DebugGUI.py` stores history in a `deque`.

Conceptually:

```python
telemetry_history = {
    "drive.left_rpm": [
        (0.00, 410.2),
        (0.10, 416.5),
        (0.20, 421.7),
        ...
    ]
}
```

The GUI currently limits each signal to:

```text
10,000 samples
```

This prevents memory usage from growing forever.

PyQtGraph reads that history and displays only the selected time window.

For example:

```text
5 seconds
10 seconds
30 seconds
1 minute
2 minutes
5 minutes
```

The robot does not need to know anything about plotting.

It simply transmits numbers.

The GUI handles the visualization.

---

# 11. Computer-to-Robot Commands

Commands tell the robot to perform an action.

Example:

```json
{
    "type": "command",
    "command": "stop"
}
```

Another example:

```json
{
    "type": "command",
    "command": "drive_test",
    "speed": 0.30,
    "duration_ms": 2000
}
```

The robot receives the packet, reads:

```text
type = command
```

then reads:

```text
command = drive_test
```

and executes the appropriate handler.

Conceptually:

```cpp
if (command == "drive_test")
{
    float speed = packet["speed"];
    int duration = packet["duration_ms"];

    startDriveTest(speed, duration);
}
```

This is how GUI buttons ultimately control robot functions.

---

# 12. Command Definitions

The GUI does not need to contain hard-coded controls for every robot function.

Instead, the robot can tell the GUI which commands exist.

For example:

```json
{
    "type": "command_definition",
    "name": "drive_test",
    "label": "Drive Motor Test",
    "description": "Run both tracks at the selected speed.",
    "args": [
        {
            "name": "speed",
            "label": "Speed",
            "type": "float",
            "min": -1.0,
            "max": 1.0,
            "step": 0.05,
            "default": 0.30
        },
        {
            "name": "duration_ms",
            "label": "Duration",
            "type": "int",
            "min": 100,
            "max": 10000,
            "step": 100,
            "default": 2000
        }
    ]
}
```

The GUI automatically creates something similar to:

```text
Drive Motor Test

Speed       [ 0.30 ]
Duration    [ 2000 ]

[ Run Drive Motor Test ]
```

When the button is pressed, the GUI reads the values and sends:

```json
{
    "type": "command",
    "command": "drive_test",
    "speed": 0.30,
    "duration_ms": 2000
}
```

This means adding a new robot debug command can be achieved entirely from firmware.

---

# 13. Command Argument Types

The current GUI can automatically generate editors for common argument types.

## Float

Definition:

```json
{
    "name": "speed",
    "type": "float",
    "min": -1,
    "max": 1,
    "step": 0.05,
    "default": 0.3
}
```

GUI control:

```text
QDoubleSpinBox
```

---

## Integer

Definition:

```json
{
    "name": "duration_ms",
    "type": "int",
    "min": 100,
    "max": 10000,
    "step": 100,
    "default": 1000
}
```

GUI control:

```text
QSpinBox
```

---

## Boolean

Definition:

```json
{
    "name": "enabled",
    "type": "bool",
    "default": true
}
```

GUI control:

```text
QCheckBox
```

---

## String

Definition:

```json
{
    "name": "message",
    "type": "string",
    "default": ""
}
```

GUI control:

```text
QLineEdit
```

---

## Enum / Selection

Definition:

```json
{
    "name": "mode",
    "type": "enum",
    "options": [
        "AUTO",
        "MANUAL",
        "TEST"
    ],
    "default": "AUTO"
}
```

GUI control:

```text
QComboBox
```

This is particularly useful because the user does not need to type mode names manually.

---

# 14. Parameters

Parameters differ from commands.

A command says:

> Do something.

A parameter says:

> Change a value used by the robot.

Examples include:

```text
PID Kp
PID Ki
PID Kd
maximum speed
servo position
sensor threshold
filter constant
target RPM
current limit
timeout
control mode
telemetry interval
```

For example:

```json
{
    "type": "parameter",
    "name": "drive.pid.kp",
    "value": 1.5
}
```

The robot receives the packet and may perform:

```cpp
driveKp = packet["value"];
```

From that point onward, the control algorithm uses the new value.

For example:

```cpp
pidOutput =
    driveKp * error
    + driveKi * integral
    + driveKd * derivative;
```

Changing `drive.pid.kp` in the GUI therefore immediately affects robot control behaviour.

This is the main mechanism for wireless tuning.

---

# 15. Parameter Definitions

As with commands, parameters are advertised by the robot.

Example:

```json
{
    "type": "parameter_definition",
    "name": "drive.pid.kp",
    "label": "Drive PID - Kp",
    "description": "Proportional gain for the drivetrain controller.",
    "datatype": "float",
    "value": 1.20,
    "min": 0.0,
    "max": 10.0,
    "step": 0.01,
    "decimals": 3
}
```

The GUI uses that definition to automatically create a suitable control.

The firmware decides:

```text
name
label
description
datatype
current value
minimum
maximum
step
number of decimals
unit
```

The GUI therefore does not need a manually written editor for `Kp`.

---

# 16. Changing a Parameter

Assume the robot advertises:

```text
drive.pid.kp = 1.20
```

The user changes the GUI to:

```text
1.60
```

and presses **Apply**.

The following sequence occurs:

```text
User changes Kp
      │
      ▼
DebugGUI.py
      │
      │ bluetooth.set_parameter(...)
      ▼
BluetoothSerial.py
      │
      ▼
JSON generated:
{"type":"parameter","name":"drive.pid.kp","value":1.6}
      │
      ▼
Serial COM port
      │
      ▼
Bluetooth
      │
      ▼
Robot UART
      │
      ▼
Robot JSON parser
      │
      ▼
driveKp = 1.6
```

The robot should then confirm the value by sending:

```json
{
    "type": "parameter_value",
    "name": "drive.pid.kp",
    "value": 1.6
}
```

The GUI receives the confirmation and updates the editor.

---

# 17. Why the Robot Should Confirm Parameters

It is better for the robot to echo the accepted value than for the GUI to simply assume the requested value was accepted.

Suppose the GUI requests:

```json
{
    "type": "parameter",
    "name": "drive.max_speed",
    "value": 1.7
}
```

but the firmware limits that parameter to:

```text
0.0 to 1.0
```

The firmware might clamp it:

```cpp
maxDriveSpeed = constrain(requestedValue, 0.0f, 1.0f);
```

and reply:

```json
{
    "type": "parameter_value",
    "name": "drive.max_speed",
    "value": 1.0
}
```

The GUI now displays the actual robot value.

This prevents the computer and robot from becoming inconsistent.

---

# 18. Parameter Requests

The computer can explicitly request the current value of a parameter.

Example:

```json
{
    "type": "parameter_request",
    "name": "drive.pid.kp"
}
```

The robot replies:

```json
{
    "type": "parameter_value",
    "name": "drive.pid.kp",
    "value": 1.20
}
```

This is useful when:

- the value may have changed internally,
- configuration was loaded from EEPROM,
- configuration was loaded from an SD card,
- another subsystem changed the value,
- the GUI reconnects.

---

# 19. Debug Mode

The robot should distinguish between normal operation and debug operation.

For example:

```cpp
enum class RobotMode
{
    NORMAL,
    DEBUG
};
```

A common policy would be:

```text
NORMAL MODE
    telemetry allowed
    logs allowed
    dangerous commands rejected
    parameter changes optionally rejected

DEBUG MODE
    telemetry allowed
    commands allowed
    parameter tuning allowed
    actuator tests allowed
    test routines allowed
```

The GUI currently sends:

```json
{
    "type": "command",
    "command": "set_debug_mode",
    "enabled": true
}
```

to enter debug mode.

And:

```json
{
    "type": "command",
    "command": "set_debug_mode",
    "enabled": false
}
```

to leave it.

The robot can then respond with:

```json
{
    "type": "state",
    "debug_mode": true
}
```

The robot firmware should decide what operations are safe to permit in each mode.

---

# 20. Robot State Messages

State packets communicate high-level robot status.

Example:

```json
{
    "type": "state",
    "debug_mode": true,
    "stopped": false,
    "fault": false,
    "uptime_ms": 52341
}
```

Possible future fields could include:

```text
robot_mode
autonomous_state
enabled
estop
battery_low
fault_code
current_task
connected
homed
drive_enabled
sorter_enabled
```

Example:

```json
{
    "type": "state",
    "debug_mode": true,
    "robot_mode": "IDLE",
    "estop": false,
    "drive_enabled": true,
    "sorter_homed": true
}
```

State differs from telemetry mainly in meaning.

Telemetry represents continuously changing measurements.

State represents operating status.

---

# 21. Logs

The robot can send human-readable messages to the GUI.

Example:

```json
{
    "type": "log",
    "level": "INFO",
    "message": "Sorting gate homed"
}
```

The current GUI displays this in the Logs tab.

Suggested levels are:

```text
DEBUG
INFO
WARNING
ERROR
CRITICAL
```

Example:

```json
{
    "type": "log",
    "level": "WARNING",
    "message": "Left encoder signal lost"
}
```

or:

```json
{
    "type": "log",
    "level": "ERROR",
    "message": "Motor driver fault"
}
```

This is much better than relying on unstructured `Serial.println()` output because software can interpret the level separately from the message.

---

# 22. Error Packets

For machine-readable communication errors, the robot can send:

```json
{
    "type": "error",
    "message": "Unknown command: spin_the_robot"
}
```

`BluetoothSerial.py` translates this to an error event and the GUI displays it.

A future version could include additional fields:

```json
{
    "type": "error",
    "code": "INVALID_PARAMETER",
    "name": "drive.pid.kp",
    "message": "Value exceeds maximum"
}
```

---

# 23. Raw Serial Tab

Every received line is emitted to the Raw Serial tab before JSON interpretation.

This means that even if JSON parsing fails, the user can still see what arrived.

For example:

```text
{"type":"telemetry","name":"battery.voltage","value":12.4}
{"type":"log","level":"INFO","message":"Robot ready"}
THIS IS NOT VALID JSON
```

The first two packets are parsed.

The third line appears in Raw Serial but is ignored by the structured protocol parser.

This is useful for diagnosing firmware bugs.

---

# 24. The Python Threading Model

Serial communication does not run directly in the main PyQt GUI thread.

That is important.

If the GUI itself called a blocking serial read, the window could freeze while waiting for data.

Instead:

```text
Main PyQt thread
    │
    ├── handles buttons
    ├── handles tables
    ├── handles plots
    └── handles windows

SerialWorker QThread
    │
    ├── reads serial
    ├── writes serial
    └── parses incoming lines
```

Communication between the worker and GUI uses PyQt signals.

For example:

```python
telemetry_received = pyqtSignal(str, object, object)
```

The worker can emit telemetry without directly touching GUI widgets.

This is the correct architecture for a responsive Qt application.

---

# 25. Sending Data From Python

`DebugGUI.py` does not directly write bytes to the COM port.

Instead it calls higher-level methods from `BluetoothSerial.py`.

Examples:

```python
self.bluetooth.send_command(
    "stop"
)
```

or:

```python
self.bluetooth.send_command(
    "drive_test",
    speed=0.30,
    duration_ms=2000
)
```

or:

```python
self.bluetooth.set_parameter(
    "drive.pid.kp",
    1.5
)
```

`BluetoothSerial.py` turns those calls into JSON.

For example:

```python
self.bluetooth.set_parameter(
    "drive.pid.kp",
    1.5
)
```

becomes:

```json
{
    "type": "parameter",
    "name": "drive.pid.kp",
    "value": 1.5
}
```

The GUI therefore never needs to care about serial formatting.

---

# 26. Receive Queue and Transmit Queue

The serial worker contains a transmit queue.

When the GUI wants to send something, it does not directly call:

```python
serial.write(...)
```

Instead the message is placed in a thread-safe queue.

Conceptually:

```text
GUI
 │
 │ send command
 ▼
TX Queue
 │
 │ SerialWorker reads queue
 ▼
COM port
```

This prevents multiple parts of the GUI from attempting to write to the port simultaneously.

---

# 27. How the Robot Should Parse Incoming Data

The robot should maintain a receive buffer.

Conceptually:

```cpp
String rxBuffer;
```

Each loop:

```cpp
while (DebugSerial.available())
{
    char c = DebugSerial.read();

    if (c == '\n')
    {
        parseMessage(rxBuffer);
        rxBuffer = "";
    }
    else
    {
        rxBuffer += c;
    }
}
```

The important design rule is:

> Never assume an entire JSON message arrives in one UART read.

Serial data is a stream.

For example, one packet might physically arrive as:

```text
{"type":"para
```

then later:

```text
meter","name":"drive.pid.kp",
```

then later:

```text
"value":1.5}\n
```

The receive buffer reassembles it until the newline appears.

---

# 28. Robot Main Loop Integration

The debug system should not take over the robot firmware.

It should be treated as another subsystem.

For example:

```cpp
void loop()
{
    updateSensors();

    updateDrive();
    updateSorter();
    updateIntake();

    Debug.update();

    sendDebugTelemetry();
}
```

`Debug.update()` handles incoming serial commands.

Normal robot functions continue to run as usual.

---

# 29. Using Received Parameters Inside the Robot

A received parameter should normally modify the same variables used by the real control system.

For example:

```cpp
float driveKp = 1.2f;
float driveKi = 0.08f;
float driveKd = 0.015f;
```

The control loop:

```cpp
float output =
    driveKp * error +
    driveKi * integral +
    driveKd * derivative;
```

When the GUI sends:

```json
{
    "type": "parameter",
    "name": "drive.pid.kp",
    "value": 1.8
}
```

the command handler performs:

```cpp
driveKp = 1.8f;
```

The next PID calculation immediately uses:

```text
Kp = 1.8
```

This makes wireless real-time tuning possible.

---

# 30. Temporary vs Persistent Parameters

Changing a parameter over the debug interface does not automatically mean it survives a reboot.

There are two useful categories.

## Temporary parameters

Stored only in RAM.

Example:

```cpp
driveKp = requestedValue;
```

After reboot:

```text
driveKp returns to compiled default
```

This is ideal for experimentation.

---

## Persistent parameters

Written to non-volatile memory.

Possible storage options on the Teensy include:

```text
EEPROM
flash-based configuration
SD card
external EEPROM
```

A future command could be:

```json
{
    "type": "command",
    "command": "save_parameters"
}
```

The robot could then save the currently tuned values.

Another command could be:

```json
{
    "type": "command",
    "command": "load_parameters"
}
```

This allows the workflow:

```text
1. Tune robot live
2. Verify performance
3. Press Save Parameters
4. Robot stores values
5. Reboot robot
6. Values remain
```

This would be a useful later addition.

---

# 31. Example PID Tuning Workflow

Suppose the drivetrain uses:

```text
Kp = 1.2
Ki = 0.08
Kd = 0.015
```

The robot advertises those parameters.

The GUI displays them.

The robot also sends:

```text
drive.error
drive.pid_output
drive.left_rpm
drive.right_rpm
```

The user can:

```text
1. Open Plots
2. Plot drive.error
3. Plot drive.pid_output
4. Start a test command
5. Change Kp
6. Press Apply
7. Observe the new response
8. Change Ki
9. Observe steady-state error
10. Change Kd
11. Observe damping
```

No USB cable is required when using the Bluetooth pair.

This is one of the primary purposes of the system.

---

# 32. Example Robot Command Workflow

Suppose the robot advertises:

```text
home_sorter
```

The GUI generates:

```text
[ Run Home Sorting Gate ]
```

The user presses it.

The computer sends:

```json
{
    "type": "command",
    "command": "home_sorter"
}
```

The robot handler runs:

```cpp
homeSortingGate();
```

Then the robot might send:

```json
{
    "type": "log",
    "level": "INFO",
    "message": "Sorting gate homing started"
}
```

When complete:

```json
{
    "type": "log",
    "level": "INFO",
    "message": "Sorting gate homed"
}
```

and perhaps:

```json
{
    "type": "state",
    "sorter_homed": true
}
```

---

# 33. Example Sensor Debugging Workflow

Suppose an encoder is behaving incorrectly.

Firmware sends:

```text
encoder.left.count
encoder.left.rpm
encoder.left.period_us
drive.left.command
```

The GUI can plot all four.

The robot can continue driving while the developer stands nearby with a laptop.

This makes it possible to determine whether a problem is:

```text
mechanical
electrical
sensor-related
software-related
control-loop-related
```

without repeatedly modifying firmware just to print different variables.

---

# 34. Recommended Debug Firmware API

The protocol becomes particularly convenient if wrapped in a Teensy debug class.

A possible future API could look like:

```cpp
Debug.begin(Serial2, 115200);
```

Register parameters:

```cpp
Debug.addParameter(
    "drive.pid.kp",
    driveKp,
    0.0f,
    10.0f,
    0.01f
);
```

Register commands:

```cpp
Debug.addCommand(
    "stop",
    emergencyStop
);
```

Send telemetry:

```cpp
Debug.telemetry(
    "drive.left_rpm",
    leftRPM
);
```

Send logs:

```cpp
Debug.info(
    "Drive controller started"
);
```

Then the main loop could simply contain:

```cpp
Debug.update();
```

The implementation of that library would handle JSON parsing and transmission.

---

# 35. Recommended Telemetry Timing

Telemetry should generally not be sent every loop iteration.

A Teensy may run control loops at hundreds or thousands of hertz.

Sending JSON at the same speed would produce unnecessary traffic.

Instead, use a separate telemetry rate.

For example:

```text
Control loop:       1000 Hz
Sensor updates:      500 Hz
Telemetry output:     20 Hz
GUI plots:            20 Hz
```

Example firmware:

```cpp
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;

uint32_t lastTelemetry = 0;

void updateDebugTelemetry()
{
    uint32_t now = millis();

    if (now - lastTelemetry < TELEMETRY_INTERVAL_MS)
        return;

    lastTelemetry = now;

    sendTelemetry();
}
```

`50 ms` gives:

```text
20 packets per second
```

which is smooth for visualisation.

---

# 36. Telemetry Bandwidth

A packet like:

```json
{"type":"telemetry","time":12345,"data":{"drive.left_rpm":420.1,"drive.right_rpm":418.3,"imu.heading":91.4}}
```

is approximately 100 to 150 bytes depending on formatting.

At 20 packets per second:

```text
~2,000 to 3,000 bytes per second
```

115200 baud provides roughly:

```text
~11,000 useful bytes per second
```

after serial framing overhead.

Therefore ordinary robotics telemetry is easily practical.

If many more signals are added, options include:

```text
send less often
send only selected telemetry
use shorter field names internally
increase baud rate
use multiple packet groups
switch to a binary protocol
```

---

# 37. Selective Telemetry — Possible Future Improvement

A useful future feature is allowing the GUI to tell the robot which signals it actually wants.

For example:

```json
{
    "type": "telemetry_subscribe",
    "signals": [
        "drive.left_rpm",
        "drive.right_rpm",
        "drive.pid_output"
    ],
    "rate_hz": 50
}
```

The robot would then only transmit those signals.

This could provide high-rate telemetry for selected variables without flooding the serial connection with everything else.

---

# 38. Heartbeats — Possible Future Improvement

The robot and GUI may periodically exchange heartbeat messages.

Example robot heartbeat:

```json
{
    "type": "heartbeat",
    "time": 153240
}
```

The GUI could detect:

```text
No heartbeat for 2 seconds
```

and show:

```text
CONNECTION LOST
```

Similarly, the robot could detect loss of the GUI.

This is especially useful with a wireless connection.

However, heartbeat loss should not automatically imply an emergency unless the robot is deliberately designed that way.

---

# 39. Message IDs and Acknowledgements — Possible Future Improvement

For operations where confirmation matters, messages could later include IDs.

Computer sends:

```json
{
    "type": "command",
    "id": 42,
    "command": "home_sorter"
}
```

Robot replies:

```json
{
    "type": "command_response",
    "id": 42,
    "success": true
}
```

or:

```json
{
    "type": "command_response",
    "id": 42,
    "success": false,
    "message": "Sorter is already moving"
}
```

This makes the protocol more robust.

It is not required for the current implementation.

---

# 40. Safety Considerations

The debug GUI should not be considered a safety system.

Wireless commands can be delayed, lost, corrupted, or disconnected.

Physical robot safety should remain implemented independently.

Examples include:

```text
physical emergency stop
motor-driver enable lines
hardware current limiting
mechanical limits
watchdogs
software command timeouts
safe startup state
safe communication-loss state
```

A GUI **STOP ROBOT** button is useful for debugging, but it should not replace a real emergency-stop mechanism.

---

# 41. Recommended Command Permissions

Commands can be classified by risk.

For example:

```text
Always allowed:
    request state
    request definitions
    read parameters
    read telemetry

Debug mode only:
    set PID values
    manually actuate servos
    run motors
    drive robot
    test sorter
    override sensors

Potentially restricted:
    disable safety limits
    erase calibration
    write persistent configuration
```

The robot firmware should enforce these rules.

Do not rely on the GUI to hide unsafe commands.

The robot is the final authority.

---

# 42. Robot Should Validate All Incoming Data

The computer should not be trusted blindly.

For example, if the GUI sends:

```json
{
    "type": "parameter",
    "name": "sorter.open_angle",
    "value": 100000
}
```

the firmware should not directly command a servo to that value.

Instead:

```cpp
sortingGateOpenAngle =
    constrain(requestedValue, 0, 180);
```

The same applies to:

```text
motor speed
current limit
servo angle
PID gains
timeouts
temperatures
RPM targets
command duration
```

The GUI provides usability.

The firmware provides safety and validation.

---

# 43. Recommended Robot Message Processing Structure

A clean firmware architecture could be:

```cpp
void handleDebugMessage(JsonDocument &doc)
{
    const char *type = doc["type"] | "";

    if (strcmp(type, "hello") == 0)
    {
        handleHello(doc);
    }
    else if (strcmp(type, "request_definitions") == 0)
    {
        sendDefinitions();
    }
    else if (strcmp(type, "parameter") == 0)
    {
        handleParameter(doc);
    }
    else if (strcmp(type, "parameter_request") == 0)
    {
        handleParameterRequest(doc);
    }
    else if (strcmp(type, "command") == 0)
    {
        handleCommand(doc);
    }
    else
    {
        sendError("Unknown message type");
    }
}
```

Then:

```cpp
handleCommand(...)
```

can contain command dispatch.

And:

```cpp
handleParameter(...)
```

can contain parameter dispatch.

This keeps the communication layer separated from the actual robot logic.

---

# 44. Example Teensy Parameter Handler

```cpp
void handleParameter(JsonDocument &doc)
{
    const char *name = doc["name"] | "";

    if (strcmp(name, "drive.pid.kp") == 0)
    {
        float requested = doc["value"] | driveKp;

        driveKp = constrain(
            requested,
            0.0f,
            10.0f
        );

        sendParameterValue(
            "drive.pid.kp",
            driveKp
        );
    }

    else if (strcmp(name, "drive.pid.ki") == 0)
    {
        float requested = doc["value"] | driveKi;

        driveKi = constrain(
            requested,
            0.0f,
            5.0f
        );

        sendParameterValue(
            "drive.pid.ki",
            driveKi
        );
    }

    else
    {
        sendError(
            "Unknown parameter"
        );
    }
}
```

---

# 45. Example Teensy Command Handler

```cpp
void handleCommand(JsonDocument &doc)
{
    const char *command =
        doc["command"] | "";

    if (strcmp(command, "stop") == 0)
    {
        stopRobot();
        sendLog("WARNING", "Robot stopped");
    }

    else if (strcmp(command, "home_sorter") == 0)
    {
        if (!debugMode)
        {
            sendError(
                "home_sorter requires debug mode"
            );

            return;
        }

        homeSorter();
    }

    else if (strcmp(command, "drive_test") == 0)
    {
        if (!debugMode)
        {
            sendError(
                "drive_test requires debug mode"
            );

            return;
        }

        float speed =
            doc["speed"] | 0.0f;

        uint32_t duration =
            doc["duration_ms"] | 1000;

        speed = constrain(
            speed,
            -1.0f,
            1.0f
        );

        duration = constrain(
            duration,
            100UL,
            10000UL
        );

        startDriveTest(
            speed,
            duration
        );
    }
}
```

---

# 46. Example Telemetry Sender

A complete telemetry packet might be generated like:

```cpp
void sendTelemetry()
{
    StaticJsonDocument<1024> doc;

    doc["type"] = "telemetry";
    doc["time"] = millis();

    JsonObject data =
        doc.createNestedObject("data");

    data["power.battery_voltage"] =
        batteryVoltage;

    data["drive.left_rpm"] =
        leftRPM;

    data["drive.right_rpm"] =
        rightRPM;

    data["drive.pid.error"] =
        driveError;

    data["drive.pid.output"] =
        driveOutput;

    data["imu.heading"] =
        heading;

    serializeJson(doc, DebugSerial);
    DebugSerial.println();
}
```

---

# 47. Direct USB Testing vs Bluetooth Testing

The protocol is transport-independent.

For initial testing:

```text
Robot/ESP32
    │
    │ USB
    ▼
Computer
```

Select the USB serial COM port.

For wireless operation:

```text
Robot
   │
   │ UART
   ▼
CH9143
   )))
   ))) Bluetooth
   )))
CH9143
   │
   │ USB
   ▼
Computer
```

Select the Bluetooth receiver COM port.

The GUI behaves identically in both cases.

That makes direct USB extremely useful for debugging the debug system itself.

---

# 48. Current GUI Tabs

The current GUI contains:

```text
Dashboard
Plots
Parameters
Commands
Logs
Raw Serial
```

## Dashboard

Shows:

```text
live telemetry
commands
```

This is intended to be the normal operating/debugging screen.

---

## Plots

Allows any numeric telemetry signal to be selected and plotted in real time.

Multiple signals may be plotted simultaneously.

---

## Parameters

Automatically displays all parameters advertised by the robot.

The user can tune values without editing code.

---

## Commands

Displays all commands advertised by the robot.

This remains available even though commands are also shown on the Dashboard.

---

## Logs

Displays structured log messages from the robot.

---

## Raw Serial

Displays every incoming serial line.

Useful for low-level debugging.

---

# 49. Adding a New Telemetry Variable

Suppose a new current sensor is added to the robot.

You do **not** edit the GUI.

Simply add something like:

```cpp
data["intake.motor_current"] =
    intakeMotorCurrent;
```

to the robot telemetry packet.

The next time the GUI receives the packet:

```text
intake.motor_current
```

automatically appears.

If it is numeric, it also becomes plottable.

---

# 50. Adding a New Parameter

Suppose a threshold is added:

```cpp
float objectDetectionThreshold = 0.65f;
```

Advertise it:

```json
{
    "type": "parameter_definition",
    "name": "vision.object_threshold",
    "label": "Object Detection Threshold",
    "datatype": "float",
    "value": 0.65,
    "min": 0.0,
    "max": 1.0,
    "step": 0.01
}
```

Then add the robot-side handler.

The GUI automatically creates the editor.

No Python GUI changes are required.

---

# 51. Adding a New Command

Suppose you add:

```text
test_intake
```

The robot advertises:

```json
{
    "type": "command_definition",
    "name": "test_intake",
    "label": "Test Intake",
    "args": [
        {
            "name": "speed",
            "type": "float",
            "min": -1,
            "max": 1,
            "step": 0.05,
            "default": 0.5
        }
    ]
}
```

The GUI automatically creates the command.

When executed it sends:

```json
{
    "type": "command",
    "command": "test_intake",
    "speed": 0.5
}
```

The robot then runs its handler.

---

# 52. Recommended Development Philosophy

The GUI should remain generic.

The firmware should describe the robot.

In other words:

```text
Do not teach the GUI:
    "this robot has a sorting gate"

Teach the robot:
    "I have a command called home_sorter"
```

and:

```text
Do not teach the GUI:
    "this robot has a left wheel encoder"

Have the robot send:
    drive.left_rpm
```

This means the same GUI could theoretically be used with:

```text
Robot A
Robot B
test rig
motor controller
sensor test board
ESP32 simulator
future robot
```

without rewriting the application.

---

# 53. Suggested Future Protocol Features

The existing architecture can be extended later with:

```text
telemetry subscriptions
message IDs
command acknowledgements
parameter save/load
configuration profiles
recording telemetry to CSV
playback of previous sessions
automatic reconnect
heartbeat monitoring
fault history
named dashboards
custom gauges
signal units
signal definitions
plot presets
calibration commands
firmware version reporting
robot identification
protocol-version negotiation
```

The current `type`-based JSON structure leaves room for all of these.

---

# 54. Example Full Session

A typical session might look like this.

## Computer connects

Computer:

```json
{"type":"hello","client":"RobotDebugGUI","protocol":1}
```

Robot:

```json
{"type":"log","level":"INFO","message":"Debug GUI connected"}
```

Robot:

```json
{"type":"state","debug_mode":false,"fault":false}
```

Computer:

```json
{"type":"request_definitions"}
```

---

## Robot advertises parameter

Robot:

```json
{
    "type":"parameter_definition",
    "name":"drive.pid.kp",
    "label":"Drive PID - Kp",
    "datatype":"float",
    "value":1.2,
    "min":0,
    "max":10,
    "step":0.01
}
```

---

## Robot advertises command

Robot:

```json
{
    "type":"command_definition",
    "name":"drive_test",
    "label":"Drive Motor Test",
    "args":[
        {
            "name":"speed",
            "type":"float",
            "min":-1,
            "max":1,
            "default":0.3
        }
    ]
}
```

---

## Computer enters debug mode

Computer:

```json
{
    "type":"command",
    "command":"set_debug_mode",
    "enabled":true
}
```

Robot:

```json
{
    "type":"state",
    "debug_mode":true,
    "fault":false
}
```

---

## Robot sends telemetry

Robot:

```json
{
    "type":"telemetry",
    "time":54321,
    "data":{
        "power.battery_voltage":12.4,
        "drive.left_rpm":421,
        "drive.right_rpm":418,
        "imu.heading":84.2
    }
}
```

---

## User tunes Kp

Computer:

```json
{
    "type":"parameter",
    "name":"drive.pid.kp",
    "value":1.7
}
```

Robot changes:

```cpp
driveKp = 1.7f;
```

Robot confirms:

```json
{
    "type":"parameter_value",
    "name":"drive.pid.kp",
    "value":1.7
}
```

---

## User starts motor test

Computer:

```json
{
    "type":"command",
    "command":"drive_test",
    "speed":0.4,
    "duration_ms":2000
}
```

Robot runs test.

Robot:

```json
{
    "type":"log",
    "level":"INFO",
    "message":"Drive test started"
}
```

Robot telemetry changes accordingly.

---

## User stops robot

Computer:

```json
{
    "type":"command",
    "command":"stop"
}
```

Robot disables motion.

Robot:

```json
{
    "type":"log",
    "level":"WARNING",
    "message":"Robot STOP command received"
}
```

Robot:

```json
{
    "type":"state",
    "debug_mode":true,
    "stopped":true
}
```

That is the complete two-way communication loop.

---

# 55. Summary

The system is built around four layers:

```text
GUI
    ↓
Python communication backend
    ↓
serial/Bluetooth transport
    ↓
robot debug protocol
```

The robot sends information upward using:

```text
telemetry
parameter definitions
parameter values
command definitions
logs
state
errors
```

The computer sends information downward using:

```text
commands
parameter changes
parameter requests
definition requests
debug-mode commands
```

The most important design feature is that the GUI is **data-driven**.

The robot describes what it can do, and the GUI adapts.

This means adding new telemetry, parameters, and debug commands should usually require changes only to the robot firmware, not to the Python application.

The resulting workflow is intended to be:

```text
write robot feature
      ↓
advertise telemetry / parameters / commands
      ↓
flash firmware
      ↓
open GUI
      ↓
connect wirelessly
      ↓
observe
      ↓
command
      ↓
tune
      ↓
test
```

without repeatedly editing Python files simply to expose another variable.

---

# 56. Current Protocol Quick Reference

## Computer → Robot

### Hello

```json
{"type":"hello","client":"RobotDebugGUI","protocol":1}
```

### Request definitions

```json
{"type":"request_definitions"}
```

### Enter debug mode

```json
{"type":"command","command":"set_debug_mode","enabled":true}
```

### Exit debug mode

```json
{"type":"command","command":"set_debug_mode","enabled":false}
```

### Command

```json
{"type":"command","command":"stop"}
```

### Command with arguments

```json
{"type":"command","command":"drive_test","speed":0.3,"duration_ms":2000}
```

### Set parameter

```json
{"type":"parameter","name":"drive.pid.kp","value":1.5}
```

### Request parameter

```json
{"type":"parameter_request","name":"drive.pid.kp"}
```

---

## Robot → Computer

### Telemetry

```json
{"type":"telemetry","name":"battery.voltage","value":12.4}
```

### Grouped telemetry

```json
{
    "type":"telemetry",
    "time":12345,
    "data":{
        "drive.left_rpm":420,
        "drive.right_rpm":418
    }
}
```

### Parameter definition

```json
{
    "type":"parameter_definition",
    "name":"drive.pid.kp",
    "datatype":"float",
    "value":1.2,
    "min":0,
    "max":10,
    "step":0.01
}
```

### Parameter value

```json
{"type":"parameter_value","name":"drive.pid.kp","value":1.2}
```

### Command definition

```json
{
    "type":"command_definition",
    "name":"stop",
    "label":"Stop Robot",
    "args":[]
}
```

### Log

```json
{"type":"log","level":"INFO","message":"Robot ready"}
```

### State

```json
{"type":"state","debug_mode":true,"fault":false}
```

### Error

```json
{"type":"error","message":"Unknown command"}
```

---

# 57. File Responsibilities

## `DebugGUI.py`

Responsible for:

```text
windows
buttons
tables
plots
parameter editors
command controls
COM-port selector
user interaction
displaying telemetry
displaying logs
```

It should not contain robot-specific serial parsing logic.

---

## `BluetoothSerial.py`

Responsible for:

```text
opening the COM port
closing the COM port
serial worker thread
transmit queue
receiving lines
JSON parsing
JSON generation
routing message types
emitting PyQt signals
```

It should not contain robot-specific GUI layout logic.

---

## Robot Firmware

Responsible for:

```text
reading UART
assembling newline-delimited messages
parsing JSON
validating parameters
executing commands
enforcing debug permissions
sending telemetry
advertising parameters
advertising commands
sending logs
sending state
ensuring actuator safety
```

This separation keeps the project maintainable as the robot grows.

---

# 58. Final Design Principle

The intended rule for this project is:

> **The GUI should not need to be rewritten every time the robot changes.**

When you add something to the robot:

```text
new sensor
new PID controller
new actuator
new servo
new state
new command
new threshold
new test
```

the firmware should expose it through the protocol.

The GUI should then discover and display it automatically.

That is what makes this debug system useful as a long-term robotics development tool rather than just a one-off serial monitor.
