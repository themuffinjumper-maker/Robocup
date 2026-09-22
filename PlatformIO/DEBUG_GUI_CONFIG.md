# Debug GUI configuration

Edit `debug_config.json` to add or remove controls in the desktop GUI. Building
the PlatformIO project validates the JSON and embeds it in the Teensy firmware.
Do not edit `include/debug_config.generated.h`; it is generated automatically.

The desktop GUI also reads this same file directly at startup. This guarantees
that controls appear even if the wireless serial link drops a definition frame;
definitions received from the Teensy still update the existing controls.

## Parameters

Add an object to the `parameters` array:

```json
{
  "name": "drive.pid.kp",
  "label": "Drive Kp",
  "description": "Proportional speed-controller gain",
  "datatype": "float",
  "value": 1.2,
  "min": 0.0,
  "max": 10.0,
  "step": 0.01
}
```

The library automatically advertises the parameter, accepts updates, clamps
numeric values to `min`/`max`, stores the current value in RAM, and confirms the
accepted value to the GUI. Removing the object removes its GUI control.

The special parameter `debug.telemetry_interval_ms` also controls the actual
telemetry interval. Other parameters are available through the workflow's JSON
registry; connecting them to new robot algorithms requires a matching C++ use.

## Digital inputs

Add switches and proximity/presence sensors to the top-level `digital_inputs`
array. The workflow configures, debounces and publishes them without changing
`main.cpp`:

```json
"digital_inputs": [
  {
    "name": "inductive_proximity",
    "label": "Inductive proximity sensor",
    "pin": 21,
    "active_low": true,
    "pullup": true,
    "debounce_ms": 20
  }
]
```

Names and pins must be unique; up to eight inputs are supported. Each publishes
`digital.<name>.detected`, `digital.<name>.raw_high`, and
`digital.<name>.transitions`. The current D21 input is documented in
`INDUCTIVE_PROXIMITY_WIRING.md`.

## Ultrasound sensors

Trigger/echo ultrasound boards are listed in `ultrasound_sensors`:

```json
"ultrasound_sensors": [
  {
    "name": "a",
    "label": "Ultrasound A",
    "trigger_pin": 14,
    "echo_pin": 24,
    "interval_ms": 100,
    "timeout_us": 30000
  },
  {
    "name": "b",
    "label": "Ultrasound B",
    "trigger_pin": 22,
    "echo_pin": 20,
    "interval_ms": 100,
    "timeout_us": 30000
  }
]
```

The non-blocking driver publishes `ultrasound.<name>.valid`, `.timed_out`,
`.echo_us`, and `.distance_mm`. Up to two sensors are supported for the A/B
interface board. The workflow pings them sequentially so one receiver cannot
mistake the other transmitter's pulse for its own. Distance is omitted when the
current measurement is not valid. Voltage requirements are documented in
`ULTRASOUND_WIRING.md`.

## TOF sensors

Add each VL53 TOF sensor to the top-level `tof_sensors` array. No change to
`main.cpp` or the desktop GUI is required:

```json
"tof_sensors": [
  {"name": "front", "label": "Front TOF", "type": "long", "port": "XSHUT1"},
  {"name": "left", "label": "Left TOF", "type": "short", "port": "XSHUT2"}
]
```

- `name` is a unique short identifier used in telemetry names.
- `label` documents the physical sensor location.
- `type` is `long` for VL53L1X or `short` for VL53L0X.
- `port` is `XSHUT0` through `XSHUT7` (the integers 0-7 are also accepted).

Names and ports must be unique, and at most eight sensors can be configured.
The firmware holds all XSHUT lines low, starts listed sensors one at a time,
and assigns address `0x30 + XSHUT number`. Each entry appears separately in
Live Telemetry as `tof.<name>.available`, `tof.<name>.timed_out`, and
`tof.<name>.distance_mm`. Unlisted sensors remain shut down.

## Commands

### Autonomous navigation

The `autonomous_navigation` command creates one Navigation card with RUN and
STOP buttons. RUN requires Debug Mode and directly enters the robot run state.
The Teensy first drives straight to the wall ahead, turns right, follows that
wall with the left ultrasound at a 200 mm target until the corner, then covers
the arena in ten parallel lanes spaced 250 mm apart. The wall follower applies
a proportional correction whenever the ultrasound moves outside a 10 mm
deadband, then returns to exact IMU heading hold. At the start of every
sweep lane, the controller captures both side-ultrasound distances and steers
to preserve those lane-specific values within 15 mm; inside that band the IMU
holds the exact orthogonal heading. The two top point TOFs and the SEN0628 form
a robust median forward range with 300 mm clearance. All `bottom_*` TOFs are
excluded from navigation and remain available for weight sensing and mapping.
Ultrasound A/B provide left/right clearance with a 200 mm limit. STOP ROBOT,
leaving Debug Mode, keyboard drive, and manual motor commands all take ownership
away from navigation and neutralise bank 2. Encoder/IMU agreement is exposed as
diagnostic telemetry and does not cancel navigation.
The BNO055 holds headings and measures each 90-degree turn; calibrated encoders
measure each lane shift. All autonomous movement commands respect the
drivetrain's 75 percent minimum.

Add an object to the `commands` array. The `action` selects firmware behaviour.
The optional `category` is displayed as a badge inside the command card, for
example `System`, `Herkulex`, `HX12K`, or `203 DC Motor`:

- `ping`: responds with a log and state.
- `stop`: disables all Herkulex torque, sends both 203 DC motor drivers neutral, and
  reports stopped state.
- `run`: leaves stopped mode without commanding motion or enabling torque.
- `herkulex_angle`: validates and sends a Herkulex angle command.
- `herkulex_read_angle`: requests position feedback and publishes the measured
  angle as `servo.measured_angle_deg` telemetry. After the first successful
  manual read, the workflow continues reading that servo every 200 ms.
- `herkulex_zero_here`: reads the current position without moving, stores it as
  a temporary software zero, and makes later `herkulex_angle` targets relative
  to that point. The zero is cleared by a Teensy reset or power cycle.
- `herkulex_velocity`: switches a DRS-0101 into continuous-turn mode and applies
  signed speed from -1023 to +1023. It requires Debug Mode and Run state.
- `herkulex_stop_velocity`: commands zero velocity and disables that servo's
  torque. Use it before setting zero or returning to position control.
- `dc_motor_203_speed`: applies independent signed -100% to +100% commands to
  the first 203 DC motor driver's channels A/B on D27/D26. It
  requires Debug Mode and Run.
- `dc_motor_203_stop`: immediately writes the 1500 us neutral pulse to both
  channels of the first driver.
- `dc_motor_203_second_speed`: applies the same independent channel control to
  the second driver's channels A/B on D25/D15. It requires Debug Mode and Run.
- `dc_motor_203_second_stop`: writes neutral to both second-driver channels.
  Global Stop and leaving Debug Mode stop both drivers.
  D15 is also Teensy A1, which is reserved as `IR_RIGHT` in the placeholder
  sensor configuration; that IR input cannot be used at the same time.
- `encoder_zero`: resets both Digital Raw 2 quadrature counts without moving
  anything. The automatic telemetry fields are `encoder.1.count`,
  `encoder.1.delta`, `encoder.1.counts_per_s` and the matching `encoder.2.*`
  fields. See `ENCODER_WIRING.md` for D2-D5 wiring and voltage precautions.
- `hx12k_angles`: accepts independent selection flags and 0-135 degree targets
  for level-shifter outputs A-D. Only selected outputs are changed. It requires
  Debug Mode and Run.
- `hx12k_disable`: detaches the PWM output. Global STOP and leaving Debug Mode
  also detach all four outputs.
- `hx12k_bumpers`: moves the paired bumper outputs together. `enabled=true`
  commands C=0 degrees and D=130 degrees; `enabled=false` commands C=130
  degrees and D=0 degrees. It requires Debug Mode and Run state.
- `tof_read_all`: immediately reads every configured TOF sensor. All sensors
  are also read automatically with normal telemetry packets.

Example:

```json
{
  "name": "set_herkulex_angle",
  "label": "Set Herkulex angle",
  "action": "herkulex_angle",
  "debug_mode_required": true,
  "args": [
    {"name": "id", "type": "int", "min": 1, "max": 253, "default": 1},
    {"name": "angle_deg", "type": "float", "min": -160, "max": 160, "step": 1, "default": 0},
    {"name": "move_time_ms", "type": "int", "min": 50, "max": 2850, "step": 50, "default": 500}
  ]
}
```

A command can replace the single Run button with multiple fixed-argument
buttons inside the same card. Each item in `buttons` supplies its label and
arguments; the GUI merges those fixed values with any ordinary argument
editors before sending the command:

```json
{
  "name": "set_bumper_servos",
  "label": "Bumper servos",
  "action": "hx12k_bumpers",
  "debug_mode_required": true,
  "args": [],
  "buttons": [
    {"label": "BUMPERS ON", "arguments": {"enabled": true}},
    {"label": "BUMPERS OFF", "arguments": {"enabled": false}}
  ]
}
```

Removing a command object removes its GUI button. A new `action` name also
needs a corresponding implementation in `BluetoothDebugWorkflow.cpp`; JSON can
describe controls and select existing actions, but cannot create new physical
robot behaviour by itself.

## Main program

`src/main.cpp` intentionally contains only construction plus `begin()` and
`update()`. Normal robot code can run beside `debugGui.update()` later without
moving protocol or GUI-definition code back into `main.cpp`.
