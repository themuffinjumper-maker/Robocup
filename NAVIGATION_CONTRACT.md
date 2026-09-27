# Navigation contract

This document is the single reference for navigation behaviour. Change a
policy only when the associated physical assumption changes, then update the
firmware and Arena View together.

## Geometry

- Arena: 4900 mm long by 2400 mm wide.
- Drive width: 290 mm. This is the dimension used to decide whether a forward
  gap can be driven through.
- Drive length: 430 mm. This is used to interpret a forward sensor range from
  the robot centre; turns are only started from route/map-checked staging
  points rather than assuming the width is the length.
- All forward-ranging sensors are modelled at the front of the robot. Their
  configured positions in the Wiring Guide provide their lateral offsets and
  ray angles.
- A gap is passable only when the 8x8 TOF estimates a contiguous opening at
  least `robot width + extra required gap width`. The controller aims at the
  centre of that opening, not an edge.

## Adjustable values

The Arena View **Navigation tuning** tab and the normal Parameters tab expose
the values that should be changed during testing:

| Control | Meaning |
| --- | --- |
| Front wall stop | Normal forward stop distance. It has a fixed 70 mm physical-contact floor. |
| Wall clearance (edge) | Gap from the outer chassis edge to a mapped wall. |
| Dummy clearance (edge) | Gap from the outer chassis edge to a dummy weight. |
| Robot width / length | Measured chassis dimensions, in millimetres. |
| Extra gap width | Additional total opening width required beyond the 290 mm chassis width. |
| Route line tolerance | Maximum lateral error before a transit waypoint may be skipped. |
| Weight search radius | Radius where TOF evidence begins to influence the planned line. |
| Weight approach clearance | Forward clearance required for controlled weight approach/follow-through. |

There are no hidden desktop or firmware map-clearance additions. A planned
route and an on-robot reroute use the displayed wall/dummy values.

## Sensor authority and checkpoints

1. Outside a weight-search radius, the uploaded route, IMU heading and
   encoder pose are authoritative.
2. Inside the radius, a valid low-object TOF hit receives a quadratic
   authority increase as the robot approaches the planned weight. The sensor
   is dominant at the site. An outer TOF is deliberately discounted because it
   sits outside the intake.
3. A single outer/inner return can turn the robot toward a likely weight and,
   when needed, reverse it to make room. It can never trigger the final
   drive-through.
4. The two inward bottom TOFs must independently identify a low object and
   agree on range before centring and collection. This pair is also a bounded,
   rate-limited localisation checkpoint against the current planned weight.
   It can correct encoder/IMU drift by at most 60 mm per correction and never
   applies to a dummy or an unconfirmed return.
5. Side ultrasounds and top TOFs continue to make separate gradual corrections
   against mapped walls. No single range return may teleport the pose.

## Code ownership

- `PlatformIO/lib/BluetoothDebugWorkflow/`: navigation state machine,
  localisation, sensor fusion, telemetry and command protocol.
- `PlatformIO/debug_config.json`: firmware parameter definitions. The build
  generates `include/debug_config.generated.h`; never edit that generated file
  directly.
- `Non-Teensy-Files/BluetoothDebugGUI/ArenaView.py`: layout editor, desktop
  route planner and the operator-facing tuning controls.
- `Non-Teensy-Files/BluetoothDebugGUI/WiringGuide.py`: persistent sensor
  geometry. Edit placements there rather than adding sensor offsets to
  navigation code.

Motor, Bluetooth, IMU, TOF and servo libraries remain separate device drivers.
Navigation policy belongs in the workflow layer, not in those drivers.
