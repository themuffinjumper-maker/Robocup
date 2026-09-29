# Local competition run

The local run reuses the existing pre-laid mission navigation and home unload
sequence. `LocalCompetition` handles the AIO9 rising edge, 90-second clock, and
confirmed-weight limit; `BluetoothDebugWorkflow` remains the controller for
navigation, encoders/IMU, TOFs, arm sorting, motors, and storage-gate unloading.

## Prepare and export the arena

1. Open the desktop Debug Console and use **Arena View → Mission Planner**.
2. Set the home, start point and heading. Add the real weight sites, dummy
   weights, walls, ramps and tubes. Rotate walls and set sensor placements and
   headings in the existing layout controls as needed.
3. Press **PLAN ROUTE** and correct any route error.
4. Press **EXPORT LOCAL FIRMWARE CONFIG**. This writes
   `PlatformIO/local_mission.json` in this project. The export contains the
   planned route, map features (including rotated walls), point-TOF and
   ultrasound geometry, layout metadata, and navigation tuning. Drum jam
   parameters are intentionally not copied into this arena configuration.

## Build and flash

Build/upload the `teensy40` PlatformIO environment after exporting. The
pre-build script embeds `local_mission.json` into
`PlatformIO/include/local_mission.generated.h`; rebuild after every arena or
tuning change. No Bluetooth command is needed to upload the mission.

## Start and round sequence

- Power-up initializes hardware and sensors but does not start motors or
  navigation.
- In the local profile, AIO9 is configured with an SX1509 internal pulldown.
  A LOW-to-HIGH transition, held for 30 ms, starts the run. Wire the button so
  it connects AIO9 to the board's **3.3 V** when pressed; do not apply 5 V.
  AIO9 is SX1509 I/O 9 on I2C0, not raw Teensy D9.
- The run arms the existing sorting mechanism, starts motor bank 1 at 100%,
  and starts the exported planned mission. The storage-release gate is handled
  by the existing navigation/home logic.
- At 90 seconds from the AIO9 edge, or after five inductive weight
  confirmations during the run, the supervisor marks remaining weight targets
  skipped and hands control to the normal map-aware return-home planner. It
  performs the existing reverse dock, gate-open and shake sequence. After
  unloading it closes the storage gate and stops the run. The live telemetry
  reports `local_competition.confirmed_weights` and its limit.

## Bluetooth monitor-only mode

The desktop GUI starts in **Monitor only — no command TX** mode: Parameters and
Commands are hidden, arena-page actuation controls are hidden, and command,
parameter and keyboard-drive sends are blocked in the GUI code. Telemetry,
recording, sensor views and the arena display remain available. The firmware
also rejects incoming Bluetooth command/drive/parameter packets whenever an
enabled local mission was embedded, so stale GUI packets cannot steer it.

To temporarily disable the embedded local profile, set `"enabled": false` in
`PlatformIO/local_mission.json` and rebuild. The GUI export button writes an
enabled competition profile again.
