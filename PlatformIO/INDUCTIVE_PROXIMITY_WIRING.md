# D21 inductive proximity sensor

The firmware reads the inductive proximity sensor on Teensy pin **D21**. Its
default JSON settings assume an active-low NPN/open-collector output and enable
the Teensy's internal pull-up. The input is debounced for 20 ms.

## CPU-side connection

| Sensor/interface | CPU |
|---|---|
| Conditioned digital output | D21 |
| Logic ground | GND |

Teensy 4.0 inputs are **3.3 V only and are not 5 V tolerant**. Do not connect a
raw 5 V, 12 V or 24 V industrial proximity-sensor output directly to D21. Use
the supplied conditioned cable/interface, an optoisolator, or a correctly sized
level shifter so the D21 signal remains between 0 V and 3.3 V. The sensor power
wiring depends on the exact sensor and interface and must follow its datasheet.

## Telemetry

The GUI automatically shows:

- `digital.inductive_proximity.detected` — debounced logical detection.
- `digital.inductive_proximity.raw_high` — debounced electrical pin level.
- `digital.inductive_proximity.transitions` — number of accepted level changes
  since the Teensy booted.

With the default active-low setting, an idle/open input reads raw high and
detected false; pulling D21 safely to ground reads raw low and detected true.
If the supplied interface is active-high, set `active_low` to `false` in
`debug_config.json`, rebuild, and upload.
