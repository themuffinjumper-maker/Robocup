# Dual ultrasound interface test mapping

The configured front ultrasound board uses the same trigger/echo method as the
supplied `110_UltrasoundDigital` example, with the robot's actual pins:

| Interface channel | Ultrasound signal | CPU pin |
|---|---|---|
| A | TRIG | D14 |
| A | ECHO | D24 |
| B | TRIG | D22 |
| B | ECHO | D20 |
| Both | Ground | GND |

Both interface-board sockets are enabled through the 8-pin CPU cable. This test
restores the original D24/D14 and D20/D22 pair grouping but reverses the signal
direction inside each pair: A is TRIG D14 / ECHO D24 and B is TRIG D22 / ECHO
D20. The workflow alternates their pings to prevent acoustic cross-talk.

The firmware sends a 10 us trigger pulse every 100 ms, waits without blocking
the Bluetooth/actuator loop, and times out after 30 ms. Echo duration is
converted with approximately 0.343 mm/us and divided by two for the round trip.
Measurements from 20–5000 mm are accepted.

Teensy 4.0 pins are **3.3 V only**. The CPU-side ECHO signal must never exceed
3.3 V. If the board outputs a 5 V echo, use the carrier's conditioned connection
or a voltage divider/level shifter. D14 is also A0, so the placeholder left IR
input cannot be used while this ultrasound echo is connected.

The GUI and recordings receive:

- `ultrasound.a.valid`
- `ultrasound.a.timed_out`
- `ultrasound.a.echo_us`
- `ultrasound.a.distance_mm` when valid

Channel B uses the same four fields under `ultrasound.b.*`.

Diagnostic firmware additionally publishes raw 12-bit ADC and interrupt data:
`echo_adc`, `echo_voltage_v`, per-ping minimum/maximum ADC and voltage,
`trigger_low_adc`, `trigger_high_adc`, `trigger_high_voltage_v`, `echo_high`,
and trigger/rise/fall counters. These fields separate weak electrical levels
from missing or missed echo pulses.
