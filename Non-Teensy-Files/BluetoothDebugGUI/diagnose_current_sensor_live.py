"""Short, automatically stopped D23/current-sensor test over the CH9143 link.

Usage: python diagnose_current_sensor_live.py [COM10] [--move]
Without --move this only listens. With --move it briefly commands bank 1
channels A and B together at the same percentage and direction, stopping
the bank between phases and on exit.
"""

import argparse
import json
import statistics
import time

import serial


def send(port, command, **arguments):
    message = {"type": "command", "command": command, **arguments}
    port.write((json.dumps(message, separators=(",", ":")) + "\n").encode())
    port.flush()


def sample(port, seconds, label):
    deadline = time.monotonic() + seconds
    frames = []
    events = []
    while time.monotonic() < deadline:
        line = port.readline()
        if not line:
            continue
        try:
            message = json.loads(line)
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        if message.get("type") == "telemetry":
            data = message.get("data", {})
            if "current_sensor.d23.raw_adc" in data:
                frames.append(data)
        elif message.get("type") in ("log", "error", "state"):
            events.append(message)
    print(f"{label}: {len(frames)} current-sensor frames")
    for key in (
        "current_sensor.d23.raw_adc",
        "current_sensor.d23.adc1_first_raw",
        "current_sensor.d23.adc1_repeat_raw",
        "current_sensor.d23.adc2_raw",
        "current_sensor.d23.pin_volts",
        "current_sensor.d23.current_a",
        "dc_motor_203.channel_a_percent",
        "dc_motor_203.channel_b_percent",
    ):
        values = [frame[key] for frame in frames if isinstance(frame.get(key), (int, float))]
        if values:
            print(f"  {key}: median={statistics.median(values):.3f}, "
                  f"min={min(values):.3f}, max={max(values):.3f}")
    for event in events[-5:]:
        print("  event:", event)
    return frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="COM10")
    parser.add_argument("--move", action="store_true")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=0.25, write_timeout=1) as port:
        try:
            baseline = sample(port, 3, "baseline")
            if not args.move:
                return
            if not baseline:
                raise RuntimeError("No current-sensor telemetry; refusing to run motors")
            latest = baseline[-1]
            if latest.get("navigation.active") or latest.get("dc_motor_203_second.active"):
                raise RuntimeError("Navigation or bank 2 active; refusing to interfere")
            if latest.get("system.stopped"):
                send(port, "set_debug_mode", enabled=True)
                send(port, "run")
                sample(port, 1.5, "run-state confirmation")
            for percent in (50, 75):
                label = f"A+B {percent}%"
                send(port, "set_dc_motor_203_speed", speed_percent=percent)
                moving = sample(port, 3, label)
                if not any(
                    frame.get("dc_motor_203.channel_a_percent") == percent
                    and frame.get("dc_motor_203.channel_b_percent") == percent
                    for frame in moving
                ):
                    raise RuntimeError(f"Bank 1 did not confirm {label}")
                send(port, "stop_dc_motor_203")
                sample(port, 1.5, "stopped")
        finally:
            if args.move:
                for _ in range(3):
                    try:
                        send(port, "stop_dc_motor_203")
                    except serial.SerialException:
                        break
                    time.sleep(0.1)
                sample(port, 1, "final bank-1 stop confirmation")


if __name__ == "__main__":
    main()
