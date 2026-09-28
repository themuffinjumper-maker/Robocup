"""Non-actuating CH9143/Teensy protocol handshake test."""

from __future__ import annotations

import argparse
import json
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="CH9143 COM port, for example COM10")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()

    valid_messages: list[dict] = []
    invalid_lines: list[str] = []

    with serial.Serial(args.port, args.baud, timeout=0.25) as link:
        link.reset_input_buffer()
        # Telemetry is continuous, so discard one line to synchronize if the
        # port was opened halfway through a packet.
        link.readline()
        hello = (json.dumps(
            {"type": "hello", "client": "smoke-test", "protocol": 1},
            separators=(",", ":"),
        ) + "\n").encode()
        deadline = time.monotonic() + args.seconds
        next_hello = 0.0
        next_heartbeat = 0.0
        handshake_seen = False
        receive_buffer = bytearray()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if not handshake_seen and now >= next_hello:
                link.write(hello)
                next_hello = now + 2.0
            elif handshake_seen and now >= next_heartbeat:
                link.write(b'{"type":"heartbeat"}\n')
                next_heartbeat = now + 3.0
            chunk = link.read(link.in_waiting or 1)
            if not chunk:
                continue
            receive_buffer.extend(chunk)
            while b"\n" in receive_buffer:
                raw_line, _, remainder = receive_buffer.partition(b"\n")
                receive_buffer = bytearray(remainder)
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                    if isinstance(message, dict):
                        valid_messages.append(message)
                        if message.get("type") != "link_probe":
                            handshake_seen = True
                    else:
                        invalid_lines.append(line)
                except json.JSONDecodeError:
                    invalid_lines.append(line)

    message_types = {message.get("type") for message in valid_messages}
    for message in valid_messages[:10]:
        print(json.dumps(message, separators=(",", ":")))

    required = {"log", "state", "parameter_definition", "command_definition", "telemetry"}
    missing = required - message_types
    print(f"Valid JSON messages: {len(valid_messages)}; invalid lines: {len(invalid_lines)}")
    if missing:
        print("FAIL: missing message types: " + ", ".join(sorted(missing)))
        return 1

    print("PASS: Bluetooth handshake, definitions, state, and telemetry received")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
