"""
BluetoothSerial.py

Generic serial/Bluetooth communications backend for the robot debug GUI.

Protocol:
    Newline-delimited JSON.

Examples from robot:

    {"type":"telemetry","name":"battery.voltage","value":12.4}

    {"type":"telemetry","time":12345,"data":{
        "battery.voltage":12.4,
        "drive.left_rpm":430,
        "drive.right_rpm":425
    }}

    {"type":"parameter_definition",
     "name":"drive.pid.kp",
     "label":"Drive Kp",
     "datatype":"float",
     "value":1.2,
     "min":0.0,
     "max":10.0,
     "step":0.01}

    {"type":"parameter_value","name":"drive.pid.kp","value":1.2}

    {"type":"command_definition",
     "name":"drive_test",
     "label":"Drive Test",
     "args":[
        {"name":"speed","type":"float","min":-1,"max":1,"step":0.05,"default":0.3},
        {"name":"duration_ms","type":"int","min":100,"max":10000,"step":100,"default":1000}
     ]}

    {"type":"log","level":"INFO","message":"Robot ready"}

Messages sent from PC:

    {"type":"hello"}
    {"type":"request_definitions"}

    {"type":"command","command":"stop"}

    {"type":"parameter","name":"drive.pid.kp","value":1.5}
"""

from __future__ import annotations

import json
import queue
import threading
import time
from typing import Any

import serial
from serial.tools import list_ports

from PyQt6.QtCore import QObject, QThread, pyqtSignal


# =====================================================================
# Serial worker
# =====================================================================

class SerialWorker(QThread):
    """
    Background serial thread.

    All serial reads and writes happen in this thread so the PyQt GUI
    never freezes waiting for serial data.
    """

    connected = pyqtSignal(str)
    disconnected = pyqtSignal()
    link_recovery = pyqtSignal(str)

    raw_received = pyqtSignal(str)
    message_received = pyqtSignal(dict)

    error = pyqtSignal(str)

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        parent: QObject | None = None,
    ):
        super().__init__(parent)

        self.port = port
        self.baudrate = baudrate

        self._serial: serial.Serial | None = None
        self._stop_event = threading.Event()

        self._tx_queue: queue.Queue[str] = queue.Queue()
        self._rx_buffer = bytearray()
        self._last_robot_message: float | None = None
        self._last_hello_sent = 0.0
        self._last_heartbeat_sent = 0.0
        self._last_silence_recovery = 0.0

    # -----------------------------------------------------------------

    def run(self):
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.05,
                write_timeout=0.5,
            )

        except Exception as exc:
            self.error.emit(f"Could not open {self.port}: {exc}")
            self.disconnected.emit()
            return

        self._last_hello_sent = time.monotonic()
        self.connected.emit(self.port)

        try:
            while not self._stop_event.is_set():

                self._process_transmit_queue()
                self._process_receive()
                self._maintain_link()
                time.sleep(0.005)

        except Exception as exc:
            if not self._stop_event.is_set():
                self.error.emit(f"Serial error: {exc}")

        finally:
            try:
                if self._serial is not None and self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass

            self._serial = None
            self.disconnected.emit()

    # -----------------------------------------------------------------

    def _process_transmit_queue(self):
        if self._serial is None:
            return

        try:
            text = self._tx_queue.get_nowait()
        except queue.Empty:
            return

        try:
            payload = text.encode("utf-8")
            offset = 0
            while offset < len(payload):
                # Pace longer commands through the BLE bridge.
                chunk = payload[offset:offset + 20]
                written = self._serial.write(chunk)
                if written is None or written <= 0:
                    raise serial.SerialTimeoutException(
                        "Serial write made no progress"
                    )
                offset += written
                if offset < len(payload):
                    time.sleep(0.003)
            self._serial.flush()
        except Exception as exc:
            self.error.emit(f"Transmit error: {exc}")

    # -----------------------------------------------------------------

    def _process_receive(self):
        if self._serial is None:
            return

        try:
            if self._serial.in_waiting <= 0:
                return

            chunk = self._serial.read(self._serial.in_waiting)
            if not chunk:
                return
            self._rx_buffer.extend(chunk)

            while b"\n" in self._rx_buffer:
                raw, _, remainder = self._rx_buffer.partition(b"\n")
                self._rx_buffer = bytearray(remainder)
                text = raw.decode("utf-8", errors="replace").strip("\r ")
                if not text:
                    continue
                self.raw_received.emit(text)
                try:
                    message = json.loads(text)
                    if isinstance(message, dict):
                        if message.get("type") != "link_probe":
                            self._last_robot_message = time.monotonic()
                        self.message_received.emit(message)
                except json.JSONDecodeError:
                    # Complete non-JSON lines remain visible in Raw Serial.
                    pass

            # Prevent an unplugged/corrupt sender from growing memory forever.
            if len(self._rx_buffer) > 65536:
                self.error.emit("Incoming serial line exceeded 65536 bytes")
                self._rx_buffer.clear()

        except serial.SerialException as exc:
            self.error.emit(str(exc))
            self._stop_event.set()

    def _maintain_link(self):
        """Retry a lost handshake and keep an established radio session alive."""
        now = time.monotonic()
        if (self._last_robot_message is not None and
                now - self._last_robot_message >= 8.0 and
                now - self._last_silence_recovery >= 8.0):
            # A paired CH9143 can leave its USB COM port open while its serial
            # stream has stopped. Reopening the same port clears that stale
            # bridge session; discard queued commands so an old drive command
            # can never be replayed when the radio comes back.
            self._last_silence_recovery = now
            self.link_recovery.emit(
                f"No robot data for 8 seconds; reopening {self.port}"
            )
            while True:
                try:
                    self._tx_queue.get_nowait()
                except queue.Empty:
                    break
            try:
                if self._serial is not None and self._serial.is_open:
                    self._serial.close()
                self._rx_buffer.clear()
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.05,
                    write_timeout=0.5,
                )
                self._last_hello_sent = 0.0
                self._last_heartbeat_sent = 0.0
            except Exception as exc:
                self._serial = None
                self.error.emit(f"Could not reopen {self.port}: {exc}")

        if self._last_robot_message is None or now - self._last_robot_message > 3.0:
            if now - self._last_hello_sent >= 2.0:
                self.send_text(json.dumps({
                    "type": "hello", "client": "RobotDebugGUI", "protocol": 1,
                }, separators=(",", ":")))
                self._last_hello_sent = now
        elif now - self._last_heartbeat_sent >= 3.0:
            self.send_text('{"type":"heartbeat"}')
            self._last_heartbeat_sent = now

    # -----------------------------------------------------------------

    def send_text(self, text: str):
        """
        Thread-safe transmission request.
        """

        if not text.endswith("\n"):
            text += "\n"

        self._tx_queue.put(text)

    # -----------------------------------------------------------------

    def stop(self):
        self._stop_event.set()


# =====================================================================
# Main communications manager
# =====================================================================

class BluetoothSerial(QObject):
    """
    Higher-level communications interface used by DebugGUI.py.
    """

    connection_changed = pyqtSignal(bool, str)

    telemetry_received = pyqtSignal(str, object, object)
    tof_8x8_received = pyqtSignal(dict)

    parameter_definition_received = pyqtSignal(dict)
    parameter_value_received = pyqtSignal(str, object)

    command_definition_received = pyqtSignal(dict)

    log_received = pyqtSignal(str, str)

    raw_received = pyqtSignal(str)

    message_received = pyqtSignal(dict)

    error_received = pyqtSignal(str)

    robot_state_received = pyqtSignal(dict)
    link_recovery = pyqtSignal(str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)

        self.worker: SerialWorker | None = None

        self.port: str | None = None
        self.baudrate: int = 115200

    # =================================================================
    # Port discovery
    # =================================================================

    @staticmethod
    def available_ports() -> list[dict]:
        """
        Return all currently available serial ports.
        """

        ports = []

        for port in list_ports.comports():
            ports.append(
                {
                    "device": port.device,
                    "description": port.description or "",
                    "manufacturer": port.manufacturer or "",
                    "hwid": port.hwid or "",
                }
            )

        ports.sort(key=lambda x: x["device"])

        return ports

    # =================================================================
    # Connection
    # =================================================================

    def connect_port(
        self,
        port: str,
        baudrate: int = 115200,
    ):
        if self.worker is not None:
            self.disconnect_port()

        self.port = port
        self.baudrate = baudrate

        self.worker = SerialWorker(
            port,
            baudrate,
        )

        self.worker.connected.connect(
            self._on_connected
        )

        self.worker.disconnected.connect(
            self._on_disconnected
        )

        self.worker.link_recovery.connect(self.link_recovery)

        self.worker.raw_received.connect(
            self.raw_received
        )

        self.worker.message_received.connect(
            self._handle_message
        )

        self.worker.error.connect(
            self.error_received
        )

        self.worker.start()

    # -----------------------------------------------------------------

    def disconnect_port(self):
        if self.worker is None:
            return

        worker = self.worker

        worker.stop()
        worker.wait(1000)

        self.worker = None

    # -----------------------------------------------------------------

    def is_connected(self) -> bool:
        return (
            self.worker is not None
            and self.worker.isRunning()
        )

    # -----------------------------------------------------------------

    def _on_connected(self, port: str):
        self.connection_changed.emit(
            True,
            port,
        )

        # Introduce the GUI to the robot.
        self.send_message(
            {
                "type": "hello",
                "client": "RobotDebugGUI",
                "protocol": 1,
            }
        )

        # The hello handler already returns all definitions. Sending a second
        # request here made the Teensy perform the entire definition broadcast
        # twice while incoming motion commands accumulated in its UART buffer.

    # -----------------------------------------------------------------

    def _on_disconnected(self):
        self.connection_changed.emit(
            False,
            self.port or "",
        )

    # =================================================================
    # Sending
    # =================================================================

    def send_message(self, message: dict[str, Any]):
        if self.worker is None:
            self.error_received.emit(
                "Cannot send message: serial port is not connected."
            )
            return

        try:
            text = json.dumps(
                message,
                separators=(",", ":"),
            )

        except Exception as exc:
            self.error_received.emit(
                f"Could not encode message: {exc}"
            )
            return

        self.worker.send_text(text)

    # -----------------------------------------------------------------

    def send_command(
        self,
        command: str,
        **arguments,
    ):
        message = {
            "type": "command",
            "command": command,
        }

        message.update(arguments)

        self.send_message(message)

    # -----------------------------------------------------------------

    def set_parameter(
        self,
        name: str,
        value: Any,
    ):
        self.send_message(
            {
                "type": "parameter",
                "name": name,
                "value": value,
            }
        )

    # -----------------------------------------------------------------

    def request_parameter(self, name: str):
        self.send_message(
            {
                "type": "parameter_request",
                "name": name,
            }
        )

    # -----------------------------------------------------------------

    def request_definitions(self):
        self.send_message(
            {
                "type": "request_definitions",
            }
        )

    # -----------------------------------------------------------------

    def enter_debug_mode(self):
        self.send_command(
            "set_debug_mode",
            enabled=True,
        )

    # -----------------------------------------------------------------

    def exit_debug_mode(self):
        self.send_command(
            "set_debug_mode",
            enabled=False,
        )

    # =================================================================
    # Incoming protocol
    # =================================================================

    def _handle_message(
        self,
        message: dict,
    ):
        self.message_received.emit(message)

        message_type = message.get(
            "type",
            "",
        )

        # --------------------------------------------------------------
        # Individual telemetry variable
        # --------------------------------------------------------------

        if message_type == "telemetry":

            timestamp = message.get("time")

            if "data" in message:

                data = message.get(
                    "data",
                    {},
                )

                if isinstance(data, dict):

                    for name, value in data.items():
                        self.telemetry_received.emit(
                            str(name),
                            value,
                            timestamp,
                        )

            elif "name" in message:

                self.telemetry_received.emit(
                    str(message["name"]),
                    message.get("value"),
                    timestamp,
                )

        elif message_type == "tof_8x8":
            self.tof_8x8_received.emit(message)

        # --------------------------------------------------------------
        # Parameter definition
        # --------------------------------------------------------------

        elif message_type == "parameter_definition":

            self.parameter_definition_received.emit(
                message
            )

        # --------------------------------------------------------------
        # Parameter value
        # --------------------------------------------------------------

        elif message_type == "parameter_value":

            name = str(
                message.get(
                    "name",
                    "",
                )
            )

            value = message.get(
                "value"
            )

            self.parameter_value_received.emit(
                name,
                value,
            )

        # --------------------------------------------------------------
        # Command definition
        # --------------------------------------------------------------

        elif message_type == "command_definition":

            self.command_definition_received.emit(
                message
            )

        # --------------------------------------------------------------
        # Log
        # --------------------------------------------------------------

        elif message_type == "log":

            level = str(
                message.get(
                    "level",
                    "INFO",
                )
            )

            text = str(
                message.get(
                    "message",
                    "",
                )
            )

            self.log_received.emit(
                level,
                text,
            )

        # --------------------------------------------------------------
        # Robot state
        # --------------------------------------------------------------

        elif message_type == "state":

            self.robot_state_received.emit(
                message
            )

        # --------------------------------------------------------------
        # Error reported by robot
        # --------------------------------------------------------------

        elif message_type == "error":

            text = str(
                message.get(
                    "message",
                    "Unknown robot error",
                )
            )

            self.error_received.emit(
                f"Robot: {text}"
            )
