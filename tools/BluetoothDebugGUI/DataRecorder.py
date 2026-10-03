"""
DataRecorder.py

Threaded SQLite recorder for the Robot Debug Console.

The public API is intentionally simple and compatible with the earlier
DataRecorder:
    start(...)
    stop()
    record_telemetry(...)
    record_log(...)
    record_command(...)
    record_parameter(...)
    record_state(...)
    record_raw(...)
    record_fault(...)

Additional features:
- background writer thread + batched commits
- session metadata updates
- complete parameter snapshots
- annotations
- schema version 4
"""

from __future__ import annotations

import json
import queue
import sqlite3
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any


class DataRecorder:
    def __init__(self):
        self._path: Path | None = None
        self._start_monotonic: float | None = None
        self._queue: queue.Queue = queue.Queue()
        self._thread: threading.Thread | None = None
        self._recording = False
        self._lock = threading.RLock()

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def path(self) -> Path | None:
        return self._path

    @property
    def elapsed(self) -> float:
        if self._start_monotonic is None:
            return 0.0
        return max(0.0, time.monotonic() - self._start_monotonic)

    @staticmethod
    def _json(value: Any) -> str:
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=str)

    def _times(self) -> tuple[float, str]:
        return (
            self.elapsed,
            datetime.now().astimezone().isoformat(timespec="milliseconds"),
        )

    def start(
        self,
        path: str | Path,
        session_name: str = "",
        port: str = "",
        baudrate: int = 0,
        metadata: dict | None = None,
    ):
        with self._lock:
            if self._recording:
                raise RuntimeError("A recording is already active.")

            self._path = Path(path)
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._start_monotonic = time.monotonic()
            self._queue = queue.Queue()
            self._recording = True

            initial_meta = {
                "session_name": session_name,
                "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                "port": port,
                "baudrate": str(baudrate),
                "format_version": "4",
                "application": "Robot Debug Console",
            }
            if metadata:
                initial_meta.update({str(k): str(v) for k, v in metadata.items()})

            self._thread = threading.Thread(
                target=self._writer_main,
                args=(self._path, initial_meta),
                name="RobotDebugDataWriter",
                daemon=True,
            )
            self._thread.start()

    def stop(self):
        with self._lock:
            if not self._recording:
                return
            self._recording = False
            self._queue.put(("__STOP__", None))

        if self._thread is not None:
            self._thread.join(timeout=10.0)

        self._thread = None
        self._start_monotonic = None

    def _writer_main(self, path: Path, initial_meta: dict):
        conn = sqlite3.connect(path)
        try:
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")
            conn.execute("PRAGMA temp_store=MEMORY")
            self._create_schema(conn)

            conn.executemany(
                "INSERT OR REPLACE INTO metadata(key,value) VALUES (?,?)",
                list(initial_meta.items()),
            )
            conn.commit()

            pending = 0
            last_commit = time.monotonic()

            while True:
                try:
                    kind, payload = self._queue.get(timeout=0.20)
                except queue.Empty:
                    kind, payload = None, None

                if kind == "__STOP__":
                    conn.execute(
                        "INSERT OR REPLACE INTO metadata(key,value) VALUES (?,?)",
                        ("closed_at", datetime.now().astimezone().isoformat(timespec="seconds")),
                    )
                    conn.commit()
                    break

                if kind is not None:
                    self._write_one(conn, kind, payload)
                    pending += 1

                now = time.monotonic()
                if pending >= 100 or (pending and now - last_commit >= 0.25):
                    conn.commit()
                    pending = 0
                    last_commit = now

        finally:
            try:
                conn.commit()
            except Exception:
                pass
            conn.close()

    @staticmethod
    def _create_schema(conn: sqlite3.Connection):
        conn.executescript("""
        CREATE TABLE IF NOT EXISTS metadata(
            key TEXT PRIMARY KEY,
            value TEXT
        );

        CREATE TABLE IF NOT EXISTS telemetry(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            robot_time REAL,
            signal TEXT NOT NULL,
            value_num REAL,
            value_text TEXT,
            value_type TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_telemetry_signal_time
            ON telemetry(signal, elapsed_s);

        CREATE TABLE IF NOT EXISTS logs(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            level TEXT,
            message TEXT
        );

        CREATE TABLE IF NOT EXISTS commands(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            command TEXT,
            arguments_json TEXT
        );

        CREATE TABLE IF NOT EXISTS parameters(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            name TEXT,
            value_json TEXT
        );

        CREATE TABLE IF NOT EXISTS parameter_snapshots(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            snapshot_json TEXT NOT NULL
        );

        CREATE TABLE IF NOT EXISTS states(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            state_json TEXT
        );

        CREATE TABLE IF NOT EXISTS raw_serial(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            line TEXT
        );

        CREATE TABLE IF NOT EXISTS faults(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            label TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_faults_time ON faults(elapsed_s);

        CREATE TABLE IF NOT EXISTS annotations(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            label TEXT NOT NULL,
            note TEXT
        );

        CREATE TABLE IF NOT EXISTS matrix_frames(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            robot_time REAL,
            frame_json TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_matrix_frames_time
            ON matrix_frames(elapsed_s);

        CREATE TABLE IF NOT EXISTS ui_snapshots(
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            elapsed_s REAL NOT NULL,
            wall_time TEXT NOT NULL,
            snapshot_json TEXT NOT NULL
        );
        CREATE INDEX IF NOT EXISTS idx_ui_snapshots_time
            ON ui_snapshots(elapsed_s);
        """)

    def _write_one(self, conn: sqlite3.Connection, kind: str, payload):
        if kind == "telemetry":
            conn.execute(
                """INSERT INTO telemetry
                   (elapsed_s,wall_time,robot_time,signal,value_num,value_text,value_type)
                   VALUES (?,?,?,?,?,?,?)""",
                payload,
            )
        elif kind == "log":
            conn.execute(
                "INSERT INTO logs(elapsed_s,wall_time,level,message) VALUES (?,?,?,?)",
                payload,
            )
        elif kind == "command":
            conn.execute(
                "INSERT INTO commands(elapsed_s,wall_time,command,arguments_json) VALUES (?,?,?,?)",
                payload,
            )
        elif kind == "parameter":
            conn.execute(
                "INSERT INTO parameters(elapsed_s,wall_time,name,value_json) VALUES (?,?,?,?)",
                payload,
            )
        elif kind == "parameter_snapshot":
            conn.execute(
                "INSERT INTO parameter_snapshots(elapsed_s,wall_time,snapshot_json) VALUES (?,?,?)",
                payload,
            )
        elif kind == "state":
            conn.execute(
                "INSERT INTO states(elapsed_s,wall_time,state_json) VALUES (?,?,?)",
                payload,
            )
        elif kind == "raw":
            conn.execute(
                "INSERT INTO raw_serial(elapsed_s,wall_time,line) VALUES (?,?,?)",
                payload,
            )
        elif kind == "fault":
            conn.execute(
                "INSERT INTO faults(elapsed_s,wall_time,label) VALUES (?,?,?)",
                payload,
            )
        elif kind == "annotation":
            conn.execute(
                "INSERT INTO annotations(elapsed_s,wall_time,label,note) VALUES (?,?,?,?)",
                payload,
            )
        elif kind == "metadata":
            conn.execute(
                "INSERT OR REPLACE INTO metadata(key,value) VALUES (?,?)",
                payload,
            )
        elif kind == "matrix":
            conn.execute(
                "INSERT INTO matrix_frames(elapsed_s,wall_time,robot_time,frame_json) VALUES (?,?,?,?)",
                payload,
            )
        elif kind == "ui_snapshot":
            conn.execute(
                "INSERT INTO ui_snapshots(elapsed_s,wall_time,snapshot_json) VALUES (?,?,?)",
                payload,
            )

    def _enqueue(self, kind: str, payload):
        if self._recording:
            self._queue.put((kind, payload))

    def set_metadata(self, key: str, value: Any):
        self._enqueue("metadata", (str(key), str(value)))

    def record_telemetry(self, signal: str, value: Any, robot_time: Any = None):
        if not self._recording:
            return

        elapsed, wall = self._times()
        rt = None
        try:
            if robot_time is not None:
                rt = float(robot_time)
        except (TypeError, ValueError):
            rt = None

        if isinstance(value, bool):
            value_num = float(value)
            value_text = "true" if value else "false"
            value_type = "bool"
        elif isinstance(value, (int, float)):
            try:
                value_num = float(value)
            except Exception:
                value_num = None
            value_text = None
            value_type = "number"
        elif value is None:
            value_num = None
            value_text = None
            value_type = "null"
        else:
            value_num = None
            value_text = str(value)
            value_type = type(value).__name__

        self._enqueue(
            "telemetry",
            (elapsed, wall, rt, str(signal), value_num, value_text, value_type),
        )

    def record_log(self, level: str, message: str):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("log", (elapsed, wall, str(level), str(message)))

    def record_command(self, command: str, arguments: dict):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("command", (elapsed, wall, str(command), self._json(arguments)))

    def record_parameter(self, name: str, value: Any):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("parameter", (elapsed, wall, str(name), self._json(value)))

    def record_parameter_snapshot(self, values: dict[str, Any]):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("parameter_snapshot", (elapsed, wall, self._json(values)))

    def record_state(self, state: dict):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("state", (elapsed, wall, self._json(state)))

    def record_raw(self, line: str):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("raw", (elapsed, wall, str(line)))

    def record_fault(self, label: str = "MANUAL FAULT MARKER") -> float | None:
        if not self._recording:
            return None
        elapsed, wall = self._times()
        self._enqueue("fault", (elapsed, wall, str(label)))
        return elapsed

    def record_annotation(self, label: str, note: str = "") -> float | None:
        if not self._recording:
            return None
        elapsed, wall = self._times()
        self._enqueue("annotation", (elapsed, wall, str(label), str(note)))
        return elapsed

    def record_matrix(self, message: dict):
        if not self._recording:
            return
        elapsed, wall = self._times()
        robot_time = message.get("time")
        try:
            robot_time = float(robot_time) if robot_time is not None else None
        except (TypeError, ValueError):
            robot_time = None
        self._enqueue("matrix", (elapsed, wall, robot_time, self._json(message)))

    def record_ui_snapshot(self, snapshot: dict):
        if not self._recording:
            return
        elapsed, wall = self._times()
        self._enqueue("ui_snapshot", (elapsed, wall, self._json(snapshot)))
