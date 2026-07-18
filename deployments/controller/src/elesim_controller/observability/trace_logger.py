"""Thread-safe structured trace log for local debugging and AI handoff."""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Mapping, Optional, TextIO


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return str(value)


class StructuredTraceLogger:
    def __init__(self, service_name: str, path: str | Path) -> None:
        self.service_name = str(service_name)
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = self.path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()

    @classmethod
    def from_env(cls, service_name: str) -> "StructuredTraceLogger":
        raw_path = os.environ.get("ELESIM_TRACE_LOG", "").strip()
        if raw_path:
            path = Path(raw_path)
            if path.suffix.lower() != ".jsonl":
                path = path / f"{service_name}-{os.getpid()}.jsonl"
        else:
            path = Path("logs/tracing") / f"{service_name}-{os.getpid()}.jsonl"
        return cls(service_name, path)

    def write(self, event: str, **fields: Any) -> None:
        payload = {
            "ts_unix_ns": time.time_ns(),
            "service": self.service_name,
            "pid": os.getpid(),
            "thread": threading.current_thread().name,
            "event": str(event),
        }
        payload.update({str(key): _json_value(value) for key, value in fields.items()})
        line = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        with self._lock:
            self._file.write(line + "\n")

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.flush()
                self._file.close()


__all__ = ["StructuredTraceLogger"]
