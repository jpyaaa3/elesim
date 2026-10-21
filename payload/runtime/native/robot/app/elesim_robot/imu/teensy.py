"""Bounded POSIX USB serial reader; never drives actuators or selects models."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import select
import termios
import threading
import time
import tty
from dataclasses import dataclass

MAX_LINE_BYTES = 2048


@dataclass(frozen=True)
class Sample:
    sensor: str
    device_time_ms: int
    quaternion_wxyz: tuple[float, ...] | None
    rpy_rad: tuple[float, ...] | None
    sequence: int | None
    received_at: float
    source_format: str = "rotation"
    full_imu: dict | None = None


def parse_line(line: bytes, received_at: float) -> Sample | None:
    """Accept legacy 10-field and sequenced 11-field reports, ignore status."""
    if len(line) > MAX_LINE_BYTES:
        raise ValueError("oversized IMU line")
    fields = line.decode("ascii").strip().split(",")
    if fields[0] not in {"IMU1", "IMU2"}:
        return None
    if len(fields) not in {10, 11} or fields[9] != "0":
        raise ValueError("invalid IMU report shape or magnetometer mode")
    stamp = int(fields[1])
    seq = int(fields[10]) if len(fields) == 11 else None
    if not 0 <= stamp < 2**32 or (seq is not None and not 0 <= seq < 2**32):
        raise ValueError("invalid IMU counter")
    values = tuple(float(value) for value in fields[2:9])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite IMU report")
    norm = math.sqrt(sum(value * value for value in values[:4]))
    if not 0.9 <= norm <= 1.1 or any(abs(v) > 180.001 for v in values[4:]):
        raise ValueError("invalid IMU orientation")
    return Sample(fields[0], stamp, tuple(v / norm for v in values[:4]),
                  tuple(math.radians(v) for v in values[4:]), seq, received_at)


def _counter(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed < 2**32:
        raise ValueError("invalid Full IMU counter")
    return parsed


def parse_full_line(line: bytes, received_at: float) -> tuple[Sample, ...]:
    """52-column full reports. newAny is NOT a rotation-update indicator."""
    if len(line) > MAX_LINE_BYTES:
        raise ValueError("oversized Full IMU line")
    fields = line.decode("ascii").strip().split(",")
    if len(fields) != 52:
        raise ValueError("Full IMU report requires 52 columns")
    tx_ms, seq = _counter(fields[0]), _counter(fields[1])
    result = []
    for index in range(2):
        block = fields[2 + 25 * index:27 + 25 * index]
        stamp, bno_us = _counter(block[0]), _counter(block[1])
        if block[2] not in {"0", "1"}:
            raise ValueError("invalid Full IMU newAny flag")
        quat = tuple(float(v) for v in block[3:7])
        accuracy = [int(v) for v in block[7:10]]
        vectors = [float(v) for v in block[10:25]]
        if not all(math.isfinite(v) for v in (*quat, *vectors)):
            raise ValueError("nonfinite Full IMU report")
        if any(not 0 <= v <= 3 for v in accuracy):
            raise ValueError("invalid Full IMU accuracy")
        norm = math.sqrt(sum(v * v for v in quat))
        if norm == 0:
            # Firmware initializes the quaternion to zero until a rotation report.
            quaternion = rpy = None
        elif 0.9 <= norm <= 1.1:
            quaternion = tuple(v / norm for v in quat)
            w, x, y, z = quaternion
            rpy = (math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
                   math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
                   math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))
        else:
            raise ValueError("invalid Full IMU quaternion")
        if block[2] == "0":
            continue
        details = {"tx_time_ms": tx_ms, "bno_time_us": bno_us,
                   "accuracy": dict(zip(("quat", "gravity", "gyro"), accuracy))}
        for group, name in enumerate(("gravity", "gyro", "linear_accel", "accel", "mag")):
            details[name] = vectors[3 * group:3 * group + 3]
        result.append(Sample(f"IMU{index + 1}", stamp, quaternion, rpy, seq,
                             received_at, "full-imu-v1", details))
    return tuple(result)


class TeensyReader:
    """One latest sample per sensor, bounded framing, reconnect and clean stop."""

    def __init__(self, device: str, *, clock=time.monotonic) -> None:
        if not device:
            raise ValueError("IMU device path is required")
        self.device = device
        self.clock = clock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._samples: dict[str, Sample] = {}
        self._buffer = bytearray()
        self._discard = False
        self._connected = False
        self._error = ""
        self._invalid_lines = 0

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("IMU reader already started")
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="teensy-imu", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("IMU reader did not stop")
            self._thread = None

    def feed(self, data: bytes) -> None:
        """Frame input incrementally without retaining arbitrarily long lines."""
        with self._lock:
            for byte in data:
                if byte == 10:
                    if not self._discard:
                        self._accept(bytes(self._buffer))
                    self._buffer.clear()
                    self._discard = False
                elif not self._discard:
                    self._buffer.append(byte)
                    if len(self._buffer) > MAX_LINE_BYTES:
                        self._buffer.clear()
                        self._discard = True
                        self._invalid_lines += 1

    def _accept(self, line: bytes) -> None:
        if line.strip() in {b"SPARKFUN_BNO080,READY", b"=== Dual BNO085 Full IMU Output ==="}:
            self._samples.clear()
        try:
            if line.strip().split(b",", 1)[0].lstrip(b"+-").isdigit():
                samples = parse_full_line(line, self.clock())
            else:
                sample = parse_line(line, self.clock())
                samples = () if sample is None else (sample,)
        except (ValueError, UnicodeError, OverflowError):
            self._invalid_lines += 1
            return
        for sample in samples:
            old = self._samples.get(sample.sensor)
            if (old is not None and old.source_format == sample.source_format
                    and old.sequence is not None and sample.sequence is not None):
                # uint32 wrap is valid; duplicate/backward reports cannot refresh age.
                if not 0 < (sample.sequence - old.sequence) % 2**32 < 2**31:
                    continue
            self._samples[sample.sensor] = sample

    def snapshot(self) -> dict:
        with self._lock:
            now = self.clock()
            sensors = {}
            for name, sample in self._samples.items():
                age = max(0.0, now - sample.received_at)
                verified = sample.source_format == "rotation" and sample.sequence is not None
                sensors[name] = {
                    "device_time_ms": sample.device_time_ms,
                    "sequence": sample.sequence,
                    "quaternion_wxyz": (list(sample.quaternion_wxyz)
                                        if sample.quaternion_wxyz is not None else None),
                    "rpy_rad": list(sample.rpy_rad) if sample.rpy_rad is not None else None,
                    "received_age_s": age,
                    "source_format": sample.source_format,
                    "freshness_verified": verified,
                    "reports_recent": self._connected and age <= 0.2,
                    "fresh": self._connected and verified and age <= 0.2,
                }
                if sample.full_imu is not None:
                    sensors[name]["full_imu"] = {
                        key: (dict(value) if isinstance(value, dict) else
                              list(value) if isinstance(value, list) else value)
                        for key, value in sample.full_imu.items()
                    }
            return {"schema_version": 1, "connected": self._connected,
                    "error": self._error, "invalid_lines": self._invalid_lines,
                    "sensors": sensors}

    def _run(self) -> None:
        while not self._stop.is_set():
            fd = None
            previous = None
            try:
                fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                previous = termios.tcgetattr(fd)
                tty.setraw(fd, termios.TCSANOW)
                attrs = termios.tcgetattr(fd)
                attrs[4] = attrs[5] = termios.B115200
                attrs[2] |= termios.CLOCAL | termios.CREAD
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                termios.tcflush(fd, termios.TCIFLUSH)
                with self._lock:
                    self._samples.clear()
                    self._buffer.clear()
                    self._discard = False
                    self._connected = True
                    self._error = ""
                while not self._stop.is_set():
                    if not select.select([fd], [], [], 0.1)[0]:
                        continue
                    try:
                        data = os.read(fd, 4096)
                    except BlockingIOError:
                        continue
                    if not data:
                        raise OSError("IMU serial disconnected")
                    self.feed(data)
            except (OSError, termios.error) as exc:
                with self._lock:
                    self._error = str(exc)[:200]
            finally:
                with self._lock:
                    self._connected = False
                if fd is not None:
                    try:
                        if previous is not None:
                            termios.tcsetattr(fd, termios.TCSANOW, previous)
                    except (OSError, termios.error):
                        pass
                    os.close(fd)
            self._stop.wait(1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Read both Teensy IMUs without motor/DDS access")
    parser.add_argument("--device", required=True)
    args = parser.parse_args()
    reader = TeensyReader(args.device)
    reader.start()
    try:
        while True:
            print(json.dumps(reader.snapshot(), allow_nan=False), flush=True)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
