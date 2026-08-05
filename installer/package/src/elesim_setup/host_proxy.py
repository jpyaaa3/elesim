"""Proxy stdin/stdout through the bounded host helper's Tailscale operation."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
from pathlib import Path
from typing import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="elesim-host-proxy")
    parser.add_argument("--socket", required=True)
    parser.add_argument("command", choices=("nc",))
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    args = parser.parse_args(argv)
    path = Path(args.socket)
    if not path.is_absolute():
        parser.error("--socket must be absolute")
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    connection.connect(str(path))
    connection.sendall(
        json.dumps(
            {"operation": "tailscale-nc", "host": args.host, "port": args.port},
            separators=(",", ":"),
        ).encode("utf-8")
        + b"\n"
    )
    response = _read_line(connection)
    payload = json.loads(response.decode("utf-8"))
    if not payload.get("ok"):
        raise RuntimeError(str(payload.get("error", "host proxy refused")))

    def upload() -> None:
        while True:
            content = sys.stdin.buffer.read(32 * 1024)
            if not content:
                break
            connection.sendall(content)
        connection.shutdown(socket.SHUT_WR)

    worker = threading.Thread(target=upload, daemon=True)
    worker.start()
    while True:
        content = connection.recv(32 * 1024)
        if not content:
            break
        sys.stdout.buffer.write(content)
        sys.stdout.buffer.flush()
    connection.close()
    return 0


def _read_line(connection: socket.socket) -> bytes:
    result = bytearray()
    while len(result) <= 4096:
        value = connection.recv(1)
        if not value:
            break
        result.extend(value)
        if value == b"\n":
            return bytes(result)
    raise RuntimeError("invalid host-helper response")


if __name__ == "__main__":
    raise SystemExit(main())
