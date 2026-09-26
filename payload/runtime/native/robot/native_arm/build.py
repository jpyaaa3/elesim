#!/usr/bin/env python3
"""Build the Robot arm controller locally for the installation host architecture."""

from __future__ import annotations

import argparse
import os
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SDK = ROOT / "vendor/dynamixel_sdk"
SOURCES = (
    "group_bulk_read.cpp",
    "group_bulk_write.cpp",
    "group_sync_read.cpp",
    "group_sync_write.cpp",
    "group_fast_bulk_read.cpp",
    "group_fast_sync_read.cpp",
    "group_handler.cpp",
    "packet_handler.cpp",
    "port_handler.cpp",
    "protocol1_packet_handler.cpp",
    "protocol2_packet_handler.cpp",
    "port_handler_linux.cpp",
)


def build(output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    compiler = os.environ.get("CXX", "g++")
    with tempfile.TemporaryDirectory(prefix="elesim-arm-build-", dir=output.parent) as temporary:
        candidate = Path(temporary) / output.name
        command = [
            compiler,
            "-std=c++17",
            "-O2",
            "-fPIC",
            "-shared",
            "-pthread",
            "-Wall",
            "-I",
            str(ROOT / "vendor"),
            "-I",
            str(SDK),
            str(ROOT / "control.cpp"),
            str(ROOT / "correction_placeholder.cpp"),
            *(str(SDK / name) for name in SOURCES),
            "-o",
            str(candidate),
        ]
        subprocess.run(command, check=True)
        os.replace(candidate, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    build(args.output)


if __name__ == "__main__":
    main()
