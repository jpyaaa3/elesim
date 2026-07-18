#!/usr/bin/env python3
"""Start the lightweight distributed stack and verify its RPC data path."""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from apps.controller.rpc import ControlRpcClient, RemoteControlService, RemotePanelState


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)


def _read_log(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-port", type=int, default=5607)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    bridge_port = int(args.base_port)
    server_port = bridge_port + 1
    rpc_port = bridge_port + 2
    server_endpoint = f"tcp://127.0.0.1:{server_port}"
    bridge_endpoint = f"tcp://127.0.0.1:{bridge_port}"
    rpc_endpoint = f"tcp://127.0.0.1:{rpc_port}"

    processes: list[subprocess.Popen] = []
    handles = []
    with tempfile.TemporaryDirectory(prefix="elesim-distributed-smoke-") as temp:
        log_dir = Path(temp)

        def start(name: str, command: list[str]) -> None:
            handle = (log_dir / f"{name}.log").open("w", encoding="utf-8")
            handles.append(handle)
            processes.append(
                subprocess.Popen(command, cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT)
            )

        try:
            start("server", [sys.executable, "server.py", "--bind", server_endpoint])
            start(
                "robot",
                [
                    sys.executable,
                    "robot_agent.py",
                    "--config",
                    "configs/config.yaml",
                    "--server",
                    server_endpoint,
                    "--id",
                    "smoke-robot",
                    "--no-camera",
                ],
            )
            start(
                "control",
                [
                    sys.executable,
                    "control_agent.py",
                    "--config",
                    "configs/config.yaml",
                    "--server",
                    server_endpoint,
                    "--target",
                    "smoke-robot",
                    "--rpc-bind",
                    rpc_endpoint,
                    "--bridge-bind",
                    bridge_endpoint,
                ],
            )

            deadline = time.monotonic() + max(1.0, float(args.timeout))
            last_error: Exception | None = None
            while time.monotonic() < deadline:
                exited = [process.returncode for process in processes if process.poll() is not None]
                if exited:
                    raise RuntimeError(f"distributed child exited early: {exited}")
                client = ControlRpcClient(rpc_endpoint, timeout_ms=500)
                try:
                    state = RemotePanelState(client)
                    service = RemoteControlService(client, state)
                    endpoints = service.available_endpoints
                    active = service.active_endpoint
                    host = service.refresh_host_state()
                    if (
                        active == "smoke-robot"
                        and any(item.get("endpoint_id") == "smoke-robot" for item in endpoints)
                        and bool(host.connected)
                    ):
                        print(f"distributed smoke ok: active={active} endpoints={len(endpoints)}")
                        return 0
                except Exception as exc:
                    last_error = exc
                finally:
                    client.close()
                time.sleep(0.2)
            raise RuntimeError(f"distributed smoke timed out: {last_error!r}")
        except Exception as exc:
            print(f"distributed smoke failed: {exc}", file=sys.stderr)
            for name in ("server", "robot", "control"):
                text = _read_log(log_dir / f"{name}.log")
                if text:
                    print(f"--- {name}.log ---\n{text}", file=sys.stderr)
            return 1
        finally:
            for process in reversed(processes):
                _stop(process)
            for handle in handles:
                handle.close()


if __name__ == "__main__":
    raise SystemExit(main())
