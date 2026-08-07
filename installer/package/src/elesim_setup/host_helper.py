"""Short-lived host broker for the containerized connection manager.

The manager receives neither the Docker daemon socket nor the tailscaled local
API.  This stdlib-only broker accepts only the generated Elesim Compose command
shapes and an optional bounded Tailscale TCP proxy on a private Unix socket.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import socket
import socketserver
import subprocess
import threading
from pathlib import Path
from typing import Callable, Sequence


_MAX_REQUEST = 256 * 1024
_MAX_OUTPUT = 64 * 1024
_ROLES = frozenset({"pilot", "sim", "ui"})
_HOSTNAME = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9.-]{0,251}[A-Za-z0-9])?$")


class HostHelperError(RuntimeError):
    pass


class _Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(
        self,
        socket_path: str,
        *,
        compose: Path,
        bin_dir: Path,
        project: str,
        tailscale_bin: Path | None,
    ) -> None:
        self.compose = compose.resolve()
        self.bin_dir = bin_dir.resolve()
        self.project = project
        self.tailscale_bin = tailscale_bin
        super().__init__(socket_path, _Handler)


class _Handler(socketserver.StreamRequestHandler):
    def setup(self) -> None:
        super().setup()
        self._reply_lock = threading.Lock()

    def handle(self) -> None:
        try:
            line = self.rfile.readline(_MAX_REQUEST + 1)
            if not line or len(line) > _MAX_REQUEST or not line.endswith(b"\n"):
                raise HostHelperError("invalid host-helper request framing")
            request = json.loads(line.decode("utf-8"))
            if not isinstance(request, dict):
                raise HostHelperError("host-helper request must be an object")
            operation = request.get("operation")
            if operation == "run":
                self._run(request)
            elif operation == "tailscale-nc":
                self._tailscale_nc(request)
            else:
                raise HostHelperError("unsupported host-helper operation")
        except (HostHelperError, OSError, ValueError, json.JSONDecodeError) as exc:
            self._reply({"ok": False, "error": str(exc)[:1024]})

    @property
    def helper(self) -> _Server:
        return self.server  # type: ignore[return-value]

    def _run(self, request: dict[str, object]) -> None:
        raw = request.get("argv")
        if not isinstance(raw, list) or not raw or not all(
            isinstance(value, str) and "\x00" not in value for value in raw
        ):
            raise HostHelperError("host-helper argv is invalid")
        argv = tuple(raw)
        _validate_command(
            argv,
            compose=self.helper.compose,
            bin_dir=self.helper.bin_dir,
            project=self.helper.project,
        )
        stream = request.get("stream", False)
        if not isinstance(stream, bool):
            raise HostHelperError("host-helper stream flag is invalid")

        def emit(name: str, chunk: bytes) -> None:
            self._reply(
                {
                    "type": "output",
                    "stream": name,
                    "data": base64.b64encode(chunk).decode("ascii"),
                }
            )

        returncode, stdout, stderr, out_cut, err_cut = _run_bounded(
            argv, on_output=emit if stream else None
        )
        self._reply(
            {
                "ok": True,
                "type": "result",
                "returncode": returncode,
                "stdout": base64.b64encode(stdout).decode("ascii"),
                "stderr": base64.b64encode(stderr).decode("ascii"),
                "stdout_truncated": out_cut,
                "stderr_truncated": err_cut,
            }
        )

    def _tailscale_nc(self, request: dict[str, object]) -> None:
        binary = self.helper.tailscale_bin
        if binary is None:
            raise HostHelperError("host Tailscale CLI is unavailable")
        host = request.get("host")
        port = request.get("port")
        if not isinstance(host, str) or not _HOSTNAME.fullmatch(host):
            raise HostHelperError("invalid Tailscale target")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise HostHelperError("invalid Tailscale target port")
        process = subprocess.Popen(
            (str(binary), "nc", host, str(port)),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self._reply({"ok": True})

        def upload() -> None:
            assert process.stdin is not None
            try:
                while True:
                    data = self.connection.recv(32 * 1024)
                    if not data:
                        break
                    process.stdin.write(data)
                    process.stdin.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                # The SSH/Tailscale peer is allowed to close its half of the
                # stream first after a successful command.  This is teardown,
                # not a failed rollout; the reader side will reap the proxy.
                return
            finally:
                try:
                    process.stdin.close()
                except OSError:
                    pass

        worker = threading.Thread(target=upload, daemon=True)
        worker.start()
        assert process.stdout is not None
        try:
            while True:
                # BufferedReader.read(size) may wait for the complete size.
                # SSH starts with tiny, bidirectional banner/KEX packets, so
                # forwarding must release whatever the pipe currently has.
                data = os.read(process.stdout.fileno(), 32 * 1024)
                if not data:
                    break
                try:
                    self.connection.sendall(data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    # The client went away while the proxied command was
                    # shutting down.  Do not turn that normal EOF into a
                    # second traceback from the request handler.
                    break
        finally:
            process.stdout.close()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

    def _reply(self, payload: dict[str, object]) -> None:
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        with self._reply_lock:
            try:
                self.connection.sendall(encoded)
            except (BrokenPipeError, ConnectionResetError, OSError):
                # A disconnected proxy cannot receive an error response.
                return


def _validate_command(
    argv: Sequence[str], *, compose: Path, bin_dir: Path, project: str
) -> None:
    net = str(bin_dir / "elesim-net")
    if argv[0] == net and len(argv) >= 2 and argv[1] in {
        "show",
        "namespace-check",
        "configure",
        "restore-snapshot",
        "doctor",
    }:
        return
    if tuple(argv) == ("docker", "version", "--format", "{{.Server.Version}}"):
        return
    prefix = (
        "docker",
        "compose",
        "-p",
        project,
        "-f",
        str(compose),
    )
    progress_prefix = (
        "docker",
        "compose",
        "--progress",
        "plain",
        "-p",
        project,
        "-f",
        str(compose),
    )
    if tuple(argv[: len(progress_prefix)]) == progress_prefix:
        suffix = tuple(argv[len(progress_prefix) :])
        if suffix and suffix[0] == "build":
            _validate_roles(suffix[1:])
            return
        raise HostHelperError("Compose progress output is allowed only for builds")
    if tuple(argv[: len(prefix)]) != prefix:
        raise HostHelperError("Docker command escapes the managed Compose project")
    suffix = tuple(argv[len(prefix) :])
    if suffix in {
        ("config", "--quiet"),
        ("ps", "--status", "running", "--services"),
    }:
        return
    if suffix and suffix[0] in {"start", "stop", "build"}:
        _validate_roles(suffix[1:])
        return
    if suffix[:4] == ("up", "-d", "--no-build", "--remove-orphans"):
        if len(suffix) == 4:
            return
    raise HostHelperError("Docker command is not an allowed Elesim lifecycle action")


def _validate_roles(values: Sequence[str]) -> None:
    if not values or len(set(values)) != len(values) or not set(values).issubset(_ROLES):
        raise HostHelperError("Docker lifecycle role selection is invalid")


def _run_bounded(
    argv: Sequence[str],
    *,
    on_output: Callable[[str, bytes], None] | None = None,
) -> tuple[int, bytes, bytes, bool, bool]:
    process = subprocess.Popen(tuple(argv), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    results: dict[str, tuple[bytes, bool]] = {}
    failures: list[BaseException] = []

    def drain(name: str, stream: object) -> None:
        retained = bytearray()
        truncated = False
        try:
            while True:
                chunk = os.read(stream.fileno(), 32 * 1024)  # type: ignore[attr-defined]
                if not chunk:
                    break
                if on_output is not None:
                    on_output(name, chunk)
                retained.extend(chunk)
                if len(retained) > _MAX_OUTPUT:
                    del retained[: len(retained) - _MAX_OUTPUT]
                    truncated = True
        except BaseException as exc:
            failures.append(exc)
            process.terminate()
        finally:
            results[name] = bytes(retained), truncated

    workers = (
        threading.Thread(target=drain, args=("stdout", process.stdout), daemon=True),
        threading.Thread(target=drain, args=("stderr", process.stderr), daemon=True),
    )
    for worker in workers:
        worker.start()
    returncode = process.wait()
    for worker in workers:
        worker.join()
    if failures:
        raise failures[0]
    stdout, out_cut = results["stdout"]
    stderr, err_cut = results["stderr"]
    return returncode, stdout, stderr, out_cut, err_cut


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="elesim-host-helper")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--compose", type=Path, required=True)
    parser.add_argument("--bin-dir", type=Path, required=True)
    parser.add_argument("--project", required=True)
    parser.add_argument("--tailscale-bin", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    socket_path = Path(args.socket)
    if not socket_path.is_absolute() or socket_path.exists():
        raise SystemExit("host-helper socket must be a new absolute path")
    compose = args.compose.resolve(strict=True)
    bin_dir = args.bin_dir.resolve(strict=True)
    net = bin_dir / "elesim-net"
    if net.is_symlink() or not net.is_file() or not os.access(net, os.X_OK):
        raise SystemExit("managed elesim-net wrapper is unavailable")
    tailscale_bin = None
    if args.tailscale_bin is not None:
        candidate = args.tailscale_bin.resolve(strict=True)
        if not candidate.is_file() or not os.access(candidate, os.X_OK):
            raise SystemExit("Tailscale CLI is not executable")
        tailscale_bin = candidate
    server = _Server(
        str(socket_path),
        compose=compose,
        bin_dir=bin_dir,
        project=str(args.project),
        tailscale_bin=tailscale_bin,
    )
    os.chmod(socket_path, 0o600)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        server.server_close()
        if socket_path.exists() or socket_path.is_symlink():
            socket_path.unlink()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
