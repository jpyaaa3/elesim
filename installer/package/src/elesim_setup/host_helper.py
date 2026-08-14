"""Short-lived host broker for the containerized connection manager.

The manager receives neither the Docker daemon socket nor the tailscaled local
API.  This stdlib-only broker accepts only the generated EleSim runtime/Compose
command shapes and an optional bounded Tailscale TCP proxy on a private Unix
socket.
"""

from __future__ import annotations

import argparse
import base64
import ipaddress
import json
import math
import os
import re
import select
import shlex
import socket
import socketserver
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Sequence


_MAX_REQUEST = 256 * 1024
_MAX_OUTPUT = 64 * 1024
_DEFAULT_COMMAND_TIMEOUT_S = 5 * 60
_MAX_COMMAND_TIMEOUT_S = 30 * 60
_ROLES = frozenset({"pilot", "sim", "ui"})
_HOSTNAME = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9.-]{0,251}[A-Za-z0-9])?$")
_GPU_SELECTOR = re.compile(
    r"^(?:[0-9]{1,6}|GPU-[A-Za-z0-9_-]{1,124}|"
    r"MIG-GPU-[A-Za-z0-9_-]{1,116}/[0-9]+/[0-9]+)$"
)


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
        raw_timeout = request.get("timeout_s", _DEFAULT_COMMAND_TIMEOUT_S)
        if isinstance(raw_timeout, bool):
            raise HostHelperError("host-helper timeout is invalid")
        try:
            timeout_s = float(raw_timeout)
        except (TypeError, ValueError) as exc:
            raise HostHelperError("host-helper timeout is invalid") from exc
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= _MAX_COMMAND_TIMEOUT_S:
            raise HostHelperError(
                f"host-helper timeout must be in (0, {_MAX_COMMAND_TIMEOUT_S:g}]"
            )

        def emit(name: str, chunk: bytes) -> None:
            if not self._reply(
                {
                    "type": "output",
                    "stream": name,
                    "data": base64.b64encode(chunk).decode("ascii"),
                }
            ):
                # The manager may time out and close its helper socket while
                # a long Docker build is still producing output.  Propagate
                # that disconnect into ``_run_bounded`` so it terminates the
                # child instead of leaving an orphan build behind.
                raise BrokenPipeError("host-helper client disconnected")

        returncode, stdout, stderr, out_cut, err_cut = _run_bounded(
            argv,
            on_output=emit if stream else None,
            cancelled=self._client_disconnected,
            timeout_s=timeout_s,
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
        target = _normalize_tailscale_target(host) if isinstance(host, str) else None
        if target is None:
            raise HostHelperError("invalid Tailscale target")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise HostHelperError("invalid Tailscale target port")
        process = subprocess.Popen(
            (str(binary), "nc", target, str(port)),
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
                except (OSError, ValueError):
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
            try:
                process.stdout.close()
            except (OSError, ValueError):
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

    def _reply(self, payload: dict[str, object]) -> bool:
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        with self._reply_lock:
            try:
                self.connection.sendall(encoded)
            except (BrokenPipeError, ConnectionResetError, OSError):
                # A disconnected proxy cannot receive an error response.
                return False
        return True

    def _client_disconnected(self) -> bool:
        """Detect a closed request socket even while a child is quiet."""

        try:
            readable, _writable, _exceptional = select.select(
                [self.connection], [], [], 0
            )
        except (OSError, ValueError):
            return True
        if not readable:
            return False
        try:
            data = self.connection.recv(
                1, socket.MSG_PEEK | getattr(socket, "MSG_DONTWAIT", 0)
            )
        except BlockingIOError:
            return False
        except (ConnectionResetError, OSError):
            return True
        return data == b""


def _validate_command(
    argv: Sequence[str], *, compose: Path, bin_dir: Path, project: str
) -> None:
    net = str(bin_dir / "elesim-net")
    if argv[0] == net and len(argv) >= 2 and argv[1] in {
        "show",
        "configuration-check",
        "namespace-check",
        "configure",
        "restore-snapshot",
        "doctor",
    }:
        return
    tailscale = str(bin_dir / "elesim-tailscale")
    if tuple(argv) in {
        (tailscale, "login"),
        (tailscale, "login", "--if-needed"),
        (tailscale, "status"),
        (tailscale, "status", "--json"),
    }:
        return
    if tuple(argv) == ("docker", "version", "--format", "{{.Server.Version}}"):
        return
    viewer_cleanup = str(bin_dir / "elesim-viewer-cleanup")
    if tuple(argv) == (viewer_cleanup,):
        return
    runtime_up = str(bin_dir / "elesim-up")
    if argv[0] == runtime_up:
        option_end = 1
        no_build = False
        viewer = False
        cuda_visible = False
        while option_end < len(argv):
            option = argv[option_end]
            if option == "--no-build":
                if no_build:
                    raise HostHelperError("--no-build may only be specified once")
                no_build = True
                option_end += 1
                continue
            if option == "--view":
                if viewer:
                    raise HostHelperError("--view may only be specified once")
                viewer = True
                option_end += 1
                continue
            if option == "--cuda-visible-devices":
                if cuda_visible or option_end + 1 >= len(argv):
                    raise HostHelperError("CUDA_VISIBLE_DEVICES option is invalid")
                value = str(argv[option_end + 1])
                if value and not _GPU_SELECTOR.fullmatch(value):
                    raise HostHelperError("CUDA_VISIBLE_DEVICES option is invalid")
                cuda_visible = True
                option_end += 2
                continue
            break
        if not no_build:
            raise HostHelperError("connection-manager launches must use --no-build")
        services = tuple(argv[option_end:])
        _validate_runtime_services(services)
        if viewer and "sim" not in services:
            raise HostHelperError("--view requires the Sim service")
        return
    compose_wrapper = str(bin_dir / "elesim-compose")
    option_end = 1
    while option_end < len(argv):
        option = argv[option_end]
        if option == "--elesim-cuda-visible-devices":
            if option_end + 1 >= len(argv):
                raise HostHelperError("CUDA_VISIBLE_DEVICES option is missing a value")
            value = str(argv[option_end + 1])
            if value and not _GPU_SELECTOR.fullmatch(value):
                raise HostHelperError("CUDA_VISIBLE_DEVICES option is invalid")
            option_end += 2
            continue
        if option == "--elesim-sim-viewer":
            if option_end + 1 >= len(argv) or argv[option_end + 1] not in {"0", "1"}:
                raise HostHelperError("Sim viewer option is invalid")
            option_end += 2
            continue
        break
    prefix = (
        compose_wrapper,
        *argv[1:option_end],
        "-p",
        project,
        "-f",
        str(compose),
    )
    progress_prefix = (
        compose_wrapper,
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
    has_runtime_options = option_end > 1
    if has_runtime_options and suffix[:4] != (
        "up",
        "-d",
        "--no-build",
        "--remove-orphans",
    ):
        raise HostHelperError(
            "runtime launch options are allowed only for an up lifecycle"
        )
    if suffix in {
        ("config", "--quiet"),
        ("config", "--services"),
        ("ps", "--all", "--services"),
        ("ps", "--status", "running", "--services"),
    }:
        return
    if suffix and suffix[0] in {"start", "stop", "build"}:
        services = suffix[1:]
        if suffix[0] == "build":
            _validate_roles(services)
        else:
            _validate_runtime_services(services)
        return
    if suffix[:4] == ("up", "-d", "--no-build", "--remove-orphans"):
        _validate_runtime_services(suffix[4:])
        return
    raise HostHelperError("Docker command is not an allowed EleSim lifecycle action")


def _validate_roles(values: Sequence[str]) -> None:
    if not values or len(set(values)) != len(values) or not set(values).issubset(_ROLES):
        raise HostHelperError("Docker lifecycle role selection is invalid")


def _validate_runtime_services(values: Sequence[str]) -> None:
    """Validate role services plus the Sim-owned managed Coturn service."""

    services = tuple(values)
    if not services:
        raise HostHelperError("Docker lifecycle must name at least one service")
    roles = services
    if services[-1] == "coturn":
        roles = services[:-1]
        if "sim" not in roles:
            raise HostHelperError("Coturn may only accompany the Sim service")
    _validate_roles(roles)


def _valid_tailscale_target(value: str) -> bool:
    """Accept Tailscale IPv4/IPv6 literals and bounded DNS names."""

    return _normalize_tailscale_target(value) is not None


def _normalize_tailscale_target(value: str) -> str | None:
    """Normalize an optional bracketed IP literal for the Tailscale CLI."""

    text = str(value).strip()
    if not text:
        return None
    unbracketed = text
    if text.startswith("[") or text.endswith("]"):
        if not (text.startswith("[") and text.endswith("]")):
            return None
        unbracketed = text[1:-1]
    try:
        address = ipaddress.ip_address(unbracketed)
    except ValueError:
        return text if _HOSTNAME.fullmatch(text) else None
    if address.is_unspecified or address.is_multicast:
        return None
    return unbracketed


def _run_bounded(
    argv: Sequence[str],
    *,
    on_output: Callable[[str, bytes], None] | None = None,
    cancelled: Callable[[], bool] | None = None,
    timeout_s: float = _DEFAULT_COMMAND_TIMEOUT_S,
) -> tuple[int, bytes, bytes, bool, bool]:
    if not math.isfinite(float(timeout_s)) or not 0 < float(timeout_s) <= _MAX_COMMAND_TIMEOUT_S:
        raise ValueError("host-helper command timeout is out of bounds")
    process = subprocess.Popen(tuple(argv), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    results: dict[str, tuple[bytes, bool]] = {}
    failures: list[Exception] = []

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
        except Exception as exc:
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
    deadline = time.monotonic() + float(timeout_s)
    cancelled_error: BaseException | None = None
    timeout_error: TimeoutError | None = None
    while True:
        try:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise subprocess.TimeoutExpired(tuple(argv), float(timeout_s))
            returncode = process.wait(timeout=min(0.1, remaining))
            break
        except subprocess.TimeoutExpired:
            expired = time.monotonic() >= deadline
            if not expired and (cancelled is None or not cancelled()):
                continue
            process.terminate()
            try:
                returncode = process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                returncode = process.wait()
            if expired:
                timeout_error = TimeoutError(
                    f"host-helper command timed out after {float(timeout_s):.1f} seconds: "
                    f"{shlex.join(tuple(str(value) for value in argv))}"
                )
            else:
                cancelled_error = BrokenPipeError("host-helper client disconnected")
            break
    for worker in workers:
        worker.join()
    if failures:
        raise failures[0]
    if cancelled_error is not None:
        raise cancelled_error
    if timeout_error is not None:
        raise timeout_error
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
