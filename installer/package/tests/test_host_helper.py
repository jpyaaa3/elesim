import base64
import json
import os
import socket
import threading
import time
from pathlib import Path

import pytest

from elesim_setup.host_helper import (
    HostHelperError,
    _Server,
    _validate_command,
    _valid_tailscale_target,
)
from elesim_setup.host_proxy import _upload_stdin
from elesim_setup.secure_deployment import _run_through_host_helper


def _paths() -> tuple[Path, Path]:
    return Path("/opt/elesim/containers/compose.yaml"), Path("/opt/elesim/bin")


def _compose_wrapper(bin_dir: Path) -> str:
    return str(bin_dir / "elesim-compose")


def test_host_helper_allows_only_fixed_compose_lifecycle_shapes() -> None:
    compose, bin_dir = _paths()
    prefix = (
        _compose_wrapper(bin_dir),
        "-p",
        "elesim-runtime",
        "-f",
        str(compose),
    )
    for suffix in (
        ("config", "--quiet"),
        ("config", "--services"),
        ("ps", "--all", "--services"),
        ("ps", "--status", "running", "--services"),
        ("build", "pilot", "ui"),
        ("stop", "sim"),
        ("stop", "sim", "coturn"),
        ("start", "pilot"),
        ("up", "-d", "--no-build", "--remove-orphans", "pilot", "sim", "coturn"),
    ):
        _validate_command(
            (*prefix, *suffix),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )
    _validate_command(
        (
            _compose_wrapper(bin_dir),
            "--progress",
            "plain",
            "-p",
            "elesim-runtime",
            "-f",
            str(compose),
            "build",
            "pilot",
            "ui",
        ),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
    )


@pytest.mark.parametrize(
    "argv",
    (
        ("docker", "run", "--privileged", "alpine"),
        (
            "docker",
            "compose",
            "-p",
            "other",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "stop",
            "pilot",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/tmp/compose.yaml",
            "build",
            "pilot",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "build",
            "manager",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "down",
        ),
        (
            "docker",
            "compose",
            "--progress",
            "plain",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "start",
            "pilot",
        ),
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "-f",
            "/opt/elesim/containers/compose.yaml",
            "up",
            "-d",
            "--no-build",
            "--remove-orphans",
            "coturn",
        ),
    ),
)
def test_host_helper_rejects_daemon_escape_shapes(argv: tuple[str, ...]) -> None:
    compose, bin_dir = _paths()
    with pytest.raises(HostHelperError):
        _validate_command(
            argv,
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )


def test_host_helper_limits_network_cli_to_installed_wrapper() -> None:
    compose, bin_dir = _paths()
    _validate_command(
        (str(bin_dir / "elesim-tailscale"), "login", "--if-needed"),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
    )
    _validate_command(
        (str(bin_dir / "elesim-net"), "show"),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
    )
    _validate_command(
        (str(bin_dir / "elesim-net"), "namespace-check"),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
    )
    with pytest.raises(HostHelperError):
        _validate_command(
            ("/tmp/elesim-net", "show"),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )
    with pytest.raises(HostHelperError):
        _validate_command(
            (str(bin_dir / "elesim-net"), "uninstall"),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )


def test_host_helper_rejects_unscoped_compose_up() -> None:
    compose, bin_dir = _paths()
    with pytest.raises(HostHelperError, match="at least one service"):
        _validate_command(
            (
                _compose_wrapper(bin_dir),
                "-p",
                "elesim-runtime",
                "-f",
                str(compose),
                "up",
                "-d",
                "--no-build",
                "--remove-orphans",
            ),
            compose=compose,
            bin_dir=bin_dir,
            project="elesim-runtime",
        )


def test_tailscale_target_accepts_ipv6_and_rejects_path_values() -> None:
    assert _valid_tailscale_target("fd7a:115c:a1e0::1234")
    assert _valid_tailscale_target("[fd7a:115c:a1e0::1234]")
    assert _valid_tailscale_target("sim.example")
    assert not _valid_tailscale_target("/tmp/socket")
    assert not _valid_tailscale_target("sim example")


def test_tailscale_stream_releases_small_banner_before_eof(tmp_path: Path) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("services: {}\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    proxy = tmp_path / "tailscale"
    proxy.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "os.write(1, b'SSH-2.0-test\\r\\n')\n"
        "data = os.read(0, 4)\n"
        "os.write(1, b'echo:' + data)\n",
        encoding="utf-8",
    )
    proxy.chmod(0o755)
    socket_path = tmp_path / "helper.sock"
    server = _Server(
        str(socket_path),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
        tailscale_bin=proxy,
    )
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(1)
            connection.connect(str(socket_path))
            connection.sendall(
                json.dumps(
                    {"operation": "tailscale-nc", "host": "100.64.0.2", "port": 22}
                ).encode("utf-8")
                + b"\n"
            )
            assert _recv_line(connection) == b'{"ok":true}\n'
            banner = b"SSH-2.0-test\r\n"
            assert _recv_exact(connection, len(banner)) == banner
            connection.sendall(b"kex!")
            connection.shutdown(socket.SHUT_WR)
            assert _recv_exact(connection, 9) == b"echo:kex!"
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)


def test_host_proxy_upload_treats_peer_close_as_normal_eof() -> None:
    left, right = socket.socketpair()
    input_fd = os.open(os.devnull, os.O_RDONLY)
    try:
        right.close()
        _upload_stdin(left, input_fd)
    finally:
        os.close(input_fd)
        left.close()


def test_host_helper_streams_actual_command_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("services: {}\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    wrapper = bin_dir / "elesim-compose"
    wrapper.write_text(
        "#!/bin/sh\nexec docker compose \"$@\"\n",
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    docker = bin_dir / "docker"
    docker.write_text(
        "#!/usr/bin/env python3\n"
        "import os, time\n"
        "os.write(1, b'#1 loading build definition\\n')\n"
        "time.sleep(0.05)\n"
        "os.write(2, b'#2 building role image\\n')\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ.get('PATH', '')}")
    socket_path = tmp_path / "helper.sock"
    server = _Server(
        str(socket_path),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
        tailscale_bin=None,
    )
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    output: list[tuple[str, str]] = []
    try:
        result = _run_through_host_helper(
            (
                    str(wrapper),
                    "--progress",
                "plain",
                "-p",
                "elesim-runtime",
                "-f",
                str(compose),
                "build",
                "pilot",
            ),
            socket_path=str(socket_path),
            timeout_s=2,
            output=lambda stream, text: output.append((stream, text)),
        )
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)

    assert result.exit_status == 0
    assert ("stdout", "#1 loading build definition\n") in output
    assert ("stderr", "#2 building role image\n") in output


def test_host_helper_enforces_client_command_timeout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("services: {}\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    wrapper = bin_dir / "elesim-compose"
    wrapper.write_text(
        "#!/bin/sh\nexec docker compose \"$@\"\n",
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    docker = bin_dir / "docker"
    docker.write_text(
        "#!/usr/bin/env python3\n"
        "import time\n"
        "time.sleep(2)\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ.get('PATH', '')}")
    socket_path = tmp_path / "helper.sock"
    server = _Server(
        str(socket_path),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
        tailscale_bin=None,
    )
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        with pytest.raises(RuntimeError, match="command timed out"):
            _run_through_host_helper(
                (
                    str(wrapper),
                    "-p",
                    "elesim-runtime",
                    "-f",
                    str(compose),
                    "up",
                    "-d",
                    "--no-build",
                    "--remove-orphans",
                    "pilot",
                ),
                socket_path=str(socket_path),
                timeout_s=0.8,
            )
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)


def test_host_helper_terminates_command_when_stream_client_disconnects(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("services: {}\n", encoding="utf-8")
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    wrapper = bin_dir / "elesim-compose"
    wrapper.write_text(
        "#!/bin/sh\nexec docker compose \"$@\"\n",
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    pid_file = tmp_path / "child.pid"
    docker = bin_dir / "docker"
    docker.write_text(
        "#!/usr/bin/env python3\n"
        "import os, pathlib, time\n"
        f"pathlib.Path({str(pid_file)!r}).write_text(str(os.getpid()))\n"
        "os.write(1, b'#1 long build\\n')\n"
        "while True: time.sleep(0.05)\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_dir}:{os.environ.get('PATH', '')}")
    socket_path = tmp_path / "helper.sock"
    server = _Server(
        str(socket_path),
        compose=compose,
        bin_dir=bin_dir,
        project="elesim-runtime",
        tailscale_bin=None,
    )
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        connection.settimeout(1)
        connection.connect(str(socket_path))
        connection.sendall(
            json.dumps(
                {
                    "operation": "run",
                    "argv": [
                        str(wrapper),
                        "--progress",
                        "plain",
                        "-p",
                        "elesim-runtime",
                        "-f",
                        str(compose),
                        "build",
                        "pilot",
                    ],
                    "stream": True,
                }
            ).encode("utf-8")
            + b"\n"
        )
        frame = json.loads(connection.recv(4096).decode("utf-8"))
        assert base64.b64decode(frame["data"]) == b"#1 long build\n"
        connection.close()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not pid_file.exists():
            time.sleep(0.01)
        assert pid_file.exists()
        child_pid = int(pid_file.read_text(encoding="utf-8"))
        while time.monotonic() < deadline:
            try:
                os.kill(child_pid, 0)
            except ProcessLookupError:
                break
            time.sleep(0.02)
        else:
            pytest.fail("host helper left a child build running after disconnect")
    finally:
        try:
            connection.close()
        except OSError:
            pass
        server.shutdown()
        server.server_close()
        worker.join(timeout=1)


def test_host_proxy_releases_small_stdin_packet_before_eof() -> None:
    left, right = socket.socketpair()
    read_fd, write_fd = os.pipe()
    worker = threading.Thread(target=_upload_stdin, args=(left, read_fd), daemon=True)
    worker.start()
    try:
        right.settimeout(1)
        os.write(write_fd, b"kex")
        assert right.recv(3) == b"kex"
    finally:
        os.close(write_fd)
        worker.join(timeout=1)
        os.close(read_fd)
        left.close()
        right.close()


def _recv_line(connection: socket.socket) -> bytes:
    result = bytearray()
    while not result.endswith(b"\n"):
        result.extend(connection.recv(1))
    return bytes(result)


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        result.extend(connection.recv(size - len(result)))
    return bytes(result)
