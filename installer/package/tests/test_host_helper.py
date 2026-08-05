import json
import os
import socket
import threading
from pathlib import Path

import pytest

from elesim_setup.host_helper import HostHelperError, _Server, _validate_command
from elesim_setup.host_proxy import _upload_stdin


def _paths() -> tuple[Path, Path]:
    return Path("/opt/elesim/containers/compose.yaml"), Path("/opt/elesim/bin")


def test_host_helper_allows_only_fixed_compose_lifecycle_shapes() -> None:
    compose, bin_dir = _paths()
    prefix = (
        "docker",
        "compose",
        "-p",
        "elesim-runtime",
        "-f",
        str(compose),
    )
    for suffix in (
        ("config", "--quiet"),
        ("ps", "--status", "running", "--services"),
        ("build", "pilot", "ui"),
        ("stop", "sim"),
        ("start", "pilot"),
        ("up", "-d", "--no-build", "--remove-orphans"),
    ):
        _validate_command(
            (*prefix, *suffix),
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
        (str(bin_dir / "elesim-net"), "show"),
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
