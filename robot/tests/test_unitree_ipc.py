from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import pytest

from elesim_robot.config import SafetyConfig
from elesim_robot.go2.config import Go2HardwareConfig
from elesim_robot.go2.odom_parser import OdomSample
from elesim_robot.go2.unitree_bridge_daemon import (
    _apply_unitree_environment,
    _ensure_workspace_environment,
    _source_workspace_environment,
)
from elesim_robot.go2.unitree_ipc import (
    MAX_REMEMBERED_CLIENT_BOOTS,
    UnitreeBridgeServer,
    UnitreeIpcClient,
)
from elesim_robot.go2.unitree_ipc_protocol import (
    MAX_PACKET_BYTES,
    UnitreeIpcProtocolError,
    decode_packet,
    encode_packet,
    new_boot_id,
    peer_credentials,
    sample_from_payload,
)


class FakeBackend:
    def __init__(self) -> None:
        self.started = threading.Event()
        self.stopped = threading.Event()
        self.velocities: list[tuple[float, float, float, float]] = []
        self.poses: list[str] = []
        self.obstacles: list[bool] = []
        self.sample = OdomSample(
            pos=(1.0, 2.0, 3.0),
            rpy=(0.1, 0.2, 0.3),
            lin_vel_body=(0.4, 0.0, 0.0),
            ang_vel_body=(0.0, 0.0, 0.1),
            timestamp_s=123.5,
        )

    def start(self) -> None:
        self.started.set()

    def stop(self) -> None:
        self.stopped.set()

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self.velocities.append((float(vx), float(vy), float(wz), time.monotonic()))

    def tick_cmd(self, _now: float | None = None) -> None:
        return

    def latest_state(self) -> OdomSample:
        return self.sample

    def call_sport_pose(self, pose: str) -> None:
        self.poses.append(str(pose))

    def set_obstacles_avoid(self, enabled: bool) -> None:
        self.obstacles.append(bool(enabled))


def _wait(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true before timeout")


def _settings(socket_path: Path, *, deadman: float = 0.2):
    config = Go2HardwareConfig(
        enabled=True,
        network_interface="unitree0",
        ros_domain_id=1,
        ipc_socket_path=str(socket_path),
        ipc_heartbeat_interval_s=min(0.03, deadman / 2.0),
        cmd_hz=50.0,
    )
    safety = replace(SafetyConfig(), command_deadman_s=deadman)
    return config, safety


def _start_server(socket_path: Path, *, deadman: float = 0.2):
    config, safety = _settings(socket_path, deadman=deadman)
    backend = FakeBackend()
    stop = threading.Event()
    server = UnitreeBridgeServer(
        config,
        safety,
        backend,
        expected_client_uid=os.getuid(),
    )
    thread = threading.Thread(target=server.serve_forever, args=(stop,), daemon=True)
    thread.start()
    _wait(lambda: socket_path.exists())
    return config, safety, backend, stop, thread


def _stop_server(stop: threading.Event, thread: threading.Thread) -> None:
    stop.set()
    thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_packet_contract_is_versioned_bounded_and_finite() -> None:
    boot = new_boot_id()
    encoded = encode_packet(
        "command",
        boot,
        7,
        {"name": "stop"},
        sent_monotonic_s=1.25,
    )
    packet = decode_packet(encoded)
    assert (packet.boot_id, packet.seq, packet.payload) == (boot, 7, {"name": "stop"})

    with pytest.raises(UnitreeIpcProtocolError, match="byte limit"):
        encode_packet(
            "error",
            boot,
            8,
            {"reason": "x" * MAX_PACKET_BYTES},
            sent_monotonic_s=1.5,
        )
    with pytest.raises(UnitreeIpcProtocolError, match="finite JSON"):
        encode_packet(
            "command",
            boot,
            9,
            {"name": "set_velocity", "vx": float("nan")},
            sent_monotonic_s=2.0,
        )


def test_so_peercred_reads_the_connected_local_process() -> None:
    left, right = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    try:
        assert peer_credentials(left).uid == os.getuid()
        assert peer_credentials(right).pid == os.getpid()
    finally:
        left.close()
        right.close()


def test_unitree_daemon_binds_cyclonedds_to_only_its_configured_graph(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    security_keys = (
        "ROS_SECURITY_ENABLE",
        "ROS_SECURITY_STRATEGY",
        "ROS_SECURITY_KEYSTORE",
        "ROS_SECURITY_ENCLAVE_OVERRIDE",
        "ROS_SECURITY_ROOT_DIRECTORY",
    )
    for key in security_keys:
        monkeypatch.setenv(key, "/private/robot")
    monkeypatch.setenv("ELESIM_DDS_STATIC_PEERS", "100.64.0.1")

    _apply_unitree_environment(interface="unitree&lan", domain_id=17)

    assert os.environ["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert os.environ["ROS_DOMAIN_ID"] == "17"
    assert os.environ["ROS_LOCALHOST_ONLY"] == "0"
    assert 'name="unitree&amp;lan"' in os.environ["CYCLONEDDS_URI"]
    assert not set(security_keys).intersection(os.environ)
    assert "ELESIM_DDS_STATIC_PEERS" not in os.environ


def test_unitree_daemon_sources_the_configured_workspace(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    setup = tmp_path / "install/setup.bash"
    setup.parent.mkdir()
    setup.write_text("export UNITREE_TEST_PREFIX=/unitree/overlay\n", encoding="utf-8")
    monkeypatch.delenv("UNITREE_TEST_PREFIX", raising=False)

    _source_workspace_environment(str(tmp_path))

    assert os.environ["UNITREE_TEST_PREFIX"] == "/unitree/overlay"


def test_unitree_workspace_environment_is_reexeced_for_native_loader_paths(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    setup = tmp_path / "install/setup.bash"
    setup.parent.mkdir()
    setup.write_text("export LD_LIBRARY_PATH=/unitree/lib\n", encoding="utf-8")
    config = tmp_path / "robot.yaml"
    config.write_text("schema_version: 4\n", encoding="utf-8")
    monkeypatch.delenv("ELESIM_UNITREE_WORKSPACE_READY", raising=False)
    captured: dict[str, object] = {}

    def fake_execve(executable: str, argv: list[str], environment: dict[str, str]) -> None:
        captured.update(executable=executable, argv=argv, environment=environment)

    monkeypatch.setattr(os, "execve", fake_execve)
    _ensure_workspace_environment(str(tmp_path), config_path=str(config))

    environment = captured["environment"]
    assert isinstance(environment, dict)
    assert environment["LD_LIBRARY_PATH"] == "/unitree/lib"
    assert environment["ELESIM_UNITREE_WORKSPACE_READY"] == str(setup)
    assert captured["argv"][-2:] == ["--config", str(config)]


def test_client_server_commands_and_latest_only_telemetry(tmp_path: Path) -> None:
    path = tmp_path / "bridge.sock"
    config, safety, backend, stop, thread = _start_server(path)
    client = UnitreeIpcClient(config, safety, expected_server_uid=os.getuid())
    try:
        client.start()
        client.set_velocity(0.2, -0.1, 0.3)
        client.call_sport_pose("stand")
        client.set_obstacles_avoid(True)
        _wait(lambda: any(item[:3] == (0.2, -0.1, 0.3) for item in backend.velocities))
        _wait(lambda: bool(backend.poses and backend.obstacles))
        _wait(
            lambda: (client.tick_cmd() is None and client.latest_state() is not None),
            timeout=1.0,
        )
        sample = client.latest_state()
        assert sample is not None
        assert sample.pos == (1.0, 2.0, 3.0)
        assert backend.poses == ["stand_up"]
        assert backend.obstacles == [True]
    finally:
        client.stop()
        _stop_server(stop, thread)
    assert not path.exists()


def test_telemetry_is_one_latest_reply_per_inbound_batch(tmp_path: Path) -> None:
    path = tmp_path / "bridge.sock"
    config, _safety, backend, stop, thread = _start_server(path, deadman=1.0)
    raw = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    raw.settimeout(0.5)
    boot = new_boot_id()
    try:
        raw.connect(str(path))
        assert decode_packet(raw.recv(MAX_PACKET_BYTES)).kind == "hello"
        raw.sendall(encode_packet("hello", boot, 0, {}, sent_monotonic_s=1.0))
        assert decode_packet(raw.recv(MAX_PACKET_BYTES)).kind == "telemetry"

        for timestamp in range(10):
            backend.sample = replace(backend.sample, timestamp_s=200.0 + timestamp)
            time.sleep(0.01)
        raw.setblocking(False)
        with pytest.raises(BlockingIOError):
            raw.recv(MAX_PACKET_BYTES)

        backend.sample = replace(backend.sample, timestamp_s=999.0)
        raw.settimeout(0.5)
        raw.sendall(
            encode_packet("heartbeat", boot, 1, {}, sent_monotonic_s=2.0)
        )
        packet = decode_packet(raw.recv(MAX_PACKET_BYTES))
        assert packet.kind == "telemetry"
        assert sample_from_payload(packet.payload["sample"]).timestamp_s == 999.0
    finally:
        raw.close()
        _stop_server(stop, thread)


def test_remembered_client_boots_are_bounded(tmp_path: Path) -> None:
    config, safety = _settings(tmp_path / "unused.sock")
    server = UnitreeBridgeServer(
        config,
        safety,
        FakeBackend(),
        expected_client_uid=os.getuid(),
    )
    boots = [new_boot_id() for _index in range(MAX_REMEMBERED_CLIENT_BOOTS + 7)]
    for boot in boots:
        server._remember_client_boot(boot)

    assert len(server._seen_client_boots) == MAX_REMEMBERED_CLIENT_BOOTS
    assert boots[0] not in server._seen_client_boots
    assert boots[-1] in server._seen_client_boots


def test_missing_keepalive_stops_motion_within_deadman_plus_one_tick(
    tmp_path: Path,
) -> None:
    path = tmp_path / "bridge.sock"
    deadman = 0.12
    config, safety, backend, stop, thread = _start_server(path, deadman=deadman)
    client = UnitreeIpcClient(config, safety, expected_server_uid=os.getuid())
    try:
        client.start()
        client.set_velocity(0.2, 0.0, 0.0)
        _wait(lambda: any(item[:3] == (0.2, 0.0, 0.0) for item in backend.velocities))
        moving_at = next(
            item[3] for item in backend.velocities if item[:3] == (0.2, 0.0, 0.0)
        )
        _wait(lambda: backend.velocities[-1][:3] == (0.0, 0.0, 0.0))
        stopped_at = backend.velocities[-1][3]
        assert stopped_at - moving_at <= deadman + (1.0 / config.cmd_hz) + 0.04
    finally:
        client.stop()
        _stop_server(stop, thread)


def test_client_disconnect_stops_motion(tmp_path: Path) -> None:
    path = tmp_path / "bridge.sock"
    config, safety, backend, stop, thread = _start_server(path)
    client = UnitreeIpcClient(config, safety, expected_server_uid=os.getuid())
    try:
        client.start()
        client.set_velocity(0.2, 0.0, 0.0)
        _wait(lambda: any(item[:3] == (0.2, 0.0, 0.0) for item in backend.velocities))
        client.stop()
        _wait(lambda: backend.velocities[-1][:3] == (0.0, 0.0, 0.0))
    finally:
        client.stop()
        _stop_server(stop, thread)


def test_server_rejects_unexpected_peer_uid(tmp_path: Path) -> None:
    path = tmp_path / "bridge.sock"
    config, safety = _settings(path)
    backend = FakeBackend()
    stop = threading.Event()
    server = UnitreeBridgeServer(
        config,
        safety,
        backend,
        expected_client_uid=os.getuid() + 1,
    )
    thread = threading.Thread(target=server.serve_forever, args=(stop,), daemon=True)
    thread.start()
    _wait(lambda: path.exists())
    raw = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    raw.settimeout(0.5)
    try:
        raw.connect(str(path))
        assert raw.recv(MAX_PACKET_BYTES) == b""
    finally:
        raw.close()
        _stop_server(stop, thread)


def test_client_rejects_nonfinite_and_out_of_range_velocity(tmp_path: Path) -> None:
    config, safety = _settings(tmp_path / "unused.sock")
    client = UnitreeIpcClient(config, safety, expected_server_uid=os.getuid())
    with pytest.raises(UnitreeIpcProtocolError, match="finite"):
        client.set_velocity(float("nan"), 0.0, 0.0)
    with pytest.raises(UnitreeIpcProtocolError, match="exceeds"):
        client.set_velocity(safety.max_go2_vx_m_s + 0.1, 0.0, 0.0)


def test_stale_replay_and_parse_fault_fail_safe(tmp_path: Path) -> None:
    for fault in ("replay", "parse"):
        path = tmp_path / f"{fault}.sock"
        config, _safety, backend, stop, thread = _start_server(path)
        raw = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            raw.connect(str(path))
            assert decode_packet(raw.recv(MAX_PACKET_BYTES)).kind == "hello"
            boot = new_boot_id()
            raw.sendall(encode_packet("hello", boot, 0, {}, sent_monotonic_s=1.0))
            command = encode_packet(
                "command",
                boot,
                1,
                {"name": "set_velocity", "vx": 0.2, "vy": 0.0, "wz": 0.0},
                sent_monotonic_s=1.1,
            )
            raw.sendall(command)
            _wait(lambda: any(item[:3] == (0.2, 0.0, 0.0) for item in backend.velocities))
            raw.sendall(command if fault == "replay" else b"{")
            _wait(lambda: backend.velocities[-1][:3] == (0.0, 0.0, 0.0))
        finally:
            raw.close()
            _stop_server(stop, thread)


def test_client_boot_identity_cannot_be_replayed_after_reconnect(
    tmp_path: Path,
) -> None:
    path = tmp_path / "bridge.sock"
    _config, _safety, _backend, stop, thread = _start_server(path)
    boot = new_boot_id()

    def connect_and_read_hello() -> socket.socket:
        raw = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        raw.settimeout(0.5)
        raw.connect(str(path))
        assert decode_packet(raw.recv(MAX_PACKET_BYTES)).kind == "hello"
        return raw

    first = connect_and_read_hello()
    first.sendall(encode_packet("hello", boot, 0, {}, sent_monotonic_s=1.0))
    assert decode_packet(first.recv(MAX_PACKET_BYTES)).kind == "telemetry"
    first.close()
    second = connect_and_read_hello()
    try:
        second.sendall(encode_packet("hello", boot, 0, {}, sent_monotonic_s=2.0))
        assert second.recv(MAX_PACKET_BYTES) == b""
    finally:
        second.close()
        _stop_server(stop, thread)


def test_robot_main_import_does_not_load_unitree_or_create_a_ros_context() -> None:
    source_root = str(Path(__file__).resolve().parents[1] / "src")
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        item for item in (source_root, environment.get("PYTHONPATH", "")) if item
    )
    code = (
        "import sys; import elesim_robot.main; "
        "assert 'elesim_robot.go2.unitree_ros2_bridge' not in sys.modules; "
        "assert 'elesim_robot.go2.unitree_bridge_daemon' not in sys.modules"
    )
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=environment,
        text=True,
        capture_output=True,
        check=False,
        timeout=10.0,
    )
    assert result.returncode == 0, result.stderr
