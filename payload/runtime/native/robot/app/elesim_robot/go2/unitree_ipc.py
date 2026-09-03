"""Robot-side adapter and bridge-side server for the local Unitree boundary."""

from __future__ import annotations

import os
import pwd
import socket
import stat
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from elesim_robot.config import SafetyConfig
from elesim_robot.go2.config import Go2HardwareConfig
from elesim_robot.go2.odom_parser import OdomSample
from elesim_robot.go2.unitree_ipc_protocol import (
    UnitreeIpcPacket,
    UnitreeIpcProtocolError,
    encode_packet,
    new_boot_id,
    peer_credentials,
    receive_packet,
    sample_from_payload,
    sample_to_payload,
    validate_command,
)


MAX_REMEMBERED_CLIENT_BOOTS = 512


def resolve_user_uid(user: str) -> int:
    name = str(user).strip()
    if not name:
        raise ValueError("IPC peer user must not be empty")
    try:
        return int(pwd.getpwnam(name).pw_uid)
    except KeyError as exc:
        raise RuntimeError(f"configured IPC peer user does not exist: {name}") from exc


def _velocity_limits(safety: SafetyConfig) -> tuple[float, float, float]:
    return (
        float(safety.max_go2_vx_m_s),
        float(safety.max_go2_vy_m_s),
        float(safety.max_go2_wz_rad_s),
    )


class UnitreeIpcClient:
    """Synchronous RobotRuntime adapter; this class never imports ROS or Unitree."""

    def __init__(
        self,
        config: Go2HardwareConfig,
        safety: SafetyConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
        expected_server_uid: Optional[int] = None,
    ) -> None:
        self._config = config
        self._safety = safety
        self._clock = clock
        self._expected_server_uid = expected_server_uid
        self._socket: Optional[socket.socket] = None
        self._boot_id = ""
        self._send_seq = 0
        self._server_boot_id = ""
        self._server_seq = -1
        self._last_heartbeat_at = 0.0
        self._latest: Optional[OdomSample] = None

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def start(self) -> None:
        if self._socket is not None:
            return
        expected_uid = (
            resolve_user_uid(self._config.ipc_bridge_user)
            if self._expected_server_uid is None
            else int(self._expected_server_uid)
        )
        self._boot_id = new_boot_id()
        self._send_seq = 0
        self._server_boot_id = ""
        self._server_seq = -1
        self._latest = None
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        sock.settimeout(max(1.0, float(self._safety.command_deadman_s)))
        try:
            sock.connect(str(self._config.ipc_socket_path))
            credentials = peer_credentials(sock)
            if credentials.uid != expected_uid:
                raise PermissionError(
                    "Unitree IPC server uid mismatch: "
                    f"expected {expected_uid}, received {credentials.uid}"
                )
            hello = encode_packet(
                "hello",
                self._boot_id,
                0,
                {},
                sent_monotonic_s=self._clock(),
            )
            sock.sendall(hello)
            packet = receive_packet(sock)
            if packet is None or packet.kind != "hello":
                raise UnitreeIpcProtocolError("bridge did not send a valid hello")
            self._server_boot_id = packet.boot_id
            self._server_seq = packet.seq
            sock.setblocking(False)
        except Exception:
            sock.close()
            raise
        self._socket = sock
        self._last_heartbeat_at = self._clock()

    def stop(self) -> None:
        sock = self._socket
        if sock is None:
            return
        try:
            self._send_command({"name": "stop"})
        except Exception:
            pass
        finally:
            self._socket = None
            sock.close()

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        payload = validate_command(
            {"name": "set_velocity", "vx": vx, "vy": vy, "wz": wz},
            max_velocity=_velocity_limits(self._safety),
        )
        if self._socket is None and all(
            abs(float(payload[key])) <= 1e-12 for key in ("vx", "vy", "wz")
        ):
            return
        self._send_command(payload)

    def call_sport_pose(self, pose: str) -> None:
        payload = validate_command(
            {"name": "sport_pose", "pose": pose},
            max_velocity=_velocity_limits(self._safety),
        )
        self._send_command(payload)

    def set_obstacles_avoid(self, enabled: bool) -> None:
        payload = validate_command(
            {"name": "obstacles_avoid", "enabled": enabled},
            max_velocity=_velocity_limits(self._safety),
        )
        self._send_command(payload)

    def tick_cmd(self, _now_s: Optional[float] = None) -> None:
        sock = self._require_socket()
        try:
            for _index in range(256):
                packet = receive_packet(sock)
                if packet is None:
                    break
                self._handle_server_packet(packet)
            now = self._clock()
            if now - self._last_heartbeat_at >= float(
                self._config.ipc_heartbeat_interval_s
            ):
                self._send("heartbeat", {})
                self._last_heartbeat_at = now
        except Exception:
            self._disconnect()
            raise

    def latest_state(self) -> Optional[OdomSample]:
        return self._latest

    def maybe_log_status(self, _now_s: Optional[float] = None) -> None:
        return

    def _handle_server_packet(self, packet: UnitreeIpcPacket) -> None:
        if packet.boot_id != self._server_boot_id:
            raise UnitreeIpcProtocolError("bridge boot identity changed in-session")
        if packet.seq <= self._server_seq:
            raise UnitreeIpcProtocolError("stale bridge packet sequence")
        self._server_seq = packet.seq
        if packet.kind == "telemetry":
            if set(packet.payload) != {"sample"}:
                raise UnitreeIpcProtocolError("invalid telemetry payload")
            self._latest = sample_from_payload(packet.payload["sample"])
            return
        if packet.kind == "error":
            reason = str(packet.payload.get("reason", "bridge error"))[:512]
            raise RuntimeError(reason)
        raise UnitreeIpcProtocolError(
            f"unexpected bridge packet kind: {packet.kind}"
        )

    def _send_command(self, payload: dict[str, object]) -> None:
        self._send("command", payload)

    def _send(self, kind: str, payload: dict[str, object]) -> None:
        sock = self._require_socket()
        if self._send_seq >= (1 << 63) - 1:
            raise UnitreeIpcProtocolError("client sequence exhausted")
        self._send_seq += 1
        packet = encode_packet(
            kind,
            self._boot_id,
            self._send_seq,
            payload,
            sent_monotonic_s=self._clock(),
        )
        try:
            sock.sendall(packet)
        except Exception:
            self._disconnect()
            raise

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise ConnectionError("Unitree IPC is not connected")
        return self._socket

    def _disconnect(self) -> None:
        sock, self._socket = self._socket, None
        if sock is not None:
            sock.close()


class UnitreeBridgeServer:
    """Single-client bridge server with local credential and deadman enforcement."""

    def __init__(
        self,
        config: Go2HardwareConfig,
        safety: SafetyConfig,
        backend: Any,
        *,
        clock: Callable[[], float] = time.monotonic,
        expected_client_uid: Optional[int] = None,
    ) -> None:
        self._config = config
        self._safety = safety
        self._backend = backend
        self._clock = clock
        self._expected_client_uid = expected_client_uid
        self._boot_id = new_boot_id()
        self._send_seq = -1
        self._seen_client_boots: set[str] = set()
        self._client_boot_history: deque[str] = deque(
            maxlen=MAX_REMEMBERED_CLIENT_BOOTS
        )
        self._listener: Optional[socket.socket] = None
        self._created_socket = False

    def serve_forever(self, stop_event: Optional[threading.Event] = None) -> None:
        stopping = stop_event or threading.Event()
        expected_uid = (
            resolve_user_uid(self._config.ipc_robot_user)
            if self._expected_client_uid is None
            else int(self._expected_client_uid)
        )
        self._backend.start()
        try:
            self._listener = self._open_listener()
            self._listener.settimeout(0.25)
            while not stopping.is_set():
                try:
                    connection, _address = self._listener.accept()
                except socket.timeout:
                    continue
                credentials = peer_credentials(connection)
                if credentials.uid != expected_uid:
                    print(
                        "[unitree_ipc] rejected client uid "
                        f"{credentials.uid}; expected {expected_uid}",
                        flush=True,
                    )
                    connection.close()
                    continue
                self._serve_client(connection, stopping)
        finally:
            try:
                self._safe_stop()
            finally:
                try:
                    self._backend.stop()
                finally:
                    if self._listener is not None:
                        self._listener.close()
                        self._listener = None
                    self._remove_owned_socket()

    def _serve_client(
        self,
        connection: socket.socket,
        stop_event: threading.Event,
    ) -> None:
        tick_period = 1.0 / max(float(self._config.cmd_hz), 1.0)
        connection.settimeout(tick_period)
        client_boot = ""
        client_seq = -1
        last_valid_at = self._clock()
        try:
            self._send(connection, "hello", {})
            while not stop_event.is_set():
                packets: list[UnitreeIpcPacket] = []
                try:
                    first = receive_packet(connection)
                    if first is not None:
                        packets.append(first)
                        connection.setblocking(False)
                        for _index in range(255):
                            packet = receive_packet(connection)
                            if packet is None:
                                break
                            packets.append(packet)
                except socket.timeout:
                    pass
                finally:
                    connection.settimeout(tick_period)

                for packet in packets:
                    if not client_boot:
                        if packet.kind != "hello" or packet.seq != 0:
                            raise UnitreeIpcProtocolError(
                                "first client packet must be hello sequence zero"
                            )
                        if packet.boot_id in self._seen_client_boots:
                            raise UnitreeIpcProtocolError(
                                "client boot identity cannot be replayed"
                            )
                        client_boot = packet.boot_id
                        client_seq = packet.seq
                        self._remember_client_boot(client_boot)
                        last_valid_at = self._clock()
                        continue
                    if packet.boot_id != client_boot:
                        raise UnitreeIpcProtocolError(
                            "client boot identity changed in-session"
                        )
                    if packet.seq <= client_seq:
                        raise UnitreeIpcProtocolError("stale client packet sequence")
                    client_seq = packet.seq
                    if packet.kind == "heartbeat":
                        last_valid_at = self._clock()
                    elif packet.kind == "command":
                        command = validate_command(
                            packet.payload,
                            max_velocity=_velocity_limits(self._safety),
                        )
                        self._dispatch(command)
                        last_valid_at = self._clock()
                    else:
                        raise UnitreeIpcProtocolError(
                            f"unexpected client packet kind: {packet.kind}"
                        )

                now = self._clock()
                if now - last_valid_at > float(self._safety.command_deadman_s):
                    raise TimeoutError("Unitree IPC keepalive expired")
                self._backend.tick_cmd(time.time())
                # Telemetry is request-coupled: at most one latest sample is
                # emitted for one inbound batch. A client that stops reading
                # therefore cannot accumulate autonomous telemetry backlog.
                if packets and client_boot:
                    sample = self._backend.latest_state()
                    if sample is not None:
                        self._try_send_telemetry(connection, sample)
        except (ConnectionError, OSError, TimeoutError, UnitreeIpcProtocolError) as exc:
            print(f"[unitree_ipc] client stopped: {exc}", flush=True)
            self._safe_stop()
        finally:
            connection.close()

    def _dispatch(self, command: dict[str, object]) -> None:
        name = str(command["name"])
        if name == "set_velocity":
            self._backend.set_velocity(
                float(command["vx"]),
                float(command["vy"]),
                float(command["wz"]),
            )
        elif name == "sport_pose":
            self._backend.call_sport_pose(str(command["pose"]))
        elif name == "obstacles_avoid":
            self._backend.set_obstacles_avoid(bool(command["enabled"]))
        elif name == "stop":
            self._safe_stop()
        else:  # validate_command owns the exhaustive allowlist.
            raise UnitreeIpcProtocolError(f"unsupported command: {name}")

    def _remember_client_boot(self, boot_id: str) -> None:
        if len(self._client_boot_history) == MAX_REMEMBERED_CLIENT_BOOTS:
            expired = self._client_boot_history.popleft()
            self._seen_client_boots.discard(expired)
        self._client_boot_history.append(boot_id)
        self._seen_client_boots.add(boot_id)

    def _safe_stop(self) -> None:
        self._backend.set_velocity(0.0, 0.0, 0.0)
        self._backend.tick_cmd(time.time())

    def _send(
        self,
        connection: socket.socket,
        kind: str,
        payload: dict[str, object],
    ) -> None:
        self._send_seq += 1
        connection.sendall(
            encode_packet(
                kind,
                self._boot_id,
                self._send_seq,
                payload,
                sent_monotonic_s=self._clock(),
            )
        )

    def _try_send_telemetry(
        self,
        connection: socket.socket,
        sample: OdomSample,
    ) -> bool:
        previous_timeout = connection.gettimeout()
        try:
            connection.setblocking(False)
            self._send(connection, "telemetry", {"sample": sample_to_payload(sample)})
            return True
        except BlockingIOError:
            return False
        finally:
            connection.settimeout(previous_timeout)

    def _open_listener(self) -> socket.socket:
        path = Path(self._config.ipc_socket_path)
        path.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
        try:
            existing = path.lstat()
        except FileNotFoundError:
            pass
        else:
            if not stat.S_ISSOCK(existing.st_mode) or existing.st_uid != os.geteuid():
                raise RuntimeError(f"refusing to replace unowned IPC path: {path}")
            path.unlink()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            listener.bind(str(path))
            os.chmod(path, 0o660)
            listener.listen(1)
        except Exception:
            listener.close()
            raise
        self._created_socket = True
        return listener

    def _remove_owned_socket(self) -> None:
        if not self._created_socket:
            return
        path = Path(self._config.ipc_socket_path)
        try:
            current = path.lstat()
            if stat.S_ISSOCK(current.st_mode) and current.st_uid == os.geteuid():
                path.unlink()
        except FileNotFoundError:
            pass
        finally:
            self._created_socket = False


def create_go2_client_if_enabled(
    config: Go2HardwareConfig,
    safety: SafetyConfig,
    *,
    use_go2: bool,
) -> Optional[UnitreeIpcClient]:
    if not config.is_active(use_go2=use_go2):
        return None
    return UnitreeIpcClient(config, safety)


__all__ = [
    "MAX_REMEMBERED_CLIENT_BOOTS",
    "UnitreeBridgeServer",
    "UnitreeIpcClient",
    "create_go2_client_if_enabled",
    "resolve_user_uid",
]
