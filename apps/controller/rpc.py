"""Local UI RPC. The control agent remains the owner of workflow state."""

from __future__ import annotations

import dataclasses
import threading
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Callable

import numpy as np
import zmq

from engine.core.protocol import ControlU, SimQ


SERVICE_CALLS = frozenset(
    {
        "apply_partial_control_u", "capture_perception_frame", "disconnect_device",
        "extend_arm_controls", "home_controls", "load_sag_model", "refresh_host_state",
        "refresh_perception_capture", "request_ports", "reset_simulation", "send_claw_command",
        "send_current_target_meta", "send_go2_obstacles_avoid", "send_go2_sport_pose",
        "send_go2_velocity", "send_ready_pose_meta", "send_sag_model_meta", "send_sim_target_xyz",
        "set_device", "set_display_offset", "start_demo4_stop_and_grasp",
        "start_gaze_stabilizer_standing", "start_gaze_stabilizer_walking", "start_ik_solve",
        "start_lji_grasp_only", "start_mobile_gaze_lji_pick_e2e", "start_perception_capture",
        "stop_gaze_stabilizer", "stop_perception_capture", "stop_pick_e2e",
        "toggle_perception_recording", "torque_off", "torque_on",
        "update_gaze_stabilizer_config", "update_perception_config",
        "send_sim_camera_input", "select_endpoint",
    }
)
SERVICE_VALUES = frozenset(
    {
        "_gaze_cfg", "_pick_config_effective", "control_mapping", "current_control_u",
        "current_host_state", "gaze_config", "has_client", "pick_e2e_running",
        "available_endpoints", "active_endpoint",
    }
)
STATE_CALLS = frozenset(
    {
        "clear_ik_status", "offset_values", "set_claw_closed", "set_mock_object_preferred_dir",
        "set_mock_object_world_xyz", "set_paused", "set_perception_record_overlay", "set_target",
        "set_target_dir", "set_torque_lock_bypass",
    }
)


def _jsonable(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Path):
        return {"__type__": "Path", "value": str(value)}
    if dataclasses.is_dataclass(value):
        return {
            "__type__": type(value).__name__,
            "value": {field.name: _jsonable(getattr(value, field.name)) for field in dataclasses.fields(value)},
        }
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "__dict__"):
        return {
            "__type__": type(value).__name__,
            "value": {
                str(key): _jsonable(item)
                for key, item in vars(value).items()
                if not str(key).startswith("_") and _is_json_candidate(item)
            },
        }
    return str(value)


def _is_json_candidate(value: Any) -> bool:
    return value is None or isinstance(value, (bool, int, float, str, Path, list, tuple, dict, np.ndarray)) or dataclasses.is_dataclass(value)


def _decode(value: Any) -> Any:
    if isinstance(value, list):
        return [_decode(item) for item in value]
    if not isinstance(value, dict):
        return value
    kind = value.get("__type__")
    raw = value.get("value")
    if kind == "Path":
        return Path(str(raw))
    if kind == "ControlU" and isinstance(raw, dict):
        return ControlU(**{key: float(item) for key, item in raw.items()})
    if kind == "SimQ" and isinstance(raw, dict):
        return SimQ(**{key: float(item) for key, item in raw.items()})
    if kind and isinstance(raw, dict):
        return SimpleNamespace(**{key: _decode(item) for key, item in raw.items()})
    return {key: _decode(item) for key, item in value.items()}


def _state_snapshot(state: Any) -> dict[str, Any]:
    return {
        key: _jsonable(value)
        for key, value in vars(state).items()
        if not key.startswith("_") and _is_json_candidate(value)
    }


class ControlRpcServer:
    def __init__(self, endpoint: str, state: Any, service: Any) -> None:
        self.endpoint = str(endpoint)
        self.state = state
        self.service = service
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(self.endpoint)
        self.stop_event = threading.Event()

    def run(self) -> None:
        poller = zmq.Poller()
        poller.register(self.socket, zmq.POLLIN)
        try:
            while not self.stop_event.is_set():
                if self.socket not in dict(poller.poll(100)):
                    continue
                request = self.socket.recv_json()
                try:
                    result = self._handle(request)
                    self.socket.send_json({"ok": True, "result": _jsonable(result)})
                except Exception as exc:
                    self.socket.send_json({"ok": False, "error": repr(exc)})
        finally:
            self.socket.close(0)

    def _handle(self, request: dict[str, Any]) -> Any:
        op = str(request.get("op", ""))
        name = str(request.get("name", ""))
        args = [_decode(value) for value in request.get("args", [])]
        kwargs = {key: _decode(value) for key, value in request.get("kwargs", {}).items()}
        if op == "snapshot":
            return _state_snapshot(self.state)
        if op == "service_call" and name in SERVICE_CALLS:
            return getattr(self.service, name)(*args, **kwargs)
        if op == "service_get" and name in SERVICE_VALUES:
            return getattr(self.service, name)
        if op == "state_call" and name in STATE_CALLS:
            return getattr(self.state, name)(*args, **kwargs)
        if op == "state_set" and name and not name.startswith("_"):
            setattr(self.state, name, kwargs.get("value"))
            return None
        raise ValueError(f"unsupported control RPC: {op} {name}")

    def close(self) -> None:
        self.stop_event.set()


class ControlRpcClient:
    def __init__(self, endpoint: str, *, timeout_ms: int = 2000) -> None:
        self.endpoint = str(endpoint)
        self.timeout_ms = int(timeout_ms)
        self.context = zmq.Context.instance()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(self.endpoint)
        self.lock = threading.Lock()

    def request(self, op: str, name: str = "", *args: Any, **kwargs: Any) -> Any:
        payload = {
            "op": op,
            "name": name,
            "args": [_jsonable(value) for value in args],
            "kwargs": {key: _jsonable(value) for key, value in kwargs.items()},
        }
        with self.lock:
            self.socket.send_json(payload)
            reply = self.socket.recv_json()
        if not reply.get("ok"):
            raise RuntimeError(str(reply.get("error", "control RPC failed")))
        return _decode(reply.get("result"))

    def close(self) -> None:
        self.socket.close(0)


class RemotePanelState:
    def __init__(self, rpc: ControlRpcClient) -> None:
        object.__setattr__(self, "_rpc", rpc)
        object.__setattr__(self, "_cache", {})
        self.sync()

    def sync(self) -> None:
        snapshot = self._rpc.request("snapshot")
        object.__getattribute__(self, "_cache").update(snapshot)

    def __getattr__(self, name: str) -> Any:
        if name in STATE_CALLS:
            return lambda *args, **kwargs: self._rpc.request("state_call", name, *args, **kwargs)
        cache = object.__getattribute__(self, "_cache")
        if name in cache:
            return cache[name]
        raise AttributeError(name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        object.__getattribute__(self, "_cache")[name] = value
        self._rpc.request("state_set", name, value=value)


class RemoteControlService:
    def __init__(self, rpc: ControlRpcClient, state: RemotePanelState) -> None:
        self.rpc = rpc
        self.state = state

    def refresh_host_state(self) -> Any:
        result = self.rpc.request("service_call", "refresh_host_state")
        self.state.sync()
        return result

    def close(self) -> None:
        self.rpc.close()

    def __getattr__(self, name: str) -> Any:
        if name in SERVICE_CALLS:
            return lambda *args, **kwargs: self.rpc.request("service_call", name, *args, **kwargs)
        if name in SERVICE_VALUES:
            return self.rpc.request("service_get", name)
        raise AttributeError(name)
