"""Generate deployment-owned YAML from one explicit network profile."""

from __future__ import annotations

import ipaddress
import tempfile
from pathlib import Path
from typing import Any, Mapping

import yaml

from .state import InstallState


GENERATED_CONFIG = "installed.yaml"
GENERATED_RUNTIME = "runtime.installed.yaml"
GENERATED_APP = "app.installed.yaml"


def tcp_endpoint(host: str, port: int) -> str:
    value = str(host).strip()
    if value.startswith("[") and value.endswith("]"):
        value = value[1:-1]
    rendered = f"[{value}]" if ":" in value else value
    return f"tcp://{rendered}:{int(port)}"


def host_is_loopback(host: str) -> bool:
    value = str(host).strip().lower().removeprefix("[").removesuffix("]")
    if value == "localhost":
        return True
    try:
        return ipaddress.ip_address(value).is_loopback
    except ValueError:
        return False


def role_directory(state: InstallState, role: str) -> Path:
    return state.prefix_path / "roles" / role


def generated_config_path(state: InstallState, role: str) -> Path:
    name = GENERATED_RUNTIME if role in {"controller", "simulator"} else GENERATED_CONFIG
    return role_directory(state, role) / "config" / name


def generated_app_config_path(state: InstallState, role: str) -> Path:
    if role != "simulator":
        raise ValueError(f"{role!r} does not have a generated application config")
    return role_directory(state, role) / "config" / GENERATED_APP


def credentials_for_role(state: InstallState, role: str) -> tuple[Path, ...]:
    root = state.security.root
    if state.security.mode != "curve" or root is None:
        return ()
    router_public = root / "curve/router/router.key"
    if role == "router":
        values = (
            root / "curve/router/router.key_secret",
            root / "curve/authorized",
            root / "curve/endpoints.yaml",
        )
        return values + ((root / "turn.secret",) if state.network.turn_urls else ())
    if role == "controller":
        return (
            root / f"curve/clients/{state.network.controller_id}.key_secret",
            router_public,
        )
    if role == "ui":
        return (root / "curve/clients/ui-main.key_secret", router_public)
    if role == "simulator":
        return (
            root / f"curve/clients/{state.network.simulator_id}.key_secret",
            router_public,
            root / "curve/media/simulator-media.key_secret",
            root / "curve/media-authorized",
        )
    if role == "robot":
        return (
            root / "curve/clients/robot-go2.key_secret",
            router_public,
            root / "curve/media/robot-media.key_secret",
            root / "curve/media-authorized",
        )
    raise ValueError(f"unknown role: {role}")


def missing_credentials(state: InstallState) -> tuple[Path, ...]:
    missing: list[Path] = []
    for role in state.roles:
        for path in credentials_for_role(state, role):
            if not path.exists() and path not in missing:
                missing.append(path)
    return tuple(missing)


def generate_role_configs(state: InstallState) -> dict[str, Path]:
    """Write only the installed copies; source-tree defaults remain untouched."""

    state.validate()
    written: dict[str, Path] = {}
    for role in state.roles:
        destination = generated_config_path(state, role)
        if role == "router":
            payload = _router_config(state, destination.parent / "default.yaml")
        elif role == "controller":
            payload = _controller_config(state, destination.parent / "runtime.yaml")
        elif role == "ui":
            payload = _ui_config(state, destination.parent / "default.yaml")
        elif role == "simulator":
            payload = _simulator_config(state, destination.parent / "runtime.yaml")
            _write_yaml(
                generated_app_config_path(state, role),
                _simulator_app_config(state),
            )
        elif role == "robot":
            payload = _robot_config(state, destination.parent / "default.yaml")
        else:
            raise ValueError(f"unknown role: {role}")
        _write_yaml(destination, payload)
        written[role] = destination
    return written


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"설정 원본이 없습니다: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"{path}: YAML root가 object가 아닙니다")
    return dict(raw)


def _write_yaml(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = yaml.safe_dump(dict(payload), sort_keys=False, allow_unicode=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        handle.write(rendered)
        temporary = Path(handle.name)
    temporary.replace(path)


def _security_client(state: InstallState, endpoint_id: str) -> dict[str, Any]:
    root = state.security.root
    if state.security.mode == "curve" and root is not None:
        return {
            "router_client_secret_file": str(root / f"curve/clients/{endpoint_id}.key_secret"),
            "router_server_public_file": str(root / "curve/router/router.key"),
            "allow_insecure_remote": False,
        }
    return {
        "router_client_secret_file": "",
        "router_server_public_file": "",
        "allow_insecure_remote": state.security.allow_insecure_remote,
    }


def _router_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    remote = not host_is_loopback(state.network.router_host)
    raw["router"] = {
        "bind_endpoint": tcp_endpoint("0.0.0.0" if remote else "127.0.0.1", state.network.router_port),
        "heartbeat_timeout_s": float((raw.get("router") or {}).get("heartbeat_timeout_s", 3.5)),
    }
    root = state.security.root
    if state.security.mode == "curve" and root is not None:
        raw["security"] = {
            "curve_server_secret_file": str(root / "curve/router/router.key_secret"),
            "curve_public_keys_dir": str(root / "curve/authorized"),
            "endpoint_registry_file": str(root / "curve/endpoints.yaml"),
            "allow_insecure_remote": False,
        }
    else:
        raw["security"] = {
            "curve_server_secret_file": "",
            "curve_public_keys_dir": "",
            "endpoint_registry_file": "",
            "allow_insecure_remote": state.security.allow_insecure_remote,
        }
    turn = dict(raw.get("turn") or {})
    turn["urls"] = list(state.network.turn_urls)
    turn["static_auth_secret_file"] = (
        str(root / "turn.secret") if root is not None and state.network.turn_urls else ""
    )
    raw["turn"] = turn
    return raw


def _controller_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.update(
        {
            "role": "controller",
            "endpoint_id": state.network.controller_id,
            "server_endpoint": tcp_endpoint(state.network.router_host, state.network.router_port),
            "active_target": state.network.simulator_id,
        }
    )
    raw["runtime"] = runtime
    security = _security_client(state, state.network.controller_id)
    root = state.security.root
    security["media_client_secret_file"] = (
        str(root / f"curve/clients/{state.network.controller_id}.key_secret")
        if state.security.mode == "curve" and root is not None
        else ""
    )
    raw["security"] = security
    return raw


def _ui_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.update(
        {
            "endpoint_id": "ui-main",
            "controller_id": state.network.controller_id,
            "simulator_id": state.network.simulator_id,
            "server_endpoint": tcp_endpoint(state.network.router_host, state.network.router_port),
        }
    )
    raw["runtime"] = runtime
    raw["security"] = _security_client(state, "ui-main")
    return raw


def _simulator_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    remote_media = not host_is_loopback(state.network.advertise_host)
    raw["runtime"] = {
        "role": "simulator",
        "endpoint_id": state.network.simulator_id,
        "server_endpoint": tcp_endpoint(state.network.router_host, state.network.router_port),
        "streams": {
            "rgbd_bind": tcp_endpoint("0.0.0.0" if remote_media else "127.0.0.1", state.network.rgbd_port),
            "rgbd_advertise": tcp_endpoint(state.network.advertise_host, state.network.rgbd_port),
            "observer_bind": "tcp://127.0.0.1:5569",
            "observer_advertise": "tcp://127.0.0.1:5569",
        },
    }
    security = _security_client(state, state.network.simulator_id)
    root = state.security.root
    security.update(
        {
            "media_server_secret_file": (
                str(root / "curve/media/simulator-media.key_secret")
                if state.security.mode == "curve" and root is not None
                else ""
            ),
            "media_client_public_keys_dir": (
                str(root / "curve/media-authorized")
                if state.security.mode == "curve" and root is not None
                else ""
            ),
        }
    )
    raw["security"] = security
    return raw


def _simulator_app_config(state: InstallState) -> dict[str, Any]:
    base = (
        "config.pc.yaml"
        if state.profile == "local-sim" and state.install_mode == "native"
        else "config.remote.yaml"
    )
    return {
        "schema_version": 1,
        "extends": base,
        "simulation": {
            "runtime": {
                "use_gpu": state.compute.gpu_mode != "cpu",
            }
        },
    }


def _robot_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.update(
        {
            "endpoint_id": "robot-go2",
            "server_endpoint": tcp_endpoint(state.network.router_host, state.network.router_port),
        }
    )
    raw["runtime"] = runtime
    camera = dict(raw.get("camera") or {})
    remote_media = not host_is_loopback(state.network.advertise_host)
    camera.update(
        {
            "bind": tcp_endpoint("0.0.0.0" if remote_media else "127.0.0.1", state.network.rgbd_port),
            "advertise": tcp_endpoint(state.network.advertise_host, state.network.rgbd_port),
        }
    )
    raw["camera"] = camera
    security = _security_client(state, "robot-go2")
    root = state.security.root
    security.update(
        {
            "media_server_secret_file": (
                str(root / "curve/media/robot-media.key_secret")
                if state.security.mode == "curve" and root is not None
                else ""
            ),
            "media_client_public_keys_dir": (
                str(root / "curve/media-authorized")
                if state.security.mode == "curve" and root is not None
                else ""
            ),
        }
    )
    raw["security"] = security
    return raw


__all__ = [
    "GENERATED_CONFIG",
    "GENERATED_APP",
    "GENERATED_RUNTIME",
    "credentials_for_role",
    "generate_role_configs",
    "generated_app_config_path",
    "generated_config_path",
    "host_is_loopback",
    "missing_credentials",
    "role_directory",
    "tcp_endpoint",
]
