"""Small role-specific deployment configs for distributed processes."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class RuntimeRoleConfig:
    role: str
    endpoint_id: str
    server_endpoint: str
    bind_endpoint: str = ""
    rpc_endpoint: str = ""
    active_target: str = ""
    camera_enabled: bool = False
    streams: dict[str, str] = field(default_factory=dict)
    router_client_secret_file: str = ""
    router_server_public_file: str = ""
    media_client_secret_file: str = ""
    allow_insecure_remote: bool = False


def load_runtime_role_config(path: str | Path) -> RuntimeRoleConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ValueError(f"{source}: schema_version must be 2")
    runtime = raw.get("runtime")
    if not isinstance(runtime, dict):
        raise ValueError(f"{source}: runtime must be an object")
    streams = runtime.get("streams", {})
    if not isinstance(streams, dict):
        raise ValueError(f"{source}: runtime.streams must be an object")
    role = str(runtime.get("role", "")).strip()
    endpoint_id = str(runtime.get("endpoint_id", "")).strip()
    if role not in {"controller", "robot", "simulator", "ui"}:
        raise ValueError(f"{source}: invalid runtime role {role!r}")
    if not endpoint_id:
        raise ValueError(f"{source}: endpoint_id is required")
    security = raw.get("security", {})
    if not isinstance(security, dict):
        raise ValueError(f"{source}: security must be an object")

    def resolved(value: object) -> str:
        text = str(value or "").strip()
        if not text:
            return ""
        candidate = Path(text).expanduser()
        return str(candidate if candidate.is_absolute() else (source.parent / candidate).resolve())

    return RuntimeRoleConfig(
        role=role,
        endpoint_id=endpoint_id,
        server_endpoint=str(runtime.get("server_endpoint", "tcp://127.0.0.1:5558")),
        bind_endpoint=str(runtime.get("bind_endpoint", "")),
        rpc_endpoint=str(runtime.get("rpc_endpoint", "")),
        active_target=str(runtime.get("active_target", "")),
        camera_enabled=bool(runtime.get("camera_enabled", False)),
        streams={str(key): str(value) for key, value in streams.items()},
        router_client_secret_file=resolved(security.get("router_client_secret_file")),
        router_server_public_file=resolved(security.get("router_server_public_file")),
        media_client_secret_file=resolved(security.get("media_client_secret_file")),
        allow_insecure_remote=bool(security.get("allow_insecure_remote", False)),
    )
