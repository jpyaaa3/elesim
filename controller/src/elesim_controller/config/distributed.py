"""Small role-specific deployment configs for distributed processes."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from elesim_protocol import DdsRuntimeSettings


@dataclass(frozen=True)
class RuntimeRoleConfig:
    role: str
    endpoint_id: str
    dds: DdsRuntimeSettings
    active_target: str = ""
    camera_enabled: bool = False
    streams: dict[str, str] = field(default_factory=dict)


def load_runtime_role_config(path: str | Path) -> RuntimeRoleConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 3:
        raise ValueError(f"{source}: schema_version must be 3")
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
    dds = raw.get("dds")
    if not isinstance(dds, dict):
        raise ValueError(f"{source}: dds must be an object")
    dds_payload = dict(dds)
    vendor_config = str(dds_payload.get("vendor_config", "")).strip()
    if vendor_config:
        candidate = Path(vendor_config).expanduser()
        dds_payload["vendor_config"] = str(
            candidate
            if candidate.is_absolute()
            else (source.parent / candidate).resolve()
        )
    keystore = str(dds_payload.get("keystore", "")).strip()
    if keystore:
        candidate = Path(keystore).expanduser()
        dds_payload["keystore"] = str(
            candidate
            if candidate.is_absolute()
            else (source.parent / candidate).resolve()
        )

    return RuntimeRoleConfig(
        role=role,
        endpoint_id=endpoint_id,
        dds=DdsRuntimeSettings.from_mapping(
            dds_payload,
            endpoint_id=endpoint_id,
        ),
        active_target=str(runtime.get("active_target", "")),
        camera_enabled=bool(runtime.get("camera_enabled", False)),
        streams={str(key): str(value) for key, value in streams.items()},
    )
