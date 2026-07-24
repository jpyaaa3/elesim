"""Role-specific ROS 2/DDS deployment configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import yaml

from elesim_protocol import DdsRuntimeSettings


@dataclass(frozen=True)
class TurnConfig:
    """Simulator-owned managed or external TURN credential configuration."""

    urls: tuple[str, ...] = ()
    realm: str = ""
    static_auth_secret_file: Path | None = None
    credential_file: Path | None = None


@dataclass(frozen=True)
class RuntimeRoleConfig:
    role: str
    endpoint_id: str
    streams: dict[str, str] = field(default_factory=dict)
    dds: DdsRuntimeSettings = field(default_factory=DdsRuntimeSettings)
    turn: TurnConfig = field(default_factory=TurnConfig)


def _section(raw: Mapping[str, Any], name: str, source: Path) -> dict[str, Any]:
    value = raw.get(name, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"{source}: {name} must be an object")
    return dict(value)


def _resolved_path(source: Path, value: object) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    candidate = Path(text).expanduser()
    return str(candidate if candidate.is_absolute() else (source.parent / candidate).resolve())


def load_runtime_role_config(path: str | Path) -> RuntimeRoleConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ValueError(f"{source}: schema_version must be 2")
    runtime = _section(raw, "runtime", source)
    streams = runtime.get("streams", {})
    if not isinstance(streams, Mapping):
        raise ValueError(f"{source}: runtime.streams must be an object")
    role = str(runtime.get("role", "")).strip()
    endpoint_id = str(runtime.get("endpoint_id", "")).strip()
    if role != "simulator":
        raise ValueError(f"{source}: runtime role must be 'simulator', got {role!r}")
    if not endpoint_id:
        raise ValueError(f"{source}: endpoint_id is required")

    dds_raw = _section(raw, "dds", source)
    for key in ("vendor_config", "keystore"):
        if str(dds_raw.get(key, "")).strip():
            dds_raw[key] = _resolved_path(source, dds_raw[key])
    dds = DdsRuntimeSettings.from_mapping(dds_raw, endpoint_id=endpoint_id)

    turn_raw = _section(raw, "turn", source)
    urls_raw = turn_raw.get("urls", ())
    if not isinstance(urls_raw, (list, tuple)):
        raise ValueError(f"{source}: turn.urls must be a list")
    urls = tuple(str(value).strip() for value in urls_raw if str(value).strip())
    if len(urls) > 8:
        raise ValueError(f"{source}: turn.urls may contain at most 8 entries")
    realm = str(turn_raw.get("realm", "")).strip()
    secret_path_text = _resolved_path(source, turn_raw.get("static_auth_secret_file"))
    secret_path = Path(secret_path_text) if secret_path_text else None
    credential_path_text = _resolved_path(source, turn_raw.get("credential_file"))
    credential_path = Path(credential_path_text) if credential_path_text else None
    if secret_path is not None and credential_path is not None:
        raise ValueError(
            f"{source}: managed and external TURN credential files are mutually exclusive"
        )
    if secret_path is not None:
        if not urls or not realm:
            raise ValueError(
                f"{source}: managed TURN requires urls, realm, and static_auth_secret_file"
            )
        if dds.security_profile != "sros2":
            raise ValueError(f"{source}: managed TURN signaling requires DDS security_profile sros2")
    elif credential_path is not None:
        if not urls:
            raise ValueError(f"{source}: external TURN credential_file requires urls")
        if realm:
            raise ValueError(f"{source}: external TURN does not use a managed realm")
    elif urls:
        raise ValueError(
            f"{source}: TURN urls require static_auth_secret_file or credential_file"
        )
    elif realm:
        raise ValueError(f"{source}: TURN realm requires managed TURN credentials")

    return RuntimeRoleConfig(
        role=role,
        endpoint_id=endpoint_id,
        streams={str(key): str(value) for key, value in streams.items()},
        dds=dds,
        turn=TurnConfig(
            urls=urls,
            realm=realm,
            static_auth_secret_file=secret_path,
            credential_file=credential_path,
        ),
    )
