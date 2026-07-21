"""Router deployment configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import yaml

from elesim_protocol import endpoint_is_loopback


def _path(base: Path, value: object) -> Path | None:
    text = str(value or "").strip()
    if not text:
        return None
    path = Path(text).expanduser()
    return path if path.is_absolute() else (base / path).resolve()


@dataclass(frozen=True)
class RouterConfig:
    bind_endpoint: str = "tcp://127.0.0.1:5558"
    heartbeat_timeout_s: float = 3.5
    curve_server_secret_file: Path | None = None
    curve_public_keys_dir: Path | None = None
    endpoint_registry_file: Path | None = None
    allow_insecure_remote: bool = False
    turn_urls: tuple[str, ...] = ()
    turn_static_auth_secret_file: Path | None = None
    turn_credential_ttl_s: int = 3600
    turn_refresh_before_s: int = 600

    @property
    def curve_enabled(self) -> bool:
        return self.curve_server_secret_file is not None

    def validate(self) -> "RouterConfig":
        if not self.bind_endpoint:
            raise ValueError("router bind endpoint must not be empty")
        if not self.curve_enabled and not endpoint_is_loopback(self.bind_endpoint):
            if not self.allow_insecure_remote:
                raise ValueError("non-loopback router bind requires CURVE")
        curve_fields = (
            self.curve_server_secret_file,
            self.curve_public_keys_dir,
            self.endpoint_registry_file,
        )
        if self.curve_enabled and any(value is None for value in curve_fields):
            raise ValueError("CURVE requires server key, authorized key directory and endpoint registry")
        if not self.curve_enabled and any(value is not None for value in curve_fields):
            raise ValueError("CURVE security fields must be configured together")
        turn_fields = bool(self.turn_urls), self.turn_static_auth_secret_file is not None
        if turn_fields[0] != turn_fields[1]:
            raise ValueError("TURN URLs and static auth secret must be configured together")
        if not 900 <= int(self.turn_credential_ttl_s) <= 86_400:
            raise ValueError("TURN credential TTL must be within 900..86400 seconds")
        if not 60 <= int(self.turn_refresh_before_s) < int(self.turn_credential_ttl_s):
            raise ValueError("TURN refresh margin must be shorter than credential TTL")
        return self


def load_config(path: str | Path) -> RouterConfig:
    source = Path(path).expanduser().resolve()
    raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping) or raw.get("schema_version") != 2:
        raise ValueError("router config schema_version must be 2")
    router = raw.get("router") or {}
    security = raw.get("security") or {}
    turn = raw.get("turn") or {}
    if not all(isinstance(value, Mapping) for value in (router, security, turn)):
        raise ValueError("router, security and turn config sections must be objects")
    base = source.parent
    return RouterConfig(
        bind_endpoint=str(router.get("bind_endpoint", "tcp://127.0.0.1:5558")),
        heartbeat_timeout_s=float(router.get("heartbeat_timeout_s", 3.5)),
        curve_server_secret_file=_path(base, security.get("curve_server_secret_file")),
        curve_public_keys_dir=_path(base, security.get("curve_public_keys_dir")),
        endpoint_registry_file=_path(base, security.get("endpoint_registry_file")),
        allow_insecure_remote=bool(security.get("allow_insecure_remote", False)),
        turn_urls=tuple(str(url) for url in turn.get("urls", ())),
        turn_static_auth_secret_file=_path(base, turn.get("static_auth_secret_file")),
        turn_credential_ttl_s=int(turn.get("credential_ttl_s", 3600)),
        turn_refresh_before_s=int(turn.get("refresh_before_s", 600)),
    ).validate()


__all__ = ["RouterConfig", "load_config"]
