"""Authenticated CURVE public-key to endpoint identity mapping."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Mapping

import yaml


class EndpointIdentityRegistry:
    def __init__(
        self,
        identities: Mapping[
            str,
            tuple[str, str] | Iterable[tuple[str, str]],
        ],
    ) -> None:
        self._identities: dict[str, frozenset[tuple[str, str]]] = {}
        for public_key, configured in identities.items():
            key = str(public_key).strip()
            if len(key) != 40 or any(character.isspace() for character in key):
                raise ValueError("endpoint public keys must be 40-character Z85 values")
            if (
                isinstance(configured, tuple)
                and len(configured) == 2
                and all(isinstance(value, str) for value in configured)
            ):
                entries = (configured,)
            else:
                entries = tuple(configured)
            normalized: set[tuple[str, str]] = set()
            for endpoint_id_raw, role_raw in entries:
                endpoint_id = str(endpoint_id_raw).strip()
                role = str(role_raw).strip()
                if not endpoint_id or not role:
                    raise ValueError("authorized endpoint ID and role must not be empty")
                identity = (endpoint_id, role)
                if identity in normalized:
                    raise ValueError(
                        f"duplicate authorized endpoint for public key {key}: {endpoint_id}/{role}"
                    )
                normalized.add(identity)
            if not normalized:
                raise ValueError("each endpoint public key must authorize at least one identity")
            self._identities[key] = frozenset(normalized)

    @classmethod
    def from_file(cls, path: str | Path) -> "EndpointIdentityRegistry":
        source = Path(path)
        raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
        if not isinstance(raw, Mapping) or raw.get("schema_version") != 1:
            raise ValueError("endpoint registry schema_version must be 1")
        clients = raw.get("clients")
        if not isinstance(clients, list) or not clients:
            raise ValueError("endpoint registry clients must be a non-empty list")
        identities: dict[str, list[tuple[str, str]]] = {}
        for item in clients:
            if not isinstance(item, Mapping):
                raise ValueError("endpoint registry client entries must be objects")
            unknown = sorted(set(item) - {"public_key", "endpoint_id", "role"})
            if unknown:
                raise ValueError("unknown endpoint registry fields: " + ", ".join(unknown))
            key = str(item.get("public_key", "")).strip()
            identity = (
                str(item.get("endpoint_id", "")).strip(),
                str(item.get("role", "")).strip(),
            )
            identities.setdefault(key, []).append(identity)
        return cls(identities)

    def authorize(self, public_key: str, endpoint_id: str, role: str) -> bool:
        allowed = self._identities.get(str(public_key).strip(), frozenset())
        return (str(endpoint_id).strip(), str(role).strip()) in allowed


__all__ = ["EndpointIdentityRegistry"]
