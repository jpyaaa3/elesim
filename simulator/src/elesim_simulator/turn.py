"""Simulator-owned short-lived Coturn REST credentials."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from elesim_protocol import TurnCredentials

from .config.distributed import TurnConfig


@dataclass(frozen=True)
class TurnCredentialIssuer:
    """Issue bounded Coturn REST credentials without disclosing its HMAC key."""

    urls: tuple[str, ...]
    realm: str
    static_auth_secret: bytes
    ttl_s: int = 3600

    def __post_init__(self) -> None:
        urls = tuple(str(url).strip() for url in self.urls if str(url).strip())
        if not urls or len(urls) > 8:
            raise ValueError("TURN issuer requires 1..8 URLs")
        if any(any(character.isspace() for character in url) for url in urls):
            raise ValueError("TURN URLs must not contain whitespace")
        if not str(self.realm).strip():
            raise ValueError("TURN realm must not be empty")
        secret = bytes(self.static_auth_secret)
        if not secret:
            raise ValueError("TURN static auth secret must not be empty")
        ttl = int(self.ttl_s)
        if ttl < 900 or ttl > 86400:
            raise ValueError("TURN credential TTL must be within 900..86400 seconds")
        object.__setattr__(self, "urls", urls)
        object.__setattr__(self, "realm", str(self.realm).strip())
        object.__setattr__(self, "static_auth_secret", secret)
        object.__setattr__(self, "ttl_s", ttl)

    def issue(self, endpoint_id: str, session_id: str, now: float) -> TurnCredentials:
        timestamp = float(now)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            raise ValueError("TURN credential time must be positive and finite")
        endpoint = str(endpoint_id).strip()
        session = str(session_id).strip()
        if not endpoint or not session:
            raise ValueError("TURN credential endpoint and session IDs are required")
        expires = int(timestamp) + self.ttl_s
        username = f"{expires}:{endpoint}:{session}"
        digest = hmac.new(
            self.static_auth_secret,
            username.encode("utf-8"),
            hashlib.sha1,
        ).digest()
        return TurnCredentials(
            urls=self.urls,
            username=username,
            credential=base64.b64encode(digest).decode("ascii"),
            expires_at=float(expires),
        )

    def __call__(self, endpoint_id: str, session_id: str, now: float) -> TurnCredentials:
        return self.issue(endpoint_id, session_id, now)


@dataclass(frozen=True)
class StaticTurnCredentialProvider:
    """Provide an operator-supplied external TURN username and credential."""

    credentials: TurnCredentials

    def __post_init__(self) -> None:
        validated = TurnCredentials.from_payload(self.credentials.to_payload())
        object.__setattr__(self, "credentials", validated)

    def __call__(self, endpoint_id: str, session_id: str, now: float) -> TurnCredentials:
        del endpoint_id, session_id
        timestamp = float(now)
        if not math.isfinite(timestamp) or timestamp <= 0.0:
            raise ValueError("TURN credential time must be positive and finite")
        if self.credentials.expires_at <= timestamp:
            raise ValueError("external TURN credentials have expired")
        return self.credentials


TurnCredentialProvider = TurnCredentialIssuer | StaticTurnCredentialProvider


def load_turn_credential_provider(config: TurnConfig) -> TurnCredentialProvider | None:
    """Load the Simulator-owned managed or external TURN credential source."""

    secret_file = config.static_auth_secret_file
    credential_file = config.credential_file
    if secret_file is not None and credential_file is not None:
        raise ValueError("managed and external TURN credentials are mutually exclusive")
    if secret_file is None and credential_file is None:
        if config.urls:
            raise ValueError("TURN URLs require a configured credential source")
        return None
    if secret_file is not None:
        path = Path(secret_file)
        if not path.is_file():
            raise ValueError(f"TURN static auth secret is not a regular file: {path}")
        if path.stat().st_size > 4096:
            raise ValueError(f"TURN static auth secret is unexpectedly large: {path}")
        secret = path.read_bytes().strip()
        return TurnCredentialIssuer(
            urls=config.urls,
            realm=config.realm,
            static_auth_secret=secret,
        )

    path = Path(credential_file)
    if not path.is_file():
        raise ValueError(f"TURN credential file is not a regular file: {path}")
    if path.stat().st_size > 4096:
        raise ValueError(f"TURN credential file is unexpectedly large: {path}")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid TURN credential JSON: {path}") from exc
    if not isinstance(raw, Mapping):
        raise ValueError(f"TURN credential JSON must be an object: {path}")
    unknown = sorted(set(raw) - {"username", "credential", "expires_at"})
    if unknown:
        raise ValueError(
            f"TURN credential JSON has unknown fields {unknown}: {path}"
        )
    credentials = TurnCredentials.from_payload(
        {
            "urls": list(config.urls),
            "username": raw.get("username"),
            "credential": raw.get("credential"),
            # Long-term TURN passwords usually have no server-side expiry.
            # Keep a finite wire value so the shared protocol remains strict.
            "expires_at": raw.get("expires_at", 253402300799.0),
        }
    )
    return StaticTurnCredentialProvider(credentials)


def load_turn_credential_issuer(config: TurnConfig) -> TurnCredentialProvider | None:
    """Backward-compatible name for the generalized credential loader."""

    return load_turn_credential_provider(config)


__all__ = [
    "StaticTurnCredentialProvider",
    "TurnCredentialIssuer",
    "TurnCredentialProvider",
    "load_turn_credential_issuer",
    "load_turn_credential_provider",
]
