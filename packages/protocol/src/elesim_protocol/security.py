"""CURVE transport configuration shared by independently deployed nodes."""

from __future__ import annotations

import ipaddress
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from zmq.auth import load_certificate


class TransportSecurityError(RuntimeError):
    pass


def _key(value: bytes | str, *, name: str) -> bytes:
    encoded = value.encode("ascii") if isinstance(value, str) else bytes(value)
    if len(encoded) != 40:
        raise TransportSecurityError(f"{name} must be a 40-byte Z85 CURVE key")
    return encoded


@dataclass(frozen=True)
class CurveClientConfig:
    public_key: bytes
    secret_key: bytes = field(repr=False)
    server_key: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "public_key", _key(self.public_key, name="client public key"))
        object.__setattr__(self, "secret_key", _key(self.secret_key, name="client secret key"))
        object.__setattr__(self, "server_key", _key(self.server_key, name="server public key"))

    @classmethod
    def from_files(
        cls,
        *,
        client_secret_file: str | Path,
        server_public_file: str | Path,
    ) -> "CurveClientConfig":
        client_public, client_secret = load_certificate(str(client_secret_file))
        server_public, _ = load_certificate(str(server_public_file))
        if client_secret is None:
            raise TransportSecurityError("client certificate does not contain a secret key")
        return cls(client_public, client_secret, server_public)

    @classmethod
    def from_client_file(
        cls,
        *,
        client_secret_file: str | Path,
        server_key: bytes | str,
    ) -> "CurveClientConfig":
        """Use a local client certificate with a server key advertised in protocol metadata."""

        client_public, client_secret = load_certificate(str(client_secret_file))
        if client_secret is None:
            raise TransportSecurityError("client certificate does not contain a secret key")
        return cls(client_public, client_secret, server_key)


@dataclass(frozen=True)
class CurveServerConfig:
    public_key: bytes
    secret_key: bytes = field(repr=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "public_key", _key(self.public_key, name="server public key"))
        object.__setattr__(self, "secret_key", _key(self.secret_key, name="server secret key"))

    @classmethod
    def from_file(cls, server_secret_file: str | Path) -> "CurveServerConfig":
        public_key, secret_key = load_certificate(str(server_secret_file))
        if secret_key is None:
            raise TransportSecurityError("server certificate does not contain a secret key")
        return cls(public_key, secret_key)


def configure_curve_client(socket: Any, config: CurveClientConfig) -> None:
    socket.curve_publickey = config.public_key
    socket.curve_secretkey = config.secret_key
    socket.curve_serverkey = config.server_key


def configure_curve_server(socket: Any, config: CurveServerConfig) -> None:
    socket.curve_publickey = config.public_key
    socket.curve_secretkey = config.secret_key
    socket.curve_server = True


def endpoint_is_loopback(endpoint: str) -> bool:
    value = str(endpoint).strip()
    if value.startswith(("ipc://", "inproc://")):
        return True
    if not value.startswith("tcp://"):
        return False
    parsed = urlsplit(value)
    host = parsed.hostname or ""
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def require_secure_remote(
    endpoint: str,
    *,
    curve_enabled: bool,
    allow_insecure_remote: bool = False,
) -> None:
    if curve_enabled or endpoint_is_loopback(endpoint) or allow_insecure_remote:
        return
    raise TransportSecurityError(
        f"remote endpoint {endpoint!r} requires CURVE; "
        "plaintext is limited to loopback unless allow_insecure_remote is enabled"
    )


def require_curve_server_auth(
    endpoint: str,
    *,
    curve_enabled: bool,
    authorized_clients: bool,
    allow_insecure_remote: bool = False,
) -> None:
    """Require encryption and a client-key allowlist on public servers."""

    require_secure_remote(
        endpoint,
        curve_enabled=curve_enabled,
        allow_insecure_remote=allow_insecure_remote,
    )
    if (
        curve_enabled
        and not endpoint_is_loopback(endpoint)
        and not authorized_clients
        and not allow_insecure_remote
    ):
        raise TransportSecurityError(
            f"remote CURVE server {endpoint!r} requires an authorized client key directory"
        )


__all__ = [
    "CurveClientConfig",
    "CurveServerConfig",
    "TransportSecurityError",
    "configure_curve_client",
    "configure_curve_server",
    "endpoint_is_loopback",
    "require_curve_server_auth",
    "require_secure_remote",
]
