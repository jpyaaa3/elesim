"""SSH fingerprint and staged secure-file helpers.

DDS discovery peers and ROS runtime settings never inherit SSH host/port
values. SROS2 keystore provisioning is intentionally external to this module.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import shutil
import socket
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from elesim_protocol import TurnCredentials


def probe_ssh_fingerprint(host: str, port: int, *, timeout_s: float = 8.0) -> str:
    import paramiko

    with socket.create_connection((host, int(port)), timeout=timeout_s) as connection:
        transport = paramiko.Transport(connection)
        try:
            transport.start_client(timeout=timeout_s)
            key = transport.get_remote_server_key()
            return _fingerprint(key.asbytes())
        finally:
            transport.close()


def install_staged_credentials(
    staged_root: Path,
    destination_root: Path,
) -> tuple[Path, ...]:
    """Atomically install already-validated regular files without overwrites."""

    staged = staged_root.resolve()
    destination = destination_root.expanduser().resolve()
    sources = sorted(path for path in staged.rglob("*") if path.is_file())
    conflicts: list[Path] = []
    for source in sources:
        relative = source.relative_to(staged)
        target = destination / relative
        if target.exists() and target.read_bytes() != source.read_bytes():
            conflicts.append(target)
    if conflicts:
        rendered = "\n".join(f"  - {path}" for path in conflicts)
        raise FileExistsError(f"기존 보안 파일을 덮어쓸 수 없습니다:\n{rendered}")

    installed: list[Path] = []
    for source in sources:
        relative = source.relative_to(staged)
        target = destination / relative
        if target.exists():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_name(f".{target.name}.tmp")
        shutil.copyfile(source, temporary)
        temporary.chmod(_credential_mode(target))
        os.replace(temporary, target)
        installed.append(target)
    return tuple(installed)


def validate_external_turn_credentials(
    path: Path,
    *,
    urls: Sequence[str],
) -> None:
    """Validate the bounded external-TURN JSON without retaining its secret."""

    source = path.expanduser().resolve()
    if not source.exists() or source.is_symlink() or not source.is_file():
        raise ValueError(f"TURN credential path is not a regular file: {source}")
    if source.stat().st_size > 4096:
        raise ValueError(f"TURN credential file is unexpectedly large: {source}")
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid TURN credential JSON: {source}") from exc
    if not isinstance(raw, Mapping):
        raise ValueError(f"TURN credential JSON must be an object: {source}")
    unknown = sorted(set(raw) - {"username", "credential", "expires_at"})
    if unknown:
        raise ValueError(
            f"TURN credential JSON has unknown fields {unknown}: {source}"
        )
    credentials = TurnCredentials.from_payload(
        {
            "urls": list(urls),
            "username": raw.get("username"),
            "credential": raw.get("credential"),
            "expires_at": raw.get("expires_at", 253402300799.0),
        }
    )
    if credentials.expires_at <= time.time():
        raise ValueError(f"TURN credentials have expired: {source}")


def _credential_mode(path: Path) -> int:
    private_names = {
        "key.pem",
        "ca.key.pem",
        "identity_ca.key.pem",
        "permissions_ca.key.pem",
        "turn.secret",
    }
    return 0o600 if path.name in private_names else 0o644


def _fingerprint(key_bytes: bytes) -> str:
    digest = hashlib.sha256(key_bytes).digest()
    encoded = base64.b64encode(digest).decode("ascii").rstrip("=")
    return f"SHA256:{encoded}"


__all__ = [
    "install_staged_credentials",
    "probe_ssh_fingerprint",
    "validate_external_turn_credentials",
]
