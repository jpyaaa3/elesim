"""SSH fingerprint and staged secure-file helpers.

DDS discovery uses the host's advertised IP; SSH reuses that destination while
keeping its own management port. SROS2 keystore provisioning is intentionally
external to this module.
"""

from __future__ import annotations

import base64
import hashlib
import ipaddress
import json
import os
import shlex
import shutil
import socket
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

from elesim_protocol import TurnCredentials


class SshProbeError(RuntimeError):
    """A host key probe failed before a fingerprint could be read."""


def probe_ssh_fingerprint(
    host: str,
    port: int,
    *,
    timeout_s: float = 8.0,
    force_tailscale_proxy: bool = False,
) -> str:
    import paramiko

    connection = _open_probe_connection(
        host,
        int(port),
        timeout_s,
        force_tailscale_proxy=force_tailscale_proxy,
    )
    transport = None
    try:
        transport = paramiko.Transport(connection)
        try:
            transport.start_client(timeout=timeout_s)
            key = transport.get_remote_server_key()
            return _fingerprint(key.asbytes())
        finally:
            transport.close()
    except (TimeoutError, socket.timeout) as exc:
        raise SshProbeError(_probe_failure(host, port, "timed out")) from exc
    except ConnectionRefusedError as exc:
        raise SshProbeError(_probe_failure(host, port, "connection refused")) from exc
    except OSError as exc:
        detail = str(exc).strip() or exc.__class__.__name__
        raise SshProbeError(_probe_failure(host, port, detail)) from exc
    except Exception as exc:
        detail = proxy_failure_detail(connection, exc)
        raise SshProbeError(_probe_failure(host, port, f"SSH handshake failed: {detail}")) from exc
    finally:
        if transport is None:
            try:
                connection.close()  # type: ignore[attr-defined]
            except Exception:
                pass


def tailscale_proxy_command(
    host: str,
    port: int,
    *,
    force: bool = False,
) -> str | None:
    """Return a host-Tailscale ``nc`` proxy command when the wrapper provides one.

    Docker Desktop/WSL may place the manager in a network namespace that cannot
    see the WSL ``tailscale0`` interface.  The generated wrapper optionally
    exposes a private, allowlisted host-helper socket; using its ``tailscale
    nc`` operation keeps the actual WireGuard path on the host while the
    manager still owns SSH host key pinning and Paramiko authentication.  The
    manager never receives the tailscaled local API socket.  No proxy is
    selected for ordinary addresses or when the wrapper did not provide it.
    Callers that already selected explicit Tailscale SSH may set ``force`` for
    MagicDNS hostnames.
    """

    if os.environ.get("ELESIM_TAILSCALE_PROXY") != "1":
        return None
    value = str(host).strip()
    if not force:
        try:
            address = ipaddress.ip_address(value)
        except ValueError:
            return None
        if address.version != 4 or address not in ipaddress.ip_network("100.64.0.0/10"):
            return None
    binary = os.environ.get("ELESIM_TAILSCALE_PROXY_BIN", "").strip()
    socket_path = os.environ.get("ELESIM_TAILSCALE_PROXY_SOCKET", "").strip()
    if (
        not binary
        or not socket_path
        or not binary.startswith("/")
        or not socket_path.startswith("/")
    ):
        return None
    if any("\x00" in value for value in (binary, socket_path)):
        return None
    return (
        f"{shlex.quote(binary)} --socket={shlex.quote(socket_path)} nc "
        f"{shlex.quote(value)} {int(port)}"
    )


def _open_probe_connection(
    host: str,
    port: int,
    timeout_s: float,
    *,
    force_tailscale_proxy: bool,
) -> object:
    proxy_command = tailscale_proxy_command(
        host,
        port,
        force=force_tailscale_proxy,
    )
    if proxy_command is None:
        try:
            return socket.create_connection((host, port), timeout=timeout_s)
        except (TimeoutError, socket.timeout) as exc:
            raise SshProbeError(_probe_failure(host, port, "timed out")) from exc
        except ConnectionRefusedError as exc:
            raise SshProbeError(_probe_failure(host, port, "connection refused")) from exc
        except OSError as exc:
            detail = str(exc).strip() or exc.__class__.__name__
            raise SshProbeError(_probe_failure(host, port, detail)) from exc

    try:
        from paramiko.proxy import ProxyCommand

        return ProxyCommand(proxy_command)
    except Exception as exc:
        detail = str(exc).strip() or exc.__class__.__name__
        raise SshProbeError(
            _probe_failure(host, port, f"Tailscale host proxy could not start: {detail}")
        ) from exc


def _probe_failure(host: str, port: int, reason: str) -> str:
    origin = "연결관리자 Docker 컨테이너"
    if os.environ.get("ELESIM_CONNECTION_PUBLISHED") != "1":
        origin = "연결관리자 실행 호스트"
    tail = (
        " Tailscale SSH라면 원격 호스트에서도 `sudo tailscale set --ssh`와 ACL의 "
        "SSH 허용을 확인하고, Tailscale 주소는 22번을 사용하십시오."
    )
    container_hint = (
        " 이 관리자는 Docker 컨테이너에서 실행 중입니다. 호스트 터미널의 "
        "`nc -vz -w 8 HOST PORT`가 성공해도 컨테이너 경로가 막힐 수 있습니다."
        if os.environ.get("ELESIM_CONNECTION_PUBLISHED") == "1"
        else ""
    )
    return (
        f"SSH host key probe가 {origin}에서 {host}:{port}에 대해 {reason}되었습니다."
        f"{container_hint}{tail}"
    )


def proxy_failure_detail(connection: object, exc: BaseException) -> str:
    """Prefer the bounded host-proxy diagnostic over Paramiko's broken pipe."""

    detail = str(exc).strip() or exc.__class__.__name__
    process = getattr(connection, "process", None)
    stderr = getattr(process, "stderr", None)
    poll = getattr(process, "poll", None)
    if process is None or stderr is None or not callable(poll) or poll() is None:
        return detail
    try:
        raw = stderr.read(4096)
    except (OSError, ValueError):
        return detail
    decoded = (
        raw.decode("utf-8", errors="replace")
        if isinstance(raw, bytes)
        else str(raw or "")
    )
    proxy_detail = " ".join(decoded.strip().split())
    return proxy_detail or detail


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


def _resolve_non_symlink_path(path: Path, *, name: str) -> Path:
    """Resolve a credential path without allowing symlinked ancestors."""

    lexical = Path(os.path.abspath(os.fspath(path.expanduser())))
    resolved = lexical.resolve()
    if lexical != resolved:
        raise ValueError(f"{name} must not contain symlinked path components: {lexical}")
    return resolved


def validate_external_turn_credentials(
    path: Path,
    *,
    urls: Sequence[str],
) -> None:
    """Validate the bounded external-TURN JSON without retaining its secret."""

    source = _resolve_non_symlink_path(path, name="TURN credential path")
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
    "SshProbeError",
    "tailscale_proxy_command",
    "proxy_failure_detail",
    "validate_external_turn_credentials",
]
