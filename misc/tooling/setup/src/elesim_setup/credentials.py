"""Role-scoped Curve credential generation and SSH distribution."""

from __future__ import annotations

import base64
import hashlib
import os
import posixpath
import shutil
import socket
import stat
import subprocess
import sys
import tempfile
from pathlib import Path, PurePosixPath
from typing import Callable, Iterable

from .configuration import credentials_for_role, missing_credentials
from .request import SetupRequest
from .state import InstallState


Log = Callable[[str], None]
_MAX_REMOTE_FILES = 1_000
_MAX_REMOTE_FILE_BYTES = 16 * 1024 * 1024


def credential_relative_paths(state: InstallState) -> tuple[PurePosixPath, ...]:
    root = state.security.root
    if root is None:
        return ()
    values: list[PurePosixPath] = []
    for role in state.roles:
        for path in credentials_for_role(state, role):
            relative = PurePosixPath(*path.relative_to(root).parts)
            if relative not in values:
                values.append(relative)
    if {"controller", "ui"}.intersection(state.roles):
        doctor = PurePosixPath("curve/clients/doctor-main.key_secret")
        if doctor not in values:
            values.append(doctor)
    return tuple(values)


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


def provision_credentials(request: SetupRequest, *, log: Log = print) -> None:
    if request.security.mode != "curve":
        return
    state = request.to_install_state()
    source = request.credential_source
    if source == "existing":
        _require_credentials(state)
        log("[credentials] existing role-scoped credentials verified")
        return
    if source == "generate":
        _generate(request, log=log)
        _require_credentials(state)
        return
    if source == "ssh":
        _download(request, state, log=log)
        _require_credentials(state)
        return
    raise ValueError("CURVE security requires an explicit credential source")


def install_staged_credentials(
    staged_root: Path,
    destination_root: Path,
) -> tuple[Path, ...]:
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
        raise FileExistsError(f"기존 credential을 덮어쓸 수 없습니다:\n{rendered}")

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


def _generate(request: SetupRequest, *, log: Log) -> None:
    root = request.security.root
    if root is None:
        raise ValueError("credential root is required")
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"credential root가 비어 있지 않습니다: {root}")
    if root.exists():
        root.rmdir()
    script = request.source_root / "misc/infra/bootstrap_security.py"
    if not script.is_file():
        raise FileNotFoundError(script)
    realm = request.turn.realm or "elesim.local"
    public_host = request.turn.public_host or request.network.advertise_host
    command = (
        sys.executable,
        str(script),
        "--output",
        str(root),
        "--coturn-env",
        str(request.prefix / "infra/coturn.env"),
        "--turn-public-ip",
        public_host,
        "--turn-realm",
        realm,
    )
    log(f"[credentials] generate central bundle at {root}")
    subprocess.run(command, check=True)


def _download(request: SetupRequest, state: InstallState, *, log: Log) -> None:
    import paramiko

    expected = request.ssh.accepted_fingerprint.strip()
    observed = probe_ssh_fingerprint(request.ssh.host, request.ssh.port)
    if not _same_fingerprint(expected, observed):
        raise RuntimeError(
            f"SSH host fingerprint changed: expected {expected}, observed {observed}"
        )

    class PinnedPolicy(paramiko.MissingHostKeyPolicy):
        def missing_host_key(self, client, hostname, key) -> None:  # type: ignore[no-untyped-def]
            actual = _fingerprint(key.asbytes())
            if not _same_fingerprint(expected, actual):
                raise paramiko.SSHException(
                    f"SSH host fingerprint mismatch: expected {expected}, observed {actual}"
                )

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(PinnedPolicy())
    key_filename = request.ssh.identity_file.strip() or None
    log(
        f"[credentials] connect {request.ssh.user}@{request.ssh.host}:"
        f"{request.ssh.port} ({observed})"
    )
    client.connect(
        hostname=request.ssh.host,
        port=request.ssh.port,
        username=request.ssh.user,
        key_filename=key_filename,
        allow_agent=True,
        look_for_keys=True,
        password=None,
        timeout=10,
        auth_timeout=10,
        banner_timeout=10,
    )
    destination = state.security.root
    if destination is None:
        client.close()
        raise ValueError("credential root is required")
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(
            prefix=".elesim-credentials-",
            dir=destination.parent,
        ) as temporary:
            staged = Path(temporary)
            with client.open_sftp() as sftp:
                counter = [0]
                for relative in credential_relative_paths(state):
                    remote = posixpath.join(
                        request.ssh.remote_root.rstrip("/"),
                        relative.as_posix(),
                    )
                    _download_path(
                        sftp,
                        remote,
                        staged.joinpath(*relative.parts),
                        counter=counter,
                    )
            installed = install_staged_credentials(staged, destination)
    finally:
        client.close()
    log(f"[credentials] installed {len(installed)} role-scoped files")


def _download_path(sftp, remote: str, local: Path, *, counter: list[int]) -> None:
    attributes = sftp.lstat(remote)
    mode = attributes.st_mode
    if stat.S_ISLNK(mode):
        raise RuntimeError(f"remote credential path is a symlink: {remote}")
    if stat.S_ISDIR(mode):
        local.mkdir(parents=True, exist_ok=True)
        for item in sftp.listdir_attr(remote):
            if item.filename in {".", ".."} or "/" in item.filename:
                raise RuntimeError(f"unsafe remote credential name: {item.filename!r}")
            _download_path(
                sftp,
                posixpath.join(remote, item.filename),
                local / item.filename,
                counter=counter,
            )
        return
    if not stat.S_ISREG(mode):
        raise RuntimeError(f"unsupported remote credential type: {remote}")
    if int(attributes.st_size) > _MAX_REMOTE_FILE_BYTES:
        raise RuntimeError(f"remote credential is unexpectedly large: {remote}")
    counter[0] += 1
    if counter[0] > _MAX_REMOTE_FILES:
        raise RuntimeError("remote credential bundle contains too many files")
    local.parent.mkdir(parents=True, exist_ok=True)
    sftp.get(remote, str(local))


def _require_credentials(state: InstallState) -> None:
    missing = missing_credentials(state)
    if missing:
        rendered = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"CURVE credential이 부족합니다:\n{rendered}")


def _credential_mode(path: Path) -> int:
    name = path.name
    return 0o600 if name.endswith(".key_secret") or name in {"turn.secret"} else 0o644


def _fingerprint(key_bytes: bytes) -> str:
    digest = hashlib.sha256(key_bytes).digest()
    encoded = base64.b64encode(digest).decode("ascii").rstrip("=")
    return f"SHA256:{encoded}"


def _same_fingerprint(first: str, second: str) -> bool:
    return first.strip().rstrip("=") == second.strip().rstrip("=")


__all__ = [
    "credential_relative_paths",
    "install_staged_credentials",
    "probe_ssh_fingerprint",
    "provision_credentials",
]
