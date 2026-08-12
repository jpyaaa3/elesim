"""Fail-closed launch marker for connection-managed SROS2 provisioning."""

from __future__ import annotations

import os
import shlex
import tempfile
from pathlib import Path

from .state import InstallState


PROVISIONING_REQUIRED_FILENAME = "provisioning-required"


def provisioning_required_path(state: InstallState) -> Path:
    return state.prefix_path / "security" / PROVISIONING_REQUIRED_FILENAME


def sync_provisioning_required(state: InstallState) -> Path:
    """Atomically make the launch marker match the installed DDS state."""

    state.require_installable_dds()
    marker = provisioning_required_path(state)
    root = marker.parent
    if root.is_symlink():
        raise ValueError(f"security root는 symlink일 수 없습니다: {root}")
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    if not root.is_dir():
        raise ValueError(f"security root가 directory가 아닙니다: {root}")
    root.chmod(0o700)
    if marker.is_symlink() or (marker.exists() and not marker.is_file()):
        raise ValueError(f"SROS2 provisioning marker가 일반 파일이 아닙니다: {marker}")

    if state.dds.managed_security_pending:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=root,
            prefix=f".{marker.name}.",
            delete=False,
        ) as handle:
            handle.write(
                "managed SROS2 role bundle is not provisioned; "
                "run elesim-connections on the operator laptop\n"
            )
            handle.flush()
            os.fsync(handle.fileno())
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, marker)
    elif marker.exists():
        marker.unlink()
    _fsync_directory(root)
    return marker


def launch_guard(marker: Path) -> str:
    """Return a shell fragment that refuses application startup while pending."""

    quoted = shlex.quote(str(marker))
    return (
        f"if [[ -e {quoted} || -L {quoted} ]]; then\n"
        "  printf 'EleSim 실행 거부: managed SROS2 role bundle이 아직 "
        "provision되지 않았습니다.\\n' >&2\n"
        "  printf 'operator laptop에서 elesim-connections를 실행해 "
        "전체 host generation을 적용하십시오.\\n' >&2\n"
        "  exit 78\n"
        "fi\n"
    )


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


__all__ = [
    "PROVISIONING_REQUIRED_FILENAME",
    "launch_guard",
    "provisioning_required_path",
    "sync_provisioning_required",
]
