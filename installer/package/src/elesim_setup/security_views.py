"""Install stable runtime views containing exactly one SROS2 role enclave."""

from __future__ import annotations

import os
import shutil
import tempfile
from pathlib import Path, PurePosixPath

from ._security_storage import SecurityAuthorityError, resolve_keystore_file
from .configuration import dds_enclave, role_keystore_path
from .state import InstallState


def prepare_role_keystore_views(state: InstallState) -> dict[str, Path]:
    """Create role roots and, when available, copy one enclave into each.

    Runtime paths stay stable across managed generation switches.  An external
    or already-provisioned aggregate keystore is treated only as source input;
    sibling enclave directories are never copied into a role view.
    """

    state.validate()
    security_root = state.prefix_path / "security"
    _private_directory(security_root)
    roles_root = security_root / "roles"
    _private_directory(roles_root)
    destinations = {
        role: role_keystore_path(state, role) for role in state.roles
    }
    for destination in destinations.values():
        _private_directory(destination)

    source_value = str(state.dds.keystore).strip()
    if state.dds.security_profile != "sros2" or not source_value:
        return destinations

    source_candidate = Path(source_value).expanduser()
    source = _validated_directory(
        source_candidate,
        "keystore",
        keystore=source_candidate,
    )
    public = _validated_directory(
        source / "public",
        "keystore public material",
        keystore=source,
    )
    enclave_sources: dict[str, tuple[PurePosixPath, Path]] = {}
    for role in state.roles:
        enclave = PurePosixPath(dds_enclave(state, role))
        relative = PurePosixPath(*enclave.parts[1:])
        enclave_source = _validated_directory(
            source / "enclaves" / Path(*relative.parts),
            f"{role} enclave",
            parent=source / "enclaves",
            keystore=source,
        )
        enclave_sources[role] = relative, enclave_source

    staging = Path(
        tempfile.mkdtemp(prefix=".role-views-", dir=security_root)
    )
    staging.chmod(0o700)
    try:
        prepared = staging / "prepared"
        for role, (relative, enclave_source) in enclave_sources.items():
            view = prepared / role
            _copy_regular_tree(public, view / "public", keystore=source)
            _copy_regular_tree(
                enclave_source,
                view / "enclaves" / Path(*relative.parts),
                keystore=source,
            )
        _install_prepared_views(
            destinations=destinations,
            prepared=prepared,
            backups=staging / "backups",
        )
    finally:
        if staging.exists() and not staging.is_symlink():
            shutil.rmtree(staging)
    return destinations


def _install_prepared_views(
    *,
    destinations: dict[str, Path],
    prepared: Path,
    backups: Path,
) -> None:
    replaced: list[tuple[Path, Path | None]] = []
    try:
        for role, destination in destinations.items():
            _private_directory(destination)
            backup = backups / role
            backup.mkdir(parents=True, mode=0o700)
            for name in ("public", "enclaves"):
                current = destination / name
                if current.is_symlink():
                    raise ValueError(
                        f"role keystore child must not be a symlink: {current}"
                    )
                if current.exists() and not current.is_dir():
                    raise ValueError(
                        f"role keystore child must be a directory: {current}"
                    )
            for name in ("public", "enclaves"):
                current = destination / name
                previous: Path | None = None
                if current.exists():
                    previous = backup / name
                    os.replace(current, previous)
                try:
                    os.replace(prepared / role / name, current)
                except BaseException:
                    if previous is not None:
                        os.replace(previous, current)
                    raise
                replaced.append((current, previous))
    except BaseException:
        for current, previous in reversed(replaced):
            if current.exists() and not current.is_symlink():
                shutil.rmtree(current)
            if previous is not None and previous.exists():
                os.replace(previous, current)
        raise


def _private_directory(path: Path) -> Path:
    if path.is_symlink():
        raise ValueError(f"security directory must not be a symlink: {path}")
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not path.is_dir():
        raise ValueError(f"security path is not a directory: {path}")
    path.chmod(0o700)
    return path


def _validated_directory(
    path: Path,
    label: str,
    *,
    parent: Path | None = None,
    keystore: Path | None = None,
) -> Path:
    candidate = path
    if candidate.is_symlink() or not candidate.is_dir():
        raise ValueError(f"{label} is not a regular directory: {candidate}")
    resolved = candidate.resolve()
    if parent is not None:
        boundary = parent.resolve()
        if boundary != resolved and boundary not in resolved.parents:
            raise ValueError(f"{label} escapes the keystore: {candidate}")
    for directory, names, files in os.walk(candidate, followlinks=False):
        current = Path(directory)
        for name in names:
            child = current / name
            if child.is_symlink():
                raise ValueError(f"{label} contains a directory symlink: {child}")
            if not child.is_dir():
                raise ValueError(f"{label} contains a special file: {child}")
        for name in files:
            child = current / name
            if child.is_symlink():
                if keystore is None:
                    raise ValueError(f"{label} contains a symlink: {child}")
                try:
                    resolve_keystore_file(child, keystore=keystore)
                except SecurityAuthorityError as exc:
                    raise ValueError(str(exc)) from exc
            elif not child.is_file():
                raise ValueError(f"{label} contains a special file: {child}")
    return resolved


def _copy_regular_tree(
    source: Path,
    destination: Path,
    *,
    keystore: Path,
) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    destination.mkdir(mode=0o700)
    for directory, names, files in os.walk(source, followlinks=False):
        current = Path(directory)
        relative = current.relative_to(source)
        target_directory = destination / relative
        target_directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        target_directory.chmod(0o700)
        for name in names:
            child = current / name
            if child.is_symlink() or not child.is_dir():
                raise ValueError(f"keystore contains an unsafe directory: {child}")
        for name in files:
            child = current / name
            try:
                copy_source = resolve_keystore_file(child, keystore=keystore)
            except SecurityAuthorityError as exc:
                raise ValueError(str(exc)) from exc
            target = target_directory / name
            shutil.copyfile(copy_source, target)
            target.chmod(0o600)


__all__ = ["prepare_role_keystore_views"]
