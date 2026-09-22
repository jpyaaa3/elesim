"""Writable cache bootstrap for the Sim process.

This module intentionally has only standard-library dependencies.  Sim imports
it before NumPy/Genesis so a stale bind mount cannot abort the process while
those packages are being imported.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import sys
import tempfile


def _prepare_cache_dir(path: Path, *, private: bool) -> None:
    """Create a cache directory and prove that this process can write it."""

    probe = path
    while True:
        if os.path.lexists(probe) and probe.is_symlink():
            raise PermissionError(f"cache path must not have a symlink ancestor: {path}")
        if probe == probe.parent:
            break
        probe = probe.parent
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    if private:
        # A stale bind mount may be owned by root.  chmod is deliberately part
        # of the check so that a directory which can be traversed but cannot be
        # made private is not selected for Numba/Genesis state.
        path.chmod(0o700)
    with tempfile.NamedTemporaryFile(
        mode="wb", prefix=".elesim-cache-probe-", dir=path, delete=True
    ):
        pass


def _cache_namespace(requested: Path | None) -> str:
    """Return a stable, non-sensitive name for a fallback cache.

    The Compose generator supplies a digest of the host-side cache path for
    instance-scoped services.  The requested container path is the fallback
    key for older generated Compose files.  Both keep repeated crash/restart
    cycles from allocating an unbounded series of ``mkdtemp`` directories.
    """

    configured = os.environ.get("ELESIM_CACHE_NAMESPACE", "").strip()
    source = configured or (str(requested) if requested is not None else "default")
    digest = hashlib.sha256(source.encode("utf-8")).hexdigest()[:16]
    return f"elesim-cache-{os.getuid()}-{digest}"


def _new_private_cache_root(requested: Path | None = None) -> Path:
    """Return a process-owned cache root even when an old bind mount is bad."""

    errors: list[OSError] = []
    # Do not trust TMPDIR here: a legacy installation can point it at the same
    # root-owned /tmp/elesim-cache tree that caused startup to fail.  Reuse one
    # process-owned fallback per requested cache instead of leaking a new
    # random directory on every Sim restart.
    parents: list[Path] = []
    for parent in (Path("/tmp"), Path.home()):
        if parent not in parents:
            parents.append(parent)
    name = _cache_namespace(requested)
    for parent in parents:
        try:
            parent.mkdir(mode=0o755, parents=True, exist_ok=True)
            root = parent / name
            _prepare_cache_dir(root, private=True)
            return root
        except OSError as error:
            errors.append(error)

    # A non-standard container may make both stable parents unavailable.  Keep
    # the old last-resort behavior, but still prove the random directory is
    # private and writable before returning it.
    for parent in parents:
        try:
            root = Path(tempfile.mkdtemp(prefix=f"{name}-", dir=parent))
            _prepare_cache_dir(root, private=True)
            return root
        except OSError as error:
            errors.append(error)
    detail = "; ".join(str(error) for error in errors) or "unknown error"
    raise PermissionError(f"unable to create a private Sim cache: {detail}")


def _warn_cache_fallback(requested: Path, fallback: Path, error: OSError) -> None:
    print(
        "[sim-cache] requested cache is not writable "
        f"({requested}: {error}); using private cache {fallback}",
        file=sys.stderr,
        flush=True,
    )


def _configure_numba_cache_dir() -> None:
    """Give Numba and Genesis writable cache directories before Genesis loads.

    Older installations mounted ``/tmp/elesim-cache`` after running Sim as a
    root container.  The current Sim process runs as the installing operator,
    so that stale mount can be unreadable even though the DDS/SROS2 setup is
    healthy.  Keep the configured cache when it is usable; otherwise switch to
    a private process-owned directory instead of crashing during import.
    """

    configured_root = os.environ.get("XDG_CACHE_HOME", "").strip()
    requested_root = (
        Path(configured_root).expanduser()
        if configured_root
        else Path.home() / ".cache"
    )
    try:
        _prepare_cache_dir(requested_root, private=False)
        cache_root = requested_root
    except OSError as error:
        cache_root = _new_private_cache_root(requested_root)
        _warn_cache_fallback(requested_root, cache_root, error)
    os.environ["XDG_CACHE_HOME"] = str(cache_root)

    configured_numba = os.environ.get("NUMBA_CACHE_DIR", "").strip()
    requested_numba = (
        Path(configured_numba).expanduser()
        if configured_numba
        else cache_root / "numba"
    )
    try:
        _prepare_cache_dir(requested_numba, private=True)
        numba_cache = requested_numba
    except OSError as error:
        # If only NUMBA_CACHE_DIR was stale, keep the already validated XDG
        # root and put Numba below it.  Should that also fail, allocate one
        # more private root and use it for both caches.
        fallback_root = cache_root
        fallback_numba = fallback_root / "numba"
        try:
            _prepare_cache_dir(fallback_numba, private=True)
        except OSError:
            fallback_root = _new_private_cache_root(requested_numba)
            fallback_numba = fallback_root / "numba"
            _prepare_cache_dir(fallback_numba, private=True)
            os.environ["XDG_CACHE_HOME"] = str(fallback_root)
        numba_cache = fallback_numba
        _warn_cache_fallback(requested_numba, numba_cache, error)
    os.environ["NUMBA_CACHE_DIR"] = str(numba_cache)


def _ensure_genesis_cache_dir() -> None:
    """Give Genesis' optional viewer cache the same safe fallback policy."""

    cache_root = os.environ.get("XDG_CACHE_HOME", "").strip()
    if cache_root:
        cache_dir = Path(cache_root).expanduser() / "genesis"
    else:
        cache_dir = Path.home() / ".cache" / "genesis"
    try:
        _prepare_cache_dir(cache_dir, private=True)
    except OSError as error:
        fallback_root = _new_private_cache_root(cache_dir)
        fallback_dir = fallback_root / "genesis"
        _prepare_cache_dir(fallback_dir, private=True)
        os.environ["XDG_CACHE_HOME"] = str(fallback_root)
        _warn_cache_fallback(cache_dir, fallback_dir, error)


__all__ = [
    "_configure_numba_cache_dir",
    "_ensure_genesis_cache_dir",
    "_new_private_cache_root",
    "_prepare_cache_dir",
    "_warn_cache_fallback",
]
