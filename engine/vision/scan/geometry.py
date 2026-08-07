"""
Geometry-fitting backend for the roll-sweep scan.

The fitting layer (cylinder RANSAC, circle RANSAC, dominant-plane removal, ROI
extraction, PLY writing) lives in ``zed_cylinder_bench`` -- the bench script
that was validated against ground-truth-measured objects. This module only
resolves and re-exports it, so the scan runs that exact code instead of a second
implementation that could silently drift from it.

Resolution order for ``zed_cylinder_bench``:
  1. already importable (installed, or on PYTHONPATH)
  2. ``ELESIM_CYLINDER_BENCH`` -- path to the file or to its directory
  3. repo root, then ``tools/``, then ``scripts/``

Resolution is LAZY: it happens on the first call, not at import. Importing this
module therefore has no side effects (it does not touch ``sys.path``), so the
config loader and the UI can depend on the scan package on a machine that has
not been given the bench file yet. Every exported function raises on CALL with
an actionable message when the backend is missing.
"""

from __future__ import annotations

import importlib
import os
import sys
import threading
from pathlib import Path
from typing import Any, Callable

REQUIRED_SYMBOLS = (
    "arc_span_deg",
    "auto_roi",
    "extract_points",
    "fit_cylinder",
    "ransac_circle",
    "remove_dominant_plane",
    "write_ply",
)

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SEARCH_DIRS = (_REPO_ROOT, _REPO_ROOT / "tools", _REPO_ROOT / "scripts")

_lock = threading.Lock()
_resolved = False
_module: Any = None
_backend_name = ""
_backend_error = ""


def _candidate_dirs() -> list[Path]:
    dirs: list[Path] = []
    env = os.environ.get("ELESIM_CYLINDER_BENCH", "").strip()
    if env:
        p = Path(env).expanduser()
        dirs.append(p.parent if p.suffix == ".py" else p)
    dirs.extend(_SEARCH_DIRS)
    return dirs


def _resolve(force: bool = False) -> None:
    global _resolved, _module, _backend_name, _backend_error
    with _lock:
        if _resolved and not force:
            return
        _resolved = True
        _module, _backend_name, _backend_error = None, "", ""
        try:
            _module = importlib.import_module("zed_cylinder_bench")
            _backend_name = "zed_cylinder_bench"
            return
        except ImportError:
            pass
        for d in _candidate_dirs():
            if not (d / "zed_cylinder_bench.py").is_file():
                continue
            s = str(d)
            if s not in sys.path:
                sys.path.insert(0, s)
            try:
                _module = importlib.import_module("zed_cylinder_bench")
                _backend_name = f"zed_cylinder_bench ({d})"
            except ImportError as exc:  # present, but its own imports failed
                _backend_error = (
                    f"found {d / 'zed_cylinder_bench.py'} but import failed: {exc}"
                )
            return
        _backend_error = (
            "zed_cylinder_bench.py not found. Put it at the repo root "
            f"({_REPO_ROOT}), or set ELESIM_CYLINDER_BENCH to its path."
        )


def reload_backend() -> str:
    """Re-run resolution (after dropping the bench file in). Returns backend()."""
    _resolve(force=True)
    return _backend_name


def backend() -> str:
    """Which fitting backend is active ('' when unavailable)."""
    _resolve()
    return _backend_name


def backend_error() -> str:
    """Why the backend is unavailable ('' when it is fine)."""
    _resolve()
    return _backend_error


def missing_symbols() -> tuple[str, ...]:
    """Required symbols the resolved backend does not provide."""
    _resolve()
    if _module is None:
        return REQUIRED_SYMBOLS
    return tuple(n for n in REQUIRED_SYMBOLS if not callable(getattr(_module, n, None)))


def available() -> bool:
    _resolve()
    return _module is not None and not missing_symbols()


def status() -> str:
    """One-line human-readable backend status, for the UI and logs."""
    _resolve()
    if _module is None:
        return f"geometry backend: MISSING -- {_backend_error}"
    miss = missing_symbols()
    if miss:
        return f"geometry backend: {_backend_name} (incomplete: missing {', '.join(miss)})"
    return f"geometry backend: {_backend_name}"


def _dispatch(name: str) -> Callable[..., Any]:
    def _call(*args: Any, **kwargs: Any) -> Any:
        _resolve()
        fn = getattr(_module, name, None) if _module is not None else None
        if not callable(fn):
            detail = _backend_error or (
                f"backend '{_backend_name}' provides no callable '{name}'"
            )
            raise RuntimeError(f"cannot call {name}(): {detail}")
        return fn(*args, **kwargs)

    _call.__name__ = name
    _call.__qualname__ = name
    _call.__doc__ = f"Dispatches to zed_cylinder_bench.{name}(); see status()."
    return _call


arc_span_deg = _dispatch("arc_span_deg")
auto_roi = _dispatch("auto_roi")
extract_points = _dispatch("extract_points")
fit_cylinder = _dispatch("fit_cylinder")
ransac_circle = _dispatch("ransac_circle")
remove_dominant_plane = _dispatch("remove_dominant_plane")
write_ply = _dispatch("write_ply")


__all__ = [
    "REQUIRED_SYMBOLS",
    "arc_span_deg",
    "auto_roi",
    "available",
    "backend",
    "backend_error",
    "extract_points",
    "fit_cylinder",
    "missing_symbols",
    "ransac_circle",
    "reload_backend",
    "remove_dominant_plane",
    "status",
    "write_ply",
]
