"""Readiness markers used by the standalone walking batch runner."""

from __future__ import annotations

from pathlib import Path


_SIM_READY_MARKERS = (
    # Current Sim logs this after scene construction and first media frames.
    "[runtime] scene/media readiness gate opened",
    # Keep compatibility with older Sim builds.
    "[sim_camera] publisher bound",
)


def sim_log_ready(log_path: Path) -> bool:
    """Return whether a Sim log contains a known readiness marker."""

    if not log_path.is_file():
        return False
    try:
        text = log_path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    return any(marker in text for marker in _SIM_READY_MARKERS)
