"""Pick an offscreen OpenGL platform before pyrender is imported.

A training server has no display, so pyrender has to render through EGL.  On
macOS there is no EGL at all and the choice must be left alone -- forcing it
there is what made `elesim-sim` abort on import (see
`elesim_sim.main._configure_gpu_render_environment`).

Import this *before* anything that pulls in `OpenGL`, which in practice means
before `genesis`.
"""

from __future__ import annotations

import os
import sys


def select_offscreen_gl(explicit: str | None = None) -> str:
    """Set PYOPENGL_PLATFORM for headless rendering and return what was chosen.

    An existing value always wins: an operator who set `osmesa` because their
    driver has no EGL should not be overridden.
    """
    current = os.environ.get("PYOPENGL_PLATFORM", "").strip()
    if current:
        return current
    if explicit:
        os.environ["PYOPENGL_PLATFORM"] = explicit
        return explicit
    if sys.platform == "darwin":
        # macOS renders through CGL; naming a platform here breaks the import.
        return "(unset: macOS uses CGL)"
    if os.environ.get("DISPLAY"):
        return "(unset: DISPLAY is set)"
    os.environ["PYOPENGL_PLATFORM"] = "egl"
    return "egl"
