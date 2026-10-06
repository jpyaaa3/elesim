"""Report the actual offscreen GL device before advertising camera readiness."""

from __future__ import annotations

from typing import Any


def check_camera_renderer(renderer: Any, *, use_gpu: bool, label: str) -> None:
    from OpenGL import GL

    renderer.make_current()
    try:
        vendor_raw = GL.glGetString(GL.GL_VENDOR)
        device_raw = GL.glGetString(GL.GL_RENDERER)
    finally:
        renderer.make_uncurrent()
    if not vendor_raw or not device_raw:
        raise RuntimeError("camera OpenGL context did not report its renderer")
    vendor = vendor_raw.decode("utf-8", errors="replace")
    device = device_raw.decode("utf-8", errors="replace")
    software = bool(getattr(renderer, "_is_software", False)) or any(
        name in device.lower()
        for name in ("llvmpipe", "softpipe", "swrast", "software", "gdi generic")
    )
    print(
        f"[sim-render] {label} vendor={vendor} renderer={device} "
        f"software={str(software).lower()}",
        flush=True,
    )
    if use_gpu and software:
        raise RuntimeError(
            f"GPU camera requested but OpenGL selected {device}. "
            "Check NVIDIA graphics capability and the EGL vendor registration "
            "(/usr/share/glvnd/egl_vendor.d/10_nvidia.json); rebuild the Sim image."
        )
