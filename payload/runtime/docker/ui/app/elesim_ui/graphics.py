"""Check the container's display and graphics backend before joining DDS."""

from __future__ import annotations

import os


def main() -> None:
    import glfw
    from OpenGL import GL

    if not glfw.init():
        raise RuntimeError(f"UI cannot initialize X11/GLFW for DISPLAY={os.environ.get('DISPLAY', '')}")
    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(16, 16, "EleSim graphics check", None, None)
        if not window:
            raise RuntimeError("UI cannot create an OpenGL context")
        glfw.make_context_current(window)
        vendor = (GL.glGetString(GL.GL_VENDOR) or b"").decode(errors="replace")
        renderer = (GL.glGetString(GL.GL_RENDERER) or b"").decode(errors="replace")
        if os.environ.get("ELESIM_UI_GRAPHICS_BACKEND") == "wslg":
            adapter = os.environ.get("MESA_D3D12_DEFAULT_ADAPTER_NAME", "").strip().casefold()
            actual = renderer.casefold()
            software = any(name in actual for name in (
                "llvmpipe", "softpipe", "software", "basic render driver", "warp",
            ))
            if not actual.startswith("d3d12 (") or software or not adapter or adapter not in actual:
                raise RuntimeError(
                    f"WSLg UI GPU rendering requested for {adapter or 'unknown adapter'}, "
                    f"found {vendor}: {renderer}"
                )
        elif os.environ.get("__GLX_VENDOR_LIBRARY_NAME") == "nvidia" and "nvidia" not in vendor.lower():
            raise RuntimeError(f"NVIDIA UI rendering requested, found {vendor}: {renderer}")
        print(f"[ui-graphics] vendor={vendor} renderer={renderer}", flush=True)
    finally:
        if window:
            glfw.destroy_window(window)
        glfw.terminate()


if __name__ == "__main__":
    main()
