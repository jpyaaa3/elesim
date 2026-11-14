"""No display required: preflight must check the requested renderer and clean up."""
from types import SimpleNamespace
import sys

import pytest

from elesim_ui import graphics


@pytest.mark.parametrize("vendor, requested, valid", [
    (b"NVIDIA Corporation", "nvidia", True),
    (b"Mesa", "nvidia", False),
    (b"Mesa", "", True),
])
def test_preflight_checks_vendor_before_runtime_and_always_closes(monkeypatch, vendor, requested, valid):
    events = []
    fake = SimpleNamespace(VISIBLE=1, FALSE=0, init=lambda: True,
        window_hint=lambda *args: None, create_window=lambda *args: "hidden",
        make_context_current=lambda window: events.append("context"),
        destroy_window=lambda window: events.append("close"),
        terminate=lambda: events.append("terminate"))
    gl = SimpleNamespace(GL_VENDOR=1, GL_RENDERER=2,
        glGetString=lambda name: vendor if name == 1 else b"renderer")
    monkeypatch.setitem(sys.modules, "glfw", fake)
    monkeypatch.setitem(sys.modules, "OpenGL", SimpleNamespace(GL=gl))
    monkeypatch.setenv("__GLX_VENDOR_LIBRARY_NAME", requested)
    if valid:
        graphics.main()
    else:
        with pytest.raises(RuntimeError, match="NVIDIA UI rendering requested"):
            graphics.main()
    assert events == ["context", "close", "terminate"]


@pytest.mark.parametrize("renderer, valid", [
    ("D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)", True),
    ("llvmpipe (LLVM 15.0.7, 256 bits)", False),
    ("D3D12 (Microsoft Basic Render Driver)", False),
    ("D3D12 (Intel Iris Xe Graphics)", False),
])
def test_wslg_preflight_requires_selected_hardware_renderer(monkeypatch, renderer, valid):
    events = []
    fake = SimpleNamespace(VISIBLE=1, FALSE=0, init=lambda: True,
        window_hint=lambda *args: None, create_window=lambda *args: "hidden",
        make_context_current=lambda window: None,
        destroy_window=lambda window: events.append("close"),
        terminate=lambda: events.append("terminate"))
    gl = SimpleNamespace(GL_VENDOR=1, GL_RENDERER=2,
        glGetString=lambda name: b"Microsoft Corporation" if name == 1 else renderer.encode())
    monkeypatch.setitem(sys.modules, "glfw", fake)
    monkeypatch.setitem(sys.modules, "OpenGL", SimpleNamespace(GL=gl))
    monkeypatch.setenv("__GLX_VENDOR_LIBRARY_NAME", "mesa")
    monkeypatch.setenv("ELESIM_UI_GRAPHICS_BACKEND", "wslg")
    monkeypatch.setenv("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA GeForce RTX 3050 Laptop GPU")
    if valid:
        graphics.main()
    else:
        with pytest.raises(RuntimeError, match="WSLg UI GPU rendering requested"):
            graphics.main()
    assert events == ["close", "terminate"]
