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
