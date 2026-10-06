from types import SimpleNamespace

import pytest

from elesim_sim.vision.render_device import check_camera_renderer


@pytest.mark.parametrize("device,use_gpu,rejected", [
    (b"NVIDIA RTX A6000/PCIe/SSE2", True, False),
    (b"llvmpipe (LLVM 15.0.7, 256 bits)", True, True),
    (b"llvmpipe (LLVM 15.0.7, 256 bits)", False, False),
])
def test_camera_readiness_rejects_gpu_request_using_software_gl(monkeypatch, capsys, device, use_gpu, rejected):
    from OpenGL import GL

    events = []
    renderer = SimpleNamespace(
        make_current=lambda: events.append("current"),
        make_uncurrent=lambda: events.append("released"),
    )
    monkeypatch.setattr(GL, "glGetString", lambda key: device if key == GL.GL_RENDERER else b"vendor")
    if rejected:
        with pytest.raises(RuntimeError, match="GPU camera requested.*llvmpipe"):
            check_camera_renderer(renderer, use_gpu=use_gpu, label="observer")
    else:
        check_camera_renderer(renderer, use_gpu=use_gpu, label="observer")
    assert events == ["current", "released"]
    assert device.decode() in capsys.readouterr().out


def test_camera_device_query_releases_context_on_failure(monkeypatch):
    from OpenGL import GL

    events = []
    renderer = SimpleNamespace(
        make_current=lambda: events.append("current"),
        make_uncurrent=lambda: events.append("released"),
    )

    def failed_query(_key):
        raise RuntimeError("lost GL context")

    monkeypatch.setattr(GL, "glGetString", failed_query)
    with pytest.raises(RuntimeError, match="lost GL context"):
        check_camera_renderer(renderer, use_gpu=True, label="observer")
    assert events == ["current", "released"]
