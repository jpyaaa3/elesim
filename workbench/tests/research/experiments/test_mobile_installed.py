"""Installed UI driver stays dependency-light and preserves typed telemetry."""
import json
from types import SimpleNamespace
from elesim_protocol import SimQ
from workbench.research.experiments.mobile_manipulation.installed_ui import plain
from workbench.tools.bench.catalog import source_closure


def test_ui_decoded_namespaces_and_canonical_q_can_be_saved_as_finite_json():
    state=SimpleNamespace(q=SimQ(-.12,0.,-.59,.59),rx_age_s=float('inf'),connected=True)
    decoded=json.loads(json.dumps(plain(state),allow_nan=False))
    assert decoded['q']['linear_m']==-.12
    assert decoded['rx_age_s'] is None and decoded['connected']


def test_installed_ui_probe_has_no_research_physics_or_truth_producer_dependency():
    files=source_closure('workbench.research.experiments.mobile_manipulation.installed_ui')
    assert {f.name for f in files}=={'__init__.py','installed_ui.py','installed_gui.py'}


def test_ui_screenshot_round_trips_rgba_and_opengl_vertical_origin(tmp_path):
    import pytest
    av=pytest.importorskip('av')
    import numpy as np
    from workbench.research.experiments.mobile_manipulation.installed_gui import InstalledGui
    raw=np.zeros((8,12,4),np.uint8)
    raw[...,3]=255
    raw[:4,:,0]=255
    raw[4:,:,1]=255
    gui=InstalledGui.__new__(InstalledGui)
    gui.width,gui.height,gui.output=12,8,tmp_path
    gui.gl=SimpleNamespace(GL_RGBA=0x1908,GL_UNSIGNED_BYTE=0x1401,
        glReadPixels=lambda *args:raw.tobytes())
    gui.screenshot('codec')
    with av.open(str(tmp_path/'ui-codec.png')) as image:
        decoded=next(image.decode(video=0)).to_ndarray(format='rgba')
    np.testing.assert_array_equal(decoded,raw[::-1])
