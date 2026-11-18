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
