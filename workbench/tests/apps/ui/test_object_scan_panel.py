"""Retained explicit start/cancel and coverage regressions for the unified UI."""
from types import SimpleNamespace as NS
import pytest
from elesim_ui.panels import visual_servoing
from workbench.tools.ui_preview import preview_panel


@pytest.mark.parametrize('active,can_start,action,expected', [
    (False,True,'scan',[('start_object_scan','apple-1',8)]),
    (True,False,'cancel_scan',[('cancel_object_scan',)]),
    (False,False,'scan',[]),
])
def test_scan_panel_submits_only_explicit_start_or_cancel(monkeypatch,active,can_start,action,expected):
    panel,cache = preview_panel(scenario='ready')
    c = panel.servoing
    c.target_id,c.max_views = 'apple-1',8
    cache.services['object_scan'].update(phase='moving' if active else 'idle',active=active,
        can_start=can_start,blocked_reason='' if can_start else 'Sim required',
        coverage={'fraction':.4,'unobserved_directions':[[0.,0.,-1.]]})
    c.perform(action)
    assert [request[0][1:] for request in cache.requests] == expected
    text = []
    monkeypatch.setattr(visual_servoing,'imgui',NS(text=text.append,text_wrapped=text.append,
        text_colored=lambda value,*_:text.append(value),progress_bar=lambda *_:None,
        tree_node=lambda *_:False))
    visual_servoing._progress(c,c.view())
    assert any('completeness is unverified' in row for row in text)
