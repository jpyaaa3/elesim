from types import SimpleNamespace as NS

import pytest

from elesim_ui.panels import object_scan


@pytest.mark.parametrize('active,can_start,button,expected', [
    (False, True, 'Start object scan', [('start_object_scan', 'apple-1', 8)]),
    (True, False, 'Cancel object scan', [('cancel_object_scan',)]),
    (False, False, 'Start object scan', []),
])
def test_scan_panel_submits_only_explicit_start_or_cancel(monkeypatch, active, can_start, button, expected):
    calls, text = [], []
    monkeypatch.setattr(object_scan, 'panel_header', lambda *a, **k: (True,))
    monkeypatch.setattr(object_scan, 'imgui', NS(text=text.append, text_wrapped=text.append,
        button=lambda label: label == button, input_text=lambda *a: (False, 'apple-1'),
        slider_int=lambda *a: (False, 8)))
    status = dict(phase='moving' if active else 'idle', active=active, can_start=can_start,
        blocked_reason='' if can_start else 'Sim required',
        coverage={'fraction': .4, 'unobserved_directions': [[0., 0., -1.]]})
    service = NS(object_scan=status,
        object_scan_async=lambda name, *args, **kw: calls.append((name, *args)))
    object_scan.draw_object_scan_panel(NS(service=service))
    assert calls == expected
    assert any('completeness is unverified' in row for row in text)
