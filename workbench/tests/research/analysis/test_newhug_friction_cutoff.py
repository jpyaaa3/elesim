"""One physical mu threshold, without a synthetic angular-error score."""
import copy
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from workbench.research.support import newhug as m
from workbench.tests.research.analysis.test_newhug_needles import candidate, map_with


def test_friction_score_has_one_coefficient_and_no_added_normal_error():
    points = [[-.1, 0], [.1, 0]]
    normals = m.rotate([[1., 0], [-1., 0]], math.radians(30))
    score = m.friction_score(points, normals)
    assert score['mu_critical'] == pytest.approx(math.tan(math.radians(30)))
    assert 'mu_robust' not in score
    assert 'normal_error_deg' not in score


@pytest.mark.parametrize('raw', [dict(settings=dict(normal_error_deg=2)),
                                dict(needle_settings=dict(mu_field='mu_robust'))])
def test_removed_error_options_are_rejected_instead_of_silently_applied(raw):
    with pytest.raises(m.InputError):
        m.build_request(raw)
        m.build_needle_settings(raw)


@pytest.mark.parametrize('mu', [0., .3, .3000000001])
def test_recording_also_requires_mu_strictly_above_the_contact_threshold(mu):
    n = m.make_needle(candidate(.3), m.NeedleSettings(), 'a', operating_mu=mu)
    assert (n is not None) == (mu > .3)


def test_solver_filters_before_ranking_and_before_callback():
    accepted = []
    r = m.solve(m.template('circle', .1, .1), m.Geometry(),
                m.SearchSettings(operating_mu=.15, results=1), circle_radius_m=.05,
                iteration_limit=16, time_limit_s=0, on_candidate=accepted.append)
    assert accepted
    assert r['search']['rejections']['friction_cutoff'] > 0
    assert all(c['friction']['mu_critical'] < .15 for c in accepted+r['candidates'])
    assert all(c['planar_force_closure_at_operating_mu'] for c in accepted)


def test_map_records_every_valid_score_below_the_global_cutoff(monkeypatch):
    def fake_solve(vertices, geometry, settings, **kwargs):
        for score in (0., .01, .1, .25, .5, .75):
            kwargs['on_candidate'](candidate(score))
        return dict(candidates=[], target_vertices_m=vertices.tolist(), settings=dict(operating_mu=.5),
                    search=dict(status='candidates_found', attempts=4))
    monkeypatch.setattr(m, 'solve', fake_solve)
    r = m.collect_needle_map(m.template('circle', .1, .1), needle_settings=m.NeedleSettings(),
                             circle_radius_m=.05)
    assert [n['friction']['mu_critical'] for n in r['needle_map']['needles']] == [0., .01, .1, .25]
    assert 'minimum' not in r['needle_map']['filter']
    assert r['needle_map']['filter']['maximum'] == .5
    assert r['needle_map']['filter']['maximum_inclusive'] is False


def collected():
    return m.collect_needle_map(m.template('circle', .1, .1),
                               settings=m.SearchSettings(operating_mu=1000),
                               needle_settings=m.NeedleSettings(iterations=16), circle_radius_m=.05)


def test_refiltering_updates_arms_raw_needles_band_and_json_without_mutating_source():
    original = collected()
    saved = copy.deepcopy(original)
    limited = m.filter_result_at_mu(original, .15)
    assert original == saved
    assert limited['settings']['operating_mu'] == .15
    for rows in (limited['candidates'], limited['needle_map']['needles'], limited['needle_map']['gathered']['needles']):
        assert rows
        assert all(c['friction']['mu_critical'] < .15 for c in rows)
    assert len(limited['needle_map']['needles']) < len(original['needle_map']['needles'])
    json.dumps(limited, allow_nan=False)
    empty = m.filter_result_at_mu(original, 0.)
    assert empty['candidates'] == []
    assert empty['needle_map']['needles'] == []
    assert empty['needle_map']['gathered']['needles'] == []
    assert empty['needle_map']['gathered']['band_m'] is None
    ET.fromstring(m.needle_svg_document(empty, gathered=True))


def test_gathering_defends_its_cutoff_and_does_not_let_high_mu_change_the_band():
    a = m.make_needle(candidate(.1), m.NeedleSettings(), 'a', operating_mu=1.)
    b = m.make_needle(candidate(.8, base=(3., 3.)), m.NeedleSettings(), 'b', operating_mu=1.)
    source = map_with([a, b])
    source['filter'] = dict(maximum=.5)
    g = m.gather_needles(source)
    assert g['friction_cutoff_excluded'] == 1
    assert all(n['friction']['mu_critical'] < .5 for n in g['needles'])
    assert g['band_m'][1] < 1.


def test_arm_svg_never_exports_a_pose_that_fails_an_overridden_cutoff():
    r = collected()
    svg = m.svg_document(r, mu=0.)
    root = ET.fromstring(svg)
    assert len(root.findall('.//{http://www.w3.org/2000/svg}polygon')) == 1


def test_ui_and_cli_do_not_advertise_or_accept_an_error_budget():
    page = m.browser_document('test')
    assert 'normalError' not in page
    assert '오차 포함' not in page
    assert '표시 마찰계수' not in page
    assert 'recordField' not in page
    assert 'recordMu' not in page
    for option in ('--normal-error-deg', '--record-mu-field', '--record-mu-min'):
        p = subprocess.run([sys.executable, m.__file__, '--solve', option, '2'], capture_output=True, text=True, timeout=15)
        assert p.returncode == 2
        assert 'unrecognized arguments' in p.stderr


def test_cli_zero_cutoff_writes_an_empty_map_instead_of_exporting_red_candidates(tmp_path):
    path = tmp_path/'map.json'
    p = subprocess.run([sys.executable, m.__file__, '--solve', '--record-needles', '--iterations', '8',
                        '--mu', '0', '--output', str(path)], capture_output=True, text=True, timeout=15)
    assert p.returncode == 2
    r = json.loads(path.read_text())
    assert r['candidates'] == []
    assert r['needle_map']['needles'] == []
    assert 'normal_error_deg' not in r['settings']
    assert 'mu_min' not in r['needle_map']['settings']
