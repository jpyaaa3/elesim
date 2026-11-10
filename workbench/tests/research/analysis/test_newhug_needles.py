"""Needle-map filtering, physical rail coordinates and circle symmetry."""
import copy
import math
import threading
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from workbench.research.support import newhug as m


def candidate(mu=.3, *, base=(.2, .3), heading=0.):
    return dict(base_xy_m=list(base), heading_rad=heading, heading_deg=math.degrees(heading),
                turns_rad=[-.8, -2.1], node_bends_deg=np.degrees([-.8/5, -2.1/5]).tolist(),
                contact_mode='node', contact_surfaces=[dict(part='node2', edge=1, fraction=.3),
                                                     dict(part='node9', edge=2, fraction=.5)],
                contacts_m=[[.05, 0], [-.05, 0]], inward_normals=[[-1, 0], [1, 0]],
                friction=dict(mu_critical=mu),
                geometry=dict(valid=True), actual_hold_verified=None)


def map_with(needles):
    return dict(target=dict(kind='circle', centre_xy_m=[0., 0.], radius_m=.05), needles=needles)


def test_recording_keeps_zero_and_very_low_friction_solutions():
    s = m.NeedleSettings()
    for mu in (0., 1e-12, .1, .499999):
        assert m.make_needle(candidate(mu), s, 'a')['friction']['mu_critical'] == mu
    assert m.make_needle(candidate(.5), s, 'a') is None


def test_map_settings_do_not_expose_a_lower_friction_filter():
    from dataclasses import asdict
    assert 'mu_min' not in asdict(m.NeedleSettings())
    assert 'recordMu' not in m.browser_document('test')
    with pytest.raises(m.InputError):
        m.build_needle_settings(dict(needle_settings=dict(mu_min=.2)))


@pytest.mark.parametrize('mu', [None, float('inf'), float('nan')])
def test_unbounded_or_missing_friction_is_not_a_recordable_solution(mu):
    c = candidate()
    c['friction']['mu_critical'] = mu
    assert m.make_needle(c, m.NeedleSettings(), 'a') is None


def test_invalid_geometry_never_becomes_a_needle():
    c = candidate()
    c['geometry']['valid'] = False
    assert m.make_needle(c, m.NeedleSettings(), 'a') is None


def test_needles_are_go2_positions_and_not_first_hinge_positions():
    s = m.NeedleSettings()
    c = candidate(heading=.7)
    n = m.make_needle(c, s, 'a')
    assert np.linalg.norm(np.diff(n['segment_xy_m'], axis=0)) == pytest.approx(260/1500)
    forward = np.array([math.cos(.7), math.sin(.7)])
    for t in np.linspace(0, 1, 11):
        park = (1-t)*np.array(n['segment_xy_m'][0])+t*np.array(n['segment_xy_m'][1])
        linear = (1-t)*n['linear_limits_m'][0]+t*n['linear_limits_m'][1]
        hinge = park+(s.neutral_hinge_forward_m+linear)*forward
        np.testing.assert_allclose(hinge, c['base_xy_m'], atol=1e-14)


def test_segment_circle_intersections_include_both_branches_and_tangency_once():
    roots = m.segment_circle_intersections([[-2., .5], [2., .5]], [0., 0.], 1.)
    assert len(roots) == 2
    for _, p in roots:
        assert np.linalg.norm(p) == pytest.approx(1.)
    assert len(m.segment_circle_intersections([[-2., 1.], [2., 1.]], [0, 0], 1.)) == 1
    assert m.segment_circle_intersections([[2., 1.], [3., 1.]], [0, 0], 1.) == []


def test_gathering_preserves_distance_and_reconstructs_the_same_hinge():
    n = m.make_needle(candidate(), m.NeedleSettings(), 'a')
    source = map_with([n])
    original = copy.deepcopy(source)
    g = m.gather_needles(source, band_m=[.3, .4], anchor_angle_deg=-90.)
    assert g['intersecting_needles'] == 1
    assert g['outside_band'] == 0
    assert g['needles']
    for row in g['needles']:
        a = row['rotation_rad']
        np.testing.assert_allclose(row['segment_xy_m'], m.rotate(n['segment_xy_m'], a), atol=1e-13)
        t = row['segment_fraction']
        np.testing.assert_allclose((1-t)*np.array(row['segment_xy_m'][0])+t*np.array(row['segment_xy_m'][1]),
                                   row['parked_xy_m'], atol=1e-12)
        assert .3 <= np.linalg.norm(row['parked_xy_m']) <= .4
        assert row['parked_xy_m'][0] == pytest.approx(0, abs=1e-12)
        forward = np.array([math.cos(row['heading_rad']), math.sin(row['heading_rad'])])
        hinge = np.array(row['parked_xy_m'])+(.343+row['selected_linear_m'])*forward
        np.testing.assert_allclose(hinge, row['hinge_xy_m'], atol=1e-12)
    assert source == original


def test_a_nonintersecting_needle_is_counted_instead_of_translated_or_stretched():
    n = m.make_needle(candidate(base=(3., 3.)), m.NeedleSettings(), 'a')
    g = m.gather_needles(map_with([n]), band_m=[.2, .4])
    assert g['needles'] == []
    assert g['outside_band'] == 1


def test_symmetry_duplicates_collapse_but_different_joint_answers_survive():
    n = m.make_needle(candidate(), m.NeedleSettings(), 'a')
    b = m.rotate_needle(n, 1.3)
    b['id'] = 'b'
    c = copy.deepcopy(n)
    c['id'] = 'c'
    c['turns_rad'][0] += .1
    g = m.gather_needles(map_with([n, b, c]), band_m=[.3, .4])
    assert g['duplicate_rotations'] >= 1
    assert len(g['needles']) == 2


def test_automatic_band_selects_consensus_and_does_not_fill_disconnected_gaps():
    consensus = m.suggest_gather_band([[.1, .3], [.2, .4], [.7, .8]])
    assert consensus['band_m'] == pytest.approx([.2, .3])
    assert consensus['peak_support'] == 2
    assert m.suggest_gather_band([])['band_m'] is None


def test_circle_gathering_rejects_other_shapes_even_if_they_look_round():
    with pytest.raises(m.InputError, match='circle'):
        m.gather_needles(dict(target=dict(kind='custom'), needles=[]), band_m=[.2, .4])


@pytest.mark.parametrize('kwargs', [dict(iterations=0), dict(iterations=1.5),
                                   dict(time_limit_s=-1),
                                   dict(linear_min_m=0, linear_max_m=-.1), dict(seed=-1)])
def test_map_options_fail_explicitly(kwargs):
    with pytest.raises(m.InputError):
        m.NeedleSettings(**kwargs).checked()


def test_map_records_all_accepted_candidates_before_top_result_truncation(monkeypatch):
    def fake_solve(vertices, geometry, settings, **kwargs):
        for i in range(12):
            kwargs['on_candidate'](candidate(.3+i/100, base=(.2+i/100, .3)))
        return dict(candidates=[candidate()], target_vertices_m=vertices.tolist(),
                    search=dict(status='candidates_found', attempts=12))
    monkeypatch.setattr(m, 'solve', fake_solve)
    result = m.collect_needle_map(m.template('circle', .1, .1), needle_settings=m.NeedleSettings(iterations=12),
                                  circle_radius_m=.05)
    assert len(result['candidates']) == 1
    assert len(result['needle_map']['needles']) == 12
    assert result['needle_map']['filter']['operator'] == '<'
    assert 'minimum' not in result['needle_map']['filter']


def test_exact_circle_clearance_detects_penetration_hidden_by_polygon_approximation():
    r = .05
    a = math.pi/64
    centre = (r-1e-5)*np.array([math.cos(a), math.sin(a)])
    tiny = centre+np.array([[0, 0], [.001, 0], [.001, .001], [0, .001]])
    assert m.circle_gap(tiny, r) < 0


def test_different_radii_survive_band_gathering_without_forcing_one_parking_point():
    s = m.NeedleSettings()
    a = m.make_needle(candidate(base=(.2, .3)), s, 'a')
    b = m.make_needle(candidate(base=(.2, .31)), s, 'b')
    outlier = m.make_needle(candidate(base=(3., 3.)), s, 'outlier')
    g = m.gather_needles(map_with([a, b, outlier]))
    assert g['outside_band'] == 1
    radii = [n['anchor_radius_m'] for n in g['needles']]
    assert max(radii)-min(radii) > .005
    for row in g['needles']:
        assert g['band_m'][0] <= row['anchor_radius_m'] <= g['band_m'][1]
        assert row['parked_xy_m'][0] == pytest.approx(0, abs=1e-12)


def test_annulus_clipping_preserves_two_separate_rail_sections_and_the_hole():
    pieces = m.clip_segment_to_band([[-2., 0], [2., 0]], [0, 0], [.5, 1.])
    np.testing.assert_allclose(pieces, [[.25, .375], [.625, .75]])


@pytest.mark.parametrize('kwargs', [dict(band_m=[.5, .3]), dict(band_m=[.3, float('inf')]),
                                   dict(support_fraction=0), dict(support_fraction=1.1)])
def test_bad_band_parameters_fail_instead_of_warping_needles(kwargs):
    with pytest.raises(m.InputError):
        m.gather_needles(map_with([]), **kwargs)


def test_exact_circle_collection_records_beyond_top_one_and_is_reproducible():
    kwargs = dict(settings=m.SearchSettings(results=1, operating_mu=1000), needle_settings=m.NeedleSettings(iterations=16, seed=32),
                  circle_radius_m=.05)
    first = m.collect_needle_map(m.template('circle', .1, .1), **kwargs)
    second = m.collect_needle_map(m.template('circle', .1, .1), **kwargs)
    assert len(first['candidates']) == 1
    assert len(first['needle_map']['needles']) == 16
    assert first['needle_map'] == second['needle_map']
    for n in first['needle_map']['needles']:
        np.testing.assert_allclose(np.linalg.norm(n['contacts_m'], axis=1), [.05, .05], atol=1e-12)
        np.testing.assert_allclose(-np.array(n['contacts_m'])/.05, n['inward_normals'], atol=1e-12)
        assert n['geometry']['camera_target_clearance_m'] > .001
        assert n['go2_body_collision_verified'] is None


def test_explicit_cancellation_returns_a_saveable_partial_map():
    stop = threading.Event()
    def update(p):
        if p.get('recorded_needles', 0) >= 4:
            stop.set()
    r = m.collect_needle_map(m.template('circle', .1, .1),
                             needle_settings=m.NeedleSettings(iterations=256), circle_radius_m=.05,
                             cancelled=stop.is_set, progress=update)
    assert r['search']['status'] == 'cancelled'
    assert 4 <= len(r['needle_map']['needles']) < 256
    assert r['needle_map']['gathered']['needles']
    ET.fromstring(m.needle_svg_document(r, gathered=True))


def test_rotating_real_solutions_preserves_exact_circle_clearance_and_friction():
    r = m.collect_needle_map(m.template('circle', .1, .1),
                             needle_settings=m.NeedleSettings(iterations=8), circle_radius_m=.05)
    g = r['needle_map']['gathered']
    assert g['needles']
    for row in g['needles']:
        surfaces, bodies, _, _ = m.robot_polygons(row['turns_rad'])
        angle = row['heading_rad']-math.pi/2
        polygons = [m.rotate(p, angle)+row['hinge_xy_m'] for p in surfaces+bodies]
        assert min(m.circle_gap(p, .05) for p in polygons) >= -m.SearchSettings().tolerance_m
        assert m.circle_gap(polygons[m.CAMERA_INDEX], .05) > .001
        score = m.friction_score(row['contacts_m'], row['inward_normals'])
        assert score['mu_critical'] == pytest.approx(row['friction']['mu_critical'], abs=1e-12)
        for t in np.linspace(0, 1, 7):
            p = (1-t)*np.array(row['band_segment_xy_m'][0])+t*np.array(row['band_segment_xy_m'][1])
            assert g['band_m'][0]-1e-10 <= np.linalg.norm(p) <= g['band_m'][1]+1e-10


def test_empty_filter_and_deadline_produce_valid_json_and_svg_without_fake_band():
    import json
    r = m.collect_needle_map(m.template('circle', .1, .1),
                             needle_settings=m.NeedleSettings(iterations=16, time_limit_s=1e-9), circle_radius_m=.05)
    assert r['search']['deadline_reached']
    assert r['needle_map']['needles'] == []
    assert r['needle_map']['gathered']['band_m'] is None
    json.dumps(r, allow_nan=False)
    ET.fromstring(m.needle_svg_document(r, gathered=True))


def test_application_gather_refuses_stale_job_and_preserves_raw_map():
    app = m.Application()
    job = app.start(m.template('circle', .1, .1), m.Geometry(), m.SearchSettings(),
                    m.NeedleSettings(iterations=8), .05)
    app.worker.join(10)
    assert not app.running and app.error is None
    before = copy.deepcopy(app.result['needle_map']['needles'])
    with pytest.raises(m.InputError, match='current job'):
        app.gather(dict(job='stale'))
    g = app.gather(dict(job=job, support_fraction=.6, anchor_angle_deg=10))
    assert g['consensus']['support_fraction'] == .6
    assert app.result['needle_map']['needles'] == before
