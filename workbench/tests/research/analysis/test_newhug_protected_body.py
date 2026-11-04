"""Only arm/gripper contacts; housing and wedge have strict protected clearance."""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from workbench.research.support import newhug as m


def body_pose(part, gap, *, circle=False, settings=None, forbidden_part=None):
    # Put a target immediately outside an exposed left boundary. The two dummy
    # contact features are irrelevant: inspect body-policy reasons independently.
    x, y = (-.065, -.14) if part == 'housing' else (-.042, -.013647)
    radius = .005
    if circle:
        vertices = m.template('circle', 2*radius, 2*radius)
        translation = np.array([x-radius-gap, y])
    else:
        vertices = np.array([[x-gap-.005, y-.0025], [x-gap, y-.0025],
                             [x-gap, y+.0025], [x-gap-.005, y+.0025]])
        translation = np.zeros(2)
    features = (m.Feature(vertices[0], np.array([-1., 0.]), -math.pi/2, 0.),
                m.Feature(vertices[1], np.array([1., 0.]), math.pi/2, .5))
    faces = m.robot_faces(m.Geometry())
    face = faces[0] if forbidden_part is None else replace(faces[0], part=forbidden_part)
    return m.validate_configuration(vertices, m.convex_parts(vertices), [0., 0.], 0., translation,
                                    face, faces[-1], .5, .5, features, m.Geometry(),
                                    settings or m.SearchSettings(), circle_radius_m=radius if circle else None)[0]


def test_contact_policy_is_explicit_and_matches_only_the_requested_groups():
    info = m.describe_arm()
    assert set(info['allowed_contact_groups']) == {'seg1', 'seg2', 'gripper_base', 'gripper_claw'}
    assert set(info['allowed_contact_part_names']) == set(m.PART_NAMES[:m.CAMERA_INDEX])
    assert set(info['protected_part_names']) == {'camera', 'housing', 'wedge'}
    assert all(m.contact_part_allowed(f.part) for f in m.robot_faces(m.Geometry()))


@pytest.mark.parametrize('part', ['housing', 'wedge'])
@pytest.mark.parametrize('circle', [False, True])
def test_touch_is_forbidden_even_with_zero_margin_and_large_geometry_tolerance(part, circle):
    settings = m.SearchSettings(body_clearance_m=0., tolerance_m=.001)
    for gap in (0., -.000025):
        metrics = body_pose(part, gap, circle=circle, settings=settings)
        assert 'body_contact' in metrics['reasons']
        assert 'body_clearance' in metrics['reasons']
        assert not metrics['valid']


@pytest.mark.parametrize('part', ['housing', 'wedge'])
@pytest.mark.parametrize('circle', [False, True])
def test_body_clearance_has_a_strict_boundary_and_reports_each_protected_part(part, circle):
    for gap in (.0002, .001, .00101):
        metrics = body_pose(part, gap, circle=circle)
        assert metrics['body_target_clearances_m'][part] == pytest.approx(gap, abs=1e-9)
        assert metrics['required_body_clearance_m'] == .001
        assert ('body_clearance' in metrics['reasons']) == (gap <= .001)
        assert 'body_contact' not in metrics['reasons']


@pytest.mark.parametrize('part', [-1, 13, 14, 15])
def test_final_validator_rejects_a_protected_or_unknown_contact_surface(part):
    metrics = body_pose('housing', .01, forbidden_part=part)
    assert 'forbidden_contact_surface' in metrics['reasons']
    assert not metrics['valid']
    json.dumps(metrics, allow_nan=False)


@pytest.mark.parametrize('gap', [-.001, .051, float('nan')])
def test_body_margin_is_validated_independently_of_camera_margin(gap):
    with pytest.raises(m.InputError):
        m.SearchSettings(body_clearance_m=gap).checked()


def test_exact_circle_map_respects_body_keepout_and_only_records_allowed_contacts():
    settings = m.SearchSettings(body_clearance_m=.05)
    r = m.collect_needle_map(m.template('circle', .1, .1), settings=settings,
                             needle_settings=m.NeedleSettings(iterations=8, time_limit_s=5), circle_radius_m=.05)
    assert r['needle_map']['needles'], r['search']
    for n in r['needle_map']['needles']:
        assert n['geometry']['body_target_clearance_m'] > .05
        assert all(s['part'] in m.ALLOWED_CONTACT_PART_NAMES for s in n['contact_surfaces'])
    assert r['needle_map']['model_scope']['protected_parts'] == ['camera', 'housing', 'wedge']
