import numpy as np
import pytest
from elesim_sim.rl.trajectory import Trajectory
from workbench.research.experiments.hug_interpolation.interpolate import CHANNELS, interpolate, weights


def case(xy, band=1):
    return dict(case_id=str(xy), xy_mm=xy, band=band, roll_deg=-90 if band == 1 else 90,
                diameter_mm=150, branch='local-01', simulation_context={'model_sha256': 'a', 'friction': .5},
                start_command=[0, -np.pi/2 if band == 1 else np.pi/2, 0, 0],
                goal_command=[0, -np.pi/2 if band == 1 else np.pi/2, .1, -.4], duration_s=2.)


def anchors():
    result = []
    for xy in ([0, 0], [10, 0], [0, 10]):
        c = case(xy)
        start, end = np.array(c['start_command']), np.array(c['goal_command'])
        mid = (start+end)/2 + np.array([0, 0, .05, -.05])
        result.append(Trajectory([0, 1, 2], [start, mid, end], CHANNELS, c, {'replay_success': True}))
    return result


def test_residual_preserves_target_endpoints_and_shape():
    target = case([2, 3]); target['goal_command'][2] = .2
    result = interpolate(anchors(), target, samples=3)
    np.testing.assert_allclose(result.commands[0], target['start_command'])
    np.testing.assert_allclose(result.commands[-1], target['goal_command'])
    np.testing.assert_allclose(result.commands[1, 2:], [.15, -.25])
    np.testing.assert_allclose(result.metadata['weights'], [.5, .2, .3])
    assert result.diagnostics['physics_validated'] is False


@pytest.mark.parametrize('key,value', [('band', 2), ('roll_deg', 90), ('diameter_mm', 155),
                                       ('branch', 'separate'), ('simulation_context', {'friction': 1})])
def test_cannot_mix_families_or_physics(key, value):
    target = case([2, 3]); target[key] = value
    with pytest.raises(ValueError):
        interpolate(anchors(), target)


def test_must_replay_originals_first():
    data = anchors(); data[1].diagnostics['replay_success'] = False
    with pytest.raises(ValueError):
        interpolate(data, case([2, 3]))


def test_no_extrapolation_or_degenerate_triangles():
    with pytest.raises(ValueError):
        weights([[0, 0], [1, 0], [0, 1]], [1, 1])
    with pytest.raises(ValueError):
        weights([[0, 0], [1, 1], [2, 2]], [.5, .5])


def test_cannot_bend_toward_camera():
    target = case([2, 3]); target['goal_command'][3] = .1
    with pytest.raises(ValueError, match='ZED'):
        interpolate(anchors(), target)


def test_prepare_two_local_triangles_without_split_leakage(tmp_path):
    import json
    from workbench.research.experiments.hug_interpolation.prepare import prepare
    rows = [dict(status='found', x_mm=-100+i*5, y_mm=150+j*5,
                 roll_solutions=[dict(roll_deg=r, seg1_deg=20, seg2_deg=-120, mu_min=.3)
                                 for r in (-90, 90)])
            for i in range(5) for j in range(5-i)]
    table = dict(metadata=dict(schema='hugmap-table', schema_version=3, object={'shape': 'circle'},
                               config={'dx_mm': 5, 'dy_mm': 5}),
                 maps=[dict(diameter_mm=150, status='complete', results=rows)])
    path = tmp_path/'map.json'; path.write_text(json.dumps(table))
    result = prepare(path)
    assert len(result['cases']) == 30
    assert sum(c['split'] == 'train' for c in result['cases']) == 6
    assert sum(c['split'] == 'test' for c in result['cases']) == 2
    assert len({c['case_id'] for c in result['cases']}) == 30
    assert result['simulation_context'] is None
    for c in result['cases']:
        assert c['goal_command'][3] == pytest.approx(np.deg2rad(-120)/5)
    rows.pop(0)
    path.write_text(json.dumps(table))
    with pytest.raises(ValueError, match='gaps'):
        prepare(path)
    table['metadata']['schema_version'] = 2
    path.write_text(json.dumps(table))
    with pytest.raises(ValueError, match='v3'):
        prepare(path)
