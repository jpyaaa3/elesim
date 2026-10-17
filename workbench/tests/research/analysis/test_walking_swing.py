"""Verify the air-stall metric independently of a successful walking summary."""
from copy import deepcopy
import json

import pytest

from workbench.research.analysis.walking_swing import main, summarize_swing_tracking


def trace():
    rows = [dict(kind='configuration', gait_hz=2.5, gait_duty=.6, settle=2., duration=12.)]
    for i, (phase, x, vz) in enumerate(((.25, .015625, 1.125), (.75, .084375, -1.125))):
        rows.append(dict(kind='sample', t=4.5+.02*i, pose_stage='walk',
            leg_control=dict(swing_starts=[[0., 0., .025]]*4,
                             touchdowns=[[.1, 0., .025]]*4, touchdown_depths=[0.]*4),
            centroidal_control=dict(contacts=[0., 1., 1., 1.],
                feet_world=[[x, 0., .05875]]*4, foot_velocity_world=[[.703125, 0., vz]]*4),
            contact_gate=dict(swing_progress=[phase]*4, search=[False]*4)))
    return rows


def test_exact_tracking_has_no_position_error_or_air_stall():
    report = summarize_swing_tracking(trace())
    assert report['all']['n'] == 2
    assert report['all']['position_error_p50_p95_mm'] == pytest.approx([0., 0.], abs=1e-10)
    assert report['all']['velocity_projection_ratio'] == pytest.approx(1.)
    assert report['all']['descending_airborne_samples'] == 1
    assert report['all']['descending_airborne_stalled_fraction'] == 0.


def test_stationary_foot_is_counted_as_stalled_only_during_descent():
    rows = trace()
    for row in rows[1:]:
        row['centroidal_control']['foot_velocity_world'] = [[0., 0., 0.]]*4
        row['centroidal_control']['feet_world'][0][0] += .01
    report = summarize_swing_tracking(rows)
    assert report['all']['position_error_p50_p95_mm'] == pytest.approx([10., 10.])
    assert report['all']['velocity_projection_ratio'] == 0.
    assert report['accelerating_half']['descending_airborne_samples'] == 0
    assert report['decelerating_half']['descending_airborne_stalled_fraction'] == 1.


def test_search_and_stance_do_not_claim_a_successful_swing():
    rows = trace()
    for row in rows[1:]:
        row['contact_gate']['search'][0] = True
    assert summarize_swing_tracking(rows)['all'] == {'n': 0}


def test_missing_latched_start_is_rejected():
    rows = trace()
    del rows[1]['leg_control']['swing_starts']
    with pytest.raises(ValueError, match='latched'):
        summarize_swing_tracking(rows)


def test_cli_writes_the_same_reproducible_summary(tmp_path, capsys):
    rows = deepcopy(trace())
    (tmp_path/'forward08.jsonl').write_text('\n'.join(json.dumps(r) for r in rows)+'\n')
    main([str(tmp_path)])
    result = json.loads((tmp_path/'swing-summary.json').read_text())
    assert result == json.loads(capsys.readouterr().out)
    assert result[0]['name'] == 'forward08'
    assert result[0]['all']['n'] == 2
