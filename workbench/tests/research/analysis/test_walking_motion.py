"""A stable, accurately moving foot must not hide a frozen/sluggish body."""
import math

import pytest

from workbench.research.analysis.walking_motion import summarize_motion


def recording(speed=.8, *, command=(.8, 0., 0.), pitch=0., yaw=0., height=.3):
    direction = math.atan2(command[1], command[0]) + yaw
    velocity = [speed*math.cos(direction), speed*math.sin(direction), 0.]
    rows = [dict(kind='configuration', command=command, settle=2., duration=6., stop=2., cycles=1)]
    rows += [dict(kind='sample', t=4.+i*.02, pose_stage='walk', position=[0., 0., height],
                  rpy=[0., pitch, yaw], faulted=False, fallen=False,
                  centroidal_control=dict(base_velocity_world=velocity)) for i in range(200)]
    rows += [dict(kind='summary', completed=True, cycle_stops=[dict(completed=True)],
                  last=dict(pose_stage='stand', fallen=False, faulted=False))]
    return rows


@pytest.mark.parametrize('speed', [0., .125, .26, -.4])
def test_current_slow_and_opposite_gaits_fail_even_when_they_complete(speed):
    result = summarize_motion(recording(speed))
    assert not result['passed']
    assert 'speed_tracking' in result['failures']


@pytest.mark.parametrize('command', [(.8, 0., 0.), (-.8, 0., 0.), (0., .35, 0.), (0., -.35, 0.)])
def test_motion_is_measured_along_the_actual_body_frame_command(command):
    requested = math.hypot(*command[:2])
    result = summarize_motion(recording(requested*.8, command=command, yaw=.7))
    assert result['passed']
    assert result['speed_ratio'] == pytest.approx(.8)


@pytest.mark.parametrize('pitch,height,failure', [(math.radians(10), .3, 'pitch'), (0., .4, 'height')])
def test_speed_does_not_buy_an_excessive_body_tilt_or_height(pitch, height, failure):
    result = summarize_motion(recording(pitch=pitch, height=height))
    assert not result['passed'] and failure in result['failures']


def test_failure_to_stop_is_not_hidden_by_successful_forward_motion():
    rows = recording()
    rows[-1]['cycle_stops'][0]['completed'] = False
    result = summarize_motion(rows)
    assert not result['passed'] and 'completion_or_stop' in result['failures']


def test_missing_actual_velocity_is_rejected():
    rows = recording()
    del rows[1]['centroidal_control']['base_velocity_world']
    with pytest.raises(ValueError, match='velocity'):
        summarize_motion(rows)
