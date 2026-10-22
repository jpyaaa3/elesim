"""Physical landing waits retain attained turning, just as linear progress."""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_68_accelerating_footholds import sample_controller


def turning_sample(*, command=.8, attained=.6, scale=1., posture=False,
                   backend='jax_mppi', blocked=True):
    c, q, _ = sample_controller()
    rpy = np.array([.12, -.10, .7])
    rotation = Rotation.from_euler('xyz', rpy)
    q[3:7] = rotation.as_quat()
    _, dq = c._bridge.read_pin_q_dq()
    dq[3:6] = rotation.inv().apply([0., 0., attained])
    c._cmd = Go2Command(vx=.8, yaw_rate=command)
    c._command_shaper = Go2CommandShaper(current=c._cmd, target=c._cmd)
    c._command_scale = lambda: scale
    c._contact_gait.blocked = blocked
    c._contact_gait.posture_hold = posture
    c._solver.backend = backend
    return c, rpy, rotation


@pytest.mark.parametrize('command,attained,expected', [
    (.8, .6, .6), (-.8, -.6, -.6),
    (.8, 1.2, .8), (-.8, -1.2, -.8),
    (.8, -.3, 0.), (-.8, .3, 0.),
    (.8, 0., 0.), (0., .6, 0.),
])
def test_landing_wait_keeps_only_bounded_attained_turning(command, attained, expected):
    c, rpy, rotation = turning_sample(command=command, attained=attained)
    sample, *_ = c._sample()
    np.testing.assert_allclose(sample.desired_angular_velocity_body,
                               rotation.inv().apply([0., 0., expected]), atol=1e-12)
    # The seed must not request extra yaw acceleration through angle lead.
    assert sample.desired_rpy[2] == pytest.approx(rpy[2])
    assert c._cmd.yaw_rate == command
    assert c._command_shaper.current.yaw_rate == command
    assert c._command_shaper.target.yaw_rate == command


def test_retained_yaw_uses_euler_rate_instead_of_body_z_rate():
    c, _, rotation = turning_sample(attained=.5)
    _, dq = c._bridge.read_pin_q_dq()
    # Add roll/pitch motion: the yaw rate still follows exact xyz kinematics.
    dq[3:6] += [0.1, .2, 0.]
    roll, pitch, _ = rotation.as_euler('xyz')
    expected = (np.sin(roll)*dq[4]+np.cos(roll)*dq[5])/np.cos(pitch)
    sample, *_ = c._sample()
    np.testing.assert_allclose(sample.desired_angular_velocity_body,
                               rotation.inv().apply([0., 0., expected]), atol=1e-12)


@pytest.mark.parametrize('scale,expected', [(0., 0.), (.25, .2), (1., .6)])
def test_attained_yaw_respects_the_effective_startup_command_scale(scale, expected):
    c, _, rotation = turning_sample(scale=scale)
    sample, *_ = c._sample()
    np.testing.assert_allclose(sample.desired_angular_velocity_body,
                               rotation.inv().apply([0., 0., expected]), atol=1e-12)


@pytest.mark.parametrize('posture,backend', [(True, 'jax_mppi'), (False, 'acados')])
def test_posture_recovery_and_legacy_cpu_landing_keep_the_existing_yaw_brake(posture, backend):
    c, _, _ = turning_sample(posture=posture, backend=backend)
    sample, *_ = c._sample()
    np.testing.assert_array_equal(sample.desired_angular_velocity_body, np.zeros(3))
    assert all(call.args[5] == 0. for call in c._predict_touchdowns.call_args_list)


def test_unblocked_reference_and_operator_acceleration_are_unchanged():
    c, rpy, _ = turning_sample(blocked=False)
    sample, *_ = c._sample()
    np.testing.assert_array_equal(sample.desired_angular_velocity_body, [0., 0., .8])
    assert sample.desired_rpy[2] == pytest.approx(rpy[2]+.5*c._solver.horizon*c._solver.dt*.8)


def test_foothold_prediction_receives_the_same_retained_yaw_as_the_controller():
    c, _, rotation = turning_sample()
    sample, *_ = c._sample()
    assert c._predict_touchdowns.call_count == 2
    for call in c._predict_touchdowns.call_args_list:
        assert call.args[5] == pytest.approx(.6)
    np.testing.assert_allclose(rotation.apply(sample.desired_angular_velocity_body),
                               [0., 0., .6], atol=1e-12)
