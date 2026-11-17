"""Latch four-foot braking before another swing can miss the stop window."""
from types import SimpleNamespace

import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.pose import JointPoseTransition
from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller


def walking(backend='jax_mppi'):
    c, state = controller()
    c._solver.backend = backend
    c._sim_time = 1.2  # Wall clock and contact-gait clock are independent.
    c._config.command_idle_threshold = .05
    c._command_shaper = Go2CommandShaper(stop_dwell_s=.2)
    c._command_shaper._zero_since_s = .99
    c._bridge = SimpleNamespace(last_q=np.r_[np.zeros(3), [0., 0., 0., 1.], np.zeros(12)],
                                last_dq=np.zeros(18))
    c._pose_stage, c._torque_mode_active, c._faulted = 'walk', True, False
    c._pose_transition = JointPoseTransition(.35)
    c._pose_transition.reset(np.zeros(12))
    c._sample = lambda: (state, state.feet_world, np.zeros((4, 3)),
                         np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    c._torques = lambda *_: np.zeros(12)
    c._entity.control_dofs_force = lambda *_args, **_kw: None
    c._as_force_tensor = np.asarray  # Exercise the state machine on the fake entity.
    calls = []
    c._entity.control_dofs_position = lambda *_args, **_kw: calls.append('position')
    def begin():
        calls.append('begin_stand')
        c._pose_transition.begin(np.zeros(12), np.full(12, .1))
        c._pose_stage = 'to_stand'
    c._begin_stand_pose = begin
    return c, calls


@pytest.mark.parametrize('phase', [.05, .3, .55, .8])
def test_gpu_freezes_new_liftoffs_after_braking_dwell_while_the_current_foot_lands(phase):
    c, calls = walking()
    c._contact_gait.time = phase/c._config.gait_hz
    c._contact_gait.confirmed[0] = False
    c._contact_gait.load[0] = 0.
    c._contact_gait.blocked = True
    c._step()
    assert c._settling
    assert c._pose_stage == 'walk'
    assert not c._contact_gait.confirmed[0]
    assert calls == []


@pytest.mark.parametrize('phase', [.05, .3, .55, .8])
def test_actual_four_foot_support_owns_the_gpu_handoff_not_the_wall_clock_phase(phase):
    c, calls = walking()
    c._contact_gait.time = phase/c._config.gait_hz
    c._contact_gait.confirmed[:] = True
    c._contact_gait.load[:] = 1.
    c._contact_gait.blocked = False
    c._step()
    assert c._pose_stage == 'to_stand'
    assert calls == ['begin_stand', 'position']


def test_legacy_cpu_keeps_its_nominal_support_window():
    c, calls = walking('acados')
    c._step()
    assert c._pose_stage == 'walk'
    assert not c._settling
    assert calls == []


@pytest.mark.parametrize('backend', ['jax_mppi', 'acados'])
def test_a_nonzero_reversal_does_not_latch_braking_at_the_zero_crossing(backend):
    c, calls = walking(backend)
    c._command_shaper.set_target(Go2Command(vx=-.8))
    c._step()
    assert not c._settling
    assert c._pose_stage == 'walk'
    assert calls == []


@pytest.mark.parametrize('command',[Go2Command(vx=.004),Go2Command(vy=-.004),Go2Command(yaw_rate=.003)])
def test_small_goal_corrections_do_not_trigger_a_stand_transition(command):
    c,calls=walking()
    c._command_shaper.set_target(command)
    c._step()
    assert not c._settling and c.locomotion_stage=='walk'
    assert not calls


def test_zero_command_hands_normal_walking_tilt_to_leg_position_leveling():
    from scipy.spatial.transform import Rotation
    c,calls=walking()
    c._bridge.last_q[3:7]=Rotation.from_euler('xyz',[.12,-.1,.2]).as_quat()
    c._contact_gait.confirmed[:]=True;c._contact_gait.load[:]=1.
    c._contact_gait.blocked=False
    c._step()
    assert c.locomotion_stage=='to_stand'
    assert calls==['begin_stand','position']
