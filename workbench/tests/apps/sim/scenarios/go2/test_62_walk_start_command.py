"""Position-ready time must not consume the walking acceleration budget."""
import numpy as np
import pytest

from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper
from elesim_sim.robot.go2.locomotion.types import Go2Command
from workbench.tests.apps.sim.scenarios.go2.test_58_stance_tracking import controller


def ready_controller(command, backend='jax_mppi'):
    c, state = controller()
    c._solver.backend = backend
    c._dt, c._sim_time, c._ready_until = .01, 1., 1.
    c._walk_started_s = None
    c._pose_stage = 'ready_hold'
    c._faulted = False
    c._torque_mode_active = False
    c._contact_diagnostics = None
    c._command_shaper = Go2CommandShaper(current=command, target=command)
    c._cmd = command
    c._config.command_idle_threshold = .05
    c._config.command_ramp_s = .15
    c._entity.control_dofs_position = lambda *args, **kwargs: None
    c._entity.control_dofs_force = lambda *args, **kwargs: None
    c._set_torque_actuation = lambda: None
    observed = []
    def sample():
        observed.append((c._cmd, c._command_scale()))
        return state, state.feet_world, np.zeros((4, 3)), np.tile(np.eye(3), (4, 1, 1)), np.zeros(12)
    c._sample = sample
    c._torques = lambda *args: np.zeros(12)
    return c, observed


@pytest.mark.parametrize('command', [Go2Command(vx=.8), Go2Command(vx=-.8),
    Go2Command(vy=.35), Go2Command(vx=.8, vy=.35, yaw_rate=.8)])
def test_gpu_start_rewinds_only_current_velocity_and_keeps_held_intent(command):
    c, observed = ready_controller(command)
    c._step()
    assert c._pose_stage == 'walk'
    assert c._command_shaper.target == command
    assert c._command_shaper.current == Go2Command()
    assert c._cmd == Go2Command()
    previous = np.zeros(3)
    for _ in range(80):
        c._step()
        actual, scale = observed[-1]
        v = np.array([actual.vx, actual.vy, actual.yaw_rate])*scale
        assert np.linalg.norm(v[:2]-previous[:2]) <= 1.2*c._dt+1e-12
        assert abs(v[2]-previous[2]) <= 3.*c._dt+1e-12
        previous = v
    np.testing.assert_allclose(previous, [command.vx, command.vy, command.yaw_rate])


def test_gpu_has_one_acceleration_limiter_in_the_actual_foothold_command():
    c, observed = ready_controller(Go2Command(vx=.8))
    c._step()
    c._step()
    actual, scale = observed[-1]
    assert actual.vx == pytest.approx(.012)
    assert scale == 1.


def test_cpu_retains_its_existing_ready_command_and_quintic_ramp():
    c, observed = ready_controller(Go2Command(vx=.8), 'acados')
    c._step()
    assert c._command_shaper.current == Go2Command(vx=.8)
    c._step()
    actual, scale = observed[-1]
    assert actual.vx == pytest.approx(.8)
    assert 0. < scale < 1.
