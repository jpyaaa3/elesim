from __future__ import annotations

import numpy as np
import pytest
from types import SimpleNamespace

from elesim_sim.robot.go2.locomotion.kinematics import NOMINAL_FOOT_OFFSET_BODY
from elesim_sim.robot.go2.locomotion.command import Go2CommandShaper, trot_all_stance
from elesim_sim.robot.go2.locomotion.pose import JointPoseTransition, smoothstep_quintic
from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, Go2Command
from elesim_sim.robot.go2.pympc_controller import (
    PyMpcGenesisController,
    contact_schedule,
    touchdown_offsets_body,
)
from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput


class FakeSolver:
    def __init__(self, force=None, status=0):
        self.force = np.asarray(
            force if force is not None else [4, 3, 90] * 4, dtype=float
        )
        self.status = status
        self.received = None

    def compute_control(self, state, reference, contacts, **kwargs):
        self.received = (state, reference, contacts, kwargs)
        return self.force, np.zeros((4, 3)), np.zeros(24), self.status


def sample() -> PyMpcInput:
    feet = np.array([[0.2, 0.1, 0.02], [0.2, -0.1, 0.02],
                     [-0.2, 0.1, 0.02], [-0.2, -0.1, 0.02]])
    return PyMpcInput(
        com_position=np.array([1.0, 2.0, 0.3]),
        com_velocity=np.array([0.1, 0.2, 0.0]),
        rpy=np.array([0.01, 0.02, 0.03]),
        angular_velocity_body=np.array([0.0, 0.0, 0.1]),
        feet_world=feet,
        desired_com_position=np.array([1.1, 2.1, 0.3]),
        desired_com_velocity=np.array([0.2, 0.1, 0.0]),
        desired_rpy=np.array([0.0, 0.0, 0.04]),
        desired_angular_velocity_body=np.array([0.0, 0.0, 0.1]),
        footholds_world=feet.copy(),
        contacts=np.array([[1, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 1]]),
        mass_kg=18.0,
        inertia_body=np.diag([0.2, 0.5, 0.5]),
    )


def _torque_config(**overrides):
    values = dict(
        gait_hz=2.5,
        gait_duty=0.6,
        force_filter_alpha=1.0,
        optimization_friction=0.55,
        fz_max_n=180.0,
        torque_warmup_s=0.0,
        torque_ramp_s=0.001,
        aux_kp=0.0,
        aux_kv=0.0,
        tau_filter_alpha=1.0,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def _init_torque_controller(controller, *, elapsed_s=1.0, config=None):
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._step_i = 0
    controller._solve_stride = 2
    controller._solver = SimpleNamespace(
        solve=lambda _sample: np.tile([0.0, 0.0, 50.0], (4, 1))
    )
    controller._timing_sink = None
    controller._config = config or _torque_config()
    controller._sim_time = elapsed_s
    controller._walk_started_s = 0.0
    controller._last_contacts = np.ones(4)
    controller._force_requested = np.zeros((4, 3))
    controller._forces = np.zeros((4, 3))
    controller._tau_filt = np.zeros(12)
    controller._tau_limited = np.zeros(12)
    controller._tau_raw = np.zeros(12)
    controller._swing_starts = np.zeros((4, 3))
    controller._touchdowns = np.zeros((4, 3))
    controller._tau_lim = np.full(12, 100.0)
    controller._kin = SimpleNamespace(ready_q=np.ones(12))
    controller._leg_dof_idxs = list(range(12))
    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: np.zeros(12)
    )


def test_diagonal_contact_sequence() -> None:
    schedule = contact_schedule(0.0, gait_hz=2.5, duty=0.5, dt=0.1, horizon=4)
    assert schedule.shape == (4, 4)
    np.testing.assert_array_equal(schedule[0], schedule[3])
    np.testing.assert_array_equal(schedule[1], schedule[2])
    np.testing.assert_array_equal(schedule[0] + schedule[1], np.ones(4))


def test_touchdown_preserves_nominal_foot_width_and_scales_commanded_stride() -> None:
    nominal = np.asarray([NOMINAL_FOOT_OFFSET_BODY[leg] for leg in ALL_LEGS])
    half_stance_s = 0.12
    placement_scale = 1.35
    placements = touchdown_offsets_body(
        nominal,
        command_body=np.array([0.2, 0.0, 0.0]),
        yaw_rate=0.0,
        half_stance_s=half_stance_s,
        placement_scale=placement_scale,
    )

    np.testing.assert_allclose(placements[:, 1], nominal[:, 1])
    np.testing.assert_allclose(
        placements[:, 0], nominal[:, 0] + 0.2 * half_stance_s * placement_scale
    )
    assert placements[0, 1] > 0.13
    assert placements[1, 1] < -0.13


def test_solver_passes_world_state_and_dynamic_payload_then_masks_swing() -> None:
    fake = FakeSolver(force=[30, -30, 300] * 4)
    adapter = PyMpcForceSolver(
        horizon=3, friction=0.5, max_normal_force_n=100,
        solver_factory=lambda: fake,
    )
    force = adapter.solve(sample())
    state, reference, contacts, kwargs = fake.received
    np.testing.assert_array_equal(state["position"], [1.0, 2.0, 0.3])
    np.testing.assert_array_equal(state["foot_FL"], [0.2, 0.1, 0.02])
    np.testing.assert_array_equal(reference["ref_foot_FL"], [[0.2, 0.1, 0.02]])
    np.testing.assert_array_equal(kwargs["inertia"], np.diag([0.2, 0.5, 0.5]).reshape(9))
    assert kwargs["mass"] == 18.0
    np.testing.assert_array_equal(contacts, sample().contacts)
    np.testing.assert_array_equal(force[0], [30, -30, 100])
    np.testing.assert_array_equal(force[1], [0, 0, 0])
    np.testing.assert_array_equal(force[2], [0, 0, 0])


def test_solver_uses_bounded_finite_force_when_iteration_limit_is_reached(capsys) -> None:
    fake = FakeSolver(force=[30, -30, 300] * 4, status=2)
    adapter = PyMpcForceSolver(
        horizon=3, friction=0.5, max_normal_force_n=100,
        solver_factory=lambda: fake,
    )

    force = adapter.solve(sample())
    adapter.solve(sample())

    np.testing.assert_array_equal(force[0], [30, -30, 100])
    np.testing.assert_array_equal(force[1], [0, 0, 0])
    np.testing.assert_array_equal(force[2], [0, 0, 0])
    assert np.all(np.isfinite(force))
    assert capsys.readouterr().out.count("using the finite, bounded force iterate") == 1


@pytest.mark.parametrize("force,status", [([1, 2, 3] * 4, 4), ([float("nan"), 0, 1] * 4, 0)])
def test_solver_failure_is_not_reused_as_valid_force(force, status) -> None:
    adapter = PyMpcForceSolver(horizon=3, solver_factory=lambda: FakeSolver(force, status))
    with pytest.raises((RuntimeError, ValueError)):
        adapter.solve(sample())


def test_invalid_inertia_and_contact_are_rejected_before_solver() -> None:
    adapter = PyMpcForceSolver(horizon=3, solver_factory=FakeSolver)
    valid = sample()
    with pytest.raises(ValueError, match="inertia"):
        adapter.solve(PyMpcInput(**{**vars(valid), "inertia_body": np.zeros((3, 3))}))
    with pytest.raises(ValueError, match="binary"):
        adapter.solve(PyMpcInput(**{**vars(valid), "contacts": np.full((4, 3), 0.5)}))


def test_stance_ground_reaction_is_opposed_by_joint_torque() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(controller)
    controller._solver = SimpleNamespace(solve=lambda _sample: np.tile([0, 0, 50], (4, 1)))
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    torques = controller._torques(
        state, state.feet_world, np.zeros((4, 3)), jacobians, np.zeros(12)
    )
    np.testing.assert_array_equal(torques.reshape(4, 3), np.tile([0, 0, -50], (4, 1)))


def test_force_filter_reprojects_stance_and_zeros_swing_legs() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(
        controller,
        config=_torque_config(
            force_filter_alpha=0.5,
            optimization_friction=0.4,
            fz_max_n=60.0,
        ),
    )
    controller._solver = SimpleNamespace(
        solve=lambda _sample: np.tile([60.0, -60.0, 100.0], (4, 1))
    )
    controller._forces[:] = [0.0, 0.0, 20.0]
    state = PyMpcInput(
        **{
            **vars(sample()),
            "contacts": np.array(
                [[1, 1, 1], [0, 0, 0], [1, 1, 1], [0, 0, 0]], dtype=float
            ),
        }
    )

    controller._torques(
        state,
        state.feet_world,
        np.zeros((4, 3)),
        np.tile(np.eye(3), (4, 1, 1)),
        np.zeros(12),
    )

    np.testing.assert_allclose(controller._forces[[0, 2]], [[24, -24, 60]] * 2)
    np.testing.assert_array_equal(controller._forces[[1, 3]], np.zeros((2, 3)))


def test_torque_startup_ramp_fades_ready_pose_assist() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(
        controller,
        elapsed_s=0.11,
        config=_torque_config(
            torque_warmup_s=0.05,
            torque_ramp_s=0.12,
            aux_kp=15.0,
            aux_kv=6.0,
        ),
    )
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    joint_velocity = np.ones(12)

    ramped = controller._torques(
        state, state.feet_world, np.zeros((4, 3)), jacobians, joint_velocity
    )

    np.testing.assert_allclose(
        ramped.reshape(4, 3), [[1.1625, 1.1625, -32.5875]] * 4
    )
    controller._sim_time = 0.17
    controller._step_i = 1  # reuse the filtered GRF; do not solve again
    completed = controller._torques(
        state, state.feet_world, np.zeros((4, 3)), jacobians, joint_velocity
    )
    np.testing.assert_allclose(completed.reshape(4, 3), [[-6.5, -6.5, -56.5]] * 4)


def test_payload_com_height_does_not_request_a_sudden_drop() -> None:
    from elesim_sim.robot.go2.pympc_controller import payload_aware_com_height

    assert payload_aware_com_height(0.42, 0.30, payload_active=False) == pytest.approx(0.30)
    assert payload_aware_com_height(0.42, 0.30, payload_active=True) == pytest.approx(0.41)
    assert payload_aware_com_height(0.29, 0.30, payload_active=True) == pytest.approx(0.30)


def test_idle_clears_previous_gait_force_and_rearms_torque_mode() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._dt = 0.02
    controller._sim_time = 1.0
    controller._walk_started_s = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05, gait_hz=2.5, gait_duty=0.6
    )
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._command_shaper._zero_since_s = 0.0
    controller._pose_transition = JointPoseTransition(0.35)
    controller._pose_transition.reset(np.ones(12))
    controller._pose_stage = "walk"
    controller._active = True
    controller._step_i = 9
    controller._force_requested = np.ones((4, 3))
    controller._forces = np.ones((4, 3))
    controller._tau_filt = np.ones(12)
    controller._tau_hold = np.ones(12)
    controller._tau_limited = np.ones(12)
    controller._tau_raw = np.ones(12)
    controller._last_contacts = np.zeros(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12))
    controller._leg_dof_idxs = list(range(12))
    calls = []
    controlled_pose = []

    def control_position(pose, **_kwargs):
        controlled_pose.append(np.asarray(pose, dtype=float).copy())

    def record_controlled_pose(pose, **kwargs):
        calls.append("position")
        control_position(pose, **kwargs)

    controller._set_stand_actuation = lambda: calls.append("stand")
    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: np.ones(12),
        control_dofs_position=record_controlled_pose,
    )
    controller.step()
    assert calls == ["stand", "position"]
    assert controller._step_i == 0
    assert not controller._active
    assert not controller._forces.any()
    assert not controller._force_requested.any()
    assert not controller._tau_filt.any()
    assert not controller._tau_hold.any()
    assert controller._last_contacts.all()
    assert np.max(np.abs(controlled_pose[0] - np.ones(12))) < 0.02


def test_idle_startup_stays_in_stand_during_stop_dwell() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._dt = 0.02
    controller._sim_time = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._pose_transition = JointPoseTransition(0.35)
    controller._pose_transition.reset(np.zeros(12))
    controller._pose_stage = "stand"
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05, gait_hz=2.5, gait_duty=0.6
    )
    controller._active = False
    controller._step_i = 0
    controller._forces = np.zeros((4, 3))
    controller._tau_hold = np.zeros(12)
    controller._last_contacts = np.ones(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12), ready_q=np.ones(12))
    controller._leg_dof_idxs = list(range(12))
    calls = []
    controller._set_stand_actuation = lambda: calls.append("stand")
    controller._entity = SimpleNamespace(
        control_dofs_position=lambda pose, **_kwargs: calls.append(tuple(pose))
    )

    controller.step()

    assert calls == [tuple(controller._kin.stand_q)]
    assert not controller._active


def test_command_shaper_limits_linear_and_yaw_acceleration_through_reversal() -> None:
    shaper = Go2CommandShaper(linear_accel_mps2=1.2, yaw_accel_radps2=3.0)
    shaper.set_target(Go2Command(vx=0.4, vy=-0.3, yaw_rate=0.8))
    forward = shaper.update(0.1)
    assert np.hypot(forward.vx, forward.vy) == pytest.approx(0.12)
    assert forward.vx == pytest.approx(0.096)
    assert forward.vy == pytest.approx(-0.072)
    assert forward.yaw_rate == pytest.approx(0.3)

    shaper.set_target(Go2Command(vx=-0.4, vy=0.3, yaw_rate=-0.8))
    previous = forward
    crossed_zero = [False, False, False]
    for _ in range(8):
        current = shaper.update(0.1)
        assert np.hypot(current.vx - previous.vx, current.vy - previous.vy) <= 0.12 + 1e-12
        assert abs(current.yaw_rate - previous.yaw_rate) <= 0.3 + 1e-12
        crossed_zero[0] |= previous.vx > 0.0 >= current.vx
        crossed_zero[1] |= previous.vy < 0.0 <= current.vy
        crossed_zero[2] |= previous.yaw_rate > 0.0 >= current.yaw_rate
        previous = current
    assert all(crossed_zero)
    assert previous.vx < 0.0
    assert previous.vy > 0.0
    assert previous.yaw_rate < 0.0
    assert not shaper.stop_ready(
        time_s=1.0, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_brief_zero_input_does_not_arm_stand_transition() -> None:
    shaper = Go2CommandShaper(stop_dwell_s=0.2)
    shaper.current = Go2Command(vx=0.04)
    shaper.set_target(Go2Command())
    shaper.update(0.01)
    assert not shaper.stop_ready(
        time_s=1.01, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )

    shaper.set_target(Go2Command(vx=-0.4))
    shaper.update(0.01)
    assert not shaper.stop_ready(
        time_s=1.02, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_stopping_waits_for_dwell_and_four_foot_support() -> None:
    shaper = Go2CommandShaper(stop_dwell_s=0.2)
    assert not shaper.stop_ready(
        time_s=1.0, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )
    assert not shaper.stop_ready(
        time_s=1.21, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )
    support_time = 3.5 / 2.2
    assert trot_all_stance(support_time, gait_hz=2.2, duty=0.6)
    assert shaper.stop_ready(
        time_s=support_time, threshold=0.05, gait_hz=2.2, gait_duty=0.6
    )


def test_joint_pose_transition_starts_continuously_and_reaches_exact_target() -> None:
    transition = JointPoseTransition(0.4)
    start = np.zeros(12)
    target = np.ones(12)
    transition.reset(start)
    transition.begin(start, target)

    samples = [transition.update(0.1)[0] for _ in range(4)]

    assert np.all(samples[0] > start)
    assert np.all(samples[0] < 0.2)
    assert np.all(np.diff(np.stack(samples), axis=0) > 0.0)
    np.testing.assert_array_equal(samples[-1], target)
    assert not transition.active


def test_new_pose_transition_can_interrupt_from_current_joint_pose() -> None:
    transition = JointPoseTransition(0.4)
    transition.reset(np.zeros(12))
    transition.begin(np.zeros(12), np.ones(12))
    current, _ = transition.update(0.2)
    transition.begin(current, np.full(12, -1.0))

    restarted, done = transition.update(0.01)

    assert not done
    assert np.max(np.abs(restarted - current)) < 0.02


def test_idle_to_motion_interpolates_stand_pose_before_ready_hold() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._metrics = None
    controller._dt = 0.1
    controller._sim_time = 0.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._command_shaper = Go2CommandShaper(stop_dwell_s=0.2)
    controller._config = SimpleNamespace(
        command_idle_threshold=0.05,
        gait_hz=2.5,
        gait_duty=0.6,
        pose_transition_s=0.4,
        ready_pose_s=0.12,
        command_ramp_s=0.4,
        ready_kp=120.0,
        ready_kv=6.0,
    )
    controller._active = False
    controller._ready_until = 0.0
    controller._walk_started_s = None
    controller._torque_mode_active = False
    controller._step_i = 0
    controller._forces = np.zeros((4, 3))
    controller._tau_hold = np.zeros(12)
    controller._last_contacts = np.ones(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12), ready_q=np.ones(12))
    controller._pose_transition = JointPoseTransition(0.4)
    controller._pose_transition.reset(controller._kin.stand_q)
    controller._pose_stage = "stand"
    controller._leg_dof_idxs = list(range(12))
    entity_state = {"q": np.zeros(12)}
    commanded_poses = []

    def control_position(pose, **_kwargs):
        entity_state["q"] = np.asarray(pose, dtype=float).copy()
        commanded_poses.append(entity_state["q"])

    controller._entity = SimpleNamespace(
        get_dofs_position=lambda **_kwargs: entity_state["q"],
        control_dofs_position=control_position,
    )
    controller._set_stand_actuation = lambda: None
    controller._set_ready_actuation = lambda: None
    controller._set_torque_actuation = lambda: None

    controller.step()
    controller.set_command(Go2Command(vx=0.45, vy=0.6, yaw_rate=0.8))
    for _ in range(4):
        controller.step()

    assert len(commanded_poses) == 5
    assert np.all(commanded_poses[1] > 0.0)
    assert np.all(commanded_poses[1] < 0.2)
    assert np.all(np.diff(np.stack(commanded_poses[1:]), axis=0) > 0.0)
    np.testing.assert_array_equal(commanded_poses[-1], controller._kin.ready_q)
    assert controller._pose_stage == "ready_hold"

    controller.step()
    controller.step()
    assert controller._pose_stage == "walk"
    assert controller._command_scale() == 0.0

    observed_command_scale = []
    def sample_with_command_ramp():
        observed_command_scale.append(controller._command_scale())
        return (
            None,
            np.zeros((4, 3)),
            np.zeros((4, 3)),
            np.zeros((4, 3, 3)),
            np.zeros(12),
        )

    controller._sample = sample_with_command_ramp
    controller._torques = lambda *_args: np.zeros(12)
    controller._entity.control_dofs_force = lambda *_args, **_kwargs: None
    controller.step()
    assert observed_command_scale[0] == pytest.approx(smoothstep_quintic(0.25))


def test_pympc_uses_a_smooth_startup_command_ramp() -> None:
    pympc = PyMpcGenesisController.__new__(PyMpcGenesisController)
    pympc._config = SimpleNamespace(command_ramp_s=0.4)
    pympc._sim_time = 1.2
    pympc._walk_started_s = 1.0
    assert pympc._command_scale() == pytest.approx(0.5)

    assert smoothstep_quintic(-0.1) == 0.0
    assert smoothstep_quintic(0.5) == pytest.approx(0.5)
    assert smoothstep_quintic(1.1) == 1.0


def test_pympc_records_contact_forces_and_preclip_torque():
    from unittest.mock import Mock

    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._metrics = None
    controller._contact_diagnostics = None
    controller._metrics = Mock()
    controller._contact_diagnostics = Mock(cadence_steps=5)
    controller._step_i = 5
    controller._dt = 0.02
    controller._sim_time = 1.2
    controller._config = SimpleNamespace(physical_friction=0.55)
    controller._force_requested = np.full((4, 3), 7.0)
    controller._forces = np.arange(12).reshape(4, 3)
    controller._tau_raw = np.full(12, 30.0)
    controller._tau_limited = np.full(12, 25.0)
    controller._tau_hold = np.full(12, 20.0)
    controller._record_contact_metrics(sample())
    request = controller._contact_diagnostics.sample.call_args.kwargs
    assert list(request["stance"].values()) == [True, False, False, True]
    assert request["elapsed_s"] == pytest.approx(0.1)
    recorded = controller._metrics.sample_contact.call_args.kwargs
    np.testing.assert_array_equal(recorded["raw_grf"], controller._force_requested)
    np.testing.assert_array_equal(recorded["tau_raw"], np.full(12, 30.0))
    np.testing.assert_array_equal(recorded["tau_limited"], np.full(12, 25.0))
    np.testing.assert_array_equal(recorded["tau_applied"], np.full(12, 20.0))
    controller._metrics.reset_mock()
    controller._contact_diagnostics.sample.return_value = None
    controller._record_contact_metrics(sample())
    controller._metrics.sample_contact.assert_not_called()


def test_pympc_emits_walking_rows_in_stand_and_torque_modes():
    from unittest.mock import Mock
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._step = Mock()
    controller._metrics = Mock()
    controller._entity = object()
    controller._cmd = Go2Command(vx=0.2)
    controller._command_source = "test"
    controller._arm_q = (0, 0, 0, 0)
    controller._tau_hold = np.ones(12)
    controller._sim_time = 1.0
    controller._rate_info = object()
    controller._bridge = SimpleNamespace(last_dq=np.arange(18.0))
    controller._faulted = False
    for active in (False, True):
        controller._torque_mode_active = active
        controller.step()
        row = controller._metrics.sample_go2.call_args.kwargs
        assert row["torque_update_flag"] == active
        assert row["go2_cmd"] == (0.2, 0.0, 0.0)
        assert (row["tau"] is not None) == active
    assert controller._metrics.sample_go2.call_count == 2


def test_contact_transition_forces_solve_between_regular_ticks():
    from unittest.mock import Mock
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    _init_torque_controller(c, elapsed_s=1.0)
    c._step_i = 1
    c._solve_stride = 2
    c._solver = SimpleNamespace(solve=Mock(return_value=np.tile([0, 0, 50.], (4, 1))))
    c._last_contacts = np.zeros(4)
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    tau = c._torques(state, state.feet_world, np.zeros((4, 3)),
                     np.tile(np.eye(3), (4, 1, 1)), np.zeros(12))
    c._solver.solve.assert_called_once()
    np.testing.assert_array_equal(tau.reshape(4, 3)[:, 2], np.full(4, -50.))


def test_gait_starts_at_same_support_phase_independent_of_uptime():
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    for start in (0.0, 7.13, 103.29):
        c._walk_started_s = start
        c._sim_time = start
        mask = contact_schedule(c._gait_time(), gait_hz=2.5, duty=0.6, dt=0.02, horizon=12)
        np.testing.assert_array_equal(mask[:, 0], np.ones(4))
        c._sim_time = start + 0.04
        assert c._gait_time() == pytest.approx(0.04)


def test_solver_failure_returns_to_stand_without_pose_step():
    from unittest.mock import Mock
    c = PyMpcGenesisController.__new__(PyMpcGenesisController)
    c._dt = 0.02
    c._sim_time = 10.0
    c._walk_started_s = 9.0
    c._faulted = False
    c._command_shaper = Go2CommandShaper()
    c._command_shaper.set_target(Go2Command(vx=0.35))
    c._config = SimpleNamespace(command_idle_threshold=0.01, gait_hz=2.5, gait_duty=0.6)
    c._pose_stage = "walk"
    c._torque_mode_active = True
    c._pose_transition = JointPoseTransition(0.4)
    c._kin = SimpleNamespace(stand_q=np.zeros(12))
    c._leg_dof_idxs = list(range(12))
    c._tau_hold = np.ones(12)
    c._force_requested = np.zeros((4, 3))
    c._forces = np.zeros((4, 3))
    c._tau_filt = np.zeros(12)
    c._tau_limited = np.zeros(12)
    c._tau_raw = np.zeros(12)
    c._entity = SimpleNamespace(get_dofs_position=lambda **kw: np.ones(12),
                                control_dofs_position=Mock())
    c._set_stand_actuation = Mock()
    c._sample = Mock(side_effect=RuntimeError("solver failure"))
    c._step()
    first = c._entity.control_dofs_position.call_args.args[0]
    assert np.all(first > 0.99)
    assert c._faulted
    for _ in range(25):
        c._step()
    np.testing.assert_allclose(c._entity.control_dofs_position.call_args.args[0], np.zeros(12))
    c._sample.assert_called_once()
