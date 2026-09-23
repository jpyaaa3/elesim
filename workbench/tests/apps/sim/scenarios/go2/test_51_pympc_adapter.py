from __future__ import annotations

import numpy as np
import pytest
from types import SimpleNamespace

from elesim_sim.robot.go2.locomotion.types import Go2Command
from elesim_sim.robot.go2.pympc_controller import PyMpcGenesisController, contact_schedule
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


def test_diagonal_contact_sequence() -> None:
    schedule = contact_schedule(0.0, gait_hz=2.5, duty=0.5, dt=0.1, horizon=4)
    assert schedule.shape == (4, 4)
    np.testing.assert_array_equal(schedule[0], schedule[3])
    np.testing.assert_array_equal(schedule[1], schedule[2])
    np.testing.assert_array_equal(schedule[0] + schedule[1], np.ones(4))


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
    controller._step_i = 0
    controller._solve_stride = 2
    controller._solver = SimpleNamespace(solve=lambda _sample: np.tile([0, 0, 50], (4, 1)))
    controller._timing_sink = None
    controller._config = SimpleNamespace(gait_hz=2.5, gait_duty=0.5)
    controller._sim_time = 0.0
    controller._last_contacts = np.ones(4)
    controller._tau_lim = np.full(12, 100.0)
    state = PyMpcInput(**{**vars(sample()), "contacts": np.ones((4, 3))})
    jacobians = np.tile(np.eye(3), (4, 1, 1))
    torques = controller._torques(
        state, state.feet_world, np.zeros((4, 3)), jacobians, np.zeros(12)
    )
    np.testing.assert_array_equal(torques.reshape(4, 3), np.tile([0, 0, -50], (4, 1)))


def test_idle_clears_previous_gait_force_and_rearms_torque_mode() -> None:
    controller = PyMpcGenesisController.__new__(PyMpcGenesisController)
    controller._dt = 0.02
    controller._sim_time = 1.0
    controller._faulted = False
    controller._cmd = Go2Command()
    controller._config = SimpleNamespace(command_idle_threshold=0.05)
    controller._active = True
    controller._step_i = 9
    controller._forces = np.ones((4, 3))
    controller._tau_hold = np.ones(12)
    controller._last_contacts = np.zeros(4)
    controller._kin = SimpleNamespace(stand_q=np.zeros(12))
    controller._leg_dof_idxs = list(range(12))
    calls = []
    controller._set_stand_actuation = lambda: calls.append("stand")
    controller._entity = SimpleNamespace(
        control_dofs_position=lambda *_args, **_kwargs: calls.append("position")
    )
    controller.step()
    assert calls == ["stand", "position"]
    assert controller._step_i == 0
    assert not controller._active
    assert not controller._forces.any()
    assert not controller._tau_hold.any()
    assert controller._last_contacts.all()
