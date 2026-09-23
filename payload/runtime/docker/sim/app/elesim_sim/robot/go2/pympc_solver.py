"""Narrow, fail-closed adapter for Quadruped-PyMPC's nominal acados solver.

The upstream MuJoCo whole-body controller is deliberately not used here:
Genesis owns the plant, foot kinematics and torque actuation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np


LEGS = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class PyMpcInput:
    com_position: np.ndarray
    com_velocity: np.ndarray
    rpy: np.ndarray
    angular_velocity_body: np.ndarray
    feet_world: np.ndarray
    desired_com_position: np.ndarray
    desired_com_velocity: np.ndarray
    desired_rpy: np.ndarray
    desired_angular_velocity_body: np.ndarray
    footholds_world: np.ndarray
    contacts: np.ndarray
    mass_kg: float
    inertia_body: np.ndarray


def _finite_array(name: str, value: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"PyMPC {name} must be finite with shape {shape}, got {result.shape}")
    return result.copy()


class PyMpcForceSolver:
    """Translate Genesis/Pinocchio state into PyMPC's world-frame contract."""

    def __init__(
        self,
        *,
        horizon: int = 12,
        dt: float = 0.02,
        friction: float = 0.55,
        max_normal_force_n: float = 180.0,
        solver_factory: Callable[[], object] | None = None,
    ) -> None:
        if horizon < 2 or dt <= 0 or friction <= 0 or max_normal_force_n <= 0:
            raise ValueError("invalid PyMPC horizon, step, friction or force cap")
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.friction = float(friction)
        self.max_normal_force_n = float(max_normal_force_n)
        if solver_factory is None:
            if (self.horizon, self.dt, self.friction, self.max_normal_force_n) != (
                12, 0.02, 0.55, 180.0
            ):
                raise RuntimeError(
                    "PyMPC image contains a fixed horizon=12 dt=0.02 mu=0.55 fz=180 "
                    "solver; rebuild it before changing these parameters"
                )
            try:
                import quadruped_pympc.config as upstream_config
                import quadruped_pympc.controllers.gradient.nominal.centroidal_nmpc_nominal as nominal
                from acados_template import AcadosOcpSolver
            except ImportError as exc:
                raise RuntimeError(
                    "Quadruped-PyMPC/acados is not installed in the Sim image"
                ) from exc
            upstream_config.mpc_params.update(
                horizon=self.horizon,
                dt=self.dt,
                mu=self.friction,
                grf_max=self.max_normal_force_n,
                grf_min=0.0,
                use_foothold_optimization=False,
                use_foothold_constraints=False,
            )
            generated = Path(nominal.__file__).parent / "c_generated_code"
            if not (generated / "centroidal_nmpc.json").is_file():
                raise RuntimeError(
                    "PyMPC acados solver was not generated in the image; "
                    "rebuild the development/Sim image with PyMPC enabled"
                )

            class PrebuiltNominal(nominal.Acados_NMPC_Nominal):
                """Load build-time generated code without runtime writes/compiles."""

                def __init__(self) -> None:
                    cfg = upstream_config.mpc_params
                    self.horizon = cfg["horizon"]
                    self.dt = cfg["dt"]
                    self.T_horizon = self.horizon * self.dt
                    self.use_RTI = cfg["use_RTI"]
                    self.use_integrators = cfg["use_integrators"]
                    self.use_warm_start = cfg["use_warm_start"]
                    self.use_foothold_constraints = cfg["use_foothold_constraints"]
                    self.use_static_stability = cfg["use_static_stability"]
                    self.use_zmp_stability = cfg["use_zmp_stability"]
                    self.use_stability_constraints = (
                        self.use_static_stability or self.use_zmp_stability
                    )
                    self.use_DDP = cfg["use_DDP"]
                    self.verbose = cfg["verbose"]
                    self.previous_status = -1
                    self.previous_contact_sequence = np.zeros((4, self.horizon))
                    self.optimal_next_state = np.zeros(24)
                    self.previous_optimal_GRF = np.zeros(12)
                    self.integral_errors = np.zeros(6)
                    self.initial_base_position = np.zeros(3)
                    self.centroidal_model = nominal.Centroidal_Model_Nominal()
                    model = self.centroidal_model.export_robot_model()
                    self.states_dim = model.x.size()[0]
                    self.inputs_dim = model.u.size()[0]
                    self.ocp = self.create_ocp_solver_description(model)
                    self.ocp.code_export_directory = str(generated)
                    self.acados_ocp_solver = AcadosOcpSolver(
                        self.ocp,
                        json_file=str(generated / "centroidal_nmpc.json"),
                        generate=False,
                        build=False,
                    )
                    for stage in range(self.horizon + 1):
                        self.acados_ocp_solver.set(stage, "x", np.zeros(self.states_dim))
                    for stage in range(self.horizon):
                        self.acados_ocp_solver.set(stage, "u", np.zeros(self.inputs_dim))
                    if self.use_RTI:
                        self.acados_ocp_solver.options_set("rti_phase", 1)
                        status = self.acados_ocp_solver.solve()
                        if status != 0:
                            raise RuntimeError(f"PyMPC RTI preparation failed: {status}")

            solver_factory = PrebuiltNominal
        self._solver = solver_factory()

    def solve(self, sample: PyMpcInput) -> np.ndarray:
        pos = _finite_array("com_position", sample.com_position, (3,))
        vel = _finite_array("com_velocity", sample.com_velocity, (3,))
        rpy = _finite_array("rpy", sample.rpy, (3,))
        omega = _finite_array("angular_velocity_body", sample.angular_velocity_body, (3,))
        feet = _finite_array("feet_world", sample.feet_world, (4, 3))
        ref_pos = _finite_array("desired_com_position", sample.desired_com_position, (3,))
        ref_vel = _finite_array("desired_com_velocity", sample.desired_com_velocity, (3,))
        ref_rpy = _finite_array("desired_rpy", sample.desired_rpy, (3,))
        ref_omega = _finite_array(
            "desired_angular_velocity_body", sample.desired_angular_velocity_body, (3,)
        )
        footholds = _finite_array("footholds_world", sample.footholds_world, (4, 3))
        contacts = _finite_array("contacts", sample.contacts, (4, self.horizon))
        if np.any((contacts != 0.0) & (contacts != 1.0)):
            raise ValueError("PyMPC contact schedule must be binary")
        inertia = _finite_array("inertia_body", sample.inertia_body, (3, 3))
        mass = float(sample.mass_kg)
        if not np.isfinite(mass) or mass <= 0 or np.min(np.linalg.eigvalsh(inertia)) <= 0:
            raise ValueError("PyMPC mass and body inertia must be positive")

        state = dict(position=pos, linear_velocity=vel, orientation=rpy, angular_velocity=omega)
        reference = dict(
            ref_position=ref_pos,
            ref_linear_velocity=ref_vel,
            ref_orientation=ref_rpy,
            ref_angular_velocity=ref_omega,
        )
        for index, leg in enumerate(LEGS):
            state[f"foot_{leg}"] = feet[index]
            reference[f"ref_foot_{leg}"] = footholds[index : index + 1]
        force, _, _, status = self._solver.compute_control(
            state, reference, contacts, mass=mass, inertia=inertia.reshape(9)
        )
        if int(status) != 0:
            raise RuntimeError(f"PyMPC acados solve failed with status {status}")
        grf = _finite_array("GRF", force, (12,)).reshape(4, 3)
        for index in range(4):
            if contacts[index, 0] == 0:
                grf[index] = 0.0
                continue
            fz = float(np.clip(grf[index, 2], 0.0, self.max_normal_force_n))
            grf[index, 2] = fz
            grf[index, :2] = np.clip(grf[index, :2], -self.friction * fz, self.friction * fz)
        return grf
