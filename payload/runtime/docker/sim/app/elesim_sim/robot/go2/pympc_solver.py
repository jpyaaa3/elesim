"""Validated state/force boundary for CPU acados and optional GPU MPPI.

The upstream MuJoCo whole-body controller is deliberately not used here:
Genesis owns the plant, foot kinematics and torque actuation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np


LEGS = ("FL", "FR", "RL", "RR")
NOMINAL_HORIZON = 12
NOMINAL_DT = 0.02
NOMINAL_FRICTION = 0.55
NOMINAL_MAX_NORMAL_FORCE_N = 180.0
NOMINAL_SQP_ITERATIONS = 3
NOMINAL_HPIPM_MODE = "robust"


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
    simulation_time_s: float | None = None
    preview_footholds_world: np.ndarray | None = None
    support_loads: np.ndarray | None = None
    hip_offsets_com_body: np.ndarray | None = None
    support_force_floor_n: float = 0.0
    landing_search: np.ndarray | None = None
    actuation_scales: np.ndarray | None = None


def _finite_array(name: str, value: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"PyMPC {name} must be finite with shape {shape}, got {result.shape}")
    return result.copy()


def support_load_horizon(contacts: np.ndarray, current: np.ndarray, dt: float) -> np.ndarray:
    """Predict the force multiplier used by the contact-gated torque controller.

    Rows in ``contacts`` are legs; rows in the result are horizon stages. The
    current stage uses the measured gate load, then confirmed stance ramps by
    one controller step per 60 ms. A future planned landing is an estimate:
    actual confirmation can be later and will pause the gait clock.
    """
    contact = np.asarray(contacts, dtype=float)
    if contact.ndim != 2 or contact.shape[0] != 4 or contact.shape[1] < 1:
        raise ValueError("PyMPC contacts must have four legs and a nonempty horizon")
    if not np.all(np.isfinite(contact)) or np.any((contact != 0.) & (contact != 1.)):
        raise ValueError("PyMPC contact schedule must be binary")
    load = _finite_array("support_loads", current, (4,))
    if np.any((load < 0.) | (load > 1.)):
        raise ValueError("PyMPC support_loads must be within [0, 1]")
    if not np.isfinite(dt) or dt <= 0.:
        raise ValueError("PyMPC support load horizon dt must be positive")
    result = np.zeros((contact.shape[1], 4), dtype=float)
    result[0] = contact[:, 0] * load
    for stage in range(1, contact.shape[1]):
        result[stage] = np.where(contact[:, stage] == 1.,
                                 np.minimum(1., result[stage - 1] + dt / 0.06), 0.)
    return result


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
        backend: str = "acados",
        gpu_samples: int = 4096,
        gpu_iterations: int = 2,
        gpu_seed: int = 42,
    ) -> None:
        if horizon < 2 or dt <= 0 or friction <= 0 or max_normal_force_n <= 0:
            raise ValueError("invalid PyMPC horizon, step, friction or force cap")
        self.horizon = int(horizon)
        self.dt = float(dt)
        self.friction = float(friction)
        self.max_normal_force_n = float(max_normal_force_n)
        if backend not in {"acados", "jax_mppi"}:
            raise ValueError(f"unknown PyMPC backend: {backend}")
        self.backend = backend
        self._last_simulation_time: float | None = None
        if backend == "jax_mppi" and solver_factory is None:
            from .pympc_jax import JaxMppiSolver

            solver_factory = lambda: JaxMppiSolver(
                horizon=self.horizon, dt=self.dt, friction=self.friction,
                max_normal_force_n=self.max_normal_force_n,
                samples=gpu_samples, iterations=gpu_iterations, seed=gpu_seed,
            )
        if solver_factory is None:
            if (
                self.horizon,
                self.dt,
                self.friction,
                self.max_normal_force_n,
            ) != (
                NOMINAL_HORIZON,
                NOMINAL_DT,
                NOMINAL_FRICTION,
                NOMINAL_MAX_NORMAL_FORCE_N,
            ):
                raise RuntimeError(
                    "PyMPC image contains a fixed horizon=12 dt=0.02 mu=0.55 "
                    "fz=180 solver; rebuild it before changing these parameters"
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
                # Align all horizon foot guesses with the current contact
                # geometry before solving. More SQP passes were much slower
                # than this warm start in the real Genesis gait comparison.
                use_warm_start=True,
                num_qp_iterations=NOMINAL_SQP_ITERATIONS,
                solver_mode=NOMINAL_HPIPM_MODE,
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

                def reset(self) -> None:
                    """Clear Python and acados warm-start state after a scene reset."""
                    self.previous_status = -1
                    self.previous_contact_sequence = np.zeros((4, self.horizon))
                    self.optimal_next_state = np.zeros(24)
                    self.previous_optimal_GRF = np.zeros(12)
                    self.integral_errors = np.zeros(6)
                    self.initial_base_position = np.zeros(3)
                    self.acados_ocp_solver.reset(reset_qp_solver_mem=True)
                    for stage in range(self.horizon + 1):
                        self.acados_ocp_solver.set(stage, "x", np.zeros(self.states_dim))
                    for stage in range(self.horizon):
                        self.acados_ocp_solver.set(stage, "u", np.zeros(self.inputs_dim))
                    if self.use_RTI:
                        self.acados_ocp_solver.options_set("rti_phase", 1)
                        status = self.acados_ocp_solver.solve()
                        if status != 0:
                            raise RuntimeError(f"PyMPC RTI reset preparation failed: {status}")

            solver_factory = PrebuiltNominal
        self._solver = solver_factory()
        self._maxiter_warning_logged = False

    def reset(self) -> None:
        """Reset a solver implementation's warm start when it supports it."""
        reset = getattr(self._solver, "reset", None)
        if callable(reset):
            reset()
        self._maxiter_warning_logged = False
        self._last_simulation_time = None

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
        preview = (_finite_array("preview_footholds_world", sample.preview_footholds_world, (4, 3))
                   if sample.preview_footholds_world is not None else footholds)
        contacts = _finite_array("contacts", sample.contacts, (4, self.horizon))
        if np.any((contacts != 0.0) & (contacts != 1.0)):
            raise ValueError("PyMPC contact schedule must be binary")
        inertia = _finite_array("inertia_body", sample.inertia_body, (3, 3))
        mass = float(sample.mass_kg)
        if (not np.isfinite(mass) or mass <= 0
                or not np.allclose(inertia, inertia.T, rtol=1e-6, atol=1e-9)
                or np.min(np.linalg.eigvalsh(inertia)) <= 0):
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
            if self.backend == "jax_mppi":
                reference[f"preview_foot_{leg}"] = preview[index : index + 1]
        parameters = dict(mass=mass, inertia=inertia.reshape(9))
        if self.backend == "jax_mppi":
            floor = float(sample.support_force_floor_n)
            if not np.isfinite(floor) or not 0. <= floor <= self.max_normal_force_n:
                raise ValueError("PyMPC support_force_floor_n must be finite within the normal force cap")
            reference["support_force_floor_n"] = floor
            if sample.landing_search is not None:
                search = _finite_array("landing_search", sample.landing_search, (4,))
                if np.any((search != 0.) & (search != 1.)):
                    raise ValueError("PyMPC landing_search must be binary")
                reference["landing_search"] = search
            if sample.hip_offsets_com_body is not None:
                reference["hip_offsets_com_body"] = _finite_array(
                    "hip_offsets_com_body", sample.hip_offsets_com_body, (4, 3))
            if sample.support_loads is not None:
                reference["support_loads"] = support_load_horizon(
                    contacts, sample.support_loads, self.dt)
            if sample.actuation_scales is not None:
                scales = _finite_array('actuation_scales', sample.actuation_scales, (self.horizon,))
                if np.any((scales < 0.) | (scales > 1.)):
                    raise ValueError('PyMPC actuation_scales must be within [0, 1]')
                loads = reference.get('support_loads', contacts.T)
                reference['support_loads'] = loads*scales[:, None]
            now = sample.simulation_time_s
            if now is not None and (not np.isfinite(now) or now < 0):
                raise ValueError("PyMPC simulation time must be finite and nonnegative")
            if (now is not None and self._last_simulation_time is not None
                    and now < self._last_simulation_time):
                raise ValueError("PyMPC simulation time went backwards without reset")
            parameters["elapsed_s"] = (now - self._last_simulation_time
                if now is not None and self._last_simulation_time is not None else 0.0)
        force, _, _, status = self._solver.compute_control(state, reference, contacts, **parameters)
        if self.backend == "jax_mppi":
            self._last_simulation_time = sample.simulation_time_s
        status_code = int(status)
        # A finite iterate remains useful when SQP reaches its iteration cap;
        # validate and bound it below. Other acados failures do not produce a
        # control input we are willing to apply.
        accepted_statuses = (0, 2) if self.backend == "acados" else (0,)
        if status_code not in accepted_statuses:
            raise RuntimeError(
                f"PyMPC {self.backend} solve failed with status {status_code}"
            )
        grf = _finite_array("GRF", force, (12,)).reshape(4, 3)
        for index in range(4):
            if contacts[index, 0] == 0:
                grf[index] = 0.0
                continue
            fz = float(np.clip(grf[index, 2], 0.0, self.max_normal_force_n))
            grf[index, 2] = fz
            grf[index, :2] = np.clip(grf[index, :2], -self.friction * fz, self.friction * fz)
        if status_code == 2 and not self._maxiter_warning_logged:
            print(
                "[go2_pympc] acados reached its iteration limit; "
                "using the finite, bounded force iterate"
            )
            self._maxiter_warning_logged = True
        return grf
