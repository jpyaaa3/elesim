"""Genesis GO2 torque controller using Quadruped-PyMPC's acados GRF solve.

This backend does not import the old convex gait, robot model or leg controller.
It deliberately remains opt-in until a real Genesis/GPU gait acceptance run.
"""

from __future__ import annotations

from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from elesim_sim.robot.go2.locomotion.kinematics import GO2_LEG_JOINTS, HIP_OFFSET_BODY, Go2KinematicsModel
from elesim_sim.robot.go2.locomotion.types import Go2Command, LegId
from elesim_sim.robot.go2.mpc.control_rate import ControlRateInfo
from elesim_sim.robot.go2.mpc.genesis_pin_bridge import GenesisPinBridge
from elesim_sim.robot.go2.mpc.payload_model import ArmPayloadCompensator
from elesim_sim.robot.go2.pympc_solver import PyMpcForceSolver, PyMpcInput
from elesim_sim.simulation.genesis.utils import to_numpy_1d


_LEGS = (LegId.FL, LegId.FR, LegId.RL, LegId.RR)
_PHASE_OFFSETS = np.array((0.0, 0.5, 0.5, 0.0))


def contact_schedule(time_s: float, *, gait_hz: float, duty: float, dt: float, horizon: int) -> np.ndarray:
    """FL/RR and FR/RL alternate; a column is one NMPC horizon step."""
    if gait_hz <= 0 or not 0 < duty < 1 or dt <= 0 or horizon < 2:
        raise ValueError("invalid trot contact schedule parameters")
    phase = (time_s + np.arange(horizon) * dt) * gait_hz
    return (((phase[None, :] + _PHASE_OFFSETS[:, None]) % 1.0) < duty).astype(float)


class PyMpcGenesisController:
    def __init__(
        self,
        entity,
        *,
        dt: float,
        config,
        arm_entity=None,
        arm_link_names: set[str] | None = None,
        metrics=None,
        command_source: str = "teleop",
        go2_urdf_path: str | Path | None = None,
        timing_sink=None,
        solver_factory=None,
    ) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise RuntimeError("Pinocchio is required by the PyMPC GO2 backend") from exc
        if go2_urdf_path is None:
            from elesim_sim.model_bundle import resolve_model_bundle

            go2_urdf_path = resolve_model_bundle() / "assets/go2/go2.urdf"
        urdf = Path(go2_urdf_path).expanduser().resolve()
        if not urdf.is_file():
            raise FileNotFoundError(f"GO2 PyMPC URDF not found: {urdf}")
        self._pin = pin
        self._model = pin.buildModelFromUrdf(str(urdf), pin.JointModelFreeFlyer())
        if self._model.nq != 19 or self._model.nv != 18:
            raise RuntimeError("GO2 PyMPC expects a floating base and twelve leg DOFs")
        if tuple(self._model.names[2:]) != GO2_LEG_JOINTS:
            raise RuntimeError("GO2 PyMPC Pinocchio joint order differs from Genesis")
        self._frames = tuple(self._model.getFrameId(f"{leg.value}_foot") for leg in _LEGS)
        if any(frame >= self._model.nframes for frame in self._frames):
            raise RuntimeError("GO2 PyMPC URDF is missing a foot frame")
        self._data = self._model.createData()
        self._entity = entity
        self._kin = Go2KinematicsModel.from_entity(entity)
        self._leg_dof_idxs = list(self._kin.all_leg_dof_idx)
        self._bridge = GenesisPinBridge(entity, self._leg_dof_idxs)
        self._payload = (
            ArmPayloadCompensator(
                arm_entity,
                mass_override_kg=float(config.payload_mass_kg),
                link_names=arm_link_names,
            )
            if arm_entity is not None and bool(config.payload_enable)
            else None
        )
        self._dt = float(dt)
        self._config = config
        self._metrics = metrics
        self._command_source = str(command_source)
        self._timing_sink = timing_sink
        self._rate_info = ControlRateInfo.from_sim_dt(self._dt, float(config.ctrl_hz))
        self._solve_stride = max(1, int(round(1.0 / (25.0 * self._dt))))
        self._solver = PyMpcForceSolver(
            horizon=12,
            dt=0.02,
            friction=float(config.optimization_friction),
            max_normal_force_n=float(config.fz_max_n),
            solver_factory=solver_factory,
        )
        self._tau_lim = self._read_torque_limits()
        self._cmd = Go2Command()
        self._arm_q = (0.0, 0.0, 0.0, 0.0)
        self._sim_time = 0.0
        self._active = False
        self._ready_until = 0.0
        self._faulted = False
        self._step_i = 0
        self._forces = np.zeros((4, 3))
        self._tau_hold = np.zeros(12)
        self._swing_starts = np.zeros((4, 3))
        self._touchdowns = np.zeros((4, 3))
        self._last_contacts = np.ones(4)
        self._set_stand_actuation()
        self._entity.set_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
        self._apply_physics_params()
        if metrics is not None:
            metrics.set_tau_limits(self._tau_lim)
        print("[go2_pympc] backend=acados nominal solve_hz=25 torque_hz="
              f"{self._rate_info.sim_hz:.1f} (experimental)")

    @property
    def control_rate_info(self) -> ControlRateInfo:
        return self._rate_info

    @property
    def tau_hold(self) -> np.ndarray:
        return self._tau_hold.copy()

    def _read_torque_limits(self) -> np.ndarray:
        lower, upper = self._entity.get_dofs_force_range(dofs_idx_local=self._leg_dof_idxs)
        lo, hi = to_numpy_1d(lower), to_numpy_1d(upper)
        if lo.shape != (12,) or hi.shape != (12,):
            raise RuntimeError("Genesis GO2 torque limits must have twelve values")
        limits = np.minimum(np.abs(lo), np.abs(hi)) * float(self._config.torque_safety_scale)
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0):
            raise RuntimeError("Genesis GO2 torque limits are invalid")
        return limits

    def _apply_physics_params(self) -> None:
        self._entity.set_friction(float(self._config.physical_friction))
        for stem, value in (
            ("armature", self._config.joint_armature),
            ("damping", self._config.joint_damping),
            ("frictionloss", self._config.joint_frictionloss),
        ):
            expected = np.full(12, float(value))
            getattr(self._entity, f"set_dofs_{stem}")(
                expected, dofs_idx_local=self._leg_dof_idxs
            )
            actual = to_numpy_1d(
                getattr(self._entity, f"get_dofs_{stem}")(
                    dofs_idx_local=self._leg_dof_idxs
                )
            )
            if actual.shape != (12,) or not np.allclose(actual, expected, atol=1e-7):
                raise RuntimeError(f"Genesis GO2 {stem} differs from configured value")

    def _set_stand_actuation(self) -> None:
        self._entity.set_dofs_kp(
            np.full(12, float(self._config.stand_kp)), dofs_idx_local=self._leg_dof_idxs
        )
        self._entity.set_dofs_kv(
            np.full(12, float(self._config.stand_kv)), dofs_idx_local=self._leg_dof_idxs
        )

    def _set_torque_actuation(self) -> None:
        self._entity.set_dofs_kp(np.zeros(12), dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(
            np.full(12, float(self._config.leg_kv_damping)),
            dofs_idx_local=self._leg_dof_idxs,
        )

    def set_command(self, cmd: Go2Command) -> None:
        self._cmd = cmd

    def set_arm_q(self, arm_q: tuple[float, float, float, float]) -> None:
        self._arm_q = tuple(float(x) for x in arm_q)

    def reset(self) -> None:
        self._cmd = Go2Command()
        self._sim_time = 0.0
        self._active = False
        self._ready_until = 0.0
        self._faulted = False
        self._step_i = 0
        self._forces.fill(0.0)
        self._tau_hold.fill(0.0)
        self._last_contacts.fill(1.0)
        self._set_stand_actuation()
        self._entity.set_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
        self._entity.zero_all_dofs_velocity()

    def _sample(self) -> tuple[PyMpcInput, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pin = self._pin
        q, dq = self._bridge.read_pin_q_dq()
        self._bridge.last_q = q.copy()
        self._bridge.last_dq = dq.copy()
        pin.forwardKinematics(self._model, self._data, q, dq)
        pin.updateFramePlacements(self._model, self._data)
        pin.computeJointJacobians(self._model, self._data, q)
        pin.centerOfMass(self._model, self._data, q, dq)
        pin.ccrba(self._model, self._data, q, dq)
        com = np.asarray(self._data.com[0], dtype=float).copy()
        vel = np.asarray(self._data.vcom[0], dtype=float).copy()
        if self._payload is not None:
            state = SimpleNamespace(data=self._data, pos_com_world=com, vel_com_world=vel)
            self._payload.apply(state)
            com = np.asarray(state.pos_com_world, dtype=float).copy()
            vel = np.asarray(state.vel_com_world, dtype=float).copy()
        rot = Rotation.from_quat(q[3:7])
        rot_m = rot.as_matrix()
        rpy = rot.as_euler("xyz")
        feet = np.array([self._data.oMf[frame].translation for frame in self._frames])
        foot_vel = np.array([
            pin.getFrameVelocity(self._model, self._data, frame, pin.LOCAL_WORLD_ALIGNED).linear
            for frame in self._frames
        ])
        jacobians = np.array([
            pin.getFrameJacobian(self._model, self._data, frame, pin.LOCAL_WORLD_ALIGNED)[:3, 6 + i * 3 : 9 + i * 3]
            for i, frame in enumerate(self._frames)
        ])
        mass = float(self._data.Ig.mass)
        inertia_body = rot_m.T @ np.asarray(self._data.Ig.inertia, dtype=float) @ rot_m
        cmd_body = np.array((float(self._cmd.vx), float(self._cmd.vy), 0.0))
        # WASD requests a horizontal body-frame velocity; body pitch/roll must
        # not turn a forward command into a requested vertical velocity.
        yaw_rot = Rotation.from_euler("z", float(rpy[2])).as_matrix()
        cmd_world = yaw_rot @ cmd_body
        horizon_t = 0.5 * self._solver.horizon * self._solver.dt
        ref_com = com + horizon_t * cmd_world
        ref_com[2] = float(self._config.z_pos_des_m)
        ref_rpy = rpy.copy()
        ref_rpy[0:2] = 0.0
        ref_rpy[2] += horizon_t * float(self._cmd.yaw_rate)
        contacts = contact_schedule(
            self._sim_time,
            gait_hz=float(self._config.gait_hz),
            duty=float(self._config.gait_duty),
            dt=self._solver.dt,
            horizon=self._solver.horizon,
        )
        footholds = feet.copy()
        floor_z = 0.025
        half_stance_s = 0.5 * float(self._config.gait_duty / self._config.gait_hz)
        yaw_lead = Rotation.from_euler(
            "z", half_stance_s * float(self._cmd.yaw_rate)
        ).as_matrix()
        for i, leg in enumerate(_LEGS):
            hip = np.asarray(HIP_OFFSET_BODY[leg], dtype=float)
            placement = yaw_lead @ hip + half_stance_s * cmd_body
            placement[:2] *= float(self._config.foot_placement_scale)
            if contacts[i, 0] == 0:
                footholds[i] = q[:3] + rot_m @ placement
                footholds[i, 2] = floor_z
        sample = PyMpcInput(
            com_position=com,
            com_velocity=vel,
            rpy=rpy,
            angular_velocity_body=dq[3:6],
            feet_world=feet,
            desired_com_position=ref_com,
            desired_com_velocity=cmd_world,
            desired_rpy=ref_rpy,
            desired_angular_velocity_body=np.array((0.0, 0.0, float(self._cmd.yaw_rate))),
            footholds_world=footholds,
            contacts=contacts,
            mass_kg=mass,
            inertia_body=inertia_body,
        )
        return sample, feet, foot_vel, jacobians, dq[6:18]

    def _torques(self, sample: PyMpcInput, feet: np.ndarray, foot_vel: np.ndarray,
                 jacobians: np.ndarray, joint_vel: np.ndarray) -> np.ndarray:
        if self._step_i % self._solve_stride == 0:
            started = time.perf_counter()
            self._forces = self._solver.solve(sample)
            if self._timing_sink is not None:
                self._timing_sink("go2_pympc_solve", time.perf_counter() - started)
        contacts = sample.contacts[:, 0]
        tau = np.zeros(12)
        phase = (self._sim_time * float(self._config.gait_hz) + _PHASE_OFFSETS) % 1.0
        duty = float(self._config.gait_duty)
        for i in range(4):
            sl = slice(i * 3, i * 3 + 3)
            if contacts[i] == 1:
                tau[sl] = -jacobians[i].T @ self._forces[i]
            else:
                if self._last_contacts[i] == 1:
                    self._swing_starts[i] = feet[i]
                    self._touchdowns[i] = sample.footholds_world[i]
                progress = np.clip((phase[i] - duty) / (1.0 - duty), 0.0, 1.0)
                smooth = progress * progress * (3.0 - 2.0 * progress)
                target = self._swing_starts[i] * (1.0 - smooth) + self._touchdowns[i] * smooth
                target[2] += 0.06 * np.sin(np.pi * progress)
                task_force = 180.0 * (target - feet[i]) - 7.0 * foot_vel[i]
                tau[sl] = jacobians[i].T @ task_force
            tau[sl] -= 0.5 * joint_vel[sl]
        self._last_contacts = contacts.copy()
        if not np.all(np.isfinite(tau)):
            raise RuntimeError("PyMPC produced nonfinite joint torque")
        return np.clip(tau, -self._tau_lim, self._tau_lim)

    def step(self) -> None:
        self._sim_time += self._dt
        if self._faulted or self._cmd.is_idle(self._config.command_idle_threshold):
            if self._active:
                self._active = False
                self._step_i = 0
                self._forces.fill(0.0)
                self._tau_hold.fill(0.0)
                self._last_contacts.fill(1.0)
                self._set_stand_actuation()
            self._entity.control_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
            return
        if not self._active:
            self._active = True
            self._ready_until = self._sim_time + float(self._config.ready_pose_s)
            self._entity.set_dofs_kp(np.full(12, float(self._config.ready_kp)), dofs_idx_local=self._leg_dof_idxs)
            self._entity.set_dofs_kv(np.full(12, float(self._config.ready_kv)), dofs_idx_local=self._leg_dof_idxs)
        if self._sim_time < self._ready_until:
            self._entity.control_dofs_position(self._kin.ready_q, dofs_idx_local=self._leg_dof_idxs)
            return
        if self._step_i == 0:
            self._set_torque_actuation()
        try:
            sample, feet, foot_vel, jacobians, joint_vel = self._sample()
            self._tau_hold = self._torques(sample, feet, foot_vel, jacobians, joint_vel)
        except Exception as exc:
            self._faulted = True
            self._active = False
            self._tau_hold.fill(0.0)
            self._set_stand_actuation()
            self._entity.control_dofs_position(self._kin.stand_q, dofs_idx_local=self._leg_dof_idxs)
            print(f"[go2_pympc] fault; latched safe stand until reset: {exc}")
            return
        self._entity.control_dofs_force(self._tau_hold, dofs_idx_local=self._leg_dof_idxs)
        self._step_i += 1
