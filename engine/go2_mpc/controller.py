from __future__ import annotations

import numpy as np

from engine.go2_locomotion.kinematics import Go2KinematicsModel
from engine.go2_locomotion.types import Go2Command
from engine.go2_mpc.config import Go2MpcConfig
from engine.go2_mpc.gait_adapter import ScaledGait
from engine.go2_mpc.genesis_pin_bridge import GenesisPinBridge, _to_numpy_1d
from engine.go2_mpc.payload_model import ArmPayloadCompensator, backward_pitch_trim_rad

LEG_NAMES = ("FL", "FR", "RL", "RR")
LEG_SLICE = {
    "FL": slice(0, 3),
    "FR": slice(3, 6),
    "RL": slice(6, 9),
    "RR": slice(9, 12),
}

HIP_LIM = 23.7
ABD_LIM = 23.7
KNEE_LIM = 45.43
_TAU_LIM = np.array(
    [
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
    ],
    dtype=float,
)


def _require_convex_mpc():
    try:
        from convex_mpc.centroidal_mpc import CentroidalMPC
        from convex_mpc.com_trajectory import ComTraj
        from convex_mpc.gait import Gait
        from convex_mpc.go2_robot_data import PinGo2Model
        from convex_mpc.leg_controller import LegController
    except ImportError as exc:
        raise ImportError(
            "convex_mpc is not installed. Run: pip install -e "
            "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git "
            "and conda install -c conda-forge pinocchio casadi"
        ) from exc
    return PinGo2Model, Gait, LegController, ComTraj, CentroidalMPC


class ConvexMpcGenesisController:
    """go2-convex-mpc stack on Genesis: Pinocchio dynamics + MPC + torque actuation."""

    def __init__(self, entity, *, dt: float, config: Go2MpcConfig, arm_entity=None) -> None:
        PinGo2Model, Gait, LegController, ComTraj, CentroidalMPC = _require_convex_mpc()

        self._entity = entity
        self._dt = float(dt)
        self._config = config
        self._cmd = Go2Command()
        self._sim_time = 0.0
        self._ctrl_i = 0
        self._sim_step_i = 0
        self._loco_time = 0.0
        self._torque_mode = False
        self._ready_mode = False
        self._ready_until_t = 0.0

        self._kin = Go2KinematicsModel.from_entity(entity)
        self._leg_dof_idxs = list(self._kin.all_leg_dof_idx)
        payload = None
        if arm_entity is not None and bool(config.payload_enable):
            from engine.go2_mpc.payload_model import ArmPayloadCompensator

            payload = ArmPayloadCompensator(
                arm_entity,
                mass_override_kg=float(config.payload_mass_kg),
            )
        self._bridge = GenesisPinBridge(entity, self._leg_dof_idxs, payload=payload)
        self._payload = payload

        self._pin = PinGo2Model()
        self._gait = ScaledGait(
            float(config.gait_hz),
            float(config.gait_duty),
            placement_scale=float(config.foot_placement_scale),
        )
        self._leg_controller = LegController()
        self._traj = ComTraj(self._pin)
        self._mpc: CentroidalMPC | None = None
        self._U_opt = np.zeros((12, 1), dtype=float)
        self._force_filt = np.zeros(12, dtype=float)
        self._tau_filt = np.zeros(12, dtype=float)
        self._tau_hold = np.zeros(12, dtype=float)
        self._z_des_m = float(config.z_pos_des_m)

        ctrl_hz = max(1.0, float(config.ctrl_hz))
        self._ctrl_decim = max(1, int(round((1.0 / self._dt) / ctrl_hz)))
        mpc_hz = 1.0 / float(config.mpc_dt_s)
        self._steps_per_mpc = max(1, int(round(ctrl_hz / mpc_hz)))
        self._ctrl_dt = float(self._ctrl_decim * self._dt)
        self._tau_lim = _TAU_LIM * float(config.torque_safety_scale)

        self._init_pose_and_actuation()
        self._bridge.sync_pin_model(self._pin)
        if payload is not None:
            com_body = payload.measure_com_body(entity)
            snap = payload.measure()
            if com_body is not None and snap is not None:
                print(
                    "[go2_mpc] arm payload: "
                    f"m={snap.mass_kg:.2f}kg "
                    f"com_body=({com_body[0]:.3f},{com_body[1]:.3f},{com_body[2]:.3f}) "
                    f"total_m={float(self._pin.data.Ig.mass):.2f}kg"
                )
        self._z_des_m = float(self._pin.pos_com_world[2])
        self._traj.generate_traj(
            self._pin,
            self._gait,
            0.0,
            0.0,
            0.0,
            self._z_des_m,
            0.0,
            time_step=float(self._config.mpc_dt_s),
        )
        self._mpc = CentroidalMPC(self._pin, self._traj)

    def _init_pose_and_actuation(self) -> None:
        self._apply_go2_physics_params()
        stand_q = np.asarray(self._kin.stand_q, dtype=float)
        self._entity.set_dofs_position(stand_q, dofs_idx_local=self._leg_dof_idxs)
        n = len(self._leg_dof_idxs)
        kp = np.full(n, float(self._config.stand_kp), dtype=float)
        kv = np.full(n, float(self._config.stand_kv), dtype=float)
        self._entity.set_dofs_kp(kp, dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(kv, dofs_idx_local=self._leg_dof_idxs)

    def _apply_go2_physics_params(self) -> None:
        n = len(self._leg_dof_idxs)
        try:
            self._entity.set_friction(0.8)
        except Exception:
            pass
        try:
            self._entity.set_dofs_armature(
                np.full(n, 0.01, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
            self._entity.set_dofs_damping(
                np.full(n, 0.1, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
            self._entity.set_dofs_frictionloss(
                np.full(n, 0.2, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
        except Exception:
            pass

    def _set_torque_actuation(self) -> None:
        n = len(self._leg_dof_idxs)
        self._entity.set_dofs_kp(np.zeros(n, dtype=float), dofs_idx_local=self._leg_dof_idxs)
        kv = np.full(n, float(self._config.leg_kv_damping), dtype=float)
        self._entity.set_dofs_kv(kv, dofs_idx_local=self._leg_dof_idxs)

    def _set_stand_actuation(self) -> None:
        n = len(self._leg_dof_idxs)
        kp = np.full(n, float(self._config.stand_kp), dtype=float)
        kv = np.full(n, float(self._config.stand_kv), dtype=float)
        self._entity.set_dofs_kp(kp, dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(kv, dofs_idx_local=self._leg_dof_idxs)

    def _set_ready_actuation(self) -> None:
        n = len(self._leg_dof_idxs)
        kp = np.full(n, float(self._config.ready_kp), dtype=float)
        kv = np.full(n, float(self._config.ready_kv), dtype=float)
        self._entity.set_dofs_kp(kp, dofs_idx_local=self._leg_dof_idxs)
        self._entity.set_dofs_kv(kv, dofs_idx_local=self._leg_dof_idxs)

    def _begin_ready_pose(self) -> None:
        self._ready_mode = True
        self._ready_until_t = float(self._sim_time) + float(self._config.ready_pose_s)
        self._set_ready_actuation()

    def _enter_torque_mode(self) -> None:
        _, _, LegController, _, _ = _require_convex_mpc()
        self._ready_mode = False
        self._torque_mode = True
        self._leg_controller = LegController()
        self._ctrl_i = 0
        self._loco_time = 0.0
        self._tau_hold = np.zeros(12, dtype=float)
        self._force_filt = np.zeros(12, dtype=float)
        self._tau_filt = np.zeros(12, dtype=float)
        self._set_torque_actuation()
        self._bridge.sync_pin_model(self._pin)
        self._z_des_m = max(float(self._config.z_pos_des_m), float(self._pin.pos_com_world[2]) - 0.01)

    def _apply_backward_pitch_trim(self, vx: float) -> None:
        if vx >= -0.05 or self._payload is None:
            return
        com_body = self._payload.measure_com_body(self._entity)
        if com_body is None:
            return
        pitch_trim = backward_pitch_trim_rad(
            com_body,
            gain_z=float(self._config.pitch_trim_gain_z),
            z_ref_m=float(self._config.pitch_trim_z_ref_m),
            max_trim_rad=float(self._config.pitch_trim_max_rad),
        )
        if abs(pitch_trim) < 1e-6:
            return
        self._traj.rpy_traj_world[1, :] = pitch_trim
        self._traj._continuousDynamics(self._pin)
        self._traj._discreteDynamics(float(self._config.mpc_dt_s))

    def _solve_mpc(self, vx: float, vy: float, z_des: float, wz: float) -> None:
        assert self._mpc is not None
        self._traj.generate_traj(
            self._pin,
            self._gait,
            float(self._sim_time),
            float(vx),
            float(vy),
            float(z_des),
            float(wz),
            time_step=float(self._config.mpc_dt_s),
        )
        self._apply_backward_pitch_trim(vx)
        try:
            sol = self._mpc.solve_QP(self._pin, self._traj, False)
            w_opt = sol["x"].full().flatten()
            n = int(self._traj.N)
            force_new = w_opt[12 * n : 12 * n + 12]
            alpha = float(np.clip(self._config.force_filter_alpha, 0.05, 1.0))
            self._force_filt = alpha * force_new + (1.0 - alpha) * self._force_filt
            self._U_opt[:, 0] = self._force_filt
        except Exception:
            pass

    def _command_scale(self) -> float:
        ramp_s = max(1e-3, float(self._config.command_ramp_s))
        return float(min(1.0, self._loco_time / ramp_s))

    def _torque_scale(self) -> float:
        warmup_s = max(0.0, float(self._config.torque_warmup_s))
        ramp_s = max(1e-3, float(self._config.torque_ramp_s))
        if self._loco_time < warmup_s:
            return 0.35
        t = self._loco_time - warmup_s
        return float(min(1.0, 0.35 + 0.65 * (t / ramp_s)))

    def _aux_kp_scale(self) -> float:
        warmup_s = max(0.0, float(self._config.torque_warmup_s))
        ramp_s = max(1e-3, float(self._config.torque_ramp_s))
        t = self._loco_time - warmup_s
        if t <= 0.0:
            return 0.0
        if t >= ramp_s:
            return 0.0
        return float(1.0 - t / ramp_s)

    def _aux_pd_torque(self) -> np.ndarray:
        q = _to_numpy_1d(self._entity.get_dofs_position(dofs_idx_local=self._leg_dof_idxs))
        dq = _to_numpy_1d(self._entity.get_dofs_velocity(dofs_idx_local=self._leg_dof_idxs))
        kv = float(self._config.aux_kv)
        tau = -kv * dq
        kp_scale = self._aux_kp_scale()
        if kp_scale > 1e-6:
            ready_q = np.asarray(self._kin.ready_q, dtype=float)
            tau += float(self._config.aux_kp) * kp_scale * (ready_q - q)
        return tau

    def _filter_tau(self, tau_cmd: np.ndarray) -> np.ndarray:
        alpha = float(np.clip(self._config.tau_filter_alpha, 0.05, 1.0))
        self._tau_filt = alpha * tau_cmd + (1.0 - alpha) * self._tau_filt
        return self._tau_filt.copy()

    def _compute_tau_cmd(self, vx: float, vy: float, wz: float) -> np.ndarray:
        if self._payload is not None:
            self._z_des_m = max(float(self._config.z_pos_des_m), float(self._pin.pos_com_world[2]) - 0.01)
        if self._ctrl_i % self._steps_per_mpc == 0:
            self._solve_mpc(vx, vy, self._z_des_m, wz)

        force = self._U_opt[:, 0]
        tau_cmd = np.zeros(12, dtype=float)
        for leg in LEG_NAMES:
            out = self._leg_controller.compute_leg_torque(
                leg,
                self._pin,
                self._gait,
                force[LEG_SLICE[leg]],
                float(self._sim_time),
            )
            tau_cmd[LEG_SLICE[leg]] = np.asarray(out.tau, dtype=float).reshape(3)

        tau_cmd = np.clip(tau_cmd, -self._tau_lim, self._tau_lim)
        tau_cmd = tau_cmd * self._torque_scale() + self._aux_pd_torque()
        tau_cmd = np.clip(tau_cmd, -self._tau_lim, self._tau_lim)
        return self._filter_tau(tau_cmd)

    def set_command(self, cmd: Go2Command) -> None:
        self._cmd = cmd

    def step(self) -> None:
        self._sim_time += self._dt

        if self._cmd.is_idle(self._config.command_idle_threshold):
            if self._torque_mode or self._ready_mode:
                self._torque_mode = False
                self._ready_mode = False
                self._set_stand_actuation()
            self._sim_step_i = 0
            self._entity.control_dofs_position(
                np.asarray(self._kin.stand_q, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
            return

        if self._ready_mode:
            self._entity.control_dofs_position(
                np.asarray(self._kin.ready_q, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
            if self._sim_time >= self._ready_until_t:
                self._enter_torque_mode()
            return

        if not self._torque_mode:
            self._begin_ready_pose()
            self._entity.control_dofs_position(
                np.asarray(self._kin.ready_q, dtype=float),
                dofs_idx_local=self._leg_dof_idxs,
            )
            return

        if self._sim_step_i % self._ctrl_decim != 0:
            self._entity.control_dofs_force(
                self._tau_hold,
                dofs_idx_local=self._leg_dof_idxs,
            )
            self._sim_step_i += 1
            return

        assert self._mpc is not None
        self._bridge.sync_pin_model(self._pin)
        self._loco_time += self._ctrl_dt

        cmd_scale = self._command_scale()
        vx = float(self._cmd.vx) * cmd_scale
        vy = float(self._cmd.vy) * cmd_scale
        wz = float(self._cmd.yaw_rate) * cmd_scale

        self._tau_hold = self._compute_tau_cmd(vx, vy, wz)
        self._entity.control_dofs_force(
            self._tau_hold,
            dofs_idx_local=self._leg_dof_idxs,
        )
        self._ctrl_i += 1
        self._sim_step_i += 1
