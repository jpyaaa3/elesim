"""Local image Jacobian visual servo for grasp approach (q-space, 3D UVZ)."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Optional, Sequence, Tuple

import numpy as np

from engine.vision.visual_servoing.uv_jacobian import default_uv_jacobian


Q_AXIS_NAMES = ("linear", "roll", "theta1", "theta2")
NUM_Q = 4
NUM_FEATURES = 3


class GraspApproachMode(str, Enum):
    LOCAL_IMG_JACOBIAN = "local_img_jacobian"
    REACQUIRE = "reacquire"
    FAILED = "failed"


class SampleRejectReason(str, Enum):
    ACCEPTED = "accepted"
    DQ_TOO_SMALL = "dq_too_small"
    OBJECT_LOST = "object_lost"
    SETTLE_TIMEOUT = "settle_timeout"
    JOINT_SATURATED = "joint_saturated"
    MOTION_MISMATCH = "motion_mismatch"


def damped_pseudoinverse_mn(jacobian: np.ndarray, damping: float) -> np.ndarray:
    """J+ for J shape (m, n): J+ = J^T (J J^T + lambda^2 I)^-1."""
    j = np.asarray(jacobian, dtype=float)
    if j.ndim != 2:
        raise ValueError(f"jacobian must be 2D, got {j.shape}")
    m = int(j.shape[0])
    lam = float(max(damping, 1e-9))
    jj_t = j @ j.T
    inv = np.linalg.inv(jj_t + (lam * lam) * np.eye(m, dtype=float))
    return j.T @ inv


def estimate_j_img_from_stacks(
    q_stack: np.ndarray,
    s_stack: np.ndarray,
) -> Tuple[np.ndarray, int, float]:
    """
    delta_s ≈ J_img @ delta_q with Q: (N,4), S: (N,m).

    B = pinv(Q) @ S  -> (4, m);  J_img = B.T -> (m, 4).
    """
    q = np.asarray(q_stack, dtype=float)
    s = np.asarray(s_stack, dtype=float)
    if q.ndim != 2 or s.ndim != 2:
        raise ValueError("q_stack and s_stack must be 2D")
    if q.shape[0] != s.shape[0] or q.shape[0] < 1:
        raise ValueError(f"stack row mismatch: Q={q.shape}, S={s.shape}")
    m = int(s.shape[1])
    n = int(q.shape[1])
    b = np.linalg.pinv(q) @ s
    j_img = b.T.reshape(m, n)
    rank = int(np.linalg.matrix_rank(j_img))
    cond = float(np.linalg.cond(j_img)) if rank > 0 else float("inf")
    return j_img, rank, cond


def display_v_seg_coupling(
    j_row_v: Sequence[float],
    command_direction: Sequence[int],
) -> tuple[float, float]:
    """Display-space ∂v/∂s1 and ∂v/∂s2 from q-space Jacobian row."""
    dirs = tuple(int(v) for v in command_direction)
    if len(dirs) != NUM_Q:
        raise ValueError(f"command_direction must have length {NUM_Q}, got {len(dirs)}")
    row = np.asarray(j_row_v, dtype=float).reshape(NUM_Q)
    return float(dirs[2]) * float(row[2]), float(dirs[3]) * float(row[3])


def patch_lji_jacobian_for_control(
    j_lji: np.ndarray,
    *,
    z_row: Sequence[float],
    seed_j: np.ndarray,
    command_direction: Sequence[int],
    measured_v_row_blend: float = 0.0,
    measured_v_row_norm_max: float = 120.0,
) -> np.ndarray:
    """
    Control-time Jacobian:
      - z row from FK (∂remain/∂q)
      - full v row from seed, optionally blended with online measured row
      - u-row seg columns from seed; roll/linear may come from online estimate
    """
    j = np.asarray(j_lji, dtype=float).reshape(NUM_FEATURES, NUM_Q).copy()
    seed = np.asarray(seed_j, dtype=float).reshape(NUM_FEATURES, NUM_Q)
    measured = j.copy()
    j[2, :] = np.asarray(z_row, dtype=float).reshape(NUM_Q)
    blend = float(np.clip(measured_v_row_blend, 0.0, 1.0))
    if blend > 1e-9 and np.all(np.isfinite(measured[1, :])):
        v_row = measured[1, :].copy()
        norm = float(np.linalg.norm(v_row))
        max_norm = float(max(measured_v_row_norm_max, 1e-6))
        if norm > max_norm:
            v_row *= max_norm / norm
        j[1, :] = (1.0 - blend) * seed[1, :] + blend * v_row
    else:
        j[1, :] = seed[1, :]
    j[0, 2:4] = seed[0, 2:4]
    return j


def z_jacobian_row_from_position_jacobian(
    position_jacobian: np.ndarray,
    approach_dir: Sequence[float],
) -> np.ndarray:
    """
    ∂(remain)/∂q with remain = (nominal − tip)·approach_dir.

    Nominal is fixed per step, so ∂remain/∂q = −approach_dirᵀ (∂tip/∂q).
    """
    j_pos = np.asarray(position_jacobian, dtype=float).reshape(3, 4)
    d = np.asarray(approach_dir, dtype=float).reshape(3)
    norm = float(np.linalg.norm(d))
    if norm <= 1e-9:
        raise ValueError("approach_dir must be non-zero")
    d = d / norm
    return -(d @ j_pos)


def default_j_lji_seed(
    *,
    center_u_gain: float,
    center_v_gain: float,
    z_bend_gain: float = 0.2,
    command_direction: Sequence[int] = (-1, 1, 1, 1),
    seg1_jacobian_scale: float = 0.30,
    seg2_jacobian_scale: float = 1.0,
) -> np.ndarray:
    """Initial 3x4 J for s=[u_d, v_d, z_err] with z_err=axial_remain (obs-target UV)."""
    dirs = tuple(int(v) for v in command_direction)
    if len(dirs) != NUM_Q:
        raise ValueError(f"command_direction must have length {NUM_Q}, got {len(dirs)}")
    s1 = float(max(abs(float(seg1_jacobian_scale)), 1e-6))
    s2 = float(max(abs(float(seg2_jacobian_scale)), 1e-6))
    j_uv = default_j_uv_seed(
        center_u_gain=float(center_u_gain),
        center_v_gain=float(center_v_gain),
        command_direction=dirs,
        seg1_jacobian_scale=s1,
        seg2_jacobian_scale=s2,
    )
    j = np.zeros((NUM_FEATURES, NUM_Q), dtype=float)
    j[0:2, :] = j_uv
    # z row is filled each control step from FK (∂remain/∂q along global approach_dir).
    _ = z_bend_gain  # deprecated; kept for config/API compatibility
    return j


def default_j_uv_seed(
    *,
    center_u_gain: float,
    center_v_gain: float,
    command_direction: Sequence[int] = (-1, 1, 1, 1),
    seg1_jacobian_scale: float = 0.30,
    seg2_jacobian_scale: float = 1.0,
) -> np.ndarray:
    """Map display-UV seed Jacobian (2x3 roll/s1/s2) into q-space (2x4)."""
    dirs = tuple(int(v) for v in command_direction)
    if len(dirs) != NUM_Q:
        raise ValueError(f"command_direction must have length {NUM_Q}, got {len(dirs)}")
    s1 = float(max(abs(float(seg1_jacobian_scale)), 1e-6))
    s2 = float(max(abs(float(seg2_jacobian_scale)), 1e-6))
    j_disp = default_uv_jacobian(
        center_u_gain=float(center_u_gain),
        center_v_gain=float(center_v_gain),
        seg1_coupling=0.5 * s1,
        seg2_coupling=0.5 * s2,
    )
    j_q = np.zeros((2, NUM_Q), dtype=float)
    j_q[:, 1] = float(dirs[1]) * j_disp[:, 0]
    j_q[:, 2] = float(dirs[2]) * j_disp[:, 1]
    j_q[:, 3] = float(dirs[3]) * j_disp[:, 2]
    return j_q


def clip_dq(
    dq: Sequence[float],
    *,
    max_dq_linear: float,
    max_dq_angle: float,
    max_dq_theta1: Optional[float] = None,
    max_dq_theta2: Optional[float] = None,
) -> np.ndarray:
    arr = np.asarray(dq, dtype=float).reshape(NUM_Q)
    out = arr.copy()
    out[0] = float(np.clip(out[0], -float(max_dq_linear), float(max_dq_linear)))
    roll_cap = float(max(max_dq_angle, 1e-9))
    t1_cap = float(max(max_dq_theta1 if max_dq_theta1 is not None else roll_cap, 1e-9))
    t2_cap = float(max(max_dq_theta2 if max_dq_theta2 is not None else roll_cap, 1e-9))
    out[1] = float(np.clip(out[1], -roll_cap, roll_cap))
    out[2] = float(np.clip(out[2], -t1_cap, t1_cap))
    out[3] = float(np.clip(out[3], -t2_cap, t2_cap))
    return out


def null_space_projector_mn(jacobian: np.ndarray, *, damping: float) -> np.ndarray:
    """N = I - J+ J for J shape (m, n)."""
    j = np.asarray(jacobian, dtype=float)
    if j.ndim != 2:
        raise ValueError(f"jacobian must be 2D, got {j.shape}")
    n = int(j.shape[1])
    j_pinv = damped_pseudoinverse_mn(j, float(damping))
    return np.eye(n, dtype=float) - j_pinv @ j


def compute_dq_lji(
    *,
    j_lji: np.ndarray,
    s_lji: Sequence[float],
    damping: float,
    gain_u: float,
    gain_v: float,
    gain_z: float,
    max_dq_linear: float,
    max_dq_angle: float,
    max_dq_theta1: Optional[float] = None,
    max_dq_theta2: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Stacked 3D LJI: z, u, v rows solved together (no null-space projection).

    Task-priority z→uv null space blocked display-opposite v when FK z row
  couples seg1/seg2 with the same sign (common bend for remain).
    """
    j = np.asarray(j_lji, dtype=float).reshape(NUM_FEATURES, NUM_Q)
    s = np.asarray(s_lji, dtype=float).reshape(NUM_FEATURES)
    lam = float(damping)
    j_stack = np.vstack(
        [
            float(gain_z) * j[2:3, :],
            float(gain_u) * j[0:1, :],
            float(gain_v) * j[1:2, :],
        ]
    )
    s_stack = np.array([float(s[2]), float(s[0]), float(s[1])], dtype=float)
    j_pinv = damped_pseudoinverse_mn(j_stack, lam)
    dq_raw = (-j_pinv @ s_stack.reshape(3, 1)).reshape(NUM_Q)
    dq = clip_dq(
        dq_raw,
        max_dq_linear=float(max_dq_linear),
        max_dq_angle=float(max_dq_angle),
        max_dq_theta1=max_dq_theta1,
        max_dq_theta2=max_dq_theta2,
    )
    return dq, dq_raw


@dataclass
class MotionSample:
    delta_q: np.ndarray
    delta_s: np.ndarray


@dataclass
class ImageJacobianEstimator3D:
    """Ring buffer for 3D [u, v, z] features."""

    window_size: int = 8
    seed_j: Optional[np.ndarray] = None
    min_measured_samples: int = 4
    condition_max: float = 100.0
    min_rank: int = 3
    _samples: Deque[MotionSample] = field(default_factory=deque)

    def __post_init__(self) -> None:
        if self.seed_j is not None:
            self.seed_j = np.asarray(self.seed_j, dtype=float).reshape(NUM_FEATURES, NUM_Q).copy()
        self._samples = deque(maxlen=max(1, int(self.window_size)))

    def clear(self) -> None:
        self._samples.clear()

    def push(self, delta_q: Sequence[float], delta_s: Sequence[float]) -> None:
        dq = np.asarray(delta_q, dtype=float).reshape(NUM_Q)
        ds = np.asarray(delta_s, dtype=float).reshape(NUM_FEATURES)
        self._samples.append(MotionSample(delta_q=dq.copy(), delta_s=ds.copy()))

    def sample_count(self) -> int:
        return len(self._samples)

    def _seed_estimate(self) -> Tuple[np.ndarray, int, float]:
        if self.seed_j is None:
            return np.zeros((NUM_FEATURES, NUM_Q), dtype=float), 0, float("inf")
        j = self.seed_j.copy()
        rank = int(np.linalg.matrix_rank(j))
        cond = float(np.linalg.cond(j)) if rank > 0 else float("inf")
        return j, rank, cond

    def _measured_ok(self, rank: int, cond: float) -> bool:
        if int(rank) < int(self.min_rank):
            return False
        if not np.isfinite(cond) or float(cond) > float(self.condition_max):
            return False
        return True

    def estimate(self) -> Tuple[np.ndarray, int, float]:
        need = max(1, int(self.min_measured_samples))
        if self.sample_count() < need:
            return self._seed_estimate()
        q_stack = np.stack([s.delta_q for s in self._samples], axis=0)
        s_stack = np.stack([s.delta_s for s in self._samples], axis=0)
        j_meas, rank, cond = estimate_j_img_from_stacks(q_stack, s_stack)
        if self._measured_ok(rank, cond):
            return j_meas, rank, cond
        return self._seed_estimate()

    def measured_estimate(
        self,
        *,
        min_samples: Optional[int] = None,
        condition_max: Optional[float] = None,
        min_rank: Optional[int] = None,
    ) -> Optional[Tuple[np.ndarray, int, float]]:
        """Return the online measured estimate only; never falls back to seed."""
        need = max(1, int(self.min_measured_samples if min_samples is None else min_samples))
        if self.sample_count() < need:
            return None
        q_stack = np.stack([s.delta_q for s in self._samples], axis=0)
        s_stack = np.stack([s.delta_s for s in self._samples], axis=0)
        j_meas, rank, cond = estimate_j_img_from_stacks(q_stack, s_stack)
        rank_min = int(self.min_rank if min_rank is None else min_rank)
        cond_max = float(self.condition_max if condition_max is None else condition_max)
        if int(rank) < rank_min:
            return None
        if np.isfinite(cond_max) and (not np.isfinite(cond) or float(cond) > cond_max):
            return None
        return j_meas, int(rank), float(cond)

    def is_usable(
        self,
        *,
        min_samples: int,
        condition_max: float,
        min_rank: int = 3,
    ) -> bool:
        if self.sample_count() < int(min_samples):
            if self.seed_j is None:
                return False
        j, rank, cond = self.estimate()
        if int(rank) < int(min_rank):
            return False
        if not np.isfinite(cond) or float(cond) > float(condition_max):
            return False
        return True


def check_sample_quality(
    *,
    delta_q: Sequence[float],
    min_dq_norm: float,
    object_lost: bool,
    settle_ok: bool,
    joint_saturated: bool,
) -> Tuple[bool, SampleRejectReason]:
    if bool(object_lost):
        return False, SampleRejectReason.OBJECT_LOST
    if not bool(settle_ok):
        return False, SampleRejectReason.SETTLE_TIMEOUT
    if bool(joint_saturated):
        return False, SampleRejectReason.JOINT_SATURATED
    dq = np.asarray(delta_q, dtype=float).reshape(NUM_Q)
    if float(np.linalg.norm(dq)) < float(min_dq_norm):
        return False, SampleRejectReason.DQ_TOO_SMALL
    return True, SampleRejectReason.ACCEPTED


def joint_saturated(
    q_before: Sequence[float],
    q_cmd: Sequence[float],
    q_after: Sequence[float],
    *,
    min_cmd: float = 1e-4,
    min_motion_frac: float = 0.10,
) -> bool:
    """True when a command produced essentially no measured joint motion.

    LJI estimates from measured ``delta_q``.  A multi-axis command can still be
    useful when only part of the arm moves, so do not reject a sample just
    because one commanded axis under-tracked or moved differently than expected.
    """
    before = np.asarray(q_before, dtype=float).reshape(NUM_Q)
    cmd = np.asarray(q_cmd, dtype=float).reshape(NUM_Q)
    after = np.asarray(q_after, dtype=float).reshape(NUM_Q)
    meas = after - before
    active = np.abs(cmd) >= float(min_cmd)
    if not bool(np.any(active)):
        return False
    cmd_norm = float(np.linalg.norm(cmd[active]))
    meas_norm = float(np.linalg.norm(meas[active]))
    if cmd_norm <= 1e-9:
        return False
    frac = min(float(min_motion_frac), 0.05)
    return meas_norm < cmd_norm * frac


@dataclass(frozen=True)
class LocalImageJacobianServoGains:
    damping: float = 0.05
    gain_u: float = 0.5
    gain_v: float = 0.5
    gain_z: float = 0.5
    max_dq_linear: float = 0.005
    max_dq_angle: float = 0.03


class LocalImageJacobianServo3D:
    """3D [u,v,z] local image Jacobian: dq = -J+ K s."""

    def __init__(
        self,
        *,
        estimator: ImageJacobianEstimator3D,
        gains: LocalImageJacobianServoGains,
        min_samples: int = 4,
        condition_max: float = 100.0,
        min_rank: int = 3,
        command_direction: Sequence[int] = (-1, -1, 1, -1),
        measured_v_row_blend: float = 0.0,
        measured_v_row_norm_max: float = 120.0,
    ) -> None:
        self.estimator = estimator
        self.gains = gains
        self.min_samples = int(min_samples)
        self.condition_max = float(condition_max)
        self.min_rank = int(min_rank)
        self.command_direction = tuple(int(v) for v in command_direction)
        self.measured_v_row_blend = float(np.clip(measured_v_row_blend, 0.0, 1.0))
        self.measured_v_row_norm_max = float(max(measured_v_row_norm_max, 1e-6))

    def j_available(self) -> bool:
        return self.estimator.is_usable(
            min_samples=self.min_samples,
            condition_max=self.condition_max,
            min_rank=self.min_rank,
        )

    def compute_dq(
        self,
        s_lji: Sequence[float],
        *,
        z_row: Optional[Sequence[float]] = None,
        max_dq_linear: Optional[float] = None,
        max_dq_angle: Optional[float] = None,
        max_dq_theta1: Optional[float] = None,
        max_dq_theta2: Optional[float] = None,
        gain_u: Optional[float] = None,
        gain_v: Optional[float] = None,
        gain_z: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int, float, bool]:
        """
        Returns (dq, dq_raw, j_lji, rank, cond, j_available).

        When z_row is set, replaces J row 2 before control (FK axial coupling).
        """
        j, rank, cond = self.estimator.estimate()
        seed_j = self.estimator.seed_j
        if z_row is not None and seed_j is not None:
            j = patch_lji_jacobian_for_control(
                j,
                z_row=z_row,
                seed_j=seed_j,
                command_direction=self.command_direction,
                measured_v_row_blend=float(self.measured_v_row_blend),
                measured_v_row_norm_max=float(self.measured_v_row_norm_max),
            )
            rank = int(np.linalg.matrix_rank(j))
            cond = float(np.linalg.cond(j)) if rank > 0 else float("inf")
        else:
            j = np.asarray(j, dtype=float).copy()
            if z_row is not None:
                j[2, :] = np.asarray(z_row, dtype=float).reshape(NUM_Q)
                rank = int(np.linalg.matrix_rank(j))
                cond = float(np.linalg.cond(j)) if rank > 0 else float("inf")
        lin_cap = (
            float(self.gains.max_dq_linear)
            if max_dq_linear is None
            else float(max_dq_linear)
        )
        ang_cap = (
            float(self.gains.max_dq_angle)
            if max_dq_angle is None
            else float(max_dq_angle)
        )
        dq, dq_raw = compute_dq_lji(
            j_lji=j,
            s_lji=s_lji,
            damping=float(self.gains.damping),
            gain_u=float(self.gains.gain_u if gain_u is None else gain_u),
            gain_v=float(self.gains.gain_v if gain_v is None else gain_v),
            gain_z=float(self.gains.gain_z if gain_z is None else gain_z),
            max_dq_linear=lin_cap,
            max_dq_angle=ang_cap,
            max_dq_theta1=max_dq_theta1,
            max_dq_theta2=max_dq_theta2,
        )
        j_ready = (
            int(rank) >= int(self.min_rank)
            and np.isfinite(cond)
            and float(cond) <= float(self.condition_max)
        )
        return (
            dq,
            dq_raw,
            j,
            int(rank),
            float(cond),
            bool(j_ready),
        )


# --- Legacy 2D helpers (kept for unit tests of null-space math) ---

NUM_UV = 2


def uv_aligned(s_uv: Sequence[float], *, tol: float) -> bool:
    s = np.asarray(s_uv, dtype=float).reshape(2)
    t = float(max(tol, 0.0))
    return bool(abs(float(s[0])) <= t and abs(float(s[1])) <= t)


def null_space_projector(j_uv: np.ndarray, *, damping: float) -> np.ndarray:
    j = np.asarray(j_uv, dtype=float).reshape(2, NUM_Q)
    j_pinv = damped_pseudoinverse_mn(j, float(damping))
    eye = np.eye(NUM_Q, dtype=float)
    return eye - j_pinv @ j


def compose_dq_align_and_approach(
    *,
    j_uv: np.ndarray,
    s_uv: Sequence[float],
    dq_approach_seed: Sequence[float],
    damping: float,
    gain_u: float,
    gain_v: float,
    approach_bias_gain: float,
    enable_approach: bool,
    max_dq_linear: float,
    max_dq_angle: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    j = np.asarray(j_uv, dtype=float).reshape(2, NUM_Q)
    s = np.asarray(s_uv, dtype=float).reshape(2)
    k = np.diag([float(gain_u), float(gain_v)])
    j_pinv = damped_pseudoinverse_mn(j, float(damping))
    dq_align = -j_pinv @ k @ s
    dq_approach = np.zeros(NUM_Q, dtype=float)
    if bool(enable_approach):
        n_proj = null_space_projector(j, damping=float(damping))
        seed = np.asarray(dq_approach_seed, dtype=float).reshape(NUM_Q)
        dq_approach = float(approach_bias_gain) * (n_proj @ seed)
    dq = clip_dq(
        dq_align + dq_approach,
        max_dq_linear=float(max_dq_linear),
        max_dq_angle=float(max_dq_angle),
    )
    return dq, dq_align, dq_approach
