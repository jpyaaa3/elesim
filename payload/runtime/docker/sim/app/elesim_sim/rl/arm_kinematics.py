"""Batched 4-DoF waypoint -> joint-target mapping for the continuum arm.

The arm exposes four commanded degrees of freedom:

===========  ==================================  ====================
DoF          joint(s)                            meaning
===========  ==================================  ====================
``linear``   ``j_plate_housing`` (prismatic)      base translation
``roll``     ``j_housing_wedge`` (revolute)       bend-plane rotation
``theta1``   proximal ``n_seg`` bend joints       per-node angle, seg 1
``theta2``   distal ``n_seg`` bend joints         per-node angle, seg 2
===========  ==================================  ====================

``theta1``/``theta2`` are **per-node** angles: every node in a segment gets the
same value, which is what makes the segment an arc of constant curvature — the
gear backbone enforces that mechanically, so the whole arm shape is always two
circular arcs.  The segment total is therefore ``n_seg * theta`` and the limit
that matters is the per-node one.

This mirrors the convention in ``elesim_sim.runtime.target_from_4dof`` but is a
separate, tensor-batched implementation: the interactive simulator maps one
env with numpy, and reusing it per-env would defeat the point of a parallel
scene.  The existing runtime is left untouched.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import torch

from .configs.loader import ArmConfig


@dataclass(frozen=True)
class ArmDofIndex:
    """Local DOF indices, resolved from the Genesis entity by joint name."""

    linear: int
    roll: int
    bend: tuple[int, ...]

    @property
    def all_indices(self) -> tuple[int, ...]:
        return (self.linear, self.roll, *self.bend)


def resolve_dof_index(entity, cfg: ArmConfig) -> ArmDofIndex:
    """Look up the arm's local DOF indices on a built Genesis entity.

    Raises rather than guessing: a silently mismatched index would drive the
    wrong joint and the failure would look like a physics problem.
    """

    def one(joint_name: str) -> int:
        joint = entity.get_joint(str(joint_name))
        idxs = getattr(joint, "dofs_idx_local", None)
        if idxs is None:
            raise RuntimeError(f"joint {joint_name!r} exposes no dofs_idx_local")
        flat = [int(i) for i in (idxs if hasattr(idxs, "__iter__") else [idxs])]
        if len(flat) != 1:
            raise RuntimeError(
                f"joint {joint_name!r} has {len(flat)} DOFs; expected exactly 1"
            )
        return flat[0]

    bend = tuple(one(name) for name in cfg.bend_joints)
    if len(bend) < 2 * cfg.n_seg:
        raise RuntimeError(
            f"arm.bend_joints resolved {len(bend)} joints but n_seg={cfg.n_seg} "
            f"requires {2 * cfg.n_seg}"
        )
    return ArmDofIndex(linear=one(cfg.linear_joint), roll=one(cfg.roll_joint), bend=bend)


class ArmWaypointMapper:
    """Holds the commanded 4-DoF waypoint per env and expands it to joints."""

    #: Order of the action/waypoint vector.
    DOF_NAMES = ("linear", "roll", "theta1", "theta2")

    def __init__(
        self,
        cfg: ArmConfig,
        *,
        n_envs: int,
        device: torch.device,
        rate_limit: Sequence[float],
    ) -> None:
        self.cfg = cfg
        self.n_envs = int(n_envs)
        self.device = device
        self.n_bend = len(cfg.bend_joints)
        self.n_seg = int(cfg.n_seg)

        lo_lin, hi_lin = cfg.limits.linear_m
        lo_roll, hi_roll = cfg.limits.roll_rad
        bend_lim = float(cfg.limits.bend_per_node_rad)
        self.lower = torch.tensor(
            [float(lo_lin), float(lo_roll), -bend_lim, -bend_lim],
            device=device,
            dtype=torch.float32,
        )
        self.upper = torch.tensor(
            [float(hi_lin), float(hi_roll), bend_lim, bend_lim],
            device=device,
            dtype=torch.float32,
        )
        # Coupled cap on how far the two segments may curl the same way at
        # once, keeping the grossest self-folds out of the action space.
        #
        # The backbone reaches its own housing when both segments curl the same
        # way far enough.  Swept over a 9x9 (theta1, theta2) grid with the
        # object parked away, every folding cell and every clean one is
        # separated exactly by |1.5*theta1 + theta2| > 60 deg per node -- not by
        # the plain sum: (24, 24) is clean at 48 while (30, 18) folds at the
        # same 48, because segment 1 is the one that swings back towards the
        # base.  Opposite signs, the S shape, never fold; their curls cancel and
        # the signed form handles that on its own.
        #
        # Cap on how far the two segments may curl the same way at once, which
        # keeps the deep self-folds out of the action space.  That is all it
        # does, and all it can do.
        #
        # Measured statically -- joints written directly, so the pose is the
        # only variable -- the backbone reaches its own housing at curl 52 for
        # roll 0, 54 for +/-45 deg and 61 for +/-90.  Curl is the variable and
        # roll shifts the threshold by about 10 deg.  But wrapping needs curl
        # 58.5, which folds in free space at every roll bar +/-90 and is
        # marginal even there, so no cap on the commanded pose can both admit
        # the wrap and forbid the fold: what makes the wrap pose safe is the
        # object holding the coil open, and that is a contact condition.
        #
        # So the cap is set just above what the wrap needs.  Keeping the fold
        # out of the *reachable* set is reward.self_contact's job, and getting
        # the policy to surround the object before closing on it is
        # enclosure_progress's.
        #
        # None disables the cap.
        self.theta1_curl_weight = float(cfg.limits.theta1_curl_weight)
        self.curl_limit = (
            None
            if cfg.limits.curl_limit_per_node_rad is None
            else float(cfg.limits.curl_limit_per_node_rad)
        )
        self.rate_limit = torch.tensor(
            [float(v) for v in rate_limit], device=device, dtype=torch.float32
        )
        if self.rate_limit.numel() != 4:
            raise ValueError("rate_limit must have 4 entries (linear, roll, t1, t2)")

        # Per-node membership mask: column j is 1 for segment-1 nodes.
        seg1 = torch.zeros(self.n_bend, device=device, dtype=torch.float32)
        seg1[: self.n_seg] = 1.0
        self._seg1_mask = seg1
        self._seg2_mask = 1.0 - seg1

        self.waypoint = torch.zeros(
            (self.n_envs, 4), device=device, dtype=torch.float32
        )

    # -- waypoint bookkeeping ---------------------------------------------

    def reset(
        self, env_ids: Optional[torch.Tensor] = None, *, home: Optional[torch.Tensor] = None
    ) -> None:
        """Reset the commanded waypoint to `home` (defaults to all zeros)."""
        target = (
            torch.zeros(4, device=self.device, dtype=torch.float32)
            if home is None
            else home.to(self.device, torch.float32).reshape(4)
        )
        target = target.clamp(self.lower, self.upper)
        target = self._project_curl(target.reshape(1, 4)).reshape(4)
        if env_ids is None:
            self.waypoint[:] = target
        elif env_ids.numel():
            self.waypoint[env_ids] = target

    def apply_action(self, action: torch.Tensor) -> torch.Tensor:
        """Advance the waypoint by a rate-limited increment.

        `action` is the policy output in roughly [-1, 1] per DoF; it is clamped
        and scaled by the per-DoF rate limit, so one macro step can never
        command an arbitrarily large jump.  Returns the new waypoint.
        """
        act = action.to(self.device, torch.float32).reshape(self.n_envs, 4)
        increment = act.clamp(-1.0, 1.0) * self.rate_limit
        candidate = (self.waypoint + increment).clamp(self.lower, self.upper)
        self.waypoint = self._truncate_to_curl_limit(self.waypoint, candidate)
        return self.waypoint

    # -- coupled bend limit ------------------------------------------------

    def _curl(self, waypoint: torch.Tensor) -> torch.Tensor:
        """Weighted curl the cap is expressed in."""
        return self.theta1_curl_weight * waypoint[:, 2] + waypoint[:, 3]

    def _curl_cap(self, waypoint: torch.Tensor) -> torch.Tensor:
        return torch.full_like(waypoint[:, 1], self.curl_limit)

    def _within_curl(self, waypoint: torch.Tensor) -> torch.Tensor:
        return self._curl(waypoint).abs() <= self._curl_cap(waypoint) + 1e-9

    def _project_curl(self, waypoint: torch.Tensor) -> torch.Tensor:
        """Scale theta1/theta2 down until their signed sum is within limit.

        Used for a waypoint with no feasible predecessor to walk back towards,
        which is only the reset pose.  Scaling both keeps the shape of the coil.
        """
        if self.curl_limit is None:
            return waypoint
        out = waypoint.clone()
        total = self._curl(out)
        cap = self._curl_cap(out)
        excess = total.abs() > cap
        if not bool(excess.any()):
            return out
        scale = (cap / total.abs().clamp_min(1e-9)).clamp(max=1.0)
        out[:, 2] = torch.where(excess, out[:, 2] * scale, out[:, 2])
        out[:, 3] = torch.where(excess, out[:, 3] * scale, out[:, 3])
        return out

    def _truncate_to_curl_limit(
        self, current: torch.Tensor, candidate: torch.Tensor
    ) -> torch.Tensor:
        """Shorten the step so it stops at the curl limit instead of crossing it.

        The step is truncated rather than the pair rescaled: `current` is
        already feasible, so walking part-way along the increment leaves the
        DoFs the policy did not ask to move where they were.  This is what a
        rate-limited axis meeting its own limit does.
        """
        if self.curl_limit is None:
            return candidate
        # Roll and the linear stage always pass; only the two bend columns are
        # held back.  Shortening the whole step instead would block the roll
        # whenever the curl was at its cap, and the wrap cannot be reached
        # without rolling.
        cap = self._curl_cap(candidate)
        s_now = self._curl(current)
        s_new = self._curl(candidate)
        out = candidate.clone()

        # Curl is linear in the bend columns, so the largest feasible fraction
        # of the bend step has a closed form.
        bound = torch.where(s_new >= 0, cap, -cap)
        denom = s_new - s_now
        alpha = torch.where(
            denom.abs() > 1e-12,
            (bound - s_now) / torch.where(denom.abs() > 1e-12, denom, torch.ones_like(denom)),
            torch.ones_like(denom),
        ).clamp(0.0, 1.0)
        over = s_new.abs() > cap
        for col in (2, 3):
            walked = current[:, col] + alpha * (candidate[:, col] - current[:, col])
            out[:, col] = torch.where(over, walked, candidate[:, col])

        # If the *current* bend is already outside the cap the roll has just
        # moved, so there is no fraction of the step to walk and the bend has to
        # come down.  Scaling both columns keeps the shape of the coil.
        stranded = s_now.abs() > cap
        if bool(stranded.any()):
            scale = (cap / s_now.abs().clamp_min(1e-9)).clamp(max=1.0)
            for col in (2, 3):
                out[:, col] = torch.where(
                    stranded, current[:, col] * scale, out[:, col]
                )
        return out

    # -- expansion ---------------------------------------------------------

    def joint_targets(self, waypoint: Optional[torch.Tensor] = None) -> torch.Tensor:
        """Expand (linear, roll, theta1, theta2) to per-joint DOF targets.

        Returns a ``(n_envs, 2 + n_bend)`` tensor ordered as
        ``[linear, roll, bend_0 .. bend_{n-1}]``, matching
        :meth:`ArmDofIndex.all_indices`.  Axis signs from the config are
        applied here so callers can hand the result straight to Genesis.
        """
        wp = self.waypoint if waypoint is None else waypoint
        wp = wp.to(self.device, torch.float32).reshape(-1, 4)
        linear = wp[:, 0:1] * float(self.cfg.linear_axis_sign)
        roll = wp[:, 1:2] * float(self.cfg.roll_axis_sign)
        theta1 = wp[:, 2:3]
        theta2 = wp[:, 3:4]
        bend = (
            theta1 * self._seg1_mask.unsqueeze(0) + theta2 * self._seg2_mask.unsqueeze(0)
        ) * float(self.cfg.bend_axis_sign)
        return torch.cat((linear, roll, bend), dim=1)

    def segment_totals(self, waypoint: Optional[torch.Tensor] = None) -> torch.Tensor:
        """Total bend of each segment, ``(n_envs, 2)`` in radians.

        Per-node angle times ``n_seg``; useful for logging and for reasoning
        about wrap geometry, where the arc angle is what matters.
        """
        wp = self.waypoint if waypoint is None else waypoint
        wp = wp.to(self.device, torch.float32).reshape(-1, 4)
        return wp[:, 2:4] * float(self.n_seg)
