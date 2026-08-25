"""Macro-step wrap-grasp environment (rsl_rl VecEnv).

This is a **macro-step MDP**, not a 50 Hz reactive controller.  One
``step(action)`` is:

1. apply a rate-limited waypoint increment to the 4-DoF command,
2. run the arm to that waypoint over ``macro_step.substeps`` physics substeps,
3. observe the *settled* state.

Episodes are a handful of macro steps (``macro_step.max_steps``), so the policy
plans a sequence of poses rather than servoing a trajectory.

Two invariants the rest of the code depends on:

* **Collisions and object disturbance are judged over the whole substep
  window.**  Reading only the settled pose misses a link that swept through
  the floor and came back out, which is exactly what the collision penalty
  exists to catch.  :class:`ContactAggregator` does that accumulation.

* **The actor never observes simulator truth.**  Its joint state arrives
  through the beta estimator, the object pose is noised, and there is an
  optional macro-step delay.  Ground-truth joints, contact forces and the
  exact object pose go to the ``privileged`` group, which only the critic
  reads.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import torch
from tensordict import TensorDict

from ..arm_kinematics import ArmWaypointMapper
from ..beta_model import BetaModel
from ..configs.loader import WrapGraspConfig, to_dict
from ..scene import WrapGraspScene
from .contacts import ContactAggregator, ContactClassifier
from .coverage import CoverageMeter, quat_to_axis
from .lift_test import GeometricCriterion, LiftObservation, LiftTest

_TWO_PI = 2.0 * math.pi

#: Failure taxonomy used by both training logs and eval reports.
FAILURE_MODES: tuple[str, ...] = ("collision", "topple", "retention", "timeout")


@dataclass
class ObsSpec:
    """Widths of the two observation groups, derived from the config toggles."""

    policy: int
    privileged: int


class WrapGraspEnv:
    """Parallel macro-step environment satisfying rsl_rl's VecEnv contract."""

    def __init__(
        self,
        cfg: WrapGraspConfig,
        *,
        n_envs: Optional[int] = None,
        scene: Optional[WrapGraspScene] = None,
    ) -> None:
        self.cfg = cfg.resolved_for_curriculum()
        self.scene = scene if scene is not None else WrapGraspScene(self.cfg, n_envs=n_envs).build()
        if not self.scene.built:
            self.scene.build()
        self.device = self.scene.device
        self.num_envs = self.scene.n_envs
        self.num_actions = 4
        self.max_episode_length = int(self.cfg.macro_step.max_steps)

        rate = self.cfg.macro_step.rate_limit
        self.mapper = ArmWaypointMapper(
            self.cfg.arm,
            n_envs=self.num_envs,
            device=self.device,
            rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
        )
        self._generator = torch.Generator(device=self.device)
        self._generator.manual_seed(int(self.cfg.runtime.seed))

        self.beta = BetaModel(
            self.cfg.beta,
            n_envs=self.num_envs,
            n_joints=len(self.cfg.arm.bend_joints),
            device=self.device,
            generator=self._generator,
        )
        self.coverage = CoverageMeter(
            n_bins=self.cfg.reward.coverage.n_bins,
            radial_band_m=self.cfg.reward.coverage.radial_band_m,
            device=self.device,
        )
        self.classifier = ContactClassifier(self.scene)
        self.contacts = ContactAggregator(self.classifier, n_envs=self.num_envs)

        from .rewards import RewardBook  # local import keeps the module graph flat

        self.rewards = RewardBook(
            self.cfg.reward,
            n_envs=self.num_envs,
            device=self.device,
            wrap_threshold_rad=self.cfg.success.coverage_target_rad,
        )
        self.geometric = GeometricCriterion(self.cfg.success, device=self.device)
        self.lift: Optional[LiftTest] = (
            LiftTest(self.cfg.success, n_envs=self.num_envs, device=self.device)
            if self.cfg.success.criterion == "lift"
            else None
        )

        self._arm_dofs = list(self.scene.arm_dofs.all_indices)
        self._bend_slice = slice(2, 2 + len(self.cfg.arm.bend_joints))
        # Robot-*local* indices: get_links_pos is indexed per entity, while
        # links.arm holds the scene-global ids that get_contacts reports.
        self._arm_link_ids = torch.tensor(
            sorted(self.scene.links.arm_local), device=self.device, dtype=torch.long
        )
        self._anchor_link = int(self.scene.links.segment2_mid_local)

        z = lambda *shape: torch.zeros(*shape, device=self.device, dtype=torch.float32)  # noqa: E731
        self.episode_length_buf = torch.zeros(
            self.num_envs, device=self.device, dtype=torch.long
        )
        self._object_radius = z(self.num_envs)
        self._object_height = z(self.num_envs)
        self._object_mass = z(self.num_envs)
        self._object_pos0 = z(self.num_envs, 3)
        self._object_axis0 = z(self.num_envs, 3)
        self._load_proxy = z(self.num_envs, self.num_actions)
        self._last_joint_cmd = z(self.num_envs, len(self._arm_dofs))
        self._waypoint_from = z(self.num_envs, self.num_actions)
        self._failure_counts = {
            mode: torch.zeros(1, device=self.device, dtype=torch.long)
            for mode in FAILURE_MODES
        }
        self._success_count = torch.zeros(1, device=self.device, dtype=torch.long)
        self._episode_count = torch.zeros(1, device=self.device, dtype=torch.long)

        self.obs_spec = self._compute_obs_spec()
        delay_lo, delay_hi = self.cfg.observation.actor.delay_steps
        self._delay_lo, self._delay_hi = int(delay_lo), int(delay_hi)
        self._obs_history: deque[torch.Tensor] = deque(
            maxlen=max(self._delay_hi, 0) + 1
        )
        self._obs_delay = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)

        self._reset_idx(None)
        self._obs = self._build_observations()

    # -- rsl_rl contract ---------------------------------------------------

    def get_observations(self) -> TensorDict:
        return self._obs

    def reset(self) -> tuple[TensorDict, dict]:
        self._reset_idx(None)
        self._obs = self._build_observations()
        return self._obs, {}

    def step(
        self, actions: torch.Tensor
    ) -> tuple[TensorDict, torch.Tensor, torch.Tensor, dict]:
        from .rewards import RewardInputs

        actions = actions.to(self.device, torch.float32)
        self._apply_action(actions)
        self.contacts.reset()
        self._simulate_macro_step()

        state = self._read_state()
        contact = self.contacts.result()

        success = self._evaluate_success(state, contact)
        reward_out = self.rewards.step(
            RewardInputs(
                phi=state["phi"],
                surface_dist=state["surface_dist"],
                object_touch=contact.object_touch,
                non_target_collision=contact.non_target_collision,
                object_displacement=state["displacement"],
                object_tilt=state["tilt"],
                success=success,
            )
        )

        self.episode_length_buf += 1
        timeout = self.episode_length_buf >= self.max_episode_length
        dones = reward_out.terminate | timeout

        extras: dict[str, Any] = {
            "log": self._logging_extras(reward_out, contact, state, timeout),
            "time_outs": timeout,
        }
        self._tally(reward_out, timeout, dones)

        done_ids = dones.nonzero(as_tuple=False).flatten()
        if done_ids.numel():
            self._reset_idx(done_ids)
        self._obs = self._build_observations()
        return self._obs, reward_out.total, dones, extras

    # -- action and simulation --------------------------------------------

    def _apply_action(self, actions: torch.Tensor) -> None:
        """Advance the waypoint, then override roll for envs under lift script."""
        follows_policy = (
            self.lift.follows_policy
            if self.lift is not None
            else torch.ones(self.num_envs, device=self.device, dtype=torch.bool)
        )
        masked = torch.where(
            follows_policy.unsqueeze(-1), actions, torch.zeros_like(actions)
        )
        self._waypoint_from = self.mapper.waypoint.clone()
        self.mapper.apply_action(masked)

    def _commanded_joints(self) -> torch.Tensor:
        return self.mapper.joint_targets()

    def _realised_joints(self, commanded: torch.Tensor) -> torch.Tensor:
        """Inject the beta residual into the bend joints only.

        Roll and the linear stage are geared differently and the residual model
        is identified for the bend backbone, so applying it to all four DoFs
        would be inventing data the config cannot back up.
        """
        realised = commanded.clone()
        bend_cmd = commanded[:, self._bend_slice]
        prev_bend = self._last_joint_cmd[:, self._bend_slice]
        realised[:, self._bend_slice] = self.beta.corrupt(
            bend_cmd, load_kg=self._object_mass, previous_commanded=prev_bend
        )
        self._last_joint_cmd = commanded.clone()
        return realised

    def _simulate_macro_step(self) -> None:
        settle = self.cfg.macro_step.settle
        substeps = int(self.cfg.macro_step.substeps)
        # Interpolate the joint target across the moving portion of the window
        # instead of stepping it to the new waypoint at once.  A whole rate
        # limit applied in one substep is an impulse, and the arm knocks the
        # object out of place before it can close around it.
        move_steps = max(1, int(round(substeps * float(self.cfg.macro_step.move_fraction))))
        start = self._waypoint_from
        end = self.mapper.waypoint
        hold = 0

        for i in range(substeps):
            alpha = min(1.0, float(i + 1) / move_steps)
            waypoint = start + (end - start) * alpha
            targets = self._realised_joints(self.mapper.joint_targets(waypoint))
            if self.lift is not None:
                targets = self._apply_lift_script(targets)
            self.scene.robot.control_dofs_position(
                targets, dofs_idx_local=self._arm_dofs
            )
            self.scene.step()
            self.contacts.accumulate()

            if self.lift is not None:
                self.lift.advance(self._lift_observation())

            if settle.mode == "velocity" and (i + 1) >= int(settle.min_substeps):
                if bool(self._is_settled().all()):
                    hold += 1
                    if hold >= int(settle.hold_substeps):
                        break
                else:
                    hold = 0

        self._load_proxy = self._read_load_proxy()

    def _apply_lift_script(self, targets: torch.Tensor) -> torch.Tensor:
        """Override the roll column for envs the lift state machine controls."""
        assert self.lift is not None
        scripted = ~self.lift.follows_policy
        if not bool(scripted.any()):
            return targets
        out = targets.clone()
        roll = self.lift.roll_command * float(self.cfg.arm.roll_axis_sign)
        out[:, 1] = torch.where(scripted, roll, out[:, 1])
        return out

    def _is_settled(self) -> torch.Tensor:
        vel = self.scene.robot.get_dofs_velocity(dofs_idx_local=self._arm_dofs)
        joint_ok = vel.abs().amax(dim=-1) < float(
            self.cfg.macro_step.settle.joint_vel_thresh
        )
        obj_ok = self.scene.object.get_vel().norm(dim=-1) < float(
            self.cfg.macro_step.settle.object_vel_thresh
        )
        return joint_ok & obj_ok

    def _read_load_proxy(self) -> torch.Tensor:
        """Per-DoF torque, the stand-in for the real arm's motor current.

        The hardware has no contact sensing, but it does report load, which is
        why this is allowed in the actor's observation while contact forces are
        not.
        """
        force = self.scene.robot.get_dofs_force(dofs_idx_local=self._arm_dofs)
        bend = force[:, self._bend_slice].mean(dim=-1, keepdim=True)
        return torch.cat((force[:, 0:1], force[:, 1:2], bend, bend), dim=-1)

    # -- state -------------------------------------------------------------

    def _lift_observation(self) -> LiftObservation:
        pos = self.scene.robot.get_links_pos()
        quat = self.scene.robot.get_links_quat()
        return LiftObservation(
            object_pos=self.scene.object.get_pos(),
            object_quat=self.scene.object.get_quat(),
            anchor_pos=pos[:, self._anchor_link, :],
            anchor_quat=quat[:, self._anchor_link, :],
        )

    def _read_state(self) -> dict[str, torch.Tensor]:
        obj_pos = self.scene.object.get_pos()
        obj_quat = self.scene.object.get_quat()
        link_pos = self.scene.robot.get_links_pos()[:, self._arm_link_ids, :]
        cov = self.coverage.measure(
            link_pos,
            obj_pos,
            obj_quat,
            radius_m=self._object_radius,
            height_m=self._object_height,
        )
        anchor = self.scene.robot.get_links_pos()[:, self._anchor_link, :]
        radial = (anchor - obj_pos)
        axis = quat_to_axis(obj_quat)
        along = (radial * axis).sum(dim=-1, keepdim=True)
        planar = radial - along * axis
        anchor_surface = planar.norm(dim=-1) - self._object_radius

        displacement = (obj_pos - self._object_pos0).norm(dim=-1)
        tilt = torch.acos(
            (axis * self._object_axis0).sum(dim=-1).abs().clamp(max=1.0)
        )
        joints = self.scene.robot.get_dofs_position(dofs_idx_local=self._arm_dofs)
        joint_vel = self.scene.robot.get_dofs_velocity(dofs_idx_local=self._arm_dofs)
        return {
            "phi": cov.phi_rad,
            "caged": cov.caged,
            "gap_rad": cov.gap_rad,
            "gap_width_m": cov.gap_width_m,
            "coverage_near": cov.n_near_links.to(torch.float32),
            "surface_dist": anchor_surface,
            "min_surface_dist": cov.min_surface_dist,
            "displacement": displacement,
            "tilt": tilt,
            "object_pos": obj_pos,
            "object_quat": obj_quat,
            "joints": joints,
            "joint_vel": joint_vel,
        }

    def _evaluate_success(
        self, state: dict[str, torch.Tensor], contact: Any
    ) -> torch.Tensor:
        if self.lift is None:
            reached = self.geometric.evaluate(state["phi"])
            if self.cfg.reward.coverage.require_caging:
                reached = reached & state["caged"]
            return reached
        roll_now = state["joints"][:, 1] * float(self.cfg.arm.roll_axis_sign)
        self.lift.arm(state["phi"], roll_now, self._lift_observation())
        return self.lift.passed

    # -- observations ------------------------------------------------------

    def _compute_obs_spec(self) -> ObsSpec:
        actor_cfg = self.cfg.observation.actor
        policy = 0
        if actor_cfg.include_joint_estimate:
            policy += 4
        if actor_cfg.include_object_geometry:
            policy += 7  # radius, height, pos(3), yaw sin/cos
        if actor_cfg.include_load_proxy:
            policy += 4
        if actor_cfg.include_step_index:
            policy += 1

        critic_cfg = self.cfg.observation.critic_privileged
        priv = 0
        if critic_cfg.include_true_joint_state:
            priv += 2 * len(self._arm_dofs)
        if critic_cfg.include_contact_forces:
            priv += 1 + len(self.scene.links.arm) + 3
        if critic_cfg.include_true_object_pose:
            priv += 7
        if critic_cfg.include_coverage:
            priv += 2
        return ObsSpec(policy=policy, privileged=priv)

    def _noise(self, shape: tuple[int, ...], sigma: float) -> torch.Tensor:
        if sigma <= 0.0:
            return torch.zeros(shape, device=self.device, dtype=torch.float32)
        return torch.randn(
            shape, device=self.device, dtype=torch.float32, generator=self._generator
        ) * float(sigma)

    def _actor_observation(self) -> torch.Tensor:
        cfg = self.cfg.observation.actor
        noise = cfg.noise
        parts: list[torch.Tensor] = []

        realised = self.scene.robot.get_dofs_position(dofs_idx_local=self._arm_dofs)
        if cfg.include_joint_estimate:
            bend_est = self.beta.estimate(
                realised[:, self._bend_slice], load_kg=self._object_mass
            )
            seg = int(self.cfg.arm.n_seg)
            estimate = torch.stack(
                (
                    realised[:, 0],
                    realised[:, 1],
                    bend_est[:, :seg].mean(dim=-1),
                    bend_est[:, seg:].mean(dim=-1),
                ),
                dim=-1,
            )
            parts.append(estimate + self._noise(estimate.shape, noise.joint_rad))

        if cfg.include_object_geometry:
            pos = self.scene.object.get_pos() + self._noise((self.num_envs, 3), noise.object_pos_m)
            axis = quat_to_axis(self.scene.object.get_quat())
            yaw = torch.atan2(axis[:, 1], axis[:, 0]) + self._noise(
                (self.num_envs,), noise.object_rot_rad
            )
            parts.append(
                torch.cat(
                    (
                        self._object_radius.unsqueeze(-1),
                        self._object_height.unsqueeze(-1),
                        pos,
                        torch.sin(yaw).unsqueeze(-1),
                        torch.cos(yaw).unsqueeze(-1),
                    ),
                    dim=-1,
                )
            )

        if cfg.include_load_proxy:
            parts.append(
                self._load_proxy + self._noise(self._load_proxy.shape, noise.load_proxy)
            )

        if cfg.include_step_index:
            frac = self.episode_length_buf.to(torch.float32) / max(
                self.max_episode_length, 1
            )
            parts.append(frac.unsqueeze(-1))
        return torch.cat(parts, dim=-1)

    def _privileged_observation(
        self, state: dict[str, torch.Tensor], contact: Any
    ) -> torch.Tensor:
        cfg = self.cfg.observation.critic_privileged
        parts: list[torch.Tensor] = []
        if cfg.include_true_joint_state:
            parts.append(state["joints"])
            parts.append(state["joint_vel"])
        if cfg.include_contact_forces:
            parts.append(contact.object_force_peak.unsqueeze(-1))
            parts.append(contact.object_link_hits.to(torch.float32))
            parts.append(
                torch.stack(
                    (
                        contact.floor_touch.to(torch.float32),
                        contact.go2_touch.to(torch.float32),
                        contact.self_touch.to(torch.float32),
                    ),
                    dim=-1,
                )
            )
        if cfg.include_true_object_pose:
            parts.append(state["object_pos"])
            parts.append(state["object_quat"])
        if cfg.include_coverage:
            parts.append(state["phi"].unsqueeze(-1))
            parts.append(state["coverage_near"].unsqueeze(-1))
        return torch.cat(parts, dim=-1)

    def _build_observations(
        self,
        state: Optional[dict[str, torch.Tensor]] = None,
        contact: Optional[Any] = None,
    ) -> TensorDict:
        if state is None:
            state = self._read_state()
        if contact is None:
            contact = self.contacts.result()
        actor = self._actor_observation()
        actor = self._apply_obs_delay(actor)
        privileged = self._privileged_observation(state, contact)
        return TensorDict(
            {"policy": actor, "privileged": privileged},
            batch_size=[self.num_envs],
            device=self.device,
        )

    def _apply_obs_delay(self, actor: torch.Tensor) -> torch.Tensor:
        """Serve each env an observation from `delay` macro steps ago.

        The delay is per-env and redrawn on reset, so the policy has to be
        robust to a stale reading rather than learning one fixed lag.
        """
        self._obs_history.append(actor.clone())
        if self._delay_hi <= 0:
            return actor
        out = actor.clone()
        available = len(self._obs_history)
        for delay in range(1, min(self._delay_hi, available - 1) + 1):
            mask = self._obs_delay == delay
            if bool(mask.any()):
                past = self._obs_history[available - 1 - delay]
                out[mask] = past[mask]
        return out

    # -- reset -------------------------------------------------------------

    def _zero_object_velocity(self, env_ids: Optional[torch.Tensor]) -> None:
        """Clear the object's velocity on reset.

        A cylinder respawned mid-flight keeps its momentum, which shows up as
        phantom "disturbance" on the first macro step of the next episode.
        """
        n = self.num_envs if env_ids is None else int(env_ids.numel())
        zeros = torch.zeros((n, 3), device=self.device, dtype=torch.float32)
        for setter_name in ("set_vel", "set_ang"):
            setter = getattr(self.scene.object, setter_name, None)
            if not callable(setter):
                continue
            try:
                if env_ids is None:
                    setter(zeros)
                else:
                    setter(zeros, envs_idx=env_ids)
            except Exception:
                # Older Genesis builds expose different signatures here; a
                # missing reset is a small physics artefact, not worth aborting.
                pass

    def _reset_idx(self, env_ids: Optional[torch.Tensor]) -> None:
        n = self.num_envs if env_ids is None else int(env_ids.numel())
        if n == 0:
            return
        dr = self.cfg.domain_randomisation
        obj = self.cfg.object

        def rnd(shape, lo, hi):
            if hi <= lo:
                return torch.full(shape, float(lo), device=self.device, dtype=torch.float32)
            u = torch.rand(shape, device=self.device, generator=self._generator)
            return lo + u * (hi - lo)

        if dr.enable:
            radius = rnd((n,), *dr.object_radius_m)
            mass = rnd((n,), *dr.object_mass_kg)
            jitter = torch.stack(
                [
                    rnd((n,), -j, j) if j > 0 else torch.zeros(n, device=self.device)
                    for j in dr.object_pos_jitter_m
                ],
                dim=-1,
            )
            yaw = rnd((n,), -dr.object_yaw_jitter_rad, dr.object_yaw_jitter_rad)
        else:
            radius = torch.full((n,), float(obj.radius_m), device=self.device)
            mass = torch.full((n,), float(obj.mass_kg), device=self.device)
            jitter = torch.zeros((n, 3), device=self.device)
            yaw = torch.zeros((n,), device=self.device)

        base = torch.tensor(
            [float(v) for v in self.cfg.object_center()],
            device=self.device,
            dtype=torch.float32,
        )
        pos = base.unsqueeze(0) + jitter
        half = torch.cos(yaw * 0.5)
        quat = torch.stack(
            (half, torch.zeros_like(half), torch.zeros_like(half), torch.sin(yaw * 0.5)),
            dim=-1,
        )
        zeros3 = torch.zeros((n, 3), device=self.device)

        if env_ids is None:
            self._object_radius[:] = radius
            self._object_height[:] = float(obj.height_m)
            self._object_mass[:] = mass
            self._object_pos0[:] = pos
            self.scene.object.set_pos(pos)
            self.scene.object.set_quat(quat)
            self._zero_object_velocity(None)
            self._object_axis0[:] = quat_to_axis(quat)
            self.episode_length_buf[:] = 0
            self._load_proxy[:] = 0.0
            self._last_joint_cmd[:] = 0.0
        else:
            self._object_radius[env_ids] = radius
            self._object_height[env_ids] = float(obj.height_m)
            self._object_mass[env_ids] = mass
            self._object_pos0[env_ids] = pos
            self.scene.object.set_pos(pos, envs_idx=env_ids)
            self.scene.object.set_quat(quat, envs_idx=env_ids)
            self._zero_object_velocity(env_ids)
            self._object_axis0[env_ids] = quat_to_axis(quat)
            self.episode_length_buf[env_ids] = 0
            self._load_proxy[env_ids] = 0.0
            self._last_joint_cmd[env_ids] = 0.0

        self.mapper.reset(env_ids)
        if env_ids is None:
            self._waypoint_from[:] = self.mapper.waypoint
        else:
            self._waypoint_from[env_ids] = self.mapper.waypoint[env_ids]
        self.beta.reset(env_ids)
        if self.lift is not None:
            self.lift.reset(env_ids)

        home = self.mapper.joint_targets()
        if env_ids is None:
            self.scene.robot.set_dofs_position(home, dofs_idx_local=self._arm_dofs)
        else:
            self.scene.robot.set_dofs_position(
                home[env_ids], dofs_idx_local=self._arm_dofs, envs_idx=env_ids
            )

        delay = torch.randint(
            self._delay_lo,
            self._delay_hi + 1,
            (n,),
            device=self.device,
            generator=self._generator,
        )
        if env_ids is None:
            self._obs_delay[:] = delay
        else:
            self._obs_delay[env_ids] = delay

        state = self._read_state()
        self.rewards.reset(env_ids, phi0=state["phi"], dist0=state["surface_dist"])

    # -- logging -----------------------------------------------------------

    def _logging_extras(
        self,
        reward_out: Any,
        contact: Any,
        state: dict[str, torch.Tensor],
        timeout: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        log: dict[str, torch.Tensor] = {}
        for name, value in reward_out.terms.items():
            log[f"reward/{name}"] = value.mean()
        for name, value in self.rewards.episode_sums().items():
            log[f"episode_sum/{name}"] = value.mean()
        log["wrap/phi_rad"] = state["phi"].mean()
        log["wrap/phi_max_rad"] = state["phi"].max()
        log["wrap/target_rad"] = torch.tensor(
            float(self.cfg.success.coverage_target_rad), device=self.device
        )
        log["wrap/surface_dist_m"] = state["surface_dist"].mean()
        # Logged whether or not it gates success: a wrap that reaches the
        # coverage target while `caged` stays at zero is not holding anything.
        log["wrap/caged"] = state["caged"].to(torch.float32).mean()
        log["wrap/gap_rad"] = state["gap_rad"].mean()
        log["wrap/gap_width_m"] = state["gap_width_m"].mean()
        log["object/displacement_m"] = state["displacement"].mean()
        log["object/tilt_rad"] = state["tilt"].mean()
        log["contact/object_touch"] = contact.object_touch.to(torch.float32).mean()
        log["contact/non_target"] = contact.non_target_collision.to(torch.float32).mean()
        log["contact/floor"] = contact.floor_touch.to(torch.float32).mean()
        log["contact/go2"] = contact.go2_touch.to(torch.float32).mean()
        log["contact/self"] = contact.self_touch.to(torch.float32).mean()
        # A saturated contact buffer means readings may be incomplete; surface
        # it rather than trusting a silently truncated collision check.
        log["contact/buffer_overflow"] = contact.overflow.to(torch.float32).mean()
        log["term/collision"] = reward_out.termination_reason["collision"].to(torch.float32).mean()
        log["term/topple"] = reward_out.termination_reason["topple"].to(torch.float32).mean()
        log["term/success"] = reward_out.termination_reason["success"].to(torch.float32).mean()
        log["term/timeout"] = timeout.to(torch.float32).mean()
        return log

    def _tally(self, reward_out: Any, timeout: torch.Tensor, dones: torch.Tensor) -> None:
        reasons = reward_out.termination_reason
        self._episode_count += int(dones.sum())
        self._success_count += int(reasons["success"].sum())
        self._failure_counts["collision"] += int(reasons["collision"].sum())
        self._failure_counts["topple"] += int(reasons["topple"].sum())
        if self.lift is not None:
            retention = self.lift.finished & (~self.lift.passed)
            self._failure_counts["retention"] += int((retention & dones).sum())
        self._failure_counts["timeout"] += int(
            (timeout & ~reward_out.terminate).sum()
        )

    def statistics(self) -> dict[str, float]:
        episodes = max(int(self._episode_count.item()), 1)
        stats = {
            "episodes": float(self._episode_count.item()),
            "success_rate": float(self._success_count.item()) / episodes,
        }
        for mode, count in self._failure_counts.items():
            stats[f"failure/{mode}"] = float(count.item()) / episodes
        return stats

    def metadata(self) -> dict[str, Any]:
        """Run-reproduction metadata, including the beta provenance flag."""
        return {
            "scene": self.scene.describe(),
            "beta": self.beta.describe(),
            "obs_dims": {"policy": self.obs_spec.policy, "privileged": self.obs_spec.privileged},
            "success_criterion": self.cfg.success.criterion,
            "coverage_target_deg": math.degrees(self.cfg.success.coverage_target_rad),
            "curriculum_stage": int(self.cfg.curriculum.stage),
            "config": to_dict(self.cfg),
        }
