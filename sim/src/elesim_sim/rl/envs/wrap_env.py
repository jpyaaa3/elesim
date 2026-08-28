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
from .lift_test import GeometricCriterion, LiftObservation, LiftTest, TugTest

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
        self._home_waypoint = torch.tensor(
            [float(v) for v in self.cfg.arm.home_waypoint],
            device=self.device,
            dtype=torch.float32,
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
        self_contact = self.cfg.reward.self_contact
        self.classifier = ContactClassifier(
            self.scene, structural_prefixes=self_contact.structural_prefixes
        )
        self.contacts = ContactAggregator(
            self.classifier,
            n_envs=self.num_envs,
            self_contact_all_is_failure=self_contact.all_is_failure,
            self_contact_terminates=self_contact.terminates,
        )

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
        self.tug: Optional[TugTest] = (
            TugTest(self.cfg.success, n_envs=self.num_envs, device=self.device)
            if self.cfg.success.criterion == "tug"
            else None
        )
        # Whichever scripted test is active, or None for the geometric gate.
        # Both freeze the policy for the envs they have taken over and both
        # report retention, so the shared paths go through this.
        self.script = self.lift if self.lift is not None else self.tug
        if self.tug is not None:
            # External forces go through the solver, not the entity: the
            # entity API exposes joint forces only, and the object is a free
            # body with no joints.  Gravity is read rather than assumed so the
            # tug force stays a multiple of the object's weight *in this
            # scene*.
            self._rigid_solver = self.scene.scene.sim.rigid_solver
            self._object_link_ids = [
                int(link.idx) for link in self.scene.object.links
            ]
            self._gravity_mag = float(
                torch.tensor(self.scene.scene.sim.gravity).norm()
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
        self._object_base0 = z(self.num_envs, 3)
        start = self.cfg.start_pose
        self._start_near = torch.tensor(
            [float(v) for v in start.near_waypoint],
            device=self.device,
            dtype=torch.float32,
        )
        self._start_t_lo = float(start.t_range[0])
        self._start_t_hi = float(start.t_range[1])
        self._start_window_n = 0
        self._start_window_ok = 0
        self._load_proxy = z(self.num_envs, self.num_actions)
        self._last_joint_cmd = z(self.num_envs, len(self._arm_dofs))
        self._waypoint_from = z(self.num_envs, self.num_actions)
        #: Set by eval to pin one object condition across every env, so a batch
        #: is just a faster way to collect episodes of the *same* condition
        #: rather than a mixture the per-condition table could not separate.
        self._eval_override: Optional[dict[str, float]] = None
        self._last_reasons: dict[str, torch.Tensor] = {}
        #: Optional callback invoked after every physics substep, as
        #: ``monitor(env, substep_index)``.  Used by the divergence diagnostic
        #: to sample state at the resolution the solver actually fails at; a
        #: macro-step-level look is far too coarse to catch it.
        self.substep_monitor: Optional[Any] = None
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
        self._settle_at_home_and_baseline_contacts()
        self._obs = self._build_observations()

    def move_support_to(self, dx_m: float, dy_m: float) -> None:
        """Shift the support so it stays under an offset object.

        An eval condition that moved the object without the support would be
        measuring free fall, not placement: the cylinder is a free body and its
        post is only 40 mm across.
        """
        if self.scene.support is None:
            return
        cx, cy = self.cfg.support.center_xy
        top = float(self.cfg.support.height_m)
        target = torch.tensor(
            [[float(cx) + float(dx_m), float(cy) + float(dy_m), top * 0.5]],
            device=self.device,
            dtype=torch.float32,
        ).expand(self.num_envs, 3)
        self.scene.support.set_pos(target)

    @property
    def start_pose_range(self) -> tuple[float, float]:
        """Where the reverse curriculum has got to, as (t_lo, t_hi)."""
        return (self._start_t_lo, self._start_t_hi)

    @start_pose_range.setter
    def start_pose_range(self, value: tuple[float, float]) -> None:
        lo, hi = (float(v) for v in value)
        self._start_t_lo = max(0.0, min(1.0, lo))
        self._start_t_hi = max(self._start_t_lo, min(1.0, hi))
        self._start_window_n = 0
        self._start_window_ok = 0

    def freeze_start_pose_at_home(self) -> None:
        """Pin every episode to Home and stop the curriculum advancing.

        Evaluation measures the deployed task, which starts at Home.  The
        curriculum position lives on the env rather than in the checkpoint, so
        a freshly built env would otherwise reset to the configured `t_range`
        and evaluate the policy three steps from the goal.
        """
        self._start_t_lo = 0.0
        self._start_t_hi = 0.0
        self._start_window_n = 0
        self._start_window_ok = 0

    def _apply_start_pose(self, env_ids: Optional[torch.Tensor], n: int) -> None:
        """Interpolate the reset waypoint from Home towards the near-goal pose.

        Per env rather than per batch, so a single rollout spans a spread of
        difficulties instead of all envs sitting at one point of the curriculum.
        """
        t = torch.rand(n, 1, device=self.device, generator=self._generator)
        t = self._start_t_lo + t * (self._start_t_hi - self._start_t_lo)
        home = self._home_waypoint.to(self.device, torch.float32).reshape(1, 4)
        target = home + t * (self._start_near.reshape(1, 4) - home)
        if env_ids is None:
            self.mapper.waypoint[:] = target
        else:
            self.mapper.waypoint[env_ids] = target
        # The cap is roll-independent but still has to hold at reset.
        self.mapper.waypoint = self.mapper._project_curl(self.mapper.waypoint)

    def _note_start_pose_outcome(self, dones: torch.Tensor, success: torch.Tensor) -> None:
        """Walk the start range back towards Home once the policy can finish.

        Keyed on the success rate over a window of finished episodes, so the
        curriculum only advances on evidence rather than on a step count.
        """
        cfg = self.cfg.start_pose
        if not cfg.enable or cfg.advance_at is None:
            return
        finished = int(dones.sum())
        if finished == 0:
            return
        self._start_window_n += finished
        self._start_window_ok += int((dones & success).sum())
        if self._start_window_n < int(cfg.window):
            return
        rate = self._start_window_ok / max(self._start_window_n, 1)
        self._start_window_n = 0
        self._start_window_ok = 0
        if rate < float(cfg.advance_at) or self._start_t_hi <= 0.0:
            return
        step = float(cfg.step)
        self._start_t_hi = max(0.0, self._start_t_hi - step)
        self._start_t_lo = max(0.0, self._start_t_lo - step)

    def _settle_at_home_and_baseline_contacts(self) -> None:
        """Let the arm settle at Home, then baseline the contacts it rests on.

        The reset writes joint positions directly, so the first physics steps
        resolve whatever overlap the Home pose starts with.  Only once that has
        settled is the contact set a fair description of "resting at Home"
        rather than of the reset transient.  Done once at construction: the
        excluded set is a property of the pose, not of an individual episode.
        """
        targets = self.mapper.joint_targets()
        for _ in range(int(self.cfg.macro_step.substeps)):
            self.scene.robot.control_dofs_position(
                targets, dofs_idx_local=self._arm_dofs
            )
            self.scene.step()
        excluded = self.classifier.set_baseline_from_current_contacts()
        if excluded:
            print(
                f"[wrap-env] excluding {excluded} link pair(s) already in "
                f"contact at the Home pose"
            )
        self.contacts.reset()

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

        contact = self.contacts.result()
        state = self._read_state(contact)

        success = self._evaluate_success(state, contact)
        reward_out = self.rewards.step(
            RewardInputs(
                phi=state["phi"],
                enclosure=state["enclosure"],
                surface_dist=self._shaping_distance(state),
                object_touch=contact.object_touch,
                non_target_collision=contact.non_target_collision,
                terminating_collision=contact.terminating_collision,
                object_displacement=state["displacement"],
                object_tilt=state["tilt"],
                success=success,
            )
        )

        self.episode_length_buf += 1
        timeout = self.episode_length_buf >= self.max_episode_length
        dones = reward_out.terminate | timeout

        self._last_reasons = reward_out.termination_reason
        extras: dict[str, Any] = {
            "log": self._logging_extras(reward_out, contact, state, timeout),
            "time_outs": timeout,
            # Exposed so evaluation can classify a finished episode; the done
            # flag alone cannot tell a collision from a dropped object.
            "termination_reason": reward_out.termination_reason,
        }
        self._tally(reward_out, timeout, dones)
        self._note_start_pose_outcome(dones, reward_out.termination_reason["success"])

        done_ids = dones.nonzero(as_tuple=False).flatten()
        if done_ids.numel():
            self._reset_idx(done_ids)
        self._obs = self._build_observations()
        return self._obs, reward_out.total, dones, extras

    # -- action and simulation --------------------------------------------

    def _apply_action(self, actions: torch.Tensor) -> None:
        """Advance the waypoint, then override roll for envs under lift script."""
        follows_policy = (
            self.script.follows_policy
            if self.script is not None
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
            if self.tug is not None:
                self._apply_tug_force()
            self.scene.step()
            self.contacts.accumulate()
            if self.substep_monitor is not None:
                self.substep_monitor(self, i)

            if self.script is not None:
                self.script.advance(self._lift_observation())

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
        # Clamped to the joint's own range.  This path writes the roll column of
        # the joint target directly, so it bypasses the waypoint clamp that
        # holds every policy action inside the limits -- a `roll_target_rad`
        # outside them would be commanded as given.
        lo, hi = self.cfg.arm.limits.roll_rad
        roll = self.lift.roll_command.clamp(float(lo), float(hi))
        roll = roll * float(self.cfg.arm.roll_axis_sign)
        out[:, 1] = torch.where(scripted, roll, out[:, 1])
        return out

    def _apply_tug_force(self) -> None:
        """Push the object for the envs under test; zero force for the rest.

        Applied every substep because Genesis clears external forces as it
        steps, and to all envs at once rather than to a subset, so the call
        count does not depend on how many envs happen to be under test.
        """
        assert self.tug is not None
        force = self.tug.external_force()
        if not bool((force != 0).any()):
            return
        self._rigid_solver.apply_links_external_force(
            force.unsqueeze(1), self._object_link_ids
        )

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

    def _shaping_distance(self, state: dict[str, torch.Tensor]) -> torch.Tensor:
        """Distance approach_shaping is measured against.

        The segment-2 midpoint is a bad proxy for "am I getting closer".  The
        wrap needs roll near +/-90 deg to put the bend plane horizontal, and
        that rotation swings the midpoint *away* from the object before the
        curl brings it back: measured along a scripted wrap, its distance runs
        236 -> 250 mm over the six macro steps of roll while the arm is doing
        exactly the right thing.  Shaping on it pays -0.5 for the manoeuvre the
        task cannot be done without, which makes standing still the better
        move, and two runs to iteration 300 came back with every episode
        classified `no_reach`.

        The nearest arm link falls monotonically over the same trajectory --
        97 -> 78 mm through the roll, then 10 -> 0.9 mm as the coil closes -- so
        it rewards the roll instead of punishing it.
        """
        if self.cfg.reward.approach_shaping_source == "anchor_link":
            return state["surface_dist"]
        return state["min_surface_dist"]

    def _read_state(self, contact: Optional[Any] = None) -> dict[str, torch.Tensor]:
        obj_pos = self.scene.object.get_pos()
        obj_quat = self.scene.object.get_quat()
        link_pos = self.scene.robot.get_links_pos()[:, self._arm_link_ids, :]
        # Contact-based coverage needs the contact set for this macro step, so
        # the aggregate is passed in rather than re-read: it is accumulated
        # across the substep window, and a coverage computed from the settled
        # instant alone would miss links that touched during the motion.
        mask = None
        rule = "span"
        source = self.cfg.reward.coverage.source
        if source != "proximity":
            aggregate = contact if contact is not None else self.contacts.result()
            mask = aggregate.object_link_hits
            rule = "span" if source == "contact_span" else "strict"
        cov = self.coverage.measure(
            link_pos,
            obj_pos,
            obj_quat,
            radius_m=self._object_radius,
            height_m=self._object_height,
            link_radius_m=self.cfg.reward.coverage.link_radius_m,
            contact_mask=mask,
            contact_rule=rule,
        )
        anchor = self.scene.robot.get_links_pos()[:, self._anchor_link, :]
        radial = (anchor - obj_pos)
        axis = quat_to_axis(obj_quat)
        along = (radial * axis).sum(dim=-1, keepdim=True)
        planar = radial - along * axis
        anchor_surface = planar.norm(dim=-1) - self._object_radius

        # Displacement is measured at the end the object stands on, not at its
        # centre.  For a 1.1 m cylinder the centre sits 0.55 m up, so tipping it
        # by an angle theta moves the centre by 0.55*sin(theta): the 60 mm
        # displacement gate then fires at 6.3 deg of tilt, 5.5x stricter than
        # the 34 deg tilt gate that was meant to catch tipping.  Any contact
        # firm enough to wrap tips the object past that -- a scripted wrap that
        # succeeds passes through 3 deg -- so `topple` fired on every episode
        # that touched the object, and the policy learned not to touch it: at
        # iteration 350 the eval reported 13817 of 13824 episodes as `no_reach`
        # with zero collisions and zero topples.
        #
        # Measured at the base, the two failures stay separate: sliding moves
        # the base, tipping does not.
        base = obj_pos - axis * (self._object_height * 0.5).unsqueeze(-1)
        displacement = (base - self._object_base0).norm(dim=-1)
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
            "gap_bearing": cov.gap_bearing_rad,
            "enclosure_raw": cov.enclosure_rad,
            "plane_alignment": cov.plane_alignment,
            # What the reward sees.  Bearing enclosure on its own credits a
            # coil curled beside the object in a vertical plane, since
            # projecting to the horizontal throws away the difference; the
            # alignment factor is what distinguishes them.
            "enclosure": cov.enclosure_rad * cov.plane_alignment,
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
        if self.tug is not None:
            self.tug.arm(
                state["phi"],
                self._lift_observation(),
                gap_bearing_rad=state["gap_bearing"],
                weight_n=self._object_mass * self._gravity_mag,
            )
            return self.tug.passed
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
            policy += 7  # radius, height, pos(3), lean x/y
        if actor_cfg.include_load_proxy:
            policy += 4
        if actor_cfg.include_step_index:
            policy += 1

        critic_cfg = self.cfg.observation.critic_privileged
        priv = 0
        if critic_cfg.include_true_joint_state:
            priv += 2 * len(self._arm_dofs)
        if critic_cfg.include_contact_forces:
            priv += 1 + len(self.scene.links.arm) + 4
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
            # The object's axis tilted into the horizontal plane: which way it
            # is leaning *and* how far, in two channels.
            #
            # This used to be sin/cos of `atan2(axis.y, axis.x)`, which named
            # itself yaw and was not.  A cylinder is symmetric about its own
            # axis, so rotating it changes neither the geometry nor that
            # quantity -- measured, 0, 20, 90 and 180 deg of yaw all read 0.00.
            # What the atan2 actually returned was the *bearing* of a lean,
            # carrying no magnitude: 1 deg and 20 deg of tilt both read -90.
            # And upright, the axis is (0, 0, 1), so it was atan2(0, 0) -- an
            # undefined direction that the observation noise then dithered.
            lean = axis[:, :2] + self._noise((self.num_envs, 2), noise.object_rot_rad)
            parts.append(
                torch.cat(
                    (
                        self._object_radius.unsqueeze(-1),
                        self._object_height.unsqueeze(-1),
                        pos,
                        lean,
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
                        contact.support_touch.to(torch.float32),
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

        override = self._eval_override
        if override is not None:
            radius = torch.full((n,), float(override["radius_m"]), device=self.device)
            mass = torch.full((n,), float(obj.mass_kg), device=self.device)
            jitter = torch.zeros((n, 3), device=self.device)
            jitter[:, 0] = float(override.get("dx_m", 0.0))
            jitter[:, 1] = float(override.get("dy_m", 0.0))
            yaw = torch.full((n,), float(override["yaw_rad"]), device=self.device)
        elif dr.enable:
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

        centre = torch.tensor(
            [float(v) for v in self.cfg.object_center()],
            device=self.device,
            dtype=torch.float32,
        )
        pos = centre.unsqueeze(0) + jitter
        half = torch.cos(yaw * 0.5)
        quat = torch.stack(
            (half, torch.zeros_like(half), torch.zeros_like(half), torch.sin(yaw * 0.5)),
            dim=-1,
        )
        zeros3 = torch.zeros((n, 3), device=self.device)

        # Genesis fixes each morph's geometry at build time, so the radius an
        # env gets is decided by *which cylinder it is handed*, not by a number
        # written at reset.  Snap the sampled radius to the nearest one the
        # scene actually built and use that entity; believing a size the object
        # does not have would corrupt every surface distance derived from it.
        built = torch.tensor(
            self.scene.object.radii, device=self.device, dtype=torch.float32
        )
        choice = (radius.unsqueeze(-1) - built.unsqueeze(0)).abs().argmin(dim=-1)
        radius = built[choice]
        if env_ids is None:
            self.scene.object.assignment[:] = choice
        else:
            self.scene.object.assignment[env_ids] = choice
        if env_ids is None:
            self._object_radius[:] = radius
            self._object_height[:] = float(obj.height_m)
            self._object_mass[:] = mass
            self._object_pos0[:] = pos
            self._object_base0[:] = pos - quat_to_axis(quat) * (
                float(obj.height_m) * 0.5
            )
            self.scene.object.set_pos(pos)
            self.scene.object.set_quat(quat)
            self._zero_object_velocity(None)
            self._object_axis0[:] = quat_to_axis(quat)
            self.scene.object.park_unassigned(None)
            self.episode_length_buf[:] = 0
            self._load_proxy[:] = 0.0
            self._last_joint_cmd[:] = 0.0
        else:
            self._object_radius[env_ids] = radius
            self._object_height[env_ids] = float(obj.height_m)
            self._object_mass[env_ids] = mass
            self._object_pos0[env_ids] = pos
            self._object_base0[env_ids] = pos - quat_to_axis(quat) * (
                float(obj.height_m) * 0.5
            )
            self.scene.object.set_pos(pos, envs_idx=env_ids)
            self.scene.object.set_quat(quat, envs_idx=env_ids)
            self._zero_object_velocity(env_ids)
            self._object_axis0[env_ids] = quat_to_axis(quat)
            self.scene.object.park_unassigned(env_ids)
            self.episode_length_buf[env_ids] = 0
            self._load_proxy[env_ids] = 0.0
            self._last_joint_cmd[env_ids] = 0.0

        # Reset to the configured Home pose, not to an implicit all-zeros that
        # happens to mean "arm straight out" -- a configuration the real arm
        # never starts from.
        #
        # Or, under the reverse curriculum, part-way towards a pose from which
        # the wrap is a few steps away.  Success had never once been paid: over
        # roughly 50,000 episodes to iteration 100 the bonus never fired and the
        # wrap angle stayed at zero, so the objective contributed nothing to the
        # gradient and the policy was shaped entirely by the dense proxies, each
        # of which has a degenerate optimum two or three steps from Home.
        # Starting near the goal puts the bonus inside reach of exploration; the
        # range then walks back towards Home as the success rate allows.
        self.mapper.reset(env_ids, home=self._home_waypoint)
        if self.cfg.start_pose.enable:
            self._apply_start_pose(env_ids, n)
        if env_ids is None:
            self._waypoint_from[:] = self.mapper.waypoint
        else:
            self._waypoint_from[env_ids] = self.mapper.waypoint[env_ids]
        self.beta.reset(env_ids)
        if self.script is not None:
            self.script.reset(env_ids)

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
        self.rewards.reset(
            env_ids,
            phi0=state["phi"],
            dist0=self._shaping_distance(state),
            enclosure0=state["enclosure"],
        )

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
        log["wrap/min_surface_dist_m"] = state["min_surface_dist"].mean()
        log["wrap/enclosure_rad"] = state["enclosure_raw"].mean()
        log["wrap/plane_alignment"] = state["plane_alignment"].mean()
        log["curriculum/start_t_lo"] = torch.tensor(self._start_t_lo, device=self.device)
        log["curriculum/start_t_hi"] = torch.tensor(self._start_t_hi, device=self.device)
        log["wrap/enclosure_effective_rad"] = state["enclosure"].mean()
        # Waypoint usage per DoF.  The wrap needs roll near +/-90 deg to put the
        # bend plane horizontal, so a policy whose roll stays near its Home zero
        # cannot be wrapping whatever else the other terms say.
        wp = self.mapper.waypoint
        log["waypoint/linear_m"] = wp[:, 0].mean()
        log["waypoint/roll_rad"] = wp[:, 1].mean()
        log["waypoint/roll_abs_rad"] = wp[:, 1].abs().mean()
        log["waypoint/roll_abs_max_rad"] = wp[:, 1].abs().max()
        log["waypoint/theta1_rad"] = wp[:, 2].mean()
        log["waypoint/theta2_rad"] = wp[:, 3].mean()
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
        log["contact/support"] = contact.support_touch.to(torch.float32).mean()
        log["contact/go2"] = contact.go2_touch.to(torch.float32).mean()
        log["contact/self"] = contact.self_touch.to(torch.float32).mean()
        log["contact/self_structural"] = (
            contact.self_structural_touch.to(torch.float32).mean()
        )
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
        if self.script is not None:
            retention = self.script.finished & (~self.script.passed)
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
