"""Actor/critic observation assembly and per-environment delay history.

The builder reads physical state from its owning environment; noise and delay
never enter the privileged critic observations.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Optional, TYPE_CHECKING

import torch
from tensordict import TensorDict

from .coverage import quat_to_axis

if TYPE_CHECKING:
    from .wrap_env import WrapGraspEnv


@dataclass
class ObsSpec:
    """Widths of the two observation groups, derived from the config toggles."""

    policy: int
    privileged: int


class ObservationBuilder:
    def __init__(self, env: WrapGraspEnv) -> None:
        self.env = env
        delay_lo, delay_hi = env.cfg.observation.actor.delay_steps
        self._delay_lo, self._delay_hi = int(delay_lo), int(delay_hi)
        self._obs_history: deque[torch.Tensor] = deque(
            maxlen=max(self._delay_hi, 0) + 1
        )
        self._obs_delay = torch.zeros(env.num_envs, device=env.device, dtype=torch.long)

    def reset_delay(self, env_ids: Optional[torch.Tensor], n: int) -> None:
        delay = torch.randint(
            self._delay_lo,
            self._delay_hi + 1,
            (n,),
            device=self.env.device,
            generator=self.env._generator,
        )
        if env_ids is None:
            self._obs_delay[:] = delay
        else:
            self._obs_delay[env_ids] = delay

    def specification(self) -> ObsSpec:
        actor_cfg = self.env.cfg.observation.actor
        policy = 0
        if actor_cfg.include_joint_estimate:
            policy += 4
        if actor_cfg.include_object_geometry:
            policy += 7  # radius, height, pos(3), lean x/y
        if actor_cfg.include_load_proxy:
            policy += 4
        if actor_cfg.include_step_index:
            policy += 1

        critic_cfg = self.env.cfg.observation.critic_privileged
        priv = 0
        if critic_cfg.include_true_joint_state:
            priv += 2 * len(self.env._arm_dofs)
        if critic_cfg.include_contact_forces:
            priv += 1 + len(self.env.scene.links.arm) + 4
        if critic_cfg.include_true_object_pose:
            priv += 7
        if critic_cfg.include_coverage:
            priv += 2
        return ObsSpec(policy=policy, privileged=priv)

    def _noise(self, shape: tuple[int, ...], sigma: float) -> torch.Tensor:
        if sigma <= 0.0:
            return torch.zeros(shape, device=self.env.device, dtype=torch.float32)
        return torch.randn(
            shape, device=self.env.device, dtype=torch.float32, generator=self.env._generator
        ) * float(sigma)

    def _actor_observation(self) -> torch.Tensor:
        cfg = self.env.cfg.observation.actor
        noise = cfg.noise
        parts: list[torch.Tensor] = []

        realised = self.env.scene.robot.get_dofs_position(dofs_idx_local=self.env._arm_dofs)
        if cfg.include_joint_estimate:
            bend_est = self.env.beta.estimate(
                realised[:, self.env._bend_slice], load_kg=self.env._object_mass
            )
            seg = int(self.env.cfg.arm.n_seg)
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
            told = str(getattr(cfg, "object_pose_source", "measured")).strip().lower()
            if told not in {"measured", "told"}:
                raise ValueError(
                    f"observation.actor.object_pose_source: {told!r} is not "
                    "'measured' or 'told'"
                )
            if told == "told":
                # What the robot is handed: the pose written into its config,
                # every step, however far the object has actually drifted from
                # it.  The scene still randomises the real one, so the policy
                # has to work from a number that is only approximately true.
                pos = torch.tensor(
                    [float(v) for v in self.env.cfg.object_center()],
                    device=self.env.device, dtype=torch.float32,
                ).unsqueeze(0).expand(self.env.num_envs, 3)
                lean = torch.zeros((self.env.num_envs, 2), device=self.env.device)
            else:
                pos = self.env.scene.object.get_pos() + self._noise(
                    (self.env.num_envs, 3), noise.object_pos_m
                )
                axis = quat_to_axis(self.env.scene.object.get_quat())
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
                lean = axis[:, :2] + self._noise(
                    (self.env.num_envs, 2), noise.object_rot_rad
                )
            parts.append(
                torch.cat(
                    (
                        self.env._object_radius.unsqueeze(-1),
                        self.env._object_height.unsqueeze(-1),
                        pos,
                        lean,
                    ),
                    dim=-1,
                )
            )

        if cfg.include_load_proxy:
            parts.append(
                self.env._load_proxy + self._noise(self.env._load_proxy.shape, noise.load_proxy)
            )

        if cfg.include_step_index:
            frac = self.env.episode_length_buf.to(torch.float32) / max(
                self.env.max_episode_length, 1
            )
            parts.append(frac.unsqueeze(-1))
        return torch.cat(parts, dim=-1)

    def _privileged_observation(
        self, state: dict[str, torch.Tensor], contact: Any
    ) -> torch.Tensor:
        cfg = self.env.cfg.observation.critic_privileged
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

    def sanitise_recovered(self, recovered: torch.Tensor) -> None:
        """Clear nan left over from a diverged env's own step.

        Resetting the env puts its joints and object back, but values carried
        through the step -- contact accumulations, displacement, tilt -- can
        still be nan, and rsl_rl checks the observation before it ever looks at
        `dones`: a run died with "observation group 'privileged' contains NaN"
        one step after a recovery.  Those episodes are already terminated, so
        zeros cost nothing.

        Only the envs that diverged are touched.  A nan anywhere else is a
        different fault and should still surface rather than be papered over.
        """
        if (
            not isinstance(recovered, torch.Tensor)
            or recovered.ndim != 1
            or recovered.numel() != self.env.num_envs
            or recovered.dtype is not torch.bool
        ):
            raise TypeError(
                "recovered env mask must be a 1-D boolean tensor with one entry "
                f"per env (got {type(recovered).__name__}, "
                f"shape {getattr(recovered, 'shape', None)}, "
                f"dtype {getattr(recovered, 'dtype', None)})"
            )
        recovered = recovered.to(self.env.device)
        if not bool(recovered.any()):
            return
        ids = recovered.nonzero(as_tuple=False).flatten()
        for key in ("policy", "privileged"):
            tensor = self.env._obs[key]
            rows = tensor[ids]
            bad = ~torch.isfinite(rows)
            if not bool(bad.any()):
                continue
            rows = torch.nan_to_num(rows, nan=0.0, posinf=0.0, neginf=0.0)
            tensor[ids] = rows
            print(
                f"[env] cleared {int(bad.sum())} non-finite {key} value(s) in "
                f"{int(recovered.sum())} recovered env(s)",
                flush=True,
            )

    def build(
        self,
        state: Optional[dict[str, torch.Tensor]] = None,
        contact: Optional[Any] = None,
    ) -> TensorDict:
        if state is None:
            state = self.env._read_state()
        if contact is None:
            contact = self.env.contacts.result()
        actor = self._actor_observation()
        actor = self._apply_obs_delay(actor)
        privileged = self._privileged_observation(state, contact)
        return TensorDict(
            {"policy": actor, "privileged": privileged},
            batch_size=[self.env.num_envs],
            device=self.env.device,
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
