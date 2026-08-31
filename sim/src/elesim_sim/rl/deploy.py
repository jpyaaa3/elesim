"""Run an exported policy without a simulator.

Everything here has to agree with the environment exactly, because a policy is
a function of a vector whose meaning is a convention: the observation order, the
rate limit the action is scaled by, the clamps the waypoint passes through, and
the timing of a macro step.  `ArmWaypointMapper` is reused rather than
reimplemented so the clamps cannot drift apart -- it needs torch and the arm
config, and nothing from Genesis.

What is *not* here, and has to come from the robot:

* the observations -- joint estimates from encoders and the arm's own residual
  compensator, the object's geometry from perception, load from motor current;
* the servo loop that follows the waypoint this returns;
* the lift roll-back, which `LiftScript` gives the command trajectory for.

One thing the simulator has that the robot does not: the wrap angle, computed
from contacts.  In the sim it is a floor under the lift request.  Here the
policy's request is the only gate, so a request made without a wrap will roll
the arm back on nothing.  A robot-side guard -- load, a camera check -- belongs
in front of `lift_requested`.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import torch


@dataclass(frozen=True)
class Interface:
    """The manifest `export` writes, as the few numbers the runner needs."""

    obs_dim: int
    action_dim: int
    rate_limit: tuple[float, float, float, float]
    home: tuple[float, float, float, float]
    macro_step_s: float
    substeps: int
    move_fraction: float
    lift_roll_target_rad: float
    lift_roll_rate_rad_per_substep: float
    lift_settle_substeps: int
    lift_hold_substeps: int
    max_steps: int

    @staticmethod
    def from_manifest(path: Path) -> "Interface":
        m = json.loads(Path(path).read_text(encoding="utf-8"))
        act = m["action"]["channels"]
        rate = (
            float(act[0]["scale_m"]), float(act[1]["scale_rad"]),
            float(act[2]["scale_rad"]), float(act[3]["scale_rad"]),
        )
        home = m["waypoint"]["home"]
        if home is None:
            raise ValueError(
                "manifest 에 home waypoint 가 없습니다: 설정의 home_preset 이 "
                "런타임에서 풀리므로, 내보낼 때 arm.home_waypoint 를 명시하세요"
            )
        t, lift = m["timing"], m["lift_script"]
        return Interface(
            obs_dim=int(m["observation"]["dim"]),
            action_dim=int(m["action"]["dim"]),
            rate_limit=rate,
            home=tuple(float(v) for v in home),  # type: ignore[arg-type]
            macro_step_s=float(t["macro_step_s"]),
            substeps=int(t["substeps"]),
            move_fraction=float(t["move_fraction"]),
            lift_roll_target_rad=float(lift["roll_target_rad"]),
            lift_roll_rate_rad_per_substep=float(lift["roll_rate_rad_per_substep"]),
            lift_settle_substeps=int(lift["settle_substeps"]),
            lift_hold_substeps=int(lift["hold_substeps"]),
            max_steps=int(t["max_steps"]),
        )


class LiftScript:
    """The scripted roll-back, as a command trajectory.

    Speed is the whole of it: measured on 32 environments, a 90 degree rotation
    taken in 0.31 s retains nothing and one taken in 1.05 s retains 72%.  A pole
    laid into the coil too fast arrives with enough momentum to keep rolling
    inside it.
    """

    def __init__(self, iface: Interface) -> None:
        self.iface = iface
        self._start = 0.0
        self._cmd = 0.0
        self._substep = 0
        self.phase = "idle"

    def start(self, roll_now: float) -> None:
        self._start = float(roll_now)
        self._cmd = float(roll_now)
        self._substep = 0
        self.phase = "rolling"

    @property
    def roll_command(self) -> float:
        return self._cmd

    @property
    def finished(self) -> bool:
        return self.phase == "done"

    def advance(self) -> float:
        """One substep.  Returns the roll angle to command."""
        i = self.iface
        if self.phase == "rolling":
            target = i.lift_roll_target_rad
            rate = i.lift_roll_rate_rad_per_substep
            step = math.copysign(rate, target - self._start) if target != self._start else 0.0
            nxt = self._cmd + step
            lo, hi = min(self._start, target), max(self._start, target)
            self._cmd = min(max(nxt, lo), hi)
            if abs(self._cmd - target) <= rate * 0.5 + 1e-9:
                self._cmd = target
                self.phase = "settling"
                self._substep = 0
        elif self.phase == "settling":
            self._substep += 1
            if self._substep >= i.lift_settle_substeps:
                self.phase = "holding"
                self._substep = 0
        elif self.phase == "holding":
            self._substep += 1
            if self._substep >= i.lift_hold_substeps:
                self.phase = "done"
        return self._cmd


def arm_config_from_manifest(path: Path):
    """Rebuild the arm config the waypoint mapper needs, from the manifest.

    So the robot needs the exported files and this module, not a copy of the
    training config: a config file that has to travel alongside is a config file
    that can be the wrong one.
    """
    from .configs.loader import ArmConfig, ArmLimits

    m = json.loads(Path(path).read_text(encoding="utf-8"))
    w, arm, sign = m["waypoint"], m["waypoint"]["arm"], m["waypoint"]["sign_conventions"]
    lim = w["limits"]
    cap = w["coupled_curl_cap"]
    return ArmConfig(
        linear_joint=arm["linear_joint"],
        roll_joint=arm["roll_joint"],
        bend_joints=tuple(arm["bend_joints"]),
        n_seg=int(arm["n_seg"]),
        linear_axis_sign=float(sign["linear_axis_sign"]),
        roll_axis_sign=float(sign["roll_axis_sign"]),
        bend_axis_sign=float(sign["bend_axis_sign"]),
        limits=ArmLimits(
            linear_m=tuple(lim["linear_m"]),
            roll_rad=tuple(lim["roll_rad"]),
            bend_per_node_rad=float(lim["theta_rad"][1]),
            curl_limit_per_node_rad=float(cap["cap_rad"]),
            theta1_curl_weight=float(cap["theta1_weight"]),
        ),
    )


class DeployedPolicy:
    """An exported policy plus the waypoint bookkeeping around it."""

    def __init__(
        self, policy_path: Path, manifest_path: Path, arm_cfg=None
    ) -> None:
        from .arm_kinematics import ArmWaypointMapper

        if arm_cfg is None:
            arm_cfg = arm_config_from_manifest(Path(manifest_path))
        self.iface = Interface.from_manifest(Path(manifest_path))
        self.policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
        self.mapper = ArmWaypointMapper(
            arm_cfg, n_envs=1, device=torch.device("cpu"),
            rate_limit=self.iface.rate_limit,
        )
        self.lift = LiftScript(self.iface)
        self._home = torch.tensor([self.iface.home], dtype=torch.float32)
        self.step_index = 0
        self.reset()

    def reset(self) -> None:
        self.mapper.reset(None, home=self._home.reshape(-1))
        self.step_index = 0
        self.lift = LiftScript(self.iface)

    @property
    def waypoint(self) -> tuple[float, float, float, float]:
        w = self.mapper.waypoint[0]
        return (float(w[0]), float(w[1]), float(w[2]), float(w[3]))

    def observation(
        self,
        *,
        joint_estimate: Sequence[float],
        object_geometry: Sequence[float],
        load_proxy: Sequence[float],
        progress: Optional[float] = None,
    ) -> torch.Tensor:
        """Assemble the observation in the order the network was trained on.

        Order is not negotiable and nothing checks it at runtime beyond the
        width, so the manifest's channel list is the reference.

        `progress` defaults to this runner's own step count.  It is settable
        because training delayed the whole observation by 0 to 2 macro steps as
        randomisation, so a caller replaying a sim rollout has to pass the value
        that came with the rest of the vector -- computing it here instead is
        what made an equivalence check drift while the actions themselves
        matched to 0.  On the robot there is no artificial delay and the default
        is right; a fresh observation is inside the trained distribution.
        """
        if progress is None:
            progress = self.step_index / max(self.iface.max_steps, 1)
        vec = list(joint_estimate) + list(object_geometry) + list(load_proxy) + [progress]
        if len(vec) != self.iface.obs_dim:
            raise ValueError(
                f"관측이 {len(vec)} 개인데 정책은 {self.iface.obs_dim} 개를 "
                f"기대합니다 (관절 4 + 물체 7 + 부하 4 + 진행률 1)"
            )
        return torch.tensor([vec], dtype=torch.float32)

    #: What to feed the load channels when the robot's units do not match the
    #: sim's.  The sim reports joint torque and the arm reports motor current in
    #: mA -- orders of magnitude apart -- and the observation normaliser is
    #: frozen at the training statistics, so it will not absorb the difference.
    #: Zero sits near the middle of the distribution the policy was trained on.
    ZERO_LOAD = (0.0, 0.0, 0.0, 0.0)

    def act(
        self,
        *,
        joint_estimate: Sequence[float],
        object_geometry: Sequence[float],
        load_proxy: Sequence[float] = ZERO_LOAD,
        progress: Optional[float] = None,
    ) -> tuple[tuple[float, float, float, float], bool]:
        """One macro step: the waypoint to drive to, and whether to lift."""
        obs = self.observation(
            joint_estimate=joint_estimate,
            object_geometry=object_geometry,
            load_proxy=load_proxy,
            progress=progress,
        )
        with torch.no_grad():
            action = self.policy(obs)
        self.mapper.apply_action(action[:, :4])
        lift = bool(action[0, 4] > 0.0) if self.iface.action_dim > 4 else False
        self.step_index += 1
        return self.waypoint, lift

    def substep_targets(
        self, previous: Sequence[float]
    ) -> list[tuple[float, float, float, float]]:
        """The per-substep waypoints for one macro step.

        The environment interpolates over the first `move_fraction` of the step
        and holds for the rest.  Commanding the new waypoint in one go is an
        impulse: the arm knocks the object away before it can close on it.
        """
        i = self.iface
        move = max(1, int(round(i.substeps * i.move_fraction)))
        start = torch.tensor(previous, dtype=torch.float32)
        end = torch.tensor(self.waypoint, dtype=torch.float32)
        out = []
        for k in range(i.substeps):
            alpha = min(1.0, float(k + 1) / move)
            w = start + (end - start) * alpha
            out.append((float(w[0]), float(w[1]), float(w[2]), float(w[3])))
        return out


def numpy_policy(npz_path):
    """Load `policy.npz` and return a callable, with no torch involved.

    For the Jetson: matching a torch build to aarch64 and a JetPack version is
    real work, and the network is 16 -> 256 -> 128 -> 64 -> 5 -- four matrix
    multiplies on a 0.4 s control step.  `export` checks this path against the
    TorchScript one on random inputs before writing either.

    ELU, because that is the activation the models were trained with; a ReLU
    here would be a silently different policy.
    """
    import numpy as np

    z = np.load(str(npz_path))
    mean = z["norm_mean"].astype(np.float32)
    std = z["norm_std"].astype(np.float32)
    eps = float(z["norm_eps"])
    n = int(z["n_layers"])
    weights = [(z[f"w{k}"].astype(np.float32), z[f"b{k}"].astype(np.float32))
               for k in range(n)]

    def run(obs):
        x = np.atleast_2d(np.asarray(obs, dtype=np.float32))
        x = (x - mean) / (std + eps)
        for k, (w, b) in enumerate(weights):
            x = x @ w.T + b
            if k < len(weights) - 1:           # ELU on the hidden layers only
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))
        return x

    return run
