"""Pilot-owned runtime for an exported wrap-grasp policy.

Sim produces ``policy.pt`` and ``interface.json``.  Those files are the
deployment contract; Pilot must not import Sim's training or environment
implementation to consume them.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence


@dataclass(frozen=True)
class Interface:
    """The bounded subset of the exported manifest needed at runtime."""

    obs_dim: int
    action_dim: int
    channel_names: tuple[str, ...]
    object_pose_source: str
    rate_limit: tuple[float, float, float, float]
    home: tuple[float, float, float, float]
    lower: tuple[float, float, float, float]
    upper: tuple[float, float, float, float]
    theta1_curl_weight: float
    curl_limit: Optional[float]
    macro_step_s: float
    substeps: int
    move_fraction: float
    lift_roll_target_rad: float
    lift_roll_rate_rad_per_substep: float
    lift_settle_substeps: int
    lift_hold_substeps: int
    max_steps: int

    @property
    def expects_load(self) -> bool:
        """Whether this exported policy consumes load channels."""
        return any(name.startswith("load/") for name in self.channel_names)

    def validate_mapping(self, mapping: Any) -> None:
        """Check the exported q-space against one Pilot instance's mapping."""
        if mapping is None:
            return
        names = ("linear_q_min_m", "roll_q_min_rad", "seg1_q_min_rad", "seg2_q_min_rad")
        highs = ("linear_q_max_m", "roll_q_max_rad", "seg1_q_max_rad", "seg2_q_max_rad")
        for index, (lo_name, hi_name) in enumerate(zip(names, highs)):
            lo = float(getattr(mapping, lo_name))
            hi = float(getattr(mapping, hi_name))
            if not (math.isfinite(lo) and math.isfinite(hi) and lo <= hi):
                raise ValueError(f"Pilot mapping has invalid bounds for waypoint channel {index}")
            if self.lower[index] < lo - 1e-6 or self.upper[index] > hi + 1e-6:
                raise ValueError(
                    f"exported waypoint limits are incompatible with Pilot mapping channel {index}"
                )

    @staticmethod
    def from_manifest(path: Path) -> "Interface":
        manifest = json.loads(Path(path).read_text(encoding="utf-8"))
        action = manifest["action"]["channels"]
        rate = (
            float(action[0]["scale_m"]),
            float(action[1]["scale_rad"]),
            float(action[2]["scale_rad"]),
            float(action[3]["scale_rad"]),
        )
        waypoint = manifest["waypoint"]
        home = waypoint["home"]
        if home is None:
            raise ValueError(
                "Manifest is missing the home waypoint: set "
                "arm.home_waypoint when exporting"
            )
        limits = waypoint["limits"]
        linear = tuple(float(v) for v in limits["linear_m"])
        roll = tuple(float(v) for v in limits["roll_rad"])
        theta = tuple(float(v) for v in limits["theta_rad"])
        for bounds in (linear, roll, theta):
            if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]:
                raise ValueError("manifest waypoint bounds must be finite ordered pairs")
        if len(home) != 4 or not all(math.isfinite(float(v)) for v in home):
            raise ValueError("manifest home must contain four finite values")
        if not all(math.isfinite(v) and v >= 0 for v in rate):
            raise ValueError("manifest action scales must be finite and nonnegative")
        cap = waypoint["coupled_curl_cap"]
        timing = manifest["timing"]
        lift = manifest["lift_script"]
        obs = manifest["observation"]
        obs_dim = int(obs["dim"])
        # Older exports omitted channel metadata.  Preserve their established
        # 16/12 layouts while making all new exports self-describing.
        names = tuple(str(c["name"]) for c in obs.get("channels", ()))
        if not names:
            names = tuple(
                ["joint/linear", "joint/roll", "joint/theta1", "joint/theta2"]
                + [f"object/{name}" for name in
                   ("radius", "height", "pos_x", "pos_y", "pos_z", "lean_x", "lean_y")]
                + (["load/linear", "load/roll", "load/bend", "load/bend_repeat"]
                   if obs_dim == 16 else [])
                + ["episode/progress"]
            )
        expected = (
            ["joint/linear", "joint/roll", "joint/theta1", "joint/theta2"]
            + [f"object/{name}" for name in
               ("radius", "height", "pos_x", "pos_y", "pos_z", "lean_x", "lean_y")]
            + (["load/linear", "load/roll", "load/bend", "load/bend_repeat"]
               if any(name.startswith("load/") for name in names) else [])
            + ["episode/progress"]
        )
        if list(names) != expected:
            raise ValueError("manifest observation channels do not match the supported Pilot order")
        if obs_dim not in (12, 16):
            raise ValueError("Pilot supports only the defined legacy 12- or 16-channel layouts")
        if len(names) != obs_dim:
            raise ValueError("manifest observation channel count does not match observation.dim")
        trained = manifest.get("trained_under", {})
        object_pose_source = str(trained.get("object_pose_source", "measured")).strip().lower()
        if object_pose_source not in {"measured", "told"}:
            raise ValueError("manifest trained_under.object_pose_source must be 'measured' or 'told'")
        return Interface(
            obs_dim=obs_dim,
            action_dim=int(manifest["action"]["dim"]),
            channel_names=names,
            object_pose_source=object_pose_source,
            rate_limit=rate,
            home=tuple(float(v) for v in home),  # type: ignore[arg-type]
            lower=(linear[0], roll[0], theta[0], theta[0]),
            upper=(linear[1], roll[1], theta[1], theta[1]),
            theta1_curl_weight=float(cap["theta1_weight"]),
            curl_limit=(None if cap["cap_rad"] is None else float(cap["cap_rad"])),
            macro_step_s=float(timing["macro_step_s"]),
            substeps=int(timing["substeps"]),
            move_fraction=float(timing["move_fraction"]),
            lift_roll_target_rad=float(lift["roll_target_rad"]),
            lift_roll_rate_rad_per_substep=float(lift["roll_rate_rad_per_substep"]),
            lift_settle_substeps=int(lift["settle_substeps"]),
            lift_hold_substeps=int(lift["hold_substeps"]),
            max_steps=int(timing["max_steps"]),
        )


class LiftScript:
    """Generate the exported policy's bounded roll-back trajectory."""

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
        iface = self.iface
        if self.phase == "rolling":
            target = iface.lift_roll_target_rad
            rate = iface.lift_roll_rate_rad_per_substep
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
            if self._substep >= iface.lift_settle_substeps:
                self.phase = "holding"
                self._substep = 0
        elif self.phase == "holding":
            self._substep += 1
            if self._substep >= iface.lift_hold_substeps:
                self.phase = "done"
        return self._cmd


class _WaypointMapper:
    """Single-policy waypoint state reconstructed solely from the manifest."""

    def __init__(self, iface: Interface) -> None:
        self.iface = iface
        self.waypoint = [0.0, 0.0, 0.0, 0.0]
        self.reset()

    def _curl(self, waypoint: Sequence[float]) -> float:
        return self.iface.theta1_curl_weight * float(waypoint[2]) + float(waypoint[3])

    def _project_home(self, waypoint: list[float]) -> list[float]:
        cap = self.iface.curl_limit
        if cap is None:
            return waypoint
        curl = self._curl(waypoint)
        if abs(curl) <= cap:
            return waypoint
        scale = cap / max(abs(curl), 1e-9)
        waypoint[2] *= scale
        waypoint[3] *= scale
        return waypoint

    def reset(self) -> None:
        self.waypoint = self._project_home([
            min(max(float(value), self.iface.lower[index]), self.iface.upper[index])
            for index, value in enumerate(self.iface.home)
        ])

    def apply_action(self, action: Sequence[float]) -> None:
        if len(action) != 4:
            raise ValueError("wrap policy action must contain four waypoint channels")
        current = self.waypoint
        candidate = []
        for index, value in enumerate(action):
            increment = min(max(float(value), -1.0), 1.0) * self.iface.rate_limit[index]
            candidate.append(min(max(current[index] + increment, self.iface.lower[index]),
                                 self.iface.upper[index]))

        cap = self.iface.curl_limit
        if cap is not None and abs(self._curl(candidate)) > cap:
            curl_now = self._curl(current)
            curl_new = self._curl(candidate)
            bound = cap if curl_new >= 0.0 else -cap
            denominator = curl_new - curl_now
            alpha = 1.0 if abs(denominator) <= 1e-12 else (bound - curl_now) / denominator
            alpha = min(max(alpha, 0.0), 1.0)
            for index in (2, 3):
                candidate[index] = current[index] + alpha * (candidate[index] - current[index])
        self.waypoint = candidate


class DeployedPolicy:
    """Execute one exported TorchScript policy without any Sim dependency."""

    ZERO_LOAD = (0.0, 0.0, 0.0, 0.0)

    def __init__(self, policy_path: Path, manifest_path: Path, *, mapping: Any = None) -> None:
        import torch

        self._torch = torch
        self.iface = Interface.from_manifest(Path(manifest_path))
        self.iface.validate_mapping(mapping)
        if self.iface.action_dim not in (4, 5):
            raise ValueError("exported wrap policy action.dim must be 4 or 5")
        self.policy = torch.jit.load(str(policy_path), map_location="cpu").eval()
        self.mapper = _WaypointMapper(self.iface)
        self.step_index = 0

    def reset(self) -> None:
        self.mapper.reset()
        self.step_index = 0

    @property
    def waypoint(self) -> tuple[float, float, float, float]:
        return tuple(self.mapper.waypoint)  # type: ignore[return-value]

    def observation(
        self,
        *,
        joint_estimate: Sequence[float],
        object_geometry: Sequence[float],
        load_proxy: Optional[Sequence[float]] = None,
        progress: Optional[float] = None,
    ) -> Any:
        if len(joint_estimate) != 4 or len(object_geometry) != 7:
            raise ValueError("wrap policy requires four joint estimates and seven geometry values")
        if progress is None:
            progress = self.step_index / max(self.iface.max_steps, 1)
        if self.iface.expects_load:
            load = list(self.ZERO_LOAD if load_proxy is None else load_proxy)
        elif load_proxy is not None and len(load_proxy) != 0:
            raise ValueError("this policy has no load channels; omit load_proxy")
        else:
            load = []
        if self.iface.expects_load and len(load) != 4:
            raise ValueError("wrap policy load channels require four load values")
        values = list(joint_estimate) + list(object_geometry) + load + [progress]
        if len(values) != self.iface.obs_dim:
            raise ValueError(
                f"Policy expects {self.iface.obs_dim} observations, but received {len(values)} "
                f"({', '.join(self.iface.channel_names)})"
            )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("wrap policy observation contains a non-finite value")
        return self._torch.tensor([values], dtype=self._torch.float32)

    def act(
        self,
        *,
        joint_estimate: Sequence[float],
        object_geometry: Sequence[float],
        load_proxy: Optional[Sequence[float]] = None,
        progress: Optional[float] = None,
    ) -> tuple[tuple[float, float, float, float], bool]:
        observation = self.observation(
            joint_estimate=joint_estimate,
            object_geometry=object_geometry,
            load_proxy=load_proxy,
            progress=progress,
        )
        with self._torch.no_grad():
            output = self.policy(observation)
        action = output[0]
        if len(action) < self.iface.action_dim:
            raise ValueError("exported policy returned fewer actions than manifest action.dim")
        action_values = [float(value) for value in action[:self.iface.action_dim]]
        if not all(math.isfinite(value) for value in action_values):
            raise ValueError("exported policy returned a non-finite action")
        self.mapper.apply_action(action_values[:4])
        lift = bool(action[4] > 0.0) if self.iface.action_dim > 4 else False
        self.step_index += 1
        return self.waypoint, lift

    def substep_targets(
        self, previous: Sequence[float]
    ) -> list[tuple[float, float, float, float]]:
        move = max(1, int(round(self.iface.substeps * self.iface.move_fraction)))
        result = []
        for step in range(self.iface.substeps):
            alpha = min(1.0, float(step + 1) / move)
            result.append(tuple(
                float(start) + (float(end) - float(start)) * alpha
                for start, end in zip(previous, self.waypoint)
            ))
        return result  # type: ignore[return-value]
