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
                "manifest 에 home waypoint 가 없습니다: export 시 "
                "arm.home_waypoint 를 명시하세요"
            )
        limits = waypoint["limits"]
        linear = tuple(float(v) for v in limits["linear_m"])
        roll = tuple(float(v) for v in limits["roll_rad"])
        theta = tuple(float(v) for v in limits["theta_rad"])
        cap = waypoint["coupled_curl_cap"]
        timing = manifest["timing"]
        lift = manifest["lift_script"]
        return Interface(
            obs_dim=int(manifest["observation"]["dim"]),
            action_dim=int(manifest["action"]["dim"]),
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

    def __init__(self, policy_path: Path, manifest_path: Path) -> None:
        import torch

        self._torch = torch
        self.iface = Interface.from_manifest(Path(manifest_path))
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
        load_proxy: Sequence[float],
        progress: Optional[float] = None,
    ) -> Any:
        if progress is None:
            progress = self.step_index / max(self.iface.max_steps, 1)
        values = list(joint_estimate) + list(object_geometry) + list(load_proxy) + [progress]
        if len(values) != self.iface.obs_dim:
            raise ValueError(
                f"관측이 {len(values)} 개인데 정책은 {self.iface.obs_dim} 개를 기대합니다 "
                "(관절 4 + 물체 7 + 부하 4 + 진행률 1)"
            )
        return self._torch.tensor([values], dtype=self._torch.float32)

    def act(
        self,
        *,
        joint_estimate: Sequence[float],
        object_geometry: Sequence[float],
        load_proxy: Sequence[float] = ZERO_LOAD,
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
        self.mapper.apply_action([float(value) for value in action[:4]])
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
