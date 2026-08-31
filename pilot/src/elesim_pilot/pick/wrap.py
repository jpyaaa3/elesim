"""Wrap-grasp action: a trained policy driving the arm through the pick loop.

The policy plans in the same space the pilot already commands in.  It emits a
four-DoF waypoint -- linear, roll, theta1, theta2 -- which is exactly what
`_command_q_and_wait` takes, so this adds a phase rather than a control path.

Three things it needs that the pick pipeline does not already produce:

* the object's geometry -- radius, height, position and lean, seven numbers.
  Perception does not fit cylinders, and the arm's camera sits on
  `gripper_base`, so it loses sight of the object exactly while wrapping it.
  The pole does not move, so these come from configuration: measured once and
  held for the episode.
* the load proxy.  The sim reports joint torque and the arm reports motor
  current in mA; the observation normaliser is frozen at the training
  statistics and will not absorb that, so zeros go in until the two are
  reconciled.  Measured with the load channels zeroed, the policy holds up.
* the roll-back.  The policy decides *when* to lift; the rotation is a scripted
  ramp, and `LiftScript` produces it.

One thing the simulator has that the arm does not: the wrap angle, computed from
per-link contacts.  In training it is a floor under the lift request.  Here the
request is the only gate, so a bad request rolls the arm back on nothing -- a
failed attempt, not a hazard.  A load-based guard belongs in front of
`lift_requested` once the current traces exist to set its threshold.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

import numpy as np

_LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class WrapGraspConfig:
    """Everything the action needs that is not already in the pick state."""

    #: Where the exported policy lives.  The pilot container mounts
    #: `roles/pilot/config` read-only at `/opt/elesim/config` and runs from
    #: `/opt/elesim`, so the policy travels the same way the rest of the role's
    #: configuration does -- no new mount, and nothing for `elesim-update` to
    #: overwrite.  Each is tried in order, so a checkout outside the container
    #: still finds `deploy/`.
    policy_path: str = "config/policy/policy.pt"
    manifest_path: str = "config/policy/interface.json"
    #: Fallbacks tried when the paths above are missing, in order.
    search_roots: tuple[str, ...] = (
        "/opt/elesim", ".", "deploy", "/opt/elesim/config/policy",
    )
    #: radius, height, x, y, z, lean_x, lean_y -- in the robot's base frame.
    #: The defaults are the nominal condition the policy was evaluated at; the
    #: pole has to actually be there, within about 30 mm, or the numbers are a
    #: claim rather than a measurement.
    object_geometry: tuple[float, ...] = (0.067, 1.1, 0.368, 0.160, 0.550, 0.0, 0.0)
    #: Motor current and joint torque are not the same units, so the load
    #: channels are zeroed until they are reconciled.
    zero_load_proxy: bool = True
    #: Seconds allowed for the arm to reach each waypoint.  The training step is
    #: 0.4 s of simulated time; this is a real settle timeout, not that.
    step_timeout_s: float = 1.0
    #: Substep period for the scripted roll-back, matching the manifest.
    lift_substep_s: float = 0.01
    max_steps: Optional[int] = None


@dataclass
class WrapGraspOutcome:
    steps: int = 0
    lift_requested: bool = False
    lift_completed: bool = False
    reason: str = ""
    waypoints: list[tuple[float, float, float, float]] = field(default_factory=list)


class WrapGraspRunner:
    """Drives one wrap-grasp attempt.

    Deliberately free of pilot imports so it can be exercised without a robot:
    the three callables are the whole interface to the arm.
    """

    def __init__(
        self,
        cfg: WrapGraspConfig,
        *,
        read_joints: Callable[[], Sequence[float]],
        command_waypoint: Callable[[Sequence[float], float], bool],
        read_load: Optional[Callable[[], Sequence[float]]] = None,
        command_roll: Optional[Callable[[float], None]] = None,
        sleep: Optional[Callable[[float], None]] = None,
        cancelled: Optional[Callable[[], bool]] = None,
    ) -> None:
        from elesim_sim.rl.deploy import DeployedPolicy, LiftScript

        self.cfg = cfg
        policy, manifest = self._locate(cfg)
        self.policy = DeployedPolicy(policy, manifest)
        self._LiftScript = LiftScript
        self._read_joints = read_joints
        self._command_waypoint = command_waypoint
        self._read_load = read_load
        self._command_roll = command_roll
        self._sleep = sleep
        self._cancelled = cancelled or (lambda: False)

    @staticmethod
    def _locate(cfg: WrapGraspConfig) -> tuple[Path, Path]:
        """Find the two exported files, and say where it looked if they are not there.

        `torch.jit.load` on a missing path raises without naming the search, and
        the pilot's working directory is not obvious from inside a container.
        """
        for root in ("",) + tuple(cfg.search_roots):
            base = Path(root) if root else Path()
            policy, manifest = base / cfg.policy_path, base / cfg.manifest_path
            if policy.is_file() and manifest.is_file():
                return policy, manifest
            # ...also accept the bare filenames under a search root, which is
            # what `cp deploy/* <root>/` produces.
            policy, manifest = base / "policy.pt", base / "interface.json"
            if policy.is_file() and manifest.is_file():
                return policy, manifest
        looked = ", ".join(str(Path(r) / cfg.policy_path)
                           for r in ("",) + tuple(cfg.search_roots))
        raise FileNotFoundError(
            f"내보낸 정책을 찾지 못했습니다. 찾은 곳: {looked}. "
            f"elesim_sim.rl.export 로 만든 policy.pt 와 interface.json 을 "
            f"pilot 역할의 config 아래(예: roles/pilot/config/policy/)에 두세요."
        )

    # -- observations ------------------------------------------------------

    def _load_proxy(self) -> Sequence[float]:
        if self.cfg.zero_load_proxy or self._read_load is None:
            return self.policy.ZERO_LOAD
        return self._read_load()

    @staticmethod
    def load_proxy_from_currents(currents: dict[str, int]) -> tuple[float, ...]:
        """Map the arm's motor currents onto the four channels the policy reads.

        The sim reports one bend load into two channels, so the two segments are
        averaged and repeated.  Units still differ from training -- this is for
        when they have been reconciled, not before.
        """
        def pick(*names: str) -> float:
            table = {
                str(k).strip().lower().replace("_", "").replace("-", ""): v
                for k, v in dict(currents or {}).items()
            }
            for name in names:
                key = name.replace("_", "").replace("-", "")
                if key in table:
                    return float(table[key])
            return 0.0

        seg = 0.5 * (pick("seg1", "s1") + pick("seg2", "s2"))
        return (pick("linear"), pick("roll"), seg, seg)

    # -- the attempt -------------------------------------------------------

    def run(self) -> WrapGraspOutcome:
        out = WrapGraspOutcome()
        self.policy.reset()
        limit = self.cfg.max_steps or self.policy.iface.max_steps

        for _ in range(int(limit)):
            if self._cancelled():
                out.reason = "cancelled"
                return out
            joints = list(self._read_joints())
            if len(joints) != 4:
                out.reason = f"관절 추정이 4개가 아니라 {len(joints)}개입니다"
                return out

            waypoint, lift = self.policy.act(
                joint_estimate=joints,
                object_geometry=self.cfg.object_geometry,
                load_proxy=self._load_proxy(),
            )
            out.steps += 1
            out.waypoints.append(waypoint)

            if not self._command_waypoint(waypoint, self.cfg.step_timeout_s):
                out.reason = "waypoint 도달 실패"
                return out

            if lift:
                out.lift_requested = True
                out.lift_completed = self._run_lift(waypoint)
                out.reason = "lift" if out.lift_completed else "lift 중단"
                return out

        out.reason = "steps 소진"
        return out

    def _run_lift(self, waypoint: Sequence[float]) -> bool:
        """The scripted roll-back.

        Speed is the point: measured on 32 environments, 90 degrees taken in
        0.31 s retains nothing and the same rotation in 1.05 s retains 72%.
        """
        if self._command_roll is None:
            _LOG.warning("들기 요청이 나왔으나 roll 명령 경로가 없습니다")
            return False
        script = self._LiftScript(self.policy.iface)
        script.start(float(self._read_joints()[1]))
        guard = 10 * (
            self.policy.iface.lift_settle_substeps
            + self.policy.iface.lift_hold_substeps
        )
        for _ in range(guard):
            if self._cancelled():
                return False
            self._command_roll(script.advance())
            if self._sleep is not None:
                self._sleep(self.cfg.lift_substep_s)
            if script.finished:
                return True
        _LOG.warning("들기 스크립트가 예산 안에 끝나지 않았습니다")
        return False


class WrapActions:
    """Mixin for `_ControlServiceCore`: one phase that runs the policy.

    Kept to the adapters, so the loop itself stays in `WrapGraspRunner` where it
    can be tested without a robot.  `_command_q_and_wait` already clamps the
    joint limits and waits for arrival; the coupled curl cap is applied on the
    policy side, inside the waypoint mapper.
    """

    def wrap_grasp_config(self) -> WrapGraspConfig:
        return getattr(self, "_wrap_cfg", None) or WrapGraspConfig()

    def start_wrap_grasp(self) -> WrapGraspOutcome:
        cfg = self.wrap_grasp_config()

        def read_joints() -> Sequence[float]:
            host = getattr(self, "host_state", None)
            q = getattr(host, "q", None) if host is not None else None
            if q is None:
                st = self.state
                return (st.linear, st.roll, st.theta1, st.theta2)
            return (float(q.linear), float(q.roll), float(q.theta1), float(q.theta2))

        def command(waypoint: Sequence[float], timeout_s: float) -> bool:
            reply = self._command_q_and_wait(
                np.asarray(waypoint, dtype=float),
                timeout_s=float(timeout_s),
                source="wrap_policy",
            )
            return reply is not None

        def command_roll(roll: float) -> None:
            wp = list(self.policy_waypoint)
            wp[1] = float(roll)
            self._command_q_and_wait(
                np.asarray(wp, dtype=float), timeout_s=0.05, source="wrap_lift"
            )

        runner = WrapGraspRunner(
            cfg,
            read_joints=read_joints,
            command_waypoint=command,
            command_roll=command_roll,
            cancelled=lambda: bool(getattr(self, "pick_cancelled", False)),
        )
        self.policy_waypoint = runner.policy.waypoint
        outcome = runner.run()
        _LOG.info(
            "wrap grasp: %d 스텝, 들기 요청 %s, 완료 %s (%s)",
            outcome.steps, outcome.lift_requested, outcome.lift_completed,
            outcome.reason,
        )
        return outcome


__all__ = [
    "WrapActions", "WrapGraspConfig", "WrapGraspOutcome", "WrapGraspRunner",
]
