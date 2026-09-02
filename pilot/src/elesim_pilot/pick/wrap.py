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
import math
import threading
import time
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
    #: 0.4 s of simulated time; this is a real settle timeout, not that.  The
    #: bend joints are the slow ones -- the sim reports 0.288 rad/s against a
    #: 0.25 rad per-step limit, so a full-rate step alone is ~0.87 s before the
    #: three consecutive in-tolerance samples the settle check wants.  Kept
    #: generous on purpose: this is a ceiling on the settle wait, not a delay --
    #: a step that settles quickly costs nothing here, while one that is short
    #: reports "waypoint 도달 실패" and stops the attempt on the first slow move.
    step_timeout_s: float = 6.0
    #: Stand-in for the wrap-angle floor the training env put under the lift.
    #:
    #: Training armed the lift on `phi >= 120 deg AND the policy asked`, and the
    #: policy asks on 67.7% of steps because the floor was there to hold it
    #: back.  Phi is computed from per-link contacts, which this arm has no
    #: sensor for, so honouring the bare request ends an attempt in two steps
    #: with nothing wrapped.  Both curl angles at or past this counts as
    #: wrapped.
    #:
    #: This started as a two-sided band, -0.30..-0.19, measured over 7168 steps
    #: of the trained policy in sim (64 envs, radii 45-100 mm, from Home): with
    #: phi over the floor, theta1 and theta2 sat in a band 0.021 rad wide there.
    #: On the robot they do not.  The arm reaches its wrap with theta2 pinned at
    #: the -0.628 joint limit, far below that band, so the lower edge refused
    #: every request: an attempt wrapped by step 11 held 23 of them and ran out
    #: of steps.  Dropping the lower edge costs precision against sim's phi --
    #: 90.8% to 76.9%, recall 99.7% to 99.8% -- and is what makes the gate fire
    #: at all on hardware.  A gate that never fires has no precision worth
    #: keeping.
    #:
    #: Still a substitute, not the trained floor: it says the arm is curled the
    #: way a wrap curls it, not that anything is held.
    wrap_gate_theta_max_rad: float = -0.19
    #: How many intermediate targets to walk on the way to each waypoint.
    #:
    #: The training env interpolates the waypoint over the first
    #: `move_fraction` of the macro step (24 of its 40 substeps) and holds for
    #: the rest.  Commanding the whole waypoint at once is a step input: the
    #: arm lurches, and against an object it shoves before it closes.  Walking
    #: the same ramp gives the arm a moving target to track instead.
    #:
    #: Halved off that 24 on the bench: with the profile accelerations shaping
    #: each hop anyway, fewer and larger intermediate targets read smoother than
    #: many small ones, and the approach finishes in a fraction of the time.
    #: The ramp stays a straight line -- this only changes how many points sit
    #: along it.
    approach_substeps: int = 12
    #: Seconds between those intermediate targets.  The sim's own 0.01 s assumes
    #: sim-rate joints; the arm needs a slower ramp than that to follow one
    #: without lurching, and the halved profile accelerations shape the rest.
    #: At 24 substeps this
    #: makes each approach about a second and a half, which is what read as
    #: smooth on the bench.  Raising it further only lengthens the ramp -- the
    #: per-step timeout covers the settle wait that follows, not this.
    approach_substep_s: float = 0.06
    #: Stop the attempt when any motor draws more than this, in mA.
    #:
    #: The robot latches a safety fault at `current_limit_ma` (2500) and cuts
    #: arm torque, which needs a restart to clear -- there is no operator path
    #: to `clear_fault`.  Stopping first leaves the arm holding position with a
    #: reason to read, and 1500 mA is comfortably under that latch while still
    #: above what an unobstructed move draws.  Zero disables the check.
    abort_current_ma: float = 2400.0
    #: How close counts as arrived, and whether the arm has to come to rest.
    #:
    #: `_wait_until_q_settled` defaults to 2 mm / 2 deg held for three
    #: consecutive samples, which is a rest condition: the arm stops dead at
    #: every waypoint and the approach reads as go-stop-go-stop.  Training had
    #: no such gate -- its macro step was a fixed 0.4 s and the next one began
    #: from wherever the arm had got to, tracking error and all.
    #:
    #: One sample inside a looser window keeps the bound that catches a step
    #: the arm never reached, without demanding it stand still first.
    #: Widened once more off 5 mm / 5 deg, which already read smooth on the
    #: bench: handing the step over a little earlier keeps the arm moving
    #: through the waypoint instead of easing into it.  The cost is that the
    #: policy plans from a slightly larger tracking error, so this is as far as
    #: it is worth taking without watching what the wrap does.
    settle_linear_tol_m: float = 8e-3
    settle_angle_tol_rad: float = 0.140        # 8 deg
    settle_consecutive: int = 1
    #: Cap on how far the curl joints may be *commanded*, in rad.
    #:
    #: The joint limit is 0.6283 (36 deg per node).  Measured on the arm, the
    #: policy commands theta2 all the way there while the object stops it at
    #: about -0.52, and the motor then pushes against that stop: 2238 mA at the
    #: moment the lift armed, 2755 during it, against the robot's own 2500 mA
    #: latch.  There is no way to win that race from here -- the robot monitors
    #: at 20 Hz and publishes telemetry at 10 -- so the load has to stay in a
    #: range this side can act in, which means not commanding past where the arm
    #: actually reaches.  None disables the cap.
    #:
    #: Sim wraps at theta2 about -0.25, a segment total of 72 deg; the arm was
    #: being driven to 187 deg.  Capping here is closer to the trained pose, not
    #: further from it.
    theta_command_max_rad: Optional[float] = 0.52
    #: A bend motor at or over this counts as wrapped, on its own.
    #:
    #: Nothing on this arm senses contact, which is what the training env's
    #: wrap-angle floor was computed from.  Motor load is the closest thing it
    #: does have: the coil closing on something shows up here before anything
    #: else does.  Measured on the bench, the bend motors sit under 600 mA while
    #: the arm moves freely and jump past 2000 the moment they meet resistance.
    #:
    #: Read the caveat honestly: that resistance is not necessarily the object.
    #: The same jump appears when a joint is driven into its own limit, which is
    #: what was happening when this threshold was only an abort -- theta2 was
    #: commanded to -0.628 and stalled at -0.508.  So this admits a lift that a
    #: hard stop would also admit.  Zero disables it.
    grasp_current_ma: float = 1500.0
    #: Which motors that reading is taken from: the two bend segments, which are
    #: the coil closing.  Linear (1) and roll (2) load for other reasons.
    grasp_current_motor_ids: tuple[str, ...] = ("3", "4")
    #: Roll turns the other way on this arm than in the training sim.
    #:
    #: The exported manifest already warns that the waypoints are in the
    #: convention `control_u_to_sim_q` produces; measured on the robot, roll is
    #: the one axis whose sense is reversed against it.  Converting here, at the
    #: adapter boundary, keeps the policy and its lift script in the convention
    #: they were trained in and leaves every other workflow -- sliders, presets,
    #: IK -- on the mapping they already use.
    #:
    #: Set to +1.0 to drive an arm whose roll matches the sim.
    roll_sign: float = -1.0
    #: Substep period for the scripted roll-back, matching the manifest.
    lift_substep_s: float = 0.01
    #: How long to hold after the roll is home.
    #:
    #: The manifest's 80 settle + 100 hold substeps are simulator physics
    #: being advanced; out here the roll has already arrived and there is
    #: nothing to advance, so walking them re-commanded the same pose 180
    #: times and added about twelve seconds of doing nothing.  Retention is
    #: not checked on the robot either -- this is time for the operator to
    #: see whether the object is actually up.
    lift_hold_s: float = 2.0
    max_steps: Optional[int] = None


@dataclass
class WrapGraspOutcome:
    steps: int = 0
    lift_requested: bool = False
    lift_completed: bool = False
    reason: str = ""
    #: How far the scripted roll-back actually turned.  A lift asked for
    #: while the arm is still near roll 0 unwinds nothing, and the script
    #: reports "finished" the moment it starts.
    lift_roll_rad: float = 0.0
    #: Lift requests the wrap gate held back, so an attempt that never
    #: curled far enough is distinguishable from one never asked for.
    lift_requests_held: int = 0
    #: {motor id: mA} that stopped the attempt, when a load limit did.
    overload_ma: dict = field(default_factory=dict)
    #: theta1/theta2 range actually reached, so a held request says whether
    #: the arm stopped short of the wrap band or curled past it.
    theta_seen: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    waypoints: list[tuple[float, float, float, float]] = field(default_factory=list)


def _say(message: str) -> None:
    """Trace one attempt on stdout, the way the rest of the pilot reports.

    `logging` is never configured in this process, so every _LOG call in this
    module has been invisible -- including the ones that were supposed to
    explain a failed attempt.
    """

    print(f"[wrap] {message}", flush=True)


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
        overloaded: Optional[Callable[[], dict]] = None,
        currents: Optional[Callable[[], dict]] = None,
        loaded: Optional[Callable[[], bool]] = None,
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
        self._overloaded = overloaded or (lambda: {})
        self._currents = currents or (lambda: {})
        self._loaded = loaded or (lambda: False)

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
            over = dict(self._overloaded() or {})
            if over:
                out.overload_ma = over
                out.reason = "부하 한계"
                _say(f"멈춤: 부하 한계 {over} (step {out.steps})")
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

            reached = self._command_waypoint(waypoint, self.cfg.step_timeout_s)
            _say(
                f"step {out.steps:2d}  "
                f"cmd=({waypoint[0]:+.3f},{waypoint[1]:+.3f},"
                f"{waypoint[2]:+.3f},{waypoint[3]:+.3f})  "
                f"meas=({joints[0]:+.3f},{joints[1]:+.3f},"
                f"{joints[2]:+.3f},{joints[3]:+.3f})  "
                f"reached={reached}  lift_asked={lift}  mA={self._currents()}"
            )
            if not reached:
                out.reason = "waypoint 도달 실패"
                _say(f"멈춤: {out.reason} (step {out.steps})")
                return out

            lo1, hi1, lo2, hi2 = out.theta_seen
            if out.steps == 1:
                lo1 = hi1 = float(joints[2])
                lo2 = hi2 = float(joints[3])
            out.theta_seen = (
                min(lo1, float(joints[2])), max(hi1, float(joints[2])),
                min(lo2, float(joints[3])), max(hi2, float(joints[3])),
            )

            # The ramp stops on contact, so the load has to be judged here as
            # well as at the top: a reading past the abort threshold must not
            # become a lift just because it also passes the grasp threshold.
            over = dict(self._overloaded() or {})
            if over:
                out.overload_ma = over
                out.reason = "부하 한계"
                _say(f"멈춤: 부하 한계 {over} (step {out.steps})")
                return out

            if lift:
                curled = self._wrap_gate(joints)
                loaded_now = self._loaded()
                if not (curled or loaded_now):
                    out.lift_requests_held += 1
                    lift = False
                else:
                    _say(f"들기 허용: curl={curled} load={loaded_now}")

            if lift:
                out.lift_requested = True
                self._run_lift(waypoint, out)
                out.reason = "lift" if out.lift_completed else "lift 중단"
                return out

        out.reason = "steps 소진"
        return out

    def _wrap_gate(self, joints: Sequence[float]) -> bool:
        """Whether the arm is curled the way a wrap curls it.

        See `WrapGraspConfig.wrap_gate_theta_max_rad` for what this stands in for
        and what it was measured against.
        """
        limit = float(self.cfg.wrap_gate_theta_max_rad)
        return all(float(joints[i]) <= limit for i in (2, 3))

    def _run_lift(self, waypoint: Sequence[float], out: WrapGraspOutcome) -> bool:
        """The scripted roll-back.

        Speed is the point: measured on 32 environments, 90 degrees taken in
        0.31 s retains nothing and the same rotation in 1.05 s retains 72%.
        """
        if self._command_roll is None:
            _LOG.warning("들기 요청이 나왔으나 roll 명령 경로가 없습니다")
            return False
        substep = 0
        script = self._LiftScript(self.policy.iface)
        roll0 = float(self._read_joints()[1])
        script.start(roll0)
        out.lift_roll_rad = abs(roll0 - float(self.policy.iface.lift_roll_target_rad))
        guard = 10 * (
            self.policy.iface.lift_settle_substeps
            + self.policy.iface.lift_hold_substeps
        )
        for _ in range(guard):
            if self._cancelled():
                return False
            over = dict(self._overloaded() or {})
            if over:
                out.overload_ma = over
                _say(f"들기 중단: 부하 한계 {over}")
                return False
            roll_cmd = script.advance()
            if substep % 10 == 0:           # ~10 Hz, the telemetry rate
                _say(f"lift  roll={roll_cmd:+.3f}  mA={self._currents()}")
            substep += 1
            self._command_roll(roll_cmd)
            if script.phase != "rolling":
                break
            if self._sleep is not None:
                self._sleep(self.cfg.lift_substep_s)
        else:
            _say("들기 스크립트가 예산 안에 끝나지 않았습니다")
            return False

        # The roll is home.  Settling and holding are simulator substeps; hold
        # once in real time rather than re-commanding the same pose 180 times.
        if self._sleep is not None:
            self._sleep(max(0.0, float(self.cfg.lift_hold_s)))
        out.lift_completed = True
        return True



#: Read from beside the app config.  Its own file because the app config's
#: schema refuses top-level keys it does not know, and these values belong to
#: the deployed policy rather than to the arm application.
WRAP_CONFIG_FILENAME = "wrap_grasp.yaml"


def wrap_grasp_config_from_mapping(
    raw: Any, *, base: Optional[WrapGraspConfig] = None
) -> WrapGraspConfig:
    """Build a config from a `wrap_grasp:` block, refusing keys it does not know.

    Bench tuning is the whole point of this block, and a silently ignored typo
    would look exactly like a value that had no effect.
    """

    import dataclasses

    cfg = base or WrapGraspConfig()
    if raw is None:
        return cfg
    if not isinstance(raw, dict):
        raise ValueError("wrap_grasp must be a mapping")

    known = {f.name for f in dataclasses.fields(WrapGraspConfig)}
    values: dict[str, Any] = {}

    obj = raw.get("object")
    if obj is not None:
        if not isinstance(obj, dict):
            raise ValueError("wrap_grasp.object must be a mapping")
        unknown = set(obj) - {"radius_m", "height_m", "xyz_m", "lean"}
        if unknown:
            raise ValueError(f"wrap_grasp.object: unknown keys {sorted(unknown)}")
        current = tuple(float(v) for v in cfg.object_geometry)
        xyz = obj.get("xyz_m", current[2:5])
        lean = obj.get("lean", current[5:7])
        if len(tuple(xyz)) != 3:
            raise ValueError("wrap_grasp.object.xyz_m needs three numbers")
        if len(tuple(lean)) != 2:
            raise ValueError("wrap_grasp.object.lean needs two numbers")
        values["object_geometry"] = (
            float(obj.get("radius_m", current[0])),
            float(obj.get("height_m", current[1])),
            *(float(v) for v in xyz),
            *(float(v) for v in lean),
        )

    for key, value in raw.items():
        if key == "object":
            continue
        if key not in known:
            raise ValueError(f"wrap_grasp: unknown key {key!r}")
        field = {f.name: f for f in dataclasses.fields(WrapGraspConfig)}[key]
        if value is None:
            values[key] = None
        elif field.type in ("int",) or isinstance(getattr(cfg, key), bool):
            values[key] = type(getattr(cfg, key))(value)
        elif isinstance(getattr(cfg, key), tuple):
            values[key] = tuple(value)
        elif isinstance(getattr(cfg, key), int) and not isinstance(getattr(cfg, key), bool):
            values[key] = int(value)
        elif isinstance(getattr(cfg, key), float):
            values[key] = float(value)
        else:
            values[key] = value
    return dataclasses.replace(cfg, **values)


class WrapActions:
    """Mixin for `_ControlServiceCore`: one phase that runs the policy.

    Kept to the adapters, so the loop itself stays in `WrapGraspRunner` where it
    can be tested without a robot.  `_command_q_and_wait` already clamps the
    joint limits and waits for arrival; the coupled curl cap is applied on the
    policy side, inside the waypoint mapper.
    """

    def wrap_grasp_config(self) -> WrapGraspConfig:
        """The config, with `wrap_grasp.yaml` beside the app config applied.

        Read once and cached: every value in here was tuned against the arm, and
        editing source to change an object radius is not a workflow.
        """

        cached = getattr(self, "_wrap_cfg", None)
        if cached is not None:
            return cached
        cfg = WrapGraspConfig()
        app = getattr(self, "_config_path", None)
        path = None if not app else Path(app).parent / WRAP_CONFIG_FILENAME
        if path is not None and path.is_file():
            try:
                import yaml

                raw = yaml.safe_load(path.read_text(encoding="utf-8"))
                cfg = wrap_grasp_config_from_mapping(raw, base=cfg)
                _say(f"설정을 읽었습니다: {path}")
            except Exception as exc:        # a bad file must not be silent
                _say(f"{path} 를 읽지 못했습니다, 기본값을 씁니다: {exc}")
                cfg = WrapGraspConfig()
        self._wrap_cfg = cfg
        return cfg

    def wrap_grasp_running(self) -> bool:
        worker = getattr(self, "_wrap_worker", None)
        return worker is not None and worker.is_alive()

    def wrap_grasp_result(self) -> str:
        return str(getattr(self, "_wrap_result", "") or "")

    def start_wrap_grasp(self) -> bool:
        """Start one attempt on its own thread and return whether it started.

        An attempt runs for up to 28 macro steps plus the lift, which is well
        over ten seconds.  The operator intent handler answers one request at a
        time, so running it inline would stall every other request -- snapshots
        included -- for the whole attempt, and the caller's reply would have
        timed out long before.  `pick_e2e` already solves this the same way:
        the call starts a worker, and the panel watches
        `wrap_grasp_running`/`wrap_grasp_result` in the view snapshot.
        """

        if self.wrap_grasp_running():
            return False
        self._wrap_result = "실행 중"
        worker = threading.Thread(
            target=self._run_wrap_grasp, name="wrap-grasp", daemon=True
        )
        self._wrap_worker = worker
        worker.start()
        return True

    def _run_wrap_grasp(self) -> None:
        try:
            outcome = self._wrap_grasp_attempt()
        except Exception as exc:                # a failed attempt is not a crash
            _LOG.exception("wrap grasp 실패")
            self._wrap_result = f"오류: {exc}"
        else:
            self._wrap_result = wrap_summary(outcome)
            _say(f"결과: {self._wrap_result}")

    def _wrap_grasp_attempt(self) -> WrapGraspOutcome:
        cfg = self.wrap_grasp_config()

        roll_sign = float(cfg.roll_sign)

        def read_joints() -> Sequence[float]:
            # `current_host_state()` is the arm's own measurement.  This used to
            # read a `host_state` attribute that does not exist on the service
            # -- that name appears only as a parameter -- so it always fell
            # through to `self.state`, the pose the pilot had *commanded*.  The
            # policy was planning from its own last command instead of from
            # where the arm actually was.
            host = self.current_host_state()
            q = getattr(host, "q", None) if host is not None else None
            if q is None:
                st = self.state
                raw = (st.linear, st.roll, st.theta1, st.theta2)
            else:
                raw = (float(q.linear_m), float(q.roll_rad),
                       float(q.theta1_rad), float(q.theta2_rad))
            # Into the policy's convention, so what it plans from and what it
            # commands agree on which way roll turns.
            return (raw[0], raw[1] * roll_sign, raw[2], raw[3])

        held: list[Optional[np.ndarray]] = [None]

        def send(q: np.ndarray) -> None:
            self.state.set_q(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            self.send_current_target(source="wrap_policy")

        def command(waypoint: Sequence[float], timeout_s: float) -> bool:
            # `_command_q_and_wait` returns the host state and drops the
            # settled flag, so a step that timed out short of its waypoint
            # still reads as success and the policy walks on from a pose the
            # arm never reached.  Take the same primitives and keep the flag.
            target = list(float(v) for v in waypoint)
            target[1] *= roll_sign          # back into the arm's convention
            cap = cfg.theta_command_max_rad
            if cap is not None:
                bound = abs(float(cap))
                for i in (2, 3):
                    target[i] = max(-bound, min(bound, target[i]))
            q_cmd = self._clamp_q(np.asarray(target, dtype=float))

            start = held[0]
            if start is None:               # first step: ramp from where it is
                start = self._clamp_q(np.asarray(
                    [v * (roll_sign if i == 1 else 1.0)
                     for i, v in enumerate(read_joints())], dtype=float
                ))
            # Watch the load *inside* the ramp.  Sampling only between steps
            # missed the whole event: motor 4 went 422 -> 2496 mA within one
            # step, so the grasp threshold was never seen and the abort landed
            # a step late, after the robot had already latched at 2500.
            steps = max(1, int(cfg.approach_substeps))
            reached_target = True
            for k in range(1, steps):
                q = start + (q_cmd - start) * (float(k) / steps)
                send(q)
                held[0] = q
                time.sleep(max(0.0, float(cfg.approach_substep_s)))
                if loaded() or overloaded():
                    # Contact.  Stop pushing and stay here: this is the pose
                    # the grasp actually closed at.
                    reached_target = False
                    break
            if reached_target:
                send(q_cmd)
                held[0] = q_cmd
                _state, settled = self._wait_until_q_settled(
                    q_cmd,
                    timeout_s=float(timeout_s),
                    linear_tol_m=float(cfg.settle_linear_tol_m),
                    angle_tol_rad=float(cfg.settle_angle_tol_rad),
                    consecutive=max(1, int(cfg.settle_consecutive)),
                )
                return bool(settled)
            return True

        def command_roll(roll: float) -> None:
            # Unwind roll and nothing else: the grasp is the rest of the pose.
            #
            # This used to read a `policy_waypoint` attribute set once before
            # the attempt from the mapper's reset value -- Home.  Every lift
            # therefore drove the arm back to Home while ramping roll, dropping
            # whatever it had just wrapped and looking, from outside, like the
            # arm going home by itself.  Nothing else read it; it is gone.
            #
            # Fire and forget: waiting for arrival on every substep cost 0.05 s
            # each and stretched a 1.05 s roll -- the speed the retention was
            # measured at -- into something several times longer.
            base = held[0]
            if base is None:                # nothing commanded yet
                return
            wp = np.array(base, dtype=float, copy=True)
            wp[1] = float(roll) * roll_sign
            send(self._clamp_q(wp))

        def currents() -> dict:
            host = self.current_host_state()
            raw = getattr(host, "motor_currents_ma", None) if host is not None else None
            return {str(k): int(v) for k, v in dict(raw or {}).items()}

        def loaded() -> bool:
            """Whether a bend motor is carrying enough to call this a grasp."""
            limit = float(cfg.grasp_current_ma)
            if limit <= 0.0:
                return False
            reading = currents()
            return any(
                abs(float(reading.get(str(motor), 0))) >= limit
                for motor in cfg.grasp_current_motor_ids
            )

        def overloaded() -> dict:
            """Which motors are over the abort threshold, if any."""
            limit = float(cfg.abort_current_ma)
            if limit <= 0.0:
                return {}
            return {
                motor: value
                for motor, value in currents().items()
                if abs(float(value)) > limit
            }

        runner = WrapGraspRunner(
            cfg,
            read_joints=read_joints,
            command_waypoint=command,
            command_roll=command_roll,
            cancelled=lambda: bool(getattr(self, "pick_cancelled", False)),
            overloaded=overloaded,
            currents=currents,
            loaded=loaded,
        )
        outcome = runner.run()
        _LOG.info(
            "wrap grasp: %d 스텝, 들기 요청 %s, 완료 %s (%s)",
            outcome.steps, outcome.lift_requested, outcome.lift_completed,
            outcome.reason,
        )
        return outcome


def wrap_summary(outcome: Optional[WrapGraspOutcome]) -> str:
    """What the operator needs off one attempt: how far it got, and why it stopped.

    Deliberately never says "success".  Off the robot there is nothing that can
    say whether the object is still held: retention was a simulator check over
    the object's pose, its clearance and its per-link contacts, and none of the
    three exists here.  What can be reported is that the scripted roll-back ran
    and how far it turned -- a lift asked for near roll 0 unwinds nothing, so
    that angle is what separates a real attempt from an empty one.
    """
    if outcome is None:
        return "결과 없음"
    steps = getattr(outcome, "steps", 0)
    reason = str(getattr(outcome, "reason", "") or "")
    if getattr(outcome, "lift_completed", False):
        turned = math.degrees(abs(float(getattr(outcome, "lift_roll_rad", 0.0) or 0.0)))
        return f"들기 실행 · {steps} 스텝 · roll {turned:.0f}° (유지 미확인)"
    if getattr(outcome, "lift_requested", False):
        return f"들기 중단 · {steps} 스텝 ({reason})"
    if str(reason) == "부하 한계":
        over = getattr(outcome, "overload_ma", {}) or {}
        return f"중단 · {steps} 스텝 (부하 한계 {over})"
    held = int(getattr(outcome, "lift_requests_held", 0) or 0)
    if held:
        lo1, hi1, lo2, hi2 = getattr(outcome, "theta_seen", (0.0, 0.0, 0.0, 0.0))
        return (f"미완 · {steps} 스텝 ({reason}, 들기요청 {held}회 보류, "
                f"th1 {lo1:+.2f}~{hi1:+.2f} th2 {lo2:+.2f}~{hi2:+.2f})")
    return f"미완 · {steps} 스텝 ({reason})"


__all__ = [
    "WrapActions", "WrapGraspConfig", "WrapGraspOutcome", "WrapGraspRunner",
    "wrap_summary",
]
