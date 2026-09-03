"""Unit tests for the wrap-grasp action.

The runner is deliberately free of pilot imports -- three callables are its
whole interface to the arm -- so the loop can be exercised without a robot or a
simulator, which is the only way these get run at all.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

import pytest

from elesim_pilot.pick.wrap import (
    WrapActions, WrapGraspConfig, WrapGraspOutcome, WrapGraspRunner,
)

DEPLOY = Path(__file__).resolve().parents[2] / "deploy"
pytestmark = pytest.mark.skipif(
    not (DEPLOY / "policy.pt").is_file(),
    reason="deploy/policy.pt 가 없습니다 (elesim_sim.rl.export 로 생성)",
)


def _cfg(**kw) -> WrapGraspConfig:
    return WrapGraspConfig(
        policy_path=str(DEPLOY / "policy.pt"),
        manifest_path=str(DEPLOY / "interface.json"),
        **kw,
    )


def test_missing_files_say_where_they_were_looked_for(tmp_path):
    """`torch.jit.load` on a missing path raises without naming the search, and
    the pilot's working directory is not obvious from inside a container.
    """
    import pytest as _pytest
    from elesim_pilot.pick.wrap import WrapGraspRunner

    cfg = WrapGraspConfig(
        policy_path="nope/policy.pt", manifest_path="nope/interface.json",
        search_roots=(str(tmp_path),),
    )
    with _pytest.raises(FileNotFoundError, match="찾은 곳"):
        WrapGraspRunner(cfg, read_joints=lambda: (0, 0, 0, 0),
                        command_waypoint=lambda w, t: True)


def test_bare_filenames_under_a_search_root_are_found(tmp_path):
    """`cp deploy/* roles/pilot/config/policy/` is the documented move."""
    import shutil
    from elesim_pilot.pick.wrap import WrapGraspRunner

    for name in ("policy.pt", "interface.json"):
        shutil.copy2(DEPLOY / name, tmp_path / name)
    cfg = WrapGraspConfig(
        policy_path="config/policy/policy.pt",
        manifest_path="config/policy/interface.json",
        search_roots=(str(tmp_path),),
    )
    runner = WrapGraspRunner(cfg, read_joints=lambda: (0, 0, 0, 0),
                            command_waypoint=lambda w, t: True)
    # The shipped observation, pinned deliberately: the stage-3 redesign
    # dropped the four load channels, and a runner still assembling them would
    # hand a 16-wide vector to a 12-wide network.
    assert runner.policy.iface.obs_dim == 12
    assert not runner.policy.expects_load


def _force_lift(runner):
    """Make the policy ask to lift on every step, and return the runner.

    The gate tests below used to rely on the shipped checkpoint asking by
    itself -- the stage-2 policy asked on 67.7% of steps, so it always did
    within the budget.  That made them tests of the checkpoint rather than of
    the gate: the stage-3 policy asks on different steps, reached the step
    limit without asking, and the gate under test never ran.  Forcing the
    request leaves the gate, the roll-back and the summary real.
    """
    inner = runner.policy.act

    def act(**kw):
        waypoint, _ = inner(**kw)
        return waypoint, True

    runner.policy.act = act
    return runner


class _Arm:
    """An arm that reaches whatever it is told, and remembers the order."""

    def __init__(self, *, reach=True):
        self.q = [-0.1656, 0.0, -0.5934, 0.5934]
        self.commanded: list[tuple] = []
        self.rolls: list[float] = []
        self.reach = reach

    def read(self) -> Sequence[float]:
        return tuple(self.q)

    def command(self, waypoint, timeout_s) -> bool:
        self.commanded.append(tuple(waypoint))
        if self.reach:
            self.q = list(waypoint)
        return self.reach

    def command_roll(self, roll: float) -> None:
        self.rolls.append(float(roll))
        self.q[1] = float(roll)


def test_it_steps_the_policy_and_commands_every_waypoint():
    arm = _Arm()
    out = WrapGraspRunner(
        _cfg(max_steps=5), read_joints=arm.read, command_waypoint=arm.command,
        command_roll=arm.command_roll, sleep=lambda _s: None,
    ).run()
    assert out.steps == len(arm.commanded) > 0
    assert all(len(w) == 4 for w in arm.commanded)


def test_a_waypoint_the_arm_cannot_reach_stops_the_attempt():
    arm = _Arm(reach=False)
    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    ).run()
    assert out.steps == 1 and "도달" in out.reason


def test_cancelling_stops_before_the_next_command():
    arm = _Arm()
    flag = {"stop": False}

    def cancelled() -> bool:
        was = flag["stop"]
        flag["stop"] = True         # cancel after the first check
        return was

    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        cancelled=cancelled,
    ).run()
    assert out.reason == "cancelled"
    assert len(arm.commanded) <= 1


def test_the_lift_ramps_the_roll_and_leaves_the_rest_alone():
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        command_roll=arm.command_roll, sleep=lambda _s: None,
    )
    arm.q[1] = -1.5673                     # wrapped at roll -90
    assert runner._run_lift((0.0, -1.5673, 0.0, 0.0), WrapGraspOutcome())
    assert arm.rolls[0] < arm.rolls[len(arm.rolls) // 2] <= 0.0
    assert arm.rolls[-1] == pytest.approx(0.0, abs=1e-9)


def test_a_lift_request_without_a_roll_path_is_reported_not_crashed():
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    )
    assert runner._run_lift((0.0, -1.5673, 0.0, 0.0), WrapGraspOutcome()) is False


def test_no_load_proxy_is_sent_to_a_policy_that_has_no_load_channels():
    """The manifest decides, not the config flag.

    `zero_load_proxy` chose *what* to send when the channels existed.  The
    stage-3 observation has none, so the question is no longer what to send but
    whether to send anything -- and a reader wired up is not a reason to.
    """
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        read_load=lambda: (999.0, 999.0, 999.0, 999.0),
    )
    assert not runner.policy.expects_load
    assert runner._load_proxy() is None


def test_the_shipped_policy_steps_without_load_channels():
    """End to end on the real export: no load in, a waypoint out."""
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    )
    waypoint, lift = runner.policy.act(
        joint_estimate=arm.read(),
        object_geometry=_cfg().object_geometry,
        load_proxy=runner._load_proxy(),
    )
    assert len(waypoint) == 4
    assert isinstance(lift, bool)


def test_currents_map_onto_the_four_channels():
    """The sim reports one bend load into two channels, so the segments average."""
    got = WrapGraspRunner.load_proxy_from_currents(
        {"linear": 100, "roll": 200, "seg1": 300, "seg2": 500}
    )
    assert got == (100.0, 200.0, 400.0, 400.0)
    # Names are matched loosely, the way the UI panel does it.
    assert WrapGraspRunner.load_proxy_from_currents({"S1": 10, "s2": 30})[2] == 20.0
    # ...and a missing joint reads zero rather than raising.
    assert WrapGraspRunner.load_proxy_from_currents({})[0] == 0.0


def test_a_bad_joint_reading_is_refused():
    arm = _Arm()
    arm.q = [0.0, 0.0, 0.0]                # three, not four
    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    ).run()
    assert "4개" in out.reason


def test_the_mixin_defaults_to_the_nominal_condition():
    cfg = WrapActions().wrap_grasp_config()
    assert len(cfg.object_geometry) == 7
    assert cfg.zero_load_proxy is True


def test_an_attempt_runs_off_the_request_thread_and_reports_through_the_snapshot():
    """The intent handler must come back before the attempt does.

    One attempt is 28 macro steps plus the lift -- over ten seconds.  The
    operator dispatcher answers one request at a time, so running it inline
    stalls every other request for that whole span and the caller's reply has
    long since timed out.  Starting it returns immediately; how it went shows
    up in the values the view snapshot carries.
    """

    import threading

    release = threading.Event()

    class _Service(WrapActions):
        def _wrap_grasp_attempt(self):
            release.wait(5.0)
            return WrapGraspOutcome(steps=7, reason="lift",
                                    lift_requested=True, lift_completed=True)

    service = _Service()
    assert service.wrap_grasp_running() is False

    assert service.start_wrap_grasp() is True
    assert service.wrap_grasp_running() is True
    # A second press while one is in flight must not start a rival attempt.
    assert service.start_wrap_grasp() is False

    release.set()
    service._wrap_worker.join(5.0)
    assert service.wrap_grasp_running() is False
    assert service.wrap_grasp_result() == "들기 실행 · 7 스텝 · roll 0° (유지 미확인)"


def test_a_failed_attempt_surfaces_as_a_result_rather_than_killing_the_worker():

    class _Service(WrapActions):
        def _wrap_grasp_attempt(self):
            raise RuntimeError("정책 파일 없음")

    service = _Service()
    assert service.start_wrap_grasp() is True
    service._wrap_worker.join(5.0)
    assert service.wrap_grasp_running() is False
    assert "정책 파일 없음" in service.wrap_grasp_result()


def test_a_step_that_never_settles_stops_the_attempt():
    """A timed-out step is not a reached waypoint.

    The pilot's send helper returns the host state and drops the settle flag,
    so treating a non-None reply as arrival let the policy plan its next step
    from a pose the arm was still travelling toward.
    """

    from elesim_pilot.pick.wrap import WrapGraspRunner

    calls: list[tuple[float, ...]] = []

    def command(waypoint, timeout_s):
        calls.append(tuple(waypoint))
        return False                      # never settles

    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: (-0.166, 0.0, 0.0, 0.0),
        command_waypoint=command,
        command_roll=lambda roll: None,
    )
    outcome = runner.run()

    assert outcome.reason == "waypoint 도달 실패"
    assert outcome.steps == 1              # it stops instead of walking on
    assert len(calls) == 1
    assert outcome.lift_requested is False


def test_a_lift_asked_for_before_the_arm_is_curled_is_held_back():
    """Training gated the lift on a wrap angle the robot cannot measure.

    Honouring a bare request unwinds a roll that never wrapped and ends the
    attempt after two steps, so the request has to be held until the arm is
    actually curled.  The request is forced here rather than waited for: which
    steps a given checkpoint asks on is not what this test is about.
    """

    from elesim_pilot.pick.wrap import WrapGraspRunner

    lifted: list[float] = []
    q = [-0.166, 0.0, 0.0, 0.0]            # straight, nowhere near a wrap

    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),), max_steps=3),
        read_joints=lambda: tuple(q),
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: lifted.append(roll),
    )
    outcome = _force_lift(runner).run()

    assert lifted == []                     # the roll-back never ran
    assert outcome.lift_requested is False
    assert outcome.lift_requests_held > 0
    assert "보류" in __import__(
        "elesim_pilot.pick.wrap", fromlist=["wrap_summary"]
    ).wrap_summary(outcome)


def test_the_gate_admits_the_curl_a_wrap_actually_reaches():
    from elesim_pilot.pick.wrap import WrapGraspRunner

    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: (-0.09, -1.57, -0.24, -0.25),
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: None,
    )
    # sim reaches its wrap around theta1/theta2 = -0.24/-0.25 ...
    assert runner._wrap_gate((-0.09, -1.57, -0.24, -0.25)) is True
    # ... and the robot with theta2 pinned at the -0.628 joint limit.  A band
    # with a lower edge admitted the first and refused the second, which is why
    # a hardware attempt wrapped by step 11 held 23 requests.
    assert runner._wrap_gate((-0.14, -1.36, -0.22, -0.628)) is True
    assert runner._wrap_gate((-0.17, 0.0, 0.0, 0.0)) is False
    assert runner._wrap_gate((-0.17, 0.0, -0.59, 0.59)) is False


def test_roll_is_converted_in_both_directions_at_the_adapter():
    """Roll turns the other way on the arm than in the training sim.

    The conversion has to be symmetric: what the policy is told it is at, and
    what gets commanded, must agree on which way roll turns, or the policy
    plans a wrap and drives its mirror image.
    """

    from elesim_pilot.pick.wrap import WrapGraspConfig

    cfg = WrapGraspConfig()
    assert cfg.roll_sign == -1.0

    sign = float(cfg.roll_sign)
    arm_roll = -1.2                          # what the arm reports
    seen_by_policy = arm_roll * sign         # what read_joints hands over
    commanded = seen_by_policy * sign        # what command sends back

    assert seen_by_policy == 1.2
    assert commanded == arm_roll             # a round trip is the identity


def test_the_adapter_applies_the_sign_to_read_command_and_lift():
    """Guard the three places, so one of them cannot be forgotten."""
    import inspect

    from elesim_pilot.pick import wrap

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    assert src.count("roll_sign") >= 4       # binding + read + command + lift
    assert "raw[1] * roll_sign" in src
    assert "target[1] *= roll_sign" in src
    assert "float(roll) * roll_sign" in src


def test_a_waypoint_is_walked_as_a_ramp_not_a_step():
    """Commanding the whole waypoint at once is an impulse.

    The training env interpolates over the first `move_fraction` of the macro
    step; against an object a step input shoves before it closes.  The adapter
    has to walk intermediate targets and only judge settling at the end.
    """
    import inspect

    from elesim_pilot.pick.wrap import WrapGraspConfig, WrapActions

    cfg = WrapGraspConfig()
    assert cfg.approach_substeps == 12        # halved off the env's 40 x 0.6
    assert cfg.approach_substep_s > 0.0

    src = inspect.getsource(WrapActions._wrap_grasp_attempt)
    # the ramp, then one settle check -- not a settle check per intermediate
    assert "approach_substeps" in src
    assert "start + (q_cmd - start)" in src
    assert src.count("_wait_until_q_settled") == 1


def test_the_ramp_starts_where_the_previous_one_ended():
    """A ramp from a stale origin re-traverses ground the arm already covered."""
    import inspect

    from elesim_pilot.pick.wrap import WrapActions

    src = inspect.getsource(WrapActions._wrap_grasp_attempt)
    assert "held[0] = q_cmd" in src
    assert "start = held[0]" in src


def test_a_motor_over_the_load_limit_stops_the_attempt():
    """Stop before the robot's own latch does.

    The robot cuts arm torque at 2500 mA and needs a restart to clear, since
    nothing sends it `clear_fault`.  Stopping at 1500 leaves the arm holding
    position and names the motor.
    """

    from elesim_pilot.pick.wrap import WrapGraspRunner, wrap_summary

    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: (-0.166, 0.0, 0.0, 0.0),
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: None,
        overloaded=lambda: {"1": -1820},
    )
    outcome = runner.run()

    assert outcome.reason == "부하 한계"
    assert outcome.steps == 0                 # refused before commanding a move
    assert outcome.overload_ma == {"1": -1820}
    assert "1820" in wrap_summary(outcome)


def test_the_load_limit_can_be_disabled():
    from elesim_pilot.pick.wrap import WrapGraspRunner

    calls: list[int] = []
    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),), abort_current_ma=0.0,
                        max_steps=1),
        read_joints=lambda: (-0.166, 0.0, 0.0, 0.0),
        command_waypoint=lambda waypoint, timeout_s: (calls.append(1), True)[1],
        command_roll=lambda roll: None,
        overloaded=lambda: {},
    )
    runner.run()
    assert calls == [1]


def test_the_lift_holds_the_grasp_and_only_unwinds_roll():
    """The lift must keep the pose it wrapped with.

    It used to base each roll command on `policy_waypoint`, which is set once
    before the attempt from the mapper's reset value -- Home.  Every lift drove
    the arm back to Home while ramping roll, dropping what it had wrapped.
    """

    from elesim_pilot.pick.wrap import WrapGraspRunner, WrapGraspOutcome

    WRAPPED = (-0.140, -1.360, -0.230, -0.628)
    sent: list[tuple[float, ...]] = []

    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: WRAPPED,
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: sent.append((roll,)),
        sleep=lambda seconds: None,
    )
    assert runner._run_lift(WRAPPED, WrapGraspOutcome()) is True

    assert sent, "roll 램프가 돌지 않았습니다"
    assert sent[-1][0] == pytest.approx(0.0, abs=1e-6)         # roll came home


def test_the_adapter_bases_the_roll_ramp_on_the_last_commanded_waypoint():
    import inspect

    from elesim_pilot.pick import wrap

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    body = src.split("def command_roll")[1].split("runner = WrapGraspRunner")[0]
    assert "base = held[0]" in body
    # the Home-valued attribute the ramp used to start from is gone entirely
    assert "policy_waypoint" not in src.replace(
        "a `policy_waypoint` attribute", ""
    )


def test_the_adapter_reads_the_arm_and_not_its_own_last_command():
    """The policy must plan from where the arm is, not from what it asked for.

    `host_state` is only ever a parameter name on the service; reading it as an
    attribute silently yielded None, so both the joint estimate and the motor
    currents fell back -- the estimate to the commanded pose, and the currents
    to an empty dict, which left the load guard inert through every attempt.
    """
    import inspect

    from elesim_pilot.pick import wrap

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    assert 'getattr(self, "host_state"' not in src
    assert src.count("self.current_host_state()") >= 2   # joints and currents
    # the measured pose uses SimQ's field names, not the panel state's
    assert "q.linear_m" in src and "q.theta2_rad" in src


def test_motor_load_opens_the_lift_gate_on_its_own():
    """Load is the closest thing this arm has to the contact sensing training used."""

    from elesim_pilot.pick.wrap import WrapGraspRunner

    lifted: list[float] = []
    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: (-0.166, 0.0, 0.0, 0.0),   # nowhere near curled
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: lifted.append(roll),
        sleep=lambda seconds: None,
        loaded=lambda: True,
    )
    outcome = _force_lift(runner).run()

    assert outcome.lift_requested is True
    assert outcome.lift_requests_held == 0
    assert lifted, "리프트가 돌지 않았습니다"


def test_the_two_current_thresholds_do_not_collide():
    """Opening the gate must sit below aborting, or a grasp aborts itself."""

    cfg = WrapGraspConfig()
    assert cfg.grasp_current_ma < cfg.abort_current_ma
    # and the abort still has to land under the robot's own 2500 mA latch,
    # which cuts arm torque and needs a restart to clear
    assert cfg.abort_current_ma < 2500.0


def test_only_the_bend_motors_count_as_a_grasp():
    """Linear and roll carry load for reasons that are not a grasp."""

    cfg = WrapGraspConfig()
    assert cfg.grasp_current_motor_ids == ("3", "4")
    assert "1" not in cfg.grasp_current_motor_ids   # linear
    assert "2" not in cfg.grasp_current_motor_ids   # roll


def test_contact_inside_the_ramp_ends_the_step_where_it_happened():
    """Load is sampled between intermediate targets, not only between steps.

    Motor 4 went 422 -> 2496 mA within one step on the bench, so a check that
    only runs at the step boundary sees neither threshold in time: the grasp
    reading is missed and the abort lands after the robot has already latched.
    """
    import inspect

    from elesim_pilot.pick import wrap

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    ramp = src.split("steps = max(1, int(cfg.approach_substeps))")[1]
    ramp = ramp.split("def command_roll")[0]
    assert "loaded() or overloaded()" in ramp
    # stopping on contact is not a failure to reach -- the attempt continues
    assert "reached_target = False" in ramp


def test_an_overload_is_not_turned_into_a_lift():
    """Past the abort threshold the attempt stops, even though 1500 also passed."""

    from elesim_pilot.pick.wrap import WrapGraspRunner

    lifted: list[float] = []
    runner = WrapGraspRunner(
        WrapGraspConfig(search_roots=(str(DEPLOY),)),
        read_joints=lambda: (-0.146, -1.43, -0.21, -0.50),
        command_waypoint=lambda waypoint, timeout_s: True,
        command_roll=lambda roll: lifted.append(roll),
        sleep=lambda seconds: None,
        loaded=lambda: True,                 # would open the gate on its own
        overloaded=lambda: {"4": 2496},      # but this outranks it
    )
    outcome = runner.run()

    assert outcome.reason == "부하 한계"
    assert lifted == []


def test_the_curl_command_is_capped_short_of_the_joint_limit():
    """Commanding past where the arm reaches turns the motor into a press.

    theta2 was commanded to the -0.6283 joint limit while the object stopped it
    near -0.52; the motor then pushed against that stop at 2238 mA rising to
    2755, against the robot's own 2500 latch.  The cap keeps the load in a range
    this side can still react to -- its telemetry is 10 Hz against the robot's
    20 Hz monitor.
    """
    import inspect

    from elesim_pilot.pick import wrap

    cfg = WrapGraspConfig()
    assert cfg.theta_command_max_rad is not None
    assert 0.0 < cfg.theta_command_max_rad < 0.6283      # inside the joint limit

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    body = src.split("def command(waypoint")[1].split("def command_roll")[0]
    assert "theta_command_max_rad" in body
    assert "for i in (2, 3):" in body                    # both curl joints
    assert "target[1]" not in body.split("cap =")[1].split("q_cmd")[0]  # not roll


def test_the_cap_can_be_disabled():
    cfg = WrapGraspConfig(theta_command_max_rad=None)
    assert cfg.theta_command_max_rad is None


def test_a_step_does_not_wait_for_the_arm_to_come_to_rest():
    """The settle gate must bound the step, not stop the arm at every waypoint.

    Its default is 2 mm / 2 deg held for three consecutive samples, which the
    arm only satisfies once it has stopped -- so the approach read as
    go-stop-go-stop.  Training had no such gate: a fixed 0.4 s macro step, and
    the next one started from wherever the arm had reached.
    """
    import inspect

    from elesim_pilot.pick import wrap

    cfg = WrapGraspConfig()
    assert cfg.settle_consecutive == 1                  # not a rest condition
    assert cfg.settle_linear_tol_m > 2e-3               # looser than the default
    assert cfg.settle_angle_tol_rad > 0.0349            # looser than 2 deg

    src = inspect.getsource(wrap.WrapActions._wrap_grasp_attempt)
    assert "consecutive=max(1, int(cfg.settle_consecutive))" in src


def test_the_config_block_can_move_the_object():
    """Changing an object radius must not mean editing source."""

    from elesim_pilot.pick.wrap import wrap_grasp_config_from_mapping

    cfg = wrap_grasp_config_from_mapping({
        "object": {"radius_m": 0.045, "xyz_m": [0.40, 0.10, 0.55]},
    })
    assert cfg.object_geometry == (0.045, 1.1, 0.40, 0.10, 0.55, 0.0, 0.0)
    # anything not named keeps the default
    assert cfg.grasp_current_ma == WrapGraspConfig().grasp_current_ma


def test_an_empty_or_missing_block_is_the_defaults():
    from elesim_pilot.pick.wrap import wrap_grasp_config_from_mapping

    assert wrap_grasp_config_from_mapping(None) == WrapGraspConfig()
    assert wrap_grasp_config_from_mapping({}) == WrapGraspConfig()


def test_a_typo_in_the_block_is_refused_rather_than_ignored():
    """A silently dropped key looks exactly like a value that had no effect."""

    from elesim_pilot.pick.wrap import wrap_grasp_config_from_mapping

    with pytest.raises(ValueError, match="unknown key"):
        wrap_grasp_config_from_mapping({"grasp_current_mA": 1200})
    with pytest.raises(ValueError, match="unknown keys"):
        wrap_grasp_config_from_mapping({"object": {"radius": 0.05}})
    with pytest.raises(ValueError, match="three numbers"):
        wrap_grasp_config_from_mapping({"object": {"xyz_m": [0.1, 0.2]}})
