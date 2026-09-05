"""Unit tests for the scripted-lift retention test.

The lift on this arm does not raise a standing object straight up: it rolls the
bend plane back to zero, which levers the object over and lays it into the coil.
So the object is *meant* to travel a long way relative to the arm while the
rotation is happening, and every bug this file pins down came from measuring
retention at the wrong moment or against the wrong thing:

* anchored at the wrap, the manoeuvre reads 166 mm of drift and fails; anchored
  after the settle it reads 5.6 mm and passes -- one grasp, two verdicts;
* flipping the phase to FAILED on the first violation stopped the ramp, so the
  rotation the test exists to perform was never carried out;
* reading the contact count off the macro-step accumulator instead of the
  current substep made every hold fail on the substep after a reset, when
  nothing had been accumulated yet.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import dataclasses
import math

import torch

from elesim_sim.rl.configs.loader import LiftConfig, SuccessConfig
from elesim_sim.rl.envs.lift_test import LiftObservation, LiftPhase, LiftTest

DEVICE = torch.device("cpu")


def _cfg(**kw) -> SuccessConfig:
    lift = dataclasses.replace(
        LiftConfig(
            trigger_rad=1.0,
            roll_target_rad=0.0,
            roll_rate_rad_per_substep=math.pi / 8,
            settle_substeps=2,
            hold_substeps=3,
            max_rel_translation_m=0.03,
            max_rel_rotation_rad=0.5,
            min_clearance_m=0.05,
            min_object_contacts=2,
        ),
        **kw,
    )
    return dataclasses.replace(SuccessConfig(criterion="lift"), lift=lift)


def _obs(
    n: int = 1,
    *,
    object_pos: tuple[float, float, float] = (0.0, 0.0, 0.0),
    clearance: float = 1.0,
    contacts: int = 4,
) -> LiftObservation:
    return LiftObservation(
        object_pos=torch.tensor([list(object_pos)] * n),
        object_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        anchor_pos=torch.zeros(n, 3),
        anchor_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        clearance=torch.full((n,), float(clearance)),
        object_contacts=torch.full((n,), int(contacts)),
    )


def _armed(cfg: SuccessConfig, *, roll: float = math.pi / 2) -> LiftTest:
    lift = LiftTest(cfg, n_envs=1, device=DEVICE)
    lift.arm(torch.tensor([2.0]), torch.tensor([roll]), _obs())
    assert int(lift.phase[0]) == int(LiftPhase.LIFTING)
    return lift


def _run(lift: LiftTest, n: int, obs: LiftObservation) -> None:
    for _ in range(n):
        lift.advance(obs)


def test_the_rotation_completes_even_once_retention_has_been_lost():
    """A violation must not stop the ramp.

    It used to: the phase went straight to FAILED, FAILED stopped the roll, and
    a run that slipped 16 deg into a 90 deg rotation left the arm parked at 74
    deg -- so neither the video nor the diagnostics showed what the manoeuvre
    does.
    """
    lift = _armed(_cfg())
    # Dropped on the floor and released: every check fails from the start.
    bad = _obs(clearance=0.0, contacts=0)
    _run(lift, 12, bad)
    assert float(lift.roll_command[0]) == 0.0
    assert int(lift.phase[0]) == int(LiftPhase.FAILED)


def test_travel_during_the_rotation_and_the_settle_is_not_slippage():
    """The object crosses the tolerance while being laid down, and still passes.

    This is the whole point of the settle phase: 90 deg of reorientation moves
    the object far further than `max_rel_translation_m`, and measuring that as
    slippage rejected the one manoeuvre that works.
    """
    lift = _armed(_cfg())
    moved = _obs(object_pos=(0.5, 0.0, 0.0))     # 500 mm from the reference
    _run(lift, 4, moved)                          # rotation
    _run(lift, 2, moved)                          # settle -> re-anchors here
    _run(lift, 3, moved)                          # hold, now stationary
    assert bool(lift.passed[0])


def test_the_hold_is_measured_from_where_the_settle_left_the_object():
    lift = _armed(_cfg())
    laid_down = _obs(object_pos=(0.5, 0.0, 0.0))
    _run(lift, 4, laid_down)
    _run(lift, 2, laid_down)
    assert int(lift.phase[0]) == int(LiftPhase.HOLDING)
    # Anchored at the settle's pose, not at the wrap's.
    assert torch.allclose(lift._ref_rel_pos[0], torch.tensor([0.5, 0.0, 0.0]))
    _run(lift, 3, _obs(object_pos=(0.54, 0.0, 0.0)))   # 40 mm > 30 mm
    assert int(lift.phase[0]) == int(LiftPhase.FAILED)


def test_an_object_resting_on_the_floor_is_not_being_held():
    """No pose tolerance can catch this: the object is exactly where it should
    be relative to the arm -- it is simply standing on the ground.
    """
    lift = _armed(_cfg())
    _run(lift, 6, _obs())                       # rotation and settle, held
    _run(lift, 3, _obs(clearance=0.0))          # same pose, now on the floor
    assert int(lift.phase[0]) == int(LiftPhase.FAILED)


def test_an_object_the_arm_has_let_go_of_is_not_being_held():
    lift = _armed(_cfg())
    _run(lift, 6, _obs())
    _run(lift, 3, _obs(contacts=1))             # below min_object_contacts
    assert int(lift.phase[0]) == int(LiftPhase.FAILED)


def test_a_clean_lift_passes():
    lift = _armed(_cfg())
    _run(lift, 4 + 2 + 3, _obs())
    assert bool(lift.passed[0])
    assert not bool(lift._violated[0])


def test_the_roll_ramps_towards_the_target_from_either_limit():
    for start in (math.pi / 2, -math.pi / 2):
        lift = _armed(_cfg(), roll=start)
        first = float(lift.roll_command[0])
        lift.advance(_obs())
        assert abs(float(lift.roll_command[0])) < abs(first)
        _run(lift, 10, _obs())
        assert float(lift.roll_command[0]) == 0.0


def test_only_wrapping_envs_follow_the_policy():
    cfg = _cfg()
    lift = LiftTest(cfg, n_envs=3, device=DEVICE)
    lift.arm(torch.tensor([0.0, 2.0, 2.0]), torch.zeros(3), _obs(3))
    assert lift.follows_policy.tolist() == [True, False, False]


def test_a_lift_request_below_the_floor_is_refused():
    """The floor stops a request that has nothing to hold onto."""
    lift = LiftTest(_cfg(), n_envs=2, device=DEVICE)
    below = torch.tensor([lift.threshold - 0.01, lift.threshold + 0.01])
    newly = lift.arm(below, torch.zeros(2), _obs(2),
                     request=torch.tensor([True, True]))
    assert newly.tolist() == [False, True]


def test_without_a_request_nothing_arms():
    """The policy's column is what decides; the wrap angle only permits."""
    lift = LiftTest(_cfg(), n_envs=2, device=DEVICE)
    deep = torch.full((2,), lift.threshold + 1.0)
    newly = lift.arm(deep, torch.zeros(2), _obs(2),
                     request=torch.tensor([False, True]))
    assert newly.tolist() == [False, True]


def test_no_request_at_all_keeps_the_old_self_arming_behaviour():
    lift = LiftTest(_cfg(), n_envs=2, device=DEVICE)
    deep = torch.full((2,), lift.threshold + 1.0)
    newly = lift.arm(deep, torch.zeros(2), _obs(2))
    assert newly.tolist() == [True, True]
