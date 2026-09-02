"""Charging a lift that was asked for while barely wrapped.

The wrap-angle gate under the lift was doing two jobs: keeping a shallow grasp
out of the retention test, and pressing the policy to keep closing.  Removing
it -- the robot has no sensor for a wrap angle -- left the second undone, and a
shallow early lift became the cheaper option: ending the episode stops paying
step cost and dodges the chance of a collision.
"""
from __future__ import annotations

import torch

from elesim_sim.rl.envs.rewards import TERM_NAMES


def _armed(phi: float, *, reference: float = 2.0944, weight: float = -3.0) -> float:
    """The term's value for one env that armed the lift at this wrap angle."""
    short = max(0.0, min(1.0, (reference - phi) / reference)) if reference > 0 else 0.0
    return short * weight


def test_the_term_is_part_of_the_reward():
    assert "lift_too_shallow" in TERM_NAMES


def test_a_lift_at_the_reference_is_not_charged():
    assert _armed(2.0944) == 0.0


def test_a_lift_past_the_reference_is_not_charged():
    """Wrapping further than asked for is not a fault."""
    assert _armed(3.5) == 0.0


def test_the_charge_grows_as_the_wrap_gets_shallower():
    shallow = _armed(0.5)
    middling = _armed(1.5)
    assert shallow < middling < 0.0


def test_a_lift_with_nothing_wrapped_pays_the_whole_weight():
    assert _armed(0.0) == -3.0


def test_a_zero_reference_disables_the_charge():
    assert _armed(0.5, reference=0.0) == 0.0


def test_the_reference_and_weight_are_configurable():
    from elesim_sim.rl.configs.loader import load_config

    cfg = load_config(None)
    assert cfg.reward.weights.lift_too_shallow < 0.0
    assert cfg.reward.coverage.shallow_lift_reference_rad > 0.0
    # the gate itself stays off: this replaces it, rather than joining it
    assert cfg.success.lift.trigger_rad == 0.0
