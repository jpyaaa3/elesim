"""Unit tests for `--set` overrides.

An override that silently loses information is worse than one that fails: a run
was started meaning to randomise object size and radius randomisation was off
the whole time, because touching one field of a curriculum stage reset its
siblings to their defaults.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest

from elesim_sim.rl.configs.loader import ConfigError, load_config


def test_setting_one_stage_field_keeps_the_others():
    """The path segment "3" has to find the integer key YAML parsed.

    Looking up the string found nothing, so the walk created a second entry
    under it, the original was shadowed, and the coercion -- mapping both to the
    integer 3 -- kept whichever came last.
    """
    before = load_config().curriculum.stages[3]
    after = load_config(
        overrides=["curriculum.stages.3.approach_shaping=true"]
    ).curriculum.stages[3]
    assert after.approach_shaping is True
    assert after.randomise_object_pose == before.randomise_object_pose
    assert after.randomise_object_radius == before.randomise_object_radius
    assert after.success_criterion == before.success_criterion


def test_the_stage_override_survives_curriculum_resolution():
    """What the environment ends up with is the resolved config, not the raw one."""
    cfg = load_config(
        overrides=[
            "curriculum.stage=3",
            "domain_randomisation.object_radius_m=[0.045,0.060]",
            "curriculum.stages.3.approach_shaping=true",
        ]
    ).resolved_for_curriculum()
    assert cfg.reward.weights.approach_shaping > 0.0
    assert cfg.domain_randomisation.object_radius_m == pytest.approx((0.045, 0.060))
    assert cfg.domain_randomisation.object_pos_jitter_m[0] > 0.0


def test_a_digit_string_is_accepted_where_an_int_is_wanted():
    assert load_config(overrides=["macro_step.max_steps=20"]).macro_step.max_steps == 20


def test_a_non_numeric_string_is_still_refused():
    """The digit-string allowance must not become "anything goes"."""
    with pytest.raises(ConfigError):
        load_config(overrides=["macro_step.max_steps=abc"])


def test_an_unknown_key_is_refused():
    with pytest.raises(ConfigError):
        load_config(overrides=["macro_step.no_such_field=1"])


def test_a_scalar_override_still_works_at_the_top_level():
    cfg = load_config(overrides=["curriculum.stage=1"])
    assert cfg.curriculum.stage == 1
