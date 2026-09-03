"""Unit tests for the per-environment object set.

Genesis bakes a morph's geometry at build time, so varying object size across
environments means building one cylinder per size and handing each env exactly
one.  What is pinned here is the bookkeeping that makes several entities behave
as one object: which env reads and writes which entity, and that the sizes an
env is not using end up out of reach.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest
import torch

from elesim_sim.rl.scene import ObjectSet


class _FakeEntity:
    """The slice of the Genesis rigid-entity API ObjectSet uses."""

    def __init__(self, n_envs: int, tag: float) -> None:
        self.pos = torch.full((n_envs, 3), tag)
        self.quat = torch.zeros(n_envs, 4)
        self.quat[:, 0] = 1.0
        self.links = [f"link{tag}"]

    def get_pos(self):
        return self.pos

    def get_quat(self):
        return self.quat

    def set_pos(self, value, envs_idx=None):
        if envs_idx is None:
            self.pos[:] = value
        else:
            self.pos[envs_idx] = value

    def set_quat(self, value, envs_idx=None):
        if envs_idx is None:
            self.quat[:] = value
        else:
            self.quat[envs_idx] = value


def _set(n_envs=4, radii=(0.045, 0.05, 0.06)):
    ents = [_FakeEntity(n_envs, float(i)) for i in range(len(radii))]
    s = ObjectSet(ents, radii, park_xy=(6.0, 6.0), park_step_m=0.6)
    s.bind(n_envs, torch.device("cpu"))
    return s


def test_reads_come_from_the_assigned_entity():
    s = _set()
    s.assignment[:] = torch.tensor([0, 2, 1, 2])
    # Each fake entity fills its position with its own index.
    assert s.get_pos()[:, 0].tolist() == [0.0, 2.0, 1.0, 2.0]


def test_writes_go_to_the_assigned_entity_only():
    s = _set()
    s.assignment[:] = torch.tensor([0, 1, 1, 2])
    s.set_pos(torch.full((4, 3), 9.0))
    assert s.entities[0].pos[:, 0].tolist() == [9.0, 0.0, 0.0, 0.0]
    assert s.entities[1].pos[:, 0].tolist() == [1.0, 9.0, 9.0, 1.0]
    assert s.entities[2].pos[:, 0].tolist() == [2.0, 2.0, 2.0, 9.0]


def test_a_partial_write_addresses_value_by_position_not_env_id():
    """`envs_idx` need not be sorted or contiguous.

    The reset writes only the environments that finished, and their ids arrive
    in whatever order; indexing the value buffer by env id instead of by
    position would silently write the wrong rows.
    """
    s = _set()
    s.assignment[:] = torch.tensor([0, 0, 0, 0])
    envs = torch.tensor([3, 1])
    s.set_pos(torch.tensor([[7.0, 0.0, 0.0], [5.0, 0.0, 0.0]]), envs_idx=envs)
    assert s.entities[0].pos[:, 0].tolist() == [0.0, 5.0, 0.0, 7.0]


def test_parking_moves_every_unused_size_away():
    s = _set()
    s.assignment[:] = torch.tensor([0, 1, 2, 0])
    s.park_unassigned(None)
    # env 0 uses entity 0, so entities 1 and 2 are parked there.
    assert s.entities[1].pos[0].tolist() == pytest.approx(s.park_pose(1))
    assert s.entities[2].pos[0].tolist() == pytest.approx(s.park_pose(2))
    # ...and entity 0 is left where it was for that env.
    assert s.entities[0].pos[0, 0].item() == 0.0


def test_parking_leaves_the_assigned_entity_alone():
    s = _set()
    s.assignment[:] = torch.tensor([1, 1, 1, 1])
    s.park_unassigned(None)
    assert s.entities[1].pos[:, 0].tolist() == [1.0, 1.0, 1.0, 1.0]


def test_park_poses_are_distinct():
    """Parked cylinders are free bodies; stacking them would knock them about."""
    s = _set()
    poses = [s.park_pose(k) for k in range(len(s.entities))]
    assert len(set(poses)) == len(poses)


def test_radius_of_follows_the_assignment():
    s = _set(radii=(0.045, 0.05, 0.06))
    s.assignment[:] = torch.tensor([2, 0, 1, 2])
    assert s.radius_of(s.assignment).tolist() == pytest.approx(
        [0.06, 0.045, 0.05, 0.06]
    )


def test_links_cover_every_entity():
    """The contact classifier has to recognise any of them as the object."""
    s = _set()
    assert len(s.links) == len(s.entities)


def test_bind_is_required_before_use():
    ents = [_FakeEntity(2, 0.0)]
    s = ObjectSet(ents, (0.05,), park_xy=(6.0, 6.0), park_step_m=0.6)
    with pytest.raises(RuntimeError):
        _ = s.assignment


def test_the_shipped_radii_are_evenly_spaced():
    """Sizes are drawn uniformly over what is built, so the spacing is the
    distribution.  Uneven spacing used to weight a size by how wide its snap
    basin happened to be: over 256 envs, 35 mm drew 16 and 87 mm drew 58 -- the
    hardest size getting the fewest episodes.
    """
    import elesim_sim.rl  # noqa: F401
    from elesim_sim.rl.configs.loader import load_config

    choices = sorted(load_config().object.radius_choices_m)
    gaps = [b - a for a, b in zip(choices, choices[1:])]
    assert max(gaps) - min(gaps) < 1e-6


def test_the_shipped_radii_span_what_the_arm_can_hold():
    """A range without built sizes to snap to is not randomisation.

    The first stage 3 run asked for 45-60 mm with only one 50 mm entity built,
    so every episode got 50 mm: a morph's geometry is fixed at build time, and
    `_reset_idx` snaps a sampled radius to the nearest one that exists.
    """
    import elesim_sim.rl  # noqa: F401
    from elesim_sim.rl.configs.loader import load_config

    cfg = load_config(overrides=["curriculum.stage=3"]).resolved_for_curriculum()
    built = sorted(cfg.object.radius_choices())
    lo, hi = cfg.domain_randomisation.object_radius_m
    assert len(built) > 1, "one entity means one size, whatever the range says"
    # The range has to be covered at both ends, or samples pile up on the edge.
    assert built[0] <= lo + 1e-9 and built[-1] >= hi - 1e-9
    # ...and measured: every size from 35 to 100 mm wraps and holds when
    # teleported into a wrap, so nothing outside that is worth building.
    assert built[0] >= 0.035 - 1e-9 and built[-1] <= 0.100 + 1e-9


def test_stage_2_still_pins_the_radius():
    """Sizes are stage 3's business; stage 2 randomises the pose only."""
    import elesim_sim.rl  # noqa: F401
    from elesim_sim.rl.configs.loader import load_config

    cfg = load_config(overrides=["curriculum.stage=2"]).resolved_for_curriculum()
    lo, hi = cfg.domain_randomisation.object_radius_m
    assert lo == hi == cfg.object.radius_m


def test_a_pinned_radius_survives_the_uniform_draw():
    """`_eval_override` asks for one size; drawing over the range discards it.

    It did: a 1728-episode evaluation reported a column of three radii while
    every condition sampled the whole 45-100 mm range, so the three "sizes" were
    the same mixture and came back at about 71% each -- and a clip row labelled
    45 mm showed three visibly different cylinders.
    """
    import elesim_sim.rl  # noqa: F401
    import torch
    from elesim_sim.rl.envs.wrap_env import choose_object_entity

    built = torch.tensor([0.050, 0.045, 0.056, 0.067, 0.078, 0.089, 0.100])
    for asked in (0.045, 0.067, 0.100):
        pinned = choose_object_entity(
            built, requested=torch.full((32,), asked),
            radius_range=(0.045, 0.100), randomise=False,
        )
        got = built[pinned]
        assert torch.allclose(got, torch.full((32,), asked)), f"asked {asked}"


def test_an_unpinned_size_is_drawn_over_the_built_sizes_in_range():
    import elesim_sim.rl  # noqa: F401
    import torch
    from elesim_sim.rl.envs.wrap_env import choose_object_entity

    built = torch.tensor([0.050, 0.045, 0.056, 0.067, 0.078, 0.089, 0.100])
    g = torch.Generator().manual_seed(0)
    idx = choose_object_entity(
        built, requested=torch.zeros(4000), radius_range=(0.045, 0.100),
        randomise=True, generator=g,
    )
    counts = torch.bincount(idx, minlength=len(built)).float()
    # Every built size in range is used, and none dominates: uniform over seven
    # is 571 of 4000, and snapping used to give one size 3.6x another.
    assert (counts > 0).all()
    assert counts.max() / counts.min() < 1.3


def test_a_range_narrower_than_the_built_sizes_selects_only_those_inside():
    import elesim_sim.rl  # noqa: F401
    import torch
    from elesim_sim.rl.envs.wrap_env import choose_object_entity

    built = torch.tensor([0.045, 0.067, 0.100])
    g = torch.Generator().manual_seed(0)
    idx = choose_object_entity(
        built, requested=torch.zeros(200), radius_range=(0.060, 0.105),
        randomise=True, generator=g,
    )
    assert {round(v * 1000) for v in built[idx].tolist()} == {67, 100}


def test_a_range_no_entity_satisfies_falls_back_to_the_nearest():
    import elesim_sim.rl  # noqa: F401
    import torch
    from elesim_sim.rl.envs.wrap_env import choose_object_entity

    built = torch.tensor([0.045, 0.067, 0.100])
    g = torch.Generator().manual_seed(0)
    idx = choose_object_entity(
        built, requested=torch.zeros(50), radius_range=(0.070, 0.075),
        randomise=True, generator=g,
    )
    assert {round(v * 1000) for v in built[idx].tolist()} == {67}
