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
