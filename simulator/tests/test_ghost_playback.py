from __future__ import annotations

import numpy as np
import pytest

from elesim_simulator.robot.arm.planning.ghost_playback import build_ghost_stream


def test_build_ghost_stream_empty_waypoints_returns_empty() -> None:
    assert build_ghost_stream([], roll_rate=1.0, bend_rate=1.0, tick_hz=20.0) == []


def test_build_ghost_stream_single_waypoint_returns_it_unchanged() -> None:
    stream = build_ghost_stream([[0.1, 0.2, 0.3, 0.4]], roll_rate=1.0, bend_rate=1.0, tick_hz=20.0)
    assert len(stream) == 1
    assert np.allclose(stream[0], [0.1, 0.2, 0.3, 0.4])


def test_build_ghost_stream_starts_and_ends_at_the_given_waypoints() -> None:
    waypoints = [[0.0, 0.0, 0.0, 0.0], [-0.05, 0.6, 0.3, -0.3], [-0.1, 1.2, 0.5, -0.6]]
    stream = build_ghost_stream(waypoints, roll_rate=1.0, bend_rate=0.5, tick_hz=50.0)
    assert len(stream) > 2
    assert np.allclose(stream[0], waypoints[0])
    assert np.allclose(stream[-1], waypoints[-1])


def test_build_ghost_stream_never_exceeds_the_per_joint_rate_between_ticks() -> None:
    waypoints = [[0.0, 0.0, 0.0, 0.0], [-0.1, 1.5, 0.8, -0.8]]
    roll_rate, bend_rate, linear_rate = 1.0, 0.5, 0.02
    tick_hz = 50.0
    stream = build_ghost_stream(
        waypoints, roll_rate=roll_rate, bend_rate=bend_rate, linear_rate=linear_rate, tick_hz=tick_hz
    )
    max_step = np.array([linear_rate, roll_rate, bend_rate, bend_rate]) / tick_hz + 1e-9
    for prev, curr in zip(stream[:-1], stream[1:]):
        assert np.all(np.abs(np.asarray(curr) - np.asarray(prev)) <= max_step)


def test_build_ghost_stream_rejects_non_positive_tick_hz() -> None:
    with pytest.raises(ValueError):
        build_ghost_stream(
            [[0.0, 0.0, 0.0, 0.0], [0.1, 0.1, 0.1, 0.1]], roll_rate=1.0, bend_rate=1.0, tick_hz=0.0
        )
