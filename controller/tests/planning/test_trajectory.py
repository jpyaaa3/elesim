from __future__ import annotations

import numpy as np
import pytest

from elesim_controller.robot.arm.planning.trajectory import (
    JointRateLimits,
    resample,
    time_parameterize,
)

RATES = JointRateLimits(linear_m_s=0.02, roll_rad_s=2.0, theta1_rad_s=0.5, theta2_rad_s=0.5)


def test_time_parameterize_empty_waypoints_returns_empty_trajectory() -> None:
    trajectory = time_parameterize([], rates=RATES)
    assert trajectory.samples == []
    assert trajectory.duration_s == 0.0


def test_time_parameterize_single_waypoint_has_zero_duration() -> None:
    q = np.array([0.0, 0.0, 0.0, 0.0])
    trajectory = time_parameterize([q], rates=RATES)
    assert len(trajectory.samples) == 1
    assert trajectory.samples[0].t_s == 0.0
    assert trajectory.duration_s == 0.0


def test_time_parameterize_duration_is_bounded_by_the_slowest_joint() -> None:
    start = np.array([0.0, 0.0, 0.0, 0.0])
    roll_only = np.array([0.0, 1.0, 0.0, 0.0])  # 1.0 rad / 2.0 rad/s = 0.5s
    bend_and_roll = np.array([0.0, 1.5, 0.4, 0.0])  # roll: 0.75s, theta1: 0.8s -> 0.8s dominates

    trajectory = time_parameterize([start, roll_only, bend_and_roll], rates=RATES)

    assert trajectory.samples[1].t_s == pytest.approx(0.5)
    assert trajectory.samples[2].t_s == pytest.approx(0.5 + 0.8)


def test_time_parameterize_respects_hold_at_start() -> None:
    start = np.array([0.0, 0.0, 0.0, 0.0])
    end = np.array([0.0, 1.0, 0.0, 0.0])
    trajectory = time_parameterize([start, end], rates=RATES, hold_at_start_s=2.0)
    assert trajectory.samples[0].t_s == pytest.approx(2.0)
    assert trajectory.samples[1].t_s == pytest.approx(2.5)


def test_resample_empty_trajectory_returns_empty_list() -> None:
    trajectory = time_parameterize([], rates=RATES)
    assert resample(trajectory, tick_hz=20.0) == []


def test_resample_single_waypoint_returns_one_sample() -> None:
    q = np.array([0.01, 0.2, 0.1, -0.1])
    trajectory = time_parameterize([q], rates=RATES)
    samples = resample(trajectory, tick_hz=20.0)
    assert len(samples) == 1
    assert np.allclose(samples[0], q)


def test_resample_rejects_nonpositive_tick_hz() -> None:
    trajectory = time_parameterize(
        [np.zeros(4), np.array([0.0, 1.0, 0.0, 0.0])], rates=RATES
    )
    with pytest.raises(ValueError):
        resample(trajectory, tick_hz=0.0)


def test_resample_starts_and_ends_at_the_waypoint_endpoints() -> None:
    start = np.array([0.0, 0.0, 0.0, 0.0])
    end = np.array([0.0, 1.0, 0.0, 0.0])  # duration 0.5s at 2.0 rad/s
    trajectory = time_parameterize([start, end], rates=RATES)
    samples = resample(trajectory, tick_hz=10.0)

    assert np.allclose(samples[0], start)
    assert np.allclose(samples[-1], end)
    # 0.5s at 10Hz -> ticks at 0.0..0.5 inclusive = 6 samples
    assert len(samples) == 6


def test_resample_interpolates_linearly_mid_segment() -> None:
    start = np.array([0.0, 0.0, 0.0, 0.0])
    end = np.array([0.0, 1.0, 0.0, 0.0])  # duration 0.5s
    trajectory = time_parameterize([start, end], rates=RATES)
    samples = resample(trajectory, tick_hz=10.0)

    # tick index 3 -> t=0.3s -> 60% of the way from start to end
    assert samples[3][1] == pytest.approx(0.6, abs=1e-9)


def test_resample_tick_count_scales_with_rate() -> None:
    start = np.array([0.0, 0.0, 0.0, 0.0])
    end = np.array([0.0, 1.0, 0.0, 0.0])
    trajectory = time_parameterize([start, end], rates=RATES)
    samples_fast = resample(trajectory, tick_hz=100.0)
    samples_slow = resample(trajectory, tick_hz=10.0)
    assert len(samples_fast) > len(samples_slow)
