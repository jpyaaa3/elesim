"""Unit tests for the stop-or-continue verdict.

The point of the tool is to stop a run that has nothing left to gain, and the
two ways it can be wrong both waste real time: calling a plateau early throws
away improvement, and missing one burns hours -- a Mac run spent 1600 iterations
moving success by 0.03 points.  So the tests pin the verdict on each shape of
series rather than the formatting.
"""

from __future__ import annotations

import math

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest

from elesim_sim.rl.progress import read, series_from_log, verdict


def _data(success, phi, curriculum=None):
    return {
        "iteration": [float(i) for i in range(len(success))],
        "success": list(success),
        "phi": list(phi),
        "curriculum": list(curriculum or []),
    }


def test_a_flat_run_is_told_to_stop():
    flat = [0.11] * 20
    call, _ = verdict(_data(flat, [1.45] * 20, [0.0] * 20), window=5)
    assert "멈춰도" in call


def test_a_rising_success_rate_keeps_going():
    rising = [0.02 * i for i in range(20)]
    call, _ = verdict(_data(rising, [1.45] * 20, [0.0] * 20), window=5)
    assert "계속" in call


def test_a_rising_wrap_angle_keeps_going_even_if_success_is_flat():
    """Success can sit still while the wrap angle climbs towards the gate.

    That happened for 1500 iterations: the rate held at 10% while Phi went from
    1.02 to 1.35 rad, and stopping on the rate alone would have cut it off.
    """
    call, _ = verdict(
        _data([0.11] * 20, [1.0 + 0.05 * i for i in range(20)], [0.0] * 20), window=5
    )
    assert "계속" in call


def test_an_unfinished_curriculum_keeps_going_however_flat():
    """A flat plateau at an easy start point is not convergence.

    The episodes are still beginning most of the way to the goal, so the number
    is not the task's.
    """
    call, lines = verdict(_data([0.5] * 20, [1.45] * 20, [0.4] * 20), window=5)
    assert "계속" in call
    assert "물러나지" in call
    # ...and the curriculum line says where it got to, not just that it did not.
    assert any("0.40" in line for line in lines)


def test_too_little_data_is_reported_as_such_not_as_a_plateau():
    call, _ = verdict(_data([0.11] * 4, [1.45] * 4, [0.0] * 4), window=5)
    assert "계속" in call
    assert "모이지" in call


def test_a_declining_run_is_not_called_flat():
    """A decline is a change, so it is not convergence."""
    falling = [0.30 - 0.02 * i for i in range(20)]
    call, _ = verdict(_data(falling, [1.45] * 20, [0.0] * 20), window=5)
    assert "계속" in call


def test_log_parsing_picks_up_every_occurrence():
    text = (
        "term/success: 0.0100\nwrap/phi_rad: 1.200\n"
        "term/success: 0.0200\nwrap/phi_rad: 1.300\n"
    )
    assert series_from_log(text, "term/success") == [0.01, 0.02]
    assert series_from_log(text, "wrap/phi_rad") == [1.2, 1.3]


def test_a_curve_csv_reads_as_radians(tmp_path):
    """`watch_eval` writes degrees; the thresholds are in radians."""
    path = tmp_path / "curve.csv"
    path.write_text(
        "iteration,episodes,success_rate,collision,topple,retention,no_wrap,"
        "no_reach,phi_mean_deg,phi_max_deg,checkpoint\n"
        "200,256,0.7060,0.2,0.0,0.03,0.02,0.0,28.6,200.0,model_200.pt\n"
        "100,256,0.5000,0.3,0.0,0.03,0.02,0.0,20.0,190.0,model_100.pt\n",
        encoding="utf-8",
    )
    data = read(path)
    # Sorted by iteration, not by file order.
    assert data["iteration"] == [100.0, 200.0]
    assert data["success"] == [0.5, 0.706]
    assert data["phi"][1] == pytest.approx(math.radians(28.6), abs=1e-6)
