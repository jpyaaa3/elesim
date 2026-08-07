from __future__ import annotations

import pytest

from elesim_simulator.runtime import _wall_with_hole_boxes


def test_wall_with_hole_boxes_returns_four_bars_matching_the_model_builder_geometry() -> None:
    bars = _wall_with_hole_boxes((0.45, 0.0, 0.3), 0.6, 0.6, 0.03, 0.25, 0.25)
    assert len(bars) == 4
    for center, size in bars:
        assert center[0] == pytest.approx(0.45)
        assert size[0] == pytest.approx(0.03)
    total_area = sum(size[1] * size[2] for _center, size in bars)
    assert total_area == pytest.approx(0.6 * 0.6 - 0.25 * 0.25)


def test_wall_with_hole_boxes_drops_a_bar_when_hole_touches_an_edge() -> None:
    bars = _wall_with_hole_boxes((0.0, 0.0, 0.0), 0.5, 0.5, 0.02, 0.5, 0.2)
    assert len(bars) == 2
