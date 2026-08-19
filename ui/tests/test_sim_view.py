from __future__ import annotations

from dataclasses import dataclass

from elesim_ui.sim_view import (
    OBSERVER_ASPECT,
    SimViewState,
    _center_crop_uv,
    _genesis_drag_zoom_delta,
    _genesis_scroll_zoom_delta,
    _fit_aspect_size,
    _mouse_delta_xy,
    _pip_rect,
    _scene_grab_delta,
)


@dataclass
class ImGuiVector:
    x: float
    y: float


def test_mouse_delta_accepts_imgui_vector_without_indexing_it() -> None:
    assert _mouse_delta_xy(ImGuiVector(3.5, -2.0)) == (3.5, -2.0)


def test_mouse_delta_accepts_tuple_and_rejects_unknown_shapes() -> None:
    assert _mouse_delta_xy((1, 2)) == (1.0, 2.0)
    assert _mouse_delta_xy(None) == (0.0, 0.0)


def test_camera_drag_uses_direct_grab_direction() -> None:
    assert _scene_grab_delta(100.0, -50.0, width=1000.0, height=500.0) == (
        -0.1,
        0.1,
    )


def test_view_state_swaps_named_streams() -> None:
    state = SimViewState()

    state.swap_streams()

    assert state.main_stream == "hand_eye_preview"
    assert state.preview_stream == "observer"


def test_genesis_wheel_zoom_uses_the_pinned_ninety_percent_ratio() -> None:
    assert _genesis_scroll_zoom_delta(1.0) < 0.0
    assert _genesis_scroll_zoom_delta(-1.0) > 0.0
    assert _genesis_scroll_zoom_delta(0.0) == 0.0


def test_genesis_right_drag_zoom_is_finite_and_bounded() -> None:
    assert _genesis_drag_zoom_delta(0.0, height=540.0) == 0.0
    assert -2.0 < _genesis_drag_zoom_delta(1.0, height=540.0) < 0.0
    assert _genesis_drag_zoom_delta(-1.0, height=540.0) > 0.0


def test_observer_display_is_fitted_to_genesis_four_by_three() -> None:
    width, height = _fit_aspect_size(960.0, 300.0, OBSERVER_ASPECT)

    assert width == 400.0
    assert height == 300.0
    assert width / height == OBSERVER_ASPECT


def test_fixed_observer_ratio_center_crops_non_four_by_three_source() -> None:
    uv0_x, uv0_y, uv1_x, uv1_y = _center_crop_uv(1920, 1080, OBSERVER_ASPECT)

    assert uv0_y == 0.0
    assert uv1_y == 1.0
    assert uv0_x > 0.0
    assert uv1_x < 1.0
    assert (uv1_x - uv0_x) / (uv1_y - uv0_y) == 1080 / 1440


def test_hand_eye_pip_is_inside_observer_at_upper_right() -> None:
    x0, y0, x1, y1 = _pip_rect((10.0, 20.0, 810.0, 620.0), source_aspect=OBSERVER_ASPECT)

    assert x0 > 10.0
    assert y0 > 20.0
    assert x1 < 810.0
    assert y1 < 620.0
    assert x1 > x0
    assert y1 > y0
    assert x1 == 798.0
    assert y0 == 32.0
