from __future__ import annotations

from dataclasses import dataclass

from elesim_ui.simulator_view import SimulatorViewState, _mouse_delta_xy


@dataclass
class ImGuiVector:
    x: float
    y: float


def test_mouse_delta_accepts_imgui_vector_without_indexing_it() -> None:
    assert _mouse_delta_xy(ImGuiVector(3.5, -2.0)) == (3.5, -2.0)


def test_mouse_delta_accepts_tuple_and_rejects_unknown_shapes() -> None:
    assert _mouse_delta_xy((1, 2)) == (1.0, 2.0)
    assert _mouse_delta_xy(None) == (0.0, 0.0)


def test_view_state_swaps_named_streams() -> None:
    state = SimulatorViewState()

    state.swap_streams()

    assert state.main_stream == "hand_eye_preview"
    assert state.preview_stream == "observer"
