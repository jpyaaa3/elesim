from types import SimpleNamespace as NS

import pytest

from elesim_protocol import SimulationCommandRequest, ProtocolError
from elesim_sim.simulation.ik_marker import IkTargetMarker
from elesim_sim.simulation.operator_control import SimulationOperatorCommand, SimulationOperatorController
from elesim_sim.vision.sim_camera.async_worker import CameraStateSnapshot


def request(arguments):
    return SimulationCommandRequest.from_payload(dict(schema_version=1, request_id="r",
        session_id="s", command="set_ik_target", arguments=arguments))


def test_goal_display_does_not_reset_or_move_simulation():
    mutations = []
    controller = SimulationOperatorController(reset_environment=lambda: mutations.append("reset"),
        observer_command=lambda *args: mutations.append("camera"))
    r = request({"position": [.1, -.2, .3], "direction": [0, 0, 1]})
    c = SimulationOperatorCommand.from_request(r, ui_id="ui")
    assert controller.apply(c)[0]
    assert controller.ik_target_position == (.1, -.2, .3)
    assert not mutations
    assert controller.paused is False and controller.epoch == 0
    snapshot = CameraStateSnapshot(epoch=0, sim_step=0, sim_time_s=0,
        ik_target_position=controller.ik_target_position,
        ik_target_direction=controller.ik_target_direction, debug_visible=False)
    assert snapshot.ik_target_position == (.1, -.2, .3)
    assert snapshot.debug_visible is False


@pytest.mark.parametrize("arguments", [
    {"position": [float("nan"), 0, 0], "direction": [0, 0, 1]},
    {"position": [11, 0, 0], "direction": [0, 0, 1]},
    {"position": [0, 0], "direction": [0, 0, 1]},
    {"position": [0, 0, 0], "direction": [0, 0, 2]},
    {"position": [0, 0, 0], "direction": [0, 0, 1], "q": [0]*4},
])
def test_display_command_rejects_nonfinite_unbounded_and_motion_fields(arguments):
    with pytest.raises(ProtocolError):
        request(arguments)


def test_scene_markers_update_and_clear_without_growing_each_frame():
    drawn, cleared = [], []
    def draw(**kwargs):
        drawn.append(kwargs)
        return len(drawn)
    scene = NS(draw_debug_sphere=draw, draw_debug_arrow=draw, clear_debug_object=cleared.append)
    marker = IkTargetMarker()
    marker.update(scene, [.1, .2, .3], [0, 0, 1])
    assert len(drawn) == 2
    for _ in range(5): marker.update(scene, [.1, .2, .3], [0, 0, 2])
    assert len(drawn) == 2  # same normalized direction
    marker.update(scene, [.1, .2, .4], [0, 0, 1])
    assert len(drawn) == 4 and cleared == [1, 2]
    marker.update(scene, [.1, .2, .4], visible=False)
    assert cleared == [1, 2, 3, 4]
    assert not marker.handles


def test_paused_display_state_is_ready_before_camera_submission():
    from elesim_sim.runtime import SimRuntime
    updates = []
    scene = NS(scene=object())
    runtime = NS(app=NS(spawn=NS(draw_debug_markers=True), sim_scene=scene, state_source=None),
        operator=NS(debug_visible=True, ik_target_position=(.1, .2, .3),
                    ik_target_direction=(0., 0., 1.), paused=True,
                    arm_ghost=None, arm_ghost_started_at=0.),
        _ghost_frame=None, _arm_ghost=NS(update=lambda *args, **kwargs:None),
        _ik_marker=NS(update=lambda *args, **kwargs: updates.append((args, kwargs))))
    SimRuntime._update_ik_target_marker(runtime)
    assert scene.ik_target_position == (.1, .2, .3)
    assert scene.ik_target_direction == (0., 0., 1.)
    assert scene.ik_debug_visible is True
    assert len(updates) == 1
