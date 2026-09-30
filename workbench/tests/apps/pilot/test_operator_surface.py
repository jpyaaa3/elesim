"""The operator surface has to be callable, not just allowlisted.

`reset_simulation` was decorated `@staticmethod` while still taking `self`, so
the dispatcher's `getattr(service, name)(*args)` reached an unbound function and
raised "missing 1 required positional argument: 'self'".  The dispatcher
returned that to the UI and logged nothing, so Respawn looked inert: the intent
arrived, no handler line followed, and there was no way to tell a dropped
request from a raising one.

Being in SERVICE_CALLS is therefore not enough -- the name has to resolve to
something the dispatcher can actually call.
"""

from __future__ import annotations

import inspect
import threading

from elesim_protocol import SERVICE_CALLS, STATE_VALUES, STATE_CALLS

from elesim_pilot.pick.actions import ControlService


def _static_taking_self(cls, name):
    """Names on `cls` that are staticmethods whose first parameter is `self`."""
    raw = inspect.getattr_static(cls, name, None)
    if not isinstance(raw, staticmethod):
        return False
    params = list(inspect.signature(raw.__func__).parameters)
    return bool(params) and params[0] == "self"


def test_no_service_call_is_a_staticmethod_taking_self():
    offenders = [
        name for name in sorted(SERVICE_CALLS)
        if hasattr(ControlService, name) and _static_taking_self(ControlService, name)
    ]
    assert offenders == [], (
        "these resolve to unbound functions, so the dispatcher cannot call them: "
        f"{offenders}"
    )


def test_reset_simulation_binds_to_the_instance():
    raw = inspect.getattr_static(ControlService, "reset_simulation")
    assert not isinstance(raw, staticmethod)
    assert list(inspect.signature(raw).parameters)[:1] == ["self"]


def test_respawn_sends_sim_reset_before_cleanup_and_fences_old_commands():
    from elesim_protocol import SimMappingConfig

    events: list[object] = []

    class State:
        controls_locked = False
        linear = roll = theta1 = theta2 = 0.0
        target_x = target_y = target_z = 0.0
        target_vx = target_vy = target_vz = 0.0
        claw_closed = False

        def set_q(self, *values):
            events.append(("set_q", values))

        def set_claw_closed(self, value):
            events.append(("claw_closed", value))

        def clear_ik_status(self):
            events.append("clear_ik")

        def set_pick_status(self, **values):
            events.append(("pick_status", values))

    class Client:
        def send_sim_reset(self):
            events.append("sim_reset")

        def send_go2_velocity(self, **values):
            events.append(("go2_velocity", values))

        def send_target_values(self, **values):
            events.append(("arm_target", values))

    service = object.__new__(ControlService)
    service._sim_reset_lock = threading.RLock()
    service._sim_reset_in_progress = threading.Event()
    service._pick_e2e_cancel = threading.Event()
    service._pick_stop_event = threading.Event()
    service._mapping_cfg = SimMappingConfig()
    service.client = Client()
    service.state = State()
    service._gaze_service = type("Gaze", (), {"request_stop": lambda self: events.append("signal_gaze_stop")})()
    service.stop_object_pick = lambda: events.append("stop_object_pick")
    service._schedule_sim_reset_cleanup = lambda: events.append("schedule_cleanup")

    service.reset_simulation()

    assert events[0] == "sim_reset"
    assert "schedule_cleanup" in events
    assert "signal_gaze_stop" in events
    assert "stop_object_pick" in events
    assert service._sim_reset_in_progress.is_set()
    assert not any(
        isinstance(event, tuple) and event[0] == "go2_velocity"
        for event in events
    )
    assert not any(
        isinstance(event, tuple) and event[0] == "arm_target"
        for event in events
    )

    # A cancelled IK/pick worker must not write a stale arm target either.
    service.send_current_target(source="target", force=True)
    assert not any(
        isinstance(event, tuple) and event[0] == "arm_target"
        for event in events
    )

    service.send_go2_velocity(vx=0.1, vy=0.0, wz=0.0)
    assert not any(
        isinstance(event, tuple) and event[0] == "go2_velocity"
        for event in events
    )

    service._sim_reset_in_progress.clear()
    service.send_go2_velocity(vx=0.1, vy=0.0, wz=0.0)
    assert events[-1][0] == "go2_velocity"


def test_respawn_signal_interrupts_demo_wait_without_starting_its_next_workflow():
    from types import SimpleNamespace

    from elesim_pilot.gaze.gaze_service import GazeControlService

    events: list[str] = []
    started = threading.Event()
    service = object.__new__(GazeControlService)
    service._demo_thread = None
    service._demo_stop = threading.Event()
    service._stop = threading.Event()
    service.start_walking_gaze = lambda: (events.append("walking_gaze"), started.set())
    service._parent = SimpleNamespace(
        send_go2_velocity=lambda **_kwargs: events.append("zero_velocity"),
        start_mobile_gaze_lji_pick_e2e=lambda: events.append("pick_workflow"),
        start_look_aim_grasp_e2e=lambda: events.append("fallback_workflow"),
    )

    service.start_stop_and_grasp_demo()
    worker = service._demo_thread
    assert started.wait(timeout=1.0)
    service.request_stop()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert events == ["walking_gaze"]


def test_respawn_cleanup_keeps_motion_fenced_until_workers_exit():
    service = object.__new__(ControlService)
    service._sim_reset_lock = threading.RLock()
    service._sim_reset_in_progress = threading.Event()
    service._sim_reset_in_progress.set()
    service._sim_reset_cleanup_pending = True
    service._sim_reset_cleanup_thread = threading.current_thread()
    service._gaze_service = type("Gaze", (), {"stop": lambda self: None})()
    service.stop_pick_e2e = lambda: None
    service._sim_reset_live_workers = lambda: []
    service._reset_pilot_readback_after_sim_reset = lambda: None

    service._finish_sim_reset_cleanup()

    assert service._sim_reset_in_progress.is_set() is False
    assert service._sim_reset_cleanup_pending is False
    assert service._sim_reset_cleanup_thread is None


def test_every_service_call_the_pilot_advertises_exists():
    """A name in the allowlist the pilot cannot resolve fails only when pressed."""
    missing = [
        name for name in sorted(SERVICE_CALLS)
        if not hasattr(ControlService, name)
    ]
    # `select_endpoint` and the mock-hug calls live on the facade, not the
    # service, so they are expected to be absent here.  Importing the facade
    # pulls in the hug geometry stack, which needs shapely; skip rather than
    # fail where it is not installed.
    pytest = __import__("pytest")
    try:
        from elesim_pilot.main import _ControlFacade
    except ImportError as exc:
        pytest.skip(f"facade import unavailable: {exc}")
    unresolved = [name for name in missing if not hasattr(_ControlFacade, name)]
    assert unresolved == [], f"no pilot object provides: {unresolved}"


def test_the_dispatcher_reports_a_failing_call_rather_than_swallowing_it():
    from elesim_pilot.operator import OperatorDispatcher

    class _Boom:
        def torque_off(self):
            raise RuntimeError("nope")

    result = OperatorDispatcher(object(), _Boom()).handle(
        {"request_id": "r1", "operation": "service_call", "name": "torque_off"}
    )
    assert result["ok"] is False
    assert "nope" in result["error"]


def test_allowlists_do_not_overlap():
    """A name in two lists is dispatched by whichever branch is checked first."""
    assert not (SERVICE_CALLS & STATE_VALUES)
    assert not (SERVICE_CALLS & STATE_CALLS)
