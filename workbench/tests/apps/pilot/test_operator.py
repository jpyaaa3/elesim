from __future__ import annotations

import math

from elesim_pilot.operator import OperatorDispatcher
from elesim_protocol import OPERATOR_VIEW_SCHEMA_VERSION, decode_value


class State:
    value = 2

    def set_target(self, value: int) -> None:
        self.value = value


class Service:
    current_host_state = {"connected": True}

    def torque_on(self) -> str:
        return "on"

    def select_imu_model(self, model_id: str) -> dict[str, object]:
        return {"requested": model_id, "active": False}


class ViewState:
    def __init__(self) -> None:
        self.pick_running = True
        self.visual_target_scale = 0.2


class ViewService:
    def __init__(self) -> None:
        self.refresh_count = 0
        self.gaze_config = {"walking_gaze_mode": "uv_ff"}
        self.available_endpoints = [{"endpoint_id": "sim-a", "role": "sim"}]
        self.active_endpoint = "sim-a"
        self.imu_models = [{"id": "identity", "version": 1, "label": "Identity"}]

    def imu_model_status(self):
        return {"requested": {}, "active": False, "error": ""}

    def refresh_host_state(self):
        self.refresh_count += 1
        return {"connected": True, "device": "virtual"}

    def has_client(self) -> bool:
        return True

    def current_control_u(self):
        return {"u_linear": 0.0, "u_roll": 1.0, "u_s1": 2.0, "u_s2": 3.0}

    def control_mapping(self):
        return {"linear_u_min": 0.0, "linear_u_max": 250.0}

    def pick_e2e_running(self) -> bool:
        return True

    def wrap_grasp_running(self) -> bool:
        return False

    def wrap_grasp_result(self) -> str:
        return ""

    def _pick_config_effective(self):
        return {"mobile_handoff_distance_m": 0.3}


def test_dispatcher_executes_only_allowlisted_operations() -> None:
    state = State()
    dispatcher = OperatorDispatcher(state, Service())
    accepted = dispatcher.handle(
        {"request_id": "r1", "operation": "service_call", "name": "torque_on", "args": [], "kwargs": {}}
    )
    assert accepted == {"request_id": "r1", "ok": True, "result": "on"}

    model_result = dispatcher.handle(
        {"request_id": "model-1", "operation": "service_call", "name": "select_imu_model",
         "args": ["identity"], "kwargs": {}}
    )
    assert model_result["ok"] is True
    assert decode_value(model_result["result"]) == {"requested": "identity", "active": False}

    rejected = dispatcher.handle(
        {"request_id": "r2", "operation": "service_call", "name": "__getattribute__", "args": [], "kwargs": {}}
    )
    assert rejected["ok"] is False


def test_dispatcher_rejects_new_operator_commands_until_respawn_cleanup_finishes() -> None:
    class ResettingService(Service):
        sim_reset_in_progress = True

        def reset_simulation(self):
            return "reset requested"

    dispatcher = OperatorDispatcher(State(), ResettingService())
    blocked = dispatcher.handle(
        {
            "request_id": "walk-1",
            "operation": "service_call",
            "name": "torque_on",
            "args": [],
            "kwargs": {},
        }
    )
    allowed_reset = dispatcher.handle(
        {
            "request_id": "reset-1",
            "operation": "service_call",
            "name": "reset_simulation",
            "args": [],
            "kwargs": {},
        }
    )

    assert blocked["ok"] is False
    assert "respawn is still stopping" in blocked["error"]
    assert allowed_reset["ok"] is True


def test_dispatcher_requires_request_id() -> None:
    result = OperatorDispatcher(State(), Service()).handle({"operation": "view_snapshot"})
    assert result == {"request_id": "", "ok": False, "error": "missing_request_id"}


def test_state_set_is_limited_to_the_protocol_allowlist() -> None:
    state = State()
    result = OperatorDispatcher(state, Service()).handle(
        {
            "request_id": "r3",
            "operation": "state_set",
            "name": "unpublished_internal_value",
            "kwargs": {"value": 99},
        }
    )

    assert result["ok"] is False
    assert not hasattr(state, "unpublished_internal_value")


def test_view_snapshot_refreshes_once_and_returns_explicit_ui_read_model() -> None:
    service = ViewService()
    result = OperatorDispatcher(ViewState(), service).handle(
        {
            "request_id": "view-1",
            "operation": "view_snapshot",
            "name": "",
            "args": [],
            "kwargs": {},
        }
    )

    assert result["ok"] is True
    view = decode_value(result["result"])
    assert view["schema_version"] == OPERATOR_VIEW_SCHEMA_VERSION
    assert view["state"]["pick_running"] is True
    assert view["service"]["current_host_state"]["connected"] is True
    assert view["service"]["has_client"] is True
    assert view["service"]["active_endpoint"] == "sim-a"
    assert view["service"]["imu_models"][0]["id"] == "identity"
    assert view["service"]["imu_model_status"]["active"] is False
    assert view["service"]["pick_config"]["mobile_handoff_distance_m"] == 0.3
    assert service.refresh_count == 1


def test_dispatcher_turns_nonfinite_results_into_a_bounded_error() -> None:
    service = ViewService()
    service.refresh_host_state = lambda: {"connected": False, "rx_age_s": math.inf}

    result = OperatorDispatcher(ViewState(), service).handle(
        {
            "request_id": "view-nonfinite",
            "operation": "view_snapshot",
            "name": "",
            "args": [],
            "kwargs": {},
        }
    )

    assert result["request_id"] == "view-nonfinite"
    assert result["ok"] is False
    assert "non-finite" in result["error"]
