from __future__ import annotations

from typing import Any

from elesim_protocol import (
    OPERATOR_OPERATIONS,
    OperatorViewSnapshot,
    SERVICE_CALLS,
    SERVICE_VALUES,
    STATE_CALLS,
    STATE_VALUES,
    decode_value,
    encode_value,
    state_snapshot,
)


#: Operations the UI polls continuously.  Everything else is a person
#: pressing something, which is worth a line in the log.
_POLL_OPERATIONS = frozenset({"snapshot", "view_snapshot"})


class OperatorDispatcher:
    """Execute the protocol's explicitly allowlisted UI intent surface."""

    def __init__(self, state: Any, service: Any) -> None:
        self.state = state
        self.service = service

    def handle(self, payload: dict[str, Any]) -> dict[str, Any]:
        request_id = str(payload.get("request_id", ""))
        operation = str(payload.get("operation", ""))
        name = str(payload.get("name", ""))
        if not request_id:
            return {"request_id": "", "ok": False, "error": "missing_request_id"}
        if operation not in OPERATOR_OPERATIONS:
            return {"request_id": request_id, "ok": False, "error": "unsupported_operation"}
        try:
            args = [decode_value(value) for value in payload.get("args", [])]
            kwargs = {
                str(key): decode_value(value)
                for key, value in dict(payload.get("kwargs", {})).items()
            }
            if operation == "snapshot":
                result = state_snapshot(self.state)
            elif operation == "view_snapshot":
                result = self._view_snapshot()
            elif operation == "service_call" and name in SERVICE_CALLS:
                result = getattr(self.service, name)(*args, **kwargs)
            elif operation == "service_get" and name in SERVICE_VALUES:
                result = getattr(self.service, name)
            elif operation == "state_call" and name in STATE_CALLS:
                result = getattr(self.state, name)(*args, **kwargs)
            elif operation == "state_set" and name in STATE_VALUES:
                setattr(self.state, name, kwargs.get("value"))
                result = None
            else:
                raise ValueError(f"operation is not allowlisted: {operation} {name}")
            if operation not in _POLL_OPERATIONS:
                print(f"[operator] {operation} {name} ok", flush=True)
            return {"request_id": request_id, "ok": True, "result": encode_value(result)}
        except Exception as exc:
            # Returning the reason to the UI and printing nothing here left the
            # pilot log silent whenever a command failed: an intent arrived, no
            # handler line followed, and there was no way to tell a dropped
            # request from a raising one.  Both look like "the button does
            # nothing".
            print(f"[operator] {operation} {name} FAILED: {exc!r}", flush=True)
            return {"request_id": request_id, "ok": False, "error": repr(exc)}

    def _view_snapshot(self) -> dict[str, Any]:
        host_state = self.service.refresh_host_state()
        service = {
            "current_host_state": host_state,
            "has_client": self.service.has_client(),
            "current_control_u": self.service.current_control_u(),
            "control_mapping": self.service.control_mapping(),
            "gaze_config": self.service.gaze_config,
            "pick_config": self.service._pick_config_effective(),
            "pick_e2e_running": self.service.pick_e2e_running(),
            "wrap_grasp_running": self.service.wrap_grasp_running(),
            "wrap_grasp_result": self.service.wrap_grasp_result(),
            "available_endpoints": self.service.available_endpoints,
            "active_endpoint": self.service.active_endpoint,
        }
        return OperatorViewSnapshot(
            state=state_snapshot(self.state),
            service=service,
        ).to_payload()
