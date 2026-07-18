from __future__ import annotations

from typing import Any

from elesim_protocol import (
    OPERATOR_OPERATIONS,
    SERVICE_CALLS,
    SERVICE_VALUES,
    STATE_CALLS,
    decode_value,
    encode_value,
    state_snapshot,
)


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
            elif operation == "service_call" and name in SERVICE_CALLS:
                result = getattr(self.service, name)(*args, **kwargs)
            elif operation == "service_get" and name in SERVICE_VALUES:
                result = getattr(self.service, name)
            elif operation == "state_call" and name in STATE_CALLS:
                result = getattr(self.state, name)(*args, **kwargs)
            elif operation == "state_set" and name and not name.startswith("_"):
                setattr(self.state, name, kwargs.get("value"))
                result = None
            else:
                raise ValueError(f"operation is not allowlisted: {operation} {name}")
            return {"request_id": request_id, "ok": True, "result": encode_value(result)}
        except Exception as exc:
            return {"request_id": request_id, "ok": False, "error": repr(exc)}
