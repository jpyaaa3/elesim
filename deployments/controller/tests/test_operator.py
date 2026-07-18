from __future__ import annotations

from elesim_controller.operator import OperatorDispatcher


class State:
    value = 2

    def set_target(self, value: int) -> None:
        self.value = value


class Service:
    current_host_state = {"connected": True}

    def torque_on(self) -> str:
        return "on"


def test_dispatcher_executes_only_allowlisted_operations() -> None:
    state = State()
    dispatcher = OperatorDispatcher(state, Service())
    accepted = dispatcher.handle(
        {"request_id": "r1", "operation": "service_call", "name": "torque_on", "args": [], "kwargs": {}}
    )
    assert accepted == {"request_id": "r1", "ok": True, "result": "on"}

    rejected = dispatcher.handle(
        {"request_id": "r2", "operation": "service_call", "name": "__getattribute__", "args": [], "kwargs": {}}
    )
    assert rejected["ok"] is False


def test_dispatcher_requires_request_id() -> None:
    result = OperatorDispatcher(State(), Service()).handle({"operation": "snapshot"})
    assert result == {"request_id": "", "ok": False, "error": "missing_request_id"}
