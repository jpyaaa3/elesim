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

from elesim_protocol import SERVICE_CALLS, SERVICE_VALUES, STATE_CALLS

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
    assert not (SERVICE_CALLS & SERVICE_VALUES)
    assert not (SERVICE_CALLS & STATE_CALLS)
