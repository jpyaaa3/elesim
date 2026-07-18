from __future__ import annotations

from elesim_protocol import SimMappingConfig
from elesim_simulator.bridge import SimProtocolBridge


def bridge() -> SimProtocolBridge:
    return SimProtocolBridge(
        server_endpoint="inproc://router",
        endpoint_id="sim-a",
        legacy_state_bind="inproc://state",
        legacy_feedback_bind="inproc://feedback",
        mapping=SimMappingConfig(),
    )


def test_simulator_accepts_canonical_q_list() -> None:
    runtime = bridge()
    ok, reason = runtime._apply_command({"command": "target", "q": [-0.1, 0.2, 0.3, -0.3]})
    assert (ok, reason) == (True, "target")
    assert runtime.current_q.linear_m == -0.1


def test_simulator_rejects_legacy_u() -> None:
    runtime = bridge()
    ok, reason = runtime._apply_command({"command": "target", "u": {"linear": 20}})
    assert (ok, reason) == (False, "legacy_u_not_supported")
