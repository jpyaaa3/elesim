from __future__ import annotations

from dataclasses import replace

from elesim_protocol import Envelope, SimMappingConfig
from elesim_simulator.control_state import SimulationStateSource
from elesim_simulator.endpoint import SimulatorEndpoint


class Client:
    def __init__(self) -> None:
        self.sent: list[tuple[str, dict[str, object]]] = []

    def send(self, message_type: str, **kwargs: object) -> None:
        self.sent.append((message_type, kwargs))


def message(payload: dict[str, object], *, seq: int = 1, lease_id: str = "lease-a") -> Envelope:
    return Envelope(
        message_type="motion_command",
        source_id="controller-a",
        target_id="sim-a",
        payload=payload,
        seq=seq,
        timestamp=1.0,
        message_id=f"message-{seq}",
        lease_id=lease_id,
    )


def endpoint() -> tuple[SimulatorEndpoint, SimulationStateSource, Client]:
    state = SimulationStateSource(SimMappingConfig())
    client = Client()
    value = SimulatorEndpoint(
        server_endpoint="inproc://unused",
        endpoint_id="sim-a",
        state=state,
        streams={},
    )
    value.grant_lease("controller-a", "lease-a")
    return value, state, client


def test_endpoint_applies_only_fresh_commands_from_active_lease() -> None:
    value, state, client = endpoint()
    command = message({"command": "target", "q": [-0.1, 0.2, 0.3, -0.3]})

    value.handle_envelope(client, command)
    value.handle_envelope(client, command)
    value.handle_envelope(client, replace(command, seq=2, lease_id="wrong"))

    assert state.estimate_q().linear_m == -0.1
    assert [entry[1]["payload"]["reason"] for entry in client.sent] == [
        "target",
        "stale_sequence",
        "lease_mismatch",
    ]


def test_estop_is_accepted_without_a_lease() -> None:
    value, state, client = endpoint()
    state.apply_target({"command": "target", "go2_vel": [0.2, 0.0, 0.0]})
    value.revoke_lease()

    value.handle_envelope(client, message({"command": "estop"}, lease_id=""))

    assert state.go2_vel() == (0.0, 0.0, 0.0)
    assert client.sent[-1][1]["payload"]["ok"] is True


def test_runtime_telemetry_is_merged_and_sent_as_canonical_q() -> None:
    value, _state, client = endpoint()
    value.publish_telemetry({"q": [-0.1, 0.2, 0.3, -0.3], "actual_tip": [0.7, 0.0, 0.2]})
    value.publish_telemetry({"go2_base_pos": [0.0, 0.0, 0.3]})
    value.flush_telemetry(client)

    message_type, kwargs = client.sent[-1]
    assert message_type == "telemetry"
    assert kwargs["target_id"] == "controller-a"
    assert kwargs["lease_id"] == "lease-a"
    assert kwargs["payload"]["q"] == [-0.1, 0.2, 0.3, -0.3]
    assert kwargs["payload"]["go2_base_pos"] == [0.0, 0.0, 0.3]
