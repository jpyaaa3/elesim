from __future__ import annotations

import json

from elesim_controller.bridge import ControlBridge
from elesim_protocol import SimMappingConfig


class Endpoint:
    def __init__(self) -> None:
        self.sent = []

    def send(self, message_type: str, **kwargs: object) -> None:
        self.sent.append((message_type, kwargs))


class Socket:
    def __init__(self) -> None:
        self.sent = []

    def send_multipart(self, parts) -> None:
        self.sent.append(parts)


def test_partial_display_input_becomes_canonical_q() -> None:
    bridge = ControlBridge(
        local_endpoint="inproc://unused",
        server_endpoint="inproc://unused-router",
        controller_id="controller-a",
        mapping=SimMappingConfig(),
    )
    bridge.active_target = "robot-a"
    bridge.lease_id = "lease-a"
    endpoint = Endpoint()
    bridge._from_local(
        endpoint,
        Socket(),
        b"client",
        json.dumps({"t": "target", "u": {"roll": 200.0}}).encode("utf-8"),
    )
    message_type, kwargs = endpoint.sent[0]
    assert message_type == "motion_command"
    assert "u" not in kwargs["payload"]
    assert len(kwargs["payload"]["q"]) == 4
