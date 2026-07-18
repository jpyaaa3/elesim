from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from apps.controller.rpc import ControlRpcClient, ControlRpcServer, RemoteControlService, RemotePanelState


@dataclass
class _HostState:
    connected: bool
    value: int


class _State:
    def __init__(self) -> None:
        self.target_x = 1.0
        self.paused = False

    def set_paused(self, value: bool) -> None:
        self.paused = bool(value)


class _Service:
    def refresh_host_state(self) -> _HostState:
        return _HostState(True, 7)


def test_remote_ui_reads_and_mutates_agent_owned_state() -> None:
    endpoint = "inproc://control-rpc-contract"
    state = _State()
    server = ControlRpcServer(endpoint, state, _Service())
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    client = ControlRpcClient(endpoint)
    remote_state = RemotePanelState(client)
    remote_service = RemoteControlService(client, remote_state)
    assert remote_state.target_x == 1.0
    remote_state.target_x = 2.5
    assert state.target_x == 2.5
    remote_state.set_paused(True)
    remote_state.sync()
    assert remote_state.paused is True
    host = remote_service.refresh_host_state()
    assert host.connected is True
    assert host.value == 7
    client.close()
    server.close()
    thread.join(timeout=1.0)

