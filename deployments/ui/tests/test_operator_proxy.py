from __future__ import annotations

from elesim_protocol import ControlU, SimMappingConfig
from elesim_ui.models import GazeStabilizerConfig, PickConfig
from elesim_ui.operator import RemoteControlService, RemotePanelState


class Session:
    def __init__(self) -> None:
        self.state = {
            "visual_target_scale": 0.16,
            "u_offset_linear": 1.0,
            "u_offset_roll": 2.0,
            "u_offset_s1": 3.0,
            "u_offset_s2": 4.0,
            "offset_revision": 7,
        }
        self.service = {
            "has_client": False,
            "current_control_u": ControlU(1.0, 2.0, 3.0, 4.0),
            "control_mapping": SimMappingConfig(),
            "gaze_config": GazeStabilizerConfig(),
            "pick_config": PickConfig(),
            "pick_e2e_running": False,
        }
        self.submitted: list[tuple[str, str, tuple, dict]] = []
        self.snapshot_requests = 0
        self.closed = False

    def seed_state(self, values: dict) -> None:
        self.state.update(values)

    def state_value(self, name: str, default=None):
        return self.state.get(name, default)

    def service_value(self, name: str, default=None):
        return self.service.get(name, default)

    def submit(self, operation: str, name: str = "", *args, **kwargs) -> str:
        self.submitted.append((operation, name, args, kwargs))
        return "request-a"

    def request_snapshot(self) -> str:
        self.snapshot_requests += 1
        return "snapshot-a"

    def close(self) -> None:
        self.closed = True


def test_remote_state_uses_cache_and_does_not_optimistically_commit_writes() -> None:
    session = Session()
    state = RemotePanelState(session)

    state.visual_target_scale = 0.25

    assert state.visual_target_scale == 0.16
    assert session.submitted[-1] == (
        "state_set",
        "visual_target_scale",
        (),
        {"value": 0.25},
    )
    assert state.offset_values() == (1.0, 2.0, 3.0, 4.0, 7)


def test_remote_service_reads_snapshot_cache_and_submits_commands_nonblocking() -> None:
    session = Session()
    state = RemotePanelState(session)
    service = RemoteControlService(session, state)

    assert service.has_client() is False
    assert service.current_control_u() == ControlU(1.0, 2.0, 3.0, 4.0)
    assert service.control_mapping() == SimMappingConfig()
    assert service.pick_e2e_running() is False

    service.stop_pick_e2e()
    assert session.submitted[-1][:2] == ("service_call", "stop_pick_e2e")
    assert session.closed is False

    assert service.refresh_host_state() is None
    assert session.snapshot_requests == 1
