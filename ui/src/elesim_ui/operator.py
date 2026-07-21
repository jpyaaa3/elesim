"""Cached, non-blocking proxies consumed by the ImGui panels."""

from __future__ import annotations

import dataclasses
from typing import Any, Callable

from elesim_protocol import (
    SERVICE_CALLS,
    SERVICE_VALUES,
    STATE_CALLS,
    STATE_VALUES,
    ControlU,
    SimMappingConfig,
)
from elesim_ui.models import (
    GazeStabilizerConfig,
    PanelStateDefaults,
    PickConfig,
)
from elesim_ui.operator_session import OperatorSession


_MISSING = object()


class RemotePanelState:
    def __init__(
        self,
        session: OperatorSession,
        *,
        initial_state: dict[str, Any] | None = None,
    ) -> None:
        object.__setattr__(self, "_session", session)
        values = dataclasses.asdict(PanelStateDefaults())
        values.update(initial_state or {})
        missing = {
            key: value
            for key, value in values.items()
            if session.state_value(key, _MISSING) is _MISSING
        }
        session.seed_state(missing)

    def sync(self) -> str:
        """Request a fresh snapshot without waiting for it."""
        return self._session.request_snapshot()

    def offset_values(self) -> tuple[float, float, float, float, int]:
        return (
            float(self._session.state_value("u_offset_linear", 0.0)),
            float(self._session.state_value("u_offset_roll", 0.0)),
            float(self._session.state_value("u_offset_s1", 0.0)),
            float(self._session.state_value("u_offset_s2", 0.0)),
            int(self._session.state_value("offset_revision", 0)),
        )

    def mock_object_world_xyz(self) -> tuple[float, float, float]:
        return (
            float(self._session.state_value("mock_object_x", 0.5)),
            float(self._session.state_value("mock_object_y", 0.0)),
            float(self._session.state_value("mock_object_z", 1.2)),
        )

    def mock_object_preferred_dir(self) -> tuple[float, float, float]:
        return (
            float(self._session.state_value("mock_object_dir_x", 1.0)),
            float(self._session.state_value("mock_object_dir_y", 0.0)),
            float(self._session.state_value("mock_object_dir_z", 0.0)),
        )

    def __getattr__(self, name: str) -> Any:
        if name in STATE_CALLS:
            return lambda *args, **kwargs: self._session.submit(
                "state_call", name, *args, **kwargs
            )
        value = self._session.state_value(name, _MISSING)
        if value is not _MISSING:
            return value
        raise AttributeError(name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        if name not in STATE_VALUES:
            raise AttributeError(f"state value is not writable through operator protocol: {name}")
        self._session.submit("state_set", name, value=value)


class RemoteControlService:
    def __init__(self, session: OperatorSession, state: RemotePanelState) -> None:
        self.session = session
        self.state = state

    def refresh_host_state(self) -> Any:
        self.session.request_snapshot()
        return self.current_host_state()

    def poll(self) -> None:
        self.session.dispatch_callbacks()

    def current_host_state(self) -> Any:
        return self.session.service_value("current_host_state")

    def has_client(self) -> bool:
        return bool(self.session.service_value("has_client", False))

    def current_control_u(self) -> ControlU:
        return self.session.service_value(
            "current_control_u",
            ControlU(u_linear=0.0, u_roll=0.0, u_s1=0.0, u_s2=0.0),
        )

    def control_mapping(self) -> SimMappingConfig:
        return self.session.service_value("control_mapping", SimMappingConfig())

    def pick_e2e_running(self) -> bool:
        return bool(self.session.service_value("pick_e2e_running", False))

    def _pick_config_effective(self) -> Any:
        return self.session.service_value("pick_config", PickConfig())

    @property
    def gaze_config(self) -> Any:
        return self.session.service_value("gaze_config", GazeStabilizerConfig())

    @property
    def _gaze_cfg(self) -> Any:
        return self.gaze_config

    @property
    def available_endpoints(self) -> list[Any]:
        return list(self.session.service_value("available_endpoints", ()))

    @property
    def active_endpoint(self) -> str:
        return str(self.session.service_value("active_endpoint", ""))

    def update_gaze_stabilizer_config(self, patch: dict[str, Any]) -> Any:
        self.session.submit("service_call", "update_gaze_stabilizer_config", patch)
        return self.gaze_config

    def load_sag_model_async(
        self,
        path: str,
        *,
        on_result: Callable[[Any], None],
        on_error: Callable[[str], None],
    ) -> str:
        return self.session.submit(
            "service_call",
            "load_sag_model",
            str(path),
            on_result=on_result,
            on_error=on_error,
            request_timeout_s=5.0,
        )

    def close(self) -> None:
        self.session.close()

    def __getattr__(self, name: str) -> Any:
        if name in SERVICE_CALLS:
            return lambda *args, **kwargs: self.session.submit(
                "service_call", name, *args, **kwargs
            )
        if name in SERVICE_VALUES:
            return self.session.service_value(name)
        raise AttributeError(name)
