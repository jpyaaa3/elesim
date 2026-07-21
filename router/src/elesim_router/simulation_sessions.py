"""Simulation operator leases and ephemeral TURN credentials."""

from __future__ import annotations

import base64
import hashlib
import hmac
import uuid
from dataclasses import dataclass
from typing import Optional

from elesim_protocol import TurnCredentials


class SimulationSessionError(RuntimeError):
    pass


@dataclass
class SimulationSession:
    session_id: str
    request_id: str
    ui_id: str
    simulator_id: str
    streams: tuple[str, ...]
    turn_expires_at: float = 0.0


class SimulationSessionRegistry:
    """Exclusive UI-to-simulator control leases, separate from motion leases."""

    def __init__(self) -> None:
        self.by_id: dict[str, SimulationSession] = {}
        self.by_ui: dict[str, SimulationSession] = {}
        self.by_simulator: dict[str, SimulationSession] = {}

    def open(
        self,
        *,
        request_id: str,
        ui_id: str,
        simulator_id: str,
        streams: tuple[str, ...],
    ) -> SimulationSession:
        current_ui = self.by_ui.get(ui_id)
        if current_ui is not None:
            if current_ui.simulator_id == simulator_id and current_ui.streams == streams:
                return current_ui
            raise SimulationSessionError("UI already owns a simulation session")
        current_simulator = self.by_simulator.get(simulator_id)
        if current_simulator is not None:
            raise SimulationSessionError("simulator already has an operator session")
        session = SimulationSession(
            session_id=uuid.uuid4().hex,
            request_id=request_id,
            ui_id=ui_id,
            simulator_id=simulator_id,
            streams=tuple(streams),
        )
        self.by_id[session.session_id] = session
        self.by_ui[session.ui_id] = session
        self.by_simulator[session.simulator_id] = session
        return session

    def close(self, session_id: str) -> Optional[SimulationSession]:
        session = self.by_id.pop(str(session_id), None)
        if session is None:
            return None
        self.by_ui.pop(session.ui_id, None)
        self.by_simulator.pop(session.simulator_id, None)
        return session

    def close_for_endpoint(self, endpoint_id: str) -> Optional[SimulationSession]:
        session = self.by_ui.get(endpoint_id) or self.by_simulator.get(endpoint_id)
        return None if session is None else self.close(session.session_id)


class TurnCredentialIssuer:
    """Issue coturn TURN REST API credentials using its shared HMAC secret."""

    def __init__(
        self,
        *,
        urls: tuple[str, ...],
        static_auth_secret: bytes | str,
        ttl_s: int = 3600,
        refresh_before_s: int = 600,
    ) -> None:
        self.urls = tuple(str(url).strip() for url in urls if str(url).strip())
        self.static_auth_secret = (
            static_auth_secret.encode("utf-8")
            if isinstance(static_auth_secret, str)
            else bytes(static_auth_secret)
        )
        self.ttl_s = max(900, int(ttl_s))
        self.refresh_before_s = max(60, min(int(refresh_before_s), self.ttl_s - 60))
        if not self.urls:
            raise ValueError("at least one TURN URL is required")
        if not self.static_auth_secret:
            raise ValueError("TURN static auth secret must not be empty")

    def issue(self, endpoint_id: str, session_id: str, *, now: float) -> TurnCredentials:
        expires = int(float(now)) + self.ttl_s
        username = f"{expires}:{endpoint_id}:{session_id}"
        digest = hmac.new(
            self.static_auth_secret,
            username.encode("utf-8"),
            hashlib.sha1,
        ).digest()
        return TurnCredentials(
            urls=self.urls,
            username=username,
            credential=base64.b64encode(digest).decode("ascii"),
            expires_at=float(expires),
        )

    def refresh_due(self, expires_at: float, *, now: float) -> bool:
        return float(expires_at) - float(now) <= float(self.refresh_before_s)


__all__ = [
    "SimulationSession",
    "SimulationSessionError",
    "SimulationSessionRegistry",
    "TurnCredentialIssuer",
]
