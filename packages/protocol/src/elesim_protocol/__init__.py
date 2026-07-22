"""Versioned wire contract shared by independently deployed Elesim nodes."""

from .messages import *  # noqa: F401,F403
from .payloads import (
    CloseSimulationSessionRequest,
    DiscoverRequest,
    MotionCommandRequest,
    OpenSimulationSessionRequest,
    OperatorIntentRequest,
    OperatorViewSnapshot,
    RegisterRequest,
    SIMULATION_COMMANDS,
    SIMULATION_SCHEMA_VERSION,
    SIMULATION_STREAMS,
    SelectTargetRequest,
    SimulationCommandRequest,
    SimulationResultPayload,
    SimulationSessionGrantedPayload,
    SimulationSessionOpenedPayload,
    SimulationSessionRevokedPayload,
    SimulationStatusPayload,
    TelemetryPayload,
    TurnCredentials,
    WebRtcSignalPayload,
    validate_routed_payload,
)
from .operator import (
    OPERATOR_OPERATIONS,
    OPERATOR_VIEW_SCHEMA_VERSION,
    SERVICE_CALLS,
    SERVICE_VALUES,
    STATE_CALLS,
    STATE_VALUES,
)
from .serde import decode_value, encode_value, state_snapshot
from .security import (
    CurveClientConfig,
    CurveServerConfig,
    TransportSecurityError,
    configure_curve_client,
    configure_curve_server,
    endpoint_is_loopback,
    require_curve_server_auth,
    require_secure_remote,
)
from .transport import EndpointClient, EndpointSession, TransportError

__version__ = "0.2.0"
