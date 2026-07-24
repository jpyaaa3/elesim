"""Versioned wire contract shared by independently deployed Elesim nodes."""

from .authority import (
    AuthorityError,
    FenceDecision,
    IdempotencyCache,
    IdempotencyConflict,
    LeaseDecision,
    LeaseFence,
    MotionLease,
    MotionLeaseAuthority,
    SessionDecision,
    SimulationSession,
    SimulationSessionAuthority,
)
from .messages import *  # noqa: F401,F403
from .payloads import (
    CloseSimulationSessionRequest,
    DiscoverRequest,
    MotionCommandRequest,
    OpenSimulationSessionRequest,
    OperatorIntentRequest,
    OperatorViewSnapshot,
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
from .peer import (
    PeerAmbiguityError,
    PeerDescriptor,
    PeerDirectory,
    PeerDirectoryError,
    PeerError,
    PeerHeartbeat,
    PeerIdentity,
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
from .rgbd import (
    DdsRgbdPublisher,
    DdsRgbdSubscriber,
    RgbdIntrinsicsSample,
    RgbdSample,
)
from .transport import (
    DdsPeerNode,
    DdsRuntimeSettings,
    DdsTransportError,
    PeerClient,
)

__version__ = "0.3.0"
