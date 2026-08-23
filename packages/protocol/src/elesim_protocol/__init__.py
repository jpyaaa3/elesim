"""Versioned wire contract shared by independently deployed EleSim nodes."""

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
from .contracts import DDS_CONTRACTS, DdsContract, contract_for, validate_registry
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
    RgbdFrame,
    RgbdIntrinsics,
    RgbdIntrinsicsSample,
    RgbdSample,
)
from .transport import (
    DdsPeerNode,
    DdsRuntimeSettings,
    DdsTransportError,
    PeerClient,
)
from .tracing import (
    configure_tracing as configure_protocol_tracing,
    current_trace_context,
    sampled as sampled_trace,
    sampled_span as sampled_protocol_span,
    shutdown_tracing as shutdown_protocol_tracing,
)

__version__ = "0.3.0"
