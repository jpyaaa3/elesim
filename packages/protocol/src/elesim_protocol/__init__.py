"""Versioned wire contract shared by independently deployed Elesim nodes."""

from .messages import *  # noqa: F401,F403
from .payloads import (
    DiscoverRequest,
    MotionCommandRequest,
    OperatorIntentRequest,
    OperatorViewSnapshot,
    RegisterRequest,
    SelectTargetRequest,
    TelemetryPayload,
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
from .transport import EndpointClient, EndpointSession, TransportError

__version__ = "0.1.0"
