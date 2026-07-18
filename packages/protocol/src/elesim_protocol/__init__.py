"""Versioned wire contract shared by independently deployed Elesim nodes."""

from .messages import *  # noqa: F401,F403
from .operator import OPERATOR_OPERATIONS, SERVICE_CALLS, SERVICE_VALUES, STATE_CALLS
from .serde import decode_value, encode_value, state_snapshot
from .transport import EndpointClient

__version__ = "0.1.0"
