"""Bounded in-process media boundary for the Sim runtime.

The media worker is deliberately an implementation detail of the ``sim``
application.  It is not a new DDS participant or deployment role.
"""

from .dispatch import FrameDispatchWorker
from .worker import (
    MediaWorkerError,
    MediaWorkerUnavailable,
    MediaWorkerClient,
    SharedFrameMailbox,
    VideoStreamSpec,
)

__all__ = [
    "FrameDispatchWorker",
    "MediaWorkerClient",
    "MediaWorkerError",
    "MediaWorkerUnavailable",
    "SharedFrameMailbox",
    "VideoStreamSpec",
]
