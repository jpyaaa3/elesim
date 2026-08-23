"""Robot tracing facade backed by the optional protocol adapter.

Robot startup and safety must remain independent of OTel packages.  The
protocol adapter is deliberately lazy and fail-open, so these names preserve
the old Robot import surface without making telemetry a runtime dependency.
"""

from __future__ import annotations

from functools import wraps
from typing import Any

from elesim_protocol.tracing import (
    configure_tracing as _configure,
    current_trace_context,
    sampled_span,
    shutdown_tracing as _shutdown,
    span,
    traced,
)


def sampled_traced(name: str, **options: Any):
    # High-rate hardware loops are intentionally represented by their parent
    # operation; tracing every motor tick would turn telemetry into load.
    sample_key = str(options.pop("sample_key", name))
    every = int(options.pop("every", 1))

    def decorate(function):
        @wraps(function)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            with sampled_span(
                name,
                sample_key=sample_key,
                every=every,
                **options,
            ):
                return function(*args, **kwargs)

        return wrapped

    return decorate


def configure_tracing(service_name: str) -> bool:
    return _configure(service_name)


def shutdown_tracing() -> None:
    _shutdown()


__all__ = [
    "configure_tracing",
    "current_trace_context",
    "sampled_traced",
    "shutdown_tracing",
    "span",
    "traced",
]
