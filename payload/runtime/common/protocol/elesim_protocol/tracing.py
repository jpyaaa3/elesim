"""Optional, fail-open tracing primitives shared by UI and Robot.

The protocol package must remain usable on a host without OpenTelemetry.  The
adapter therefore imports OTel lazily, never puts application payloads in a
span, and exposes a W3C carrier provider that can be handed to ``PeerClient``.
"""

from __future__ import annotations

import os
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from functools import wraps
from typing import Any, Callable, Generator, Mapping, Optional


_LOCK = threading.RLock()
_CONFIGURED = False
_TRACER: Any = None
_PROVIDER: Any = None
_PROPAGATE: Any = None
_SPAN_KIND: Any = None
_SAMPLE_COUNTS: dict[str, int] = {}


def enabled() -> bool:
    return os.environ.get("ELESIM_TRACE", "").strip().lower() in {
        "1", "true", "yes", "on"
    }


def _clean(attributes: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in (attributes or {}).items():
        if value is None:
            continue
        if isinstance(value, (bool, int, float, str)):
            result[str(key)] = value
        elif isinstance(value, (list, tuple)) and all(
            isinstance(item, (bool, int, float, str)) for item in value
        ):
            result[str(key)] = list(value)
        else:
            result[str(key)] = str(value)
    return result


def _endpoint(raw: str) -> str:
    value = str(raw).strip().rstrip("/") or "http://127.0.0.1:4318"
    return value if value.endswith("/v1/traces") else f"{value}/v1/traces"


def configure_tracing(service_name: str) -> bool:
    """Enable OTel when installed and requested; return whether it is active."""

    global _CONFIGURED, _TRACER, _PROVIDER, _PROPAGATE, _SPAN_KIND
    if not enabled():
        return False
    with _LOCK:
        if _CONFIGURED:
            return _TRACER is not None
        _CONFIGURED = True
        try:
            from opentelemetry import propagate, trace
            from opentelemetry.exporter.otlp.proto.http.trace_exporter import (
                OTLPSpanExporter,
            )
            from opentelemetry.sdk.resources import Resource
            from opentelemetry.sdk.trace import TracerProvider
            from opentelemetry.sdk.trace.export import BatchSpanProcessor
            from opentelemetry.trace import SpanKind

            provider = TracerProvider(
                resource=Resource.create(
                    {"service.name": str(service_name).strip() or "elesim"}
                )
            )
            provider.add_span_processor(
                BatchSpanProcessor(
                    OTLPSpanExporter(
                        endpoint=_endpoint(
                            os.environ.get("ELESIM_OTEL_ENDPOINT", "")
                        )
                    )
                )
            )
            trace.set_tracer_provider(provider)
            _PROVIDER = provider
            _TRACER = trace.get_tracer("elesim.protocol", "1.0")
            _PROPAGATE = propagate
            _SPAN_KIND = SpanKind
            return True
        except Exception:
            # Telemetry is strictly optional.  DDS, media and Robot safety
            # continue with an empty carrier when packages/exporter are absent.
            _TRACER = None
            _PROPAGATE = None
            _SPAN_KIND = None
            return False


def shutdown_tracing() -> None:
    global _PROVIDER
    with _LOCK:
        provider = _PROVIDER
        _PROVIDER = None
        if provider is not None:
            try:
                provider.shutdown()
            except Exception:
                pass


def current_trace_context() -> dict[str, str]:
    """Return the active W3C trace carrier for a DDS envelope."""

    propagate = _PROPAGATE
    if propagate is None:
        return {}
    carrier: dict[str, str] = {}
    try:
        propagate.inject(carrier)
    except Exception:
        return {}
    return {str(key): str(value) for key, value in carrier.items()}


def sampled(key: str, *, every: int = 1) -> bool:
    """Return whether this event should create a span.

    Sampling is process-local and intentionally deterministic for a stream:
    the first event is retained, then every ``every``th event.  With tracing
    disabled (or unavailable) the fast path does not touch the counter.
    """

    if _TRACER is None or not enabled():
        return False
    sample_every = max(1, int(every))
    key = f"{threading.current_thread().name}:{key}"
    with _LOCK:
        count = _SAMPLE_COUNTS.get(key, 0) + 1
        _SAMPLE_COUNTS[key] = count
    return count == 1 or count % sample_every == 0


def _parent(carrier: Optional[Mapping[str, str]]) -> Any:
    propagate = _PROPAGATE
    if propagate is None or not carrier:
        return None
    try:
        return propagate.extract(dict(carrier))
    except Exception:
        return None


def _kind(value: str) -> Any:
    if _SPAN_KIND is None:
        return None
    return getattr(_SPAN_KIND, str(value).upper(), _SPAN_KIND.INTERNAL)


@dataclass
class ActiveSpan:
    name: str
    otel_span: Any = None

    def event(self, name: str, **attributes: Any) -> None:
        if self.otel_span is None:
            return
        try:
            self.otel_span.add_event(str(name), _clean(attributes))
        except Exception:
            pass


@contextmanager
def span(
    name: str,
    *,
    attributes: Optional[Mapping[str, Any]] = None,
    kind: str = "internal",
    trace_context: Optional[Mapping[str, str]] = None,
) -> Generator[ActiveSpan, None, None]:
    context_manager: Any = None
    otel_span: Any = None
    tracer = _TRACER
    clean_attributes = _clean(attributes)
    clean_attributes.setdefault("code.function.name", str(name))
    clean_attributes.setdefault("elesim.code.symbol", str(name))
    if tracer is not None:
        try:
            context_manager = tracer.start_as_current_span(
                str(name),
                context=_parent(trace_context),
                kind=_kind(kind),
                attributes=clean_attributes,
            )
            otel_span = context_manager.__enter__()
        except Exception:
            context_manager = None
            otel_span = None
    started = time.monotonic()
    error_info: tuple[Any, Any, Any] = (None, None, None)
    try:
        yield ActiveSpan(str(name), otel_span)
    except BaseException as exc:
        error_info = (type(exc), exc, exc.__traceback__)
        if otel_span is not None:
            try:
                otel_span.record_exception(exc)
            except Exception:
                pass
        raise
    finally:
        if otel_span is not None:
            try:
                otel_span.set_attribute("elesim.duration_ms", (time.monotonic() - started) * 1000.0)
            except Exception:
                pass
        if context_manager is not None:
            try:
                context_manager.__exit__(*error_info)
            except Exception:
                pass


def traced(name: str, **options: Any) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    def decorate(function: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(function)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            with span(name, **options):
                return function(*args, **kwargs)

        return wrapped

    return decorate


@contextmanager
def sampled_span(
    name: str,
    *,
    sample_key: str,
    every: int,
    attributes: Optional[Mapping[str, Any]] = None,
    kind: str = "internal",
    trace_context: Optional[Mapping[str, str]] = None,
) -> Generator[ActiveSpan, None, None]:
    if not sampled(sample_key, every=every):
        yield ActiveSpan(str(name))
        return
    with span(
        name,
        attributes=attributes,
        kind=kind,
        trace_context=trace_context,
    ) as active:
        yield active


__all__ = [
    "ActiveSpan",
    "configure_tracing",
    "current_trace_context",
    "enabled",
    "shutdown_tracing",
    "sampled",
    "sampled_span",
    "span",
    "traced",
]
