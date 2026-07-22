"""Optional OpenTelemetry tracing with a local JSONL fallback log."""

from __future__ import annotations

import os
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from functools import wraps
from typing import Any, Callable, Generator, Mapping, MutableMapping, Optional, TypeVar

from .trace_logger import StructuredTraceLogger

_F = TypeVar("_F", bound=Callable[..., Any])
_LOCK = threading.Lock()
_CONFIGURED = False
_SERVICE_NAME = "elesim"
_TRACER: Any = None
_PROVIDER: Any = None
_LOGGER: Optional[StructuredTraceLogger] = None
_OTEL_CONTEXT: Any = None
_OTEL_PROPAGATE: Any = None
_OTEL_SPAN_KIND: Any = None
_SAMPLE_COUNTS: dict[str, int] = {}


def enabled() -> bool:
    return os.environ.get("ELESIM_TRACE", "").strip().lower() in {"1", "true", "yes", "on"}


def _clean_attributes(attributes: Optional[Mapping[str, Any]]) -> dict[str, Any]:
    clean: dict[str, Any] = {}
    for key, value in (attributes or {}).items():
        if value is None:
            continue
        if isinstance(value, (bool, int, float, str)):
            clean[str(key)] = value
        elif isinstance(value, (list, tuple)) and all(isinstance(item, (bool, int, float, str)) for item in value):
            clean[str(key)] = list(value)
        else:
            clean[str(key)] = str(value)
    return clean


def _otlp_trace_endpoint(raw: str) -> str:
    endpoint = str(raw).strip().rstrip("/")
    if not endpoint:
        endpoint = "http://127.0.0.1:4318"
    if not endpoint.endswith("/v1/traces"):
        endpoint += "/v1/traces"
    return endpoint


def configure_tracing(service_name: str) -> bool:
    global _CONFIGURED, _SERVICE_NAME, _TRACER, _PROVIDER, _LOGGER
    global _OTEL_CONTEXT, _OTEL_PROPAGATE, _OTEL_SPAN_KIND
    if not enabled():
        return False
    with _LOCK:
        if _CONFIGURED:
            return _TRACER is not None
        _CONFIGURED = True
        _SERVICE_NAME = str(service_name).strip() or "elesim"
        _LOGGER = StructuredTraceLogger.from_env(_SERVICE_NAME)
        _LOGGER.write("tracing.configure", endpoint=os.environ.get("ELESIM_OTEL_ENDPOINT", ""))
        try:
            from opentelemetry import context, propagate, trace
            from opentelemetry.exporter.otlp.proto.http.trace_exporter import OTLPSpanExporter
            from opentelemetry.sdk.resources import Resource
            from opentelemetry.sdk.trace import TracerProvider
            from opentelemetry.sdk.trace.export import BatchSpanProcessor
            from opentelemetry.trace import SpanKind

            resource = Resource.create({"service.name": _SERVICE_NAME, "process.pid": os.getpid()})
            provider = TracerProvider(resource=resource)
            exporter = OTLPSpanExporter(endpoint=_otlp_trace_endpoint(os.environ.get("ELESIM_OTEL_ENDPOINT", "")))
            provider.add_span_processor(BatchSpanProcessor(exporter))
            trace.set_tracer_provider(provider)
            _PROVIDER = provider
            _TRACER = trace.get_tracer("elesim", "1.0")
            _OTEL_CONTEXT = context
            _OTEL_PROPAGATE = propagate
            _OTEL_SPAN_KIND = SpanKind
            return True
        except Exception as exc:
            _LOGGER.write("tracing.otel_unavailable", error=repr(exc))
            return False


def shutdown_tracing() -> None:
    global _PROVIDER, _LOGGER
    with _LOCK:
        if _PROVIDER is not None:
            try:
                _PROVIDER.shutdown()
            except Exception:
                pass
            _PROVIDER = None
        if _LOGGER is not None:
            _LOGGER.write("tracing.shutdown")
            _LOGGER.close()
            _LOGGER = None


def log_event(event: str, **fields: Any) -> None:
    if _LOGGER is not None:
        _LOGGER.write(event, **fields)


def _span_ids(otel_span: Any) -> tuple[str, str]:
    if otel_span is None:
        return "", ""
    try:
        ctx = otel_span.get_span_context()
        if not ctx.is_valid:
            return "", ""
        return f"{ctx.trace_id:032x}", f"{ctx.span_id:016x}"
    except Exception:
        return "", ""


def _kind(kind: str) -> Any:
    if _OTEL_SPAN_KIND is None:
        return None
    return getattr(_OTEL_SPAN_KIND, str(kind).upper(), _OTEL_SPAN_KIND.INTERNAL)


@dataclass
class ActiveSpan:
    name: str
    otel_span: Any = None

    def event(self, name: str, **attributes: Any) -> None:
        clean = _clean_attributes(attributes)
        if self.otel_span is not None:
            try:
                self.otel_span.add_event(str(name), clean)
            except Exception:
                pass
        trace_id, span_id = _span_ids(self.otel_span)
        log_event("span.event", span=self.name, name=name, trace_id=trace_id, span_id=span_id, attributes=clean)


@contextmanager
def span(
    name: str,
    *,
    attributes: Optional[Mapping[str, Any]] = None,
    kind: str = "internal",
    parent_context: Any = None,
) -> Generator[ActiveSpan, None, None]:
    clean = _clean_attributes(attributes)
    started_ns = time.time_ns()
    otel_cm: Any = None
    otel_span: Any = None
    if _TRACER is not None:
        try:
            otel_cm = _TRACER.start_as_current_span(
                str(name),
                context=parent_context,
                kind=_kind(kind),
                attributes=clean,
            )
            otel_span = otel_cm.__enter__()
        except Exception:
            otel_cm = None
            otel_span = None
    active = ActiveSpan(str(name), otel_span)
    error = ""
    try:
        yield active
    except BaseException as exc:
        error = repr(exc)
        if otel_span is not None:
            try:
                otel_span.record_exception(exc)
            except Exception:
                pass
        raise
    finally:
        trace_id, span_id = _span_ids(otel_span)
        log_event(
            "span.end",
            span=name,
            kind=kind,
            duration_ms=(time.time_ns() - started_ns) / 1_000_000.0,
            trace_id=trace_id,
            span_id=span_id,
            error=error,
            attributes=clean,
        )
        if otel_cm is not None:
            try:
                otel_cm.__exit__(None, None, None)
            except Exception:
                pass


def traced(name: str, *, kind: str = "internal") -> Callable[[_F], _F]:
    def decorate(fn: _F) -> _F:
        @wraps(fn)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            with span(name, kind=kind):
                return fn(*args, **kwargs)

        return wrapped  # type: ignore[return-value]

    return decorate


def sampled_traced(
    name: str,
    *,
    sample_key: str,
    every: int,
    kind: str = "internal",
) -> Callable[[_F], _F]:
    def decorate(fn: _F) -> _F:
        @wraps(fn)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            with sampled_span(name, sample_key=sample_key, every=every, kind=kind):
                return fn(*args, **kwargs)

        return wrapped  # type: ignore[return-value]

    return decorate


def traced_thread_target(name: str, target: Callable[[], Any], **attributes: Any) -> Callable[[], Any]:
    parent = _OTEL_CONTEXT.get_current() if _OTEL_CONTEXT is not None else None

    @wraps(target)
    def run() -> Any:
        with span(name, attributes=attributes, parent_context=parent):
            return target()

    return run


def inject_trace_context(message: MutableMapping[str, Any]) -> MutableMapping[str, Any]:
    if _OTEL_PROPAGATE is None:
        return message
    carrier: dict[str, str] = {}
    try:
        _OTEL_PROPAGATE.inject(carrier)
    except Exception:
        return message
    if carrier:
        message["_trace"] = carrier
    return message


def extract_trace_context(message: Mapping[str, Any]) -> Any:
    if _OTEL_PROPAGATE is None:
        return None
    carrier = message.get("_trace", {})
    if not isinstance(carrier, Mapping):
        return None
    try:
        return _OTEL_PROPAGATE.extract(dict(carrier))
    except Exception:
        return None


def message_attributes(message: Mapping[str, Any], endpoint: str = "") -> dict[str, Any]:
    attrs: dict[str, Any] = {
        "messaging.system": "zeromq",
        "messaging.message.type": str(message.get("t", "unknown")),
    }
    if endpoint:
        attrs["server.address"] = str(endpoint)
    for key in ("seq", "source", "ok", "reason"):
        if key in message and isinstance(message[key], (bool, int, float, str)):
            attrs[f"elesim.message.{key}"] = message[key]
    return attrs


def should_trace_message(message: Mapping[str, Any]) -> bool:
    if not enabled():
        return False
    message_type = str(message.get("t", "unknown")).lower()
    every = 1
    if message_type in {"state", "sim_state", "hello", "perception_observation"}:
        every = 60
    elif message_type == "target" and str(message.get("source", "")).lower() == "slider":
        every = 10
    return sampled(f"message:{message_type}", every=every)


def sampled(key: str, *, every: int = 1) -> bool:
    if not enabled():
        return False
    sample_every = max(1, int(every))
    counter_key = f"{threading.current_thread().name}:{key}"
    with _LOCK:
        count = _SAMPLE_COUNTS.get(counter_key, 0) + 1
        _SAMPLE_COUNTS[counter_key] = count
    return count == 1 or count % sample_every == 0


@contextmanager
def sampled_span(
    name: str,
    *,
    sample_key: str,
    every: int,
    attributes: Optional[Mapping[str, Any]] = None,
    kind: str = "internal",
) -> Generator[ActiveSpan, None, None]:
    if not sampled(sample_key, every=every):
        yield ActiveSpan(str(name))
        return
    with span(name, attributes=attributes, kind=kind) as active:
        yield active


@contextmanager
def message_span(
    operation: str,
    message: MutableMapping[str, Any],
    *,
    endpoint: str,
    direction: str,
) -> Generator[ActiveSpan, None, None]:
    producer = str(direction).lower() in {"send", "producer"}
    parent = None if producer else extract_trace_context(message)
    if not should_trace_message(message):
        yield ActiveSpan(str(operation))
        return
    with span(
        operation,
        attributes=message_attributes(message, endpoint),
        kind="producer" if producer else "consumer",
        parent_context=parent,
    ) as active:
        if producer:
            inject_trace_context(message)
        yield active


__all__ = [
    "ActiveSpan",
    "configure_tracing",
    "enabled",
    "extract_trace_context",
    "inject_trace_context",
    "log_event",
    "message_attributes",
    "message_span",
    "sampled",
    "sampled_span",
    "sampled_traced",
    "should_trace_message",
    "shutdown_tracing",
    "span",
    "traced",
    "traced_thread_target",
]
