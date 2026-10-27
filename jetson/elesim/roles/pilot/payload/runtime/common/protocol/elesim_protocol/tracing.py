"""Optional, fail-open tracing primitives shared by all runtime roles.

The protocol package must remain usable on a host without OpenTelemetry.  The
adapter therefore imports OTel lazily, never puts application payloads in a
span, and exposes a W3C carrier provider that can be handed to ``PeerClient``.
"""

from __future__ import annotations

import os
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass
from functools import wraps
from typing import Any, Callable, Generator, Mapping, MutableMapping, Optional


_LOCK = threading.RLock()
_CONFIGURED = False
_TRACER: Any = None
_PROVIDER: Any = None
_PROPAGATE: Any = None
_SPAN_KIND: Any = None
_SAMPLE_COUNTS: dict[str, int] = {}
_LOGGER: Optional["StructuredTraceLogger"] = None


def _json_value(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return str(value)


class StructuredTraceLogger:
    """Thread-safe JSONL fallback shared by runtime applications."""

    def __init__(self, service_name: str, path: str | Path) -> None:
        self.service_name = str(service_name)
        self.path = Path(path)
        self._handler: Optional[RotatingFileHandler] = None
        self._logger: Optional[logging.Logger] = None
        self._lock = threading.Lock()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(
                self.path,
                maxBytes=10 * 1024 * 1024,
                backupCount=3,
                encoding="utf-8",
            )
            logger = logging.getLogger(f"elesim.trace.{id(self)}")
            logger.setLevel(logging.INFO)
            logger.propagate = False
            handler.setFormatter(logging.Formatter("%(message)s"))
            logger.addHandler(handler)
            self._handler = handler
            self._logger = logger
        except Exception:
            # Telemetry must never affect DDS, media, or Robot safety paths.
            self._handler = None
            self._logger = None

    @classmethod
    def from_env(cls, service_name: str) -> "StructuredTraceLogger":
        raw_path = os.environ.get("ELESIM_TRACE_LOG", "").strip()
        if raw_path:
            path = Path(raw_path)
            if path.suffix.lower() != ".jsonl":
                path = path / f"{service_name}-{os.getpid()}.jsonl"
        else:
            path = Path("logs/tracing") / f"{service_name}-{os.getpid()}.jsonl"
        return cls(service_name, path)

    def write(self, event: str, **fields: Any) -> None:
        payload = {
            "ts_unix_ns": time.time_ns(),
            "service": self.service_name,
            "pid": os.getpid(),
            "thread": threading.current_thread().name,
            "event": str(event),
        }
        try:
            payload.update({str(key): _json_value(value) for key, value in fields.items()})
            line = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        except Exception:
            return
        with self._lock:
            if self._logger is not None:
                try:
                    self._logger.info(line)
                except Exception:
                    pass

    def close(self) -> None:
        with self._lock:
            if self._logger is not None and self._handler is not None:
                try:
                    self._handler.flush()
                    self._logger.removeHandler(self._handler)
                    self._handler.close()
                except Exception:
                    pass
                self._logger = None
                self._handler = None


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


def configure_tracing(service_name: str, *, local_log: bool = False) -> bool:
    """Enable OTel when installed and requested; return whether it is active."""

    global _CONFIGURED, _TRACER, _PROVIDER, _PROPAGATE, _SPAN_KIND, _LOGGER
    if not enabled():
        return False
    with _LOCK:
        if _CONFIGURED:
            return _TRACER is not None
        _CONFIGURED = True
        if local_log:
            try:
                _LOGGER = StructuredTraceLogger.from_env(str(service_name).strip() or "elesim")
                _LOGGER.write("tracing.configure", endpoint=os.environ.get("ELESIM_OTEL_ENDPOINT", ""))
            except Exception:
                _LOGGER = None
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
                    {
                        "service.name": str(service_name).strip() or "elesim",
                        "process.pid": os.getpid(),
                    }
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
        except Exception as exc:
            if _LOGGER is not None:
                _LOGGER.write("tracing.otel_unavailable", error=repr(exc))
            # Telemetry is strictly optional.  DDS, media and Robot safety
            # continue with an empty carrier when packages/exporter are absent.
            _TRACER = None
            _PROPAGATE = None
            _SPAN_KIND = None
            return False


def shutdown_tracing() -> None:
    global _PROVIDER, _LOGGER
    with _LOCK:
        provider = _PROVIDER
        _PROVIDER = None
        if provider is not None:
            try:
                provider.shutdown()
            except Exception:
                pass
        if _LOGGER is not None:
            _LOGGER.write("tracing.shutdown")
            _LOGGER.close()
            _LOGGER = None


def log_event(event: str, **fields: Any) -> None:
    if _LOGGER is not None:
        _LOGGER.write(event, **fields)


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


def inject_trace_context(message: MutableMapping[str, Any]) -> MutableMapping[str, Any]:
    carrier = current_trace_context()
    if carrier:
        message["_trace"] = carrier
    return message


def traced_thread_target(name: str, target: Callable[[], Any], **attributes: Any) -> Callable[[], Any]:
    parent = current_trace_context()

    @wraps(target)
    def run() -> Any:
        with span(name, attributes=attributes, trace_context=parent):
            return target()

    return run


def message_attributes(message: Mapping[str, Any], endpoint: str = "") -> dict[str, Any]:
    attrs: dict[str, Any] = {
        "messaging.system": "ros2",
        "messaging.message.type": str(message.get("type", message.get("t", "unknown"))),
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


@contextmanager
def message_span(
    operation: str,
    message: MutableMapping[str, Any],
    *,
    endpoint: str,
    direction: str,
) -> Generator[ActiveSpan, None, None]:
    producer = str(direction).lower() in {"send", "producer"}
    parent = None if producer else message.get("_trace")
    if not should_trace_message(message):
        yield ActiveSpan(str(operation))
        return
    with span(
        operation,
        attributes={**message_attributes(message, endpoint), "code.function.name": operation},
        kind="producer" if producer else "consumer",
        trace_context=parent,
    ) as active:
        if producer:
            inject_trace_context(message)
        yield active


def sampled(key: str, *, every: int = 1) -> bool:
    """Return whether this event should create a span.

    Sampling is process-local and intentionally deterministic for a stream:
    the first event is retained, then every ``every``th event.  With tracing
    disabled (or unavailable) the fast path does not touch the counter.
    """

    if not enabled():
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
        clean = _clean(attributes)
        if self.otel_span is not None:
            try:
                self.otel_span.add_event(str(name), clean)
            except Exception:
                pass
        trace_id, span_id = _span_ids(self.otel_span)
        log_event(
            "span.event", span=self.name, name=name,
            trace_id=trace_id, span_id=span_id, attributes=clean,
        )


def _span_ids(otel_span: Any) -> tuple[str, str]:
    if otel_span is None:
        return "", ""
    try:
        context = otel_span.get_span_context()
        if not context.is_valid:
            return "", ""
        return f"{context.trace_id:032x}", f"{context.span_id:016x}"
    except Exception:
        return "", ""


@contextmanager
def span(
    name: str,
    *,
    attributes: Optional[Mapping[str, Any]] = None,
    kind: str = "internal",
    trace_context: Optional[Mapping[str, str]] = None,
) -> Generator[ActiveSpan, None, None]:
    if _TRACER is None and _LOGGER is None:
        yield ActiveSpan(str(name))
        return
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
    started = time.time_ns()
    error = ""
    error_info: tuple[Any, Any, Any] = (None, None, None)
    try:
        yield ActiveSpan(str(name), otel_span)
    except BaseException as exc:
        error = repr(exc)
        error_info = (type(exc), exc, exc.__traceback__)
        if otel_span is not None:
            try:
                otel_span.record_exception(exc)
            except Exception:
                pass
        raise
    finally:
        trace_id, span_id = _span_ids(otel_span)
        log_event(
            "span.end", span=name, kind=kind,
            duration_ms=(time.time_ns() - started) / 1_000_000.0,
            trace_id=trace_id, span_id=span_id, error=error,
            attributes=clean_attributes,
        )
        if otel_span is not None:
            try:
                otel_span.set_attribute("elesim.duration_ms", (time.time_ns() - started) / 1_000_000.0)
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


def sampled_traced(
    name: str,
    *,
    sample_key: str,
    every: int,
    kind: str = "internal",
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    def decorate(function: Callable[..., Any]) -> Callable[..., Any]:
        @wraps(function)
        def wrapped(*args: Any, **kwargs: Any) -> Any:
            with sampled_span(name, sample_key=sample_key, every=every, kind=kind):
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
    "StructuredTraceLogger",
    "configure_tracing",
    "current_trace_context",
    "enabled",
    "inject_trace_context",
    "log_event",
    "message_attributes",
    "message_span",
    "shutdown_tracing",
    "sampled",
    "sampled_span",
    "sampled_traced",
    "span",
    "traced",
    "traced_thread_target",
]
