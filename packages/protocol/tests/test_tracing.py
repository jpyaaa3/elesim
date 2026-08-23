from __future__ import annotations

from elesim_protocol import tracing


def test_optional_tracing_is_noop_without_otel_request(monkeypatch):
    monkeypatch.delenv("ELESIM_TRACE", raising=False)
    assert tracing.configure_tracing("test") is False
    assert tracing.current_trace_context() == {}
    assert tracing.sampled("test", every=10) is False
    with tracing.span("test.span", attributes={"code.function.name": "test"}):
        pass
    with tracing.sampled_span("test.sampled", sample_key="test", every=10):
        pass
    tracing.shutdown_tracing()
