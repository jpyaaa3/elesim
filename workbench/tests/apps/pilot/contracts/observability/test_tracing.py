from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from elesim_protocol.tracing import StructuredTraceLogger
from elesim_protocol import tracing


class StructuredTraceLoggerTests(unittest.TestCase):
    def test_unwritable_parent_does_not_break_application(self) -> None:
        with mock.patch.object(Path, "mkdir", side_effect=OSError("read-only")):
            logger = StructuredTraceLogger("svc", "/unused/trace.jsonl")
        logger.write("event")
        logger.close()

    def test_rotation_keeps_bounded_history(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td, "trace.jsonl")
            logger = StructuredTraceLogger("svc", path)
            self.assertEqual(logger._handler.maxBytes, 10 * 1024 * 1024)
            logger._handler.maxBytes = 256
            for seq in range(20):
                logger.write("event", seq=seq, detail="x" * 100)
            logger.close()
            self.assertEqual(len(list(Path(td).glob("trace.jsonl*"))), 4)
            self.assertEqual(json.loads(path.read_text())["seq"], 19)

    def test_writes_copyable_json_lines(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td, "trace.jsonl")
            logger = StructuredTraceLogger("svc", path)
            logger.write("transport.send", endpoint="tcp://127.0.0.1:5558", values=(1, 2))
            logger.close()
            payload = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(payload["service"], "svc")
            self.assertEqual(payload["event"], "transport.send")
            self.assertEqual(payload["values"], [1, 2])


class TracingContractTests(unittest.TestCase):
    def tearDown(self) -> None:
        tracing.shutdown_tracing()
        tracing._CONFIGURED = False
        tracing._TRACER = None
        tracing._PROPAGATE = None
        tracing._SPAN_KIND = None
        tracing._SAMPLE_COUNTS.clear()

    def test_disabled_is_noop(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertFalse(tracing.configure_tracing("test"))
            message = {"t": "hello"}
            self.assertIs(tracing.inject_trace_context(message), message)
            self.assertNotIn("_trace", message)

    def test_default_configuration_does_not_create_local_archive(self) -> None:
        with mock.patch.dict(os.environ, {"ELESIM_TRACE": "1"}), mock.patch.object(
            StructuredTraceLogger, "from_env"
        ) as create, mock.patch.dict("sys.modules", {"opentelemetry": None}):
            self.assertFalse(tracing.configure_tracing("robot"))
            create.assert_not_called()

    def test_consumer_preserves_carrier_mapping(self) -> None:
        carrier = {"traceparent": "test-parent"}
        with mock.patch.object(tracing, "should_trace_message", return_value=True), mock.patch.object(
            tracing, "span"
        ) as create:
            with tracing.message_span("receive", {"t": "target", "_trace": carrier}, endpoint="sim", direction="receive"):
                pass
            self.assertEqual(create.call_args.kwargs["trace_context"], carrier)

    def test_json_only_sampling_and_decorator_keep_result(self) -> None:
        with mock.patch.dict(os.environ, {"ELESIM_TRACE": "1"}):
            @tracing.sampled_traced("work", sample_key="work", every=3)
            def work():
                return 7

            self.assertEqual([work() for _ in range(4)], [7] * 4)
            self.assertEqual([tracing.sampled("sample", every=3) for _ in range(4)], [True, False, True, False])

    def test_message_attributes_do_not_copy_payload(self) -> None:
        attrs = tracing.message_attributes(
            {"t": "target", "seq": 7, "source": "ik", "large": [1] * 100},
            "tcp://127.0.0.1:5558",
        )
        self.assertEqual(attrs["messaging.message.type"], "target")
        self.assertEqual(attrs["elesim.message.seq"], 7)
        self.assertNotIn("large", attrs)

    def test_span_logs_error_without_otel(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            logger = StructuredTraceLogger("test", Path(td, "trace.jsonl"))
            tracing._LOGGER = logger
            with self.assertRaisesRegex(RuntimeError, "boom"):
                with tracing.span("failing"):
                    raise RuntimeError("boom")
            logger.close()
            tracing._LOGGER = None
            payload = json.loads(Path(td, "trace.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(payload["span"], "failing")
            self.assertIn("boom", payload["error"])


if __name__ == "__main__":
    unittest.main()
