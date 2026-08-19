from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from elesim_pilot.observability.trace_logger import StructuredTraceLogger
from elesim_pilot.observability import tracing


class StructuredTraceLoggerTests(unittest.TestCase):
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
        tracing._OTEL_CONTEXT = None
        tracing._OTEL_PROPAGATE = None

    def test_disabled_is_noop(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertFalse(tracing.configure_tracing("test"))
            message = {"t": "hello"}
            self.assertIs(tracing.inject_trace_context(message), message)
            self.assertNotIn("_trace", message)

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
