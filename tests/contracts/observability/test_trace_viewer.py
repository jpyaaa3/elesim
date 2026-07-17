from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tools.debug.trace_viewer import TraceTailer, filter_records, format_record


class TraceTailerTests(unittest.TestCase):
    def test_discovers_new_files_and_only_reads_appends(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log_dir = Path(td)
            tailer = TraceTailer(log_dir)
            path = log_dir / "elesim-host-1.jsonl"
            path.write_text(json.dumps({"ts_unix_ns": 1, "service": "elesim-host", "event": "one"}) + "\n")
            self.assertEqual([record["event"] for record in tailer.poll()], ["one"])
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps({"ts_unix_ns": 2, "service": "elesim-host", "event": "two"}) + "\n")
            self.assertEqual([record["event"] for record in tailer.poll()], ["two"])

    def test_malformed_line_becomes_error_record(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            path = Path(td, "bad.jsonl")
            path.write_text("not-json\n", encoding="utf-8")
            records = TraceTailer(Path(td)).poll()
            self.assertEqual(records[0]["event"], "logger.decode_error")


class TraceFormattingTests(unittest.TestCase):
    def test_formats_span_as_readable_line(self) -> None:
        line = format_record(
            {
                "ts_unix_ns": 1_700_000_000_123_000_000,
                "service": "elesim-host",
                "event": "span.end",
                "span": "host.ctrl.receive",
                "duration_ms": 2.5,
                "attributes": {"messaging.message.type": "target", "elesim.message.seq": 7},
            }
        )
        self.assertIn("HOST", line)
        self.assertIn("host.ctrl.receive", line)
        self.assertIn("2.5ms", line)
        self.assertIn("seq=7", line)

    def test_filters_service_query_and_errors(self) -> None:
        records = [
            {"service": "elesim-host", "event": "span.end", "span": "ok", "error": ""},
            {"service": "elesim-host", "event": "span.end", "span": "bad", "error": "boom"},
            {"service": "elesim-sim", "event": "span.end", "span": "bad", "error": "boom"},
        ]
        filtered = filter_records(records, service="elesim-host", query="bad", errors_only=True)
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0]["error"], "boom")


if __name__ == "__main__":
    unittest.main()
