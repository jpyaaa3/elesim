from __future__ import annotations

from pathlib import Path

from workbench.tools.quality.line_coverage import (
    CoverageProfile,
    parse_project_summary,
    trace_command,
)


def test_project_summary_ignores_tests_and_third_party_modules() -> None:
    output = """
  100    82%   elesim_pilot.pick.workflow   (/repo/workflow.py)
   50    40%   test_workflow   (/repo/test_workflow.py)
   20    90%   numpy.core   (/usr/numpy.py)
"""

    assert parse_project_summary(output) == (
        ("elesim_pilot.pick.workflow", 100, 82),
    )


def test_trace_command_requests_missing_lines_and_project_output(tmp_path: Path) -> None:
    command = trace_command(
        CoverageProfile("tests", ("src",)),
        tmp_path,
        python="python-test",
    )

    assert command[:3] == ("python-test", "-m", "trace")
    assert "--missing" in command
    assert f"--coverdir={tmp_path}" in command
    assert command[-3:] == ("pytest", "tests", "-q")
