from __future__ import annotations

from elesim_pilot.pick.workflow import (
    PickWorkflowPhase,
    PickWorkflowResult,
    run_pick_workflow,
)


def test_workflow_runs_each_named_phase_in_order() -> None:
    calls: list[str] = []
    statuses: list[str] = []
    phases = tuple(
        PickWorkflowPhase(label=name, state_phase=name, start=lambda name=name: calls.append(name))
        for name in ("look", "aim", "grasp")
    )

    result = run_pick_workflow(
        phases,
        timeout_s=3.0,
        begin_phase=lambda phase: statuses.append(phase.label),
        wait_phase=lambda _label, _timeout: True,
        failed=lambda: False,
        cancelled=lambda: False,
    )

    assert result == PickWorkflowResult.completed()
    assert calls == ["look", "aim", "grasp"]
    assert statuses == calls


def test_workflow_stops_immediately_when_a_phase_fails() -> None:
    calls: list[str] = []
    failed = False

    def fail_aim() -> None:
        nonlocal failed
        calls.append("aim")
        failed = True

    result = run_pick_workflow(
        (
            PickWorkflowPhase("look", "look", lambda: calls.append("look")),
            PickWorkflowPhase("aim", "aim", fail_aim),
            PickWorkflowPhase("grasp", "grasp", lambda: calls.append("grasp")),
        ),
        timeout_s=3.0,
        begin_phase=lambda _phase: None,
        wait_phase=lambda _label, _timeout: True,
        failed=lambda: failed,
        cancelled=lambda: False,
    )

    assert not result.success
    assert result.reason == "failed"
    assert result.phase == "aim"
    assert calls == ["look", "aim"]


def test_false_wait_is_classified_as_cancel_or_timeout() -> None:
    phase = PickWorkflowPhase("look", "look", lambda: None)

    cancelled = run_pick_workflow(
        (phase,),
        timeout_s=3.0,
        begin_phase=lambda _phase: None,
        wait_phase=lambda _label, _timeout: False,
        failed=lambda: False,
        cancelled=lambda: True,
    )
    timed_out = run_pick_workflow(
        (phase,),
        timeout_s=3.0,
        begin_phase=lambda _phase: None,
        wait_phase=lambda _label, _timeout: False,
        failed=lambda: False,
        cancelled=lambda: False,
    )

    assert cancelled.reason == "cancelled"
    assert timed_out.reason == "timeout"


def test_wait_completion_does_not_hide_failure_or_cancellation() -> None:
    for outcome in ("failed", "cancelled"):
        for final_phase in (False, True):
            state = {"failed": False, "cancelled": False}
            calls = []

            def wait(_label, _timeout):
                state[outcome] = True
                return True

            phases = [PickWorkflowPhase("look", "look", lambda: calls.append("look"))]
            if not final_phase:
                phases.append(PickWorkflowPhase("grasp", "grasp", lambda: calls.append("grasp")))
            result = run_pick_workflow(
                phases, timeout_s=3.0, begin_phase=lambda _phase: None,
                wait_phase=wait, failed=lambda: state["failed"],
                cancelled=lambda: state["cancelled"],
            )
            assert not result.success
            assert result.reason == outcome
            assert result.phase == "look"
            assert calls == ["look"]


def test_wait_exception_is_reported_with_phase_and_allows_next_run() -> None:
    def broken_wait(_label, _timeout):
        raise RuntimeError("phase feedback lost")

    options = dict(
        phases=(PickWorkflowPhase("look", "look", lambda: None),),
        timeout_s=3.0, begin_phase=lambda _phase: None,
        failed=lambda: False, cancelled=lambda: False,
    )
    failed = run_pick_workflow(**options, wait_phase=broken_wait)
    assert failed.reason == "exception"
    assert failed.phase == "look"
    assert failed.detail == "phase feedback lost"
    assert run_pick_workflow(**options, wait_phase=lambda _label, _timeout: True).success
