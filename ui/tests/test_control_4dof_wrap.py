"""The wrap-grasp button's summary line.

Drawing needs an imgui context, so what is checked is the part an operator
actually reads: whether one attempt succeeded, got as far as requesting a lift,
or never did -- and that a failed attempt reads as a failure rather than a
crash.
"""

from __future__ import annotations

from elesim_ui.panels.control_4dof import _wrap_summary


class _Outcome:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def _o(**kw):
    base = dict(steps=0, reason="", lift_requested=False, lift_completed=False)
    base.update(kw)
    return _Outcome(**base)


def test_a_completed_lift_reads_as_success():
    assert _wrap_summary(_o(steps=9, reason="lift", lift_requested=True,
                            lift_completed=True)) == "성공 · 9 스텝"


def test_a_requested_lift_that_did_not_hold_says_so():
    got = _wrap_summary(_o(steps=12, reason="lift 중단", lift_requested=True))
    assert "들기 실패" in got and "12" in got and "lift 중단" in got


def test_never_reaching_a_lift_is_reported_with_the_reason():
    got = _wrap_summary(_o(steps=28, reason="steps 소진"))
    assert "미완" in got and "steps 소진" in got


def test_a_waypoint_the_arm_could_not_reach_shows_its_reason():
    got = _wrap_summary(_o(steps=3, reason="waypoint 도달 실패"))
    assert "도달" in got


def test_no_outcome_is_not_an_exception():
    assert _wrap_summary(None) == "결과 없음"
