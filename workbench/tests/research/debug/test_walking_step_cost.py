"""Keep the walking probe's timed path representative of the runtime arm hold."""
from types import SimpleNamespace

import pytest

from workbench.research.debug import profile_sim_walking as probe


@pytest.mark.parametrize("ticks", [1, 3])
def test_step_cost_includes_command_arm_hold_and_physics_on_every_tick(ticks):
    now = 0.0
    calls = []
    arm = SimpleNamespace(linear_m=.11, roll_rad=.22, theta1_rad=.33, theta2_rad=.44)
    def charged(label, seconds):
        def invoke(*args):
            nonlocal now
            calls.append((label, args))
            now += seconds
        return invoke
    scene = SimpleNamespace(
        go2=SimpleNamespace(set_planar_velocity=charged("command", .001),
                            set_arm_q_for_metrics=charged("arm_metrics", .002)),
        apply_sim_q=charged("arm", .007),
        step=charged("physics", .011),
    )
    for _ in range(ticks):
        cost = probe.advance_controlled_step(scene, (.8, 0., .8), arm, clock=lambda: now)
        assert cost == pytest.approx(dict(command=.003, arm=.007, physics=.011, total=.021))
    assert calls == [entry for _ in range(ticks) for entry in (
        ("command", (.8, 0., .8)), ("arm_metrics", ((.11, .22, .33, .44),)),
        ("arm", (arm,)), ("physics", ()),
    )]


def test_failed_arm_write_does_not_advance_physics_or_report_success():
    calls = []
    def fail(_):
        raise RuntimeError("arm write failed")
    scene = SimpleNamespace(
        go2=SimpleNamespace(set_planar_velocity=lambda *args: None,
                            set_arm_q_for_metrics=lambda *args: None),
        apply_sim_q=fail,
        step=lambda: calls.append("step"),
    )
    with pytest.raises(RuntimeError, match="arm write failed"):
        probe.advance_controlled_step(scene, (0., 0., 0.),
            SimpleNamespace(linear_m=0., roll_rad=0., theta1_rad=0., theta2_rad=0.))
    assert calls == []
