"""A certified straight joint segment stays straight under delayed feedback."""
import numpy as np
import pytest
from elesim_pilot.robot.arm.approach import JointApproach


def test_different_axis_travel_times_do_not_leave_the_certified_segment():
    start=np.array([-.12,0.,-.59,.59]);goal=np.array([-.08,1.,-.2,.1])
    motion=JointApproach(start,goal);actual=start.copy();previous=start.copy()
    for _ in range(1000):
        command=motion.step(actual,.05)
        fractions=(command-start)/(goal-start)
        np.testing.assert_allclose(fractions,np.repeat(motion.fraction,4),atol=1e-12)
        assert np.all(abs(command-previous)<=np.array([.005,.04,.04,.04])*.05+1e-12)
        actual=command;previous=command
        if motion.fraction==1:break
    np.testing.assert_allclose(actual,goal)


def test_commands_wait_for_delayed_measurements_and_keep_the_same_tracking_gate():
    start=np.array([-.12,0.,-.59,.59]);goal=np.array([-.06,.4,-.2,.1])
    motion=JointApproach(start,goal)
    for _ in range(30):motion.step(start,.05)
    assert 0<motion.fraction<.1
    assert np.all(abs(motion.command-start)<=motion.tolerance*.8+1e-12)
    fraction=motion.fraction
    for _ in range(30):motion.step(start,.05)
    assert motion.fraction==pytest.approx(fraction)
    actual=motion.command.copy();motion.step(actual,.05)
    assert motion.fraction>fraction
    actual=motion.command.copy();actual[0]-=.0011
    with pytest.raises(RuntimeError,match='follow'):
        motion.step(actual,.05)
