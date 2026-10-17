"""Keep the leading hips level while testing a bounded trailing crouch."""
from dataclasses import dataclass

import numpy as np
import pytest

from workbench.research.debug.profile_sim_walking import trailing_crouch_reference


@dataclass(frozen=True)
class Reference:
    desired_com_position: np.ndarray
    desired_rpy: np.ndarray


@pytest.mark.parametrize('vx', [.8, -.8])
@pytest.mark.parametrize('drop', [.01, .02])
def test_crouch_lowers_only_the_trailing_hip_reference(vx, drop):
    sample = Reference(np.array([1., 2., .3]), np.array([0., 0., .5]))
    changed = trailing_crouch_reference(sample, command_vx=vx, crouch_m=drop)
    height = changed.desired_com_position[2]
    pitch = changed.desired_rpy[1]
    leading_x = np.sign(vx)*.1934
    assert height-leading_x*np.sin(pitch) == pytest.approx(.3)
    assert height+leading_x*np.sin(pitch) == pytest.approx(.3-drop)
    np.testing.assert_array_equal(sample.desired_com_position, [1., 2., .3])
    np.testing.assert_array_equal(sample.desired_rpy, [0., 0., .5])
    np.testing.assert_array_equal(changed.desired_com_position[:2], [1., 2.])
    assert changed.desired_rpy[2] == .5


@pytest.mark.parametrize('vx,drop', [(.8, 0.), (0., .02)])
def test_disabled_or_stopped_reference_is_unchanged(vx, drop):
    sample = Reference(np.array([0., 0., .3]), np.zeros(3))
    assert trailing_crouch_reference(sample, command_vx=vx, crouch_m=drop) is sample


def test_crouch_fades_continuously_with_the_shaped_command():
    sample = Reference(np.array([0., 0., .3]), np.zeros(3))
    a = trailing_crouch_reference(sample, command_vx=.1, crouch_m=.02)
    b = trailing_crouch_reference(sample, command_vx=.2, crouch_m=.02)
    assert .3-a.desired_com_position[2] == pytest.approx(.5*(.3-b.desired_com_position[2]))


@pytest.mark.parametrize('vx,drop', [(float('nan'), .02), (.8, -.01), (.8, .04), (.8, float('inf'))])
def test_invalid_diagnostic_reference_is_rejected(vx, drop):
    with pytest.raises(ValueError):
        trailing_crouch_reference(Reference(np.zeros(3), np.zeros(3)), command_vx=vx, crouch_m=drop)
