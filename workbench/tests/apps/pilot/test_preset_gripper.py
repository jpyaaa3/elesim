import threading
from types import SimpleNamespace

import pytest

from elesim_pilot.pick.actions import ControlService


@pytest.mark.parametrize("source,expected", [("slider", None), ("target", True)])
def test_arm_presets_do_not_replay_gripper_but_workflows_can_send_it(source, expected):
    sent = []
    service = object.__new__(ControlService)
    service._sim_reset_lock = threading.RLock()
    service._sim_reset_in_progress = threading.Event()
    service.state = SimpleNamespace(controls_locked=False, linear=-0.1, roll=0, theta1=0,
                                    theta2=0, target_x=0, target_y=0, target_z=0,
                                    target_vx=1, target_vy=0, target_vz=0, claw_closed=True)
    service.client = SimpleNamespace(send_target_values=lambda **kw: sent.append(kw))
    service.send_current_target(source=source)
    assert sent[0]["claw_closed"] is expected
