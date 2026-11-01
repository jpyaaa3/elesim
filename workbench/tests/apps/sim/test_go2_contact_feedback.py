"""An arm pose write must not erase the previous completed contact solve."""
from types import SimpleNamespace

import pytest

pytest.importorskip('genesis')

from elesim_sim.runtime import Go2Locomotion, SimScene


@pytest.mark.parametrize('mirror', [False, True])
def test_scene_captures_contacts_after_successful_physics_before_pose_restoration(mirror):
    events = []
    go2 = Go2Locomotion.__new__(Go2Locomotion)
    go2._mirror = mirror
    go2._controller = SimpleNamespace(
        capture_physics_contacts=lambda: events.append('contacts'))
    go2.step = lambda: events.append('control')
    go2.reapply_last_mirror_pose = lambda: events.append('restore')
    scene = SimScene(
        go2=go2,
        scene=SimpleNamespace(step=lambda **_: events.append('physics')),
        update_visualizer_on_step=False,
    )
    scene.step()
    assert events == (['control', 'physics', 'restore'] if mirror
                      else ['control', 'physics', 'contacts'])
    assert scene.sim_step_count == 1


def test_failed_physics_does_not_publish_a_new_contact_sample():
    events = []
    def fail(**_):
        raise RuntimeError('physics failed')
    scene = SimScene(
        go2=SimpleNamespace(step=lambda: None, mirror_mode=False,
                            capture_physics_contacts=lambda: events.append('contacts')),
        scene=SimpleNamespace(step=fail), update_visualizer_on_step=False,
    )
    with pytest.raises(RuntimeError, match='physics failed'):
        scene.step()
    assert events == []
    assert scene.sim_step_count == 0
