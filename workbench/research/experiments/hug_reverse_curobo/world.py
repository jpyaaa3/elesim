"""Conservative cylinder obstacle in the measured arm plate frame."""
import numpy as np
from scipy.spatial.transform import Rotation


def cylinder_world(center, quaternion_wxyz, radius, height, plate_pos, plate_quat, strips=16):
    if radius <= 0 or height <= 0 or strips < 2:
        raise ValueError('Invalid cylinder dimensions')
    base = Rotation.from_quat(np.roll(plate_quat, -1))
    obj = Rotation.from_quat(np.roll(quaternion_wxyz, -1))
    relative = base.inv()*obj
    quat = np.roll(relative.as_quat(), 1).tolist()
    origin = base.inv().apply(np.asarray(center)-plate_pos)
    boxes = {}
    edges = np.linspace(-radius, radius, strips+1)
    for i, (left, right) in enumerate(zip(edges[:-1], edges[1:])):
        nearest = 0. if left <= 0 <= right else min(abs(left), abs(right))
        half_y = np.sqrt(max(0., radius*radius-nearest*nearest))
        location = origin+relative.apply([(left+right)/2, 0, 0])
        boxes[f'cylinder_{i:02d}'] = dict(dims=[right-left,2*half_y,height],pose=[*location.tolist(),*quat])
    return dict(cuboid=boxes)


def from_env(env):
    from workbench.research.experiments.hug_c.metrics import local_link_index
    idx = local_link_index(env.scene.robot, 'plate')
    def cpu(x): return x.detach().cpu().numpy()
    return cylinder_world(cpu(env.scene.object.get_pos())[0],cpu(env.scene.object.get_quat())[0],
        env.case['diameter_mm']/2000,env.s['height_m'],
        cpu(env.scene.robot.get_links_pos())[0,idx],cpu(env.scene.robot.get_links_quat())[0,idx])
