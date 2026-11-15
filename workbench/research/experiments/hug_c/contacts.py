"""Task model adapter; contact math is shared with the Sim runtime."""
import numpy as np
from elesim_sim.simulation.contact_metrics import (
    cross, inverse_rotate, classify_faces, object_contact_faces,
)

def camera_side_axes():
    """Derive the side sign and gripper rotations from the actual URDF."""
    import xml.etree.ElementTree as ET
    from .configure import BUNDLE,fk
    root=ET.parse(BUNDLE/'robot.urdf').getroot()
    from elesim_sim.simulation.contact_metrics import camera_side_axes_from_frames
    names=[f'node{i}' for i in range(10)]+['camera','gripper_base','gripper_claw_left','gripper_claw_right']
    frames={name:fk(root,name) for name in names}
    return {name:axis.tolist() for name,axis in camera_side_axes_from_frames(frames).items()}
