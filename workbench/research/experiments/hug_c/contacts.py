"""Camera-side contact classification in each link's own frame."""
import numpy as np


def cross(a,b):
    return a[...,[1,2,0]]*b[...,[2,0,1]]-a[...,[2,0,1]]*b[...,[1,2,0]]


def inverse_rotate(quaternion, vector):
    u=-quaternion[...,1:];t=2*cross(u,vector)
    return vector+quaternion[...,:1]*t+cross(u,t)


def classify_faces(position,normal,link_position,link_quaternion,camera_axis,*,margin=.001,normal_tolerance=.15,xp=np):
    """Return (proper face, camera face), including tip/corner contacts.

    `normal` points outward from the ROBOT, irrespective of the engine's A/B
    order. A cap normal can be tangent to the camera-side axis, so contact
    position chooses the half-face and the normal rejects contradictory faces.
    The 1 mm centre strip is neutral; neutral contacts earn no grasp credit.
    """
    local=inverse_rotate(link_quaternion,position-link_position)
    n=inverse_rotate(link_quaternion,normal)
    n=n/xp.sqrt(xp.clip((n*n).sum(-1)[...,None],1e-20,None))
    side=(local*camera_axis).sum(-1);direction=(n*camera_axis).sum(-1)
    return (side < -margin)&(direction < normal_tolerance), (side > margin)&(direction > -normal_tolerance)


def object_contact_faces(raw,positions,quaternions,global_to_local,axes,roles,objects,settings,xp=np):
    """Classify engine contacts without A/B or global/local link-ID assumptions."""
    a=xp.clip(raw['link_a'],0,len(roles)-1);b=xp.clip(raw['link_b'],0,len(roles)-1)
    robot_a=(roles[a]>0)&objects[b];robot_b=(roles[b]>0)&objects[a]
    robot=xp.where(robot_a,a,b);local=global_to_local[robot]
    kw={'device':positions.device} if hasattr(positions,'device') and xp is not np else {}
    env=xp.arange(positions.shape[0],**kw)[:,None]
    # Pinned Genesis 1.4.5 normals point B -> A (verified with box/plane).
    # Outward from the robot is the reverse when the robot occupies A.
    normal=xp.where(robot_a[...,None],-raw['normal'],raw['normal'])
    proper,bad=classify_faces(raw['position'],normal,positions[env,local],quaternions[env,local],axes[robot],
        margin=settings['face_margin_m'],normal_tolerance=settings['face_normal_tolerance'],xp=xp)
    force=xp.sqrt((raw['force_a']**2).sum(-1))
    active=raw['valid_mask']&(robot_a|robot_b)&(force>settings['face_force_min_n'])
    good=active&proper;bad=active&bad
    touch=xp.stack(((good&(roles[robot]==1)).any(-1),(good&(roles[robot]==2)).any(-1)),-1)
    return touch,bad.any(-1)


def camera_side_axes():
    """Derive the side sign and gripper rotations from the actual URDF."""
    import xml.etree.ElementTree as ET
    from .configure import BUNDLE,fk
    root=ET.parse(BUNDLE/'robot.urdf').getroot()
    terminal=fk(root,'node9');camera=fk(root,'camera')
    offset=terminal[:3,:3].T@(camera[:3,3]-terminal[:3,3])
    if abs(offset[2])<.02:
        raise ValueError('Camera side cannot be identified from the model')
    axis=np.array([0.,0.,np.sign(offset[2])])
    result={f'node{i}':axis.tolist() for i in range(10)}
    for name in ('gripper_base','gripper_claw_left','gripper_claw_right'):
        rotation=fk(root,name)[:3,:3]
        result[name]=(rotation.T@terminal[:3,:3]@axis).tolist()
    return result
