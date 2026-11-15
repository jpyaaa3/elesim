"""Pure contact classification shared by runtime and research; no Genesis imports."""
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



def camera_side_axes_from_frames(frames):
    """Link-local protected-side axes, valid even when the arm is bent.

    Nodes bend about local Y. Their protected side remains local Z; rotating
    every node's axis from node9 would reverse the side on deeply curled links.
    The gripper and camera have fixed relative rotations to node9.
    """
    terminal=frames['node9'];camera=frames['camera']
    offset=terminal[:3,:3].T@(camera[:3,3]-terminal[:3,3])
    if abs(offset[2])<.02:raise ValueError('Camera side cannot be identified from the model')
    axis=np.array([0.,0.,np.sign(offset[2])])
    result={f'node{i}':axis.copy() for i in range(10)}
    for name in ('gripper_base','gripper_claw_left','gripper_claw_right'):
        if name in frames:result[name]=frames[name][:3,:3].T@terminal[:3,:3]@axis
    return result


def assembly_contact_pairs(by_name):
    pairs=[(by_name[f'node{i}'],by_name[f'node{i+1}']) for i in range(9)
           if f'node{i}' in by_name and f'node{i+1}' in by_name]
    if 'wedge' in by_name and 'node0' in by_name:pairs.append((by_name['wedge'],by_name['node0']))
    rigid=[by_name[n] for n in ('node9','gripper_base','gripper_claw_left','gripper_claw_right','camera') if n in by_name]
    return pairs+[(a,b) for i,a in enumerate(rigid) for b in rigid[i+1:]]


def arm_contact_faults(raw,roles,objects,assembly_pairs,*,max_penetration=.005,xp=np):
    """Roles: 0 environment, 1/2 arm, 3 protected body, 4 camera.

    Leg/floor contacts are allowed. Moving arm/environment and protected-body/
    object contacts are forbidden. Exact adjacent assembly pairs are exempt.
    """
    a=xp.clip(raw['link_a'],0,len(roles)-1);b=xp.clip(raw['link_b'],0,len(roles)-1)
    ma=(roles[a]==1)|(roles[a]==2);mb=(roles[b]==1)|(roles[b]==2)
    pa=(roles[a]>=3);pb=(roles[b]>=3)
    ca=roles[a]==4;cb=roles[b]==4;oa=objects[a];ob=objects[b]
    force=xp.sqrt((raw['force_a']**2).sum(-1));valid=raw['valid_mask']
    adjacent=xp.zeros_like(valid)
    for left,right in assembly_pairs:adjacent|=((a==left)&(b==right))|((a==right)&(b==left))
    forbidden=(ma&~ob)|(mb&~oa)|(oa&pb)|(ob&pa)|(ca&~ob)|(cb&~oa)
    # An ordinary arm/object contact is admissible, including between nodes.
    active=valid&(force>1e-4)
    bad=(active&forbidden&~adjacent).any(-1)
    relevant=ma|mb|ca|cb|(oa&pb)|(ob&pa)
    penetration=raw.get('penetration',xp.zeros_like(force))
    bad|=(valid&relevant&~adjacent&(penetration>max_penetration)).any(-1)
    bad|=(valid&~xp.isfinite(force)).any(-1)
    arm_object=(ma&ob)|(mb&oa)
    bad|=(active&arm_object&(~xp.isfinite(raw['normal']).all(-1)
        |~xp.isfinite(raw['position']).all(-1)|((raw['normal']**2).sum(-1)<.25))).any(-1)
    return bad
