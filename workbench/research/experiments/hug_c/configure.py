"""Home-derived C-hug contract and task-local robot assets; no GPU imports."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from workbench.research.experiments.hug_interpolation.configure import (
    BUNDLE, ROOT, VERSIONS, asset_digest, fk, materialize_robot, sha, write_json)
from workbench.research.experiments.wrap_grasp.app_config import load_mechanism, UI_PRESETS

DEFAULT_NUM_ENVS = 512
CONTROL_MODE = 'policy-four-dof-home-lift90-v4'
EXPERIMENT_VERSION = 5
POLICY_VERSION = 4
REWARD_CONTRACT = 'bounded-milestones-lift90-v5'
LIFT_CONTRACT = 'canonical-wrap-minus90-to-roll0-hold-v1'
DEFAULTS = dict(dt=.01, solver_substeps=4, max_collision_pairs=256, max_contacts=512,
    action_ticks=10, episode_steps=200, hold_duration_s=5.,
    action_delta_linear_m=.005, action_delta_roll_rad=.10, action_delta_bend_total_rad=.15,
    friction=.5, floor_friction=.01, mass_kg=.30, height_m=1.1, spawn_height_m=.32,
    lift_clearance_m=.02, hold_slip_m=.01, hold_contact_fraction=.8,
    lift_roll_target_rad=0.,lift_roll_tolerance_rad=math.radians(5.),
    max_penetration_m=.005, max_joint_spread_rad=.05, kp=200., kv=20., torque_limit=100.,
    linear_min_m=-.17333333333333334, linear_max_m=0.,
    roll_min_rad=-math.pi/2, roll_max_rad=math.pi/2, bend_per_node_rad=math.pi/5, seed=42,
    reward_gamma=.995,reward_q_hint=1.,reward_grasp=3.,reward_lift=5.,reward_hold=10.,reward_success=30.,
    penalty_wrong_face_per_s=1.,penalty_drop=3.,penalty_fault=30.,penalty_timeout=5.,
    penalty_action=.001,penalty_time_per_s=.3,grasp_confirm_s=.2,
    penalty_workspace_per_s=.2,workspace_margin_m=.05,workspace_scale_m=.10,
    face_margin_m=.001,face_normal_tolerance=.15,face_force_min_n=.01)
CHANNELS = ('linear_m', 'roll_rad', 'theta1_rad', 'theta2_rad')
ACTIONS = ('linear', 'roll', 'q3_total', 'q4_total')
OBSERVATIONS = ('linear_m', 'roll_rad', 'q3_total_rad', 'q4_total_rad',
    'command_linear_m', 'command_roll_rad', 'command_q3_total_rad', 'command_q4_total_rad',
    'goal_q3_total_rad', 'goal_q4_total_rad',
    'object_gripper_dx_m', 'object_gripper_dy_m', 'object_gripper_dz_m',
    'object_vx_m_s', 'object_vy_m_s', 'object_vz_m_s', 'diameter_m',
    'grasp_seen', 'hold_fraction', 'episode_fraction')
PHYSICS_KEYS = ('dt','solver_substeps','max_collision_pairs','max_contacts','friction','floor_friction',
                'mass_kg','height_m','spawn_height_m','max_penetration_m','max_joint_spread_rad',
                'kp','kv','torque_limit')


def validate_settings(s):
    if set(s) != set(DEFAULTS):
        raise ValueError('Unknown/missing four-DoF C-hug setting')
    allow_nonpositive = {'seed','linear_min_m','linear_max_m','roll_min_rad','lift_roll_target_rad'}
    for k,v in s.items():
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):
            raise ValueError(f'Invalid setting {k}')
        if k not in allow_nonpositive and v<=0:
            raise ValueError(f'{k} must be positive')
    for k in ('seed','solver_substeps','max_collision_pairs','max_contacts','action_ticks','episode_steps'):
        if type(s[k]) is not int or s[k]<(0 if k=='seed' else 1):
            raise ValueError(f'{k} must be a valid integer')
    if not .01<=s['floor_friction']<s['friction']<=5 or not 0<s['hold_contact_fraction']<=1:
        raise ValueError('Invalid friction/contact fraction')
    if s['hold_duration_s']>=episode_duration(s) or not np.isclose(s['hold_duration_s']/s['dt'],round(s['hold_duration_s']/s['dt'])):
        raise ValueError('Hold must fit within episode and contain whole physics ticks')
    if not 0<s['reward_gamma']<1 or not 0<s['face_normal_tolerance']<1:
        raise ValueError('Invalid reward discount/contact normal tolerance')
    if (s['lift_roll_target_rad']!=0 or not 0<s['lift_roll_tolerance_rad']<=math.radians(10)
        or not s['roll_min_rad']<=0<=s['roll_max_rad']):
        raise ValueError('Lift requires canonical roll 0 with a positive tolerance <=10 degrees')
    if (not s['dt']<=s['grasp_confirm_s']<s['hold_duration_s']
        or not np.isclose(s['grasp_confirm_s']/s['dt'],round(s['grasp_confirm_s']/s['dt']))):
        raise ValueError('Grasp confirmation must contain whole physics ticks and precede hold')
    if (s['linear_min_m']<-.173333333334 or s['linear_max_m']>0 or s['linear_min_m']>=s['linear_max_m']
        or s['roll_min_rad']< -math.pi/2 or s['roll_max_rad']>math.pi/2
        or s['roll_min_rad']>=s['roll_max_rad'] or s['bend_per_node_rad']>math.pi/5):
        raise ValueError('Requested limits exceed the physical mechanism')


def episode_duration(s):
    return s['episode_steps']*s['action_ticks']*s['dt']


def digest_code():
    paths=list(Path(__file__).parent.glob('*.py'))
    paths+=list((ROOT/'payload/runtime/docker/sim/app/elesim_sim/rl').glob('*.py'))
    paths += [ROOT/'workbench/research/experiments/hug_interpolation/environment.py',
              ROOT/'workbench/research/experiments/hug_interpolation/configure.py',
              ROOT/'workbench/research/experiments/hug_interpolation/video.py',
              ROOT/'workbench/research/experiments/wrap_grasp/app_config.py',
              ROOT/'workbench/research/experiments/wrap_grasp/scene.py',
              ROOT/'workbench/research/experiments/wrap_grasp/configs/loader.py',
              ROOT/'workbench/research/experiments/wrap_grasp/configs/default.yaml',
              ROOT/'payload/config/sim/config.yaml',
              ROOT/'payload/runtime/common/protocol/elesim_protocol/messages.py',
              ROOT/'payload/runtime/docker/pilot/app/elesim_pilot/pick/actions.py']
    h=hashlib.sha256()
    for p in sorted(set(paths)):
        h.update(str(p.relative_to(ROOT)).encode());h.update(p.read_bytes())
    return h.hexdigest()


def content_digest(data):
    return hashlib.sha256(json.dumps({k:data[k] for k in ('source','selection','cases','triangle')},
                                    sort_keys=True,allow_nan=False).encode()).hexdigest()


def configure(pilot, overrides=None):
    data=json.loads(Path(pilot).read_text())
    if data.get('schema')!='elesim-c-hug-pilot' or data.get('schema_version')!=1:
        raise ValueError('Require the function-derived C-hug pilot')
    if data.get('source',{}).get('kind')!='mapper-function' or data['source'].get('point_data_used') is not False:
        raise ValueError('Cases must be generated from mapper functions')
    from .source import validate_preparation
    validate_preparation(data['source'].get('preparation'))
    if data.get('selection',{}).get('preparation_envelope_required') is not True or any(
            c.get('preparation_passed') is not True
            or not isinstance(c.get('preparation_margin_mm'),(int,float))
            or not math.isfinite(c['preparation_margin_mm']) or c['preparation_margin_mm']<=0 for c in data['cases']):
        raise ValueError('Reselect all C targets from the preparation-clear solution area')
    s=dict(DEFAULTS,**(overrides or {}));validate_settings(s)
    screened_settings=data['source'].get('catalog_physics_settings')
    if screened_settings is not None and any(screened_settings.get(k)!=s[k] for k in PHYSICS_KEYS):
        raise ValueError('Physics settings differ from the screened catalog; screen a matching catalog first')
    if data['source'].get('catalog_contact_filtered') and any(screened_settings.get(k)!=s[k] for k in
            ('face_margin_m','face_normal_tolerance','face_force_min_n')):
        raise ValueError('Contact face settings differ from the screened catalog')
    if data['selection']['mu_max']>=s['friction']:
        raise ValueError('Simulation friction must exceed the geometric cutoff')
    mech=load_mechanism();home=list(mech.presets['home'])
    root=ET.parse(BUNDLE/'robot.urdf').getroot();frame=fk(root,'node0')
    if not np.allclose(frame[:3,:3],np.eye(3),atol=1e-8):
        raise ValueError('Unexpected arm mounting frame')
    arm_z=float(frame[2,3])+s['spawn_height_m']
    if not .05<arm_z<s['height_m']-.05:
        raise ValueError('Cylinder must span the horizontal arm plane with margin')
    ids=set()
    for c in data['cases']:
        q=np.array(c['q_deg'],float)
        if (c['case_id'] in ids or q.shape!=(2,) or not np.isfinite(q).all() or not np.all((q<0)&(q>=-180))
            or c['split'] not in ('train','validation','test') or not 0<=c['mu_min']<=data['selection']['mu_max']):
            raise ValueError('Invalid C-hug case')
        ids.add(c['case_id']);c['band']=1;c['roll_deg']=-90;c['branch']=data['triangle']['branch']
        c['start_command']=home.copy();c['goal_command']=[0.,-math.pi/2,*np.deg2rad(q/5).tolist()]
        from .control import command_bounds
        lower,upper=command_bounds(s)
        if any(np.any(np.array(command)<lower-1e-9) or np.any(np.array(command)>upper+1e-9)
               for command in (home,c['goal_command'])):
            raise ValueError('Home or geometric goal is outside configured physical bounds')
        x,y=c['xy_mm'];c['object_xyz_m']=[float(frame[0,3])+(y-144.5)/1000,float(frame[1,3])-x/1000,s['height_m']/2]
    if len([c for c in data['cases'] if c['split']=='train'])!=3 or set(data['triangle']['anchor_ids'])!={c['case_id'] for c in data['cases'] if c['split']=='train'}:
        raise ValueError('Exactly three training anchors required')
    from .contacts import camera_side_axes
    data.update(schema='elesim-c-hug-experiment',schema_version=EXPERIMENT_VERSION,control_mode=CONTROL_MODE,settings=s,
        home=dict(preset='home',ui_controls=UI_PRESETS['home'],command=home,
                  total_deg=[math.degrees(home[2])*5,math.degrees(home[3])*5],source=mech.source),
        simulation_context=dict(model_sha256=asset_digest(),code_sha256=digest_code(),
            pilot_sha256=content_digest(data),settings=s,versions=VERSIONS,
            criterion=(f'Free four-DoF motion from Home; C grasp, loaded clearance and roll 0 +/- '
                       f'{math.degrees(s["lift_roll_tolerance_rad"]):g} deg for {s["hold_duration_s"]:g} s'),
            lift_contract=LIFT_CONTRACT,
            control_mode=CONTROL_MODE,action_names=list(ACTIONS),
            observation_names=list(OBSERVATIONS),
            reward_contract=REWARD_CONTRACT,
            workspace_contract='fixed XY hull of Home/goal arm collision boxes and initial object, soft margin',
            camera_side_axes=camera_side_axes(),reward_gamma=s['reward_gamma'],
            preparation_contract=data['source']['preparation'],
            goal_error_convention='q3/q4 only, normalized by full physical bend range',
            command_convention='linear m, roll rad, theta1/theta2 per-node rad; q3/q4=5*theta',
            frame='map x=-world Y, y=world X; housing centre; node0 offset (0,144.5) mm'),
        readiness='configured-requires-physics-preflight')
    return data


def load_experiment(path):
    data=json.loads(Path(path).read_text())
    if data.get('schema')!='elesim-c-hug-experiment' or data.get('schema_version')!=EXPERIMENT_VERSION or data.get('control_mode')!=CONTROL_MODE:
        raise ValueError('Unsupported C-hug experiment: configure a fresh four-DoF run')
    validate_settings(data['settings']);ctx=data['simulation_context']
    from .contacts import camera_side_axes
    if (ctx.get('control_mode')!=CONTROL_MODE or ctx.get('action_names')!=list(ACTIONS)
        or ctx.get('observation_names')!=list(OBSERVATIONS)
        or ctx.get('camera_side_axes')!=camera_side_axes() or ctx.get('reward_gamma')!=data['settings']['reward_gamma']
        or ctx.get('reward_contract')!=REWARD_CONTRACT or ctx.get('lift_contract')!=LIFT_CONTRACT
        or ctx['pilot_sha256']!=content_digest(data) or ctx['settings']!=data['settings']
        or ctx['model_sha256']!=asset_digest() or ctx['code_sha256']!=digest_code()):
        raise ValueError('Experiment/model/code changed; configure a fresh run')
    return data



def validate_command(case,s,tick,value,previous=None):
    from .control import action_rates,command_bounds
    value=np.asarray(value,float);lower,upper=command_bounds(s)
    if value.shape!=(4,) or not np.isfinite(value).all() or np.any(value<lower-1e-6) or np.any(value>upper+1e-6):
        raise ValueError('Replay exceeds physical four-DoF bounds')
    if tick==0 and not np.allclose(value,case['start_command'],atol=1e-6):
        raise ValueError('Replay must start at the real Home preset')
    if previous is not None and np.any(abs(value-np.asarray(previous))>action_rates(s)/s['action_ticks']+1e-6):
        raise ValueError('Replay exceeds policy command rate limits')
    return value
