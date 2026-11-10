"""Caption-free, whole-population video; presentation never enters GA fitness."""
from contextlib import contextmanager
import math
from pathlib import Path
import numpy as np


def due(generation_number, interval=10):
    return generation_number > 0 and generation_number % interval == 0


def overview(offsets):
    offsets=np.asarray(offsets,dtype=float)
    if offsets.ndim!=2 or offsets.shape[1]!=3 or not np.isfinite(offsets).all():
        raise ValueError('Expected finite world-space environment offsets')
    # Bounding sphere includes arm, object and Go2 at every grid location.
    low=offsets.min(0)+[-1.,-1.,0.]
    high=offsets.max(0)+[1.2,1.2,1.5]
    center=(low+high)/2
    direction=np.array([0.,-.45,1.]);direction/=np.linalg.norm(direction)
    forward=-direction
    right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
    up=np.cross(right,forward)
    corners=np.array([[x,y,z] for x in (low[0],high[0])
                      for y in (low[1],high[1]) for z in (low[2],high[2])])-center
    vertical=math.tan(math.radians(25))*.92
    horizontal=vertical*16/9
    depth=corners@forward
    distance=max(np.max(abs(corners@up)/vertical-depth),
                 np.max(abs(corners@right)/horizontal-depth),1.)
    return dict(pos=tuple(center+distance*direction),lookat=tuple(center),up=(0.,0.,1.))


def create_env(experiment,case,assets,*,n_envs,backend):
    """Scoped factory substitution keeps the pinned task physics source intact."""
    import genesis as gs
    from unittest.mock import patch
    from workbench.research.experiments.hug_c import environment
    cameras=[]
    def scene_factory(config):
        rigid=dict(max_collision_pairs=config.max_collision_pairs)
        if config.max_contacts is not None:rigid['max_contacts']=config.max_contacts
        scene=gs.Scene(sim_options=gs.options.SimOptions(dt=config.dt,substeps=config.substeps),
            rigid_options=gs.options.RigidOptions(**rigid),show_viewer=False,
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(n_envs)),
                show_world_frame=False,show_link_frame=False))
        camera=scene.add_camera(res=(1920,1080),pos=(1,-2,3),lookat=(0,0,.5),
                               fov=50,near=.1,far=1000.,GUI=False,env_idx=None)
        build=scene.build
        def build_grid(**kwargs):
            # Genesis drawing offsets do not change any lane's physics poses.
            # Genesis rows advance world Y; world X is horizontal in this view.
            kwargs.update(env_spacing=(1.8,1.8),n_envs_per_row=math.ceil(math.sqrt(n_envs*9/16)))
            return build(**kwargs)
        scene.build=build_grid
        cameras.append(camera)
        return scene
    with patch.object(environment,'create_scene',scene_factory):
        env=environment.HugEnv(experiment,case,assets,n_envs=n_envs,backend=backend,auto_reset=False)
    env.scene.cameras['parade']=cameras[0]
    offsets=env.scene.scene.envs_offset
    if hasattr(offsets,'cpu'):offsets=offsets.cpu().numpy()
    cameras[0].set_pose(**overview(offsets))
    return env


@contextmanager
def record(env,path,every=4):
    from workbench.research.experiments.hug_interpolation.video import make_writer
    path=Path(path)
    if path.exists() or path.suffix.lower()!='.mp4':raise ValueError('Choose a new MP4')
    camera=env.scene.cameras['parade'];previous=env.tick_monitor
    writer=make_writer(path,1/(env.s['dt']*every));ticks=0
    def capture(current):
        nonlocal ticks
        ticks+=1
        if ticks%every==0:writer.append_data(camera.render()[0])
        if previous is not None:previous(current)
    env.tick_monitor=capture
    try:
        # evaluate() resets before the first physics tick; avoid stale initial frame.
        yield
    finally:
        env.tick_monitor=previous
        writer.close()
