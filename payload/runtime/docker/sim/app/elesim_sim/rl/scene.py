"""Lazy Genesis scene construction independent of task, object and reward."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PhysicsConfig:
    dt: float = .01
    substeps: int = 4
    max_collision_pairs: int = 1024
    show_viewer: bool = False
    max_contacts: int | None = None

    def __post_init__(self):
        if (not math.isfinite(self.dt) or self.dt <= 0
                or type(self.substeps) is not int or self.substeps < 1
                or type(self.max_collision_pairs) is not int or self.max_collision_pairs < 1):
            raise ValueError('Invalid physics configuration')
        if self.max_contacts is not None and (type(self.max_contacts) is not int or self.max_contacts < 1):
            raise ValueError('max_contacts must be a positive integer or None')


def init_genesis(backend='gpu', seed=0):
    if backend not in ('gpu', 'cpu'):
        raise ValueError('backend must be gpu or cpu')
    import genesis as gs

    if not getattr(gs, '_initialized', False):
        gs.init(backend=gs.gpu if backend == 'gpu' else gs.cpu,
                logging_level='warning', seed=int(seed))


def create_scene(config: PhysicsConfig):
    """Caller adds arbitrary assets/cameras before scene.build(n_envs=...)."""
    import genesis as gs

    rigid = {'max_collision_pairs': config.max_collision_pairs}
    if config.max_contacts is not None:
        rigid['max_contacts'] = config.max_contacts
    return gs.Scene(
        sim_options=gs.options.SimOptions(dt=config.dt, substeps=config.substeps),
        rigid_options=gs.options.RigidOptions(**rigid),
        show_viewer=config.show_viewer,
    )
