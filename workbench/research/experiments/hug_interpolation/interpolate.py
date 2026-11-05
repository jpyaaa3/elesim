"""Local, fixed-diameter interpolation of already replayed anchor trajectories.

This is a candidate generator, not a certificate of dynamic grasp feasibility.
"""
import numpy as np
from elesim_sim.rl.trajectory import Trajectory

CHANNELS = ('linear_m', 'roll_rad', 'theta1_rad', 'theta2_rad')
# Entire contexts must match: model hash, physics, controller and frame are not
# permitted to change silently between independently trained anchors.
CONTEXT = ('band', 'roll_deg', 'diameter_mm', 'branch', 'simulation_context')


def weights(vertices, point):
    vertices = np.asarray(vertices, dtype=float)
    point = np.asarray(point, dtype=float)
    if vertices.shape != (3, 2) or point.shape != (2,) or not np.isfinite(vertices).all() or not np.isfinite(point).all():
        raise ValueError('Need three finite XY vertices and one XY query')
    matrix = np.vstack((vertices.T, np.ones(3)))
    if np.linalg.cond(matrix) > 1e10:
        raise ValueError('Degenerate anchor triangle')
    result = np.linalg.solve(matrix, np.r_[point, 1.])
    if np.any(result < -1e-9):
        raise ValueError('Extrapolation outside the anchor triangle is forbidden')
    result = np.maximum(result, 0)
    return result / result.sum()


def interpolate(anchors, target, *, samples=201):
    """Blend time-normalized residuals around caller-supplied start/goal commands.

Target goal must come from the geometric solver, not Cartesian angle averaging.
No topology is inferred from band number alone; the experiment must supply a
local connected branch ID and test the interior independently in physics.
"""
    if not isinstance(target.get('simulation_context'), dict) or not target['simulation_context']:
        raise ValueError('A calibrated simulation_context is required')
    if not target.get('branch'):
        raise ValueError('An explicit local branch is required')
    if len(anchors) != 3 or isinstance(samples, bool) or int(samples) != samples or samples < 2:
        raise ValueError('Need three anchors and >=2 samples')
    for anchor in anchors:
        anchor.validate()
        if anchor.channels != CHANNELS:
            raise ValueError('Unsupported command convention')
        for key in CONTEXT:
            if key not in target or key not in anchor.metadata or anchor.metadata[key] != target[key]:
                raise ValueError(f'Incompatible or missing {key}')
        if anchor.diagnostics.get('replay_success') is not True:
            raise ValueError('Replay every original anchor successfully before interpolation')
    if target['band'] not in (1, 2) or target['roll_deg'] != (-90 if target['band'] == 1 else 90):
        raise ValueError('Band/roll mismatch')
    w = weights([a.metadata['xy_mm'] for a in anchors], target['xy_mm'])
    start = np.asarray(target['start_command'], dtype=float)
    goal = np.asarray(target['goal_command'], dtype=float)
    if start.shape != (4,) or goal.shape != (4,):
        raise ValueError('Start and goal must be four physical command values')
    duration = float(target['duration_s'])
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('duration_s must be positive')
    motion_end = float(target.get('motion_end_s', duration))
    if not 0 < motion_end <= duration:
        raise ValueError('Invalid approach/hold boundary')
    normalized_time = np.linspace(0, 1, int(samples))
    phase = np.minimum(normalized_time*duration/motion_end, 1)[:, None]
    commands = (1-phase)*start + phase*goal
    for weight, anchor in zip(w, anchors):
        if 'motion_end_s' in target:
            if anchor.metadata.get('motion_end_s') != motion_end or not np.isclose(anchor.times[-1], duration):
                raise ValueError('Anchor phase timing differs')
            hold = anchor.commands[anchor.times >= motion_end-1e-9]
            if not np.allclose(hold, anchor.commands[-1], atol=1e-6):
                raise ValueError('Anchor does not hold its final command')
        baseline = (1-phase)*anchor.commands[0] + phase*anchor.commands[-1]
        commands += weight*(anchor.sample(normalized_time*anchor.times[-1])-baseline)
    if np.any(np.abs(commands[:, 0]) > 1e-9) or not np.allclose(commands[:, 1], np.deg2rad(target['roll_deg'])):
        raise ValueError('Interpolation changed fixed linear/roll coordinates')
    limit = np.pi/5  # this experiment uses five nodes per segment
    if np.any(np.abs(commands[:, 2:]) > limit+1e-6) or np.any(commands[:, 3] > 1e-9):
        raise ValueError('Candidate violates bend limits or bends seg2 toward ZED')
    metadata = dict(target, interpolation='time-normalized-endpoint-residual',
                    weights=w.tolist(), anchor_ids=[a.metadata['case_id'] for a in anchors])
    return Trajectory(normalized_time*duration, commands, CHANNELS, metadata,
                      dict(replay_success=None, physics_validated=False)).validate()
