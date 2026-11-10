"""Bounded real-valued GA over waypoint positions and segment durations."""
import numpy as np


def seed_population(count, knots, start, goal, lower, upper, rng, *, lift_goal=None):
    positions = np.linspace(start, goal, knots + 1)[1:]
    if lift_goal is not None:
        split=max(1,knots//2)
        positions=np.vstack((np.linspace(start,goal,split+1)[1:],
                             np.linspace(goal,lift_goal,knots-split+1)[1:]))
    population = np.zeros((count, knots, 5))
    population[:, :, :4] = (positions-lower)/(upper-lower)
    population[:, :, :4] += rng.normal(0, .18, (count, knots, 4))
    population[:, :, 4] = rng.uniform(.1, 1, (count, knots))
    population[0, :, :4] = (positions-lower)/(upper-lower)
    return np.clip(population, 0, 1)


def decode(genomes, start, lower, upper, rates, steps):
    """Fixed Home, variable knot times, bounded commands and per-step slew."""
    n, knots, _ = genomes.shape
    if not np.isfinite(genomes).all() or np.any((genomes < 0) | (genomes > 1)):
        raise ValueError('Genome must be finite and normalized')
    paths = np.empty((n, steps+1, 4)); paths[:, 0] = start
    for i, genome in enumerate(genomes):
        # Reserve the last third of the episode for holding the last command.
        times = np.r_[0., np.cumsum(.1+genome[:, 4])]
        times *= (steps*2/3)/times[-1]
        points = np.vstack((start, lower+genome[:, :4]*(upper-lower)))
        for step in range(1, steps+1):
            target = np.array([np.interp(step, times, points[:, j]) for j in range(4)])
            paths[i, step] = paths[i, step-1]+np.clip(target-paths[i, step-1], -rates, rates)
    return paths


def quality(report):
    # No q-match, historical touch, or PPO-return credit. Feasibility first.
    if report['fault'] or report['wrong_face_any'] or report.get('object_motion_violation',False):
        return (0, 0, 0, 0, 0, 0)
    if report['success']:
        return (2, 0, 0, 0, 0, -report['duration_s'])
    return (1, report['hold_fraction'], report['lift_fraction'],
            report['grasp_fraction'], -report['terminal_distance_m'], 0)


def breed(population, reports, rng):
    order = sorted(range(len(reports)), key=lambda i: quality(reports[i]), reverse=True)
    elite_count = max(1, len(population)//8)
    result = [population[i].copy() for i in order[:elite_count]]
    parents = population[order[:max(2, len(population)//2)]]
    while len(result) < len(population):
        a, b = parents[rng.integers(len(parents), size=2)]
        child = np.where(rng.random(a.shape) < .5, a, b)
        child += (rng.random(a.shape) < .3)*rng.normal(0, .15, a.shape)
        if rng.random() < .1: child = rng.random(a.shape)
        result.append(np.clip(child, 0, 1))
    return np.asarray(result)
