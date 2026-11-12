"""Reverse path genomes and lexicographic natural selection (no PPO reward)."""
import numpy as np


def seed_population(count, knots, start, home, lower, upper, rng):
    if count < 2 or knots < 1:
        raise ValueError('Require at least two candidates and one intermediate knot')
    # Last row holds terminal Home and searchable total movement duration.
    p = rng.random((count, knots + 1, 5))
    line = np.linspace(start, home, knots + 2)[1:]
    p[:, :, :4] = np.clip((line-lower)/(upper-lower) + rng.normal(0, .18, (count, knots+1, 4)), 0, 1)
    p[0, :, :4] = (line-lower)/(upper-lower)
    return p


def decode(genomes, start, home, lower, upper, rates, steps):
    """Per-physics-tick slew; fixed endpoints, freely timed intermediate knots.

    Duration gene spans one tick to the entire episode. Rate limiting may leave
    a slow/long path incomplete; it must never teleport to its Home endpoint.
    """
    g = np.asarray(genomes)
    if g.ndim != 3 or g.shape[2] != 5 or not np.isfinite(g).all() or np.any((g<0)|(g>1)):
        raise ValueError('Expected finite normalized (population, knots+1, 5) genomes')
    paths = np.empty((len(g), steps+1, 4)); paths[:, 0] = start
    for i, genome in enumerate(g):
        points = np.vstack((start, lower+genome[:-1, :4]*(upper-lower), home))
        weights = np.r_[.05+genome[:-1, 4], 1.]
        times = np.r_[0., np.cumsum(weights)]
        times *= (1+genome[-1, 4]*(steps-1))/times[-1]
        for t in range(1, steps+1):
            target = np.array([np.interp(t, times, points[:, j]) for j in range(4)])
            paths[i,t] = paths[i,t-1]+np.clip(target-paths[i,t-1], -rates, rates)
    return paths


def quality(r):
    """Completion drives selection even when travel disqualifies a parent.

    Among budget-only completions, minimize worst relative excess, then total
    relative excess. These are ranking terms, not additional acceptance limits.
    """
    over=(-r['motion_excess_max_ratio'],-r['motion_excess_sum_ratio'])
    if r['hard_stopped']:
        return (0, -r['last_safe_error'], *over, r['duration_s'])
    if r['success'] and not r['rejected']:
        return (3, -r['duration_s'], -r['path_length'], -r['peak_xy_m'], -r['peak_tilt_rad'])
    if r['completed']:
        return (2, *over, -r['duration_s'], -r['path_length'], -r['peak_xy_m'])
    return (1, r['home_window_s']>0, r['home_window_s'], -r['home_error'],
            *over, -r['object_contact_force_n'], -r['path_length'], -r['peak_xy_m'])


def breed(population, reports, rng):
    order = sorted(range(len(reports)), key=lambda i: quality(reports[i]), reverse=True)
    result = [population[i].copy() for i in order[:max(1,len(population)//8)]]
    parents = population[order[:max(2,len(population)//2)]]
    while len(result)<len(population):
        a,b = parents[rng.integers(len(parents),size=2)]
        child = np.where(rng.random(a.shape)<.5,a,b)
        child += (rng.random(a.shape)<.3)*rng.normal(0,.15,a.shape)
        if rng.random()<.1: child=rng.random(a.shape)
        result.append(np.clip(child,0,1))
    return np.asarray(result)
