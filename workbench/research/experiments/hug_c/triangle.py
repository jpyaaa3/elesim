"""Maximize rasterized S∩triangle; do not require the triangle to lie in S."""
import itertools
import math
import numpy as np


def cross(a,b):
    return a[...,0]*b[...,1]-a[...,1]*b[...,0]


def weights(vertices, points):
    v, p = np.asarray(vertices, float), np.asarray(points, float)
    if v.shape != (3, 2) or p.shape[-1:] != (2,) or not np.isfinite(v).all() or not np.isfinite(p).all():
        raise ValueError('Require finite q coordinates')
    matrix = (v[1:]-v[0]).T
    if abs(np.linalg.det(matrix)) < 1e-8:
        raise ValueError('Degenerate q triangle')
    uv = np.linalg.solve(matrix, (p-v[0]).reshape(-1, 2).T).T
    return np.column_stack((1-uv.sum(1), uv)).reshape((*p.shape[:-1], 3))


def contains(vertices, points):
    v, p = np.asarray(vertices), np.asarray(points)
    a, b, c = v
    area = cross(b-a, c-a)
    if abs(area) < 1e-8:
        return np.zeros(len(p), bool)
    sign = 1 if area > 0 else -1
    return ((cross(b-a, p-a)*sign >= -1e-8) &
            (cross(c-b, p-b)*sign >= -1e-8) &
            (cross(a-c, p-c)*sign >= -1e-8))


class RasterArea:
    """Scanline integration of a triangle against occupied q cells.

    Fractional edge cells are included. Four vertical samples per cell avoid
    the boundary-count bias of counting feasible vertices inside a triangle.
    """
    def __init__(self, points, step, origin=-180., extent=180., subdivisions=4):
        self.step,self.origin,self.subdivisions=step,origin,subdivisions
        n=int(round(extent/step));self.grid=np.zeros((n,n),float)
        ij=np.floor((np.asarray(points)-origin)/step).astype(int)
        self.grid[ij[:,1],ij[:,0]]=1
        self.prefix=np.pad(np.cumsum(self.grid,axis=1),((0,0),(1,0)))
        self.ys=origin+(np.arange(n*subdivisions)+.5)*step/subdivisions
        self.row=np.arange(n*subdivisions)//subdivisions

    def covered(self, vertices):
        v=np.asarray(vertices);lo=np.searchsorted(self.ys,v[:,1].min());hi=np.searchsorted(self.ys,v[:,1].max())
        y=self.ys[lo:hi];rows=self.row[lo:hi]
        if not len(y):return 0.
        left=np.full(len(y),np.inf);right=np.full(len(y),-np.inf)
        for a,b in zip(v,np.roll(v,-1,axis=0)):
            if abs(b[1]-a[1])<1e-12:continue
            t=(y-a[1])/(b[1]-a[1]);x=a[0]+t*(b[0]-a[0]);active=(t>=0)&(t<1)
            left=np.minimum(left,np.where(active,x,np.inf));right=np.maximum(right,np.where(active,x,-np.inf))
        valid=np.isfinite(left)&np.isfinite(right);rows=rows[valid]
        def integral(x):
            u=np.clip((x[valid]-self.origin)/self.step,0,self.grid.shape[1]);i=np.minimum(u.astype(int),self.grid.shape[1]-1)
            return self.prefix[rows,i]+(u-i)*self.grid[rows,i]
        return float(np.maximum(0,integral(right)-integral(left)).sum()*self.step**2/self.subdivisions)


def select(rows, step, *, seed=42, pool_size=64, restarts=12):
    """Exact enumeration for small pools; seeded vertex exchange otherwise.

    The reported objective is a raster sample count times step², not a proof
    of the continuous global optimum. All vertex candidates remain in S.
    """
    p = np.array([r['q_deg'] for r in rows], float)
    if len(p) < 3:
        raise ValueError('Need at least three feasible C targets')
    keys = {tuple(np.floor((x+180)/step).astype(int)): i for i, x in enumerate(p)}
    raster=RasterArea(p,step)
    boundary = [i for key, i in keys.items() if any((key[0]+dx, key[1]+dy) not in keys
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))]
    rng = np.random.default_rng(seed)
    if len(boundary) > pool_size:
        # Keep the sampled extremes as well as distributed boundary candidates.
        directions = np.column_stack((np.cos(np.linspace(0, 2*np.pi, 16, endpoint=False)),
                                      np.sin(np.linspace(0, 2*np.pi, 16, endpoint=False))))
        extreme = np.argmax(p@directions.T, axis=0)
        pool = np.unique(np.r_[extreme, rng.choice(boundary, pool_size, replace=False)])
    else:
        pool = np.array(boundary)
    mu = np.array([r['mu_min'] for r in rows])

    def score(ids):
        v = p[list(ids)]
        area = abs(cross(v[1]-v[0], v[2]-v[0]))/2
        if area < 1e-8:
            return (-1, -math.inf, -math.inf)
        count = round(min(area,raster.covered(v)),9)
        perimeter2 = sum(float(np.sum((v[i]-v[(i+1)%3])**2)) for i in range(3))
        return count, -float(mu[list(ids)].max()), area/perimeter2

    exact = len(rows) <= 24
    if exact:
        best = max(itertools.combinations(range(len(rows)), 3), key=score)
    else:
        trials = [tuple(rng.choice(pool, 3, replace=False)) for _ in range(512)]
        trials.sort(key=score, reverse=True)
        winners = []
        for start in trials[:restarts]:
            ids, value = list(start), score(start)
            for _ in range(12):
                changed = False
                for axis in range(3):
                    for candidate in pool:
                        proposed = ids.copy(); proposed[axis] = int(candidate)
                        result = score(proposed)
                        if result > value:
                            ids, value, changed = proposed, result, True
                if not changed:
                    break
            winners.append(tuple(ids))
        best = max(winners, key=score)
        # Refine the best result on every observed boundary point.
        ids, value = list(best), score(best)
        for _ in range(5):
            changed = False
            for axis in range(3):
                for candidate in boundary:
                    proposed = ids.copy(); proposed[axis] = candidate
                    result = score(proposed)
                    if result > value:
                        ids, value, changed = proposed, result, True
            if not changed:
                break
        best = tuple(ids)
    result = score(best)
    if result[0] < 0:
        raise ValueError('Feasible C solutions do not form a nondegenerate triangle')
    v = p[list(best)]; area = float(abs(cross(v[1]-v[0], v[2]-v[0]))/2)
    return list(map(int, best)), dict(method='exhaustive-raster' if exact else 'seeded-boundary-vertex-exchange',
        continuous_global_optimum_proven=False, resolution_total_deg=step,
        feasible_samples=len(rows), covered_equivalent_cells=result[0]/step**2, coverage=result[0]/(len(rows)*step**2),
        solution_area_deg2=len(rows)*step**2, covered_area_deg2=result[0],
        triangle_area_deg2=area, discarded_area_deg2=max(0., area-result[0]),
        discarded_area_note='Occupied-cell area with fractional x edges and 4 y subsamples per cell',
        vertices_total_deg=v.tolist(), search_seed=seed)
