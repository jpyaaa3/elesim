"""A bounded local flat-floor grid: unknown cells never become free by default."""
import heapq
import numpy as np
from .contracts import planar,angle


class LocalMap:
    def __init__(self,resolution=.04,max_cells=40000):
        self.resolution=resolution;self.max_cells=max_cells
        self.free=set();self.blocked=set();self.revision=0
    def cell(self,xy):return tuple(np.floor(np.asarray(xy)[:2]/self.resolution).astype(int))
    def observe(self,points,floor_z=0.):
        p=np.asarray(points,float)
        if p.ndim!=2 or p.shape[1]!=3 or len(p)>100000 or not np.isfinite(p).all():raise ValueError('invalid map points')
        for row in p:
            c=self.cell(row)
            if abs(row[2]-floor_z)<.025:self.free.add(c)
            elif floor_z+.04<row[2]<floor_z+1.7:self.blocked.add(c)
        self.free-=self.blocked
        if len(self.free|self.blocked)>self.max_cells:raise RuntimeError('local map budget exceeded')
        self.revision+=1
    def footprint_free(self,pose,footprint):
        cells=self._footprint_cells(pose,footprint)
        return all(c in self.free and c not in self.blocked for c in cells)
    def _footprint_cells(self,pose,footprint):
        t=planar(pose);local=np.asarray(footprint,float)
        points=local@t[:2,:2].T+t[:2,3]
        # Footprint is an interior-filled sampling, not just its boundary.
        cells=np.unique(np.floor(points/self.resolution).astype(int),axis=0)
        return [(int(x),int(y)) for x,y in cells]
    def footprint_status(self,pose,footprint):
        cells=self._footprint_cells(pose,footprint)
        blocked=[c for c in cells if c in self.blocked]
        unknown=[c for c in cells if c not in self.free and c not in self.blocked]
        return dict(total_cells=len(cells),unknown_cells=len(unknown),blocked_cells=len(blocked),
            unknown_xy_m=((np.array(unknown[:16]).reshape(-1,2)+.5)*self.resolution).tolist(),
            blocked_xy_m=((np.array(blocked[:16]).reshape(-1,2)+.5)*self.resolution).tolist())
    def segment_free(self,a,b,footprint):
        a,b=np.asarray(a,float),np.asarray(b,float);d=b-a;d[2]=angle(d[2])
        n=max(1,int(np.ceil(max(np.linalg.norm(d[:2])/(self.resolution/2),abs(d[2])/.05))))
        return all(self.footprint_free(a+d*t,footprint) for t in np.linspace(0,1,n+1))
    def path(self,start,goal,footprint,*,max_expansions=6000):
        start,goal=np.asarray(start,float),np.asarray(goal,float)
        if self.segment_free(start,goal,footprint):return np.array([start,goal])
        # Translation at start yaw followed by an explicitly checked final turn.
        # Holonomic GO2 can translate sideways; each segment is still swept.
        if not self.footprint_free(start,footprint):raise ValueError('current footprint not observed free')
        first,last=self.cell(start),self.cell(goal)
        queue=[(0.,first)];cost={first:0.};parent={};expanded=0
        while queue and expanded<max_expansions:
            _,c=heapq.heappop(queue);expanded+=1
            if c==last:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt=(c[0]+dx,c[1]+dy)
                p=np.r_[(np.array(nxt)+.5)*self.resolution,start[2]]
                before=start if c==first else np.r_[(np.array(c)+.5)*self.resolution,start[2]]
                if not self.segment_free(before,p,footprint):continue
                new=cost[c]+1
                if new>=cost.get(nxt,float('inf')):continue
                cost[nxt]=new;parent[nxt]=c
                heapq.heappush(queue,(new+np.linalg.norm(np.array(nxt)-last),nxt))
        if last not in cost:raise ValueError('no observed-free local base path')
        cells=[last]
        while cells[-1]!=first:cells.append(parent[cells[-1]])
        path=[start]+[np.r_[(np.array(c)+.5)*self.resolution,start[2]] for c in cells[::-1][1:]]+[goal]
        if not all(self.segment_free(a,b,footprint) for a,b in zip(path,path[1:])):
            raise ValueError('final approach or turn blocked')
        return np.asarray(path)


def rectangular_footprint(bounds,spacing=.02,padding=.03):
    bounds=np.asarray(bounds,float)
    if bounds.shape!=(2,2) or not np.isfinite(bounds).all() or np.any(bounds[:,0]>=bounds[:,1]):
        raise ValueError('invalid transport envelope')
    axes=[np.linspace(lo-padding,hi+padding,max(2,int(np.ceil((hi-lo+2*padding)/spacing))+1)) for lo,hi in bounds]
    return np.array(np.meshgrid(*axes)).reshape(2,-1).T


def transport_footprint(centers, radii, body_bounds=((- .38,.38),(-.25,.25)),spacing=.02,padding=.02):
    """Union of filled sphere projections and body, retaining concave gaps."""
    centers=np.asarray(centers,float);radii=np.asarray(radii,float)
    if centers.ndim!=2 or centers.shape[1]!=3 or radii.shape!=(len(centers),):raise ValueError('invalid transport spheres')
    chunks=[rectangular_footprint(body_bounds,spacing,padding)]
    for center,radius in zip(centers,radii):
        r=radius+padding
        axis=np.arange(-r,r+spacing*.5,spacing)
        xy=np.array(np.meshgrid(axis,axis)).reshape(2,-1).T
        chunks.append(xy[np.linalg.norm(xy,axis=1)<=r]+center[:2])
    return np.unique(np.round(np.concatenate(chunks)/spacing).astype(int),axis=0)*spacing
