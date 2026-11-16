"""A bounded local flat-floor grid: unknown cells never become free by default."""
import heapq
import numpy as np
from .contracts import planar,angle


def floor_cell_points(sensor,world_from_camera,*,resolution,max_cells,floor_z=0.,cell_window=None,
                      origin_xy=(0.,0.)):
    """Keep an actual depth witness per floor cell before cloud thinning.

    Occluded and invalid pixels contribute nothing. Returned measurements still
    pass the ordinary robot-volume filter and occupied-cell precedence.
    """
    from elesim_pilot.vision.floor_calibration import camera_transform
    pose=camera_transform(world_from_camera)
    depth=np.asarray(sensor.depth_raw,dtype=float)*sensor.depth_scale
    intr=sensor.intrinsics
    x=(np.arange(depth.shape[1])-intr.cx)/intr.fx
    y=(np.arange(depth.shape[0])-intr.cy)/intr.fy
    height=depth*(pose[2,0]*x[None,:]+pose[2,1]*y[:,None]+pose[2,2])+pose[2,3]
    v,u=np.nonzero(np.isfinite(depth)&(depth>=.1)&(depth<=5)&(abs(height-floor_z)<.025))
    z=depth[v,u]
    points=np.column_stack([x[u]*z,y[v]*z,z])@pose[:3,:3].T+pose[:3,3]
    cells=np.floor((points[:,:2]-np.asarray(origin_xy))/resolution).astype(int)
    if cell_window is not None:
        low,high=cell_window
        inside=np.all((cells>=low)&(cells<high),axis=1)
        cells,points=cells[inside],points[inside]
    # np.unique(axis=0) sorts structured records while holding the GIL on
    # large depth frames. Stable numeric-key sorting preserves the first
    # actual pixel in each cell without starving the feedback receiver.
    order=np.lexsort((cells[:,1],cells[:,0]))
    ordered=cells[order]
    first=np.r_[True,np.any(ordered[1:]!=ordered[:-1],axis=1)] if len(order) else np.zeros(0,bool)
    keep=order[first]
    if len(keep)>max_cells:raise RuntimeError('local map budget exceeded')
    return points[keep]


class LocalMap:
    def __init__(self,resolution=.04,max_cells=40000,*,origin_xy=(0.,0.)):
        self.resolution=resolution;self.max_cells=max_cells
        self.origin=np.array(origin_xy,float,copy=True)
        if self.origin.shape!=(2,) or not np.isfinite(self.origin).all():raise ValueError('invalid map origin')
        self.origin.setflags(write=False)
        self.free=set();self.blocked=set();self.revision=0;self.window=None
    def cell(self,xy):return tuple(np.floor((np.asarray(xy)[:2]-self.origin)/self.resolution).astype(int))
    def cell_centers(self,cells):return (np.asarray(cells).reshape(-1,2)+.5)*self.resolution+self.origin
    def recenter(self,xy):
        """Keep a capacity-derived local window; discarded cells become unknown."""
        xy=np.asarray(xy,float)
        if xy.shape!=(2,) or not np.isfinite(xy).all():raise ValueError('invalid local map center')
        width=int(np.sqrt(self.max_cells));low=np.asarray(self.cell(xy))-width//2
        high=low+width
        if self.window is not None and np.array_equal(low,self.window[0]):return
        self.window=(low,high)
        self.free={c for c in self.free if self._inside(c)}
        self.blocked={c for c in self.blocked if self._inside(c)}
        self.revision+=1
    def _inside(self,cell):
        return self.window is None or all(lo<=v<hi for v,lo,hi in zip(cell,*self.window))
    def observe(self,points,floor_z=0.):
        p=np.asarray(points,float)
        if p.ndim!=2 or p.shape[1]!=3 or len(p)>100000 or not np.isfinite(p).all():raise ValueError('invalid map points')
        for row in p:
            c=self.cell(row)
            if not self._inside(c):continue
            if abs(row[2]-floor_z)<.025:self.free.add(c)
            elif floor_z+.04<row[2]<floor_z+1.7:self.blocked.add(c)
        self.free-=self.blocked
        if len(self.free|self.blocked)>self.max_cells:raise RuntimeError('local map budget exceeded')
        self.revision+=1
    def footprint_free(self,pose,footprint,*,margin_m=0.):
        cells=self._footprint_cells(pose,footprint,margin_m=margin_m)
        return all(c in self.free and c not in self.blocked for c in cells)
    def _footprint_cells(self,pose,footprint,*,margin_m=0.):
        t=planar(pose);local=np.asarray(footprint,float)
        points=local@t[:2,:2].T+t[:2,3]
        if not np.isfinite(margin_m) or margin_m<0:raise ValueError('invalid footprint margin')
        # Footprint is an interior-filled sampling, not just its boundary.
        low=np.floor((points-self.origin-margin_m)/self.resolution).astype(int)
        high=np.floor((points-self.origin+margin_m)/self.resolution).astype(int)
        if not len(low):raise ValueError('empty transport footprint')
        origin=np.min(low,axis=0);shape=np.max(high,axis=0)-origin+2
        if np.prod(shape)>4*self.max_cells:raise ValueError('transport footprint raster budget exceeded')
        # Exact union of the same inclusive cell rectangles, without sorting
        # hundreds of thousands of duplicate structured records at fine grids.
        delta=np.zeros(tuple(shape),dtype=np.int32)
        lower=low-origin;upper=high-origin+1
        for x,y,sign in ((lower[:,0],lower[:,1],1),(upper[:,0],lower[:,1],-1),
                         (lower[:,0],upper[:,1],-1),(upper[:,0],upper[:,1],1)):
            np.add.at(delta,(x,y),sign)
        cells=np.argwhere(delta.cumsum(axis=0).cumsum(axis=1)>0)+origin
        return [(int(x),int(y)) for x,y in cells]
    def footprint_status(self,pose,footprint,*,margin_m=0.):
        cells=self._footprint_cells(pose,footprint,margin_m=margin_m)
        blocked=[c for c in cells if c in self.blocked]
        unknown=[c for c in cells if c not in self.free and c not in self.blocked]
        return dict(total_cells=len(cells),unknown_cells=len(unknown),blocked_cells=len(blocked),
            unknown_xy_m=self.cell_centers(unknown[:16]).tolist(),
            blocked_xy_m=self.cell_centers(blocked[:16]).tolist())
    def segment_free(self,a,b,footprint,*,margin_m=0.):
        a,b=np.asarray(a,float),np.asarray(b,float);d=b-a;d[2]=angle(d[2])
        n=max(1,int(np.ceil(max(np.linalg.norm(d[:2])/(self.resolution/2),abs(d[2])/.05))))
        samples=np.linspace(0,1,n+1)
        if not self.free:
            return all(self.footprint_free(a+d*t,footprint,margin_m=margin_m) for t in samples)
        cells=np.array(tuple(self.free),dtype=np.int64)
        origin=cells.min(axis=0);shape=cells.max(axis=0)-origin+1
        # Generic sparse maps can span arbitrarily distant islands. Keep the
        # original sparse query rather than allocating their bounding box.
        if int(shape[0])*int(shape[1])>4*self.max_cells:
            return all(self.footprint_free(a+d*t,footprint,margin_m=margin_m) for t in samples)
        unsafe=np.ones(tuple(shape),dtype=np.int32);relative=cells-origin
        unsafe[relative[:,0],relative[:,1]]=0
        if self.blocked:
            blocked=np.array(tuple(self.blocked),dtype=np.int64)-origin
            blocked=blocked[np.all((blocked>=0)&(blocked<shape),axis=1)]
            unsafe[blocked[:,0],blocked[:,1]]=1
        summed=np.zeros(tuple(shape+1),dtype=np.int32)
        summed[1:,1:]=unsafe.cumsum(axis=0,dtype=np.int32).cumsum(axis=1,dtype=np.int32)
        local=np.asarray(footprint,float)
        if not np.isfinite(margin_m) or margin_m<0:raise ValueError('invalid footprint margin')
        for fraction in samples:
            t=planar(a+d*fraction);points=local@t[:2,:2].T+t[:2,3]
            low=np.floor((points-self.origin-margin_m)/self.resolution).astype(int)
            high=np.floor((points-self.origin+margin_m)/self.resolution).astype(int)
            if not len(low):raise ValueError('empty transport footprint')
            extent=high.max(axis=0)-low.min(axis=0)+2
            if np.prod(extent)>4*self.max_cells:raise ValueError('transport footprint raster budget exceeded')
            low-=origin;high-=origin
            if np.any(low<0) or np.any(high>=shape):return False
            # Every original sample contributes this exact inclusive cell
            # rectangle. Its union is free iff each rectangle has zero unsafe
            # cells; summed areas avoid rebuilding that union at every pose.
            upper=high+1
            count=(summed[upper[:,0],upper[:,1]]-summed[low[:,0],upper[:,1]]
                -summed[upper[:,0],low[:,1]]+summed[low[:,0],low[:,1]])
            if np.any(count):return False
        return True
    def path(self,start,goal,footprint,*,max_expansions=6000,margin_m=0.):
        start,goal=np.asarray(start,float),np.asarray(goal,float)
        if self.segment_free(start,goal,footprint,margin_m=margin_m):return np.array([start,goal])
        # Translation at start yaw followed by an explicitly checked final turn.
        # Holonomic GO2 can translate sideways; each segment is still swept.
        if not self.footprint_free(start,footprint,margin_m=margin_m):raise ValueError('current footprint not observed free')
        first,last=self.cell(start),self.cell(goal)
        queue=[(0.,first)];cost={first:0.};parent={};expanded=0
        while queue and expanded<max_expansions:
            _,c=heapq.heappop(queue);expanded+=1
            if c==last:break
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt=(c[0]+dx,c[1]+dy)
                p=np.r_[self.cell_centers([nxt])[0],start[2]]
                before=start if c==first else np.r_[self.cell_centers([c])[0],start[2]]
                if not self.segment_free(before,p,footprint,margin_m=margin_m):continue
                new=cost[c]+1
                if new>=cost.get(nxt,float('inf')):continue
                cost[nxt]=new;parent[nxt]=c
                heapq.heappush(queue,(new+np.linalg.norm(np.array(nxt)-last),nxt))
        if last not in cost:raise ValueError('no observed-free local base path')
        cells=[last]
        while cells[-1]!=first:cells.append(parent[cells[-1]])
        path=[start]+[np.r_[self.cell_centers([c])[0],start[2]] for c in cells[::-1][1:]]+[goal]
        if not all(self.segment_free(a,b,footprint,margin_m=margin_m) for a,b in zip(path,path[1:])):
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
