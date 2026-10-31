"""Small usable adapters; learned models remain independently supplied plugins."""
from __future__ import annotations

import numpy as np

from elesim_pilot.vision.partial_cloud import ArmCameraModel
from .contracts import InstanceMask, Mesh, Pose


class ArmPose:
    api_version, stage, version = 1, "pose", "arm-fk-v1"

    def __init__(self, *, options, assets):
        self.model = ArmCameraModel(assets["arm"], assets["hand_eye"],
            base_world_transform=options.get("base_world_transform"))

    def estimate(self, measurement, task):
        if measurement.base_world_transform is None:
            transform = self.model.camera_pose(measurement.sensor.encoder_q)
        else:
            from elesim_pilot.vision.perception_bridge.hand_eye import camera_world_transform
            context = {**self.model.context, "base_world_T": measurement.base_world_transform}
            transform = camera_world_transform(context, measurement.sensor.encoder_q,
                self.model.mount, parent_frame=self.model.parent_frame)
        return Pose(task.frame_id, transform)


class DetectorSegmenter:
    """Use the existing detector only for an unambiguous single selected object.

    General tree/apple instance tracking is supplied by a segmenter plugin.
    This adapter refuses multiple same-label instances instead of mixing them.
    """
    api_version, stage, version = 1, "segmenter", "detector-single-instance-v3"

    def __init__(self, *, options, assets):
        from elesim_pilot.vision.perception.detector import create_detector, load_detector_config
        config = load_detector_config(assets["detector"])
        self.mask_erode_px = options.get('mask_erode_px',0)
        if type(self.mask_erode_px) is not int or not 0 <= self.mask_erode_px <= 8:
            raise ValueError('mask_erode_px must be an integer in 0..8')
        self.depth_component_gap_m = options.get('depth_component_gap_m', 0.)
        if (isinstance(self.depth_component_gap_m, bool)
                or not isinstance(self.depth_component_gap_m, (int,float))
                or not 0 <= self.depth_component_gap_m <= .2):
            raise ValueError('depth_component_gap_m must be a finite distance in 0..0.2 metres')
        self._depth_diagnostics = {}
        config.update({key:value for key,value in options.items()
                       if key not in {'mask_erode_px', 'depth_component_gap_m'}})
        self.single_component = str(config.get('type', '')).lower() == 'hsv'
        self.min_area = int(config.get('min_area_px', 20))
        if self.single_component:
            config['keep_largest_component'] = False
        self.label = str(config.get("target_label", ""))
        self.detector = create_detector(config)

    def segment(self, measurement, task):
        image = measurement.sensor.color_bgr
        many = getattr(self.detector, "list_detections", None)
        results = many(image) if callable(many) else [self.detector.detect(image)]
        results = [r for r in results if r is not None and (not self.label or r.label == self.label)]
        if len(results) > 1:
            raise ValueError("multiple target instances require an instance-tracking segmenter")
        if not results:
            return ()
        result = results[0]
        if getattr(result, "mask_kind", "unknown") != "instance":
            raise ValueError("detector returned bbox/unknown mask, not an instance segmentation")
        mask = np.asarray(result.mask) > 0
        if getattr(self, 'single_component', False):
            import cv2
            count, components, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
            usable = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= self.min_area]
            if len(usable) > 1:
                raise ValueError('multiple HSV target components require an instance-tracking segmenter')
            if not usable:
                return ()
            mask = components == usable[0]
        if self.mask_erode_px:
            import cv2
            size = 2*self.mask_erode_px+1
            mask = cv2.erode(mask.astype(np.uint8),np.ones((size,size),np.uint8)) > 0
            if int(mask.sum()) < self.min_area:
                return ()
        if self.depth_component_gap_m:
            mask = self._depth_supported(mask, measurement.sensor)
            if int(mask.sum()) < self.min_area:
                return ()
        return (InstanceMask(task.target_id, result.label, mask, float(result.confidence)),)

    def _depth_supported(self, mask, sensor):
        """Reject tiny depth islands while retaining disconnected inner surfaces.

        Colour closing/JPEG edges can include isolated pixels from a distant
        background. Require the same pixel support as the detector's min area,
        with four-neighbour edges bounded by a metric depth discontinuity.
        No object shape, pose or truth is used to choose a component.
        """
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        depth = sensor.depth_raw.astype(float)*sensor.depth_scale
        valid = mask & np.isfinite(depth) & (depth >= .1) & (depth <= 5.)
        count = int(valid.sum())
        labels = np.full(mask.shape, -1, dtype=np.int32)
        labels[valid] = np.arange(count, dtype=np.int32)
        kept = np.zeros(mask.shape, dtype=bool)
        components = 0
        if count:
            edges = []
            for a,b,da,db in ((labels[:,:-1],labels[:,1:],depth[:,:-1],depth[:,1:]),
                              (labels[:-1,:],labels[1:,:],depth[:-1,:],depth[1:,:])):
                adjacent = (a >= 0) & (b >= 0) & (np.abs(da-db) <= self.depth_component_gap_m)
                edges.append((a[adjacent], b[adjacent]))
            left = np.concatenate([a for a,b in edges])
            right = np.concatenate([b for a,b in edges])
            graph = coo_matrix((np.ones(len(left), dtype=np.uint8), (left,right)),
                               shape=(count,count)).tocsr()
            components, groups = connected_components(graph, directed=False)
            sizes = np.bincount(groups)
            kept[valid] = sizes[groups] >= self.min_area
        self._depth_diagnostics = dict(depth_gap_m=float(self.depth_component_gap_m),
            min_component_pixels=self.min_area, input_mask_pixels=int(mask.sum()),
            kept_pixels=int(kept.sum()), rejected_pixels=int(mask.sum()-kept.sum()),
            depth_components=int(components))
        return kept

    def diagnostics(self):
        return dict(self._depth_diagnostics)


class ObservedOnly:
    """Explicit completion bypass; never labels generated points as observed."""
    api_version, stage, version = 1, "completion", "observed-only-v1"

    def __init__(self, *, options, assets):
        if options or assets:
            raise ValueError("observed-only accepts no model parameters")

    def complete(self, cloud, task):
        return np.empty((0, 3))


class VoxelSurface:
    """Mesh occupied voxels; an approximation, not closed-shape recovery.

    The learned mesher slot can replace this reference without changing runners
    or Sim. Measured and model-generated points stay separate in the artifact.
    """
    api_version, stage, version = 1, "mesher", "supported-voxel-surface-v3"

    def __init__(self, *, options, assets):
        self.voxel_m = float(options.get("voxel_m", .02))
        if assets or not np.isfinite(self.voxel_m) or not .001 <= self.voxel_m <= .2:
            raise ValueError("voxel mesh resolution must be .001..0.2 metres")
        self.min_generated_neighbors = options.get('min_generated_neighbors', 0)
        if type(self.min_generated_neighbors) is not int or not 0 <= self.min_generated_neighbors <= 64:
            raise ValueError('min_generated_neighbors must be an integer in 0..64')
        self.generated_neighbor_radius_m = float(options.get('generated_neighbor_radius_m', self.voxel_m/2))
        if not np.isfinite(self.generated_neighbor_radius_m) or not 0 < self.generated_neighbor_radius_m <= .2:
            raise ValueError('generated_neighbor_radius_m must be a finite distance in (0,0.2] metres')
        self.max_occupied_voxels = options.get('max_occupied_voxels', 4000)
        # At most 12 triangles per voxel, within the 100000-triangle contract.
        if type(self.max_occupied_voxels) is not int or not 1 <= self.max_occupied_voxels <= 8000:
            raise ValueError('max_occupied_voxels must be an integer in 1..8000')
        self._diagnostics = {}

    def mesh(self, shape, task):
        observed, generated = shape.cloud.observed, shape.predicted
        retained = generated
        if self.min_generated_neighbors and len(generated):
            from scipy.spatial import cKDTree
            source = np.concatenate((observed, generated))
            distances, _ = cKDTree(source).query(generated,k=self.min_generated_neighbors+1)
            retained = generated[distances[:,-1] <= self.generated_neighbor_radius_m]
        self._diagnostics = dict(voxel_m=self.voxel_m,
            min_generated_neighbors=self.min_generated_neighbors,
            generated_neighbor_radius_m=self.generated_neighbor_radius_m,
            observed_points=len(observed),generated_points=len(generated),
            meshed_generated_points=len(retained),
            rejected_generated_points=len(generated)-len(retained))
        points = np.concatenate((observed, retained))
        cells = {tuple(v) for v in np.floor(points / self.voxel_m).astype(np.int64)}
        self._diagnostics.update(occupied_voxels=len(cells),max_occupied_voxels=self.max_occupied_voxels)
        if len(cells) > self.max_occupied_voxels:
            raise ValueError("voxel mesher budget exceeded; increase voxel_m or supply another mesher")
        # Outward oriented quads, triangulated with shared vertices.
        sides = (
            ((-1,0,0), ((0,0,0),(0,0,1),(0,1,1),(0,1,0))),
            ((1,0,0), ((1,0,0),(1,1,0),(1,1,1),(1,0,1))),
            ((0,-1,0), ((0,0,0),(1,0,0),(1,0,1),(0,0,1))),
            ((0,1,0), ((0,1,0),(0,1,1),(1,1,1),(1,1,0))),
            ((0,0,-1), ((0,0,0),(0,1,0),(1,1,0),(1,0,0))),
            ((0,0,1), ((0,0,1),(1,0,1),(1,1,1),(0,1,1))),
        )
        indices, vertices, faces = {}, [], []
        for cell in sorted(cells):
            for direction, corners in sides:
                if tuple(cell[i]+direction[i] for i in range(3)) in cells:
                    continue
                quad = []
                for corner in corners:
                    vertex = tuple(cell[i]+corner[i] for i in range(3))
                    if vertex not in indices:
                        indices[vertex] = len(vertices)
                        vertices.append(tuple(float(v)*self.voxel_m for v in vertex))
                    quad.append(indices[vertex])
                faces.extend(((quad[0],quad[1],quad[2]), (quad[0],quad[2],quad[3])))
        return Mesh(shape.cloud.object_id, task.frame_id, vertices, np.asarray(faces, dtype=np.int64))

    def diagnostics(self):
        return dict(self._diagnostics)
