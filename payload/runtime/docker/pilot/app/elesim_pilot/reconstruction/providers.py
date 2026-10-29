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
    api_version, stage, version = 1, "segmenter", "detector-single-instance-v1"

    def __init__(self, *, options, assets):
        from elesim_pilot.vision.perception.detector import create_detector, load_detector_config
        config = load_detector_config(assets["detector"])
        config.update(options)
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
        return (InstanceMask(task.target_id, result.label, mask, float(result.confidence)),)


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
    api_version, stage, version = 1, "mesher", "observed-voxel-surface-v1"

    def __init__(self, *, options, assets):
        self.voxel_m = float(options.get("voxel_m", .02))
        if assets or not np.isfinite(self.voxel_m) or not .001 <= self.voxel_m <= .2:
            raise ValueError("voxel mesh resolution must be .001..0.2 metres")

    def mesh(self, shape, task):
        points = np.concatenate((shape.cloud.observed, shape.predicted))
        cells = {tuple(v) for v in np.floor(points / self.voxel_m).astype(np.int64)}
        if len(cells) > 4000:
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
