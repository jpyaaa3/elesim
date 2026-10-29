"""External model package stand-ins, imported through the public factory API."""
import json

import numpy as np

from elesim_pilot.reconstruction.contracts import InstanceMask, Plan, Pose


class Segmenter:
    api_version, stage, version = 1, "segmenter", "test-instances-v1"

    def __init__(self, *, options, assets):
        self.options = options

    def segment(self, measurement, task):
        # Two semantic classes and stable instance identities across views.
        image = measurement.sensor.color_bgr
        return tuple(InstanceMask(key, label, image[:, :, channel] > 0)
            for key, label, channel in (("apple-1", "apple", 1), ("tree-1", "tree", 2))
            if (image[:, :, channel] > 0).any())


class CameraPose:
    api_version, stage, version = 1, "pose", "test-pose-v1"

    def __init__(self, *, options, assets):
        pass

    def estimate(self, measurement, task):
        transform = np.eye(4)
        transform[0, 3] = .01 * (measurement.sensor.seq - 1)
        return Pose(task.frame_id, transform)


class ScanPose(CameraPose):
    def estimate(self, measurement, task):
        transform = np.eye(4)
        transform[0, 3] = measurement.sensor.encoder_q[0]
        return Pose(task.frame_id, transform)


class Correction:
    api_version, stage, version = 1, "pose_correction", "test-correction-v1"
    requires_imu = True

    def __init__(self, *, options, assets):
        self.offset = options.get("offset_m", 0.)
        self.scale = options.get("scale", 1.)

    def correct(self, measurement, pose, task):
        transform = pose.world_from_camera.copy()
        transform[0, 3] = transform[0, 3] * self.scale + self.offset
        return Pose(pose.frame_id, transform)


class Completion:
    api_version, stage, version = 1, "completion", "test-completion-v1"

    def __init__(self, *, options, assets):
        self.offset = json.loads(open(assets["weights"]).read())["offset_m"] if assets else .2

    def complete(self, cloud, task):
        return cloud.observed.mean(axis=0, keepdims=True) + [0., 0., self.offset]


class Planner:
    api_version, stage, version = 1, "planner", "test-plan-v1"

    def __init__(self, *, options, assets):
        self.options = options

    def plan(self, context):
        task = context.task
        start_q = context.start_q + [self.options.get("start_offset_m", 0.), 0., 0., 0.]
        return Plan(task.target_id, task.frame_id, (0., 1.),
            np.array([start_q, [self.options.get("linear_m", -.01), 0., 0., 0.]]),
            tuple(self.options.get("contact_ids", [task.target_id])))


class WrongApi(CameraPose):
    api_version = 2


class WrongStage(CameraPose):
    stage = "completion"


class Broken(CameraPose):
    def __init__(self, *, options, assets):
        raise ImportError("external model dependency missing")
