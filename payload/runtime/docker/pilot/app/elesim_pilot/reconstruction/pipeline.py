"""Execute replaceable models without acquiring or sending motion commands."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import time

import numpy as np

from elesim_protocol import SimMappingConfig
from elesim_pilot.vision.partial_cloud import VoxelCloud, reconstruct_view
from .contracts import CompletedObject, InstanceMask, Measurement, ObjectCloud, Plan, PlanningContext, Pose, SceneObject, Task
from .models import load_models


class PipelineCancelled(RuntimeError):
    pass


@dataclass(frozen=True)
class Result:
    task: Task
    objects: tuple[SceneObject, ...]
    plan: Plan | None
    provenance: dict


def call_model(model, method, *args, cancelled=lambda: False, progress=lambda event: None):
    if cancelled():
        raise PipelineCancelled("reconstruction cancelled")
    if getattr(model.implementation, "requires_imu", False):
        if not args or not isinstance(args[0], Measurement) or args[0].imu is None:
            raise ValueError(f"{model.stage} requires a calibrated, fresh IMU sample")
    started = time.monotonic()
    try:
        value = getattr(model.implementation, method)(*args)
    except Exception as exc:
        raise RuntimeError(f"{model.stage}/{model.provenance['factory']}: {exc}") from exc
    if cancelled():
        raise PipelineCancelled("reconstruction cancelled")
    progress({"stage": model.stage, "elapsed_s": time.monotonic()-started})
    return value


class Pipeline:
    def __init__(self, models, corrections=(), *, mapping=None, voxel_m=.004):
        if not {"segmenter", "pose", "completion", "mesher"} <= set(models):
            raise ValueError("missing reconstruction models")
        self.models, self.corrections = dict(models), tuple(corrections)
        self.mapping = mapping or SimMappingConfig()
        self.voxel_m = voxel_m

    @classmethod
    def from_config(cls, path, **kwargs):
        models, corrections = load_models(path)
        return cls(models, corrections, **kwargs)

    def observe(self, measurement, task, **callbacks):
        measurement = measurement.validated()
        pose = call_model(self.models["pose"], "estimate", measurement, task, **callbacks)
        for correction in self.corrections:
            if not isinstance(pose, Pose) or pose.frame_id != task.frame_id:
                raise ValueError("pose model changed scene frame")
            pose = call_model(correction, "correct", measurement, pose, task, **callbacks)
        if not isinstance(pose, Pose) or pose.frame_id != task.frame_id:
            raise ValueError("pose model changed scene frame")
        instances = call_model(self.models["segmenter"], "segment", measurement, task, **callbacks)
        if not isinstance(instances, (tuple, list)) or len(instances) > 16:
            raise ValueError("segmenter must return at most 16 instances")
        seen, occupied = set(), np.zeros(measurement.sensor.depth_raw.shape, dtype=bool)
        for instance in instances:
            if not isinstance(instance, InstanceMask) or instance.mask.shape != occupied.shape:
                raise ValueError("invalid instance-mask contract")
            if instance.object_id in seen or np.any(occupied & instance.mask):
                raise ValueError("duplicate identities or overlapping instance masks")
            seen.add(instance.object_id)
            occupied |= instance.mask
        return measurement, pose, tuple(instances)

    def finish(self, clouds, task, sensor, *, views, extra_provenance=None, **callbacks):
        clouds = tuple(clouds)
        ids = {cloud.object_id for cloud in clouds}
        if len(ids) != len(clouds) or not 1 <= len(ids) <= 16:
            raise ValueError("invalid scene object identities/budget")
        if task.target_id not in ids or any(key not in ids for key in task.roles):
            raise ValueError("selected target or explicitly declared scene object was not observed")
        objects = []
        for cloud in sorted(clouds, key=lambda value: value.object_id):
            if cloud.frame_id != task.frame_id or cloud.role != task.role(cloud.object_id):
                raise ValueError("scene cloud frame/role changed")
            predicted = call_model(self.models["completion"], "complete", cloud, task, **callbacks)
            shape = CompletedObject(cloud, predicted)
            mesh = call_model(self.models["mesher"], "mesh", shape, task, **callbacks)
            objects.append(SceneObject(shape, mesh))
        objects = tuple(objects)
        source = (sensor.source_id, sensor.source_boot_id, sensor.calibration_id)
        plan = None
        if "planner" in self.models:
            context = PlanningContext(task, objects, sensor.encoder_q, self.mapping,
                source, sensor.seq, sensor.ts)
            plan = call_model(self.models["planner"], "plan", context, **callbacks)
            if not isinstance(plan, Plan):
                raise ValueError("planner must return a Plan proposal")
            plan = plan.validate(context)
        provenance = {"model_api_version": 1, "source": list(source), "last_seq": sensor.seq,
            "last_timestamp_s": sensor.ts, "views": views, "operation": task.operation,
            "target_id": task.target_id, "mapping": asdict(self.mapping),
            "models": {key: value.provenance for key, value in self.models.items()},
            "pose_corrections": [value.provenance for value in self.corrections],
            **(extra_provenance or {})}
        return Result(task, objects, plan, provenance)

    def run(self, measurements, task: Task, *, cancelled=lambda: False, progress=lambda event: None):
        clouds, labels = {}, {}
        identity, previous_seq, previous_time, count = None, -1, 0., 0
        imu_calibration = None

        for measurement in measurements:
            if cancelled():
                raise PipelineCancelled("reconstruction cancelled")
            count += 1
            if count > 32:
                raise ValueError("reconstruction view budget exceeded (32)")
            if not isinstance(measurement, Measurement):
                raise TypeError("pipeline input must be Measurement")
            measurement = measurement.validated()
            if measurement.imu is not None:
                calibration = measurement.imu.calibration_id
                if imu_calibration is not None and imu_calibration != calibration:
                    raise ValueError("IMU calibration changed during reconstruction")
                imu_calibration = calibration
            sensor = measurement.sensor
            current = (sensor.source_id, sensor.source_boot_id, sensor.calibration_id)
            if identity is not None and identity != current:
                raise ValueError("source boot/calibration changed during reconstruction")
            if sensor.seq <= previous_seq or sensor.ts < previous_time:
                raise ValueError("replayed or reordered observation")
            identity, previous_seq, previous_time = current, sensor.seq, sensor.ts
            measurement, pose, instances = self.observe(measurement, task,
                cancelled=cancelled, progress=progress)
            for instance in instances:
                if labels.get(instance.object_id, instance.label) != instance.label:
                    raise ValueError("object identity changed label across views")
                labels[instance.object_id] = instance.label
                if len(labels) > 16:
                    raise ValueError("scene object budget exceeded")
                view = reconstruct_view(sensor, instance.mask, pose.world_from_camera)
                if not len(view.points_before):
                    continue
                cloud = clouds.setdefault(instance.object_id, VoxelCloud(voxel_m=self.voxel_m))
                cloud.add(view.points_before)
        if task.target_id not in clouds or any(key not in clouds for key in task.roles):
            raise ValueError("selected target or explicitly declared scene object was not observed")
        return self.finish([ObjectCloud(key, labels[key], task.role(key), task.frame_id, value.points())
            for key, value in clouds.items()], task, sensor, views=count,
            extra_provenance={"imu_calibration_id": imu_calibration}, cancelled=cancelled, progress=progress)
