"""Per-observation horizontal-plane tilt correction for reconstruction."""
from __future__ import annotations

from elesim_pilot.vision.floor_calibration import FloorEstimator, FloorSettings, depth_points, detect_planes
from .contracts import Pose
from .pipeline import ObservationPending


class FloorTilt:
    api_version, stage, version = 1, 'pose_correction', 'depth-plane-tilt-v1'

    def __init__(self, *, options, assets):
        if assets or set(options)-{'reference_normal', 'settings'} or 'reference_normal' not in options:
            raise ValueError('floor correction requires an explicit reference_normal policy')
        self.settings = FloorSettings(**options.get('settings', {}))
        self.estimator = FloorEstimator(self.settings, reference_normal=options['reference_normal'])
        self._identity, self._sequence, self._stamp = None, -1, 0.
        self._status = dict(applied=False, state='waiting_for_depth')

    def correct(self, measurement, pose, task):
        sensor = measurement.sensor
        intr = sensor.intrinsics
        identity = (sensor.source_id, sensor.source_boot_id, sensor.calibration_id,
                    sensor.depth_scale, intr.fx, intr.fy, intr.cx, intr.cy, intr.width, intr.height)
        if identity == self._identity and (sensor.seq <= self._sequence or sensor.ts <= self._stamp):
            self._status = dict(applied=False, state='rejected', reason='replayed observation')
            raise ValueError('floor correction rejected replayed observation')
        self._identity, self._sequence, self._stamp = identity, sensor.seq, sensor.ts
        points = depth_points(sensor.depth_raw, intr, sensor.depth_scale, max_points=self.settings.max_points)
        planes = detect_planes(points, self.settings)
        # LiveScan checks freshness; recorded replay uses the capture clock.
        status = self.estimator.update(planes, pose.world_from_camera,
            stamp=sensor.ts, identity=identity, now=sensor.ts)
        corrected = self.estimator.corrected_pose(now=sensor.ts)
        self._status = dict(status, applied=corrected is not None, sequence=sensor.seq,
                            source=list(identity[:3]))
        if corrected is None:
            raise ObservationPending('floor correction: '+status['reason'])
        return Pose(pose.frame_id, corrected)

    def diagnostics(self):
        return dict(self._status)
