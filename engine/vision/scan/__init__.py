"""
Roll-sweep geometry scan: FK-pose multi-view fusion.

Ported from the hand-held ZED VIO fusion bench, with two substitutions:

1. Pose source is FORWARD KINEMATICS (node9 link pose from the arm joint state,
   composed with the hand-eye extrinsics), not the ZED's visual-inertial
   odometry. VIO needs the camera to translate through the scene to stay
   observable; a wrist-mounted camera swept by a single roll joint barely
   translates, so VIO degenerates exactly where this scan operates.

2. The viewpoint sweep is a commanded roll traverse (joint limit to joint
   limit, repeated), so angular coverage is a controlled quantity instead of
   whatever a hand happened to trace.

Everything else -- transform-and-merge fusion with no ICP, per-frame plane
removal, world-frame surface classification with per-point provenance -- is
kept, because the point of the exercise is that registration quality comes
from the pose source alone.
"""

from __future__ import annotations

from .fk_pose import FkPoseProvider, FkPoseSample
from .fusion import (
    FusionResult,
    box_crop,
    classify_surface_world,
    clean_world_frame,
    fuse,
    project_points,
    transform_points,
    voxel_downsample,
)
from .plan import RollScanConfig, RollSweepPlan, build_plan
from .roll_sweep import RollSweepScan, ScanProgress

__all__ = [
    "FkPoseProvider",
    "FkPoseSample",
    "FusionResult",
    "RollScanConfig",
    "RollSweepPlan",
    "RollSweepScan",
    "ScanProgress",
    "box_crop",
    "build_plan",
    "classify_surface_world",
    "clean_world_frame",
    "fuse",
    "project_points",
    "transform_points",
    "voxel_downsample",
]
