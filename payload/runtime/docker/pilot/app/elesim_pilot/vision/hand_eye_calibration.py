"""Offline fixed camera-mount calibration from independent plane observations.

The fit assumes the supplied base-to-parent poses are trustworthy. It does
not simultaneously learn arm sag: mount error and pose-dependent deformation
must be evaluated separately, using poses not used by this optimizer.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .floor_calibration import Plane, camera_transform, plane_in_world


@dataclass(frozen=True)
class HandEyeFit:
    transform: np.ndarray
    reference: Plane
    normal_rms_deg: float
    distance_rms_m: float
    condition: float

    def payload(self):
        return {"transform": self.transform.tolist(), "reference": self.reference.payload(),
                "normal_rms_deg": self.normal_rms_deg, "distance_rms_m": self.distance_rms_m,
                "condition": self.condition}


def fit_hand_eye(parent_poses, observed_planes, initial_transform, *, reference=None):
    """Fit one fixed SE(3) mount, rejecting motions that leave it unobservable."""
    if not 6 <= len(parent_poses) <= 2048 or len(parent_poses) != len(observed_planes):
        raise ValueError("hand-eye calibration requires 6..2048 paired poses and planes")
    poses = [camera_transform(p) for p in parent_poses]
    initial = camera_transform(initial_transform)
    known_reference = reference is not None
    worlds = [plane_in_world(p, t @ initial) for p, t in zip(observed_planes, poses)]
    if reference is None:
        normals = np.array([p.normal for p in worlds])
        normals *= np.where(normals @ normals[0] < 0, -1, 1)[:, None]
        normal = np.mean(normals, axis=0)
        normal /= np.linalg.norm(normal)
        offsets = [p.offset_m * (1 if np.dot(p.normal, normal) >= 0 else -1) for p in worlds]
        reference = Plane(tuple(normal), float(np.median(offsets)))
    normal0 = np.asarray(reference.normal)
    axis = np.eye(3)[int(np.argmin(np.abs(normal0)))]
    tangent0 = np.cross(normal0, axis)
    tangent0 /= np.linalg.norm(tangent0)
    tangent1 = np.cross(normal0, tangent0)
    signs = [1 if np.dot(p.normal, normal0) >= 0 else -1 for p in worlds]
    parent_rotations = np.array([p[:3, :3] for p in poses])
    parent_translations = np.array([p[:3, 3] for p in poses])
    measured_normals = np.array([p.normal for p in observed_planes])*np.array(signs)[:, None]
    measured_offsets = np.array([p.offset_m for p in observed_planes])*np.array(signs)

    def unpack(x):
        delta = np.eye(4)
        delta[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
        delta[:3, 3] = x[3:6]
        mount = delta @ initial
        if known_reference:
            plane = reference
        else:
            normal = Rotation.from_rotvec(tangent0*x[6]+tangent1*x[7]).apply(normal0)
            plane = Plane(tuple(normal), reference.offset_m+x[8])
        return mount, plane

    def residuals(x, *, scaled=True):
        mount, plane = unpack(x)
        normals = np.einsum("nij,jk,nk->ni", parent_rotations, mount[:3, :3], measured_normals)
        origins = np.einsum("nij,j->ni", parent_rotations, mount[:3, 3])+parent_translations
        offsets = measured_offsets-np.einsum("ni,ni->n", normals, origins)
        normal_error = np.cross(normals, plane.normal)
        return np.column_stack((normal_error, offsets-plane.offset_m)).ravel()/(0.01 if scaled else 1)

    initial_x = np.zeros(6 if known_reference else 9)
    fit = least_squares(residuals, initial_x, loss="soft_l1", max_nfev=300,
                        xtol=1e-11, ftol=1e-11, gtol=1e-11)
    singular = np.linalg.svd(fit.jac, compute_uv=False)
    if (not fit.success or not np.all(np.isfinite(fit.x)) or singular[-1] < singular[0]*1e-6):
        raise ValueError("hand-eye parameters are not observable; collect more diverse motion (rank deficient)")
    mount, plane = unpack(fit.x)
    raw = residuals(fit.x, scaled=False).reshape(-1, 4)
    normals = np.einsum("nij,jk,nk->ni", parent_rotations, mount[:3, :3], measured_normals)
    angles = np.arccos(np.clip(normals @ plane.normal, -1, 1))
    normal_rms = float(np.degrees(np.sqrt(np.mean(angles**2))))
    return HandEyeFit(mount, plane, normal_rms,
                      float(np.sqrt(np.mean(raw[:, 3]**2))), float(singular[0]/singular[-1]))


def fit_recording(path, *, allow_unverified_clock=False):
    """Fit an exported recording, reserving every fifth observation for validation."""
    import json
    from pathlib import Path
    source = Path(path)
    if source.stat().st_size > 128*1024*1024:
        raise ValueError("floor recording exceeds 128 MiB")
    records = []
    with source.open(encoding="utf-8") as handle:
        for index, line in enumerate(handle):
            if len(line) > 65536 or index >= 2000:
                raise ValueError("floor recording exceeds its record bounds")
            raw = json.loads(line)
            if raw.get("result", {}).get("correction_mode") == "orientation":
                raise ValueError("orientation-only references do not provide a fixed metric plane for mount translation fitting")
            if raw.get("result", {}).get("valid"):
                records.append(raw)
    poses, planes, initial, reference_key, identity = [], [], None, None, None
    for row in records:
        if not row.get("synchronization_verified") and not allow_unverified_clock:
            raise ValueError("capture/pose timing is unverified; verify timing before fitting")
        mount = row.get("pose_evidence", {}).get("initial_mount")
        if mount is None or row.get("predicted_camera_pose") is None:
            continue
        mount = camera_transform(mount)
        ref = row["result"].get("reference") or {}
        key = (tuple(ref.get("normal", ())), ref.get("offset_m"), ref.get("kind"))
        source_identity = tuple(row.get("identity", ()))
        if initial is None:
            initial, reference_key, identity = mount, key, source_identity
        if not np.allclose(mount, initial) or key != reference_key or identity != source_identity:
            raise ValueError("recording mixes camera mounts, source epochs or reference planes")
        observed = row["result"].get("observed")
        if observed is None:
            continue
        poses.append(camera_transform(row["predicted_camera_pose"]) @ np.linalg.inv(initial))
        planes.append(Plane(tuple(observed["normal"]), observed["offset_m"]))
    if len(poses) < 10:
        raise ValueError("at least 10 valid source-pose observations are required")
    train = [i for i in range(len(poses)) if i % 5 != 0]
    validation = [i for i in range(len(poses)) if i % 5 == 0]
    reference = (Plane(reference_key[0], reference_key[1])
                 if reference_key[2] == "configured" else None)
    fit = fit_hand_eye([poses[i] for i in train], [planes[i] for i in train], initial,
                      reference=reference)

    def errors(mount):
        angles, distances = [], []
        for i in validation:
            measured = plane_in_world(planes[i], poses[i] @ mount)
            if np.dot(measured.normal, fit.reference.normal) < 0:
                measured = measured.flipped()
            angles.append(float(np.degrees(np.arccos(np.clip(np.dot(measured.normal, fit.reference.normal), -1, 1)))))
            distances.append(abs(measured.offset_m-fit.reference.offset_m))
        return {"normal_p95_deg": float(np.percentile(angles, 95)),
                "distance_p95_m": float(np.percentile(distances, 95))}

    return {**fit.payload(), "training_observations": len(train),
            "validation_observations": len(validation), "source_identity": list(identity),
            "timing_verified": all(bool(row.get("synchronization_verified")) for row in records),
            "validation_before": errors(initial), "validation_after": errors(fit.transform),
            "validation_scope": "held-out plane consistency, not full 6-DOF ground truth"}


def main(argv=None):
    import argparse
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description="Fit a fixed camera mount from exported floor observations")
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-unverified-clock", action="store_true")
    args = parser.parse_args(argv)
    result = fit_recording(args.recording, allow_unverified_clock=args.allow_unverified_clock)
    # This is a reviewable candidate, never an installed hand-eye file update.
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"candidate": str(args.output),
                      "validation_after": result["validation_after"]}, allow_nan=False))


if __name__ == "__main__":
    main()
