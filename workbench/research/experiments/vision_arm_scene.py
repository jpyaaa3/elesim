"""Independent arm-URDF RGB-D producer for a pose x error test matrix.

Only sensors/ is public. This process never imports Pilot. Static pose samples
exercise actual arm geometry, rather than a Cartesian camera rig; they do not
establish actuator dynamics or physical sensor accuracy.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.stats import qmc


class ArmUrdf:
    """Reference FK using URDF origins/axes, independent of Pilot's FK model."""

    def __init__(self, urdf_path, hand_eye_path, *, base_height_m=.8):
        self.path = Path(urdf_path)
        self.joints = ET.parse(self.path).getroot().findall("joint")
        raw = json.loads(Path(hand_eye_path).read_text())
        raw = raw.get("optical_frame", raw)
        self.parent = raw["parent_frame"]
        self.mount = np.eye(4)
        self.mount[:3, :3] = Rotation.from_quat(raw["quaternion_xyzw"]).as_matrix()
        self.mount[:3, 3] = raw["translation_m"]
        self.base = np.eye(4)
        self.base[2, 3] = base_height_m
        self.bends = [j.get("name") for j in self.joints if j.get("type") == "revolute"
                      and j.get("name") != "j_housing_wedge"]
        if len(self.bends) != 10:
            raise ValueError("expected the installed ten-node arm")

    def joint_values(self, q):
        q = np.asarray(q, dtype=float).reshape(4)
        result = {"j_plate_housing": float(q[0]), "j_housing_wedge": float(q[1])}
        result.update({name: float(q[2 if index < 5 else 3]) for index, name in enumerate(self.bends)})
        return result

    def link_poses(self, q):
        values = self.joint_values(q)
        poses = {"plate": self.base.copy()}
        for joint in self.joints:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            local = np.eye(4)
            if origin is not None:
                local[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                local[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            value = values.get(joint.get("name"), 0.0)
            axis_node = joint.find("axis")
            axis = np.fromstring(axis_node.get("xyz", "1 0 0"), sep=" ") if axis_node is not None else np.array([1., 0, 0])
            if joint.get("type") == "revolute":
                local[:3, :3] = local[:3, :3] @ Rotation.from_rotvec(axis*value).as_matrix()
            elif joint.get("type") == "prismatic":
                local[:3, 3] += local[:3, :3] @ axis*value
            poses[child] = poses[parent] @ local
        return poses

    def camera_pose(self, q):
        return self.link_poses(q)[self.parent] @ self.mount


def sample_cases(lower, upper, *, poses=24, error_seeds=2, seed=217):
    if not 4 <= poses <= 128 or not 1 <= error_seeds <= 4:
        raise ValueError("arm matrix budget requires 4..128 poses and 1..4 error seeds")
    lower, upper = np.asarray(lower), np.asarray(upper)
    values = qmc.scale(qmc.LatinHypercube(d=4, seed=seed).random(poses), lower, upper)
    rng = np.random.default_rng(seed+1)
    result = []
    for condition in ("clean", "tilt_height", "mount", "noisy", "wrong_plane",
                      "near_table", "ambiguous_plane", "missing_depth"):
        for error_seed in range(error_seeds):
            rotation = rng.uniform(-np.radians(6), np.radians(6), 3)
            translation = rng.uniform(-.04, .04, 3)
            if condition in ("tilt_height", "noisy"):
                rotation[2], translation[:2] = 0, 0
            elif condition != "mount":
                rotation[:], translation[:] = 0, 0
            noise = float(rng.uniform(.001, .003)) if condition == "noisy" else .0005
            dropout = float(rng.uniform(.1, .35)) if condition == "noisy" else 0.0
            table_height = (float(rng.uniform(.35, .5)) if condition == "wrong_plane" else
                            float(rng.uniform(.08, .14)))
            boot = uuid.uuid4().hex
            for pose_id, q in enumerate(values):
                result.append({"case_id": len(result), "pose_id": pose_id, "q": q.tolist(),
                    "condition": condition, "error_seed": error_seed, "boot": boot,
                    "rotation_vector": rotation.tolist(), "translation": translation.tolist(),
                    "noise_sigma_m": noise, "dropout": dropout, "table_height_m": table_height})
    return result


def case_pose(nominal, case):
    actual = nominal.copy()
    rotation = Rotation.from_rotvec(case["rotation_vector"]).as_matrix()
    if case["condition"] == "mount":
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = rotation, case["translation"]
        return actual @ delta
    actual[:3, :3] = rotation @ actual[:3, :3]
    actual[:3, 3] += case["translation"]
    return actual


def scene_boxes(seed=889):
    rng = np.random.default_rng(seed)
    locations = [(.65, -.4), (.65, 0), (.65, .4), (1.05, -.65), (1.05, 0),
                 (1.05, .65), (-.45, -.4), (-.45, .4), (0, -.8), (0, .8)]
    boxes = []
    for x, y in locations:
        size = rng.uniform([.10, .10, .10], [.18, .18, .18])
        center = np.array([x, y, size[2]/2])
        angle = float(rng.uniform(-.5, .5))
        boxes.append({"center": center, "size": size,
                      "rotation": Rotation.from_euler("z", angle).as_matrix(), "yaw": angle})
    return boxes


def analytic_frame(pose, intrinsics, boxes, case):
    """CPU geometry fixture; Genesis additionally renders the actual arm mesh."""
    fx, fy, cx, cy, width, height = intrinsics
    v, u = np.mgrid[:int(height), :int(width)]
    rays = np.stack(((u-cx)/fx, (v-cy)/fy, np.ones_like(u)), axis=-1) @ pose[:3, :3].T
    origin = pose[:3, 3]
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = -origin[2]/rays[..., 2]
    distance = np.where(distance > .05, distance, 100.)
    color = np.full((*distance.shape, 3), 155, dtype=np.uint8)
    geometries = [(b["center"], b["size"], b["rotation"], [12, 24, 230]) for b in boxes]
    if case["condition"] in ("wrong_plane", "near_table", "ambiguous_plane"):
        side = .9 if case["condition"] == "ambiguous_plane" else 20.
        geometries.append((np.array([.6, 0, case["table_height_m"]-.02]),
                           np.array([side, side, .04]), np.eye(3), [155, 155, 155]))
    for center, size, rotation, bgr in geometries:
        local_origin, local_rays = (origin-center) @ rotation, rays @ rotation
        with np.errstate(divide="ignore", invalid="ignore"):
            first, second = (-size/2-local_origin)/local_rays, (size/2-local_origin)/local_rays
        near = np.max(np.minimum(first, second), axis=-1)
        far = np.min(np.maximum(first, second), axis=-1)
        hit = (far >= near) & (near > .05) & (near < distance)
        distance[hit], color[hit] = near[hit], bgr
    return color, distance.astype(np.float32)


def produce_matrix(output, *, data_root, config_path, backend="analytic", poses=24,
                   error_seeds=2, seed=217, width=320, height=240, base_height=.8):
    import hashlib
    import yaml

    output, data_root = Path(output), Path(data_root)
    sensors, truth = output/"sensors", output/"truth"
    sensors.mkdir(parents=True, exist_ok=False)
    truth.mkdir()
    arm = ArmUrdf(data_root/"models/assemblies/zed-mini/arm.urdf",
                  data_root/"calibration/cameras/zed_mini.hand_eye.json", base_height_m=base_height)
    config = yaml.safe_load(Path(config_path).read_text())["robot"]["arm"]["model"]
    linear = next(j for j in arm.joints if j.get("name") == "j_plate_housing").find("limit")
    bend = np.radians(config["bend_deg"])
    lower = [float(linear.get("lower")), np.radians(config["roll_min_deg"]), -bend, -bend]
    upper = [float(linear.get("upper")), np.radians(config["roll_max_deg"]), bend, bend]
    cases = sample_cases(lower, upper, poses=poses, error_seeds=error_seeds, seed=seed)
    boxes = scene_boxes(seed+672)
    focal = height/2/np.tan(np.radians(30))
    intrinsics = np.array([focal, focal, (width-1)/2, (height-1)/2, width, height])
    scene = entity = camera = table = None
    fk_error = 0.
    if backend == "genesis":
        import genesis as gs
        from elesim_sim.vision.render_device import check_camera_renderer
        from elesim_sim.vision.sim_camera.mount import intrinsics_from_fov

        gs.init(backend=gs.gpu, logging_level="warning")
        scene = gs.Scene(show_viewer=False,
            sim_options=gs.options.SimOptions(dt=.01, substeps=1, gravity=(0, 0, 0)),
            rigid_options=gs.options.RigidOptions(enable_collision=False, enable_self_collision=False,
                enable_joint_limit=False, disable_constraint=True),
            vis_options=gs.options.VisOptions(ambient_light=(.7, .7, .7)))
        scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Default(color=(.6, .6, .6)))
        for box in boxes:
            scene.add_entity(gs.morphs.Box(size=tuple(box["size"]), pos=tuple(box["center"]),
                euler=(0, 0, float(np.degrees(box["yaw"]))), fixed=True),
                surface=gs.surfaces.Default(color=(.9, .08, .04)))
        # Separate finite/infinite tabletop geometries are moved only by the
        # private producer. They occlude the true floor in negative controls.
        tables = {side: scene.add_entity(gs.morphs.Box(size=(side, side, .04),
                    pos=(.6, 0, 20), fixed=True), surface=gs.surfaces.Default(color=(.6, .6, .6)))
                  for side in (.9, 20.)}
        entity = scene.add_entity(gs.morphs.URDF(file=str(arm.path), fixed=True,
                    pos=(0, 0, base_height), merge_fixed_links=False))
        camera = scene.add_camera(res=(width, height), fov=60, GUI=False, debug=False)
        scene.build()
        check_camera_renderer(scene.visualizer.rasterizer._renderer, use_gpu=True, label="arm-matrix")
        configured = intrinsics_from_fov(width=width, height=height, fov_deg=60)
        intrinsics[:4] = [configured.fx, configured.fy, configured.cx, configured.cy]
        k = np.asarray(camera.intrinsics)
        np.testing.assert_allclose(intrinsics[:4], [k[0, 0], k[1, 1], k[0, 2]-.5, k[1, 2]-.5], rtol=1e-6)
        names = list(arm.joint_values([0, 0, 0, 0]))
        indices = [int(entity.get_joint(name).dofs_idx_local[0])
                   if hasattr(entity.get_joint(name).dofs_idx_local, "__len__")
                   else int(entity.get_joint(name).dofs_idx_local) for name in names]
    frames = []
    try:
        for case in cases:
            nominal = arm.camera_pose(case["q"])
            actual = case_pose(nominal, case)
            if camera is None:
                color, ideal = analytic_frame(actual, intrinsics, boxes, case)
            else:
                import torch
                from elesim_sim.vision.sim_camera.mount import _render_camera

                values = arm.joint_values(case["q"])
                entity.set_dofs_position(torch.as_tensor([values[name] for name in names], device=gs.device,
                                                        dtype=gs.tc_float), dofs_idx_local=indices)
                link = entity.get_link(arm.parent)
                raw_pos, raw_quat = link.get_pos(), link.get_quat()
                pos = raw_pos.detach().cpu().numpy().reshape(3)
                quat = raw_quat.detach().cpu().numpy().reshape(4)
                live = np.eye(4)
                live[:3, :3] = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_matrix()
                live[:3, 3] = pos
                expected = arm.link_poses(case["q"])[arm.parent]
                error = float(np.max(np.abs(live-expected)))
                fk_error = max(fk_error, error)
                if error > 2e-5:
                    raise RuntimeError(f"Genesis arm FK differs from reference URDF: {error}")
                actual = case_pose(live @ arm.mount, case)
                for side, table in tables.items():
                    visible_side = .9 if case["condition"] == "ambiguous_plane" else 20.
                    enabled = case["condition"] in ("wrong_plane", "near_table", "ambiguous_plane") and side == visible_side
                    table.set_pos((.6, 0, case["table_height_m"]-.02 if enabled else 20.))
                camera.set_pose(transform=actual @ np.diag([1, -1, -1, 1]))
                rgb, depth, _, _ = _render_camera(camera, rgb=True, depth=True, force_render=True)
                if hasattr(rgb, "cpu"): rgb = rgb.cpu().numpy()
                if hasattr(depth, "cpu"): depth = depth.cpu().numpy()
                color = np.ascontiguousarray(np.asarray(rgb)[..., :3][..., ::-1], dtype=np.uint8)
                ideal = np.asarray(depth, dtype=np.float32)
            rng = np.random.default_rng(seed+100000+case["case_id"])
            truth_name = f"case-{case['case_id']:04d}.npz"
            v, u = np.mgrid[:height, :width]
            rays = np.stack(((u-intrinsics[2])/intrinsics[0], (v-intrinsics[3])/intrinsics[1],
                             np.ones_like(u)), axis=-1)
            world_z = (rays @ actual[:3, :3].T)[..., 2]*ideal + actual[2, 3]
            floor_visible = int(np.count_nonzero((np.abs(world_z) < .001) & (ideal >= .1) & (ideal <= 5)))
            np.savez_compressed(truth/truth_name, ideal_depth=ideal, true_camera_pose=actual,
                nominal_camera_pose=nominal, true_intrinsics=intrinsics,
                object_centers=np.array([b["center"] for b in boxes]),
                object_sizes=np.array([b["size"] for b in boxes]),
                object_rotations=np.array([b["rotation"] for b in boxes]))
            for repetition in range(3):
                observed = ideal + rng.normal(0, case["noise_sigma_m"], ideal.shape).astype(np.float32)
                observed[(ideal <= .05) | (ideal >= 99) | ~np.isfinite(ideal)] = 0
                if case["dropout"]:
                    observed[rng.random(ideal.shape) < case["dropout"]] = 0
                if case["condition"] == "missing_depth": observed[:] = 0
                name = f"view-{case['case_id']:04d}-{repetition}.npz"
                stamp = 10000 + len(frames)*.1
                np.savez_compressed(sensors/name, schema_version=np.array(1), color_bgr=color,
                    depth_raw=observed, depth_scale=np.array(1.), intrinsics=intrinsics,
                    seq=np.array(len(frames)+1), ts=np.array(stamp), source_id=np.array("arm-camera"),
                    source_boot_id=np.array(case["boot"]), calibration_id=np.array("arm4-zed-v1"),
                    encoder_q=np.array(case["q"]))
                frames.append({"name": name, "case_id": case["case_id"], "repetition": repetition})
            case.update(truth_file=truth_name, floor_visible_pixels=floor_visible)
            if case["case_id"] % 32 == 0:
                print(json.dumps({"cases_completed": case["case_id"]+1, "cases_total": len(cases)}), flush=True)
    finally:
        if scene is not None: scene.destroy()
    manifest = {"schema_version": 2, "backend": backend, "cases": cases, "frames": frames,
        "joint_lower": list(map(float, lower)), "joint_upper": list(map(float, upper)),
        "base_height_m": base_height, "urdf_sha256": hashlib.sha256(arm.path.read_bytes()).hexdigest(),
        "genesis_reference_fk_max_abs": fk_error,
        "scope": "static actual arm geometry at a known fixed bench base; no actuator dynamics or exposure timing",
        "sampling": "Latin hypercube joint poses crossed with independent error seeds; no visibility filtering"}
    (truth/"manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    return {"cases": len(cases), "frames": len(frames), "backend": backend}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--data-root", type=Path, default=Path("/opt/elesim/data"))
    parser.add_argument("--config", type=Path, default=Path("/opt/elesim/config/config.yaml"))
    parser.add_argument("--backend", choices=("analytic", "genesis"), default="analytic")
    parser.add_argument("--poses", type=int, default=24)
    parser.add_argument("--error-seeds", type=int, default=2)
    parser.add_argument("--seed", type=int, default=217)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    args = parser.parse_args()
    print(json.dumps(produce_matrix(args.output, data_root=args.data_root, config_path=args.config,
        backend=args.backend, poses=args.poses, error_seeds=args.error_seeds, seed=args.seed,
        width=args.width, height=args.height)))


if __name__ == "__main__": main()
