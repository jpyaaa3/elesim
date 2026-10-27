"""Regression fixture extracted from a retired experiment; no standalone CLI."""


from __future__ import annotations


import json


from pathlib import Path


import uuid


import numpy as np


from scipy.spatial.transform import Rotation


def rig_pose(q):
    pose = np.eye(4)
    pose[:3, 3] = q[:3]
    pose[:3, :3] = Rotation.from_euler("xyz", q[3:]).as_matrix() @ np.diag([1, -1, -1])
    return pose


def analytic_image(pose, intrinsic, center, size, rotation, width, height):
    v, u = np.mgrid[:height, :width]
    rays = np.stack(((u-intrinsic[0])/intrinsic[1], (v-intrinsic[2])/intrinsic[1],
                     np.ones_like(u)), axis=-1) @ pose[:3, :3].T
    origin = pose[:3, 3]
    floor_z = -origin[2]/rays[..., 2]
    local_origin = (origin-center) @ rotation
    local_rays = rays @ rotation
    with np.errstate(divide="ignore", invalid="ignore"):
        first = (-size/2-local_origin)/local_rays
        second = (size/2-local_origin)/local_rays
    near = np.max(np.minimum(first, second), axis=-1)
    far = np.min(np.maximum(first, second), axis=-1)
    hit = (far >= near) & (near > 0) & (near < floor_z)
    depth = np.where(hit, near, floor_z).astype(np.float32)
    color = np.full((height, width, 3), 155, dtype=np.uint8)
    color[hit] = [12, 24, 230]
    return color, depth


def produce(output, *, backend="analytic", seeds=3, width=640, height=480):
    output = Path(output)
    sensors, truth = output/"sensors", output/"truth"
    sensors.mkdir(parents=True, exist_ok=False)
    truth.mkdir(exist_ok=False)
    renderer = None
    if backend == "genesis":
        import genesis as gs
        gs.init(backend=gs.gpu, logging_level="warning")
        renderer = gs
    fov = 60.0
    focal = height/2/np.tan(np.deg2rad(fov/2))
    intrinsic = np.array([focal, focal, (width-1)/2, (height-1)/2, width, height])
    true_intrinsic = intrinsic.copy()
    frames, index = [], 0
    for seed in range(seeds):
        rng = np.random.default_rng(815+seed)
        size = rng.uniform([0.10, 0.08, 0.08], [0.17, 0.14, 0.14])
        center = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06), size[2]/2])
        object_yaw = float(rng.uniform(-0.4, 0.4))
        object_rotation = Rotation.from_euler("z", object_yaw).as_matrix()
        scene = camera = None
        if renderer is not None:
            gs = renderer
            scene = gs.Scene(show_viewer=False,
                sim_options=gs.options.SimOptions(dt=0.01, substeps=1),
                vis_options=gs.options.VisOptions(ambient_light=(0.7, 0.7, 0.7)))
            scene.add_entity(gs.morphs.Plane(), surface=gs.surfaces.Default(color=(0.6,0.6,0.6)))
            scene.add_entity(gs.morphs.Box(size=tuple(size), pos=tuple(center),
                euler=(0,0,float(np.degrees(object_yaw))), fixed=True),
                surface=gs.surfaces.Default(color=(0.9,0.08,0.04)))
            camera = scene.add_camera(res=(width,height), fov=fov, GUI=False, debug=False)
            scene.build()
            from elesim_sim.vision.render_device import check_camera_renderer
            from elesim_sim.vision.sim_camera.mount import intrinsics_from_fov
            check_camera_renderer(scene.visualizer.rasterizer._renderer,
                                  use_gpu=True, label="blind-vision")
            sensor_intrinsic = intrinsics_from_fov(width=width, height=height, fov_deg=fov)
            intrinsic[:4] = [sensor_intrinsic.fx, sensor_intrinsic.fy,
                             sensor_intrinsic.cx, sensor_intrinsic.cy]
            # Read the renderer's actual optical calibration. This describes
            # sensor pixels, never the hidden camera extrinsic or object shape.
            if hasattr(camera, "intrinsics"):
                k = np.asarray(camera.intrinsics)
                true_intrinsic[:4] = [k[0,0], k[1,1], k[0,2]-.5, k[1,2]-.5]
                np.testing.assert_allclose(intrinsic, true_intrinsic, rtol=1e-6, atol=1e-6)
        try:
            for condition in ("observable", "unobservable"):
                boot = uuid.uuid4().hex
                hidden_mount = np.eye(4)
                if condition == "observable":
                    hidden_mount[:3,:3] = Rotation.from_rotvec([0.055,-0.04,0]).as_matrix()
                    hidden_mount[:3,3] = [0,0,-0.06]
                else:
                    hidden_mount[:3,:3] = Rotation.from_euler("z",0.14).as_matrix()
                    hidden_mount[:3,3] = [0.045,-0.025,0]
                for view_index in range(8):
                    q = np.array([0.10*np.cos(view_index*.7), 0.09*np.sin(view_index*.7),
                                  0.85+0.03*np.sin(view_index*.4), 0,0,0.06*np.sin(view_index*.5)])
                    actual = rig_pose(q) @ hidden_mount
                    if camera is None:
                        color, ideal = analytic_image(actual,
                            (intrinsic[2],intrinsic[0],intrinsic[3]), center,size,object_rotation,width,height)
                    else:
                        camera.set_pose(transform=actual @ np.diag([1,-1,-1,1]))
                        rgb, depth, _, _ = camera.render(rgb=True, depth=True, segmentation=False)
                        if hasattr(rgb,"cpu"): rgb=rgb.cpu().numpy()
                        if hasattr(depth,"cpu"): depth=depth.cpu().numpy()
                        color = np.ascontiguousarray(np.asarray(rgb)[...,:3][...,::-1],dtype=np.uint8)
                        ideal = np.asarray(depth,dtype=np.float32)
                    observed = ideal + rng.normal(0,0.0005,ideal.shape).astype(np.float32)
                    stamp = 10000+index*0.1
                    name = f"view-{index:04d}.npz"
                    np.savez_compressed(sensors/name, schema_version=np.array(1), color_bgr=color,
                        depth_raw=observed, depth_scale=np.array(1.0), intrinsics=intrinsic,
                        seq=np.array(view_index+1), ts=np.array(stamp), source_id=np.array("virtual-camera"),
                        source_boot_id=np.array(boot), calibration_id=np.array("pinhole-v1"), encoder_q=q)
                    np.savez_compressed(truth/name, ideal_depth=ideal, true_camera_pose=actual,
                        object_center=center, object_size=size, object_rotation=object_rotation,
                        true_intrinsics=true_intrinsic)
                    frames.append({"name":name,"boot":boot,"condition":condition,"seed":seed})
                    index += 1
        finally:
            if scene is not None and hasattr(scene,"destroy"):
                scene.destroy()
    (truth/"manifest.json").write_text(json.dumps({"schema_version":1,"backend":backend,
        "rig":"ideal virtual Cartesian encoder rig; no real arm validation",
        "scope":"static plane and visible object surfaces; no grasp/contact acceptance",
        "frames":frames},indent=2)+"\n")
    return {"frames":index,"backend":backend,"sensors":str(sensors),"truth":str(truth)}
