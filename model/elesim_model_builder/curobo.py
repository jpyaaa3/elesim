"""Export the shipped Genesis geometry as a four-axis cuRobo v0.7.8 model."""
from __future__ import annotations

import hashlib
import math
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def _min_projection(points, lower, upper):
    """Exact minima of x*cos(t)+z*sin(t), including interior extrema."""
    x, z = points[:, 0], points[:, 2]
    values = np.minimum(x * np.cos(lower) + z * np.sin(lower),
                        x * np.cos(upper) + z * np.sin(upper))
    phase = np.arctan2(z, x) + math.pi
    minimum_at = phase + 2 * math.pi * np.ceil((lower - phase) / (2 * math.pi))
    return float(np.min(np.where(minimum_at <= upper + 1e-12, -np.hypot(x, z), values)))


def _separation_bound(path, parent_vertices, child_vertices):
    """Conservative +X separation over independent joint limit intervals.

    EleSim's linear/X-roll/Y-bend chain admits an analytic interval bound.
    Coupled bend constraints only reduce this independent-joint superset.
    Unsupported rotations return None; they are never guessed collision-free.
    """
    lower = upper = offset = 0.0
    for joint in path:
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        if abs(rpy[0]) > 1e-10 or abs(rpy[2]) > 1e-10:
            return None
        offset += _min_projection(xyz.reshape(1, 3), lower, upper)
        lower += rpy[1]
        upper += rpy[1]
        kind = joint.get("type")
        if kind == "fixed":
            continue
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        limit = joint.find("limit")
        if kind == "prismatic" and np.allclose(axis, [1, 0, 0]) and lower == upper == 0:
            offset += float(limit.get("lower"))
        elif kind == "revolute" and np.allclose(axis, [0, 1, 0]):
            lower += float(limit.get("lower"))
            upper += float(limit.get("upper"))
        elif kind == "revolute" and np.allclose(axis, [1, 0, 0]) and lower == upper == 0:
            # An initial rotation about X does not change any later X projection.
            pass
        else:
            return None
    return offset + _min_projection(child_vertices, lower, upper) - float(parent_vertices[:, 0].max())


def build_curobo_model(*, assembly: Path, arm_model: Path, output: Path,
                       robot_config: Path = Path("payload/config/robot/default.yaml"),
                       voxel_m: float = 0.03) -> Path:
    # Optional build dependency only. Pilot never imports the model builder.
    import trimesh

    if not 0.005 <= voxel_m <= 0.03:
        raise ValueError("voxel_m must be between 0.005 and 0.03")
    context = json.loads(arm_model.read_text())["context"]
    hardware = yaml.safe_load(robot_config.read_text())["arm"]
    # Use the complete Robot-configured physical speed caps for planning.
    velocities = [hardware["max_linear_m_s"], hardware["max_roll_rad_s"],
                  hardware["max_bend_rad_s"], hardware["max_bend_rad_s"]]
    root = ET.parse(assembly / "arm.urdf").getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    bend = context["bend_joint_names"]
    n_seg = int(context["n_seg"])
    if len(bend) != 2 * n_seg:
        raise ValueError("expected two equally coupled bend sections")
    active = [context["linear_joint_name"], context["roll_joint_name"], bend[0], bend[n_seg]]
    limit = context["limit"]["value"]
    bend_limit = np.deg2rad(float(limit["bend_deg"]))
    bounds = [(context["linear_min_m"], context["linear_max_m"]),
              (np.deg2rad(limit["roll_min_deg"]), np.deg2rad(limit["roll_max_deg"])),
              (-bend_limit, bend_limit), (-bend_limit, bend_limit)]
    for name, (lower, upper), velocity in zip(active, bounds, velocities):
        element = joints[name].find("limit")
        element.set("lower", str(float(lower)))
        element.set("upper", str(float(upper)))
        element.set("velocity", str(float(velocity)))
    for i, name in enumerate(bend):
        master = bend[0 if i < n_seg else n_seg]
        if name != master:
            ET.SubElement(joints[name], "mimic", joint=master, multiplier="1", offset="0")
            joints[name].find("limit").set("lower", str(float(-bend_limit)))
            joints[name].find("limit").set("upper", str(float(bend_limit)))
            joints[name].find("limit").set("velocity", str(float(velocities[2])))
    # Match Pilot FK: all non-commanded joints (including claws) remain at zero.
    for name, joint in joints.items():
        if name not in active and name not in bend:
            joint.set("type", "fixed")
            for tag in ("limit", "mimic", "dynamics"):
                item = joint.find(tag)
                if item is not None:
                    joint.remove(item)

    spheres = {}
    sources = {"arm.urdf": hashlib.sha256((assembly / "arm.urdf").read_bytes()).hexdigest(),
               "robot_config.yaml": hashlib.sha256(robot_config.read_bytes()).hexdigest(),
               "arm_model.json": hashlib.sha256(arm_model.read_bytes()).hexdigest()}
    cache = {}
    geometry_vertices = {}
    for link in root.findall("link"):
        values = []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                raise ValueError("expected mesh collision geometry")
            path = (assembly / mesh.get("filename")).resolve()
            sources[mesh.get("filename")] = hashlib.sha256(path.read_bytes()).hexdigest()
            scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            key = (str(path), tuple(scale))
            if key not in cache:
                shape = trimesh.load_mesh(path, process=False)
                shape.apply_scale(scale)
                # Each occupied, filled voxel is enclosed by its circumsphere.
                cache[key] = (shape.voxelized(voxel_m).fill().points, np.asarray(shape.vertices))
            origin = collision.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rotation = Rotation.from_euler("xyz", rpy)
            centers = rotation.apply(cache[key][0]) + xyz
            geometry_vertices.setdefault(link.get("name"), []).append(rotation.apply(cache[key][1]) + xyz)
            values.extend({"center": [round(float(v), 9) for v in point],
                           "radius": float(np.sqrt(3) * voxel_m / 2 + 1e-6)} for point in centers)
        if values:
            spheres[link.get("name")] = values
        # GPU runtime uses the generated spheres, with no source mesh dependency.
        for element in list(link.findall("visual")) + list(link.findall("collision")):
            link.remove(element)

    if sum(map(len, spheres.values())) >= 1024:
        raise ValueError("cuRobo v0.7.8 supports fewer than 1024 collision spheres; increase voxel_m")

    # Ignore only mechanically connected pairs, including rigid subassemblies.
    groups = {link.get("name"): {link.get("name")} for link in root.findall("link")}
    for joint in joints.values():
        if joint.get("type") == "fixed":
            left, right = joint.find("parent").get("link"), joint.find("child").get("link")
            union = groups[left] | groups[right]
            for name in union:
                groups[name] = union
    pairs = set()
    for joint in joints.values():
        left, right = joint.find("parent").get("link"), joint.find("child").get("link")
        for a in groups[left]:
            for b in groups[right]:
                if a != b and a in spheres and b in spheres:
                    pairs.add(tuple(sorted((a, b))))
    geometry_vertices = {name: np.concatenate(values) for name, values in geometry_vertices.items()}
    parent_joints = {joint.find("child").get("link"): joint for joint in joints.values()}
    separated = []
    for child in spheres:
        cursor, path = child, []
        while cursor in parent_joints:
            joint = parent_joints[cursor]
            path.insert(0, joint)
            cursor = joint.find("parent").get("link")
            pair = tuple(sorted((cursor, child)))
            if cursor not in spheres or pair in pairs:
                continue
            gap = _separation_bound(path, geometry_vertices[cursor], geometry_vertices[child])
            if gap is not None and gap > 1e-6:
                pairs.add(pair)
                separated.append(dict(parent=cursor, child=child, min_gap_m=gap - 1e-9))
    ignore = {}
    for left, right in sorted(pairs):
        ignore.setdefault(left, []).append(right)

    grasp_offset = (np.asarray(context["old_tip_local_offset"]) +
                    np.asarray(context["grasp_offset_node_local"])).tolist()
    ET.SubElement(root, "link", name="elesim_grasp")
    tool = ET.SubElement(root, "joint", name="j_elesim_grasp", type="fixed")
    ET.SubElement(tool, "parent", link=context["terminal_link_name"])
    ET.SubElement(tool, "child", link="elesim_grasp")
    grasp_rpy = Rotation.from_matrix(context["approach_rot_tip"]).as_euler("xyz")
    ET.SubElement(tool, "origin", xyz=" ".join(map(str, grasp_offset)),
                  rpy=" ".join(map(str, grasp_rpy)))
    kinematics = dict(urdf_path="arm.urdf", base_link=context["fk_root_link"],
                      ee_link="elesim_grasp", link_names=list(spheres) + ["elesim_grasp"],
                      collision_link_names=list(spheres), collision_spheres=spheres,
                      collision_sphere_buffer=0.0, self_collision_ignore=ignore,
                      self_collision_buffer={name: 0.0 for name in spheres},
                      cspace=dict(joint_names=active, retract_config=[0.0] * 4,
                                  null_space_weight=[1.0] * 4,
                                  cspace_distance_weight=[1.0] * 4,
                                  max_acceleration=1.0, max_jerk=10.0))
    output.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output / "arm.urdf", encoding="unicode", xml_declaration=True)
    (output / "robot.yml").write_text(yaml.safe_dump({"robot_cfg": {"kinematics": kinematics}}, sort_keys=False))
    metadata = dict(schema_version=1, curobo_version="0.7.8+elesim.1", joint_names=active,
                    q_bounds=bounds, sources=sources, voxel_m=voxel_m,
                    max_velocity=velocities, robot_velocity_scale=1.0,
                    separated_pairs=separated,
                    sphere_count=sum(map(len, spheres.values())),
                    grasp_offset_local=grasp_offset)
    metadata["artifacts"] = {name: hashlib.sha256((output / name).read_bytes()).hexdigest()
                             for name in ("arm.urdf", "robot.yml")}
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return output / "robot.yml"


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--assembly", type=Path, default=Path("payload/data/models/assemblies/zed-mini"))
    parser.add_argument("--arm-model", type=Path, default=Path("payload/data/models/arm/default.json"))
    parser.add_argument("--output", type=Path, default=Path("payload/data/models/planning/zed-mini"))
    parser.add_argument("--robot-config", type=Path, default=Path("payload/config/robot/default.yaml"))
    parser.add_argument("--voxel-m", type=float, default=0.03)
    args = parser.parse_args()
    print(build_curobo_model(assembly=args.assembly, arm_model=args.arm_model, output=args.output,
                            robot_config=args.robot_config, voxel_m=args.voxel_m))


if __name__ == "__main__":
    main()
