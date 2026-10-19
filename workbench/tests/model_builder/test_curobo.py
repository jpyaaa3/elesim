"""Parity of the deployment artifact, not a replacement for CUDA smoke tests."""
import hashlib
import itertools
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import cdist
import yaml

from elesim_pilot.robot.arm.iklib.kinematics import _forward_grasp_world, _forward_link_tf
from elesim_model_builder.curobo import _min_projection

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "payload/data/models/planning/zed-mini"


def urdf_transforms(q):
    metadata = json.loads((MODEL / "metadata.json").read_text())
    joints = ET.parse(MODEL / "arm.urdf").getroot().findall("joint")
    values = dict(zip(metadata["joint_names"], q))
    transforms = {"plate": np.eye(4)}
    for joint in joints:
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        local = np.eye(4)
        local[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        local[:3, 3] = xyz
        name = joint.get("name")
        mimic = joint.find("mimic")
        angle = values.get(name, 0.0) if mimic is None else values[mimic.get("joint")]
        motion = np.eye(4)
        if joint.get("type") != "fixed":
            axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
            if joint.get("type") == "prismatic":
                motion[:3, 3] = axis * angle
            else:
                motion[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
        transforms[joint.find("child").get("link")] = transforms[joint.find("parent").get("link")] @ local @ motion
    return transforms


def urdf_forward(q, *, full=False):
    transform = urdf_transforms(q)["elesim_grasp"]
    return transform if full else transform[:3, 3]


def test_coupled_urdf_matches_pilot_fk():
    context = json.loads((ROOT / "payload/data/models/arm/default.json").read_text())["context"]
    # Isolate the plate-relative model from the runtime's floating base pose.
    context["base_world_T"] = np.eye(4)
    random = np.random.default_rng(196)
    bounds = np.asarray(json.loads((MODEL / "metadata.json").read_text())["q_bounds"])
    for q in [np.zeros(4), bounds[:, 0], bounds[:, 1], *random.uniform(bounds[:, 0], bounds[:, 1], size=(80, 4))]:
        assert urdf_forward(q) == pytest.approx(_forward_grasp_world(context, q), abs=2e-8)
        expected_rotation = _forward_link_tf(context, q)[context["terminal_link_name"]][1] @ np.asarray(context["approach_rot_tip"])
        assert urdf_forward(q, full=True)[:3, :3] == pytest.approx(expected_rotation, abs=2e-8)
        # Includes fixed gripper/camera frames, whose connector anchors differ
        # from the CAD link origins even though the grasp point FK was correct.
        pilot_links = _forward_link_tf(context, q)
        for name, transform in urdf_transforms(q).items():
            if name == "elesim_grasp":
                continue
            assert pilot_links[name][0] == pytest.approx(transform[:3, 3], abs=2e-8), name
            assert pilot_links[name][1] == pytest.approx(transform[:3, :3], abs=2e-8), name


def test_artifact_is_portable_and_sources_are_current():
    metadata = json.loads((MODEL / "metadata.json").read_text())
    assembly = ROOT / "payload/data/models/assemblies/zed-mini"
    for name, digest in metadata["sources"].items():
        special = {"arm_model.json": ROOT / "payload/data/models/arm/default.json",
                   "robot_config.yaml": ROOT / "payload/config/robot/default.yaml"}
        source = special.get(name, assembly / name)
        assert hashlib.sha256(source.read_bytes()).hexdigest() == digest
    root = ET.parse(MODEL / "arm.urdf").getroot()
    assert not root.findall(".//mesh")
    independent = [joint.get("name") for joint in root.findall("joint")
                   if joint.get("type") != "fixed" and joint.find("mimic") is None]
    assert independent == metadata["joint_names"]
    assert len(root.findall(".//mimic")) == 8
    assert metadata["q_bounds"][0] == pytest.approx([-259/1.5/1000, 0.0])
    config = yaml.safe_load((MODEL / "robot.yml").read_text())["robot_cfg"]["kinematics"]
    assert config["urdf_path"] == "arm.urdf"
    assert set(config["collision_link_names"]) == set(config["collision_spheres"])
    assert metadata["sphere_count"] > 0


def test_projection_bound_includes_interior_extrema():
    rng = np.random.default_rng(42)
    for _ in range(30):
        vertices = rng.uniform(-.2, .2, (12, 3))
        lower = float(rng.uniform(-8, 8))
        upper = lower + float(rng.uniform(.01, 14))
        theta = np.linspace(lower, upper, 20000)
        sampled = float((vertices[:, 0, None] * np.cos(theta) + vertices[:, 2, None] * np.sin(theta)).min())
        bound = _min_projection(vertices, lower, upper)
        assert bound <= sampled + 1e-12
        assert sampled - bound < 1e-7


def test_separated_pairs_remain_separated_with_moving_base_and_bends():
    metadata = json.loads((MODEL / "metadata.json").read_text())
    context = json.loads((ROOT / "payload/data/models/arm/default.json").read_text())["context"]
    context["base_world_T"] = np.eye(4)
    assembly = ROOT / "payload/data/models/assemblies/zed-mini"
    vertices = {}
    for link in ET.parse(assembly / "arm.urdf").getroot().findall("link"):
        pieces = []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            points = np.array([[float(x) for x in line.split()[1:4]]
                               for line in (assembly / mesh.get("filename")).read_text().splitlines()
                               if line.startswith("v ")])
            points *= np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
            origin = collision.find("origin")
            points = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).apply(points)
            points += np.fromstring(origin.get("xyz"), sep=" ")
            pieces.append(points)
        if pieces:
            vertices[link.get("name")] = np.concatenate(pieces)
    bounds = np.array(metadata["q_bounds"])
    samples = list(itertools.product(*bounds)) + list(np.random.default_rng(9).uniform(bounds[:, 0], bounds[:, 1], (80, 4)))
    assert metadata["separated_pairs"]
    for q in samples:
        transforms = _forward_link_tf(context, q)
        for pair in metadata["separated_pairs"]:
            parent, child = pair["parent"], pair["child"]
            parent_p, parent_r = transforms[parent]
            child_p, child_r = transforms[child]
            local = (vertices[child] @ child_r.T + child_p - parent_p) @ parent_r
            gap = float(local[:, 0].min() - vertices[parent][:, 0].max())
            assert gap >= pair["min_gap_m"] - 1e-8, pair


def test_nominal_pose_has_no_false_sphere_collision():
    context = json.loads((ROOT / "payload/data/models/arm/default.json").read_text())["context"]
    context["base_world_T"] = np.eye(4)
    transforms = _forward_link_tf(context, np.zeros(4))
    config = yaml.safe_load((MODEL / "robot.yml").read_text())["robot_cfg"]["kinematics"]
    spheres = {name: np.array([s["center"] + [s["radius"]] for s in values])
               for name, values in config["collision_spheres"].items()}
    ignored = {tuple(sorted((a, b))) for a, values in config["self_collision_ignore"].items() for b in values}
    for left, right in itertools.combinations(spheres, 2):
        if tuple(sorted((left, right))) in ignored:
            continue
        a, b = spheres[left], spheres[right]
        world_a = a[:, :3] @ transforms[left][1].T + transforms[left][0]
        world_b = b[:, :3] @ transforms[right][1].T + transforms[right][0]
        gap = float((cdist(world_a, world_b) - a[:, 3, None] - b[None, :, 3]).min())
        assert gap >= 0, (left, right, gap)
