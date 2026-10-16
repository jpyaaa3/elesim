"""Real CUDA/EleSim FK, IK and joint-planning probe. Never opens DDS or hardware."""
from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from elesim_pilot.robot.arm.curobo import CuRoboError, CuRoboPlanner
from elesim_pilot.robot.arm.iklib.kinematics import _forward_grasp_world, _forward_link_tf


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--arm-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    print("Initializing cuRobo (offline, no DDS or hardware)", flush=True)
    planner = CuRoboPlanner(args.model, world={})
    initialization_s = time.monotonic() - started
    print("Checking GPU FK and collision constraints", flush=True)
    context = json.loads(args.arm_model.read_text())["context"]
    context["base_world_T"] = np.eye(4)
    rng = np.random.default_rng(42)
    samples = [np.zeros(4), *rng.uniform(planner.bounds[:, 0], planner.bounds[:, 1], size=(32, 4))]
    errors = []
    sphere_errors = []
    orientation_errors = []
    geometry = yaml.safe_load((args.model / "robot.yml").read_text())["robot_cfg"]["kinematics"]
    valid = []
    rejected = []
    for q in samples:
        expected = _forward_grasp_world(context, q)
        pose = planner.forward(q)
        actual = np.asarray(pose["position"])
        errors.append(float(np.linalg.norm(expected - actual)))
        transforms = _forward_link_tf(context, q)
        rotation = transforms[context["terminal_link_name"]][1] @ np.asarray(context["approach_rot_tip"])
        quaternion = pose["quaternion_wxyz"]
        actual_rotation = Rotation.from_quat(quaternion[1:] + quaternion[:1]).as_matrix()
        orientation_errors.append(float(Rotation.from_matrix(rotation.T @ actual_rotation).magnitude()))
        expected_spheres = []
        for name in geometry["collision_link_names"]:
            local = np.array([s["center"] + [s["radius"]] for s in geometry["collision_spheres"][name]])
            local[:, :3] = local[:, :3] @ transforms[name][1].T + transforms[name][0]
            expected_spheres.extend(local)
        state = planner._validator.kinematics.get_state(planner._tensor(q, planner._order))
        actual_spheres = state.link_spheres_tensor[0].detach().cpu().numpy()
        sphere_errors.append(float(np.abs(actual_spheres - np.asarray(expected_spheres)).max()))
        try:
            planner.validate_configuration(q)
            valid.append(q)
        except CuRoboError:
            rejected.append(q)
    assert max(errors) < 2e-5, f"GPU FK mismatch: {max(errors)} m"
    assert max(sphere_errors) < 2e-5, f"GPU collision geometry mismatch: {max(sphere_errors)}"
    assert max(orientation_errors) < 2e-5, f"GPU tool rotation mismatch: {max(orientation_errors)} rad"
    assert valid, "no collision-free seed found; inspect sphere coverage and ignored pairs"
    print(f"FK max error {max(errors):.3g} m; {len(valid)} collision-free seeds", flush=True)
    start = valid[0]
    assert rejected, "probe needs at least one self-colliding pose"
    try:
        planner.validate_configuration(rejected[0])
    except CuRoboError:
        pass
    else:
        raise AssertionError("self-colliding pose was accepted")
    # Stock v0.7.8's serial fallback leaves positive output after a collision.
    planner.validate_configuration(start)
    goal = start.copy()
    goal[0] = max(float(planner.bounds[0, 0]), float(start[0]) - .01)
    planner.validate_configuration(goal)
    plan_started = time.monotonic()
    trajectory = planner.plan_joint(start, goal)
    first_plan_s = time.monotonic() - plan_started
    plan_started = time.monotonic()
    planner.plan_joint(start, goal)
    repeat_plan_s = time.monotonic() - plan_started
    print(f"Joint plan: {len(trajectory.positions)} waypoints", flush=True)
    target = planner.forward(goal)["position"]
    ik_started = time.monotonic()
    solved = planner.solve_position(target, seed=start, tolerance_m=.002)
    first_ik_s = time.monotonic() - ik_started
    ik_started = time.monotonic()
    planner.solve_position(target, seed=start, tolerance_m=.002)
    repeat_ik_s = time.monotonic() - ik_started
    planner.validate_configuration(solved)
    # A large obstacle enclosing the robot must fail, not return an unchecked path.
    from curobo.geom.types import WorldConfig
    planner.motion.update_world(WorldConfig.from_dict({"cuboid": {"blocked": {
        "dims": [4., 4., 4.], "pose": [0., 0., 0., 1., 0., 0., 0.]
    }}}))
    try:
        planner.validate_configuration(start)
    except CuRoboError:
        pass
    else:
        raise AssertionError("obstacle collision was not rejected")
    try:
        planner.plan_joint(start, goal)
    except CuRoboError:
        pass
    else:
        raise AssertionError("blocked planning returned a path")
    result = dict(gpu_fk_samples=len(errors), max_fk_error_m=max(errors),
                  max_sphere_error_m=max(sphere_errors), max_orientation_error_rad=max(orientation_errors),
                  collision_free_samples=len(valid), waypoints=len(trajectory.positions),
                  trajectory_dt_s=trajectory.dt_s, start=start.tolist(), goal=goal.tolist(),
                  ik_solution=list(solved), obstacle_rejected=True,
                  self_collision_recovery=True,
                  initialization_s=initialization_s, first_plan_s=first_plan_s,
                  repeat_plan_s=repeat_plan_s, first_ik_s=first_ik_s,
                  repeat_ik_s=repeat_ik_s,
                  elapsed_s=time.monotonic() - started)
    import torch
    result["environment"] = dict(gpu=torch.cuda.get_device_name(0),
                                 torch=torch.__version__, cuda=torch.version.cuda,
                                 curobo=importlib.metadata.version("nvidia-curobo"),
                                 numpy=np.__version__)
    result["model_sha256"] = hashlib.sha256((args.model / "metadata.json").read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
