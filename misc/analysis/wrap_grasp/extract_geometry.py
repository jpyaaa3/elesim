#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Extract wrap-grasp geometry parameters from the real arm/collision config.

Reads controller/config/arm_model.json, controller/config/collision_model.json
and controller/config/sag/sag_model.json and reports every value the wrap-grasp
feasibility analysis needs, in SI units, with provenance. Values that cannot be
found are reported as null with an explanation -- never guessed.

Run with:
    PYTHONPATH=packages/protocol/src:controller/src python3 \
        misc/analysis/wrap_grasp/extract_geometry.py [--out geometry_report.json]
"""

from __future__ import annotations

import argparse
import json
import math
import os
from typing import Any, Optional

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
ARM_MODEL_PATH = os.path.join(REPO_ROOT, "controller", "config", "arm_model.json")
COLLISION_MODEL_PATH = os.path.join(REPO_ROOT, "controller", "config", "collision_model.json")
SAG_MODEL_PATH = os.path.join(REPO_ROOT, "controller", "config", "sag", "sag_model.json")

# RA-L paper expectations, for the deviation table only -- never used as fallback values.
EXPECTED = {
    "h_pitch_m": 0.0415,
    "n_seg": 5,
    "alpha_max_deg": 36.0,
    "segment_bend_range_deg": 180.0,
    "arm_length_m": 0.415,
    "tendon_offset_w_m": 0.026,
    "gripper_max_opening_m": 0.080,
    "min_wrap_diameter_m": 0.070,
}


def _load_json(path: str) -> Optional[dict]:
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _null(reason: str) -> dict:
    return {"value": None, "reason": reason}


def extract_arc_geometry(arm_model: dict) -> dict:
    context = arm_model["context"]
    chain = list(context["fk_joint_chain"])
    bend_names = [str(x) for x in context["bend_joint_names"]]
    n_seg = int(context["n_seg"])

    joint_chain_summary = [
        {
            "name": m["name"],
            "parent": m["parent"],
            "child": m["child"],
            "type": m["type"],
            "origin_parent_m": m["origin_parent"],
            "axis_parent": m["axis_parent"],
        }
        for m in chain
    ]

    # Bend-joint pivot positions at q=0: for joint parent->child, the pivot sits
    # exactly at the child's nominal position (see kinematics._forward_link_tf:
    # p_child = p_parent + R_parent @ origin_parent, rotation applied only to
    # what follows). At q=0 all R are identity, so nominal part_pose_root gives
    # pivot locations directly without re-deriving FK.
    part_pose_root = context["part_pose_root"]
    # Build child-link name for each bend joint from the chain (authoritative,
    # not string-parsed from the joint name).
    child_of = {m["name"]: m["child"] for m in chain}
    bend_pivot_positions_m = [part_pose_root[child_of[n]] for n in bend_names]

    h_list_m = []
    for i in range(1, len(bend_pivot_positions_m)):
        a = bend_pivot_positions_m[i - 1]
        b = bend_pivot_positions_m[i]
        d = math.sqrt(sum((bi - ai) ** 2 for ai, bi in zip(a, b)))
        h_list_m.append(d)

    h_values_rounded = [round(h, 6) for h in h_list_m]
    h_uniform = (max(h_values_rounded) - min(h_values_rounded)) < 1e-4 if h_values_rounded else False
    h_mean = sum(h_list_m) / len(h_list_m) if h_list_m else None

    # Lead-in: distance from the "wedge" frame origin (roll-joint pivot) to the
    # first bend-joint pivot. Not part of the periodic h sequence -- it is a
    # rigid, non-bending stub.
    wedge_pos = part_pose_root["wedge"]
    first_pivot = bend_pivot_positions_m[0]
    lead_in_m = math.sqrt(sum((b - a) ** 2 for a, b in zip(wedge_pos, first_pivot)))

    limit = context["limit"]["value"]
    bend_deg = float(limit["bend_deg"])
    roll_min_deg = float(limit["roll_min_deg"])
    roll_max_deg = float(limit["roll_max_deg"])
    linear_min_m = float(context["linear_min_m"])
    linear_max_m = float(context["linear_max_m"])

    alpha_max_deg = bend_deg  # theta1/theta2 are applied directly per-joint in _build_q_map (no /n_seg)
    segment_bend_range_deg = alpha_max_deg * n_seg

    # Total backbone length: lead-in + all node-to-node pitches, terminal_link (node9) to gripper_base,
    # gripper_base to old_tip, old_tip to grasp point. Reported as several distinct, labeled lengths
    # rather than one number, since "arm length" is ambiguous across these choices.
    terminal_link = context["terminal_link_name"]
    node9_pos = part_pose_root[terminal_link]
    backbone_wedge_to_node9_m = math.sqrt(sum((b - a) ** 2 for a, b in zip(wedge_pos, node9_pos)))
    gripper_base_pos = part_pose_root["gripper_base"]
    node9_to_gripper_base_m = math.sqrt(sum((b - a) ** 2 for a, b in zip(node9_pos, gripper_base_pos)))
    old_tip_local = context["old_tip_local_offset"]
    old_tip_len_m = math.sqrt(sum(x * x for x in old_tip_local))
    grasp_offset_local = context["grasp_offset_node_local"]
    grasp_offset_len_m = math.sqrt(sum(x * x for x in grasp_offset_local))

    return {
        "fk_joint_chain": joint_chain_summary,
        "bend_joint_names": bend_names,
        "n_seg": n_seg,
        "n_bend_joints_total": len(bend_names),
        "bend_pivot_positions_m_root_frame": {n: p for n, p in zip(bend_names, bend_pivot_positions_m)},
        "h_pitch_m": {
            "values": h_values_rounded,
            "uniform": h_uniform,
            "mean_m": h_mean,
            "note": (
                "Spacing between consecutive bend-joint pivots (child-link nominal "
                "positions at q=0), NOT link/capsule boundaries. Excludes the wedge lead-in."
            ),
        },
        "wedge_lead_in_m": {
            "value": lead_in_m,
            "note": (
                "Rigid, non-bending stub from the roll-joint (wedge) frame origin to the "
                "first bend-joint pivot (j_wedge_node0). Not part of the periodic h sequence."
            ),
        },
        "joint_limits": {
            "linear_m": {"min": linear_min_m, "max": linear_max_m},
            "roll_deg": {"min": roll_min_deg, "max": roll_max_deg},
            "bend_deg_per_joint": {"min": -bend_deg, "max": bend_deg},
            "note": (
                "theta1/theta2 in q=(linear,roll,theta1,theta2) are applied directly to each "
                "bend joint within their segment in kinematics._build_q_map (out[joint]=theta+sag_err), "
                "i.e. theta IS the per-node angle alpha, not a total-segment angle divided by n_seg."
            ),
        },
        "alpha_max_deg_per_node": alpha_max_deg,
        "segment_bend_range_deg": segment_bend_range_deg,
        "lengths_m": {
            "wedge_to_node9_backbone": backbone_wedge_to_node9_m,
            "node9_to_gripper_base": node9_to_gripper_base_m,
            "old_tip_local_offset_len": old_tip_len_m,
            "grasp_offset_node_local_len": grasp_offset_len_m,
            "note": (
                "'Total arm length' is ambiguous; reporting each labeled segment length "
                "rather than a single figure. See strategy summary for the RA-L comparison."
            ),
        },
    }


def extract_sag_status(sag_model: Optional[dict]) -> dict:
    if sag_model is None:
        return {
            "sag_model_json_exists": False,
            "mode": None,
            "reason": f"file not found at {SAG_MODEL_PATH}",
        }
    model_type = sag_model.get("model_type")
    refined_keys = ("c1_family", "c1_params", "a1", "b1_coeffs")
    looks_refined = any(k in sag_model for k in refined_keys)
    if model_type:
        mode = str(model_type)
    elif looks_refined:
        mode = "func_finder_refined_v1 (inferred: model_type key absent, but c*/a*/b*_coeffs keys present)"
    elif any(k in sag_model for k in ("seg1_distribution", "seg1_amplitude")):
        mode = "distribution+amplitude expression mode"
    else:
        mode = "equal_offset / zero (no recognized mode keys)"

    def _count(key: str) -> Any:
        v = sag_model.get(key)
        if isinstance(v, list):
            return len(v)
        return v

    coeff_counts = {k: _count(k) for k in ("c1_params", "c2_params", "a1", "a2", "b1_coeffs", "b2_coeffs")}
    return {
        "sag_model_json_exists": True,
        "mode": mode,
        "coefficient_counts": coeff_counts,
        "note": "Coefficient VALUES intentionally omitted -- only structure/mode reported, per task rules.",
        "disabled_in_this_analysis": True,
        "disable_reason": (
            "sag_model.json is not trusted per task instructions; nominal (uncorrected) geometry "
            "is used for the sweep by default. Shape uncertainty is instead modeled via --shape-margin."
        ),
    }


def extract_collision_geometry(collision_model: Optional[dict], arm_model: dict) -> dict:
    if collision_model is None:
        return {"collision_model_json_exists": False, "reason": f"file not found at {COLLISION_MODEL_PATH}"}

    link_capsules = collision_model.get("link_capsules", {})
    link_boxes = collision_model.get("link_boxes", {})
    context = arm_model["context"]
    bend_link_names = ["wedge"] + [
        m["child"] for m in context["fk_joint_chain"] if str(m["name"]) in set(context["bend_joint_names"])
    ]

    capsule_report = {}
    for name in bend_link_names:
        entry = link_capsules.get(name)
        if entry is None:
            capsule_report[name] = _null(f"no link_capsules entry for '{name}' in collision_model.json")
            continue
        radius_m = float(entry["radius"])
        capsule_report[name] = {
            "shape": "capsule (isotropic)",
            "p0_local_m": entry["p0_local"],
            "p1_local_m": entry["p1_local"],
            "radius_m": radius_m,
            "d_inner_m": radius_m,
            "d_outer_m": radius_m,
        }

    box_report = {
        name: [
            {"center_local_m": b["center_local"], "half_extents_local_m": b["half_extents_local"]}
            for b in boxes
        ]
        for name, boxes in link_boxes.items()
    }

    return {
        "link_capsules_backbone": capsule_report,
        "link_boxes": box_report,
        "default_radius_m": collision_model.get("default_radius"),
        "d_inner_vs_d_outer": {
            "value": None,
            "reason": (
                "The capsule fitting method (misc/tooling/model_builder/.../collision_model.py: "
                "chain_axis_capsule) takes radius = max perpendicular distance from ANY mesh "
                "vertex to the backbone axis line -- a single isotropic scalar. There is no "
                "concave/convex split anywhere in the fitting code or the JSON schema, so "
                "d_inner and d_outer are identical (= the capsule radius) by construction, not "
                "independently measured quantities."
            ),
        },
        "tendon_routing_offset_w": {
            "value": None,
            "reason": (
                "grep -rni 'tendon' across controller/src, misc/tooling/model_builder/src, and "
                "controller/config returns zero matches. No tendon-channel offset, lobe, or "
                "asymmetric envelope is represented in the collision model at all -- the object "
                "could plausibly contact a tendon guide before the disc surface in reality, but "
                "this codebase has no data to quantify that; not modeled, not just unreported."
            ),
        },
    }


def extract_gripper(arm_model: dict) -> dict:
    context = arm_model["context"]
    return {
        "old_tip_local_offset_m": context["old_tip_local_offset"],
        "grasp_offset_node_local_m": context["grasp_offset_node_local"],
        "approach_axis_local": context["approach_axis_local"],
        "approach_link_name": context["approach_link_name"],
        "max_opening_width_m": {
            "value": None,
            "reason": (
                "No calibrated gripper opening-width spec exists anywhere in the repo (checked "
                "controller/config/*.json, *.yaml, and all gripper/claw Python source). The real "
                "gripper is driven as a boolean claw_closed open/close command "
                "(controller/src/elesim_controller/pick/state.py PanelState.claw_closed; "
                "controller/src/elesim_controller/pick/actions.py send_claw_command), not a "
                "continuous width. The only numeric claw range in the repo is a synthetic FK "
                "joint limit used purely for placing the claw meshes for collision/FK purposes "
                "(misc/tooling/model_builder/src/elesim_model_builder/json_builder.py: "
                "j_gripper_base_claw_left/right, limit_deg=(-0.02,0.0)/(0.0,0.02) -- i.e. 0.02m "
                "per claw, 0.04m combined if both were driven independently), explicitly not a "
                "measured hardware spec. Using it as 'gripper max opening' would misrepresent a "
                "modeling placeholder as ground truth, so it is left null."
            ),
            "unverified_fk_placeholder_combined_m": 0.04,
        },
    }


def extract_camera() -> dict:
    hand_eye_path = os.path.join(REPO_ROOT, "controller", "config", "calibration", "hand_eye.camera.json")
    hand_eye = _load_json(hand_eye_path)
    return {
        "hand_eye_extrinsic": (
            {
                "source": hand_eye_path,
                "parent_frame": hand_eye.get("parent_frame"),
                "child_frame": hand_eye.get("child_frame"),
                "translation_m": hand_eye.get("translation_m"),
                "quaternion_xyzw": hand_eye.get("quaternion_xyzw"),
                "note": (
                    "Mounted relative to 'node9', NOT the tip/gripper_base/camera-link frame used "
                    "by fk_joint_chain's own fixed 'camera' joint (j_gripper_base_camera, parent "
                    "gripper_base). These are two independent representations of camera placement "
                    "that are not verified consistent with each other in this analysis."
                ),
            }
            if hand_eye is not None
            else _null(f"file not found at {hand_eye_path}")
        ),
        "runtime_stream_config": {
            "source": "controller/config/config.yaml: simulation.cameras.hand_eye",
            "fov_deg": 60.0,
            "resolution": [640, 480],
            "max_hz": 30.0,
            "depth_enabled": True,
        },
        "depth_valid_range_m": {
            "value": {"z_min": 0.15, "z_max": 2.5},
            "note": (
                "These are Python keyword-argument DEFAULTS on "
                "estimate_object_position_camera() in "
                "controller/src/elesim_controller/vision/perception/depth_pose.py, used for "
                "outlier rejection during pose estimation -- not a sensor datasheet spec, and "
                "overridable per call. Reported as-is, flagged as a code default rather than a "
                "calibrated sensor range."
            ),
        },
    }


def extract_go2_mount() -> dict:
    return {
        "go2_mount_offset_m": {
            "value": [0.1, 0.0, 0.07],
            "source": "controller/config/config.yaml: robot.go2.spawn.mount_offset_m (actual deployed value)",
        },
        "go2_spawn_height_m": {
            "value": 0.32,
            "source": "controller/config/config.yaml: robot.go2.spawn.spawn_height (actual deployed value)",
        },
        "use_go2": {
            "value": True,
            "source": "controller/config/config.yaml: robot.go2.use_go2",
        },
        "schema_defaults_for_reference": {
            "go2_mount_offset_m": [0.0, 0.0, 0.08],
            "go2_spawn_height_m": 0.42,
            "source": "controller/src/elesim_controller/config/schema.py SpawnConfig defaults",
            "note": "config.yaml overrides these defaults; the deployed values above are what's actually used.",
        },
        "note": (
            "Go2ArmMount (controller/src/elesim_controller/robot/arm/mounts/go2_mount.py) is a "
            "runtime dataclass built from these config values via Go2ArmMount.from_context(); it "
            "is not itself a static config value."
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="geometry_report.json")
    args = parser.parse_args()

    arm_model = _load_json(ARM_MODEL_PATH)
    if arm_model is None:
        raise FileNotFoundError(ARM_MODEL_PATH)
    collision_model = _load_json(COLLISION_MODEL_PATH)
    sag_model = _load_json(SAG_MODEL_PATH)

    report: dict[str, Any] = {
        "sources": {
            "arm_model": ARM_MODEL_PATH,
            "collision_model": COLLISION_MODEL_PATH,
            "sag_model": SAG_MODEL_PATH,
        },
        "expected_ra_l_values": EXPECTED,
        "A_arc_geometry": extract_arc_geometry(arm_model),
        "B_collision_geometry": extract_collision_geometry(collision_model, arm_model),
        "C_gripper": extract_gripper(arm_model),
        "D_camera": extract_camera(),
        "E_base_mount": extract_go2_mount(),
        "F_sag_model_status": extract_sag_status(sag_model),
    }

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
