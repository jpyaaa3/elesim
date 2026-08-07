from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

import elesim_model_builder.cli as model_cli
from elesim_model_builder.collision_model import (
    _single_child_offset,
    bounding_box,
    bounding_box_excluding_outlier_cluster,
    bounding_box_pair_by_largest_gap,
    bounding_capsule,
    build_collision_model,
    build_cylinder_obstacle_capsule,
    build_go2_body_shapes,
    build_link_boxes,
    build_link_capsules,
    build_wall_with_hole_boxes,
    chain_axis_capsule,
    fit_plate_link_boxes,
    parse_urdf_link_collision_box,
)

ROOT = Path(__file__).resolve().parents[4]
ASSETS = ROOT / "misc/model/source/assets"
BLUEPRINT = ROOT / "misc/model/source/blueprint.json"
BASE_CONFIG = ROOT / "controller/config/config.yaml"
GO2_URDF = ASSETS / "go2" / "go2.urdf"

EXPECTED_LINK_NAMES = {
    "plate", "housing", "wedge",
    "node0", "node1", "node2", "node3", "node4",
    "node5", "node6", "node7", "node8", "node9",
    "gripper_base", "gripper_claw_left", "gripper_claw_right", "camera",
}


def test_bounding_capsule_is_tighter_than_a_single_sphere_for_a_real_mesh() -> None:
    p0, p1, radius = bounding_capsule(ASSETS / "node" / "node_mesh.obj")
    sphere_equivalent_radius = 0.0666  # farthest-vertex-from-center distance, computed by hand
    assert radius < sphere_equivalent_radius
    assert 0.0 < radius < 0.06
    assert not np.allclose(p0, p1)  # node is elongated enough to need a real segment


def test_bounding_box_pair_by_largest_gap_separates_plates_outlier_cluster() -> None:
    """`plate`'s real mesh has ~6% of its vertices forming a cluster far from
    the main body along local X, joined to it only by a handful of grossly
    oversized triangles (confirmed by measuring every face's area -- up to
    ~6400x the mesh's median face size) -- a mesh-export defect, not a real
    second sub-feature."""
    plate_mesh = ASSETS / "plate" / "plate_mesh.obj"
    single_center, single_half_extents = bounding_box(plate_mesh)

    main_box, secondary_box = bounding_box_pair_by_largest_gap(plate_mesh, axis=0)
    main_center, main_half = main_box
    secondary_center, secondary_half = secondary_box

    # The single-box fit spans the full gap between the two clusters -- each
    # split-out box must be tighter along the split axis.
    assert main_half[0] < single_half_extents[0]
    assert secondary_half[0] < single_half_extents[0]
    # The two clusters must not overlap along the split axis (that's the
    # whole point of splitting at the largest gap between them).
    main_lo, main_hi = main_center[0] - main_half[0], main_center[0] + main_half[0]
    secondary_lo, secondary_hi = secondary_center[0] - secondary_half[0], secondary_center[0] + secondary_half[0]
    assert main_hi < secondary_lo or secondary_hi < main_lo
    # `main_box` is fit to the larger *vertex-count* group (the densely
    # tessellated thin mounting plate), which for this mesh sits right at
    # the part's own local origin (where it joins `housing`) -- unlike the
    # sparser, coarser outlier cluster further out, which happens to have
    # the larger raw bounding *volume* despite fewer vertices.
    assert abs(main_center[0]) < abs(secondary_center[0])


def test_bounding_box_excluding_outlier_cluster_matches_the_main_cluster_only() -> None:
    plate_mesh = ASSETS / "plate" / "plate_mesh.obj"
    main_box, _secondary_box = bounding_box_pair_by_largest_gap(plate_mesh, axis=0)
    trimmed_center, trimmed_half_extents = bounding_box_excluding_outlier_cluster(plate_mesh, axis=0)
    assert trimmed_center == pytest.approx(main_box[0])
    assert trimmed_half_extents == pytest.approx(main_box[1])


def test_build_link_boxes_fits_three_plate_boxes_but_one_housing_box() -> None:
    with open(BLUEPRINT, "r", encoding="utf-8") as handle:
        blueprint = json.load(handle)
    boxes = build_link_boxes(blueprint=blueprint, source_root=ASSETS.parent, box_fit_names=("plate", "housing"))
    assert len(boxes["plate"]) == 3
    assert len(boxes["housing"]) == 1
    expected = fit_plate_link_boxes(ASSETS / "plate" / "plate_mesh.obj")
    for entry, (center, _half) in zip(boxes["plate"], expected):
        assert entry["center_local"] == pytest.approx(list(center))


def test_fit_plate_link_boxes_main_body_bridge_and_orin_dont_overlap_along_x() -> None:
    """Main body, bridge, and Orin should be three non-overlapping (along X)
    boxes covering the mesh's whole length, not gaps or double-coverage."""
    plate_mesh = ASSETS / "plate" / "plate_mesh.obj"
    boxes = fit_plate_link_boxes(plate_mesh)
    assert len(boxes) == 3
    main, bridge, orin = boxes
    main_x = (main[0][0] - main[1][0], main[0][0] + main[1][0])
    bridge_x = (bridge[0][0] - bridge[1][0], bridge[0][0] + bridge[1][0])
    # bridge and Orin overlap along X (Orin sits on top of part of the
    # bridge) -- only main-vs-tail is expected to be cleanly separated.
    assert main_x[1] <= bridge_x[0] or bridge_x[1] <= main_x[0]
    # Bridge and Orin must share a Z boundary with no gap between them (see
    # fit_plate_link_boxes's docstring on why Orin's box is extended down).
    bridge_top_z = bridge[0][2] + bridge[1][2]
    orin_bottom_z = orin[0][2] - orin[1][2]
    assert bridge_top_z == pytest.approx(orin_bottom_z)


def _load_real_fk_joint_chain() -> list[dict]:
    arm_model_path = ROOT / "controller/config/arm_model.json"
    with open(arm_model_path, "r", encoding="utf-8") as handle:
        return json.load(handle)["context"]["fk_joint_chain"]


def test_chain_axis_capsule_uses_the_given_axis_not_mesh_aabb() -> None:
    """Regression: `node`'s mesh is *wider* across its own rotation axis
    (0.088m) than along the chain's real travel direction (0.05m, per
    ``origin_parent``) -- the AABB-based ``bounding_capsule`` picks the
    former, orienting the capsule sideways to the chain. This silently let
    a live, same-direction theta1/theta2 bend (~32/26 degrees) visibly
    self-intersect the arm while the AABB-fit model reported +3.8cm clear.
    """
    node_mesh = ASSETS / "node" / "node_mesh.obj"
    chain_axis = (0.05, 0.0, 0.0)  # the real origin_parent for j_node0_node1

    p0, p1, chain_radius = chain_axis_capsule(node_mesh, chain_axis)
    assert p0 == pytest.approx([0.0, 0.0, 0.0])
    assert p1 == pytest.approx([0.05, 0.0, 0.0])

    aabb_p0, aabb_p1, aabb_radius = bounding_capsule(node_mesh)
    aabb_length = float(np.linalg.norm(np.asarray(aabb_p1) - np.asarray(aabb_p0)))
    assert aabb_length == pytest.approx(0.088, abs=1e-6)  # the AABB heuristic's (wrong) axis
    assert chain_radius != pytest.approx(aabb_radius)


def test_chain_axis_capsule_degenerate_axis_falls_back_to_bounding_capsule() -> None:
    """A zero-length chain axis (no real travel direction, e.g. `plate`) must
    fall back to `bounding_capsule`'s real two-point fit, not collapse to a
    single-point sphere measured from the local origin -- that produced a
    0.555m-radius sphere for `plate` in production (confirmed live), since
    its local origin sits at one edge of the mesh, not its centroid."""
    node_mesh = ASSETS / "node" / "node_mesh.obj"
    p0, p1, radius = chain_axis_capsule(node_mesh, (0.0, 0.0, 0.0))
    expected_p0, expected_p1, expected_radius = bounding_capsule(node_mesh)
    assert p0 == pytest.approx(expected_p0)
    assert p1 == pytest.approx(expected_p1)
    assert radius == pytest.approx(expected_radius)
    assert not np.allclose(p0, p1)  # a real two-point capsule, not a degenerate point


def test_single_child_offset_returns_none_for_branching_and_leaf_links() -> None:
    chain = _load_real_fk_joint_chain()
    assert _single_child_offset("gripper_base", chain) is None  # 3 children: claws + camera
    assert _single_child_offset("camera", chain) is None  # leaf, no children
    assert _single_child_offset("gripper_claw_left", chain) is None  # leaf


def test_single_child_offset_returns_the_real_chain_travel_direction() -> None:
    chain = _load_real_fk_joint_chain()
    offset = _single_child_offset("node0", chain)
    assert offset is not None
    assert offset.tolist() == pytest.approx([0.05, 0.0, 0.0])


def test_build_link_capsules_uses_chain_axis_fit_when_chain_is_given() -> None:
    with open(BLUEPRINT, "r", encoding="utf-8") as handle:
        blueprint = json.load(handle)
    chain = _load_real_fk_joint_chain()

    without_chain = build_link_capsules(blueprint=blueprint, source_root=ASSETS.parent)
    with_chain = build_link_capsules(blueprint=blueprint, source_root=ASSETS.parent, fk_joint_chain=chain)

    node0_without = without_chain["node0"]
    node0_with = with_chain["node0"]
    length_without = float(
        np.linalg.norm(np.asarray(node0_without["p1_local"]) - np.asarray(node0_without["p0_local"]))
    )
    length_with = float(np.linalg.norm(np.asarray(node0_with["p1_local"]) - np.asarray(node0_with["p0_local"])))
    assert length_without == pytest.approx(0.088, abs=1e-6)
    assert length_with == pytest.approx(0.05, abs=1e-6)

    # Branching/leaf links are unaffected either way -- no single child offset exists for them.
    assert with_chain["camera"] == without_chain["camera"]
    assert with_chain["gripper_base"] == without_chain["gripper_base"]


def test_parse_urdf_link_collision_box_reads_the_real_go2_chassis_box() -> None:
    length, width, height = parse_urdf_link_collision_box(GO2_URDF)
    assert (length, width, height) == pytest.approx((0.3762, 0.0935, 0.114))


def test_parse_urdf_link_collision_box_missing_link_raises() -> None:
    with pytest.raises(RuntimeError, match="not found"):
        parse_urdf_link_collision_box(GO2_URDF, link_name="does_not_exist")


def test_build_go2_body_shapes_torso_box_matches_the_real_urdf_collision_box() -> None:
    result = build_go2_body_shapes(GO2_URDF)
    assert len(result["go2_boxes"]) == 1
    torso = result["go2_boxes"][0]
    assert torso["label"] == "base"
    assert torso["center_body"] == pytest.approx([0.0, 0.0, 0.0])
    assert torso["half_extents_body"] == pytest.approx([0.1881, 0.04675, 0.057])
    assert np.allclose(torso["rot_body"], np.eye(3))


def test_build_go2_body_shapes_head_is_rigidly_fixed_to_base() -> None:
    """`Head_upper`/`Head_lower` attach via `type="fixed"` joints (confirmed
    by reading the real URDF), so both must come back as `go2_capsules`
    (rigid, base-frame) -- never as leg segments, which need live joint
    angles these don't have."""
    result = build_go2_body_shapes(GO2_URDF)
    labels = {c["label"] for c in result["go2_capsules"]}
    assert labels == {"Head_upper", "Head_lower"}
    by_label = {c["label"]: c for c in result["go2_capsules"]}
    # Head_lower is a <sphere> -- a degenerate (zero-length) capsule.
    assert by_label["Head_lower"]["p0_body"] == by_label["Head_lower"]["p1_body"]
    assert by_label["Head_lower"]["radius"] == pytest.approx(0.047)


def test_build_go2_body_shapes_leg_segments_cover_all_four_legs_with_flat_leg_q_ordering() -> None:
    result = build_go2_body_shapes(GO2_URDF)
    segments = result["go2_leg_segments"]
    assert len(segments) == 12
    by_name = {seg["name"]: seg for seg in segments}
    expected_order = [
        f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")
    ]
    assert [seg["leg_q_index"] for seg in segments] == list(range(12))
    assert [seg["name"] for seg in segments] == expected_order
    # hip's parent is "base" directly; thigh/calf chain off their own leg's
    # previous segment, not "base".
    assert by_name["FL_hip"]["parent"] == "base"
    assert by_name["FL_thigh"]["parent"] == "FL_hip"
    assert by_name["FL_calf"]["parent"] == "FL_thigh"
    # A genuine (if small) real-hardware asymmetry: FL's calf collision
    # geometry is slightly different from the other three legs' -- a
    # hand-written "mirror FL" formula would have silently gotten this
    # wrong for FR/RL/RR (see build_go2_body_shapes's docstring).
    assert by_name["FL_calf"]["radius"] == pytest.approx(0.012)
    assert by_name["FR_calf"]["radius"] == pytest.approx(0.013)
    assert by_name["RL_calf"]["radius"] == pytest.approx(0.013)
    assert by_name["RR_calf"]["radius"] == pytest.approx(0.013)


def test_build_collision_model_covers_every_fk_link_and_auto_detects_go2_urdf(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    result = build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
    )

    assert result == output
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    box_fit_names = {"housing", "gripper_base", "gripper_claw_left", "gripper_claw_right"}
    # `housing`/gripper parts are box-fit (see module docstring), not
    # capsule-fit -- covered by `link_boxes` instead. `plate` stays
    # capsule-fit (its own geometry doesn't matter -- see
    # DEFAULT_SELF_IGNORE_LINKS -- it's excluded from checks entirely).
    assert set(payload["link_capsules"].keys()) == EXPECTED_LINK_NAMES - box_fit_names
    for capsule in payload["link_capsules"].values():
        assert capsule["radius"] > 0.0
        assert len(capsule["p0_local"]) == 3
        assert len(capsule["p1_local"]) == 3
    assert set(payload["link_boxes"].keys()) == box_fit_names
    assert len(payload["link_boxes"]["housing"]) == 1
    assert len(payload["link_boxes"]["gripper_base"]) == 1
    assert len(payload["link_boxes"]["gripper_claw_left"]) == 1
    assert len(payload["link_boxes"]["gripper_claw_right"]) == 1
    for boxes in payload["link_boxes"].values():
        for box in boxes:
            assert len(box["center_local"]) == 3
            assert len(box["half_extents_local"]) == 3
            assert all(x > 0.0 for x in box["half_extents_local"])
    assert ["gripper_claw_left", "gripper_claw_right"] in payload["ignore_pairs"]
    assert set(payload["go2_ignore_links"]) == {"plate", "housing"}
    assert set(payload["self_ignore_links"]) == {"plate"}

    # go2.urdf sits under the real assets tree used in this test -- its full
    # body-rigid structure (torso box, Head_upper/Head_lower capsules) and
    # all 12 leg segments should be picked up automatically, with no
    # explicit --go2-body-* flags needed (see build_go2_body_shapes).
    assert len(payload["go2_boxes"]) == 1
    torso = payload["go2_boxes"][0]
    assert torso["label"] == "base"
    assert torso["half_extents_body"] == pytest.approx([0.1881, 0.04675, 0.057])
    assert {c["label"] for c in payload["go2_capsules"]} == {"Head_upper", "Head_lower"}
    assert len(payload["go2_leg_segments"]) == 12
    leg_names = {seg["name"] for seg in payload["go2_leg_segments"]}
    assert leg_names == {f"{leg}_{part}" for leg in ("FL", "FR", "RL", "RR") for part in ("hip", "thigh", "calf")}
    leg_q_indices = sorted(seg["leg_q_index"] for seg in payload["go2_leg_segments"])
    assert leg_q_indices == list(range(12))


def test_build_collision_model_explicit_go2_dimensions_override_urdf_autodetect(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        go2_body_length_m=1.0,
        go2_body_width_m=1.0,
        go2_body_height_m=1.0,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    # An explicit override skips URDF auto-detection entirely -- torso-only,
    # as a box (an exact fit for the 3 given dimensions, unlike the old
    # single-capsule override this replaced), no head/legs.
    assert payload["go2_capsules"] == []
    assert payload["go2_leg_segments"] == []
    assert len(payload["go2_boxes"]) == 1
    box = payload["go2_boxes"][0]
    assert box["half_extents_body"] == pytest.approx([0.5, 0.5, 0.5])
    assert box["center_body"] == pytest.approx([0.0, 0.0, 0.0])


def test_build_collision_model_without_a_go2_urdf_present_emits_no_capsule(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        go2_urdf_path=tmp_path / "no_such_go2.urdf",
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["go2_capsules"] == []
    assert payload["go2_boxes"] == []
    assert payload["go2_leg_segments"] == []
    assert payload["obstacle_boxes"] == []


def test_build_wall_with_hole_boxes_returns_four_non_overlapping_bars() -> None:
    boxes = build_wall_with_hole_boxes(
        center_world=(0.45, 0.0, 0.3),
        width_m=0.6,
        height_m=0.6,
        thickness_m=0.03,
        hole_width_m=0.25,
        hole_height_m=0.25,
    )
    assert len(boxes) == 4
    labels = {box["label"] for box in boxes}
    assert labels == {"wall_top", "wall_bottom", "wall_left", "wall_right"}
    for box in boxes:
        assert box["center_world"][0] == pytest.approx(0.45)
        assert box["half_extents_world"][0] == pytest.approx(0.015)
        assert box["rot_world"] == [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    # The 4 bars should tile the wall's full Y/Z extent with no gap and no
    # overlap: total area == wall area - hole area.
    wall_area = 0.6 * 0.6
    hole_area = 0.25 * 0.25
    bars_area = sum(4.0 * box["half_extents_world"][1] * box["half_extents_world"][2] for box in boxes)
    assert bars_area == pytest.approx(wall_area - hole_area)


def test_build_wall_with_hole_boxes_drops_a_bar_when_hole_touches_an_edge() -> None:
    """A hole exactly as wide as the wall leaves no left/right bar to emit."""
    boxes = build_wall_with_hole_boxes(
        center_world=(0.0, 0.0, 0.0),
        width_m=0.5,
        height_m=0.5,
        thickness_m=0.02,
        hole_width_m=0.5,
        hole_height_m=0.2,
    )
    labels = {box["label"] for box in boxes}
    assert labels == {"wall_top", "wall_bottom"}


def test_build_cylinder_obstacle_capsule_spans_the_given_height_centered_at_center_world() -> None:
    capsule = build_cylinder_obstacle_capsule(center_world=(0.55, 0.0, 0.5), radius_m=0.1, height_m=1.0)
    assert capsule["p0_world"] == pytest.approx([0.55, 0.0, 0.0])
    assert capsule["p1_world"] == pytest.approx([0.55, 0.0, 1.0])
    assert capsule["radius"] == pytest.approx(0.1)
    assert capsule["label"] == "cylinder"


def test_build_cylinder_obstacle_capsule_uses_the_given_label() -> None:
    capsule = build_cylinder_obstacle_capsule(
        center_world=(0.0, 0.0, 0.0), radius_m=0.05, height_m=0.4, label="post"
    )
    assert capsule["label"] == "post"


def test_build_collision_model_with_obstacle_capsules_persists_them_in_output(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    obstacle_capsules = [
        build_cylinder_obstacle_capsule(center_world=(0.55, 0.0, 0.5), radius_m=0.1, height_m=1.0)
    ]
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        go2_urdf_path=tmp_path / "no_such_go2.urdf",
        obstacle_capsules=obstacle_capsules,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["obstacle_boxes"] == []
    assert len(payload["obstacle_capsules"]) == 1
    assert payload["obstacle_capsules"][0]["label"] == "cylinder"
    assert payload["obstacle_capsules"][0]["radius"] == pytest.approx(0.1)


def test_build_collision_model_with_obstacle_boxes_persists_them_in_output(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    obstacle_boxes = build_wall_with_hole_boxes(
        center_world=(0.45, 0.0, 0.3),
        width_m=0.6,
        height_m=0.6,
        thickness_m=0.03,
        hole_width_m=0.25,
        hole_height_m=0.25,
    )
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        go2_urdf_path=tmp_path / "no_such_go2.urdf",
        obstacle_boxes=obstacle_boxes,
    )
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert len(payload["obstacle_boxes"]) == 4
    assert {box["label"] for box in payload["obstacle_boxes"]} == {
        "wall_top", "wall_bottom", "wall_left", "wall_right"
    }


def test_build_collision_model_with_go2_dimensions_emits_one_box(tmp_path: Path) -> None:
    output = tmp_path / "collision_model.json"
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        go2_body_length_m=0.7,
        go2_body_width_m=0.31,
        go2_body_height_m=0.4,
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert len(payload["go2_boxes"]) == 1
    box = payload["go2_boxes"][0]
    assert box["center_body"] == pytest.approx([0.0, 0.0, 0.0])
    assert box["half_extents_body"] == pytest.approx([0.35, 0.155, 0.2])


def test_build_collision_model_with_config_catches_the_live_confirmed_self_collision(tmp_path: Path) -> None:
    """The concrete bug this fix was built for: theta1/theta2 bent ~32/26
    degrees the *same* direction, confirmed live (against a running
    Simulator) to visibly self-intersect the arm at wedge-node9. Before
    the chain-axis capsule fit, this reported +3.8cm clear -- a false
    negative a real operator could have driven the arm into.
    """
    import numpy as np

    from elesim_controller.robot.arm.iklib.kinematics import Q_NEUTRAL
    from elesim_controller.robot.arm.iklib.solver import load_solver_context
    from elesim_controller.robot.arm.planning.collision import CollisionModel, check_configuration

    output = tmp_path / "collision_model.json"
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        config_path=BASE_CONFIG,
        discovery_samples=500,
    )

    _bundle, context = load_solver_context(str(BASE_CONFIG))
    model = CollisionModel.from_json(str(output))

    self_colliding_q = np.array([-0.1711, -0.7156, 0.5585, 0.4503])
    result = check_configuration(context=context, q=self_colliding_q, model=model)
    assert not result.ok
    assert result.reason == "self_collision"

    # Normal reference poses must still read as clear -- this isn't just a
    # blanket tightening that would reject everything.
    assert check_configuration(context=context, q=np.asarray(Q_NEUTRAL, dtype=float), model=model).ok


def test_build_collision_model_rejects_length_without_width_or_height(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        build_collision_model(
            blueprint_path=BLUEPRINT,
            source_root=ASSETS.parent,
            output=tmp_path / "collision_model.json",
            go2_body_length_m=0.7,
        )


def test_build_collision_model_with_config_discovers_always_colliding_pairs(tmp_path: Path) -> None:
    """End-to-end regression against a *fresh* random sweep (a different
    seed than discovery used).

    Deliberately does NOT assert Q_NEUTRAL/Q_BENT are collision-free: those
    are just the IK solver's numerically-convenient optimizer seeds, not
    verified collision-free reference targets -- `min_fraction` exists so a
    handful of atypical reference poses can't derail a real always-violating
    signal from the random sweep (they both happen to read clear under the
    current chain-axis capsule fit, but that isn't guaranteed by construction).

    Also deliberately does NOT require a low fresh-sweep violation rate: this
    arm's joint limits allow bending far enough to genuinely fold back over
    its own `plate`/`housing` mount across a *large*, and legitimately
    pose-dependent, slice of the raw uniformly-random range -- confirmed by
    hand-adding the dominant offending pair round after round
    (node1/plate -> node2/plate -> node3/plate -> ...), which never
    converged and always capped out at the same shallow ~5.6cm worst-case
    gap, i.e. the *same* recurring set of extreme folds, not runaway
    proxy-fit error. That's real geometry a planner should route around
    (and does -- see ``controller/tests/planning/test_planned_move.py``),
    not a defect to suppress via ever more ignore-pairs. This only guards
    against regressing back toward the *actual* pre-fix defects: (a) a
    near-100% violation rate from `plate`'s capsule collapsing to a
    0.555m-radius sphere, and (b) gross, multi-decimetre overlaps (up to
    -0.44m, confirmed live) rather than shallow, few-centimetre ones.
    """
    import numpy as np

    from elesim_controller.robot.arm.iklib.kinematics import _ReachModel
    from elesim_controller.robot.arm.iklib.solver import load_solver_context
    from elesim_controller.robot.arm.planning.collision import CollisionModel, check_configuration

    output = tmp_path / "collision_model.json"
    build_collision_model(
        blueprint_path=BLUEPRINT,
        source_root=ASSETS.parent,
        output=output,
        config_path=BASE_CONFIG,
        discovery_samples=500,
    )

    _bundle, context = load_solver_context(str(BASE_CONFIG))
    model = CollisionModel.from_json(str(output))

    reach_model = _ReachModel(context=context, limit=context["limit"])
    rng = np.random.default_rng(42)  # different seed than build_collision_model's discovery sweep
    violations = 0
    worst_clearance_m = 0.0
    n = 500
    for _ in range(n):
        q = np.array(
            [
                rng.uniform(reach_model.linear_min, reach_model.linear_max),
                rng.uniform(reach_model.roll_min, reach_model.roll_max),
                rng.uniform(-reach_model.bend_lim, reach_model.bend_lim),
                rng.uniform(-reach_model.bend_lim, reach_model.bend_lim),
            ]
        )
        result = check_configuration(context=context, q=q, model=model)
        if not result.ok:
            violations += 1
            worst_clearance_m = min(worst_clearance_m, result.min_clearance_m)
    assert violations / n < 0.85, f"{violations}/{n} fresh-sweep violations -- near-total failure, proxy likely broken"
    assert worst_clearance_m > -0.10, f"worst overlap {worst_clearance_m:.4f}m -- gross overlap, proxy likely broken"


def test_installed_cli_dispatches_to_the_packaged_builder(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    expected = {
        "blueprint_path": tmp_path / "blueprint.json",
        "source_root": tmp_path,
        "output": tmp_path / "collision_model.json",
        "default_radius": 0.03,
        "go2_body_length_m": None,
        "go2_body_width_m": None,
        "go2_body_height_m": None,
        "go2_urdf_path": None,
        "go2_urdf_link_name": "base",
        "config_path": None,
        "discovery_samples": 500,
        "obstacle_boxes": [],
        "obstacle_capsules": [],
    }
    received: dict[str, object] = {}

    def fake_build_collision_model(**kwargs: object) -> Path:
        received.update(kwargs)
        return kwargs["output"]

    monkeypatch.setattr(model_cli, "build_collision_model", fake_build_collision_model)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "elesim-build-collision-model",
            "--assets",
            str(tmp_path / "assets"),
            "--blueprint",
            str(expected["blueprint_path"]),
            "--output",
            str(expected["output"]),
        ],
    )

    model_cli.collision_model_main()

    assert received == expected
