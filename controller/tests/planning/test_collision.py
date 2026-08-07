from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from elesim_controller.robot.arm.iklib.kinematics import Q_NEUTRAL, _forward_link_tf
from elesim_controller.robot.arm.iklib.solver import load_solver_context
from elesim_controller.robot.arm.planning.collision import (
    CollisionModel,
    Go2BodyBox,
    Go2BodyCapsule,
    Go2LegSegment,
    LinkBox,
    LinkCapsule,
    WorldBox,
    WorldCapsule,
    adjacent_link_pairs,
    box_box_gap,
    capsule_box_gap,
    capsule_capsule_gap,
    check_configuration,
    closest_point_on_segment,
    discover_always_colliding_pairs,
    environment_collision_check,
    go2_collision_check,
    go2_leg_world_shapes,
    segment_box_distance,
    segment_segment_distance,
    self_collision_check,
    simplify_go2_to_bounding_box,
)

CONFIG_PATH = Path(__file__).parents[2] / "config" / "config.yaml"


@pytest.fixture(scope="module")
def ik_context() -> dict:
    _bundle, context = load_solver_context(str(CONFIG_PATH))
    return context


def _point_capsule(radius: float) -> LinkCapsule:
    return LinkCapsule(p0_local=(0.0, 0.0, 0.0), p1_local=(0.0, 0.0, 0.0), radius=radius)


def test_closest_point_on_segment_clamps_to_endpoints() -> None:
    a, b = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
    assert np.allclose(closest_point_on_segment((2.0, 0.0, 0.0), a, b), (1.0, 0.0, 0.0))
    assert np.allclose(closest_point_on_segment((-2.0, 0.0, 0.0), a, b), (0.0, 0.0, 0.0))
    assert np.allclose(closest_point_on_segment((0.5, 1.0, 0.0), a, b), (0.5, 0.0, 0.0))


def test_closest_point_on_segment_handles_degenerate_segment() -> None:
    a = b = (1.0, 2.0, 3.0)
    assert np.allclose(closest_point_on_segment((0.0, 0.0, 0.0), a, b), a)


def test_segment_segment_distance_parallel_segments() -> None:
    dist = segment_segment_distance((0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0))
    assert dist == pytest.approx(1.0)


def test_segment_segment_distance_crossing_segments_is_zero() -> None:
    dist = segment_segment_distance((-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
    assert dist == pytest.approx(0.0, abs=1e-9)


def test_segment_segment_distance_skew_segments() -> None:
    # Two perpendicular segments offset along z; classic textbook case.
    dist = segment_segment_distance((0, 0, 0), (1, 0, 0), (0, 0, 1), (0, 1, 1))
    assert dist == pytest.approx(1.0)


def test_segment_segment_distance_degenerate_points() -> None:
    dist = segment_segment_distance((0, 0, 0), (0, 0, 0), (3, 4, 0), (3, 4, 0))
    assert dist == pytest.approx(5.0)


def test_capsule_capsule_gap_sign() -> None:
    gap = capsule_capsule_gap((0, 0, 0), (0, 0, 0), 0.1, (1, 0, 0), (1, 0, 0), 0.1)
    assert gap == pytest.approx(0.8)
    overlap = capsule_capsule_gap((0, 0, 0), (0, 0, 0), 0.6, (1, 0, 0), (1, 0, 0), 0.6)
    assert overlap < 0.0


def test_segment_box_distance_zero_when_segment_enters_the_box() -> None:
    dist = segment_box_distance((0, 0, 0), (0, 0, 0), (0, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    assert dist == pytest.approx(0.0)


def test_segment_box_distance_positive_when_clear() -> None:
    dist = segment_box_distance((1, 0, 0), (2, 0, 0), (0, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    # abs tolerance, not the default rel=1e-6 -- the golden-section search
    # is tuned to ~0.03mm precision (see _GOLDEN_SECTION_ITERS), plenty for
    # real clearance checks (cm-scale) but looser than an exact closed form.
    assert dist == pytest.approx(0.9, abs=1e-4)


def test_capsule_box_gap_sign() -> None:
    gap = capsule_box_gap((1, 0, 0), (2, 0, 0), 0.05, (0, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    assert gap == pytest.approx(0.85, abs=1e-4)
    overlap = capsule_box_gap((0, 0, 0), (0, 0, 0), 0.05, (0, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    assert overlap < 0.0


def test_box_box_gap_separated_axis_aligned() -> None:
    gap = box_box_gap((0, 0, 0), (0.1, 0.1, 0.1), np.eye(3), (0.5, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    assert gap == pytest.approx(0.3)


def test_box_box_gap_face_to_face_overlap_is_not_missed() -> None:
    """Regression: an earlier corner-sampling implementation reported this
    exact configuration as 0.0 ("just touching") -- no corner of either box
    sits inside the other even though a real 0.05m overlap volume exists
    between their faces (both boxes share the same Y/Z extent and center,
    so the only overlap is along X). Only a proper separating-axis test
    catches it."""
    gap = box_box_gap((0, 0, 0), (0.1, 0.1, 0.1), np.eye(3), (0.15, 0, 0), (0.1, 0.1, 0.1), np.eye(3))
    assert gap == pytest.approx(-0.05)


def test_box_box_gap_fully_nested() -> None:
    gap = box_box_gap((0, 0, 0), (0.05, 0.05, 0.05), np.eye(3), (0, 0, 0), (0.5, 0.5, 0.5), np.eye(3))
    assert gap < -0.5


def test_box_box_gap_separated_with_relative_rotation() -> None:
    from scipy.spatial.transform import Rotation as Rot

    rot45 = Rot.from_euler("z", 45, degrees=True).as_matrix()
    gap = box_box_gap((0, 0, 0), (0.1, 0.1, 0.1), np.eye(3), (0.5, 0, 0), (0.1, 0.1, 0.1), rot45)
    assert gap > 0.0
    assert gap == pytest.approx(0.5 - 0.1 - 0.1 * np.sqrt(2.0))


def test_adjacent_link_pairs_covers_every_joint(ik_context: dict) -> None:
    pairs = adjacent_link_pairs(ik_context["fk_joint_chain"])
    assert frozenset(("plate", "housing")) in pairs
    assert frozenset(("gripper_base", "gripper_claw_left")) in pairs
    assert frozenset(("node8", "node9")) in pairs
    assert frozenset(("plate", "node9")) not in pairs
    # Siblings of the same parent (both claws hang off gripper_base) are
    # expected to sit close together and must not be flagged either.
    assert frozenset(("gripper_claw_left", "gripper_claw_right")) in pairs


def test_self_collision_check_flags_forced_overlap(ik_context: dict) -> None:
    link_tf = {
        "node3": (np.zeros(3), np.eye(3)),
        "wedge": (np.array([10.0, 10.0, 10.0]), np.eye(3)),
        "node5": (np.zeros(3), np.eye(3)),
    }
    model = CollisionModel(
        link_capsules={"node3": _point_capsule(0.05), "wedge": _point_capsule(0.05), "node5": _point_capsule(0.05)}
    )
    result = self_collision_check(
        link_tf, fk_joint_chain=ik_context["fk_joint_chain"], model=model
    )
    assert result.ok is False
    assert result.reason == "self_collision"
    assert {result.link_a, result.link_b} == {"node3", "node5"}


def test_self_collision_check_ignores_mechanically_joined_links(ik_context: dict) -> None:
    link_tf = {
        "plate": (np.zeros(3), np.eye(3)),
        "housing": (np.zeros(3), np.eye(3)),
    }
    model = CollisionModel(link_capsules={"plate": _point_capsule(0.1), "housing": _point_capsule(0.1)})
    result = self_collision_check(
        link_tf, fk_joint_chain=ik_context["fk_joint_chain"], model=model
    )
    assert result.ok is True


def test_self_collision_check_default_self_ignore_links_is_empty(ik_context: dict) -> None:
    """`plate`/`housing` used to be wholesale-ignored by default -- their
    capsule fit collapsed to a giant single-point sphere, which falsely
    overlapped nearly every pose. Now that the fit is correct, nothing is
    ignored by default; a forced overlap involving `plate` must be caught
    like any other link pair."""
    link_tf = {
        "plate": (np.zeros(3), np.eye(3)),
        "node5": (np.zeros(3), np.eye(3)),  # forced exact overlap
    }
    model = CollisionModel(link_capsules={"plate": _point_capsule(0.1), "node5": _point_capsule(0.1)})
    assert model.self_ignore_links == frozenset()
    result = self_collision_check(link_tf, fk_joint_chain=ik_context["fk_joint_chain"], model=model)
    assert result.ok is False


def test_self_collision_check_honors_explicit_empty_self_ignore_links(ik_context: dict) -> None:
    link_tf = {
        "plate": (np.zeros(3), np.eye(3)),
        "node5": (np.zeros(3), np.eye(3)),
    }
    model = CollisionModel(
        link_capsules={"plate": _point_capsule(0.1), "node5": _point_capsule(0.1)},
        self_ignore_links=frozenset(),
    )
    result = self_collision_check(link_tf, fk_joint_chain=ik_context["fk_joint_chain"], model=model)
    assert result.ok is False


def test_self_collision_check_passes_for_neutral_pose_with_real_capsule_model(ik_context: dict) -> None:
    link_tf = _forward_link_tf(ik_context, Q_NEUTRAL)
    model = CollisionModel(link_capsules={}, default_radius=0.02)
    result = self_collision_check(
        link_tf, fk_joint_chain=ik_context["fk_joint_chain"], model=model
    )
    assert result.ok is True


def test_check_configuration_self_collision_only(ik_context: dict) -> None:
    model = CollisionModel(link_capsules={}, default_radius=0.02)
    result = check_configuration(context=ik_context, q=Q_NEUTRAL, model=model)
    assert result.ok is True
    assert np.isfinite(result.min_clearance_m)


def test_go2_collision_check_detects_capsule_overlap() -> None:
    link_tf = {"node9": (np.array([1.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        go2_capsules=(Go2BodyCapsule(p0_body=(0.0, 0.0, 0.0), p1_body=(2.0, 0.0, 0.0), radius=0.2, label="chassis"),),
        go2_ignore_links=frozenset(),
    )
    result = go2_collision_check(link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0))
    assert result.ok is False
    assert result.reason == "go2_collision"
    assert result.link_a == "node9"
    assert result.link_b == "chassis"


def test_go2_collision_check_respects_ignore_links() -> None:
    link_tf = {"plate": (np.array([1.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"plate": _point_capsule(0.05)},
        go2_capsules=(Go2BodyCapsule(p0_body=(0.0, 0.0, 0.0), p1_body=(2.0, 0.0, 0.0), radius=0.2, label="chassis"),),
        go2_ignore_links=frozenset({"plate"}),
    )
    result = go2_collision_check(link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0))
    assert result.ok is True


def test_go2_collision_check_no_capsules_is_always_ok() -> None:
    link_tf = {"node9": (np.array([1.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(link_capsules={"node9": _point_capsule(0.05)})
    result = go2_collision_check(link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0))
    assert result.ok is True
    assert result.min_clearance_m == float("inf")


def test_go2_collision_check_detects_torso_box_overlap() -> None:
    link_tf = {"node9": (np.array([0.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        go2_boxes=(Go2BodyBox(center_body=(0.0, 0.0, 0.0), half_extents_body=(0.2, 0.2, 0.2), label="torso"),),
    )
    result = go2_collision_check(link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0))
    assert result.ok is False
    assert result.reason == "go2_collision"
    assert result.link_b == "torso"


def test_go2_leg_world_shapes_fk_matches_hand_computed_pose() -> None:
    """A minimal synthetic 2-segment leg: hip (revolute about Z, offset
    (1,0,0) from base) -> thigh (revolute about X, offset (0,1,0) from hip),
    each a capsule of length 1 along local X."""
    hip = Go2LegSegment(
        name="test_hip",
        parent="base",
        origin_from_parent=(1.0, 0.0, 0.0),
        origin_rot_in_parent=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        axis_in_parent=(0.0, 0.0, 1.0),
        leg_q_index=0,
        shape_type="capsule",
        p0_local=(0.0, 0.0, 0.0),
        p1_local=(1.0, 0.0, 0.0),
        radius=0.05,
    )
    thigh = Go2LegSegment(
        name="test_thigh",
        parent="test_hip",
        origin_from_parent=(0.0, 1.0, 0.0),
        origin_rot_in_parent=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        axis_in_parent=(1.0, 0.0, 0.0),
        leg_q_index=1,
        shape_type="capsule",
        p0_local=(0.0, 0.0, 0.0),
        p1_local=(1.0, 0.0, 0.0),
        radius=0.05,
    )

    # Zero joint angles: pure translation chain, no rotation anywhere.
    shapes = go2_leg_world_shapes((0.0, 0.0, 0.0), np.eye(3), [0.0, 0.0], [hip, thigh])
    (is_box_a, hip_p0, hip_p1, hip_r, hip_label) = shapes[0]
    assert is_box_a is False
    assert hip_label == "test_hip"
    assert np.allclose(hip_p0, [1.0, 0.0, 0.0])
    assert np.allclose(hip_p1, [2.0, 0.0, 0.0])
    (is_box_b, thigh_p0, thigh_p1, thigh_r, thigh_label) = shapes[1]
    assert thigh_label == "test_thigh"
    # thigh's origin is (0,1,0) in the hip's (unrotated) frame -> world (1,1,0).
    assert np.allclose(thigh_p0, [1.0, 1.0, 0.0])
    assert np.allclose(thigh_p1, [2.0, 1.0, 0.0])

    # Rotate the hip 90 deg about Z: its own +X axis now points along world +Y,
    # so the hip capsule's far end and the thigh's origin move accordingly.
    shapes_rotated = go2_leg_world_shapes((0.0, 0.0, 0.0), np.eye(3), [np.pi / 2.0, 0.0], [hip, thigh])
    _, hip_p0_r, hip_p1_r, _, _ = shapes_rotated[0]
    assert np.allclose(hip_p0_r, [1.0, 0.0, 0.0])
    assert np.allclose(hip_p1_r, [1.0, 1.0, 0.0], atol=1e-9)
    # thigh's origin offset (0,1,0) is now expressed in the hip's *rotated*
    # frame: a +90deg Z rotation maps hip-local +Y to world -X, so
    # thigh's world origin is hip_pos (1,0,0) + (-1,0,0) = (0,0,0).
    _, thigh_p0_r, _, _, _ = shapes_rotated[1]
    assert np.allclose(thigh_p0_r, [0.0, 0.0, 0.0], atol=1e-9)


def test_go2_collision_check_with_leg_q_detects_leg_overlap() -> None:
    leg_segment = Go2LegSegment(
        name="FL_calf",
        parent="base",
        origin_from_parent=(0.0, 0.0, 0.0),
        origin_rot_in_parent=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        axis_in_parent=(0.0, 0.0, 1.0),
        leg_q_index=0,
        shape_type="capsule",
        p0_local=(0.0, 0.0, 0.0),
        p1_local=(0.0, 0.0, 0.0),
        radius=0.3,
    )
    link_tf = {"node9": (np.array([0.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        go2_leg_segments=(leg_segment,),
    )
    # Without leg_q, the leg isn't checked at all.
    clear = go2_collision_check(link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0))
    assert clear.ok is True
    assert clear.min_clearance_m == float("inf")
    # With leg_q, the (overlapping, since both are centered at the origin) leg capsule is checked.
    result = go2_collision_check(
        link_tf, model=model, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0), leg_q=[0.0]
    )
    assert result.ok is False
    assert result.link_b == "FL_calf"


def test_simplify_go2_to_bounding_box_merges_torso_and_head_into_one_box() -> None:
    model = CollisionModel(
        link_capsules={},
        go2_boxes=(Go2BodyBox(center_body=(0.0, 0.0, 0.0), half_extents_body=(0.2, 0.1, 0.1), label="torso"),),
        go2_capsules=(Go2BodyCapsule(p0_body=(0.3, 0.0, 0.0), p1_body=(0.3, 0.0, 0.1), radius=0.05, label="head"),),
    )
    simplified = simplify_go2_to_bounding_box(model)
    assert simplified.go2_capsules == ()
    assert simplified.go2_leg_segments == ()
    assert len(simplified.go2_boxes) == 1
    box = simplified.go2_boxes[0]
    # torso spans x in [-0.2, 0.2]; head capsule spans x in [0.25, 0.35]
    # (center 0.3 +/- radius 0.05) -> merged AABB x in [-0.2, 0.35].
    assert box.center_body[0] == pytest.approx(0.075)
    assert box.half_extents_body[0] == pytest.approx(0.275)


def test_simplify_go2_to_bounding_box_folds_in_leg_shapes_at_the_given_leg_q() -> None:
    leg_segment = Go2LegSegment(
        name="FL_calf",
        parent="base",
        origin_from_parent=(0.0, 0.0, 0.0),
        origin_rot_in_parent=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        axis_in_parent=(0.0, 0.0, 1.0),
        leg_q_index=0,
        shape_type="capsule",
        p0_local=(0.0, 0.0, 0.0),
        p1_local=(1.0, 0.0, 0.0),
        radius=0.05,
    )
    model = CollisionModel(
        link_capsules={},
        go2_boxes=(Go2BodyBox(center_body=(0.0, 0.0, 0.0), half_extents_body=(0.1, 0.1, 0.1), label="torso"),),
        go2_leg_segments=(leg_segment,),
    )
    # Without leg_q, only the torso box is there to merge.
    no_legs = simplify_go2_to_bounding_box(model)
    assert no_legs.go2_boxes[0].half_extents_body[0] == pytest.approx(0.1)

    # With leg_q, the leg capsule (spanning local x in [0, 1] + radius) pulls
    # the merged box's +x extent out to cover it.
    with_legs = simplify_go2_to_bounding_box(model, leg_q=[0.0])
    box = with_legs.go2_boxes[0]
    assert box.half_extents_body[0] > 0.5  # torso alone would only be 0.1

    # And go2_collision_check against the merged box still reports the
    # collision the leg-aware full-model check found in the sibling test
    # above -- the merged box is conservative, not looser.
    link_tf = {"node9": (np.array([0.5, 0.0, 0.0]), np.eye(3))}
    result = go2_collision_check(
        link_tf, model=with_legs, go2_pos=(0.0, 0.0, 0.0), go2_rpy_rad=(0.0, 0.0, 0.0)
    )
    assert result.ok is False
    assert result.link_b == "go2_merged"


def test_simplify_go2_to_bounding_box_is_a_noop_without_go2_shapes() -> None:
    model = CollisionModel(link_capsules={})
    simplified = simplify_go2_to_bounding_box(model, leg_q=[0.0] * 12)
    assert simplified is model


def test_environment_collision_check_clear_when_no_obstacles() -> None:
    link_tf = {"node9": (np.array([0.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(link_capsules={"node9": _point_capsule(0.05)})
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is True
    assert result.min_clearance_m == float("inf")


def test_environment_collision_check_detects_capsule_link_vs_obstacle_box_overlap() -> None:
    link_tf = {"node9": (np.array([0.45, 0.0, 0.3]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        obstacle_boxes=(WorldBox(center_world=(0.45, 0.0, 0.3), half_extents_world=(0.02, 0.3, 0.3), label="wall"),),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is False
    assert result.reason == "environment_collision"
    assert result.link_a == "node9"
    assert result.link_b == "wall"


def test_environment_collision_check_detects_box_link_vs_obstacle_box_overlap() -> None:
    link_tf = {"housing": (np.array([0.45, 0.0, 0.3]), np.eye(3))}
    model = CollisionModel(
        link_capsules={},
        link_boxes={"housing": (LinkBox(center_local=(0.0, 0.0, 0.0), half_extents_local=(0.05, 0.05, 0.05)),)},
        obstacle_boxes=(WorldBox(center_world=(0.45, 0.0, 0.3), half_extents_world=(0.02, 0.3, 0.3), label="wall"),),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is False
    assert result.link_a == "housing"
    assert result.link_b == "wall"


def test_environment_collision_check_reports_clear_gap_when_separated() -> None:
    link_tf = {"node9": (np.array([0.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        obstacle_boxes=(WorldBox(center_world=(2.0, 0.0, 0.0), half_extents_world=(0.02, 0.3, 0.3), label="wall"),),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is True
    assert result.min_clearance_m == pytest.approx(2.0 - 0.02 - 0.05)


def test_environment_collision_check_detects_capsule_link_vs_obstacle_capsule_overlap() -> None:
    link_tf = {"node9": (np.array([0.45, 0.0, 0.5]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        obstacle_capsules=(
            WorldCapsule(p0_world=(0.45, 0.0, 0.0), p1_world=(0.45, 0.0, 1.0), radius=0.1, label="cylinder"),
        ),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is False
    assert result.reason == "environment_collision"
    assert result.link_a == "node9"
    assert result.link_b == "cylinder"


def test_environment_collision_check_detects_box_link_vs_obstacle_capsule_overlap() -> None:
    link_tf = {"housing": (np.array([0.45, 0.0, 0.5]), np.eye(3))}
    model = CollisionModel(
        link_capsules={},
        link_boxes={"housing": (LinkBox(center_local=(0.0, 0.0, 0.0), half_extents_local=(0.05, 0.05, 0.05)),)},
        obstacle_capsules=(
            WorldCapsule(p0_world=(0.45, 0.0, 0.0), p1_world=(0.45, 0.0, 1.0), radius=0.1, label="cylinder"),
        ),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is False
    assert result.link_a == "housing"
    assert result.link_b == "cylinder"


def test_environment_collision_check_reports_clear_gap_for_obstacle_capsule_when_separated() -> None:
    link_tf = {"node9": (np.array([0.0, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        obstacle_capsules=(
            WorldCapsule(p0_world=(2.0, 0.0, 0.0), p1_world=(2.0, 0.0, 1.0), radius=0.1, label="cylinder"),
        ),
    )
    result = environment_collision_check(link_tf, model=model)
    assert result.ok is True
    assert result.min_clearance_m == pytest.approx(2.0 - 0.1 - 0.05)


def test_check_configuration_detects_environment_collision(ik_context: dict) -> None:
    link_tf = _forward_link_tf(ik_context, Q_NEUTRAL)
    some_link, (pos, _rot) = next(iter(link_tf.items()))
    # A tiny default_radius keeps self-collision clear (see
    # test_discover_always_colliding_pairs_is_empty_for_a_tiny_default_radius)
    # so the environment check is what actually trips here.
    model = CollisionModel(
        link_capsules={},
        default_radius=0.001,
        obstacle_boxes=(
            WorldBox(center_world=tuple(float(v) for v in pos), half_extents_world=(0.5, 0.5, 0.5), label="wall"),
        ),
    )
    result = check_configuration(context=ik_context, q=Q_NEUTRAL, model=model)
    assert result.ok is False
    assert result.reason == "environment_collision"
    assert result.link_b == "wall"


def test_environment_collision_check_negative_clearance_tolerates_a_small_overlap() -> None:
    # Point capsule (radius 0.05) placed so its raw gap against the box's
    # surface (at x=0.5) is exactly -0.003m: (0.547 - 0.5) - 0.05 = -0.003.
    link_tf = {"node9": (np.array([0.547, 0.0, 0.0]), np.eye(3))}
    model = CollisionModel(
        link_capsules={"node9": _point_capsule(0.05)},
        obstacle_boxes=(WorldBox(center_world=(0.0, 0.0, 0.0), half_extents_world=(0.5, 0.5, 0.5), label="wall"),),
    )
    strict = environment_collision_check(link_tf, model=model)
    assert strict.ok is False
    assert strict.min_clearance_m == pytest.approx(-0.003)

    tolerant = environment_collision_check(link_tf, model=model, clearance_m=-0.01)
    assert tolerant.ok is True

    still_strict = environment_collision_check(link_tf, model=model, clearance_m=-0.002)
    assert still_strict.ok is False


def test_check_configuration_environment_clearance_only_loosens_the_environment_check(ik_context: dict) -> None:
    link_tf = _forward_link_tf(ik_context, Q_NEUTRAL)
    _some_link, (pos, _rot) = next(iter(link_tf.items()))
    model = CollisionModel(
        link_capsules={},
        default_radius=0.001,
        obstacle_boxes=(
            WorldBox(center_world=tuple(float(v) for v in pos), half_extents_world=(0.5, 0.5, 0.5), label="wall"),
        ),
    )
    strict = check_configuration(context=ik_context, q=Q_NEUTRAL, model=model)
    assert strict.ok is False
    assert strict.reason == "environment_collision"

    # A huge negative environment_clearance_m swallows even this deep an
    # overlap, confirming it targets the environment check specifically
    # (self-collision here is already clear -- tiny default_radius).
    loosened = check_configuration(
        context=ik_context, q=Q_NEUTRAL, model=model, environment_clearance_m=-1.0
    )
    assert loosened.ok is True


def test_collision_model_from_dict_roundtrip() -> None:
    data = {
        "schema_version": 1,
        "link_capsules": {
            "plate": {"p0_local": [-0.05, 0.0, 0.0], "p1_local": [0.05, 0.0, 0.0], "radius": 0.05},
            "node0": {"p0_local": [0.0, 0.0, 0.0], "p1_local": [0.0, 0.0, 0.0], "radius": 0.03},
        },
        "default_radius": 0.025,
        "go2_capsules": [
            {"p0_body": [-0.3, 0.0, 0.0], "p1_body": [0.3, 0.0, 0.0], "radius": 0.18, "label": "chassis"}
        ],
        "obstacle_boxes": [
            {"center_world": [0.45, 0.0, 0.3], "half_extents_world": [0.02, 0.3, 0.3], "label": "wall"}
        ],
        "obstacle_capsules": [
            {"p0_world": [0.55, 0.0, 0.0], "p1_world": [0.55, 0.0, 1.0], "radius": 0.1, "label": "cylinder"}
        ],
        "ignore_pairs": [["gripper_claw_left", "gripper_claw_right"]],
        "go2_ignore_links": ["plate", "housing", "wedge"],
        "self_ignore_links": ["plate"],
    }
    model = CollisionModel.from_dict(data)
    assert model.capsule_for("plate").radius == pytest.approx(0.05)
    assert model.capsule_for("node9").radius == pytest.approx(0.025)  # falls back to default_radius
    assert frozenset(("gripper_claw_left", "gripper_claw_right")) in model.ignore_pairs
    assert model.go2_capsules[0].label == "chassis"
    assert model.obstacle_boxes[0].label == "wall"
    assert model.obstacle_capsules[0].label == "cylinder"
    assert model.obstacle_capsules[0].radius == pytest.approx(0.1)
    assert "wedge" in model.go2_ignore_links
    assert model.self_ignore_links == frozenset({"plate"})


def test_collision_model_from_dict_defaults_self_ignore_links_when_absent() -> None:
    model = CollisionModel.from_dict({"schema_version": 1, "link_capsules": {}})
    assert model.self_ignore_links == frozenset()


def test_discover_always_colliding_pairs_is_empty_for_a_tiny_default_radius(ik_context: dict) -> None:
    model = CollisionModel(link_capsules={}, default_radius=0.001)
    result = discover_always_colliding_pairs(context=ik_context, model=model, num_samples=100, seed=1)
    assert result == frozenset()


def test_discover_always_colliding_pairs_finds_forced_overlap(ik_context: dict) -> None:
    model = CollisionModel(link_capsules={}, default_radius=1.0, self_ignore_links=frozenset())
    result = discover_always_colliding_pairs(context=ik_context, model=model, num_samples=20, seed=1)
    assert len(result) > 0
    assert frozenset(("plate", "node0")) in result


def test_discover_always_colliding_pairs_checks_extra_q_samples_too(ik_context: dict) -> None:
    """A pin (e.g. the neutral pose) that random sampling would almost never
    hit exactly must still be able to force a pair into the result."""
    link_tf = _forward_link_tf(ik_context, Q_NEUTRAL)
    names = list(link_tf.keys())
    # Pick a genuinely far-apart pair so no random sample collides here --
    # only the pinned sample (reused as-is) should be able to flag it.
    forced_pair = frozenset((names[0], names[-1]))
    model = CollisionModel(link_capsules={}, default_radius=0.001, self_ignore_links=frozenset())
    baseline = discover_always_colliding_pairs(context=ik_context, model=model, num_samples=30, seed=2)
    assert forced_pair not in baseline

    huge_model = CollisionModel(link_capsules={}, default_radius=50.0, self_ignore_links=frozenset())
    result = discover_always_colliding_pairs(
        context=ik_context, model=huge_model, num_samples=0, seed=2, extra_q_samples=(Q_NEUTRAL,)
    )
    assert forced_pair in result


def test_discover_always_colliding_pairs_never_reincludes_an_ignored_pair(ik_context: dict) -> None:
    model = CollisionModel(
        link_capsules={},
        default_radius=1.0,
        ignore_pairs=frozenset({frozenset(("plate", "node0"))}),
    )
    result = discover_always_colliding_pairs(context=ik_context, model=model, num_samples=20, seed=1)
    assert frozenset(("plate", "node0")) not in result


def test_world_capsule_applies_local_offset_and_rotation() -> None:
    model = CollisionModel(
        link_capsules={"node0": LinkCapsule(p0_local=(-0.02, 0.0, 0.0), p1_local=(0.02, 0.0, 0.0), radius=0.03)}
    )
    rot_90_about_z = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    p0_world, p1_world, radius = model.world_capsule("node0", pos=np.array([1.0, 2.0, 3.0]), rot=rot_90_about_z)
    assert radius == pytest.approx(0.03)
    assert np.allclose(p0_world, [1.0, 1.98, 3.0])
    assert np.allclose(p1_world, [1.0, 2.02, 3.0])
