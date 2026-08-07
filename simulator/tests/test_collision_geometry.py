from __future__ import annotations

import numpy as np
import pytest

from elesim_simulator.collision_geometry import (
    BOX_EDGE_INDICES,
    CollisionGeometryModel,
    simplify_go2_to_bounding_box,
    world_box_corners,
    world_capsule,
)


def _model(**overrides) -> CollisionGeometryModel:
    data = {
        "link_capsules": {
            "node0": {"p0_local": [0.0, 0.0, 0.0], "p1_local": [0.05, 0.0, 0.0], "radius": 0.03},
            "plate": {"p0_local": [0.0, 0.0, 0.0], "p1_local": [0.0, 0.0, 0.0], "radius": 0.5},
        },
        "link_boxes": {
            "housing": [{"center_local": [0.0, 0.0, 0.1], "half_extents_local": [0.1, 0.05, 0.1]}],
        },
        "go2_capsules": [{"p0_body": [-0.2, 0.0, 0.0], "p1_body": [0.2, 0.0, 0.0], "radius": 0.05}],
        "self_ignore_links": ["plate"],
        "go2_ignore_links": ["plate", "housing"],
    }
    data.update(overrides)
    return CollisionGeometryModel.from_dict(data)


def test_from_dict_parses_capsules_boxes_and_ignore_links() -> None:
    model = _model()
    assert set(model.link_capsules.keys()) == {"node0", "plate"}
    assert set(model.link_boxes.keys()) == {"housing"}
    assert len(model.go2_capsules) == 1
    assert model.self_ignore_links == frozenset({"plate"})
    assert model.go2_ignore_links == frozenset({"plate", "housing"})


def test_from_dict_parses_obstacle_boxes() -> None:
    model = _model(
        obstacle_boxes=[
            {"center_world": [0.75, 0.0, 0.3], "half_extents_world": [0.015, 0.3, 0.0875], "label": "demo_wall_top"}
        ]
    )
    assert len(model.obstacle_boxes) == 1
    obstacle = model.obstacle_boxes[0]
    assert np.allclose(obstacle.center_world, [0.75, 0.0, 0.3])
    assert obstacle.label == "demo_wall_top"
    assert np.allclose(obstacle.rot_world, np.eye(3))


def test_from_dict_parses_obstacle_capsules() -> None:
    model = _model(
        obstacle_capsules=[
            {"p0_world": [0.55, 0.0, 0.0], "p1_world": [0.55, 0.0, 1.0], "radius": 0.1, "label": "cylinder"}
        ]
    )
    assert len(model.obstacle_capsules) == 1
    obstacle = model.obstacle_capsules[0]
    assert np.allclose(obstacle.p0_world, [0.55, 0.0, 0.0])
    assert np.allclose(obstacle.p1_world, [0.55, 0.0, 1.0])
    assert obstacle.radius == pytest.approx(0.1)
    assert obstacle.label == "cylinder"


def test_is_inert_only_when_excluded_from_both_checks() -> None:
    model = _model()
    # `plate`: excluded from both self- and GO2-body checks -> never actually checked.
    assert model.is_inert("plate") is True
    # `housing`: excluded from GO2-body only, still checked for self-collision.
    assert model.is_inert("housing") is False
    # `node0`: excluded from neither.
    assert model.is_inert("node0") is False


def test_is_inert_defaults_to_false_when_ignore_links_absent() -> None:
    model = CollisionGeometryModel.from_dict({"link_capsules": {}, "link_boxes": {}})
    assert model.is_inert("anything") is False


def test_world_capsule_applies_link_pose() -> None:
    model = _model()
    capsule = model.link_capsules["node0"]
    p0, p1, radius = world_capsule(np.array([1.0, 2.0, 3.0]), np.eye(3), capsule)
    assert np.allclose(p0, [1.0, 2.0, 3.0])
    assert np.allclose(p1, [1.05, 2.0, 3.0])
    assert radius == 0.03


def test_simplify_go2_to_bounding_box_merges_torso_and_chassis_into_one_box() -> None:
    model = _model(
        go2_boxes=[
            {
                "center_body": [0.0, 0.0, 0.0],
                "half_extents_body": [0.1, 0.1, 0.1],
                "rot_body": np.eye(3).tolist(),
                "label": "torso",
            }
        ],
    )
    # default _model() already has a go2_capsules chassis capsule spanning x in [-0.2, 0.2] + radius 0.05.
    simplified = simplify_go2_to_bounding_box(model)
    assert simplified.go2_capsules == ()
    assert simplified.go2_leg_segments == ()
    assert len(simplified.go2_boxes) == 1
    box = simplified.go2_boxes[0]
    # chassis capsule spans x in [-0.25, 0.25] (center +/- radius), which is
    # wider than the torso box's own x in [-0.1, 0.1] -> merged half-extent
    # along x should come from the capsule, not the torso box.
    assert box.half_extents_body[0] == pytest.approx(0.25)


def test_simplify_go2_to_bounding_box_folds_in_leg_shapes_at_the_given_leg_q() -> None:
    model = _model(
        go2_capsules=[],
        go2_boxes=[
            {
                "center_body": [0.0, 0.0, 0.0],
                "half_extents_body": [0.1, 0.1, 0.1],
                "rot_body": np.eye(3).tolist(),
                "label": "torso",
            }
        ],
        go2_leg_segments=[
            {
                "name": "FL_calf",
                "parent": "base",
                "origin_from_parent": [0.0, 0.0, 0.0],
                "origin_rot_in_parent": np.eye(3).tolist(),
                "axis_in_parent": [0.0, 0.0, 1.0],
                "leg_q_index": 0,
                "shape_type": "capsule",
                "p0_local": [0.0, 0.0, 0.0],
                "p1_local": [1.0, 0.0, 0.0],
                "radius": 0.05,
            }
        ],
    )
    no_legs = simplify_go2_to_bounding_box(model)
    assert no_legs.go2_boxes[0].half_extents_body[0] == pytest.approx(0.1)

    with_legs = simplify_go2_to_bounding_box(model, leg_q=np.array([0.0]))
    box = with_legs.go2_boxes[0]
    assert box.half_extents_body[0] > 0.5  # torso alone would only be 0.1


def test_simplify_go2_to_bounding_box_is_a_noop_without_go2_shapes() -> None:
    model = CollisionGeometryModel.from_dict({"link_capsules": {}, "link_boxes": {}})
    simplified = simplify_go2_to_bounding_box(model, leg_q=np.zeros(12))
    assert simplified is model


def test_world_box_corners_returns_eight_corners_and_edges_connect_them() -> None:
    model = _model()
    box = model.link_boxes["housing"][0]
    corners = world_box_corners(np.zeros(3), np.eye(3), box)
    assert corners.shape == (8, 3)
    assert len(BOX_EDGE_INDICES) == 12
    for i, j in BOX_EDGE_INDICES:
        # Each edge connects two corners differing along exactly one axis.
        diff = np.abs(corners[i] - corners[j])
        assert np.count_nonzero(diff > 1e-9) == 1
