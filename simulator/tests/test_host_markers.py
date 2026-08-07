from __future__ import annotations

from elesim_simulator.runtime import resolve_host_marker


def test_resolve_host_marker_rejects_non_world_frame() -> None:
    marker = {"frame": "body", "name": "ready_pose", "pos": [0.0, 0.0, 0.0]}
    assert resolve_host_marker(marker) is None


def test_resolve_host_marker_rejects_missing_or_malformed_pos() -> None:
    assert resolve_host_marker({"name": "ready_pose"}) is None
    assert resolve_host_marker({"name": "ready_pose", "pos": [0.0, 0.0]}) is None
    assert resolve_host_marker({"name": "ready_pose", "pos": "not-a-vector"}) is None


def test_resolve_host_marker_rejects_unknown_name() -> None:
    assert resolve_host_marker({"name": "arbitrary", "pos": [0.0, 0.0, 0.0]}) is None


def test_resolve_host_marker_rejects_camera_names_even_though_they_look_like_markers() -> None:
    assert resolve_host_marker({"name": "camera_optical", "pos": [0.0, 0.0, 0.0]}) is None


def test_resolve_host_marker_ready_pose_matches_original_single_marker_keys() -> None:
    spec = resolve_host_marker({"name": "ready_pose", "pos": [1.0, 2.0, 3.0]})
    assert spec is not None
    assert spec.pos == (1.0, 2.0, 3.0)
    assert spec.sphere_key == "ready_pose:sphere"
    assert spec.arrow_key is None
    assert spec.rgba == (0.1, 1.0, 0.1, 0.95)  # documented default
    assert spec.radius == 0.012  # documented default


def test_resolve_host_marker_ready_pose_dir_has_no_sphere_key() -> None:
    spec = resolve_host_marker({"name": "ready_pose_dir", "pos": [0.0, 0.0, 0.0], "dir": [0.0, 0.0, 1.0]})
    assert spec is not None
    assert spec.sphere_key is None
    assert spec.arrow_key == "ready_pose_dir:dir"
    assert spec.direction == (0.0, 0.0, 1.0)


def test_resolve_host_marker_custom_color_and_radius() -> None:
    spec = resolve_host_marker(
        {"name": "ready_pose", "pos": [0.0, 0.0, 0.0], "color": [1.0, 0.0, 0.0], "radius": 0.05}
    )
    assert spec is not None
    assert spec.rgba == (1.0, 0.0, 0.0, 0.95)
    assert spec.radius == 0.05


def test_resolve_host_marker_planned_waypoint_uses_key_to_stay_distinct() -> None:
    spec_a = resolve_host_marker({"name": "planned_waypoint", "key": "0", "pos": [0.0, 0.0, 0.0]})
    spec_b = resolve_host_marker({"name": "planned_waypoint", "key": "1", "pos": [1.0, 0.0, 0.0]})
    assert spec_a is not None and spec_b is not None
    assert spec_a.sphere_key == "planned_waypoint:0:sphere"
    assert spec_b.sphere_key == "planned_waypoint:1:sphere"
    assert spec_a.sphere_key != spec_b.sphere_key


def test_resolve_host_marker_planned_waypoint_without_key_falls_back_to_shared_key() -> None:
    # Documents the one-marker-per-name fallback: two waypoints sent without
    # a `key` would overwrite each other's rendered sphere -- callers driving
    # multiple simultaneous waypoints must set a distinct `key` per marker.
    spec_a = resolve_host_marker({"name": "planned_waypoint", "pos": [0.0, 0.0, 0.0]})
    spec_b = resolve_host_marker({"name": "planned_waypoint", "pos": [1.0, 0.0, 0.0]})
    assert spec_a.sphere_key == spec_b.sphere_key == "planned_waypoint:sphere"


def test_resolve_host_marker_capsule_shape_draws_a_line_not_a_point() -> None:
    spec = resolve_host_marker(
        {
            "name": "planned_waypoint",
            "key": "collision_link_a",
            "shape": "capsule",
            "p0": [0.0, 0.0, 0.0],
            "p1": [0.05, 0.0, 0.0],
            "radius": 0.03,
        }
    )
    assert spec is not None
    assert spec.kind == "capsule"
    assert spec.p0 == (0.0, 0.0, 0.0)
    assert spec.p1 == (0.05, 0.0, 0.0)
    assert spec.line_key == "planned_waypoint:collision_link_a:line"
    assert spec.sphere_key is None
    assert spec.box_key is None


def test_resolve_host_marker_capsule_shape_rejects_missing_endpoints() -> None:
    assert resolve_host_marker({"name": "planned_waypoint", "shape": "capsule", "p0": [0.0, 0.0, 0.0]}) is None
    assert resolve_host_marker({"name": "planned_waypoint", "shape": "capsule"}) is None


def test_resolve_host_marker_box_shape_draws_the_real_extent() -> None:
    spec = resolve_host_marker(
        {
            "name": "planned_waypoint",
            "key": "collision_link_b",
            "shape": "box",
            "bounds": [[-0.5, -0.1, -0.05], [0.0, 0.1, 0.1]],
        }
    )
    assert spec is not None
    assert spec.kind == "box"
    assert spec.bounds == ((-0.5, -0.1, -0.05), (0.0, 0.1, 0.1))
    assert spec.box_key == "planned_waypoint:collision_link_b:box"
    assert spec.sphere_key is None
    assert spec.line_key is None


def test_resolve_host_marker_box_shape_rejects_malformed_bounds() -> None:
    assert resolve_host_marker({"name": "planned_waypoint", "shape": "box"}) is None
    assert resolve_host_marker({"name": "planned_waypoint", "shape": "box", "bounds": [[0.0, 0.0, 0.0]]}) is None


def test_resolve_host_marker_unknown_shape_falls_back_to_sphere_validation() -> None:
    # An unrecognised "shape" value isn't specially handled -- falls through
    # to the default sphere path, which then requires "pos".
    assert resolve_host_marker({"name": "planned_waypoint", "shape": "nonsense"}) is None
    spec = resolve_host_marker({"name": "planned_waypoint", "shape": "nonsense", "pos": [0.0, 0.0, 0.0]})
    assert spec is not None
    assert spec.kind == "sphere"
