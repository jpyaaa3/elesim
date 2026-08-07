from __future__ import annotations

import argparse
from pathlib import Path

import json

from elesim_model_builder.arm_model import build_arm_model
from elesim_model_builder.bundle import build_simulator_bundle
from elesim_model_builder.collision_model import (
    build_collision_model,
    build_cylinder_obstacle_capsule,
    build_wall_with_hole_boxes,
)


def sim_bundle_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="misc/model/source/assets")
    parser.add_argument("--output", default="model/bundles/default")
    parser.add_argument("--use-go2", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-hardware", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--mount", nargs=3, type=float, default=(0.35, 0.0, 0.08))
    args = parser.parse_args()
    output = build_simulator_bundle(
        asset_root=Path(args.assets),
        output_dir=Path(args.output),
        use_hardware=bool(args.use_hardware),
        use_go2=bool(args.use_go2),
        mount_xyz=tuple(args.mount),
    )
    print(output)


def arm_model_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="controller/config/config.pc.yaml")
    parser.add_argument("--assets", default="misc/model/source/assets")
    parser.add_argument("--output", default="controller/config/arm_model.json")
    args = parser.parse_args()
    output = build_arm_model(
        config=Path(args.config),
        assets=Path(args.assets),
        output=Path(args.output),
    )
    print(output)


def collision_model_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="misc/model/source/assets")
    parser.add_argument("--blueprint", default=None, help="defaults to <assets>/../blueprint.json")
    parser.add_argument("--output", default="controller/config/collision_model.json")
    parser.add_argument("--default-radius", type=float, default=0.03)
    parser.add_argument(
        "--go2-body-length",
        type=float,
        default=None,
        help="override; default reads assets/go2/go2.urdf's base-link collision box",
    )
    parser.add_argument("--go2-body-width", type=float, default=None, help="override, see --go2-body-length")
    parser.add_argument("--go2-body-height", type=float, default=None, help="override, see --go2-body-length")
    parser.add_argument(
        "--go2-urdf",
        default=None,
        help="defaults to <assets>/go2/go2.urdf; pass --go2-body-length/-width/-height to skip URDF parsing",
    )
    parser.add_argument("--go2-urdf-link", default="base")
    parser.add_argument(
        "--config",
        default=None,
        help=(
            "controller IK config used to auto-discover always-colliding link "
            "pairs (random-pose sweep); omit to skip discovery"
        ),
    )
    parser.add_argument("--discovery-samples", type=int, default=500)
    parser.add_argument(
        "--wall-with-hole",
        nargs=8,
        type=float,
        default=None,
        metavar=("CX", "CY", "CZ", "WIDTH", "HEIGHT", "THICKNESS", "HOLE_WIDTH", "HOLE_HEIGHT"),
        help=(
            "add a wall-with-a-hole obstacle: world center (cx,cy,cz), the wall's "
            "width/height/thickness (m), and a centered rectangular hole's width/height (m)"
        ),
    )
    parser.add_argument(
        "--cylinder-obstacle",
        nargs=5,
        type=float,
        default=None,
        metavar=("CX", "CY", "CZ", "RADIUS", "HEIGHT"),
        help="add a solid vertical cylindrical obstacle: world center (cx,cy,cz), radius/height (m)",
    )
    parser.add_argument(
        "--obstacles-json",
        default=None,
        help="path to a JSON file containing a list of obstacle_boxes dicts to merge in as-is",
    )
    args = parser.parse_args()

    assets_root = Path(args.assets)
    blueprint_path = Path(args.blueprint) if args.blueprint else assets_root.parent / "blueprint.json"
    obstacle_boxes: list[dict] = []
    obstacle_capsules: list[dict] = []
    if args.wall_with_hole:
        cx, cy, cz, width, height, thickness, hole_width, hole_height = args.wall_with_hole
        obstacle_boxes.extend(
            build_wall_with_hole_boxes(
                center_world=(cx, cy, cz),
                width_m=width,
                height_m=height,
                thickness_m=thickness,
                hole_width_m=hole_width,
                hole_height_m=hole_height,
            )
        )
    if args.cylinder_obstacle:
        cx, cy, cz, radius, height = args.cylinder_obstacle
        obstacle_capsules.append(
            build_cylinder_obstacle_capsule(center_world=(cx, cy, cz), radius_m=radius, height_m=height)
        )
    if args.obstacles_json:
        with open(args.obstacles_json, "r", encoding="utf-8") as handle:
            obstacle_boxes.extend(json.load(handle))
    output = build_collision_model(
        blueprint_path=blueprint_path,
        source_root=assets_root.parent,
        output=Path(args.output),
        default_radius=float(args.default_radius),
        go2_body_length_m=args.go2_body_length,
        go2_body_width_m=args.go2_body_width,
        go2_body_height_m=args.go2_body_height,
        go2_urdf_path=Path(args.go2_urdf) if args.go2_urdf else None,
        go2_urdf_link_name=str(args.go2_urdf_link),
        config_path=Path(args.config) if args.config else None,
        discovery_samples=int(args.discovery_samples),
        obstacle_boxes=obstacle_boxes,
        obstacle_capsules=obstacle_capsules,
    )
    print(output)


if __name__ == "__main__":
    sim_bundle_main()
