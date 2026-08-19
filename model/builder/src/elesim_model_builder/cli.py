from __future__ import annotations

import argparse
from pathlib import Path

from elesim_model_builder.arm_model import build_arm_model
from elesim_model_builder.bundle import build_sim_bundle


def sim_bundle_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--assets", default="model/bundles/default/assets")
    parser.add_argument("--output", default="model/bundles/default")
    parser.add_argument("--use-go2", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-hardware", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--mount", nargs=3, type=float, default=(0.35, 0.0, 0.08))
    args = parser.parse_args()
    output = build_sim_bundle(
        asset_root=Path(args.assets),
        output_dir=Path(args.output),
        use_hardware=bool(args.use_hardware),
        use_go2=bool(args.use_go2),
        mount_xyz=tuple(args.mount),
    )
    print(output)


def arm_model_main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="pilot/config/config.yaml")
    parser.add_argument("--assets", default="model/bundles/default/assets")
    parser.add_argument("--output", default="pilot/config/arm_model.json")
    args = parser.parse_args()
    output = build_arm_model(
        config=Path(args.config),
        assets=Path(args.assets),
        output=Path(args.output),
    )
    print(output)


if __name__ == "__main__":
    sim_bundle_main()
