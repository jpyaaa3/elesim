#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from typing import Optional

from engine.config_loader import load_app_config_from_ini
from engine.controller.client import ControlClient
from engine.controller.perception_capture import PerceptionCapture, PerceptionSnapshot


def main() -> None:
    ap = argparse.ArgumentParser(description="Jetson perception worker (RealSense → host relay)")
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    args = ap.parse_args()

    bundle = load_app_config_from_ini(str(args.config))
    mode = str(bundle.perception_config.mode).strip().lower()
    if mode == "sim":
        print(
            "[perception_worker] mode=sim requires sim.py on the same machine; "
            "use mode=camera on Jetson",
            file=sys.stderr,
        )
        raise SystemExit(1)
    if mode == "mock":
        print("[perception_worker] mode=mock is for dev only; use mode=camera on Jetson", file=sys.stderr)
        raise SystemExit(2)

    endpoint = str(bundle.sim_config.host_ctrl_port).strip()
    client = ControlClient(endpoint, cfg=bundle.mapping_config)
    pick_cfg = bundle.pick_config
    cap: Optional[PerceptionCapture] = None
    stop = False

    def _on_snapshot(snap: PerceptionSnapshot) -> None:
        if snap.failed:
            print(f"[perception_worker] failed: {snap.status_msg}")
        elif int(snap.frame_idx) % 60 == 0 and snap.label:
            print(
                "[perception_worker] frame=%d label=%s conf=%.2f uv=%s"
                % (
                    int(snap.frame_idx),
                    str(snap.label),
                    float(snap.confidence),
                    str(snap.center_uv),
                )
            )

    def _publish(**kwargs: object) -> Optional[tuple[float, float, float]]:
        return client.send_perception_observation(
            object_camera_xyz=kwargs["object_camera_xyz"],  # type: ignore[arg-type]
            label=str(kwargs.get("label", "")),
            confidence=float(kwargs.get("confidence", 0.0)),
            image_center_uv=kwargs["image_center_uv"],  # type: ignore[arg-type]
            image_scale=float(kwargs.get("image_scale", 0.0)),
            depth_valid=bool(kwargs.get("depth_valid", True)),
            object_world=kwargs.get("object_world"),  # type: ignore[arg-type]
            camera_world_origin=kwargs.get("camera_world_origin"),  # type: ignore[arg-type]
            camera_world_look=kwargs.get("camera_world_look"),  # type: ignore[arg-type]
            camera_world_right=kwargs.get("camera_world_right"),  # type: ignore[arg-type]
        )

    def _shutdown(*_args: object) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    cap = PerceptionCapture(
        bundle.perception_config,
        publish_fn=_publish,
        on_snapshot=_on_snapshot,
        target_uv_fn=lambda: (float(pick_cfg.target_uv_u), float(pick_cfg.target_uv_v)),
    )
    print(f"[perception_worker] connecting host at {endpoint} mode={mode}")
    cap.start()
    try:
        while not stop:
            time.sleep(0.25)
    finally:
        if cap is not None:
            cap.stop(timeout_s=10.0)
        client.close()
        print("[perception_worker] stopped")


if __name__ == "__main__":
    main()
