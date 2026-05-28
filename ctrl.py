#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import os

from engine import ik as ik_pipeline
from engine.controller import (
    ControlClient,
    ControlService,
    PanelState,
)
from ui.control_panel import ControlPanel


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config.ini"),
        help="path to ini config file",
    )
    args = ap.parse_args()

    bundle, ik_context = ik_pipeline.load_solver_context(args.config)
    link = ControlClient(str(bundle.sim_config.host_ctrl_port), cfg=bundle.mapping_config)
    state = PanelState(
        sag_model_path="",
        raw_sag_model=None,
    )
    service = ControlService(
        state,
        client=link,
        mapping_cfg=bundle.mapping_config,
        ik_cfg=bundle.ik_config,
        ik_context=ik_context,
        config_path=args.config,
    )
    gui = ControlPanel(
        state,
        service,
        use_hardware=bool(bundle.sim_config.use_hardware),
    )
    try:
        service.refresh_host_state()
        service.send_current_target_meta(source="target")
        gui.run()
    finally:
        service.close()


if __name__ == "__main__":
    main()
