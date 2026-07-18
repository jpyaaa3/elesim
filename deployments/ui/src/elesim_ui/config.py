from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from elesim_ui.models import GazeStabilizerConfig, HardwareConfig, PerceptionConfig, PickConfig


@dataclass(frozen=True)
class UiConfig:
    endpoint_id: str
    controller_id: str
    simulator_id: str
    server_endpoint: str
    use_hardware: bool
    use_go2: bool
    go2_vx: float
    go2_vy: float
    go2_wz: float
    hardware: HardwareConfig
    perception: PerceptionConfig
    pick: PickConfig
    gaze: GazeStabilizerConfig


def _section(raw: dict, name: str) -> dict:
    value = raw.get(name, {})
    return value if isinstance(value, dict) else {}


def load_config(path: str | Path) -> UiConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    runtime = _section(raw, "runtime")
    presentation = _section(raw, "presentation")
    go2 = _section(presentation, "go2")
    return UiConfig(
        endpoint_id=str(runtime.get("endpoint_id", "ui-main")),
        controller_id=str(runtime.get("controller_id", "controller-main")),
        simulator_id=str(runtime.get("simulator_id", "sim-default")),
        server_endpoint=str(runtime.get("server_endpoint", "tcp://127.0.0.1:5558")),
        use_hardware=bool(presentation.get("use_hardware", False)),
        use_go2=bool(presentation.get("use_go2", True)),
        go2_vx=float(go2.get("vx_mps", 0.35)),
        go2_vy=float(go2.get("vy_mps", 0.25)),
        go2_wz=float(go2.get("wz_radps", 0.8)),
        hardware=HardwareConfig(**_section(raw, "hardware")),
        perception=PerceptionConfig(**_section(raw, "perception")),
        pick=PickConfig(**_section(raw, "pick")),
        gaze=GazeStabilizerConfig(**_section(raw, "gaze")),
    )
