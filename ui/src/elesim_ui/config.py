from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from elesim_protocol import DdsRuntimeSettings
from elesim_ui.models import GazeStabilizerConfig, HardwareConfig, PerceptionConfig, PickConfig


@dataclass(frozen=True)
class UiConfig:
    endpoint_id: str
    controller_id: str
    simulator_id: str
    dds: DdsRuntimeSettings
    use_hardware: bool
    use_go2: bool
    go2_vx: float
    go2_vy: float
    go2_wz: float
    hardware: HardwareConfig
    perception: PerceptionConfig
    pick: PickConfig
    gaze: GazeStabilizerConfig


def _section(raw: Mapping[str, Any], name: str) -> dict[str, Any]:
    value = raw.get(name, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return dict(value)


def _resolved(source: Path, value: object) -> str:
    text = str(value or "").strip()
    if not text:
        return ""
    candidate = Path(text).expanduser()
    return str(candidate if candidate.is_absolute() else (source.parent / candidate).resolve())


def load_config(path: str | Path) -> UiConfig:
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ValueError(f"{source}: schema_version must be 2")
    runtime = _section(raw, "runtime")
    presentation = _section(raw, "presentation")
    go2 = _section(presentation, "go2")
    endpoint_id = str(runtime.get("endpoint_id", "ui-main")).strip()
    if not endpoint_id:
        raise ValueError(f"{source}: runtime.endpoint_id is required")
    dds_raw = _section(raw, "dds")
    for key in ("vendor_config", "keystore"):
        if str(dds_raw.get(key, "")).strip():
            dds_raw[key] = _resolved(source, dds_raw[key])

    return UiConfig(
        endpoint_id=endpoint_id,
        controller_id=str(runtime.get("controller_id", "controller-main")),
        simulator_id=str(runtime.get("simulator_id", "sim-default")),
        dds=DdsRuntimeSettings.from_mapping(dds_raw, endpoint_id=endpoint_id),
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
