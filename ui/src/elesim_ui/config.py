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
    router_client_secret_file: str
    router_server_public_file: str
    allow_insecure_remote: bool
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
    source = Path(path)
    raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, dict) or raw.get("schema_version") != 2:
        raise ValueError(f"{source}: schema_version must be 2")
    runtime = _section(raw, "runtime")
    security = _section(raw, "security")
    presentation = _section(raw, "presentation")
    go2 = _section(presentation, "go2")
    def resolved(value: object) -> str:
        text = str(value or "").strip()
        if not text:
            return ""
        candidate = Path(text).expanduser()
        return str(candidate if candidate.is_absolute() else (source.parent / candidate).resolve())

    return UiConfig(
        endpoint_id=str(runtime.get("endpoint_id", "ui-main")),
        controller_id=str(runtime.get("controller_id", "controller-main")),
        simulator_id=str(runtime.get("simulator_id", "sim-default")),
        server_endpoint=str(runtime.get("server_endpoint", "tcp://127.0.0.1:5558")),
        router_client_secret_file=resolved(security.get("router_client_secret_file")),
        router_server_public_file=resolved(security.get("router_server_public_file")),
        allow_insecure_remote=bool(security.get("allow_insecure_remote", False)),
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
