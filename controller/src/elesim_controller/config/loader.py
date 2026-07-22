"""Controller YAML configuration entry point."""

from __future__ import annotations

import os

from elesim_controller.config.schema import AppConfigBundle
from elesim_controller.config.yaml_loader import load_app_config_from_yaml


def load_app_config(path: str) -> AppConfigBundle:
    if not path:
        raise FileNotFoundError("config path is empty")
    suffix = os.path.splitext(str(path))[1].lower()
    if suffix not in (".yaml", ".yml"):
        raise ValueError(f"controller configuration must be YAML, got {suffix or '<none>'!r}")
    return load_app_config_from_yaml(str(path))
