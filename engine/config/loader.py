"""Format-neutral application configuration entry point."""

from __future__ import annotations

import os
import warnings

from engine.config.schema import AppConfigBundle
from engine.config.yaml_loader import load_app_config_from_yaml


def load_app_config(path: str) -> AppConfigBundle:
    if not path:
        raise FileNotFoundError("config path is empty")
    suffix = os.path.splitext(str(path))[1].lower()
    if suffix in (".yaml", ".yml"):
        return load_app_config_from_yaml(str(path))
    if suffix == ".ini":
        from engine.config.legacy_ini import load_app_config_from_ini

        warnings.warn(
            "INI configuration is deprecated; migrate to YAML",
            DeprecationWarning,
            stacklevel=2,
        )
        return load_app_config_from_ini(str(path))
    raise ValueError(f"unsupported config format {suffix or '<none>'!r}: {path}")


def load_app_config_from_ini(path: str) -> AppConfigBundle:
    """Deprecated compatibility API. New callers must use load_app_config()."""
    from engine.config.legacy_ini import load_app_config_from_ini as load_legacy

    warnings.warn(
        "load_app_config_from_ini() is deprecated; use load_app_config()",
        DeprecationWarning,
        stacklevel=2,
    )
    return load_legacy(path)
