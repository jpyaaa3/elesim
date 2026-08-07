from __future__ import annotations

from pathlib import Path

import elesim_sim.runtime as runtime
from elesim_sim.config import load_app_config


ROOT = Path(__file__).resolve().parents[1]


def test_viewer_flag_overrides_remote_profile(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(ROOT / "config/config.remote.yaml"),
        argv=["--viewer"],
    )

    remote = load_app_config(str(ROOT / "config/config.remote.yaml"))
    assert remote.sim_config.enable_viewer is False
    assert captured["cfg"].enable_viewer is True


def test_no_viewer_flag_keeps_remote_profile_headless(monkeypatch) -> None:
    captured = {}

    class FakeGenesisApp:
        def __init__(self, **kwargs):
            captured.update(kwargs)

        def run(self) -> None:
            return None

    monkeypatch.setattr(runtime, "GenesisApp", FakeGenesisApp)
    runtime.run_runtime(
        config_path=str(ROOT / "config/config.pc.yaml"),
        argv=["--no-viewer"],
    )

    assert captured["cfg"].enable_viewer is False
