from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from elesim_setup.state import (
    ComputeSettings,
    InstallState,
    NetworkSettings,
    SecuritySettings,
    TurnSettings,
)


ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture
def local_state(tmp_path: Path):
    def factory(
        *,
        roles=("router",),
        profile="custom",
        security=None,
        network=None,
        compute=None,
        turn=None,
        install_mode="native",
    ) -> InstallState:
        return InstallState(
            profile=profile,
            roles=tuple(roles),
            prefix=str(tmp_path / "install"),
            bin_dir=str(tmp_path / "bin"),
            source_root=str(ROOT),
            network=network or NetworkSettings(),
            security=security or SecuritySettings(),
            compute=compute or ComputeSettings(),
            turn=turn or TurnSettings(),
            install_mode=install_mode,
        )

    return factory


def copy_role_configs(state: InstallState) -> None:
    for role in state.roles:
        source = ROOT / role / "config"
        destination = state.prefix_path / "roles" / role / "config"
        shutil.copytree(source, destination, dirs_exist_ok=True)
