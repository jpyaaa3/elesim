from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from elesim_setup.state import (
    ComputeSettings,
    DdsSettings,
    InstallState,
    NetworkSettings,
    TurnSettings,
)


ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture
def local_state(tmp_path: Path):
    def factory(
        *,
        roles=("simulator",),
        profile="custom",
        dds=None,
        network=None,
        compute=None,
        turn=None,
        install_mode=None,
    ) -> InstallState:
        selected_roles = tuple(roles)
        selected_mode = (
            ("native" if selected_roles == ("robot",) else "container")
            if install_mode is None
            else install_mode
        )
        return InstallState(
            profile=profile,
            roles=selected_roles,
            prefix=str(tmp_path / "install"),
            bin_dir=str(tmp_path / "bin"),
            source_root=str(ROOT),
            network=network or NetworkSettings(),
            dds=dds or DdsSettings(),
            compute=compute or ComputeSettings(),
            turn=turn or TurnSettings(),
            install_mode=selected_mode,
        )

    return factory


def copy_role_configs(state: InstallState) -> None:
    for role in state.roles:
        source = ROOT / role / "config"
        destination = state.prefix_path / "roles" / role / "config"
        shutil.copytree(source, destination, dirs_exist_ok=True)
