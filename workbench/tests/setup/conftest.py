from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from elesim_setup.state import (
    ComputeSettings,
    ContainerNetworkSettings,
    DeveloperAttachmentSettings,
    DdsSettings,
    InstallState,
    NetworkSettings,
    TurnSettings,
)


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def local_state(tmp_path: Path):
    def factory(
        *,
        roles=("sim",),
        profile="custom",
        dds=None,
        network=None,
        compute=None,
        turn=None,
        container_network=None,
        developer_attachment=None,
        install_mode=None,
        source_repository="jpyaaa3/elesim",
        source_ref="main",
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
            source_repository=source_repository,
            source_ref=source_ref,
            network=network or NetworkSettings(),
            dds=dds or DdsSettings(),
            compute=compute or ComputeSettings(),
            turn=turn or TurnSettings(),
            container_network=container_network or ContainerNetworkSettings(),
            developer_attachment=(
                developer_attachment or DeveloperAttachmentSettings()
            ),
            install_mode=selected_mode,
        )

    return factory


def copy_role_configs(state: InstallState) -> None:
    for role in state.roles:
        source = ROOT / "payload/config" / role
        destination = state.prefix_path / "apps" / role / "config"
        shutil.copytree(source, destination, dirs_exist_ok=True)
