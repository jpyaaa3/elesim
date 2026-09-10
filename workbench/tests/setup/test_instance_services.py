from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.container_installer import ContainerInstaller


def test_instance_role_service_uses_explicit_private_roots(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    installer = ContainerInstaller(state)
    installer.run()
    config = tmp_path / "instance-config"
    cache = tmp_path / "instance-cache"
    data = tmp_path / "instance-data"
    service = installer._role_service(
        "sim",
        config_root=config,
        cache_root=cache,
        data_root=data,
        instance_scoped=True,
    )
    volumes = service["volumes"]
    assert f"{config}:/opt/elesim/config:ro" in volumes
    assert f"{cache}:/tmp/elesim-cache:rw" in volumes
    assert f"{data}:/opt/elesim/data:ro" in volumes
    assert not any(str(state.prefix_path / "security") in value for value in volumes)
    assert not any(str(state.prefix_path / "apps/sim/config") in value for value in volumes)
    assert service["ipc"] == "private"


def test_instance_role_service_rejects_legacy_or_unsafe_roots(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("pilot",), install_mode="container")
    installer = ContainerInstaller(state)
    installer.run()
    roots = (tmp_path / "config", tmp_path / "cache", tmp_path / "data")
    with pytest.raises(ValueError, match="legacy"):
        installer._role_service(
            "pilot",
            config_root=state.prefix_path / "apps/pilot/config",
            cache_root=roots[1],
            data_root=roots[2],
            instance_scoped=True,
        )
    link = tmp_path / "link"
    link.symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        installer._role_service(
            "pilot",
            config_root=link / "config",
            cache_root=roots[1],
            data_root=roots[2],
            instance_scoped=True,
        )


def test_instance_role_service_rejects_cross_system_roots(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    installer = ContainerInstaller(state)
    installer.run()
    with pytest.raises(ValueError, match="different systems"):
        installer._role_service(
            "sim",
            config_root=tmp_path / "instances/alpha/endpoints/sim/config",
            cache_root=tmp_path / "instances/beta/endpoints/sim/cache",
            data_root=tmp_path / "release/data",
            instance_scoped=True,
        )
