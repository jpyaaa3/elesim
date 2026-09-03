from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.container_installer import refresh_compose_dds_environment
from elesim_setup.network import _snapshot


def test_compose_refresh_rejects_broken_symlink(local_state) -> None:
    state = local_state(roles=("pilot",))
    compose = state.prefix_path / "containers/compose.yaml"
    compose.parent.mkdir(parents=True)
    compose.symlink_to(compose.parent / "missing.yaml")

    with pytest.raises(ValueError, match="regular file"):
        refresh_compose_dds_environment(state)


def test_configuration_snapshot_rejects_broken_symlink(tmp_path: Path) -> None:
    linked = tmp_path / "config.yaml"
    linked.symlink_to(tmp_path / "missing.yaml")

    with pytest.raises(ValueError, match="일반 파일"):
        _snapshot(linked)
