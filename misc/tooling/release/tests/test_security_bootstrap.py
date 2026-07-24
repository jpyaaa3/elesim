from __future__ import annotations

import zipfile
from pathlib import Path

import pytest
import yaml

from misc.infra.bootstrap_security import generate
from misc.tooling.release.build import build_wheel, copy_infrastructure


def test_security_bootstrap_generates_all_deployment_identities(tmp_path: Path) -> None:
    output = tmp_path / "generated"
    coturn_env = tmp_path / "coturn.env"

    generate(
        output,
        coturn_env,
        turn_public_ip="203.0.113.10",
        turn_realm="sim.example.com",
        force=False,
    )

    registry = yaml.safe_load((output / "curve/endpoints.yaml").read_text())
    identities = {
        (entry["endpoint_id"], entry["role"])
        for entry in registry["clients"]
    }
    assert identities == {
        ("controller-main", "controller"),
        ("ui-main", "ui"),
        ("ui-main-simulator", "ui"),
        ("doctor-main", "ui"),
        ("sim-default", "simulator"),
        ("robot-go2", "robot"),
    }
    assert (output / "curve/router/router.key_secret").is_file()
    assert (output / "curve/media/simulator-media.key_secret").is_file()
    assert (output / "curve/media/robot-media.key_secret").is_file()
    assert (output / "curve/media-authorized/controller-main.key").is_file()
    assert len(tuple((output / "curve/media-authorized").glob("*.key"))) == 1
    env = coturn_env.read_text(encoding="utf-8")
    assert "TURN_PUBLIC_IP=203.0.113.10" in env
    assert "TURN_REALM=sim.example.com" in env
    assert "TURN_STATIC_AUTH_SECRET=" in env


@pytest.mark.parametrize("public_ip,realm", [("", "example.com"), ("host", "")])
def test_security_bootstrap_rejects_empty_turn_identity(
    tmp_path: Path,
    public_ip: str,
    realm: str,
) -> None:
    with pytest.raises(ValueError):
        generate(
            tmp_path / "generated",
            tmp_path / "coturn.env",
            turn_public_ip=public_ip,
            turn_realm=realm,
            force=False,
        )


def test_release_infrastructure_contains_bootstrap_and_coturn(tmp_path: Path) -> None:
    source = Path(__file__).resolve().parents[3] / "infra"
    release_root = tmp_path / "releases"

    copy_infrastructure(source, release_root)

    assert (release_root / "infra/bootstrap_security.py").is_file()
    assert (release_root / "infra/coturn/compose.yaml").is_file()
    assert (release_root / "infra/setup/bootstrap.py").is_file()
    assert (release_root / "infra/setup/bootstrap.sh").is_file()
    assert (release_root / "infra/containers/Dockerfile.app").is_file()
    assert (release_root / "infra/development/Dockerfile").is_file()
    assert (release_root / "infra/setup/package/pyproject.toml").is_file()
    assert (
        release_root
        / "infra/setup/package/src/elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf"
    ).is_file()
    assert not (release_root / "infra/setup/package/build").exists()
    assert not tuple((release_root / "infra/setup/package").rglob("*.egg-info"))
    assert not tuple((release_root / "infra/setup/package").rglob("__pycache__"))


def test_setup_wheel_contains_browser_assets_and_cjk_font(tmp_path: Path) -> None:
    project = Path(__file__).resolve().parents[2] / "setup"
    wheel_dir = tmp_path / "wheels"
    wheel_dir.mkdir()

    wheel = build_wheel(project, wheel_dir)

    with zipfile.ZipFile(wheel) as archive:
        members = set(archive.namelist())
    assert "elesim_setup/web/index.html" in members
    assert "elesim_setup/web/app.js" in members
    assert "elesim_setup/web/i18n.json" in members
    assert "elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf" in members
