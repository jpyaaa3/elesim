from __future__ import annotations

import os
import subprocess
import sys
import zipfile
from pathlib import Path

from misc.tools.release.build import build_wheel, copy_infrastructure


def test_release_infrastructure_contains_dds_aware_installers(tmp_path: Path) -> None:
    source = Path(__file__).resolve().parents[4] / "environment"
    release_root = tmp_path / "releases"
    stale_package = release_root / "infra/setup/package"
    stale_package.mkdir(parents=True)
    (stale_package / "requirements-media.lock").write_text("stale\n", encoding="utf-8")
    (stale_package / "tests").mkdir()

    copy_infrastructure(source, release_root)

    assert not (release_root / "infra/bootstrap_security.py").exists()
    assert not (release_root / "infra/coturn").exists()
    assert (release_root / "infra/setup/bootstrap.py").is_file()
    assert (release_root / "infra/setup/install.sh").is_file()
    assert (release_root / "infra/setup/bootstrap-contract.json").is_file()
    assert (release_root / "infra/containers/Dockerfile.app").is_file()
    assert (release_root / "infra/development/Dockerfile").is_file()
    assert (release_root / "infra/setup/package/pyproject.toml").is_file()
    assert (
        release_root
        / "infra/setup/package/elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf"
    ).is_file()
    package = release_root / "infra/setup/package"
    assert {path.name for path in package.iterdir()} == {
        "pyproject.toml",
        "requirements.lock",
        "elesim_setup",
    }
    assert not (package / "tests").exists()
    assert not (package / "requirements-media.lock").exists()
    assert not tuple(package.rglob("*.egg-info"))
    assert not tuple(package.rglob("__pycache__"))


def test_setup_wheel_contains_browser_assets_and_cjk_font(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[4]
    release_root = tmp_path / "releases"
    copy_infrastructure(root / "environment", release_root)
    project = release_root / "infra/setup/package"
    wheel_dir = tmp_path / "wheels"
    wheel_dir.mkdir()

    wheel = build_wheel(project, wheel_dir)

    with zipfile.ZipFile(wheel) as archive:
        members = set(archive.namelist())
        entry_points = archive.read(
            next(name for name in members if name.endswith(".dist-info/entry_points.txt"))
        ).decode("utf-8")
    assert "elesim_setup/web/index.html" in members
    assert "elesim_setup/web/app.js" in members
    assert "elesim_setup/web/i18n.json" in members
    assert "elesim_setup/web/icon.svg" in members
    assert "elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf" in members
    assert "elesim_setup/connection_web/index.html" in members
    assert "elesim_setup/connection_web/app.js" in members
    assert "elesim_setup/connection_web/i18n.json" in members
    assert "elesim_setup/connection_web/icon.svg" in members
    assert "elesim_setup/ownership.py" in members
    assert "elesim_setup/uninstall.py" in members
    assert "elesim_setup/shell.py" in members
    assert "elesim-setup = elesim_setup.cli:main" in entry_points
    assert "elesim-connections = elesim_setup.connections:main" in entry_points
    assert "elesim-uninstall = elesim_setup.uninstall:main" in entry_points

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(wheel), str(root / "payload/runtime/common/protocol"))
    )
    completed = subprocess.run(
        (sys.executable, "-m", "elesim_setup.cli", "gui", "--help"),
        cwd=tmp_path,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=30,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout
    assert "gui" in completed.stdout
