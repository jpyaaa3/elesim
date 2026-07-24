from __future__ import annotations

import os
import subprocess
import sys
import zipfile
from pathlib import Path

from misc.tooling.release.build import build_wheel, copy_infrastructure


def test_release_infrastructure_contains_dds_aware_installers(tmp_path: Path) -> None:
    source = Path(__file__).resolve().parents[3] / "infra"
    release_root = tmp_path / "releases"

    copy_infrastructure(source, release_root)

    assert not (release_root / "infra/bootstrap_security.py").exists()
    assert not (release_root / "infra/coturn").exists()
    assert (release_root / "infra/setup/bootstrap.py").is_file()
    assert (release_root / "infra/setup/bootstrap.sh").is_file()
    assert (release_root / "infra/setup/bootstrap-contract.json").is_file()
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
    root = Path(__file__).resolve().parents[3]
    release_root = tmp_path / "releases"
    copy_infrastructure(root / "infra", release_root)
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
    assert "elesim_setup/web/fonts/NotoSansCJKkr-Regular.otf" in members
    assert "elesim-setup = elesim_setup.cli:main" in entry_points

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(wheel), str(root.parent / "packages/protocol/src"))
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
