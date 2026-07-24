from __future__ import annotations

import tempfile
from pathlib import Path

from misc.tooling.release.build import copy_interfaces, copy_simulator_bundle


def test_simulator_release_contains_only_the_immutable_bundle() -> None:
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        source = root / "model"
        bundle = source / "bundles/default"
        bundle.mkdir(parents=True)
        (bundle / "bundle.json").write_text("{}", encoding="utf-8")
        source_assets = source / "source/assets"
        source_assets.mkdir(parents=True)
        (source_assets / "must-not-ship.obj").write_text("source", encoding="utf-8")
        release = root / "release"

        copy_simulator_bundle(source, release)

        assert (release / "model/bundles/default/bundle.json").is_file()
        assert not (release / "misc/model/source").exists()


def test_release_copies_ros_interface_source_without_colcon_artifacts(
    tmp_path: Path,
) -> None:
    source = tmp_path / "elesim_interfaces"
    for relative in ("package.xml", "CMakeLists.txt", "msg/RgbdFrame.msg"):
        path = source / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative, encoding="utf-8")
    (source / "build").mkdir()
    (source / "build/generated").write_text("no", encoding="utf-8")
    release = tmp_path / "release"

    copy_interfaces(source, release)

    copied = release / "interfaces/elesim_interfaces"
    assert (copied / "package.xml").is_file()
    assert (copied / "msg/RgbdFrame.msg").is_file()
    assert not (copied / "build").exists()
