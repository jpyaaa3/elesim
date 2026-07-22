from __future__ import annotations

import tempfile
from pathlib import Path

from misc.tooling.release.build import copy_simulator_bundle


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
