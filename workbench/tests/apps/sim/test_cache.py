from __future__ import annotations

import os
import shutil
from pathlib import Path

import pytest

import elesim_sim.cache as cache


def test_configure_numba_cache_falls_back_from_stale_mount(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    # A regular file at the requested XDG path models the unusable/root-owned
    # bind mount without requiring privileged ownership changes in the test.
    requested_root = tmp_path / "legacy-cache"
    requested_root.write_text("stale", encoding="utf-8")
    fallback_root = tmp_path / "private-cache"

    monkeypatch.setenv("XDG_CACHE_HOME", str(requested_root))
    monkeypatch.setenv("NUMBA_CACHE_DIR", str(requested_root / "numba"))
    monkeypatch.setattr(
        cache,
        "_new_private_cache_root",
        lambda _requested=None: fallback_root,
    )

    cache._configure_numba_cache_dir()

    assert Path(os.environ["XDG_CACHE_HOME"]) == fallback_root
    assert Path(os.environ["NUMBA_CACHE_DIR"]) == fallback_root / "numba"
    assert (fallback_root / "numba").is_dir()
    assert "using private cache" in capsys.readouterr().err


def test_private_fallback_is_stable_across_restarts(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setenv("ELESIM_CACHE_NAMESPACE", str(tmp_path))
    first = cache._new_private_cache_root(tmp_path / "legacy")
    second = cache._new_private_cache_root(tmp_path / "legacy")
    try:
        assert first == second
        assert first.is_dir()
        assert first.stat().st_mode & 0o777 == 0o700
    finally:
        shutil.rmtree(first, ignore_errors=True)


def test_cache_probe_rejects_symlink(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(PermissionError, match="symlink"):
        cache._prepare_cache_dir(link, private=True)


def test_cache_probe_rejects_symlink_ancestor(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "link"
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(PermissionError, match="symlink ancestor"):
        cache._prepare_cache_dir(link / "numba", private=True)
