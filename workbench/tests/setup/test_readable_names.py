import json
from concurrent.futures import ThreadPoolExecutor

import pytest

from elesim_setup.readable_names import NAME_PATTERN, random_name, reserve_name


def test_names_are_short_and_readable():
    for _ in range(100):
        assert NAME_PATTERN.fullmatch(random_name())


def test_reuses_identity_and_retries_collisions(tmp_path):
    path = tmp_path / "names.json"
    assert reserve_name(path, "images", "hash1", generate=lambda: "quiet_otter") == "quiet_otter"
    assert reserve_name(path, "images", "hash1", generate=lambda: pytest.fail("must reuse")) == "quiet_otter"
    choices = iter(("quiet_otter", "silver_pigeon", "bright_fox"))
    assert reserve_name(path, "images", "hash2", unavailable=("silver_pigeon",), generate=lambda: next(choices)) == "bright_fox"
    assert path.stat().st_mode & 0o777 == 0o600


def test_concurrent_reservations_share_one_binding(tmp_path):
    path = tmp_path / "names.json"
    with ThreadPoolExecutor(max_workers=8) as pool:
        names = list(pool.map(lambda _: reserve_name(path, "images", "same"), range(24)))
    assert len(set(names)) == 1
    assert len(json.loads(path.read_text())["names"]["images"]) == 1


def test_symlinks_and_corruption_fail_closed(tmp_path):
    target = tmp_path / "target"
    target.write_text("not json")
    path = tmp_path / "names.json"
    path.symlink_to(target)
    with pytest.raises(ValueError, match="symlink"):
        reserve_name(path, "images", "hash")
    with pytest.raises(ValueError):
        reserve_name(target, "images", "hash")
    assert target.read_text() == "not json"


def test_exhaustion_does_not_reassign_existing_names(tmp_path):
    path = tmp_path / "names.json"
    reserve_name(path, "images", "first", generate=lambda: "quiet_otter")
    before = path.read_bytes()
    with pytest.raises(ValueError, match="unused"):
        reserve_name(path, "images", "second", generate=lambda: "quiet_otter")
    assert path.read_bytes() == before
