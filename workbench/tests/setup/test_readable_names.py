import json
from concurrent.futures import ThreadPoolExecutor

import pytest

from elesim_setup.readable_names import (
    ADJECTIVES,
    ANIMALS,
    NAME_PATTERN,
    lookup_image_name,
    lookup_image_names,
    lookup_name,
    random_name,
    reserve_image_name,
    reserve_name,
    reserve_role_release_name,
    mark_release_names_published,
    release_reservation_identity,
    role_release_reservation_identity,
)


def test_readable_name_pool_is_internal_and_large_enough():
    assert len(ADJECTIVES) == len(set(ADJECTIVES)) == 71
    assert len(ANIMALS) == len(set(ANIMALS)) == 123
    assert len(ADJECTIVES) * len(ANIMALS) == 8733


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


def test_image_aliases_are_unique_across_roles(tmp_path):
    path = tmp_path / "names.json"
    choices = iter(("quiet_otter", "quiet_otter", "bright_fox"))
    pilot = reserve_image_name(
        path,
        "pilot",
        "a" * 64,
        generate=lambda: next(choices),
    )
    sim = reserve_image_name(
        path,
        "sim",
        "b" * 64,
        generate=lambda: next(choices),
    )
    assert pilot == "quiet_otter"
    assert sim == "bright_fox"
    assert reserve_image_name(
        path,
        "pilot",
        "a" * 64,
        generate=lambda: pytest.fail("must reuse"),
    ) == pilot
    assert lookup_image_name(path, "pilot", "a" * 64) == pilot
    assert lookup_image_names(path, "sim", "b" * 64) == (sim,)


def test_colliding_legacy_aliases_get_new_bindings_without_rewriting_history(tmp_path):
    path = tmp_path / "names.json"
    pilot_fingerprint = "a" * 64
    sim_fingerprint = "b" * 64
    reserve_name(
        path,
        "pilot",
        pilot_fingerprint,
        generate=lambda: "quiet_otter",
    )
    reserve_name(
        path,
        "sim",
        sim_fingerprint,
        generate=lambda: "quiet_otter",
    )
    before = json.loads(path.read_text())

    pilot = reserve_image_name(
        path,
        "pilot",
        pilot_fingerprint,
        generate=lambda: "bright_fox",
    )
    sim = reserve_image_name(
        path,
        "sim",
        sim_fingerprint,
        generate=lambda: "golden_eagle",
    )
    assert pilot == "bright_fox"
    assert sim == "golden_eagle"
    after = json.loads(path.read_text())
    assert after["names"]["pilot"][pilot_fingerprint] == before["names"]["pilot"][pilot_fingerprint]
    assert after["names"]["sim"][sim_fingerprint] == before["names"]["sim"][sim_fingerprint]
    assert after["names"]["images"] == {
        f"pilot:{pilot_fingerprint}": "bright_fox",
        f"sim:{sim_fingerprint}": "golden_eagle",
    }


def test_unique_legacy_alias_is_promoted_without_a_tag_change(tmp_path):
    path = tmp_path / "names.json"
    fingerprint = "a" * 64
    reserve_name(path, "pilot", fingerprint, generate=lambda: "quiet_otter")
    assert reserve_image_name(path, "pilot", fingerprint) == "quiet_otter"
    assert lookup_image_names(path, "pilot", fingerprint) == ("quiet_otter",)
    names = json.loads(path.read_text())["names"]
    assert names["images"][f"pilot:{fingerprint}"] == "quiet_otter"


def test_role_release_aliases_are_unique_and_input_bound(tmp_path):
    path = tmp_path / "names.json"
    pilot_fingerprint = "a" * 64
    sim_fingerprint = "b" * 64
    digest = "c" * 64
    choices = iter(("golden_snail", "silver_pigeon", "bright_fox"))
    pilot = reserve_role_release_name(
        path,
        "git-" + "1" * 40,
        "pilot",
        pilot_fingerprint,
        digest,
        generate=lambda: next(choices),
    )
    sim = reserve_role_release_name(
        path,
        "git-" + "1" * 40,
        "sim",
        sim_fingerprint,
        digest,
        generate=lambda: next(choices),
    )
    assert pilot == "golden_snail"
    assert sim == "silver_pigeon"
    assert pilot != sim
    assert reserve_role_release_name(
        path,
        "git-" + "1" * 40,
        "pilot",
        pilot_fingerprint,
        digest,
        generate=lambda: pytest.fail("must reuse the exact role input"),
    ) == pilot
    assert reserve_role_release_name(
        path,
        "git-" + "2" * 40,
        "pilot",
        pilot_fingerprint,
        digest,
        generate=lambda: next(choices),
    ) == "bright_fox"
    registry = json.loads(path.read_text())
    assert registry["names"]["releases"][
        role_release_reservation_identity(
            "git-" + "1" * 40, "pilot", pilot_fingerprint, digest
        ) + ":0"
    ] == "golden_snail"
    assert registry["names"]["releases"][
        role_release_reservation_identity(
            "git-" + "1" * 40, "sim", sim_fingerprint, digest
        ) + ":0"
    ] == "silver_pigeon"


def test_legacy_release_alias_reservation_remains_readable(tmp_path):
    path = tmp_path / "names.json"
    fingerprints = {"pilot": "a" * 64, "sim": "b" * 64}
    digest = "c" * 64
    alias = reserve_name(
        path,
        "releases",
        release_reservation_identity("git-" + "1" * 40, tuple(fingerprints), fingerprints, digest),
        generate=lambda: "golden_snail",
    )
    assert alias == "golden_snail"
    assert lookup_name(
        path,
        "releases",
        release_reservation_identity(
            "git-" + "1" * 40, tuple(fingerprints), fingerprints, digest
        ),
    ) == alias


def test_completed_update_advances_but_failed_retry_reuses(tmp_path):
    path = tmp_path / "names.json"
    args = (path, "git-" + "1" * 40, "sim", "a" * 64, "b" * 64)
    first = reserve_role_release_name(*args)
    assert reserve_role_release_name(*args) == first
    mark_release_names_published(path, (first,))
    second = reserve_role_release_name(*args)
    assert second != first
    assert reserve_role_release_name(*args) == second
    mark_release_names_published(path, (first,))
    assert reserve_role_release_name(*args) == second
    mark_release_names_published(path, (second,))
    assert reserve_role_release_name(*args) not in (first, second)
