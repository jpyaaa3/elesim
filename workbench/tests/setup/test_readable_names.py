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
    random_install_name,
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
    assert random_install_name() in ADJECTIVES
    assert random_name() in ANIMALS


def test_single_word_collisions_receive_numbers(tmp_path):
    path = tmp_path / "names.json"
    assert reserve_name(path, "installs", "first", generate=lambda: "quick") == "quick"
    assert reserve_name(path, "installs", "second", generate=lambda: "quick") == "quick2"
    assert reserve_image_name(path, "sim", "a" * 64, generate=lambda: "lion") == "lion"
    assert reserve_image_name(path, "pilot", "b" * 64, generate=lambda: "lion") == "lion2"
    assert reserve_image_name(path, "ui", "c" * 64, generate=lambda: "lion") == "lion3"


def test_default_image_names_use_free_words_before_numbering(tmp_path, monkeypatch):
    monkeypatch.setattr("elesim_setup.readable_names.secrets.choice", lambda words: words[0])
    path = tmp_path / "names.json"
    assert reserve_image_name(path, "sim", "a" * 64) == ANIMALS[0]
    assert reserve_image_name(path, "pilot", "b" * 64) == ANIMALS[1]
    assert reserve_role_release_name(path, "git-" + "1" * 40, "ui", "c" * 64, "d" * 64) == ANIMALS[2]


def test_reclaimed_default_image_name_becomes_eligible_again(tmp_path, monkeypatch):
    from elesim_setup.readable_names import reclaim_unused_image_names

    monkeypatch.setattr("elesim_setup.readable_names.secrets.choice", lambda words: words[0])
    path = tmp_path / "names.json"
    old = reserve_role_release_name(path, "git-" + "1" * 40, "sim", "a" * 64, "d" * 64)
    live = reserve_role_release_name(path, "git-" + "2" * 40, "pilot", "b" * 64, "d" * 64)
    assert (old, live) == ANIMALS[:2]
    assert reclaim_unused_image_names(path, (live,)) == (old,)
    assert reserve_role_release_name(path, "git-" + "3" * 40, "ui", "c" * 64, "d" * 64) == old


def test_default_image_name_is_numbered_only_after_all_animals_are_used(tmp_path, monkeypatch):
    monkeypatch.setattr("elesim_setup.readable_names.secrets.choice", lambda words: words[0])
    path = tmp_path / "names.json"
    for index, animal in enumerate(ANIMALS):
        assert reserve_name(path, "images", f"existing:{index}", generate=lambda value=animal: value) == animal
    assert reserve_image_name(path, "sim", "a" * 64) == f"{ANIMALS[0]}2"


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


def test_completed_update_reuses_the_published_alias(tmp_path):
    path = tmp_path / "names.json"
    args = (path, "git-" + "1" * 40, "sim", "a" * 64, "b" * 64)
    first = reserve_role_release_name(*args, generate=lambda: "calm_eagle")
    assert reserve_role_release_name(*args) == first
    mark_release_names_published(path, (first,))
    assert reserve_role_release_name(
        *args, generate=lambda: pytest.fail("published inputs must keep their alias")
    ) == first


def test_reclaimed_release_alias_is_reusable_without_changing_other_scopes(tmp_path):
    from elesim_setup.readable_names import reclaim_unused_image_names

    path = tmp_path / "names.json"
    old = (path, "git-" + "1" * 40, "sim", "a" * 64, "b" * 64)
    live = (path, "git-" + "2" * 40, "sim", "c" * 64, "b" * 64)
    assert reserve_role_release_name(*old, generate=lambda: "amber_falcon") == "amber_falcon"
    assert reserve_role_release_name(*live, generate=lambda: "silver_fox") == "silver_fox"
    mark_release_names_published(path, ("amber_falcon", "silver_fox"))
    assert reclaim_unused_image_names(path, ("silver_fox",)) == ("amber_falcon",)
    names = json.loads(path.read_text())["names"]
    assert "amber_falcon" not in names["releases"].values()
    assert "amber_falcon" not in names["published"].values()
    assert "silver_fox" in names["releases"].values()
    assert reclaim_unused_image_names(path, ("silver_fox",)) == ()
    assert reserve_role_release_name(
        path, "git-" + "3" * 40, "sim", "d" * 64, "b" * 64,
        generate=lambda: "amber_falcon",
    ) == "amber_falcon"


def test_image_scope_reservation_is_reclaimed_only_without_a_live_tag(tmp_path):
    from elesim_setup.readable_names import reclaim_unused_image_names

    path = tmp_path / "names.json"
    old = reserve_image_name(path, "sim", "a" * 64, generate=lambda: "calm_otter")
    live = reserve_image_name(path, "ui", "b" * 64, generate=lambda: "silver_owl")
    assert reclaim_unused_image_names(path, (live,)) == (old,)
    assert lookup_image_name(path, "sim", "a" * 64) == ""
    assert lookup_image_name(path, "ui", "b" * 64) == live


def test_development_image_aliases_are_not_auto_reclaimed(tmp_path):
    from elesim_setup.readable_names import reclaim_unused_image_names

    path = tmp_path / "names.json"
    current = reserve_image_name(path, "dev", "a" * 64, generate=lambda: "wolf")
    previous = reserve_name(path, "dev", "b" * 64, generate=lambda: "wolf2")
    old_role = reserve_image_name(path, "sim", "c" * 64, generate=lambda: "otter")

    assert reclaim_unused_image_names(path, ()) == (old_role,)
    assert lookup_image_name(path, "dev", "a" * 64) == current
    assert lookup_name(path, "dev", "b" * 64) == previous
    assert lookup_image_name(path, "sim", "c" * 64) == ""


def test_legacy_role_alias_is_reclaimed_only_after_its_tag_disappears(tmp_path, monkeypatch):
    from elesim_setup.readable_names import reclaim_unused_image_names

    monkeypatch.setattr("elesim_setup.readable_names.secrets.choice", lambda words: words[0])
    path = tmp_path / "names.json"
    old = reserve_name(path, "sim", "a" * 64, generate=lambda: ANIMALS[0])
    live = reserve_name(path, "pilot", "b" * 64, generate=lambda: ANIMALS[1])
    assert reclaim_unused_image_names(path, (live,)) == (old,)
    assert lookup_image_names(path, "sim", "a" * 64) == ()
    assert lookup_image_names(path, "pilot", "b" * 64) == (live,)
    assert reserve_image_name(path, "ui", "c" * 64) == old


def test_role_release_alias_reuses_latest_legacy_generation(tmp_path):
    path = tmp_path / "names.json"
    args = (path, "git-" + "1" * 40, "sim", "a" * 64, "b" * 64)
    identity = role_release_reservation_identity(*args[1:])
    reserve_name(path, "releases", identity + ":0", generate=lambda: "calm_eagle")
    mark_release_names_published(path, ("calm_eagle",))
    reserve_name(path, "releases", identity + ":1", generate=lambda: "silver_fox")
    mark_release_names_published(path, ("silver_fox",))

    assert reserve_role_release_name(
        *args, generate=lambda: pytest.fail("legacy published alias must be reused")
    ) == "silver_fox"
