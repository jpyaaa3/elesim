from __future__ import annotations

import json
import shutil

import pytest

from elesim_setup.instance_identity import image_reference
from elesim_setup.releases import (
    ReleaseManifest,
    list_releases,
    load_release,
    publish_release,
    release_key,
    runtime_data_digest,
)


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"
REVISION = "git-" + "a" * 40
FINGERPRINT = "b" * 64


def manifest() -> ReleaseManifest:
    return ReleaseManifest(
        install_uuid=INSTALL,
        source_revision=REVISION,
        platform="linux/amd64",
        role_images={"pilot": image_reference(INSTALL, "pilot", FINGERPRINT)},
        image_ids={"pilot": "sha256:" + "c" * 64},
        build_fingerprints={"pilot": FINGERPRINT},
        runtime_data_digest="d" * 64,
    )


def published(tmp_path, *, prefix_name="prefix", content=b"content") -> tuple[ReleaseManifest, Path]:
    source = tmp_path / "source"
    source.mkdir(parents=True)
    (source / "file").write_bytes(content)
    value = ReleaseManifest(
        **{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)}
    )
    path = publish_release(tmp_path / prefix_name, value, source)
    return value, path


def test_release_round_trip_and_key_are_content_addressed(tmp_path) -> None:
    value, root = published(tmp_path)
    path = root / "manifest.json"
    assert path == tmp_path / "prefix" / "releases" / release_key(value) / "manifest.json"
    assert load_release(path) == value
    assert publish_release(tmp_path / "prefix", value, tmp_path / "source") == root


def test_release_listing_accepts_its_private_lock_and_rejects_unknown_entries(tmp_path) -> None:
    value, root = published(tmp_path)
    assert list_releases(tmp_path / "prefix", install_uuid=INSTALL) == (value,)

    unknown = root.parent / ".abandoned-stage"
    unknown.mkdir()
    with pytest.raises(ValueError, match="registry entry"):
        list_releases(tmp_path / "prefix", install_uuid=INSTALL)
    unknown.rmdir()

    lock = root.parent / ".publish.lock"
    lock.unlink()
    lock.symlink_to(tmp_path / "outside-lock")
    with pytest.raises(ValueError, match="lock is unsafe"):
        list_releases(tmp_path / "prefix", install_uuid=INSTALL)


def test_release_rejects_tampering_and_no_overwrite(tmp_path) -> None:
    value, root = published(tmp_path)
    path = root / "manifest.json"
    raw = json.loads(path.read_text())
    raw["runtime_data_digest"] = "e" * 64
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="different.*corrupt"):
        publish_release(tmp_path / "prefix", value, tmp_path / "source")
    with pytest.raises(ValueError, match="does not match"):
        load_release(path)


@pytest.mark.parametrize(
    "changes",
    [
        {"schema_version": True},
        {"source_revision": "git-" + "a" * 39},
        {"platform": "linux/386"},
        {"image_ids": {"pilot": "sha256:" + "a" * 63}},
        {"build_fingerprints": {"pilot": "A" * 64}},
        {"role_images": {"pilot": "elesim/pilot:local"}},
    ],
)
def test_release_validation_is_strict(changes) -> None:
    values = manifest().__dict__ | changes
    with pytest.raises(ValueError):
        ReleaseManifest(**values).validate()


@pytest.mark.parametrize("role", ["robot", "router", "arm64"])
def test_release_rejects_non_docker_role(role: str) -> None:
    values = manifest().__dict__ | {
        "role_images": {role: image_reference(INSTALL, role, FINGERPRINT)},
        "image_ids": {role: "sha256:" + "c" * 64},
        "build_fingerprints": {role: FINGERPRINT},
    }
    with pytest.raises(ValueError, match="only pilot, sim, and ui"):
        ReleaseManifest(**values).validate()


def test_release_rejects_symlink_ancestor_before_writing(tmp_path) -> None:
    outside = tmp_path / "outside"
    (outside / "existing").mkdir(parents=True)
    alias = tmp_path / "alias"
    alias.symlink_to(outside, target_is_directory=True)
    source = tmp_path / "source"
    source.mkdir()
    (source / "file").write_text("content")
    value = ReleaseManifest(**{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)})
    with pytest.raises(ValueError, match="symlink"):
        publish_release(alias / "existing" / "new", value, source)
    assert not (outside / "existing" / "new").exists()


def test_release_load_rejects_misplaced_manifest_and_symlink(tmp_path) -> None:
    _, root = published(tmp_path)
    path = root / "manifest.json"
    wrong = tmp_path / "releases" / ("f" * 64)
    wrong.mkdir(parents=True)
    copy = wrong / "manifest.json"
    copy.write_bytes(path.read_bytes())
    with pytest.raises(ValueError, match="directory"):
        load_release(copy)
    copy.unlink()
    copy.symlink_to(path)
    with pytest.raises(ValueError, match="symlink"):
        load_release(copy)


def test_publish_release_copies_hashed_data_and_is_idempotent(tmp_path) -> None:
    source = tmp_path / "source"
    (source / "nested").mkdir(parents=True)
    (source / "a.txt").write_text("alpha")
    (source / "nested/b.txt").write_text("beta")
    value = ReleaseManifest(**{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)})
    destination = publish_release(tmp_path / "prefix", value, source)
    assert publish_release(tmp_path / "prefix", value, source) == destination
    assert (destination / "data/a.txt").read_text() == "alpha"
    (source / "a.txt").write_text("mutated")
    assert (destination / "data/a.txt").read_text() == "alpha"
    with pytest.raises(ValueError, match="does not match"):
        publish_release(tmp_path / "prefix", value, source)


def test_publish_rejects_digest_mismatch_and_symlink_without_outside_write(tmp_path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    value = manifest()
    with pytest.raises(ValueError, match="does not match"):
        publish_release(tmp_path / "prefix", value, source)
    outside = tmp_path / "outside.txt"
    outside.write_text("keep")
    (source / "link").symlink_to(outside)
    with pytest.raises(ValueError, match="unsupported|symlink"):
        runtime_data_digest(source)
    assert outside.read_text() == "keep"


def test_release_rejects_hardlinked_source_and_manifest(tmp_path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    original = source / "file"
    original.write_text("content")
    (source / "alias").hardlink_to(original)
    with pytest.raises(ValueError, match="unlinked regular"):
        runtime_data_digest(source)

    _, root = published(tmp_path / "published")
    hardlink = root / "manifest-copy.json"
    hardlink.hardlink_to(root / "manifest.json")
    with pytest.raises(ValueError, match="unlinked regular"):
        load_release(hardlink)


def test_publish_does_not_replace_existing_metadata_or_cleans_failed_stage(tmp_path, monkeypatch) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "file").write_text("content")
    value = ReleaseManifest(**{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)})
    prefix = tmp_path / "prefix"
    metadata = publish_release(prefix, value, source)
    shutil.rmtree(metadata / "data")
    with pytest.raises(ValueError, match="incomplete"):
        publish_release(prefix, value, source)
    assert (metadata / "manifest.json").is_file()

    def fail_copy(*args):
        raise OSError("injected copy failure")
    monkeypatch.setattr("elesim_setup.releases._copy_runtime_data", fail_copy)
    other_prefix = tmp_path / "other"
    with pytest.raises(OSError, match="injected"):
        publish_release(other_prefix, value, source)
    releases = other_prefix / "releases"
    assert not tuple(path for path in releases.glob(".*") if path.name != ".publish.lock")


def test_publish_race_keeps_competing_destination_and_exposes_no_partial_release(tmp_path, monkeypatch) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "file").write_text("content")
    value = ReleaseManifest(**{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)})
    prefix = tmp_path / "prefix"
    destination = prefix / "releases" / release_key(value)

    def competing_publish(_temporary, target):
        target.mkdir()
        (target / "marker").write_text("competitor")
        raise FileExistsError(target)

    monkeypatch.setattr("elesim_setup.releases._rename_noreplace", competing_publish)
    with pytest.raises(ValueError, match="appeared"):
        publish_release(prefix, value, source)
    assert (destination / "marker").read_text() == "competitor"
    assert not (destination / "manifest.json").exists()
    assert not tuple(path for path in (prefix / "releases").glob(f".{destination.name}-*"))


def test_publish_rejects_symlink_and_fifo_lock(tmp_path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "file").write_text("content")
    value = ReleaseManifest(**{**manifest().__dict__, "runtime_data_digest": runtime_data_digest(source)})
    prefix = tmp_path / "prefix"
    root = prefix / "releases"
    root.mkdir(parents=True)
    lock = root / ".publish.lock"
    lock.symlink_to(tmp_path / "outside.lock")
    with pytest.raises(ValueError, match="unsafe"):
        publish_release(prefix, value, source)
    lock.unlink()
    import os
    os.mkfifo(lock)
    with pytest.raises(ValueError, match="regular"):
        publish_release(prefix, value, source)
    lock.unlink()
    outside = tmp_path / "outside.lock"
    outside.write_text("")
    lock.hardlink_to(outside)
    with pytest.raises(ValueError, match="singly-linked"):
        publish_release(prefix, value, source)
