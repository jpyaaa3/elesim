from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from elesim_setup.instance_identity import image_reference, project_name
from elesim_setup.ownership import (
    DockerOwnership,
    OwnershipManifest,
    write_ownership_manifest,
)
from elesim_setup.release_publication import (
    ReleasePublicationError,
    publish_from_evidence,
)
import elesim_setup.release_publication as release_publication


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _ownership(state, images):
    state.prefix_path.mkdir(parents=True, exist_ok=True)
    state.bin_path.mkdir(parents=True, exist_ok=True)
    compose = state.prefix_path / "containers/compose.yaml"
    compose.parent.mkdir(exist_ok=True)
    compose.write_text("services: {}\n", encoding="utf-8")
    return write_ownership_manifest(
        prefix=state.prefix_path,
        bin_dir=state.bin_path,
        edition="general",
        inventory_roots=(state.prefix_path / "containers",),
        managed_roots=(state.prefix_path / "containers",),
        created_roots=(state.prefix_path, state.bin_path),
        wrapper_paths=(),
        docker=DockerOwnership(
            install_uuid=INSTALL,
            compose_file=str(compose),
            project=project_name(INSTALL),
            containers=(),
            local_images=tuple(images),
            context="test-context",
            engine_id="engine-a",
        ),
        install_uuid=INSTALL,
    )


def _inputs(
    local_state,
    tmp_path: Path,
    roles=("pilot", "sim"),
    *,
    owned_images: bool = True,
):
    state = local_state(roles=roles, source_repository="jpyaaa3/elesim")
    snapshot = state.prefix_path / "containers/runtime-snapshot"
    (snapshot / "data").mkdir(parents=True)
    (snapshot / "data/model.json").write_text("{}\n", encoding="utf-8")
    for role in roles:
        (snapshot / "config" / role).mkdir(parents=True)
        (snapshot / "config" / role / "runtime.yaml").write_text(
            f"role: {role}\n", encoding="utf-8"
        )
    evidence = state.prefix_path / "containers/build-evidence.json"
    role_values = {}
    for index, role in enumerate(roles, start=1):
        fingerprint = (chr(ord("a") + index) * 64)
        role_values[role] = {
            "image_reference": image_reference(INSTALL, role, fingerprint),
            "image_id": "sha256:" + (chr(ord("c") + index) * 64),
            "install_uuid": INSTALL,
            "build_fingerprint": fingerprint,
            "project": project_name(INSTALL),
        }
    evidence.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "install_uuid": INSTALL,
                "project": project_name(INSTALL),
                "platform": "linux/amd64",
                "roles": role_values,
            }
        ),
        encoding="utf-8",
    )
    ownership = _ownership(
        state,
        (
            (value["image_reference"] for value in role_values.values())
            if owned_images
            else ()
        ),
    )
    return state, ownership, snapshot, evidence


def test_publish_consumes_host_evidence_without_docker(local_state, tmp_path: Path):
    state, ownership, snapshot, evidence = _inputs(local_state, tmp_path)
    result = publish_from_evidence(
        state,
        ownership,
        source_revision="git-" + "d" * 40,
        runtime_snapshot=snapshot,
        evidence_path=evidence,
    )
    assert result.release_key == result.release_path.name
    assert result.release_path.parent == state.prefix_path / "releases"
    assert (result.release_path / "data/data/model.json").is_file()


def test_publish_records_new_install_scoped_images_for_uninstall(
    local_state, tmp_path: Path
):
    state, ownership, snapshot, evidence = _inputs(
        local_state, tmp_path, owned_images=False
    )
    result = publish_from_evidence(
        state,
        ownership,
        source_revision="git-" + "d" * 40,
        runtime_snapshot=snapshot,
        evidence_path=evidence,
    )
    updated = OwnershipManifest.load(ownership.path)
    assert updated.docker is not None
    assert {
        value["image_reference"]
        for value in json.loads(evidence.read_text(encoding="utf-8"))["roles"].values()
    }.issubset(updated.docker.local_images)
    assert result.release_path.is_dir()


def test_publication_failure_does_not_append_image_ownership(
    local_state, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state, ownership, snapshot, evidence = _inputs(
        local_state, tmp_path, owned_images=False
    )
    before = ownership.path.read_bytes()

    def fail_publish(*_args, **_kwargs):
        raise OSError("injected publication failure")

    monkeypatch.setattr(release_publication, "publish_release", fail_publish)
    with pytest.raises(OSError, match="injected publication"):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "d" * 40,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )

    assert ownership.path.read_bytes() == before


def test_ownership_failure_after_publication_is_explicit_and_retryable(
    local_state, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    state, ownership, snapshot, evidence = _inputs(
        local_state, tmp_path, owned_images=False
    )
    original_append = release_publication.append_docker_image_ownership
    calls = 0

    def fail_once(**kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("injected ownership failure")
        return original_append(**kwargs)

    monkeypatch.setattr(
        release_publication, "append_docker_image_ownership", fail_once
    )
    with pytest.raises(ReleasePublicationError, match="published but image ownership"):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "d" * 40,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )

    published = tuple((state.prefix_path / "releases").iterdir())
    release_dirs = [path for path in published if path.is_dir()]
    assert len(release_dirs) == 1
    assert OwnershipManifest.load(ownership.path).docker.local_images == ()

    monkeypatch.setattr(
        release_publication, "append_docker_image_ownership", original_append
    )
    result = publish_from_evidence(
        state,
        ownership,
        source_revision="git-" + "d" * 40,
        runtime_snapshot=snapshot,
        evidence_path=evidence,
    )
    assert result.release_path.is_dir()
    repaired = OwnershipManifest.load(ownership.path)
    assert repaired.docker is not None
    assert len(repaired.docker.local_images) == 2


@pytest.mark.parametrize(
    "mutate",
    (
        lambda payload: payload["roles"].update({"extra": payload["roles"]["pilot"]}),
        lambda payload: payload["roles"].pop("sim"),
        lambda payload: payload["roles"]["pilot"].update({"image_reference": "elesim/pilot:local"}),
        lambda payload: payload["roles"]["pilot"].update({"image_id": "sha256:short"}),
        lambda payload: payload["roles"]["pilot"].update({"build_fingerprint": "A" * 64}),
    ),
)
def test_rejects_malformed_or_non_exact_evidence(local_state, tmp_path: Path, mutate):
    state, ownership, snapshot, evidence = _inputs(local_state, tmp_path)
    payload = json.loads(evidence.read_text(encoding="utf-8"))
    mutate(payload)
    evidence.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="sha256-" + "e" * 64,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )


def test_rejects_foreign_identity_and_revision(local_state, tmp_path: Path):
    state, ownership, snapshot, evidence = _inputs(local_state, tmp_path)
    payload = json.loads(evidence.read_text(encoding="utf-8"))
    payload["install_uuid"] = "fedcba98-7654-3210-fedc-ba9876543210"
    evidence.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="not-a-revision",
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )


def test_rejects_symlink_hardlink_and_snapshot_escape(local_state, tmp_path: Path):
    state, ownership, snapshot, evidence = _inputs(local_state, tmp_path)
    outside = tmp_path / "outside.json"
    outside.write_text(evidence.read_text(encoding="utf-8"), encoding="utf-8")
    evidence.unlink()
    evidence.symlink_to(outside)
    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "a" * 40,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )

    evidence.unlink()
    os.mkfifo(evidence)
    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "a" * 40,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )

    evidence.unlink()
    os.link(outside, evidence)
    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "a" * 40,
            runtime_snapshot=snapshot,
            evidence_path=evidence,
        )

    with pytest.raises(ReleasePublicationError):
        publish_from_evidence(
            state,
            ownership,
            source_revision="git-" + "a" * 40,
            runtime_snapshot=tmp_path,
            evidence_path=evidence,
        )
