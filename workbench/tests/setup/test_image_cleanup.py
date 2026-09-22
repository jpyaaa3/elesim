import json
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest

from elesim_setup import image_cleanup
from elesim_setup.instance_identity import image_reference, project_name
from elesim_setup.ownership import DockerOwnership, OwnershipManifest, write_ownership_manifest
from elesim_setup.releases import ReleaseManifest, publish_release, release_key, runtime_data_digest


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


@pytest.fixture
def scenario(tmp_path):
    prefix = tmp_path / "install"
    prefix.mkdir()
    (prefix / "bin").mkdir()
    (prefix / "instances/.locks").mkdir(parents=True)
    (prefix / "containers").mkdir()
    compose = prefix / "containers/compose.yaml"
    compose.write_text("services: {}\n")
    data = tmp_path / "data"
    data.mkdir()
    releases, images = [], {}
    for fingerprint, identity in (("a", "c"), ("b", "d")):
        tag = image_reference(INSTALL, "pilot", fingerprint * 64)
        image = "sha256:" + identity * 64
        release = ReleaseManifest(INSTALL, "git-" + fingerprint * 40, "linux/amd64",
                                  {"pilot": tag}, {"pilot": image}, {"pilot": fingerprint * 64},
                                  runtime_data_digest(data))
        publish_release(prefix, release, data)
        releases.append(release)
        images[image] = {"Id": image, "RepoTags": [tag], "Config": {"Labels": {
            "io.elesim.install_uuid": INSTALL, "com.docker.compose.project": project_name(INSTALL),
            "io.elesim.build_fingerprint": fingerprint * 64,
        }}}
    write_ownership_manifest(
        prefix=prefix, bin_dir=prefix / "bin", edition="general", install_uuid=INSTALL,
        inventory_roots=(prefix / "containers",),
        managed_roots=(prefix / "containers", prefix / "instances", prefix / "releases"),
        created_roots=(prefix,), wrapper_paths=(),
        docker=DockerOwnership(INSTALL, str(compose), project_name(INSTALL), (),
                               tuple(tag for record in images.values() for tag in record["RepoTags"]),
                               "test-context", "engine-a"),
    )
    state = {"prefix": prefix, "releases": releases, "images": images, "containers": {},
             "removed": [], "engine": "engine-a", "current": releases[1].role_images["pilot"]}

    def docker(context, *args):
        assert context == "test-context"
        if args[0] == "info":
            return state["engine"]
        if args[0] == "compose":
            assert args[-2:] == ("config", "--images")
            return state["current"]
        if args[0] == "ps":
            assert args == ("ps", "-aq", "--no-trunc")
            return "\n".join(state["containers"])
        if args[:2] == ("container", "inspect"):
            return json.dumps([{"Image": state["containers"][args[2]]}])
        if args[:2] == ("image", "ls"):
            return "\n".join(images)
        if args[:2] == ("image", "inspect"):
            record = images.get(args[2])
            if record is None:
                record = next(record for record in images.values() if args[2] in record["RepoTags"])
            return json.dumps([record])
        assert args[:2] == ("image", "rm") and len(args) == 3
        state["removed"].append(args[2])
        del images[args[2]]
        return ""

    state["docker"] = docker
    return state


def collect(state):
    return image_cleanup.collect(state["prefix"], docker=state["docker"])


def test_collect_only_previous_unreferenced_image_and_repeat_is_safe(scenario):
    old, current = scenario["releases"]
    assert collect(scenario) == (old.image_ids["pilot"],)
    assert current.image_ids["pilot"] in scenario["images"]
    assert collect(scenario) == ()
    assert (scenario["prefix"] / "releases" / release_key(old) / "manifest.json").is_file()


def test_registered_stopped_system_keeps_its_old_release(scenario):
    old = scenario["releases"][0]
    directory = scenario["prefix"] / "instances/experiment"
    directory.mkdir()
    (directory / "state.json").write_text(json.dumps({
        "schema_version": 3, "system_id": "experiment", "release_key": release_key(old),
    }))
    assert collect(scenario) == ()


def test_any_container_even_foreign_or_stopped_protects_image(scenario):
    scenario["containers"]["f" * 64] = scenario["releases"][0].image_ids["pilot"]
    assert collect(scenario) == ()


def test_foreign_alias_is_not_untagged(scenario):
    old = scenario["images"][scenario["releases"][0].image_ids["pilot"]]
    old["RepoTags"].append("research/backup:keep")
    assert collect(scenario) == ()


def test_daemon_mismatch_fails_before_removal(scenario):
    scenario["engine"] = "foreign"
    with pytest.raises(ValueError, match="Engine identity"):
        collect(scenario)
    assert not scenario["removed"]


def test_owned_tag_with_foreign_labels_fails_before_removal(scenario):
    old = scenario["images"][scenario["releases"][0].image_ids["pilot"]]
    old["Config"]["Labels"]["io.elesim.install_uuid"] = "foreign"
    with pytest.raises(ValueError, match="ownership mismatch"):
        collect(scenario)
    assert not scenario["removed"]


@pytest.mark.parametrize("bad", ["symlink", "malformed", "lease", "staging"])
def test_incomplete_registry_defers_all_deletion(scenario, bad):
    root = scenario["prefix"] / "instances"
    if bad == "lease":
        (root / ".locks/system.lease").write_text("{}")
    elif bad == "staging":
        (root / ".staging").mkdir()
    elif bad == "symlink":
        (root / "system").symlink_to(scenario["prefix"], target_is_directory=True)
    else:
        (root / "system").mkdir()
        (root / "system/state.json").write_text("{}")
    with pytest.raises(ValueError):
        collect(scenario)
    assert not scenario["removed"]


def test_unpublished_current_build_cannot_collect_previous_release(scenario):
    scenario["current"] = image_reference(INSTALL, "pilot", "e" * 64)
    with pytest.raises(ValueError, match="no published release"):
        collect(scenario)
    assert not scenario["removed"]


def test_dangling_build_cache_not_in_release_provenance_is_preserved(scenario):
    extra = "sha256:" + "e" * 64
    scenario["images"][extra] = {"Id": extra, "RepoTags": [], "Config": {"Labels": {}}}
    collect(scenario)
    assert extra in scenario["images"]


def test_replacing_last_instance_pin_allows_collection(scenario):
    old, current = scenario["releases"]
    directory = scenario["prefix"] / "instances/experiment"
    directory.mkdir()
    state = {"schema_version": 3, "system_id": "experiment", "release_key": release_key(old)}
    path = directory / "state.json"
    path.write_text(json.dumps(state))
    assert collect(scenario) == ()
    state["release_key"] = release_key(current)
    path.write_text(json.dumps(state))
    assert collect(scenario) == (old.image_ids["pilot"],)


def test_repository_digest_is_preserved(scenario):
    scenario["images"][scenario["releases"][0].image_ids["pilot"]]["RepoDigests"] = ["backup@sha256:" + "e" * 64]
    assert collect(scenario) == ()


def test_local_self_repository_digest_does_not_block_collection(scenario):
    old = scenario["releases"][0]
    image = scenario["images"][old.image_ids["pilot"]]
    repository = image["RepoTags"][0].rsplit(":", 1)[0]
    image["RepoDigests"] = [repository + "@" + old.image_ids["pilot"]]
    assert collect(scenario) == (old.image_ids["pilot"],)


def test_owned_historical_aliases_do_not_block_collection(scenario):
    old = scenario["releases"][0]
    image = scenario["images"][old.image_ids["pilot"]]
    alias = "elesim/pilot:quiet_otter-amber_falcon"
    image["RepoTags"].append(alias)

    manifest_path = scenario["prefix"] / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    assert manifest.docker is not None
    project = project_name(INSTALL, install_name="quiet_otter")
    for record in scenario["images"].values():
        record["Config"]["Labels"]["com.docker.compose.project"] = project
    updated = replace(
        manifest,
        docker=replace(
            manifest.docker,
            project=project,
            install_name="quiet_otter",
            local_images=(*manifest.docker.local_images, alias),
        ),
    ).validate()
    manifest_path.write_text(json.dumps(updated.to_dict(), indent=2) + "\n")

    assert collect(scenario) == (old.image_ids["pilot"],)


def test_metadata_race_refuses_removal(scenario):
    original = scenario["docker"]
    old_id = scenario["releases"][0].image_ids["pilot"]
    inspections = 0

    def docker(context, *args):
        nonlocal inspections
        if args == ("image", "inspect", old_id):
            inspections += 1
            if inspections == 2:
                scenario["images"][old_id]["RepoTags"].append("research/backup:keep")
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(ValueError, match="metadata changed"):
        collect(scenario)
    assert not scenario["removed"]


def test_docker_removal_refusal_is_not_forced_or_ignored(scenario):
    original = scenario["docker"]

    def docker(context, *args):
        if args[:2] == ("image", "rm"):
            assert len(args) == 3  # No --force retry or prune fallback.
            raise subprocess.CalledProcessError(1, args, stderr="image is in use")
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(subprocess.CalledProcessError):
        collect(scenario)
    assert not scenario["removed"]
