import json
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest

from elesim_setup import image_cleanup
from elesim_setup.instance_identity import (
    image_reference,
    instance_container_name,
    project_name,
    service_key,
)
from elesim_setup.ownership import DockerOwnership, OwnershipManifest, write_ownership_manifest
from elesim_setup.releases import ReleaseManifest, publish_release, release_key, runtime_data_digest
from elesim_setup.readable_names import reserve_role_release_name, role_release_reservation_identity, lookup_name


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
    (prefix / "containers/compose.instances.yaml").write_text("services: {}\n")
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
             "container_metadata": {}, "services": {}, "commands": [],
             "removed": [], "removed_containers": [], "engine": "engine-a",
             "current": releases[1].role_images["pilot"]}

    def docker(context, *args):
        assert context == "test-context"
        state["commands"].append(args)
        if args[0] == "info":
            return state["engine"]
        if args[0] == "compose":
            if args[-3:] == ("config", "--format", "json"):
                return json.dumps({"services": state["services"]})
            assert args[-2:] == ("config", "--images")
            return state["current"]
        if args[0] == "ps":
            assert args == ("ps", "-aq", "--no-trunc")
            return "\n".join(state["containers"])
        if args[:2] == ("container", "inspect"):
            image = state["containers"][args[2]]
            reference = state.get("container_references", {}).get(args[2], image)
            record = {"Id": args[2], "Image": image}
            if reference is not None:
                record["Config"] = {"Image": reference}
            metadata = state["container_metadata"].get(args[2], {})
            for key, value in metadata.items():
                if key == "Config" and isinstance(value, dict):
                    record[key] = {**record.get(key, {}), **value}
                else:
                    record[key] = value
            return json.dumps([record])
        if args[:2] == ("container", "rm"):
            assert len(args) == 3  # Never force removal or remove attached volumes.
            container = args[2]
            assert container in state["containers"]
            state["removed_containers"].append(container)
            del state["containers"][container]
            return ""
        if args[:2] == ("image", "ls"):
            if args[2:] == ("--format", "{{.Repository}}:{{.Tag}}"):
                return "\n".join(tag for record in images.values() for tag in record["RepoTags"])
            return "\n".join(images)
        if args[:2] == ("image", "inspect"):
            record = images.get(args[2])
            if record is None:
                record = next((record for record in images.values() if args[2] in record["RepoTags"]), None)
            if record is None:
                raise subprocess.CalledProcessError(1, args, stderr="No such image")
            return json.dumps([record])
        assert args[:2] == ("image", "rm") and len(args) == 3
        reference = args[2]
        if reference in images:
            state["removed"].append(reference)
            del images[reference]
            return ""
        record = next(record for record in images.values() if reference in record["RepoTags"])
        state["removed"].append(reference)
        record["RepoTags"].remove(reference)
        if not record["RepoTags"]:
            del images[record["Id"]]
        return ""

    state["docker"] = docker
    return state


def collect(state):
    return image_cleanup.collect(state["prefix"], docker=state["docker"])


def register_system(state, release, *, system="experiment", endpoints=None):
    directory = state["prefix"] / "instances" / system
    directory.mkdir()
    (directory / "state.json").write_text(json.dumps({
        "schema_version": 3,
        "system_id": system,
        "release_key": release_key(release),
        "endpoints": endpoints if endpoints is not None else [
            {"role": "pilot", "endpoint_id": "pilot_one"},
        ],
    }))
    return directory


def owned_container(state, *, container="e" * 64, system="retired", endpoint="pilot_old",
                    name=None, oneoff="True", historical=False, status="exited"):
    owner = OwnershipManifest.load(state["prefix"] / "install-ownership.json")
    old = state["releases"][0]
    labels = {
        "io.elesim.install_uuid": INSTALL,
        "com.docker.compose.project": owner.docker.project,
        "com.docker.compose.project.config_files": str(
            state["prefix"] / "containers" / (
                "compose.yaml" if historical else "compose.instances.yaml"
            )
        ),
        "com.docker.compose.service": "pilot" if historical else service_key(system, endpoint),
        "com.docker.compose.oneoff": oneoff,
    }
    if not historical:
        labels.update({
            "io.elesim.system_id": system,
            "io.elesim.endpoint_id": endpoint,
            "io.elesim.role": "pilot",
        })
    state["containers"][container] = old.image_ids["pilot"]
    state["container_metadata"][container] = {
        "Name": "/" + (name or "operator-chosen-old-run"),
        "Config": {"Image": old.role_images["pilot"], "Labels": labels},
        "State": {"Status": status, "Running": False, "Paused": False, "Restarting": False},
    }
    return container


def owned_named_alias(state, image_id, name):
    alias = f"elesim/pilot:quiet_otter-{name}"
    state["images"][image_id]["RepoTags"].append(alias)
    path = state["prefix"] / "install-ownership.json"
    manifest = OwnershipManifest.load(path)
    project = project_name(INSTALL, install_name="quiet_otter")
    for record in state["images"].values():
        record["Config"]["Labels"]["com.docker.compose.project"] = project
    updated = replace(manifest, docker=replace(
        manifest.docker, project=project, install_name="quiet_otter",
        local_images=(*manifest.docker.local_images, alias),
    )).validate()
    path.write_text(json.dumps(updated.to_dict(), indent=2) + "\n")
    return alias


def test_unused_alias_is_collected_even_when_current_image_is_retained(scenario):
    old, current = scenario["releases"]
    image_id = current.image_ids["pilot"]
    alias = owned_named_alias(scenario, image_id, "amber_falcon")

    assert collect(scenario) == (old.image_ids["pilot"],)
    assert scenario["images"][image_id]["RepoTags"] == [current.role_images["pilot"]]
    assert alias in scenario["removed"]
    assert collect(scenario) == ()


def test_shared_image_preserves_pinned_tag_and_retires_unused_release(scenario):
    old, current = scenario["releases"]
    image_id = current.image_ids["pilot"]
    pinned_tag = owned_named_alias(scenario, image_id, "silver_fox")
    unused_tag = owned_named_alias(scenario, image_id, "amber_falcon")
    data = scenario["prefix"] / "shared-data"
    data.mkdir()
    pinned = replace(current, source_revision="git-" + "e" * 40,
                     role_images={"pilot": pinned_tag})
    unused = replace(current, source_revision="git-" + "f" * 40,
                     role_images={"pilot": unused_tag})
    for release in (pinned, unused):
        publish_release(scenario["prefix"], release, data)
    register_system(scenario, pinned)
    registry = scenario["prefix"] / "containers/image-names.json"
    unused_input = (unused.source_revision, "pilot", "b" * 64, unused.runtime_data_digest)
    assert reserve_role_release_name(registry, *unused_input,
                                     generate=lambda: "amber_falcon") == "amber_falcon"

    assert collect(scenario) == (old.image_ids["pilot"],)
    assert set(scenario["images"][image_id]["RepoTags"]) == {
        current.role_images["pilot"], pinned_tag,
    }
    assert {release_key(item) for item in image_cleanup.recorded_available_releases(scenario["prefix"])} == {
        release_key(current), release_key(pinned),
    }
    assert lookup_name(registry, "releases", role_release_reservation_identity(*unused_input) + ":0") == ""
    assert reserve_role_release_name(registry, "git-" + "1" * 40, "pilot", "1" * 64,
                                     unused.runtime_data_digest,
                                     generate=lambda: "amber_falcon") == "amber_falcon"


@pytest.mark.parametrize("reference", ["tag", "id", "missing"])
def test_container_image_and_exact_tag_are_protected_separately(scenario, reference):
    old = scenario["releases"][0]
    image_id = old.image_ids["pilot"]
    used = owned_named_alias(scenario, image_id, "silver_fox")
    unused = owned_named_alias(scenario, image_id, "amber_falcon")
    container = "f" * 64
    scenario["containers"][container] = image_id
    scenario["container_references"] = {
        container: {"tag": used, "id": image_id, "missing": None}[reference],
    }

    assert collect(scenario) == ()
    tags = scenario["images"][image_id]["RepoTags"]
    if reference == "tag":
        assert tags == [used]
    elif reference == "id":
        assert tags == [old.role_images["pilot"]]
    else:
        assert set(tags) == {old.role_images["pilot"], used, unused}


def test_container_reference_change_aborts_before_any_tag_removal(scenario):
    old, current = scenario["releases"]
    alias = owned_named_alias(scenario, current.image_ids["pilot"], "silver_fox")
    original = scenario["docker"]
    polls = 0

    def docker(context, *args):
        nonlocal polls
        if args == ("ps", "-aq", "--no-trunc"):
            polls += 1
            if polls == 2:
                scenario["containers"]["f" * 64] = current.image_ids["pilot"]
                scenario["container_references"] = {"f" * 64: alias}
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(ValueError, match="container references changed"):
        collect(scenario)
    assert not scenario["removed"]
    assert old.image_ids["pilot"] in scenario["images"]


def test_each_container_keeps_its_original_tag_on_a_shared_image(scenario):
    old = scenario["releases"][0]
    image_id = old.image_ids["pilot"]
    first = owned_named_alias(scenario, image_id, "silver_fox")
    second = owned_named_alias(scenario, image_id, "amber_falcon")
    unused = owned_named_alias(scenario, image_id, "quiet_lion")
    scenario["containers"] = {"e" * 64: image_id, "f" * 64: image_id}
    scenario["container_references"] = {"e" * 64: first, "f" * 64: second}

    assert collect(scenario) == ()
    assert set(scenario["images"][image_id]["RepoTags"]) == {first, second}
    assert unused in scenario["removed"]


@pytest.mark.parametrize("boundary", ["foreign_alias", "unowned_alias", "foreign_digest"])
def test_retained_image_alias_cleanup_respects_external_references(scenario, boundary):
    old, current = scenario["releases"]
    image_id = current.image_ids["pilot"]
    alias = owned_named_alias(scenario, image_id, "amber_falcon")
    image = scenario["images"][image_id]
    if boundary == "foreign_alias":
        image["RepoTags"].append("research/backup:keep")
    elif boundary == "unowned_alias":
        image["RepoTags"].append("elesim/pilot:quiet_otter-silver_fox")
    else:
        image["RepoDigests"] = ["backup@sha256:" + "e" * 64]

    assert collect(scenario) == (old.image_ids["pilot"],)
    assert alias not in scenario["removed"]
    assert alias in image["RepoTags"]


def test_retained_image_with_bad_ownership_aborts_all_cleanup(scenario):
    current = scenario["releases"][1]
    image_id = current.image_ids["pilot"]
    owned_named_alias(scenario, image_id, "amber_falcon")
    scenario["images"][image_id]["Config"]["Labels"]["io.elesim.install_uuid"] = "foreign"

    with pytest.raises(ValueError, match="ownership mismatch"):
        collect(scenario)
    assert not scenario["removed"]


def test_collect_only_previous_unreferenced_image_and_repeat_is_safe(scenario):
    old, current = scenario["releases"]
    assert collect(scenario) == (old.image_ids["pilot"],)
    assert current.image_ids["pilot"] in scenario["images"]
    assert collect(scenario) == ()
    assert (scenario["prefix"] / "releases" / release_key(old) / "manifest.json").is_file()


def test_available_releases_excludes_collected_historical_manifest(scenario):
    assert {release_key(item) for item in image_cleanup.available_releases(scenario["prefix"], docker=scenario["docker"])} == {
        release_key(item) for item in scenario["releases"]
    }
    collect(scenario)
    assert image_cleanup.available_releases(scenario["prefix"], docker=scenario["docker"]) == (scenario["releases"][1],)
    assert image_cleanup.recorded_available_releases(scenario["prefix"]) == (scenario["releases"][1],)


def test_available_release_index_rejects_wrong_engine_and_schema(scenario):
    collect(scenario)
    path = scenario["prefix"] / "maintenance/available-releases.json"
    data = json.loads(path.read_text())
    data["engine_id"] = "other-engine"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="invalid available release index"):
        image_cleanup.recorded_available_releases(scenario["prefix"])
    data["engine_id"] = "engine-a"
    data["schema_version"] = True
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="invalid available release index"):
        image_cleanup.recorded_available_releases(scenario["prefix"])


def test_local_connection_choices_hide_collected_release_without_docker_socket(scenario):
    from elesim_connections.connection_gui import ConnectionManagerApplication

    old, current = scenario["releases"]
    collect(scenario)
    prefix = scenario["prefix"]
    app = ConnectionManagerApplication(
        state_path=prefix / "connections.json", token="test-session-token",
        runner=lambda *_: None, local_install_root=prefix,
        local_bin_dir=prefix / "bin",
    )
    result = app.installation_choices({
        "local": True, "install_root": str(prefix),
        "bin_dir": str(prefix / "bin"), "ssh": None,
    })
    keys = [item["key"] for item in result["installations"][0]["releases"]]
    assert keys == [release_key(current)]
    assert release_key(old) not in keys


def test_available_releases_rejects_reused_tag_with_different_image_id(scenario):
    old, current = scenario["releases"]
    scenario["images"][old.image_ids["pilot"]]["RepoTags"].remove(old.role_images["pilot"])
    scenario["images"][current.image_ids["pilot"]]["RepoTags"].append(old.role_images["pilot"])
    assert image_cleanup.available_releases(scenario["prefix"], docker=scenario["docker"]) == (current,)


def test_registered_stopped_system_keeps_its_old_release(scenario):
    old = scenario["releases"][0]
    register_system(scenario, old)
    assert collect(scenario) == ()


def test_any_container_even_foreign_or_stopped_protects_image(scenario):
    scenario["containers"]["f" * 64] = scenario["releases"][0].image_ids["pilot"]
    assert collect(scenario) == ()


@pytest.mark.parametrize("named", [False, True])
def test_unreferenced_development_image_is_never_auto_collected(scenario, named):
    image = "sha256:" + "e" * 64
    tag = image_reference(
        INSTALL, "dev", "e" * 64,
        **({"install_name": "cozy", "image_name": "wolf"} if named else {}),
    )
    project = project_name(INSTALL, install_name="cozy" if named else "")
    if named:
        for record in scenario["images"].values():
            record["Config"]["Labels"]["com.docker.compose.project"] = project
    scenario["images"][image] = {"Id": image, "RepoTags": [tag], "Config": {"Labels": {
        "io.elesim.install_uuid": INSTALL,
        "com.docker.compose.project": project,
        "io.elesim.build_fingerprint": "e" * 64,
    }}}
    manifest_path = scenario["prefix"] / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    assert manifest.docker is not None
    updated = replace(manifest, docker=replace(
        manifest.docker, project=project, install_name="cozy" if named else "",
        local_images=(*manifest.docker.local_images, tag),
    )).validate()
    manifest_path.write_text(json.dumps(updated.to_dict(), indent=2) + "\n")

    assert collect(scenario) == (scenario["releases"][0].image_ids["pilot"],)
    assert image in scenario["images"]
    assert tag not in scenario["removed"]


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
    owned_container(scenario)
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
    assert not scenario["removed_containers"]


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
    state = {"schema_version": 3, "system_id": "experiment", "release_key": release_key(old),
             "endpoints": [{"role": "pilot", "endpoint_id": "pilot_one"}]}
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
    assert len(scenario["removed"]) == 2
    assert all(value.startswith("elesim/pilot:") for value in scenario["removed"])


def test_collected_named_release_alias_can_be_reused_without_touching_live_alias(scenario):
    old, current = scenario["releases"]
    project = project_name(INSTALL, install_name="quiet_otter")
    old_tag = "elesim/pilot:quiet_otter-amber_falcon"
    current_tag = "elesim/pilot:quiet_otter-silver_fox"
    scenario["images"][old.image_ids["pilot"]]["RepoTags"].append(old_tag)
    scenario["images"][current.image_ids["pilot"]]["RepoTags"].append(current_tag)
    scenario["current"] += "\n" + current_tag
    for record in scenario["images"].values():
        record["Config"]["Labels"]["com.docker.compose.project"] = project
    manifest_path = scenario["prefix"] / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    assert manifest.docker is not None
    updated = replace(manifest, docker=replace(
        manifest.docker, project=project, install_name="quiet_otter",
        local_images=(*manifest.docker.local_images, old_tag, current_tag),
    )).validate()
    manifest_path.write_text(json.dumps(updated.to_dict(), indent=2) + "\n")
    registry = scenario["prefix"] / "containers/image-names.json"
    old_input = ("git-" + "a" * 40, "pilot", "a" * 64, "c" * 64)
    current_input = ("git-" + "b" * 40, "pilot", "b" * 64, "c" * 64)
    assert reserve_role_release_name(registry, *old_input, generate=lambda: "amber_falcon") == "amber_falcon"
    assert reserve_role_release_name(registry, *current_input, generate=lambda: "silver_fox") == "silver_fox"

    assert collect(scenario) == (old.image_ids["pilot"],)
    assert lookup_name(registry, "releases", role_release_reservation_identity(*old_input) + ":0") == ""
    assert lookup_name(registry, "releases", role_release_reservation_identity(*current_input) + ":0") == "silver_fox"
    assert reserve_role_release_name(registry, "git-" + "e" * 40, "pilot", "e" * 64, "c" * 64,
                                     generate=lambda: "amber_falcon") == "amber_falcon"


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


@pytest.mark.parametrize("status", ["exited", "dead"])
def test_stopped_owned_orphan_is_removed_before_its_image(scenario, status):
    container = owned_container(scenario, status=status)
    old = scenario["releases"][0]

    assert collect(scenario) == (old.image_ids["pilot"],)
    assert scenario["removed_containers"] == [container]
    removals = [args for args in scenario["commands"] if args[:2] in {
        ("container", "rm"), ("image", "rm"),
    }]
    assert removals[0] == ("container", "rm", container)
    assert collect(scenario) == ()


@pytest.mark.parametrize(("field", "value"), [
    ("Status", "created"), ("Status", "running"), ("Status", "paused"),
    ("Status", "restarting"), ("Status", "removing"), ("Status", None),
    ("Running", True), ("Paused", True), ("Restarting", True),
    ("Running", None), ("Paused", "false"), ("Restarting", 0),
    ("Running", "missing"), ("Paused", "missing"), ("Restarting", "missing"),
])
def test_orphan_requires_explicitly_inactive_completed_state(scenario, field, value):
    container = owned_container(scenario)
    state = scenario["container_metadata"][container]["State"]
    if value == "missing":
        del state[field]
    else:
        state[field] = value

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]
    assert container in scenario["containers"]


@pytest.mark.parametrize(("label", "value"), [
    ("io.elesim.install_uuid", "foreign"),
    ("com.docker.compose.project", "foreign-project"),
    ("com.docker.compose.project.config_files", "/elsewhere/compose.yaml"),
    ("com.docker.compose.project.config_files", ""),
    ("com.docker.compose.service", "pilot"),
    ("com.docker.compose.service", "missing"),
    ("io.elesim.system_id", "../retired"),
    ("io.elesim.endpoint_id", "unsafe/endpoint"),
    ("io.elesim.role", "robot"), ("io.elesim.role", "dev"),
    ("io.elesim.role", "tools"), ("io.elesim.role", "coturn"),
    ("io.elesim.role", "sim"),
    ("io.elesim.system_id", "missing"),
    ("io.elesim.endpoint_id", "missing"),
    ("io.elesim.role", "missing"),
    ("io.elesim.service_kind", "coturn"),
    ("io.elesim.service_kind", "tailscale"),
    ("io.elesim.service_kind", ""),
])
def test_orphan_ownership_and_runtime_role_must_be_complete(scenario, label, value):
    container = owned_container(scenario)
    labels = scenario["container_metadata"][container]["Config"]["Labels"]
    if value == "missing":
        del labels[label]
    else:
        labels[label] = value

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


@pytest.mark.parametrize("files", [
    ("compose.yaml",), ("compose.instances.yaml",),
    ("compose.yaml", "compose.instances.yaml"),
])
def test_scoped_orphan_requires_aggregate_within_owned_compose_file_boundaries(scenario, files):
    container = owned_container(scenario)
    labels = scenario["container_metadata"][container]["Config"]["Labels"]
    labels["com.docker.compose.project.config_files"] = ",".join(
        str(scenario["prefix"] / "containers" / name) for name in files
    )

    scoped = "compose.instances.yaml" in files
    expected = (scenario["releases"][0].image_ids["pilot"],) if scoped else ()
    assert collect(scenario) == expected
    assert scenario["removed_containers"] == ([container] if scoped else [])


@pytest.mark.parametrize("suffix", [",/foreign/compose.yaml", ".backup"])
def test_orphan_rejects_extra_or_similarly_named_compose_file(scenario, suffix):
    container = owned_container(scenario)
    labels = scenario["container_metadata"][container]["Config"]["Labels"]
    labels["com.docker.compose.project.config_files"] += suffix

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


@pytest.mark.parametrize("oneoff", [False, True])
def test_declared_exact_container_name_is_preserved_even_when_stopped(scenario, oneoff):
    name = "currently-declared-pilot"
    scenario["services"] = {"pilot": {"container_name": name}}
    container = owned_container(scenario, name=name, oneoff=str(oneoff))

    assert collect(scenario) == ()
    assert container in scenario["containers"]
    assert not scenario["removed_containers"]


def test_currently_declared_base_service_normal_container_is_preserved(scenario):
    scenario["services"] = {"pilot": {}}
    owned_container(scenario, historical=True, oneoff="False")

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


def test_completed_historical_base_oneoff_is_removed_while_service_is_declared(scenario):
    scenario["services"] = {"pilot": {"container_name": "current-pilot"}}
    container = owned_container(scenario, historical=True, name="human-chosen-debug-run")

    assert collect(scenario) == (scenario["releases"][0].image_ids["pilot"],)
    assert scenario["removed_containers"] == [container]


@pytest.mark.parametrize("oneoff", ["False", "", None, "missing"])
def test_historical_base_container_without_oneoff_proof_is_preserved(scenario, oneoff):
    container = owned_container(scenario, historical=True, oneoff=oneoff)
    labels = scenario["container_metadata"][container]["Config"]["Labels"]
    if oneoff == "missing":
        del labels["com.docker.compose.oneoff"]

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


@pytest.mark.parametrize("service", ["dev", "tools", "coturn", "tailscale", "robot"])
def test_historical_infrastructure_and_nonruntime_oneoffs_are_preserved(scenario, service):
    container = owned_container(scenario, historical=True)
    scenario["container_metadata"][container]["Config"]["Labels"][
        "com.docker.compose.service"
    ] = service

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


@pytest.mark.parametrize("naming", ["hash-v1", "system-v1"])
def test_registered_endpoint_exact_container_name_is_preserved(scenario, naming):
    old, current = scenario["releases"]
    if naming == "system-v1":
        owned_named_alias(scenario, current.image_ids["pilot"], "amber_falcon")
    path = scenario["prefix"] / "install-ownership.json"
    owner = OwnershipManifest.load(path)
    owner = replace(owner, docker=replace(owner.docker, container_naming=naming)).validate()
    path.write_text(json.dumps(owner.to_dict()))
    register_system(scenario, old)
    name = instance_container_name(
        INSTALL, "experiment", "pilot", "pilot_one",
        install_name=owner.docker.install_name, naming=naming,
    )
    # A preserved endpoint name is protected even if an old oneoff's labels
    # describe an endpoint which has since been replaced in the registry.
    container = owned_container(scenario, name=name)

    assert collect(scenario) == ()
    assert container in scenario["containers"]
    assert not scenario["removed_containers"]


@pytest.mark.parametrize("schema", [2, 3])
def test_registered_coturn_endpoint_identifier_preserves_exact_runtime_container(scenario, schema):
    directory = register_system(scenario, scenario["releases"][0], endpoints=[
        {"role": "pilot", "endpoint_id": "coturn"},
    ])
    path = directory / "state.json"
    state = json.loads(path.read_text())
    state["schema_version"] = schema
    path.write_text(json.dumps(state))
    name = instance_container_name(INSTALL, "experiment", "pilot", "coturn")
    container = owned_container(scenario, name=name)

    assert collect(scenario) == ()
    assert container in scenario["containers"]
    assert not scenario["removed_containers"]


def test_registered_endpoint_custom_name_oneoff_is_still_collected(scenario):
    register_system(scenario, scenario["releases"][0])
    container = owned_container(
        scenario, system="experiment", endpoint="pilot_one", name="debug-chosen-name",
    )

    assert collect(scenario) == ()  # The registered release remains pinned.
    assert scenario["removed_containers"] == [container]


@pytest.mark.parametrize("canonical", [False, True])
def test_normal_scoped_orphan_requires_its_exact_derived_name(scenario, canonical):
    name = instance_container_name(INSTALL, "retired", "pilot", "pilot_old")
    container = owned_container(
        scenario, name=name if canonical else "custom-normal-container", oneoff="False",
    )

    expected = (scenario["releases"][0].image_ids["pilot"],) if canonical else ()
    assert collect(scenario) == expected
    assert scenario["removed_containers"] == ([container] if canonical else [])


def test_foreign_container_with_install_name_prefix_is_preserved(scenario):
    container = "e" * 64
    scenario["containers"][container] = scenario["releases"][0].image_ids["pilot"]
    scenario["container_metadata"][container] = {
        "Name": "/elesim-" + INSTALL.replace("-", "") + "-pilot-old",
        "State": {"Status": "exited", "Running": False, "Paused": False, "Restarting": False},
    }

    assert collect(scenario) == ()
    assert not scenario["removed_containers"]


def test_surviving_container_protects_shared_image_and_exact_tag(scenario):
    old = scenario["releases"][0]
    orphan = owned_container(scenario)
    survivor = "f" * 64
    scenario["containers"][survivor] = old.image_ids["pilot"]
    scenario["container_references"] = {survivor: old.role_images["pilot"]}

    assert collect(scenario) == ()
    assert scenario["removed_containers"] == [orphan]
    assert scenario["images"][old.image_ids["pilot"]]["RepoTags"] == [old.role_images["pilot"]]
    assert not scenario["removed"]


@pytest.mark.parametrize("directories", [
    ("logs",), ("security",), ("secrets",), ("logs", "security", "secrets"),
])
def test_removed_system_with_only_retained_directories_does_not_block_cleanup(scenario, directories):
    root = scenario["prefix"] / "instances/removed_system"
    for name in directories:
        retained = root / name
        retained.mkdir(parents=True)
        (retained / "keep.txt").write_text("operator-retained data")
    container = owned_container(scenario)

    assert collect(scenario) == (scenario["releases"][0].image_ids["pilot"],)
    assert scenario["removed_containers"] == [container]
    assert all((root / name / "keep.txt").is_file() for name in directories)


@pytest.mark.parametrize("endpoints", [
    [], None, [{"role": "robot", "endpoint_id": "robot_one"}],
    [{"role": "pilot", "endpoint_id": "unsafe/id"}],
    [{"role": "pilot"}],
    [{"role": "pilot", "endpoint_id": "pilot_one"},
     {"role": "pilot", "endpoint_id": "pilot_two"}],
])
def test_invalid_registry_endpoints_abort_before_orphan_or_image_removal(scenario, endpoints):
    directory = register_system(scenario, scenario["releases"][0])
    path = directory / "state.json"
    state = json.loads(path.read_text())
    if endpoints is None:
        del state["endpoints"]
    else:
        state["endpoints"] = endpoints
    path.write_text(json.dumps(state))
    owned_container(scenario)

    with pytest.raises(ValueError):
        collect(scenario)
    assert not scenario["removed_containers"]
    assert not scenario["removed"]


def test_bad_image_provenance_aborts_before_orphan_removal(scenario):
    container = owned_container(scenario)
    image = scenario["images"][scenario["containers"][container]]
    image["Config"]["Labels"]["io.elesim.install_uuid"] = "foreign"

    with pytest.raises(ValueError, match="ownership mismatch"):
        collect(scenario)
    assert not scenario["removed_containers"]
    assert not scenario["removed"]


def test_container_metadata_race_aborts_before_orphan_or_image_removal(scenario):
    container = owned_container(scenario)
    original = scenario["docker"]
    polls = 0

    def docker(context, *args):
        nonlocal polls
        if args == ("ps", "-aq", "--no-trunc"):
            polls += 1
            if polls == 2:
                scenario["container_metadata"][container]["State"]["Running"] = True
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(ValueError, match="container.*changed"):
        collect(scenario)
    assert not scenario["removed_containers"]
    assert not scenario["removed"]


def test_image_metadata_race_aborts_before_orphan_removal(scenario):
    owned_container(scenario)
    old = scenario["releases"][0].image_ids["pilot"]
    original = scenario["docker"]
    polls = 0

    def docker(context, *args):
        nonlocal polls
        if args == ("image", "inspect", old):
            polls += 1
            if polls == 2:
                scenario["images"][old]["RepoTags"].append("research/backup:keep")
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(ValueError, match="metadata changed"):
        collect(scenario)
    assert not scenario["removed_containers"]
    assert not scenario["removed"]


def test_orphan_removal_refusal_propagates_without_force_retry(scenario):
    container = owned_container(scenario)
    original = scenario["docker"]
    attempts = []

    def docker(context, *args):
        if args[:2] == ("container", "rm"):
            attempts.append(args)
            raise subprocess.CalledProcessError(1, args, stderr="container is in use")
        return original(context, *args)

    scenario["docker"] = docker
    with pytest.raises(subprocess.CalledProcessError):
        collect(scenario)
    assert attempts == [("container", "rm", container)]
    assert container in scenario["containers"]
    assert not scenario["removed_containers"]
    assert not scenario["removed"]
