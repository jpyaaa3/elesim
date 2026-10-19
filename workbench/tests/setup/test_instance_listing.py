from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from elesim_setup import instance_listing
from elesim_setup.instance_identity import image_reference, project_name
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.ownership import (
    DockerOwnership,
    install_host_uninstaller_bundle,
    write_ownership_manifest,
)
from elesim_setup.releases import (
    ReleaseManifest,
    publish_release,
    release_key,
    runtime_data_digest,
)


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"
FOREIGN_INSTALL = "12345678-9abc-def0-1234-56789abcdef0"
ROLE_ORDER = ("pilot", "sim", "ui")


@pytest.fixture
def installation(tmp_path):
    prefix = tmp_path / "install"
    (prefix / "bin").mkdir(parents=True)
    (prefix / "containers").mkdir()
    compose = prefix / "containers/compose.yaml"
    compose.write_text("services: {}\n", encoding="utf-8")
    write_ownership_manifest(
        prefix=prefix,
        bin_dir=prefix / "bin",
        edition="general",
        install_uuid=INSTALL,
        inventory_roots=(prefix / "containers",),
        managed_roots=(prefix / "containers",),
        created_roots=(prefix,),
        wrapper_paths=(),
        docker=DockerOwnership(
            install_uuid=INSTALL,
            compose_file=str(compose),
            project=project_name(INSTALL, install_name="quick"),
            containers=(),
            local_images=(),
            context="test-context",
            engine_id="engine-a",
            install_name="quick",
        ),
    )
    return prefix


def _release(prefix, aliases, *, revision="a", install_uuid=INSTALL):
    source = prefix.parent / "release-data"
    source.mkdir(exist_ok=True)
    role_images = {
        role: (
            f"elesim/{role}:quick-{alias}"
            if alias is not None
            else image_reference(install_uuid, role, "b" * 64)
        )
        for role, alias in aliases.items()
    }
    manifest = ReleaseManifest(
        install_uuid=install_uuid,
        source_revision="git-" + revision * 40,
        platform="linux/amd64",
        role_images=role_images,
        image_ids={role: "sha256:" + "c" * 64 for role in aliases},
        build_fingerprints={role: "b" * 64 for role in aliases},
        runtime_data_digest=runtime_data_digest(source),
    )
    publish_release(prefix, manifest, source)
    return manifest


def _register(prefix, system, release, roles=("pilot",), *, schema=3):
    value = InstanceState(
        system_id=system,
        release_key=release_key(release),
        endpoints=tuple(InstanceEndpoint(role, f"{system}-{role}") for role in roles),
        domain_id=7,
    ).to_dict()
    if schema == 2:
        value["schema_version"] = 2
        del value["role_ids"]
    path = prefix / "instances" / system / "state.json"
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def _snapshot(prefix):
    paths = [prefix, *sorted(prefix.rglob("*"))]
    return {
        str(path.relative_to(prefix)): (
            path.lstat().st_mode,
            path.lstat().st_mtime_ns,
            path.read_bytes() if path.is_file() and not path.is_symlink() else None,
        )
        for path in paths
    }


def test_lists_only_assigned_roles_from_each_pinned_release_without_mutation(
    installation, monkeypatch
):
    pinned = _release(installation, {"ui": "otter", "sim": "wolf", "pilot": "lion"})
    _release(installation, {"pilot": "fox", "sim": "bear", "ui": "eagle"}, revision="d")
    _register(installation, "zulu", pinned, ("ui", "pilot", "sim"))
    _register(installation, "alpha", pinned, ("sim",))
    # Mutable image-name reservations and a stale availability cache cannot
    # change the names recorded by a registered instance's immutable pin.
    (installation / "containers/image-names.json").write_text("{}\n", encoding="utf-8")
    (installation / "maintenance").mkdir(exist_ok=True)
    (installation / "maintenance/available-releases.json").write_text("{}\n", encoding="utf-8")
    before = _snapshot(installation)

    def unexpected_process(*args, **kwargs):
        pytest.fail("listing local instance metadata must not run subprocesses")

    monkeypatch.setattr("subprocess.run", unexpected_process)
    rows = instance_listing.list_instances(installation)

    assert rows == (
        {"system_id": "alpha", "roles": {"sim": "wolf"}},
        {"system_id": "zulu", "roles": {"pilot": "lion", "sim": "wolf", "ui": "otter"}},
    )
    assert tuple(rows[1]["roles"]) == ROLE_ORDER
    assert _snapshot(installation) == before
    assert not (installation / "instances/.locks").exists()


@pytest.mark.parametrize("schema", [2, 3])
def test_reads_supported_instance_schemas_and_historical_aliases(installation, schema):
    release = _release(installation, {"pilot": "silver_fox", "ui": "owl"})
    _register(installation, "alpha", release, ("pilot",), schema=schema)

    assert instance_listing.list_instances(installation) == (
        {"system_id": "alpha", "roles": {"pilot": "silver_fox"}},
    )


def test_legacy_fingerprint_tag_has_no_invented_call_sign(installation):
    release = _release(installation, {"pilot": None, "sim": "wolf"})
    _register(installation, "alpha", release, ("pilot", "sim"))

    rows = instance_listing.list_instances(installation)

    assert rows == ({"system_id": "alpha", "roles": {"pilot": None, "sim": "wolf"}},)
    assert instance_listing.format_instances(rows).splitlines() == [
        "alpha",
        "  pilot: (no call sign)",
        "  sim: wolf",
    ]


def test_empty_registry_is_reported_without_creating_files(installation, capsys):
    before = _snapshot(installation)

    assert instance_listing.list_instances(installation) == ()
    assert instance_listing.format_instances(()) == "No registered instances."
    assert instance_listing.main(["--prefix", str(installation)]) == 0
    assert capsys.readouterr().out.strip() == "No registered instances."
    assert _snapshot(installation) == before
    assert not (installation / "instances").exists()


def test_cli_json_preserves_grouped_instance_ids_and_null_call_sign(installation, capsys):
    release = _release(installation, {"pilot": "lion", "ui": None})
    _register(installation, "alpha", release, ("ui", "pilot"))

    assert instance_listing.main(["--prefix", str(installation), "--json"]) == 0

    output = capsys.readouterr()
    assert output.err == ""
    assert json.loads(output.out) == {
        "instances": [{"system_id": "alpha", "roles": {"pilot": "lion", "ui": None}}]
    }


@pytest.mark.parametrize("retained", [("logs",), ("security", "logs", "secrets")])
def test_skips_retained_directories_from_removed_instances(installation, retained):
    for name in retained:
        directory = installation / "instances/removed" / name
        directory.mkdir(parents=True)
        (directory / "snapshot").write_text("retained", encoding="utf-8")
    release = _release(installation, {"pilot": "lion"})
    _register(installation, "active", release)
    before = _snapshot(installation)

    assert instance_listing.list_instances(installation) == (
        {"system_id": "active", "roles": {"pilot": "lion"}},
    )
    assert _snapshot(installation) == before


@pytest.mark.parametrize(
    "change",
    [
        "malformed_json",
        "unsupported_schema",
        "boolean_schema",
        "mismatched_system",
        "traversal_release",
        "empty_endpoints",
        "duplicate_role",
        "duplicate_endpoint",
        "robot_endpoint",
        "missing_role_image",
    ],
)
def test_rejects_malformed_or_inconsistent_instance_metadata(installation, change, capsys):
    release = _release(installation, {"pilot": "lion"})
    path = _register(installation, "alpha", release)
    value = json.loads(path.read_text(encoding="utf-8"))
    if change == "malformed_json":
        path.write_text("{", encoding="utf-8")
    else:
        if change == "unsupported_schema":
            value["schema_version"] = 1
        elif change == "boolean_schema":
            value["schema_version"] = True
        elif change == "mismatched_system":
            value["system_id"] = "other"
        elif change == "traversal_release":
            value["release_key"] = "../outside"
        elif change == "empty_endpoints":
            value["endpoints"] = []
        elif change == "duplicate_role":
            value["endpoints"].append({"role": "pilot", "endpoint_id": "other-pilot"})
        elif change == "duplicate_endpoint":
            value["endpoints"].append({"role": "ui", "endpoint_id": "alpha-pilot"})
        elif change == "robot_endpoint":
            value["endpoints"] = [{"role": "robot", "endpoint_id": "robot-1"}]
        elif change == "missing_role_image":
            value["endpoints"] = [{"role": "sim", "endpoint_id": "sim"}]
        path.write_text(json.dumps(value), encoding="utf-8")

    before = _snapshot(installation)
    assert instance_listing.main(["--prefix", str(installation), "--json"]) != 0
    output = capsys.readouterr()
    assert output.out == ""
    assert output.err.strip()
    assert _snapshot(installation) == before


def test_rejects_pinned_release_from_another_installation(installation, capsys):
    release = _release(installation, {"pilot": "lion"}, install_uuid=FOREIGN_INSTALL)
    _register(installation, "alpha", release)

    assert instance_listing.main(["--prefix", str(installation)]) != 0
    assert capsys.readouterr().err.strip()


def test_rejects_missing_pinned_manifest(installation, capsys):
    release = _release(installation, {"pilot": "lion"})
    _register(installation, "alpha", release)
    (installation / "releases" / release_key(release) / "manifest.json").unlink()

    assert instance_listing.main(["--prefix", str(installation)]) != 0
    assert capsys.readouterr().err.strip()


@pytest.mark.parametrize("ownership", ["native", "legacy"])
def test_rejects_ownership_without_a_scoped_container_install(installation, ownership, capsys):
    path = installation / "install-ownership.json"
    value = json.loads(path.read_text(encoding="utf-8"))
    if ownership == "native":
        value["docker"] = None
    else:
        value["docker"]["project"] = "elesim-runtime"
        value["docker"]["install_name"] = ""
    path.write_text(json.dumps(value), encoding="utf-8")
    before = _snapshot(installation)

    assert instance_listing.main(["--prefix", str(installation)]) != 0
    assert capsys.readouterr().err.strip()
    assert _snapshot(installation) == before


@pytest.mark.parametrize("unsafe", ["state_symlink", "state_hardlink", "registry_symlink", "release_symlink"])
def test_rejects_unsafe_metadata_paths(installation, unsafe, tmp_path, capsys):
    release = _release(installation, {"pilot": "lion"})
    path = _register(installation, "alpha", release)
    if unsafe == "state_symlink":
        target = tmp_path / "external-state.json"
        path.rename(target)
        path.symlink_to(target)
    elif unsafe == "state_hardlink":
        os.link(path, tmp_path / "external-state.json")
    elif unsafe == "registry_symlink":
        target = tmp_path / "external-instances"
        path.parent.parent.rename(target)
        path.parent.parent.symlink_to(target, target_is_directory=True)
    else:
        root = installation / "releases" / release_key(release)
        target = tmp_path / "external-release"
        root.rename(target)
        root.symlink_to(target, target_is_directory=True)

    assert instance_listing.main(["--prefix", str(installation)]) != 0
    output = capsys.readouterr()
    assert output.out == ""
    assert output.err.strip()


def test_missing_prefix_fails_without_creating_it(tmp_path, capsys):
    prefix = tmp_path / "missing"

    assert instance_listing.main(["--prefix", str(prefix)]) != 0
    assert capsys.readouterr().err.strip()
    assert not prefix.exists()


def test_copied_host_bundle_lists_without_the_installed_setup_package(installation, tmp_path):
    release = _release(installation, {"pilot": "lion"})
    _register(installation, "alpha", release)
    bundle = install_host_uninstaller_bundle(
        prefix=installation,
        bin_dir=installation / "bin",
    )
    before = _snapshot(installation)

    completed = subprocess.run(
        (
            sys.executable, "-B", "-S", "-m", "elesim_setup.instance_listing",
            "--prefix", str(installation), "--json",
        ),
        cwd=tmp_path,
        env={
            "PATH": "/usr/bin:/bin",
            "PYTHONPATH": str(bundle.root),
            "PYTHONNOUSERSITE": "1",
        },
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stderr == ""
    assert json.loads(completed.stdout) == {
        "instances": [{"system_id": "alpha", "roles": {"pilot": "lion"}}]
    }
    assert _snapshot(installation) == before
