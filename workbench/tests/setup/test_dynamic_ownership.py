from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

from elesim_setup.instance_identity import (
    container_name,
    manager_container_name,
    project_name,
    service_key,
)
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.ownership import (
    DockerOwnership,
    OwnershipError,
    OwnershipManifest,
    append_instance_docker_ownership,
    append_manager_docker_ownership,
    write_ownership_manifest,
)


UUID = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
ENGINE = "engine-123"


def _instance(system: str, endpoint: str = "pilot-1") -> InstanceState:
    return InstanceState(
        system_id=system,
        release_key="a" * 64,
        endpoints=(InstanceEndpoint("pilot", endpoint),),
        domain_id=10 + ord(system[-1]) - ord("a"),
    )


def _manifest(tmp_path: Path):
    prefix = tmp_path / "install"
    bin_dir = tmp_path / "bin"
    prefix.mkdir()
    bin_dir.mkdir()
    compose = prefix / "containers" / "compose.yaml"
    compose.parent.mkdir()
    compose.write_text("services: {}\n", encoding="utf-8")
    wrapper = bin_dir / "elesim-up"
    wrapper.write_text("#!/bin/sh\n", encoding="utf-8")
    wrapper.chmod(0o755)
    manifest = write_ownership_manifest(
        prefix=prefix,
        bin_dir=bin_dir,
        edition="general",
        inventory_roots=(compose,),
        managed_roots=(prefix / "containers",),
        created_roots=(prefix, bin_dir),
        wrapper_paths=(wrapper,),
        docker=DockerOwnership(
            install_uuid=UUID,
            compose_file=str(compose),
            project=project_name(UUID),
            containers=("elesim-manager",),
            local_images=(),
            context="desktop-linux",
            engine_id=ENGINE,
        ),
        install_uuid=UUID,
    )
    return manifest


def test_append_instance_container_is_exact_and_preserves_manifest(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    before = manifest.to_dict()
    expected = container_name(UUID, service_key("alpha", "pilot-1"))

    updated = append_instance_docker_ownership(
        manifest_path=manifest.path,
        install_uuid=UUID,
        project=project_name(UUID),
        docker_context="desktop-linux",
        docker_engine_id=ENGINE,
        instance=_instance("alpha"),
    )

    assert expected in updated.docker.containers
    after = updated.to_dict()
    assert {key: value for key, value in after.items() if key != "docker"} == {
        key: value for key, value in before.items() if key != "docker"
    }
    assert after["docker"]["local_images"] == before["docker"]["local_images"]
    assert after["docker"]["context"] == "desktop-linux"
    assert json.loads(manifest.path.read_text(encoding="utf-8"))["docker"]["containers"] == sorted(
        updated.docker.containers
    )


def test_append_is_scoped_and_pinned_only(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    with pytest.raises(OwnershipError, match="legacy"):
        append_instance_docker_ownership(
            manifest_path=manifest.path,
            install_uuid=UUID,
            project="elesim-runtime",
            docker_context="desktop-linux",
            docker_engine_id=ENGINE,
            instance=_instance("alpha"),
        )
    with pytest.raises(OwnershipError, match="foreign"):
        append_instance_docker_ownership(
            manifest_path=manifest.path,
            install_uuid=UUID,
            project=project_name(UUID),
            docker_context="other-context",
            docker_engine_id=ENGINE,
            instance=_instance("alpha"),
        )


def test_append_manager_uses_exact_system_identity(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    updated = append_manager_docker_ownership(
        manifest_path=manifest.path,
        install_uuid=UUID,
        project=project_name(UUID),
        docker_context="desktop-linux",
        docker_engine_id=ENGINE,
        system_id="lab_alpha",
    )
    assert manager_container_name(UUID, "lab_alpha") in updated.docker.containers
    assert manager_container_name(UUID, "manager") not in updated.docker.containers

    with pytest.raises(ValueError):
        append_manager_docker_ownership(
            manifest_path=manifest.path,
            install_uuid=UUID,
            project=project_name(UUID),
            docker_context="desktop-linux",
            docker_engine_id=ENGINE,
            system_id="Lab",
        )
def test_concurrent_appenders_do_not_lose_each_other(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    instances = (_instance("alpha"), _instance("bravo"), _instance("charlie"))

    def append(instance: InstanceState) -> None:
        append_instance_docker_ownership(
            manifest_path=manifest.path,
            install_uuid=UUID,
            project=project_name(UUID),
            docker_context="desktop-linux",
            docker_engine_id=ENGINE,
            instance=instance,
        )

    with ThreadPoolExecutor(max_workers=3) as pool:
        tuple(pool.map(append, instances))

    loaded = OwnershipManifest.load(manifest.path)
    assert loaded.docker is not None
    for instance in instances:
        expected = container_name(
            UUID, service_key(instance.system_id, instance.endpoints[0].endpoint_id)
        )
        assert expected in loaded.docker.containers


def test_append_rejects_symlinked_manifest_parent(tmp_path: Path) -> None:
    manifest = _manifest(tmp_path)
    alias = tmp_path / "alias"
    alias.symlink_to(Path(manifest.prefix), target_is_directory=True)
    with pytest.raises(OwnershipError):
        append_instance_docker_ownership(
            manifest_path=alias / "install-ownership.json",
            install_uuid=UUID,
            project=project_name(UUID),
            docker_context="desktop-linux",
            docker_engine_id=ENGINE,
            instance=_instance("alpha"),
        )
