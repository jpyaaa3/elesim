from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import elesim_setup.instance_remove as removal
from elesim_setup.instance_identity import container_name, project_name, service_key
from elesim_setup.instance_runtime import InstanceRuntime


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _manifest(prefix: Path, names: list[str], *, labels: dict[str, object] | None = None) -> None:
    prefix.mkdir(parents=True, exist_ok=True)
    payload = {
        "prefix": str(prefix),
        "install_uuid": INSTALL,
        "docker": {
            "install_uuid": INSTALL,
            "project": project_name(INSTALL),
            "context": "default",
            "engine_id": "engine",
            "containers": names,
        },
    }
    (prefix / "install-ownership.json").write_text(json.dumps(payload), encoding="utf-8")


def test_host_bridge_removes_only_exact_owned_target(monkeypatch, tmp_path: Path) -> None:
    system = "alpha"
    service = service_key(system, "alpha-pilot")
    name = container_name(INSTALL, service)
    aggregate = tmp_path / "compose.instances.yaml"
    aggregate.write_text("services: {}\n", encoding="utf-8")
    _manifest(tmp_path, [name])
    inspected = {
        "Name": f"/{name}",
        "Config": {
            "Labels": {
                "io.elesim.install_uuid": INSTALL,
                "com.docker.compose.project": project_name(INSTALL),
                "com.docker.compose.service": service,
                "io.elesim.system_id": system,
                "io.elesim.endpoint_id": "alpha-pilot",
                "io.elesim.role": "pilot",
                "com.docker.compose.project.config_files": str(aggregate),
            }
        },
    }
    calls: list[tuple[str, ...]] = []
    present = True

    def fake_run(command: tuple[str, ...]):
        nonlocal present
        calls.append(command)
        if command[-3:] == ("info", "--format", "{{.ID}}"):
            return SimpleNamespace(returncode=0, stdout="engine\n", stderr="")
        if "rm" in command:
            present = False
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    def fake_inspect(context: str, target: str):
        return inspected if present else None

    monkeypatch.setattr(removal, "_run", fake_run)
    monkeypatch.setattr(removal, "_inspect", fake_inspect)
    removal._verify_and_remove_containers(
        prefix=tmp_path,
        system=system,
        install_uuid=INSTALL,
        project=project_name(INSTALL),
        context="default",
        engine="engine",
        compose=aggregate,
        target=({"service": service, "role": "pilot", "endpoint_id": "alpha-pilot"},),
    )
    assert any(command[-2:] == ("-f", name) for command in calls)
    assert all("--all" not in command and "down" not in command for command in calls)


def test_host_bridge_rejects_foreign_exact_name_before_mutation(monkeypatch, tmp_path: Path) -> None:
    system = "alpha"
    service = service_key(system, "alpha-pilot")
    name = container_name(INSTALL, service)
    aggregate = tmp_path / "compose.instances.yaml"
    aggregate.write_text("services: {}\n", encoding="utf-8")
    _manifest(tmp_path, [name])
    foreign = {
        "Name": f"/{name}",
        "Config": {"Labels": {"io.elesim.install_uuid": "foreign"}},
    }
    monkeypatch.setattr(
        removal,
        "_run",
        lambda command: SimpleNamespace(returncode=0, stdout="engine\n", stderr=""),
    )
    monkeypatch.setattr(removal, "_inspect", lambda context, target: foreign)
    with pytest.raises(removal.InstanceRemovalError, match="foreign"):
        removal._verify_and_remove_containers(
            prefix=tmp_path,
            system=system,
            install_uuid=INSTALL,
            project=project_name(INSTALL),
            context="default",
            engine="engine",
            compose=aggregate,
            target=({"service": service, "role": "pilot", "endpoint_id": "alpha-pilot"},),
        )


def test_managed_turn_adds_only_target_system_coturn(tmp_path: Path) -> None:
    instances = tmp_path / "instances" / "alpha"
    instances.mkdir(parents=True)
    (instances / "state.json").write_text(
        json.dumps(
            {
                "schema_version": 2,
                "system_id": "alpha",
                "release_key": "a" * 64,
                "endpoints": [
                    {"role": "sim", "endpoint_id": "alpha-sim"},
                    {"role": "pilot", "endpoint_id": "alpha-pilot"},
                ],
                "turn": {"mode": "managed"},
            }
        ),
        encoding="utf-8",
    )
    target = removal._read_target(tmp_path, "alpha")
    assert {item["service"] for item in target} == {
        service_key("alpha", "alpha-sim"),
        service_key("alpha", "alpha-pilot"),
        service_key("alpha", "coturn"),
    }
    assert {item["endpoint_id"] for item in target} == {
        "alpha-sim", "alpha-pilot", "coturn"
    }


def test_runtime_rejects_arbitrary_or_unsafe_host_lease(tmp_path: Path) -> None:
    lock_root = tmp_path / "instances" / ".locks"
    lock_root.mkdir(parents=True)
    runtime = object.__new__(InstanceRuntime)
    runtime.lock_root = lock_root
    runtime.install_uuid = INSTALL
    token = "a" * 64
    with pytest.raises(PermissionError):
        runtime._validate_host_lease("alpha", token)
    lease = lock_root / "alpha.lease"
    lease.write_text(
        json.dumps({"version": 1, "token": token, "system_id": "alpha", "install_uuid": INSTALL}),
        encoding="utf-8",
    )
    lease.chmod(0o644)
    with pytest.raises(PermissionError, match="unsafe metadata"):
        runtime._validate_host_lease("alpha", token)
