from __future__ import annotations

import hashlib
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    DeploymentUnit,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)
from elesim_setup.connections import ConnectionDeploymentRunner, RuntimeRollbackError
from elesim_setup.instance_identity import image_reference, project_name
from elesim_setup.instances import InstanceEndpoint, InstanceState
from elesim_setup.releases import ReleaseManifest, release_key
from elesim_setup.state import InstallState


LOCAL_UUID = "01234567-89ab-cdef-0123-456789abcdef"
REMOTE_UUID = "11234567-89ab-cdef-0123-456789abcdef"


def _release(
    install_uuid: str, roles: tuple[str, ...], *, marker: str = "c"
) -> ReleaseManifest:
    return ReleaseManifest(
        install_uuid=install_uuid,
        source_revision="git-" + marker * 40,
        platform="linux/amd64",
        role_images={role: image_reference(install_uuid, role, marker * 64) for role in roles},
        image_ids={role: "sha256:" + marker * 64 for role in roles},
        build_fingerprints={role: marker * 64 for role in roles},
        runtime_data_digest=marker * 64,
    ).validate()


def test_scoped_multihost_planner_uses_each_units_release_and_host_dds(
    tmp_path: Path, monkeypatch
) -> None:
    local_root = tmp_path / "local"
    local_root.mkdir()
    local_release = _release(LOCAL_UUID, ("pilot",))
    remote_release = _release(REMOTE_UUID, ("sim", "ui"))
    local_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("pilot", "pilot-a"),),
        install_root=str(local_root),
        bin_dir=str(local_root / "bin"),
        install_uuid=LOCAL_UUID,
        project=project_name(LOCAL_UUID),
        release_key=release_key(local_release),
    )
    remote_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("sim", "sim-b"), RoleAssignment("ui", "ui-b")),
        install_root="/opt/elesim",
        bin_dir="/usr/local/bin",
        install_uuid=REMOTE_UUID,
        project=project_name(REMOTE_UUID),
        release_key=release_key(remote_release),
    )
    topology = ConnectionTopology(
        "scoped",
        "trusted-network",
        (
            ManagedHost(
                "local", True, DdsEndpoint("10.0.0.1", "eth0"), None,
                units=(local_unit,), install_root=str(local_root), bin_dir=str(local_root / "bin"),
            ),
            ManagedHost(
                "remote", False, DdsEndpoint("10.0.0.2", "tailscale0"),
                SshEndpoint("remote.example", 22, "operator", "", "SHA256:" + "A" * 43),
                units=(remote_unit,),
            ),
        ),
        dds_graph=DdsGraphSettings(discovery_mode="static"),
    ).validate()
    state = InstallState(
        profile="custom", roles=("pilot", "sim", "ui"), prefix=str(local_root),
        bin_dir=str(local_root / "bin"), source_root=str(tmp_path),
    )

    class Manifest:
        install_uuid = LOCAL_UUID
        prefix_path = local_root
        docker = SimpleNamespace(project=project_name(LOCAL_UUID))

    class RemoteOperation:
        def scoped_identity(self, _host, unit):
            return {"install_uuid": unit.install_uuid, "project": unit.project}

        def scoped_releases(self, _host, unit):
            return (remote_release.to_dict(),)

        def scoped_install_state(self, _host, _unit):
            return {"compute": {"gpu_mode": "cpu", "gpu_device": ""}}

        def scoped_instance_state(self, _host, _unit, _system):
            return None

        def close(self):
            pass

    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=local_root)
    monkeypatch.setattr("elesim_setup.connections.OwnershipManifest.load", lambda _path: Manifest())
    monkeypatch.setattr(
        "elesim_setup.connections.list_releases",
        lambda _prefix, install_uuid: (local_release,) if install_uuid == LOCAL_UUID else (remote_release,),
    )
    monkeypatch.setattr(runner, "_state_for_local_scope", lambda _uuid: state)
    fake_remote = RemoteOperation()
    monkeypatch.setattr(
        runner,
        "_operations",
        lambda _topology: {"local": fake_remote, "remote": fake_remote},
    )
    plans = runner._scoped_unit_plans(topology, runner._operations(topology))

    assert [plan[3].install_uuid for plan in plans] == [LOCAL_UUID, REMOTE_UUID]
    assert [plan[2].release_key for plan in plans] == [release_key(local_release), release_key(remote_release)]
    assert plans[0][2].interface == "eth0"
    assert plans[1][2].interface == "tailscale0"
    assert plans[1][2].static_peers == ("10.0.0.2", "10.0.0.1")
    for plan in plans:
        assert (
            plan[2].pilot_id,
            plan[2].sim_id,
            plan[2].ui_id,
        ) == ("pilot-a", "sim-b", "ui-b")


def test_scoped_identity_planner_requires_canonical_uuid_project_pair() -> None:
    unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("sim", "sim-main"),),
        install_uuid=REMOTE_UUID,
        project=project_name(REMOTE_UUID),
    )

    assert ConnectionDeploymentRunner._validate_scoped_identity(
        {"install_uuid": REMOTE_UUID, "project": project_name(REMOTE_UUID)},
        unit,
    ) == (REMOTE_UUID, project_name(REMOTE_UUID))

    with pytest.raises(ValueError, match="not derived from UUID"):
        ConnectionDeploymentRunner._validate_scoped_identity(
            {"install_uuid": REMOTE_UUID, "project": project_name(LOCAL_UUID)},
            unit,
        )
    with pytest.raises(ValueError, match="fields are invalid"):
        ConnectionDeploymentRunner._validate_scoped_identity(
            {
                "schema_version": 2,
                "install_uuid": REMOTE_UUID,
                "project": project_name(REMOTE_UUID),
            },
            unit,
        )


def _registration_topology(tmp_path: Path, security_profile: str) -> ConnectionTopology:
    """A two-host, two-unit topology used without any live host adapters."""

    local_root = tmp_path / "local"
    local_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("pilot", "pilot-local"),),
        install_root=str(local_root),
        bin_dir=str(local_root / "bin"),
    )
    remote_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("sim", "sim-remote"), RoleAssignment("ui", "ui-remote")),
    )
    return ConnectionTopology(
        "scoped",
        security_profile,
        (
            ManagedHost(
                "local",
                True,
                DdsEndpoint("10.0.0.1", "eth0"),
                None,
                units=(local_unit,),
            ),
            ManagedHost(
                "remote",
                False,
                DdsEndpoint("10.0.0.2", "eth0"),
                SshEndpoint(
                    "remote.example",
                    22,
                    "operator",
                    "",
                    "SHA256:" + "A" * 43,
                ),
                units=(remote_unit,),
            ),
        ),
        dds_graph=DdsGraphSettings(discovery_mode="static"),
    ).validate()


def _registration_plans(
    topology: ConnectionTopology,
    current_release: ReleaseManifest,
    *,
    previous: InstanceState | None = None,
    previous_release: ReleaseManifest | None = None,
) -> list[
    tuple[
        ManagedHost,
        DeploymentUnit,
        InstanceState,
        ReleaseManifest,
        InstanceState | None,
        ReleaseManifest | None,
    ]
]:
    plans = []
    for host in topology.hosts:
        unit = host.primary_unit
        instance = InstanceState(
            topology.system_id,
            release_key(current_release),
            tuple(InstanceEndpoint(a.role, a.endpoint_id) for a in unit.assignments),
            0,
            discovery_mode="static",
            static_peers=topology.discovery_peers(host.host_id),
            interface=host.dds.interface,
            security_profile=topology.security_profile,
        )
        local_previous = previous if host.host_id == "local" else None
        plans.append(
            (
                host,
                unit,
                instance,
                current_release,
                local_previous,
                previous_release if local_previous is not None else None,
            )
        )
    return plans


class _RegistrationOperations:
    def __init__(self, events: list[tuple[str, str]], *, fail_host: str | None = None) -> None:
        self.events = events
        self.fail_host = fail_host

    def prepare_runtime_network(self, host, _output):
        self.events.append(("network", host.host_id))
        return None

    def register_scoped_instance(
        self, host, _unit, instance, release, _bundle=None, *, replace_existing=False
    ):
        self.events.append(("register", host.host_id))
        if host.host_id == self.fail_host:
            raise RuntimeError(f"register failed on {host.host_id}")

    def remove_scoped_instance(self, host, _unit, system_id):
        self.events.append(("remove", host.host_id))

    def status(self, _host):
        return {"state": "stopped", "running_roles": []}

    def close(self):
        pass


def _patch_scoped_runner(
    runner: ConnectionDeploymentRunner,
    topology: ConnectionTopology,
    plans,
    operations,
    monkeypatch,
) -> None:
    monkeypatch.setattr(runner, "_local_install_scope", lambda: True)
    monkeypatch.setattr(runner, "_scoped_unit_plans", lambda _topology, _operations: plans)
    monkeypatch.setattr(runner, "_operations", lambda _topology: operations)


def test_scoped_multihost_network_is_prepared_on_every_host_before_registration(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events: list[tuple[str, str]] = []
    operations = {
        host.host_id: _RegistrationOperations(events) for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(
        runner, topology, _registration_plans(topology, release), operations, monkeypatch
    )

    runner(topology, "deploy", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("register", "local"),
        ("register", "remote"),
    ]


def test_scoped_multihost_failure_removes_fresh_prior_registration(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events: list[tuple[str, str]] = []
    operations = {
        host.host_id: _RegistrationOperations(events, fail_host="remote")
        for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(
        runner, topology, _registration_plans(topology, release), operations, monkeypatch
    )

    with pytest.raises(RuntimeError, match="register failed on remote"):
        runner(topology, "deploy", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("register", "local"),
        ("register", "remote"),
        ("remove", "local"),
    ]


def test_scoped_authority_activation_waits_for_every_host_registration(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events: list[tuple[str, str]] = []

    class Authority:
        def __init__(self, _root):
            pass

        @staticmethod
        def active():
            return None

    class Bundle:
        generation = "g-20260909t000000000000z-abcdef123456"

        def for_roles(self, _roles):
            return self

    class Issued:
        generation = "g-20260909t000000000000z-abcdef123456"
        bundles = {host.host_id: Bundle() for host in topology.hosts}

        def activate_authority(self):
            events.append(("authority", "activate"))

        def rollback_authority(self):
            events.append(("authority", "rollback"))

    class Issuer:
        def __init__(self, _authority):
            pass

        def issue(self, _topology, _generation):
            return Issued()

    operations = {
        host.host_id: _RegistrationOperations(events) for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(
        runner, topology, _registration_plans(topology, release), operations, monkeypatch
    )
    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_setup.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr(
        "elesim_setup.connections.new_generation_id",
        lambda: Issued.generation,
    )

    runner(topology, "deploy", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("register", "local"),
        ("register", "remote"),
        ("authority", "activate"),
    ]


def test_scoped_replacement_rollback_restores_prior_release_and_security(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    prior = InstanceState(
        "scoped",
        release_key(old_release),
        (InstanceEndpoint("pilot", "pilot-local"),),
        7,
        discovery_mode="static",
        static_peers=topology.discovery_peers("local"),
        interface="eth0",
        security_profile="trusted-network",
        security_generation="prior-generation",
    )
    calls: list[tuple[InstanceState, ReleaseManifest | None, bool]] = []

    class Operations(_RegistrationOperations):
        def register_scoped_instance(
            self, host, unit, instance, release, bundle=None, *, replace_existing=False
        ):
            calls.append((instance, release, replace_existing))
            super().register_scoped_instance(
                host, unit, instance, release, bundle,
                replace_existing=replace_existing,
            )

    events: list[tuple[str, str]] = []
    operations = {
        host.host_id: Operations(events, fail_host="remote") for host in topology.hosts
    }
    plans = _registration_plans(
        topology,
        new_release,
        previous=prior,
        previous_release=old_release,
    )
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)

    with pytest.raises(RuntimeError, match="register failed on remote"):
        runner(topology, "deploy", lambda _message: None)

    assert calls[0] == (plans[0][2], new_release, True)
    # This is intentionally an ordinary regression assertion: the current
    # runner passes release=None during compensation, so it cannot restore a
    # prior release manifest when replacement crosses release generations.
    assert calls[-1] == (prior, old_release, True)


def test_scoped_rollback_failure_is_reported_with_original_failure(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events: list[tuple[str, str]] = []

    class Operations(_RegistrationOperations):
        def remove_scoped_instance(self, host, _unit, _system_id):
            events.append(("remove", host.host_id))
            raise RuntimeError("cleanup failed on local")

    operations = {
        host.host_id: Operations(events, fail_host="remote") for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(
        runner, topology, _registration_plans(topology, release), operations, monkeypatch
    )

    with pytest.raises(RuntimeError, match="cleanup failed on local"):
        runner(topology, "deploy", lambda _message: None)


def _scoped_prior(
    topology: ConnectionTopology,
    host: ManagedHost,
    release: ReleaseManifest,
    generation: str,
) -> InstanceState:
    unit = host.primary_unit
    return InstanceState(
        topology.system_id,
        release_key(release),
        tuple(InstanceEndpoint(a.role, a.endpoint_id) for a in unit.assignments),
        0,
        discovery_mode=topology.dds_graph.discovery_mode,
        static_peers=topology.discovery_peers(host.host_id),
        interface=host.dds.interface,
        security_profile=topology.security_profile,
        security_generation=generation,
    )


def _rotation_plans(
    topology: ConnectionTopology,
    current_release: ReleaseManifest,
    previous_release: ReleaseManifest,
) -> list[
    tuple[
        ManagedHost,
        DeploymentUnit,
        InstanceState,
        ReleaseManifest,
        InstanceState,
        ReleaseManifest,
    ]
]:
    plans = []
    for host in topology.hosts:
        unit = host.primary_unit
        instance = InstanceState(
            topology.system_id,
            release_key(current_release),
            tuple(InstanceEndpoint(a.role, a.endpoint_id) for a in unit.assignments),
            0,
            discovery_mode=topology.dds_graph.discovery_mode,
            static_peers=topology.discovery_peers(host.host_id),
            interface=host.dds.interface,
            security_profile=topology.security_profile,
        )
        previous = _scoped_prior(
            topology,
            host,
            previous_release,
            f"old-{host.host_id}-generation",
        )
        plans.append((host, unit, instance, current_release, previous, previous_release))
    return plans


class _RotationOperations(_RegistrationOperations):
    def __init__(
        self,
        events: list[tuple[str, str]],
        calls: list[tuple[str, InstanceState, ReleaseManifest, object | None, bool]],
        *,
        fail_host: str | None = None,
        running_hosts: set[str] | None = None,
        fail_restore: bool = False,
    ) -> None:
        super().__init__(events, fail_host=fail_host)
        self.calls = calls
        self.running_hosts = running_hosts or set()
        self.fail_restore = fail_restore

    def status(self, host):
        self.events.append(("status", host.host_id))
        if host.host_id in self.running_hosts:
            return {"state": "running", "running_roles": list(host.roles)}
        return {"state": "stopped", "running_roles": []}

    def register_scoped_instance(
        self, host, _unit, instance, release, bundle=None, *, replace_existing=False
    ):
        self.calls.append((host.host_id, instance, release, bundle, replace_existing))
        restoring = str(instance.security_generation).startswith("old-")
        self.events.append(("restore" if restoring else "register", host.host_id))
        if restoring and self.fail_restore:
            raise RuntimeError(f"restore failed on {host.host_id}")
        if not restoring and host.host_id == self.fail_host:
            raise RuntimeError(f"register failed on {host.host_id}")


def _patch_rotation_security(monkeypatch, topology: ConnectionTopology, events):
    generation = "g-20260909t000000000000z-rotation123456"

    class Authority:
        def __init__(self, _root):
            pass

        @staticmethod
        def active():
            return object()

    class Bundle:
        def __init__(self):
            self.generation = generation

        def for_roles(self, _roles):
            return self

    class Issued:
        bundles = {host.host_id: Bundle() for host in topology.hosts}

        def activate_authority(self):
            events.append(("authority", "activate"))

        def rollback_authority(self):
            events.append(("authority", "rollback"))

    Issued.generation = generation

    class Issuer:
        def __init__(self, _authority):
            pass

        def issue(self, _topology, requested_generation):
            assert requested_generation == generation
            events.append(("authority", "issue"))
            return Issued()

    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_setup.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr("elesim_setup.connections.new_generation_id", lambda: generation)
    return generation


def test_scoped_rotation_requires_an_existing_instance_on_every_unit(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    local_prior = _scoped_prior(
        topology, topology.hosts[0], old_release, "old-local-generation"
    )
    plans = _registration_plans(
        topology,
        new_release,
        previous=local_prior,
        previous_release=old_release,
    )
    events: list[tuple[str, str]] = []
    operations = {
        host.host_id: _RegistrationOperations(events) for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)
    _patch_rotation_security(monkeypatch, topology, events)

    with pytest.raises(RuntimeError, match="existing instance on every unit: remote/runtime"):
        runner(topology, "rotate", lambda _message: None)

    assert events == [("network", "local"), ("network", "remote")]


def test_scoped_rotation_requires_all_targets_stopped_before_issuing_generation(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    plans = _rotation_plans(topology, new_release, old_release)
    events: list[tuple[str, str]] = []
    calls: list[tuple[str, InstanceState, ReleaseManifest, object | None, bool]] = []
    operations = {
        host.host_id: _RotationOperations(
            events, calls, running_hosts={"remote"}
        )
        for host in topology.hosts
    }
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)
    _patch_rotation_security(monkeypatch, topology, events)

    with pytest.raises(RuntimeError, match="not stopped on remote"):
        runner(topology, "rotate", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("status", "local"),
        ("status", "remote"),
    ]
    assert calls == []


def test_scoped_rotation_replaces_every_unit_then_activates_authority(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    plans = _rotation_plans(topology, new_release, old_release)
    events: list[tuple[str, str]] = []
    calls: list[tuple[str, InstanceState, ReleaseManifest, object | None, bool]] = []
    operations = {
        host.host_id: _RotationOperations(events, calls)
        for host in topology.hosts
    }
    generation = _patch_rotation_security(monkeypatch, topology, events)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)

    runner(topology, "rotate", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("status", "local"),
        ("status", "remote"),
        ("authority", "issue"),
        ("register", "local"),
        ("register", "remote"),
        ("authority", "activate"),
    ]
    assert len(calls) == 2
    for host_id, instance, release, bundle, replace_existing in calls:
        assert host_id in {"local", "remote"}
        assert instance.security_generation == generation
        assert release == new_release
        assert bundle is not None
        assert bundle.generation == generation
        assert replace_existing is True


def test_scoped_rotation_failure_restores_prior_state_release_and_generation(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    plans = _rotation_plans(topology, new_release, old_release)
    events: list[tuple[str, str]] = []
    calls: list[tuple[str, InstanceState, ReleaseManifest, object | None, bool]] = []
    operations = {
        host.host_id: _RotationOperations(events, calls, fail_host="remote")
        for host in topology.hosts
    }
    _patch_rotation_security(monkeypatch, topology, events)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)

    with pytest.raises(RuntimeError, match="register failed on remote"):
        runner(topology, "rotate", lambda _message: None)

    assert events == [
        ("network", "local"),
        ("network", "remote"),
        ("status", "local"),
        ("status", "remote"),
        ("authority", "issue"),
        ("register", "local"),
        ("register", "remote"),
        ("authority", "rollback"),
        ("restore", "local"),
    ]
    _host, restored, release, bundle, replace_existing = calls[-1]
    assert restored == plans[0][4]
    assert restored.security_generation == "old-local-generation"
    assert release == old_release
    assert bundle is None
    assert replace_existing is True


def test_scoped_rotation_surfaces_compensation_failure(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    plans = _rotation_plans(topology, new_release, old_release)
    events: list[tuple[str, str]] = []
    calls: list[tuple[str, InstanceState, ReleaseManifest, object | None, bool]] = []
    operations = {
        host.host_id: _RotationOperations(
            events,
            calls,
            fail_host="remote",
            fail_restore=host.host_id == "local",
        )
        for host in topology.hosts
    }
    _patch_rotation_security(monkeypatch, topology, events)
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    _patch_scoped_runner(runner, topology, plans, operations, monkeypatch)

    with pytest.raises(RuntimeRollbackError) as captured:
        runner(topology, "rotate", lambda _message: None)

    assert "register failed on remote" in str(captured.value.cause)
    assert captured.value.rollback_action == "scoped registration restore"
    assert [(host_id, str(error)) for host_id, error in captured.value.rollback_errors] == [
        ("local/runtime", "restore failed on local")
    ]
    assert ("authority", "rollback") in events


@pytest.mark.parametrize(
    ("security_profile", "action"),
    (("trusted-network", "deploy"), ("sros2", "provision")),
)
def test_scoped_actions_reject_native_robot_unit_explicitly(
    tmp_path: Path, monkeypatch, security_profile: str, action: str
) -> None:
    prefix = tmp_path / "install"
    prefix.mkdir()
    runtime = DeploymentUnit(
        "runtime",
        (RoleAssignment("pilot", "pilot-local"),),
        install_root=str(prefix),
        bin_dir=str(prefix / "bin"),
    )
    robot = DeploymentUnit(
        "robot-native",
        (RoleAssignment("robot", "robot-local"),),
        install_mode="native",
        lifecycle="systemd",
        install_root=str(prefix),
        bin_dir=str(prefix / "bin"),
    )
    topology = ConnectionTopology(
        "scoped",
        security_profile,
        (
            ManagedHost(
                "local",
                True,
                DdsEndpoint("10.0.0.1", "eth0"),
                None,
                units=(runtime,),
            ),
            ManagedHost(
                "robot-host",
                False,
                DdsEndpoint("10.0.0.2", "eth0"),
                SshEndpoint(
                    "robot.example",
                    22,
                    "operator",
                    "",
                    "SHA256:" + "A" * 43,
                ),
                units=(robot,),
                jetson=True,
            ),
        ),
    ).validate()
    events: list[tuple[str, str]] = []

    class Operations:
        def prepare_runtime_network(self, host, _output):
            events.append(("network", host.host_id))

        def close(self):
            pass

    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=prefix
    )
    monkeypatch.setattr(runner, "_local_install_scope", lambda: True)
    monkeypatch.setattr(
        runner,
        "_operations",
        lambda _topology: {
            "local": Operations(),
            "robot-host": Operations(),
        },
    )

    with pytest.raises(
        ValueError,
        match=r"cannot silently skip native Robot units: robot-host/robot-native",
    ):
        runner(topology, action, lambda _message: None)

    assert events == [("network", "local"), ("network", "robot-host")]


def _recovery_journal(
    runner: ConnectionDeploymentRunner,
    topology: ConnectionTopology,
    plans,
    *,
    action: str,
    authority_before: str | None,
    authority_target: str | None,
    authority_observed: str | None,
    bundle_digest: str = "",
) -> dict[str, object]:
    journal = runner._new_scoped_journal(action, topology)
    journal.update(
        {
            "transaction_id": "txn-recovery-test",
            "authority": {
                "before": authority_before,
                "target": authority_target,
                "observed": authority_observed,
            },
            "units": [
                {
                    "host_id": host.host_id,
                    "unit_id": unit.unit_id,
                    "install_uuid": unit.install_uuid,
                    "project": unit.project,
                    "roles": list(unit.roles),
                    "before": None if before is None else before.to_dict(),
                    "target": target.to_dict(),
                    "before_release_key": (
                        None if previous_release is None else release_key(previous_release)
                    ),
                    "target_release_key": release_key(release),
                    "bundle_manifest_sha256": bundle_digest,
                    "status": "registering",
                    "last_error": None,
                }
                for host, unit, target, release, before, previous_release in plans
            ],
        }
    )
    return journal


class _RecoveryOperations:
    def __init__(self, states, releases, events):
        self.states = states
        self.releases = releases
        self.events = events

    def scoped_releases(self, host, _unit):
        return self.releases[host.host_id]

    def scoped_instance_state(self, host, _unit, _system_id):
        return self.states[host.host_id]

    def register_scoped_instance(
        self, host, _unit, instance, _release, _bundle=None, *, replace_existing=False
    ):
        self.events.append(("register", host.host_id, replace_existing))
        self.states[host.host_id] = instance

    def remove_scoped_instance(self, host, _unit, _system_id):
        self.events.append(("remove", host.host_id))
        self.states[host.host_id] = None

    def status(self, _host):
        return {"state": "stopped", "running_roles": []}


class _RecoveryAuthority:
    def __init__(self, root: Path, observed: str | None):
        self.root = root
        self.observed = observed

    def active(self):
        if self.observed is None:
            return None
        return SimpleNamespace(generation=self.observed)

    def generation_metadata(self, generation):
        return {"system_id": "scoped", "generation": generation}


def test_scoped_recovery_rolls_back_when_authority_is_before_or_null(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    base_plans = _registration_plans(topology, new_release)
    plans = [
        (
            host,
            unit,
            replace(target, security_generation="target-generation"),
            release,
            before,
            previous_release,
        )
        for host, unit, target, release, before, previous_release in base_plans
    ]
    # Keep both current endpoints on the target side to emulate a crash after
    # all host writes but before the Authority activation/commit boundary.
    states = {
        host.host_id: target for host, _unit, target, _release_, _before, _prior in plans
    }
    events: list[tuple] = []
    operations = _RecoveryOperations(
        states,
        {host.host_id: (new_release,) for host in topology.hosts},
        events,
    )
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    journal = _recovery_journal(
        runner,
        topology,
        plans,
        action="provision",
        authority_before=None,
        authority_target="target-generation",
        authority_observed=None,
        bundle_digest="a" * 64,
    )
    monkeypatch.setattr(
        "elesim_setup.connections.Sros2Authority",
        lambda root: _RecoveryAuthority(root, None),
    )

    runner._recover_scoped_transaction(
        topology,
        {host.host_id: operations for host in topology.hosts},
        journal,
        lambda _message: None,
    )

    assert [event[:2] for event in events] == [
        ("remove", "local"),
        ("remove", "remote"),
    ]
    assert all(
        states[host.host_id] == before
        for host, _unit, _target, _release_, before, _prior in plans
    )
    assert journal["status"] == "completed"


def test_scoped_recovery_forward_completes_when_target_authority_is_active(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "sros2")
    old_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="a")
    new_release = _release(LOCAL_UUID, ("pilot", "sim", "ui"), marker="e")
    base_plans = _rotation_plans(topology, new_release, old_release)
    plans = [
        (
            host,
            unit,
            replace(target, security_generation="target-generation"),
            release,
            replace(before, security_generation="old-generation"),
            previous_release,
        )
        for host, unit, target, release, before, previous_release in base_plans
    ]
    states = {
        host.host_id: before for host, _unit, _target, _release_, before, _prior in plans
    }
    events: list[tuple] = []
    operations = _RecoveryOperations(
        states,
        {host.host_id: (old_release, new_release) for host in topology.hosts},
        events,
    )
    class Bundle:
        @classmethod
        def from_directory(cls, **_kwargs):
            return cls()

        def for_roles(self, _roles):
            return self

        @staticmethod
        def manifest_bytes():
            return b"recovery-bundle"

    digest = hashlib.sha256(b"recovery-bundle").hexdigest()
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    journal = _recovery_journal(
        runner,
        topology,
        plans,
        action="rotate",
        authority_before="old-generation",
        authority_target="target-generation",
        authority_observed="target-generation",
        bundle_digest=digest,
    )
    monkeypatch.setattr(
        "elesim_setup.connections.Sros2Authority",
        lambda root: _RecoveryAuthority(root, "target-generation"),
    )
    monkeypatch.setattr("elesim_setup.connections.SecurityBundle", Bundle)

    runner._recover_scoped_transaction(
        topology,
        {host.host_id: operations for host in topology.hosts},
        journal,
        lambda _message: None,
    )

    assert [event[:2] for event in events] == [
        ("register", "local"),
        ("register", "remote"),
    ]
    assert all(
        states[host.host_id] == target
        for host, _unit, target, _release_, _before, _prior in plans
    )
    assert journal["status"] == "completed"


def test_scoped_recover_does_not_prepare_or_persist_network_changes(
    tmp_path: Path, monkeypatch
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    plans = _registration_plans(topology, release)
    states = {host.host_id: None for host in topology.hosts}
    events: list[tuple] = []
    operations = _RecoveryOperations(
        states,
        {host.host_id: (release,) for host in topology.hosts},
        events,
    )

    def network_must_not_run(_host, _output):
        raise AssertionError("recover must not prepare runtime network")

    operations.prepare_runtime_network = network_must_not_run
    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=tmp_path / "local"
    )
    journal = _recovery_journal(
        runner,
        topology,
        plans,
        action="deploy",
        authority_before=None,
        authority_target=None,
        authority_observed=None,
    )
    runner._write_transaction_journal(topology, journal)
    monkeypatch.setattr(runner, "_local_install_scope", lambda: True)
    monkeypatch.setattr(
        runner,
        "_operations",
        lambda _topology: {host.host_id: operations for host in topology.hosts},
    )

    runner(topology, "recover", lambda _message: None)

    assert events == []


def test_scoped_journal_validation_rejects_unknown_fields_even_if_terminal(
    tmp_path: Path,
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    journal = _recovery_journal(
        runner,
        topology,
        _registration_plans(topology, release),
        action="deploy",
        authority_before=None,
        authority_target=None,
        authority_observed=None,
    )
    journal["status"] = "completed"
    journal["unexpected"] = True
    runner._write_transaction_journal(topology, journal)

    with pytest.raises(RuntimeError, match="fields are invalid"):
        runner._refuse_unresolved_scoped_journal(topology)


def test_scoped_unresolved_failed_journal_refuses_new_deployment(
    tmp_path: Path,
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    journal = _recovery_journal(
        runner,
        topology,
        _registration_plans(topology, release),
        action="deploy",
        authority_before=None,
        authority_target=None,
        authority_observed=None,
    )
    journal["status"] = "failed"
    runner._write_transaction_journal(topology, journal)

    with pytest.raises(RuntimeError, match="unresolved scoped transaction journal"):
        runner._refuse_unresolved_scoped_journal(topology)


def test_scoped_transaction_lock_serializes_same_system_managers(
    tmp_path: Path,
) -> None:
    topology = _registration_topology(tmp_path, "trusted-network")
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    first = runner._acquire_scoped_transaction_lock(topology)
    acquired: list[int] = []

    def contender() -> None:
        descriptor = runner._acquire_scoped_transaction_lock(topology)
        acquired.append(descriptor)
        runner._release_scoped_transaction_lock(descriptor)

    thread = threading.Thread(target=contender)
    thread.start()
    time.sleep(0.05)
    assert acquired == []
    runner._release_scoped_transaction_lock(first)
    thread.join(timeout=2)
    assert not thread.is_alive()
    assert len(acquired) == 1
