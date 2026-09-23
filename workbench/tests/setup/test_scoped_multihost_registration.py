from __future__ import annotations

import hashlib
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from elesim_connections.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    DeploymentUnit,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)
from elesim_connections.connections import (
    ConnectionDeploymentRunner,
    HostActivationState,
    RuntimeRollbackError,
)
from elesim_connections.secure_deployment import ScopedInstanceNotRegisteredError
from elesim_setup.instance_identity import image_reference, project_name
from elesim_setup.instances import InstanceEndpoint, InstanceRegistry, InstanceState
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
    local_previous = InstanceState(
        system_id="scoped",
        release_key=release_key(local_release),
        endpoints=(InstanceEndpoint("pilot", "pilot-a"),),
        domain_id=0,
        pilot_id="pilot-a",
        discovery_mode="static",
        interface="eth0",
        security_profile="trusted-network",
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

        def scoped_instance_state(self, host, _unit, _system):
            return local_previous if host.host_id == "local" else None

        def close(self):
            pass

    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=local_root)
    monkeypatch.setattr("elesim_connections.connections.OwnershipManifest.load", lambda _path: Manifest())
    monkeypatch.setattr(
        "elesim_connections.connections.list_releases",
        lambda _prefix, install_uuid: (local_release,) if install_uuid == LOCAL_UUID else (remote_release,),
    )
    monkeypatch.setattr(
        InstanceRegistry,
        "load",
        lambda *_args, **_kwargs: pytest.fail(
            "planner must not acquire the read-only manager-side instance lock"
        ),
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
    assert plans[0][4] == local_previous
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


class _BootOperations(_RegistrationOperations):
    def __init__(self, events, **kwargs):
        super().__init__(events, **kwargs)
        self.registered = []

    def runtime_inventory(self, host):
        return {"gpu_policy": {role: {"mode": "inherit", "device": ""} for role in host.roles if role in {"pilot", "sim"}}}

    def runtime_network_check(self, host):
        self.events.append(("network-check", host.host_id))

    def preflight(self, host):
        return SimpleNamespace(require_for=lambda *args, **kwargs: None)

    def runtime_launch_preflight(self, host):
        self.events.append(("preflight", host.host_id))

    def build(self, host, output):
        self.events.append(("build", host.host_id))

    def launch(self, host):
        self.events.append(("launch", host.host_id))

    def register_scoped_instance(self, host, unit, instance, release, bundle=None, **kwargs):
        super().register_scoped_instance(host, unit, instance, release, bundle, **kwargs)
        self.registered.append(instance)


@pytest.mark.parametrize("fail_host", [None, "remote"])
def test_prepare_defers_registration_and_start_persists_selected_options(tmp_path, monkeypatch, fail_host):
    from elesim_connections.secure_deployment import RuntimeLaunchOptions

    topology = _registration_topology(tmp_path, "trusted-network")
    local, remote = topology.hosts
    local = replace(local, units=(replace(local.primary_unit, install_uuid=LOCAL_UUID, project=project_name(LOCAL_UUID)),))
    topology = replace(topology, hosts=(local, remote))
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events = []
    operations = {host.host_id: _BootOperations(events, fail_host=fail_host) for host in topology.hosts}
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(runner, topology, _registration_plans(topology, release), operations, monkeypatch)
    monkeypatch.setattr(runner, "_report_runtime_readiness", lambda *args: None)

    runner(topology, "prepare", lambda message: None)
    assert not any(event == "register" for event, host in events)
    events.clear()
    runner.set_runtime_launch_options(RuntimeLaunchOptions.from_payload({
        "pilot_gpu_inherit": True, "pilot_gpu_device": "0",
        "sim_gpu_inherit": True, "sim_gpu_device": "1", "viewer": True,
    }))
    if fail_host:
        with pytest.raises(RuntimeError, match="register failed"):
            runner(topology, "start", lambda message: None)
        assert not any(event == "launch" for event, host in events)
        assert ("remove", "local") in events
    else:
        runner(topology, "start", lambda message: None)
        last_register = max(i for i, (event, host) in enumerate(events) if event == "register")
        first_preflight = min(i for i, (event, host) in enumerate(events) if event == "preflight")
        first_launch = min(i for i, (event, host) in enumerate(events) if event == "launch")
        assert last_register < first_preflight < first_launch
        for operation in operations.values():
            for instance in operation.registered:
                for endpoint in instance.endpoints:
                    if endpoint.role in {"pilot", "sim"}:
                        assert instance.role_compute[endpoint.role].gpu_device == ("0" if endpoint.role == "pilot" else "1")
                assert instance.viewer == any(endpoint.role == "sim" for endpoint in instance.endpoints)
        # A repeated start with the same choices must not replace registrations.
        plans = _registration_plans(topology, release)
        plans = [(host, unit, target, manifest, operations[host.host_id].registered[-1], manifest)
                 for host, unit, target, manifest, _, _ in plans]
        monkeypatch.setattr(runner, "_scoped_unit_plans", lambda *args: plans)
        events.clear()
        runner.set_runtime_launch_options(RuntimeLaunchOptions.from_payload({
            "pilot_gpu_inherit": True, "pilot_gpu_device": "0",
            "sim_gpu_inherit": True, "sim_gpu_device": "1", "viewer": True,
        }))
        runner(topology, "start", lambda message: None)
        assert not any(event == "register" for event, host in events)
        assert sum(event == "launch" for event, host in events) == len(topology.hosts)


def test_gpu_inventory_is_available_before_instance_registration(tmp_path, monkeypatch):
    topology = _registration_topology(tmp_path, "trusted-network")
    runner = ConnectionDeploymentRunner(tmp_path / "authority")

    class Unregistered(_BootOperations):
        def status(self, host):
            raise ScopedInstanceNotRegisteredError("instance is not registered")

    monkeypatch.setattr(runner, "_operations", lambda topology: {host.host_id: Unregistered([]) for host in topology.hosts})
    result = runner.runtime_status(topology)
    assert all(host["inventory_ready"] for host in result["hosts"])
    assert all(host["reachable"] for host in result["hosts"])
    assert all(not host["registered"] for host in result["hosts"])
    assert all(host["state"] == "unregistered" for host in result["hosts"])
    assert any(host["gpu_policy"] for host in result["hosts"])


def test_runtime_status_preserves_partial_registration(tmp_path, monkeypatch):
    topology = _registration_topology(tmp_path, "trusted-network")
    runner = ConnectionDeploymentRunner(tmp_path / "authority")

    class Partial(_BootOperations):
        def status(self, host):
            return {"state": "degraded", "registered": False,
                    "running_roles": list(host.roles[:1])}

    monkeypatch.setattr(runner, "_operations", lambda topology: {
        host.host_id: Partial([]) for host in topology.hosts
    })
    result = runner.runtime_status(topology)
    assert all(host["reachable"] and not host["registered"] for host in result["hosts"])
    assert all(host["running_roles"] for host in result["hosts"])


@pytest.mark.parametrize("active_authority", [False, True])
def test_managed_start_registers_before_launch_not_during_prepare(tmp_path, monkeypatch, active_authority):
    topology = _registration_topology(tmp_path, "sros2")
    release = _release(LOCAL_UUID, ("pilot", "sim", "ui"))
    events = []
    operations = {host.host_id: _BootOperations(events) for host in topology.hosts}
    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=tmp_path / "local")
    _patch_scoped_runner(runner, topology, _registration_plans(topology, release), operations, monkeypatch)
    generation = _patch_rotation_security(monkeypatch, topology, events)
    if not active_authority:
        monkeypatch.setattr("elesim_connections.connections.Sros2Authority.active", lambda self: None)
    monkeypatch.setattr(runner, "_report_runtime_readiness", lambda *args: None)

    runner(topology, "prepare", lambda message: None)
    assert not any(event in {"register", "authority", "launch"} for event, _ in events)
    runner(topology, "start", lambda message: None)
    last_register = max(i for i, (event, _) in enumerate(events) if event == "register")
    first_launch = min(i for i, (event, _) in enumerate(events) if event == "launch")
    assert last_register < first_launch
    assert ("authority", "activate") in events
    assert all(instance.security_generation == generation
               for operation in operations.values() for instance in operation.registered)


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
    monkeypatch.setattr("elesim_connections.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_connections.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr(
        "elesim_connections.connections.new_generation_id",
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

    monkeypatch.setattr("elesim_connections.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_connections.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr("elesim_connections.connections.new_generation_id", lambda: generation)
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
@pytest.mark.parametrize("failure", [None, "native-apply", "register", "rollback"])
def test_scoped_actions_include_native_robot_transaction(
    tmp_path: Path, monkeypatch, security_profile: str, action: str, failure
) -> None:
    prefix = tmp_path / "install"
    runtime = DeploymentUnit(
        "runtime",
        (RoleAssignment("pilot", "pilot-local"),),
        install_uuid=LOCAL_UUID,
        project=project_name(LOCAL_UUID),
        install_root=str(prefix),
        bin_dir=str(prefix / "bin"),
    )
    robot = DeploymentUnit(
        "robot-native",
        (RoleAssignment("robot", "robot-local"),),
        install_mode="native",
        lifecycle="systemd",
        install_uuid=REMOTE_UUID,
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
    from elesim_connections.secure_deployment import HostActivationState, RemoteCapabilities
    from elesim_setup.state import DdsSettings

    release = _release(LOCAL_UUID, ("pilot",))
    local = topology.host("local")
    instance = InstanceState(
        "scoped",
        release_key(release),
        (InstanceEndpoint("pilot", "pilot-local"),),
        0,
        security_profile=security_profile,
    )
    plans = [(local, runtime, instance, release, None, None)]
    native_state = InstallState(profile="custom", roles=("robot",), install_mode="native",
                               prefix=str(prefix), bin_dir=str(prefix / "bin"),
                               source_root=str(tmp_path),
                               dds=DdsSettings(interface="eth0", security_profile="trusted-network"))
    before = HostActivationState(None, native_state.to_dict(), (),
                                 {"robot-native": None})
    current = before
    active = None

    class Authority:
        def __init__(self, _root):
            pass

        def active(self):
            return SimpleNamespace(generation=active) if active else None

    class Bundle:
        def for_roles(self, roles):
            assert tuple(roles) in {("robot",), ("pilot",)}
            return self

        def manifest_bytes(self):
            return b"manifest"

    class Issued:
        bundles = {host.host_id: Bundle() for host in topology.hosts}

        def activate_authority(self):
            nonlocal active
            active = "g-test"
            events.append(("authority", "activate"))

        def rollback_authority(self):
            nonlocal active
            active = None

    class Issuer:
        def __init__(self, _authority):
            pass

        def issue(self, _topology, _generation):
            return Issued()

    monkeypatch.setattr("elesim_connections.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_connections.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr("elesim_connections.connections.new_generation_id", lambda: "g-test")

    class Operations(_RegistrationOperations):
        def __init__(self):
            super().__init__(events)

        def prepare_runtime_network(self, host, _output):
            events.append(("network", host.host_id))

        def runtime_network_check(self, host):
            pass

        def preflight(self, host):
            assert host.roles == ("robot",)
            return RemoteCapabilities(False, True, True, True, "aarch64")

        def capture_state(self, host):
            assert host.roles == ("robot",)
            return current

        def stage(self, host, bundle):
            events.append(("stage", host.host_id))

        def activate(self, host, generation):
            nonlocal current
            state = replace(native_state, dds=replace(native_state.dds,
                            system_id="scoped", security_profile=security_profile,
                            security_provisioning="managed" if generation else "none"))
            current = HostActivationState(generation, state.to_dict(), (),
                                          {"robot-native": generation})
            events.append(("apply", host.host_id))
            if failure == "native-apply":
                raise RuntimeError("native apply failed after mutation")

        def configure_topology(self, host):
            self.activate(host, None)

        def verify(self, host, generation, running):
            assert not running

        def verify_topology(self, host, running):
            assert not running

        def rollback(self, host, previous):
            nonlocal current
            events.append(("native-rollback", host.host_id))
            if failure == "rollback":
                raise RuntimeError("native restore failed")
            current = previous

        def discard_generation(self, host, generation):
            assert current == before

        def register_scoped_instance(self, host, *args, **kwargs):
            events.append(("register", host.host_id))
            if failure in {"register", "rollback"}:
                raise RuntimeError("register failed")

    runner = ConnectionDeploymentRunner(
        tmp_path / "authority", local_install_root=prefix
    )
    monkeypatch.setattr(runner, "_local_install_scope", lambda: True)
    monkeypatch.setattr(runner, "_scoped_unit_plans", lambda *_args: plans)
    monkeypatch.setattr(
        runner,
        "_operations",
        lambda _topology: {
            "local": Operations(),
            "robot-host": Operations(),
        },
    )

    if failure:
        with pytest.raises(RuntimeError, match="failed"):
            runner(topology, action, lambda _message: None)
        assert current == before if failure != "rollback" else current != before
    else:
        runner(topology, action, lambda _message: None)
        assert current != before
        assert ("apply", "robot-host") in events
        assert ("register", "local") in events
        if security_profile == "sros2":
            assert events.index(("register", "local")) < events.index(("authority", "activate"))
    # Load through the real validator, including snapshots used after restart.
    journal = runner._load_scoped_journal(topology)
    assert journal["schema_version"] == 3
    assert journal["status"] == ("blocked" if failure == "rollback" else "rolled-back" if failure else "completed")


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
        "elesim_connections.connections.Sros2Authority",
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
        "elesim_connections.connections.Sros2Authority",
        lambda root: _RecoveryAuthority(root, "target-generation"),
    )
    monkeypatch.setattr("elesim_connections.connections.SecurityBundle", Bundle)

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


def test_scoped_recovery_forward_reapplies_native_robot_target(
    tmp_path: Path, monkeypatch
) -> None:
    """An Authority commit must also finish a native Robot unit after restart."""

    local_uuid = LOCAL_UUID
    remote_uuid = REMOTE_UUID
    local_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("ui", "ui-local"),),
        install_uuid=local_uuid,
        project=project_name(local_uuid),
    )
    runtime_unit = DeploymentUnit(
        "runtime",
        (RoleAssignment("pilot", "pilot-remote"),),
        install_uuid=remote_uuid,
        project=project_name(remote_uuid),
    )
    robot_unit = DeploymentUnit(
        "robot-native",
        (RoleAssignment("robot", "robot-remote"),),
        install_mode="native",
        lifecycle="systemd",
        install_uuid=remote_uuid,
        install_root="/opt/elesim-robot",
        bin_dir="/opt/elesim-robot/bin",
    )
    topology = ConnectionTopology(
        "scoped",
        "sros2",
        (
            ManagedHost(
                "local",
                True,
                DdsEndpoint("100.64.0.1", "tailscale0"),
                None,
                units=(local_unit,),
            ),
            ManagedHost(
                "jetson",
                False,
                DdsEndpoint("100.64.0.2", "tailscale0"),
                SshEndpoint(
                    "jetson.example", 22, "operator", "", "SHA256:" + "A" * 43
                ),
                units=(runtime_unit, robot_unit),
                jetson=True,
            ),
        ),
        dds_graph=DdsGraphSettings(discovery_mode="static"),
    ).validate()

    old_generation = "old-generation"
    target_generation = "target-generation"
    from elesim_setup.state import DdsSettings

    def native_state(generation: str) -> InstallState:
        return InstallState(
        profile="custom",
        roles=("robot",),
        prefix="/opt/elesim-robot",
        bin_dir="/opt/elesim-robot/bin",
            source_root="/home/operator/src",
            install_mode="native",
            dds=DdsSettings(
                system_id="scoped",
                discovery_mode="static",
                static_peers=topology.discovery_peers("jetson"),
                interface="tailscale0",
                security_profile="sros2",
                security_provisioning="managed",
                security_generation=generation,
                security_bundle="/opt/elesim-robot/security/current/keystore",
                keystore="/opt/elesim-robot/security/current/keystore",
                enclave="/elesim/scoped",
            ),
        ).validate()

    before_native = HostActivationState(
        old_generation,
        native_state(old_generation).to_dict(),
        (),
        {"robot-native": old_generation},
    )
    target_native = HostActivationState(
        target_generation,
        native_state(target_generation).to_dict(),
        (),
        {"robot-native": target_generation},
    )

    releases = {
        "local": _release(local_uuid, ("ui",), marker="a"),
        "jetson": _release(remote_uuid, ("pilot",), marker="e"),
    }
    targets = {
        "local": InstanceState(
            "scoped",
            release_key(releases["local"]),
            (InstanceEndpoint("ui", "ui-local"),),
            0,
            pilot_id="pilot-remote",
            sim_id="sim-main",
            ui_id="ui-local",
            discovery_mode="static",
            static_peers=topology.discovery_peers("local"),
            interface="tailscale0",
            security_profile="sros2",
            security_generation=target_generation,
        ),
        "jetson": InstanceState(
            "scoped",
            release_key(releases["jetson"]),
            (InstanceEndpoint("pilot", "pilot-remote"),),
            0,
            pilot_id="pilot-remote",
            sim_id="sim-main",
            ui_id="ui-local",
            discovery_mode="static",
            static_peers=topology.discovery_peers("jetson"),
            interface="tailscale0",
            security_profile="sros2",
            security_generation=target_generation,
        ),
    }
    before = {key: replace(value, security_generation=old_generation) for key, value in targets.items()}
    plans = [
        (
            topology.host(host_id),
            topology.host(host_id).runtime_units[0],
            targets[host_id],
            releases[host_id],
            before[host_id],
            releases[host_id],
        )
        for host_id in ("local", "jetson")
    ]
    runner = ConnectionDeploymentRunner(tmp_path / "authority")
    journal = _recovery_journal(
        runner,
        topology,
        plans,
        action="rotate",
        authority_before=old_generation,
        authority_target=target_generation,
        authority_observed=target_generation,
        bundle_digest=hashlib.sha256(b"recovery-bundle").hexdigest(),
    )
    journal["schema_version"] = 3
    journal["native_units"] = [
        {
            "host_id": "jetson",
            "unit_id": "robot-native",
            "install_uuid": remote_uuid,
            "before": runner._activation_payload(before_native),
            "target": runner._activation_payload(target_native),
            "status": "target",
        }
    ]

    events: list[tuple] = []

    class MixedOperations(_RecoveryOperations):
        def __init__(self) -> None:
            super().__init__(
                {"local": before["local"], "jetson": before["jetson"]},
                {host_id: (release,) for host_id, release in releases.items()},
                events,
            )
            self.native = before_native

        def scoped_identity(self, _host, unit):
            return {"install_uuid": unit.install_uuid, "project": unit.project}

        def capture_state(self, host):
            return self.native if host.robot_units else self.states[host.host_id]

        def stage(self, host, _bundle):
            events.append(("native-stage", host.host_id))

        def activate(self, host, generation):
            events.append(("native-activate", host.host_id))
            assert generation == target_generation
            self.native = target_native

        def verify(self, host, generation, running):
            events.append(("native-verify", host.host_id))
            assert generation == target_generation and not running

    # A previous recovery may have stopped while applying the native target.
    # Its progress marker must remain loadable for the next recovery attempt.
    journal["host_id"] = "jetson/robot-native"
    journal["native_units"][0]["status"] = "recovering"
    runner._validate_scoped_journal(journal, topology)

    operations = MixedOperations()

    class Bundle:
        @classmethod
        def from_directory(cls, **_kwargs):
            return cls()

        def for_roles(self, _roles):
            return self

        @staticmethod
        def manifest_bytes():
            return b"recovery-bundle"

    monkeypatch.setattr(
        "elesim_connections.connections.Sros2Authority",
        lambda root: _RecoveryAuthority(root, target_generation),
    )
    monkeypatch.setattr("elesim_connections.connections.SecurityBundle", Bundle)

    runner._recover_scoped_transaction(
        topology,
        {"local": operations, "jetson": operations},
        journal,
        lambda _message: None,
    )

    assert ("native-stage", "jetson") in events
    assert ("native-activate", "jetson") in events
    assert ("native-verify", "jetson") in events
    assert events[-2:] == [("register", "local", True), ("register", "jetson", True)]
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
    journal["phase"] = "complete"
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


def test_completed_scoped_journal_from_previous_topology_is_ignored(
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
    journal["phase"] = "complete"
    runner._write_transaction_journal(topology, journal)
    changed = replace(topology, dds_graph=replace(topology.dds_graph, domain_id=7))

    runner._refuse_unresolved_scoped_journal(changed)


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
