from __future__ import annotations

import sys
import json
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import pytest

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    ManagedHost,
    RoleAssignment,
    SshEndpoint,
)
from elesim_setup.secure_deployment import (
    GenerationRollout,
    HostActivationState,
    HostKeyVerificationError,
    InstalledElesimLifecycle,
    LocalHostOperations,
    ParamikoConnector,
    RemoteCapabilities,
    RemoteCommandResult,
    RolloutError,
    SecurityBundle,
    SecurityFile,
    SshHostOperations,
    TopologyRollout,
    ssh_sha256_fingerprint,
)


FINGERPRINT = "SHA256:" + "A" * 43


def _ssh(host: str = "server.example", identity: str = "~/.ssh/key") -> SshEndpoint:
    return SshEndpoint(host, 2222, "operator", identity, FINGERPRINT)


def _topology() -> ConnectionTopology:
    return ConnectionTopology(
        "lab",
        "sros2",
        (
            ManagedHost(
                "laptop",
                "Laptop",
                True,
                DdsEndpoint("100.64.0.1", "tailscale0"),
                None,
                (
                    RoleAssignment("controller", "controller-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
            ManagedHost(
                "server",
                "Server",
                False,
                DdsEndpoint("100.64.0.2", "tailscale0"),
                _ssh(),
                (RoleAssignment("simulator", "sim-main"),),
            ),
            ManagedHost(
                "robot",
                "Robot",
                False,
                DdsEndpoint("100.64.0.3", "tailscale0"),
                _ssh("robot.example"),
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                jetson=True,
                install_root="/opt/elesim-robot",
                bin_dir="/usr/local/bin",
                lifecycle="systemd",
            ),
        ),
    ).validate()


def _bundle(host_id: str, generation: str = "g2") -> SecurityBundle:
    roles = {
        "laptop": ("controller", "ui"),
        "server": ("simulator",),
        "robot": ("robot",),
    }[host_id]
    files = [
        SecurityFile("public/identity_ca.cert.pem", b"public", 0o644),
        SecurityFile(f"enclaves/{host_id}/key.pem", b"private", 0o600),
    ]
    for role in roles:
        files.extend(
            (
                SecurityFile(
                    f"roles/{role}/keystore/public/identity_ca.cert.pem",
                    b"public",
                    0o644,
                ),
                SecurityFile(
                    f"roles/{role}/keystore/enclaves/{role}/key.pem",
                    f"{generation}-{role}".encode(),
                    0o600,
                ),
            )
        )
    return SecurityBundle(
        system_id="lab",
        host_id=host_id,
        generation=generation,
        files=tuple(files),
    ).validate()


def test_bundle_manifest_is_bounded_hashed_and_contains_no_payload() -> None:
    bundle = _bundle("server")

    manifest = bundle.manifest()

    assert manifest["host_id"] == "server"
    assert manifest["files"][0]["path"] == "enclaves/server/key.pem"
    assert manifest["files"][0]["mode"] == "0600"
    assert manifest["files"][0]["sha256"]
    assert b"private" not in bundle.manifest_bytes()


def test_authority_export_directory_adapter_skips_the_old_manifest(
    tmp_path: Path,
) -> None:
    exported = tmp_path / "bundle"
    key = exported / "keystore/enclaves/elesim/lab/controller/main/key.pem"
    key.parent.mkdir(parents=True)
    key.write_bytes(b"role-key")
    key.chmod(0o600)
    (exported / "manifest.json").write_text("{}", encoding="utf-8")

    bundle = SecurityBundle.from_directory(
        system_id="lab", host_id="laptop", generation="g2", root=exported
    )

    assert [file.relative_path for file in bundle.files] == [
        "keystore/enclaves/elesim/lab/controller/main/key.pem"
    ]


@pytest.mark.parametrize(
    "file",
    [
        SecurityFile("../escape", b"x"),
        SecurityFile("/absolute", b"x"),
        SecurityFile("authority/private/root.pem", b"x"),
        SecurityFile("public/identity_ca.key.pem", b"x"),
        SecurityFile("manifest.json", b"x"),
        SecurityFile("large", b"x" * (1024 * 1024 + 1)),
    ],
)
def test_bundle_rejects_escape_authority_private_and_oversized_files(file) -> None:
    with pytest.raises(ValueError):
        SecurityBundle("lab", "server", "g2", (file,)).validate()


def test_paramiko_connector_uses_pinned_key_and_explicit_identity_only(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clients = []

    class Client:
        def __init__(self):
            self.policy = None
            self.arguments = None
            self.closed = False
            clients.append(self)

        def set_missing_host_key_policy(self, policy) -> None:
            self.policy = policy

        def connect(self, **arguments) -> None:
            self.arguments = arguments

        def close(self) -> None:
            self.closed = True

    monkeypatch.setitem(sys.modules, "paramiko", SimpleNamespace(SSHClient=Client))
    identity = tmp_path / "id_ed25519"
    endpoint = SshEndpoint(
        "server.example", 2222, "operator", str(identity), FINGERPRINT
    )

    session = ParamikoConnector(timeout_s=4).connect(endpoint)

    assert clients[0].arguments["password"] is None
    assert clients[0].arguments["allow_agent"] is False
    assert clients[0].arguments["look_for_keys"] is False
    assert clients[0].arguments["key_filename"] == str(identity)
    assert clients[0].arguments["port"] == 2222
    with pytest.raises(HostKeyVerificationError, match="mismatch"):
        clients[0].policy.missing_host_key(
            None, "server.example", SimpleNamespace(asbytes=lambda: b"wrong-key")
        )
    session.__exit__(None, None, None)
    assert clients[0].closed


def test_paramiko_resolves_tilde_against_operator_home(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    clients = []

    class Client:
        def set_missing_host_key_policy(self, _policy) -> None:
            pass

        def connect(self, **arguments) -> None:
            self.arguments = arguments
            clients.append(self)

        def close(self) -> None:
            pass

    operator_home = tmp_path / "host-home"
    container_home = tmp_path / "generated-container-home"
    monkeypatch.setenv("HOME", str(container_home))
    monkeypatch.setenv("ELESIM_OPERATOR_HOME", str(operator_home))
    monkeypatch.setitem(
        sys.modules,
        "paramiko",
        SimpleNamespace(SSHClient=Client),
    )

    session = ParamikoConnector().connect(_ssh(identity="~/.ssh/id_ed25519"))

    assert clients[0].arguments["key_filename"] == str(
        operator_home / ".ssh/id_ed25519"
    )
    session.__exit__(None, None, None)


def test_paramiko_connector_agent_mode_does_not_search_key_files(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    actual = ssh_sha256_fingerprint(b"server-key")

    class Client:
        def set_missing_host_key_policy(self, policy) -> None:
            self.policy = policy

        def connect(self, **arguments) -> None:
            self.arguments = arguments

        def close(self) -> None:
            pass

    client = Client()
    monkeypatch.setitem(
        sys.modules, "paramiko", SimpleNamespace(SSHClient=lambda: client)
    )
    endpoint = SshEndpoint("server.example", 22, "operator", "", actual)

    session = ParamikoConnector().connect(endpoint)

    assert client.arguments["allow_agent"] is True
    assert client.arguments["look_for_keys"] is False
    assert "key_filename" not in client.arguments
    client.policy.missing_host_key(
        None, "server.example", SimpleNamespace(asbytes=lambda: b"server-key")
    )
    session.__exit__(None, None, None)


class FakeSession:
    def __init__(self) -> None:
        self.commands: list[tuple[tuple[str, ...], bool]] = []
        self.uploads: list[tuple[PurePosixPath, bytes, int]] = []
        self.readlink = ""

    def __enter__(self):
        return self

    def __exit__(self, *_args) -> None:
        pass

    def run(self, argv, *, check=True) -> RemoteCommandResult:
        values = tuple(argv)
        self.commands.append((values, check))
        if values[0] == "readlink":
            return RemoteCommandResult(0, self.readlink)
        if values[:2] in {("test", "-e"), ("test", "-L")}:
            return RemoteCommandResult(1)
        return RemoteCommandResult(0)

    def upload_bytes(self, path, content, mode) -> None:
        self.uploads.append((path, content, mode))


class FakeConnector:
    def __init__(self, session: FakeSession) -> None:
        self.session = session
        self.endpoints = []

    def connect(self, endpoint):
        self.endpoints.append(endpoint)
        return self.session


class FakeLifecycle:
    def preflight(self, _session, _host, _root):
        return RemoteCapabilities(True, True, False, True)

    def snapshot(self, _session, host):
        return {"roles": list(host.roles)}

    def configure(self, _session, _host, _generation, _root) -> None:
        pass

    def restore(self, _session, _host, _configuration) -> None:
        pass

    def stop(self, _session, _host) -> None:
        pass

    def start(self, _session, _host) -> None:
        pass

    def verify(self, _session, _host, _generation) -> None:
        pass


def test_ssh_host_operations_stage_manifest_then_atomically_activate() -> None:
    host = _topology().host("server")
    topology = _topology()
    session = FakeSession()
    operations = SshHostOperations(
        FakeConnector(session), FakeLifecycle(), topology
    )

    operations.stage(host, _bundle("server"))
    operations.activate(host, "g2")

    uploaded_names = [path.name for path, _content, _mode in session.uploads]
    assert uploaded_names[-1] == "manifest.json"
    assert {"identity_ca.cert.pem", "key.pem"}.issubset(uploaded_names)
    move_stage = [argv for argv, _check in session.commands if argv[0] == "mv"][0]
    assert move_stage[-1] == "/opt/elesim/security/generations/g2"
    activation = [
        argv for argv, _check in session.commands if argv[:2] == ("mv", "-Tf")
    ]
    assert activation[0][-1] == "/opt/elesim/security/current"


def test_local_host_operations_use_install_root_security_directory(
    tmp_path: Path,
) -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["install_root"] = str(tmp_path / "install")
    raw["hosts"][0]["bin_dir"] = str(tmp_path / "bin")
    topology = ConnectionTopology.from_dict(raw)
    host = topology.host("laptop")
    Path(host.install_root).mkdir()
    operations = LocalHostOperations(FakeLifecycle(), topology)

    operations.stage(host, _bundle("laptop"))
    operations.activate(host, "g2")

    current = Path(host.install_root) / "security/current"
    assert current.is_symlink()
    assert current.resolve().name == "g2"
    assert (current / "manifest.json").is_file()
    controller_root = Path(host.install_root) / "security/roles/controller"
    ui_root = Path(host.install_root) / "security/roles/ui"
    assert (controller_root / "enclaves/controller/key.pem").read_bytes() == (
        b"g2-controller"
    )
    assert (ui_root / "enclaves/ui/key.pem").read_bytes() == b"g2-ui"


def test_role_view_switch_and_clear_keep_stable_root_inode(tmp_path: Path) -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["install_root"] = str(tmp_path / "install")
    raw["hosts"][0]["bin_dir"] = str(tmp_path / "bin")
    topology = ConnectionTopology.from_dict(raw)
    host = topology.host("laptop")
    role_root = Path(host.install_root) / "security/roles/controller"
    role_root.mkdir(parents=True)
    inode = role_root.stat().st_ino
    operations = LocalHostOperations(FakeLifecycle(), topology)

    operations.stage(host, _bundle("laptop", "g1"))
    operations.activate(host, "g1")
    previous = HostActivationState("g1", {"dds": {}, "network": {}})
    operations.stage(host, _bundle("laptop", "g2"))
    operations.activate(host, "g2")
    assert role_root.stat().st_ino == inode
    assert (role_root / "enclaves/controller/key.pem").read_bytes() == (
        b"g2-controller"
    )

    operations.rollback(host, previous)
    assert role_root.stat().st_ino == inode
    assert (role_root / "enclaves/controller/key.pem").read_bytes() == (
        b"g1-controller"
    )
    operations.rollback(
        host,
        HostActivationState(None, {"dds": {}, "network": {}}),
    )
    assert role_root.stat().st_ino == inode
    assert not (role_root / "public").exists()
    assert not (role_root / "enclaves").exists()


def test_role_view_activation_refuses_symlink_root(tmp_path: Path) -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["install_root"] = str(tmp_path / "install")
    raw["hosts"][0]["bin_dir"] = str(tmp_path / "bin")
    topology = ConnectionTopology.from_dict(raw)
    host = topology.host("laptop")
    security = Path(host.install_root) / "security"
    target = tmp_path / "wrong-role-root"
    target.mkdir(parents=True)
    (security / "roles").mkdir(parents=True)
    (security / "roles/controller").symlink_to(target, target_is_directory=True)
    operations = LocalHostOperations(FakeLifecycle(), topology)
    operations.stage(host, _bundle("laptop"))

    with pytest.raises(RuntimeError, match="symlink"):
        operations.activate(host, "g2")


class LifecycleSession(FakeSession):
    def run(self, argv, *, check=True) -> RemoteCommandResult:
        values = tuple(argv)
        self.commands.append((values, check))
        if values[-1:] == ("show",):
            return RemoteCommandResult(
                0,
                json.dumps(
                    {
                        "roles": ["simulator"],
                        "prefix": "/opt/elesim",
                        "bin_dir": "/usr/local/bin",
                        "install_mode": "container",
                        "dds": {},
                        "network": {},
                    }
                ),
            )
        if values[:2] == ("uname", "-m"):
            return RemoteCommandResult(0, "x86_64\n")
        return RemoteCommandResult(0, "26.1\n" if values[0] == "docker" else "")


def test_concrete_lifecycle_preflight_and_managed_configuration_command() -> None:
    topology = _topology()
    host = topology.host("server")
    session = LifecycleSession()
    lifecycle = InstalledElesimLifecycle(topology)

    capabilities = lifecycle.preflight(
        session, host, PurePosixPath("/opt/elesim/security")
    )
    lifecycle.configure(
        session, host, "g2", PurePosixPath("/opt/elesim/security")
    )
    lifecycle.stop(session, host)
    lifecycle.start(session, host)

    assert capabilities.docker
    assert capabilities.security_root_writable
    configure = next(
        argv for argv, _check in session.commands if "configure" in argv
    )
    assert "--dds-security-provisioning" in configure
    assert configure[configure.index("--dds-security-generation") + 1] == "g2"
    assert configure[configure.index("--dds-security-bundle") + 1] == (
        "/opt/elesim/security/current/keystore"
    )
    assert configure[configure.index("--dds-enclave") + 1] == "/elesim/lab"
    assert configure[configure.index("--simulator-id") + 1] == "sim-main"
    assert configure[configure.index("--controller-id") + 1] == "controller-main"
    assert configure[configure.index("--ui-id") + 1] == "ui-main"
    assert configure[configure.index("--robot-id") + 1] == "robot-main"
    compose_commands = [
        argv for argv, _check in session.commands if argv[:2] == ("docker", "compose")
    ]
    assert (
        "docker",
        "compose",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "down",
        "--remove-orphans",
    ) in compose_commands
    assert (
        "docker",
        "compose",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "up",
        "-d",
        "--remove-orphans",
    ) in compose_commands


def test_simulation_only_configuration_does_not_emit_robot_endpoint() -> None:
    topology = ConnectionTopology(
        "lab_sim",
        "trusted-network",
        (
            ManagedHost(
                "laptop",
                "Simulation laptop",
                True,
                DdsEndpoint("100.64.0.40", "tailscale0"),
                None,
                (
                    RoleAssignment("controller", "controller-main"),
                    RoleAssignment("simulator", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
        ),
        topology_mode="simulation-only",
    ).validate()
    host = topology.host("laptop")
    session = LifecycleSession()
    lifecycle = InstalledElesimLifecycle(topology)

    lifecycle.configure(session, host, None, PurePosixPath("/opt/elesim/security"))

    configure = next(
        argv for argv, _check in session.commands if "configure" in argv
    )
    assert "--robot-id" not in configure
    assert configure[configure.index("--simulator-id") + 1] == "sim-main"


@dataclass
class FakeOperations:
    host_id: str
    events: list[str]
    fail_once: str = ""

    def _event(self, name: str) -> None:
        rendered = f"{name}:{self.host_id}"
        self.events.append(rendered)
        if self.fail_once == name:
            self.fail_once = ""
            raise RuntimeError(rendered)

    def preflight(self, host):
        self._event("preflight")
        return RemoteCapabilities(
            docker=host.install_mode == "container",
            systemd=host.install_mode == "native",
            jetson=host.jetson,
            security_root_writable=True,
        )

    def capture_state(self, _host):
        self._event("current")
        return HostActivationState("g1", {"dds": {}})

    def stage(self, _host, _bundle):
        self._event("stage")

    def stop(self, _host):
        self._event("stop")

    def activate(self, _host, _generation):
        self._event("activate")

    def configure_topology(self, _host):
        self._event("configure")

    def start(self, _host):
        self._event("start")

    def verify(self, _host, _generation):
        self._event("verify")

    def verify_topology(self, _host):
        self._event("verify-topology")

    def rollback(self, _host, _previous):
        self._event("rollback")


def _rollout_parts(*, fail_host: str = "", fail_once: str = ""):
    topology = _topology()
    events: list[str] = []
    operations = {
        host.host_id: FakeOperations(
            host.host_id,
            events,
            fail_once=fail_once if host.host_id == fail_host else "",
        )
        for host in topology.hosts
    }
    bundles = {host.host_id: _bundle(host.host_id) for host in topology.hosts}
    return topology, events, operations, bundles


def test_simulation_only_trusted_rollout_accepts_one_compose_host() -> None:
    topology = ConnectionTopology(
        "lab_sim",
        "trusted-network",
        (
            ManagedHost(
                "laptop",
                "Simulation laptop",
                True,
                DdsEndpoint("100.64.0.40", "tailscale0"),
                None,
                (
                    RoleAssignment("controller", "controller-main"),
                    RoleAssignment("simulator", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
        ),
        topology_mode="simulation-only",
    ).validate()
    events: list[str] = []
    operations = {"laptop": FakeOperations("laptop", events)}

    result = TopologyRollout(topology, operations).apply()

    assert result.previous_generations == {"laptop": "g1"}
    assert events[-1] == "verify-topology:laptop"


def test_rollout_stages_every_host_before_stopping_any_runtime() -> None:
    topology, events, operations, bundles = _rollout_parts()

    result = GenerationRollout(topology, operations).apply("g2", bundles)

    last_stage = max(index for index, event in enumerate(events) if event.startswith("stage:"))
    first_stop = min(index for index, event in enumerate(events) if event.startswith("stop:"))
    assert last_stage < first_stop
    assert result.previous_generations == {
        "laptop": "g1",
        "server": "g1",
        "robot": "g1",
    }
    assert events[-3:] == ["verify:laptop", "verify:server", "verify:robot"]


def test_rollout_failure_restores_every_switched_host_before_restart() -> None:
    topology, events, operations, bundles = _rollout_parts(
        fail_host="server", fail_once="verify"
    )

    with pytest.raises(RolloutError, match="during verify") as captured:
        GenerationRollout(topology, operations).apply("g2", bundles)

    assert captured.value.rollback_errors == ()
    rollback_positions = [
        index for index, event in enumerate(events) if event.startswith("rollback:")
    ]
    recovery_start_positions = [
        index
        for index, event in enumerate(events)
        if event.startswith("start:") and index > rollback_positions[-1]
    ]
    assert [events[index] for index in rollback_positions] == [
        "rollback:robot",
        "rollback:server",
        "rollback:laptop",
    ]
    assert recovery_start_positions
    assert min(recovery_start_positions) > max(rollback_positions)


def test_stage_failure_never_stops_a_runtime() -> None:
    topology, events, operations, bundles = _rollout_parts(
        fail_host="server", fail_once="stage"
    )

    with pytest.raises(RolloutError, match="during stage"):
        GenerationRollout(topology, operations).apply("g2", bundles)

    assert not any(event.startswith("stop:") for event in events)
    assert not any(event.startswith("rollback:") for event in events)


def test_authority_commit_failure_rolls_back_live_hosts() -> None:
    topology, events, operations, bundles = _rollout_parts()

    def fail_commit() -> None:
        raise RuntimeError("authority metadata write failed")

    with pytest.raises(RolloutError, match="commit-authority"):
        GenerationRollout(topology, operations).apply(
            "g2", bundles, commit=fail_commit
        )

    assert [event for event in events if event.startswith("rollback:")] == [
        "rollback:robot",
        "rollback:server",
        "rollback:laptop",
    ]


def test_progress_callback_cancellation_triggers_runtime_rollback() -> None:
    topology, events, operations, bundles = _rollout_parts()
    progress_events: list[tuple[str, str | None]] = []

    def cancel(phase: str, host_id: str | None) -> None:
        progress_events.append((phase, host_id))
        if (phase, host_id) == ("switch", "server"):
            raise RuntimeError("cancelled")

    with pytest.raises(RolloutError) as captured:
        GenerationRollout(topology, operations).apply(
            "g2", bundles, progress=cancel
        )

    assert str(captured.value.cause) == "cancelled"
    assert ("switch", "server") in progress_events
    assert "rollback:laptop" in events


def test_trusted_network_topology_rollout_changes_no_security_bundle() -> None:
    topology, events, operations, _bundles = _rollout_parts()
    raw = topology.to_dict()
    raw["security_profile"] = "trusted-network"
    trusted = ConnectionTopology.from_dict(raw)

    result = TopologyRollout(trusted, operations).apply()

    assert not any(event.startswith("stage:") for event in events)
    assert not any(event.startswith("activate:") for event in events)
    last_configure = max(
        index for index, event in enumerate(events) if event.startswith("configure:")
    )
    first_start = min(
        index for index, event in enumerate(events) if event.startswith("start:")
    )
    assert last_configure < first_start
    assert result.previous_generations["server"] == "g1"
