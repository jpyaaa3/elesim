from __future__ import annotations

import sys
import json
import io
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import pytest

import elesim_setup.secure_deployment as secure_deployment
from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    DdsGraphSettings,
    DeploymentUnit,
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
    RemoteCommandError,
    RemoteCommandResult,
    RolloutError,
    RuntimeLaunchOptions,
    SecurityBundle,
    SecurityFile,
    SshHostOperations,
    TopologyRollout,
    _command_timeout,
    _lifecycle_command,
    _managed_turn_from_state,
    _LocalSession,
    _ParamikoSession,
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
                True,
                DdsEndpoint("100.64.0.1", "tailscale0"),
                None,
                (
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
            ),
            ManagedHost(
                "server",
                False,
                DdsEndpoint("100.64.0.2", "tailscale0"),
                _ssh(),
                (RoleAssignment("sim", "sim-main"),),
            ),
            ManagedHost(
                "robot",
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
        dds_graph=DdsGraphSettings(discovery_mode="static"),
    ).validate()


def _bundle(host_id: str, generation: str = "g2") -> SecurityBundle:
    roles = {
        "laptop": ("pilot", "ui"),
        "server": ("sim",),
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


def test_compose_build_and_launch_are_separate_from_security_resume() -> None:
    host = _topology().host("server")

    assert _lifecycle_command(host, action="start") == (
        "/usr/local/bin/elesim-compose",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "start",
        "sim",
    )
    assert _lifecycle_command(host, action="build") == (
        "/usr/local/bin/elesim-compose",
        "--progress",
        "plain",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "build",
        "sim",
    )
    assert _lifecycle_command(host, action="launch") == (
        "/usr/local/bin/elesim-up",
        "--no-build",
        "sim",
    )
    assert _lifecycle_command(
        host, action="start", include_coturn=True
    )[-3:] == ("start", "sim", "coturn")


def test_runtime_launch_options_are_bounded_and_use_normal_runtime_launcher() -> None:
    host = _topology().host("server")
    options = RuntimeLaunchOptions.from_payload(
        {"gpu_inherit": True, "gpu_device": "3", "viewer": True}
    )
    assert options is not None
    assert options.launcher_flags() == (
        "--cuda-visible-devices",
        "3",
        "--view",
    )
    assert _lifecycle_command(host, action="launch", runtime_options=options) == (
        "/usr/local/bin/elesim-up",
        "--no-build",
        "--cuda-visible-devices",
        "3",
        "--view",
        "sim",
    )
    with pytest.raises(ValueError, match="gpu_device"):
        RuntimeLaunchOptions.from_payload(
            {"gpu_inherit": True, "gpu_device": "gpu0", "viewer": False}
        )
    with pytest.raises(ValueError, match="unsupported fields"):
        RuntimeLaunchOptions.from_payload(
            {"gpu_inherit": False, "gpu_device": "", "viewer": False, "env": {}}
        )


def test_viewer_launch_flag_is_scoped_to_the_sim_unit() -> None:
    topology = _topology()
    options = RuntimeLaunchOptions(True, "2", True)
    session = FakeSession()

    InstalledElesimLifecycle(topology).launch(
        session,
        topology.host("laptop"),
        options,
    )

    command = session.commands[-1][0]
    assert command[:4] == (
        "/usr/local/bin/elesim-up",
        "--no-build",
        "--cuda-visible-devices",
        "2",
    )
    assert "--view" not in command


def test_viewer_cleanup_is_scoped_to_the_container_sim_unit() -> None:
    topology = _topology()
    lifecycle = InstalledElesimLifecycle(topology)
    session = FakeSession()

    lifecycle.cleanup_viewer(session, topology.host("server"), ("sim",))

    assert session.commands[-1][0] == (
        "/usr/local/bin/elesim-viewer-cleanup",
    )
    command_count = len(session.commands)
    lifecycle.cleanup_viewer(
        session,
        topology.host("laptop"),
        ("pilot", "ui"),
    )
    assert len(session.commands) == command_count


def test_mixed_host_lifecycle_commands_remain_unit_scoped() -> None:
    host = ManagedHost(
        "jetson",
        False,
        DdsEndpoint("100.64.0.31", "tailscale0"),
        _ssh("jetson.example"),
        jetson=True,
        units=(
            DeploymentUnit(
                "runtime",
                (RoleAssignment("pilot", "pilot-main"), RoleAssignment("ui", "ui-main")),
                install_root="/opt/elesim-runtime",
            ),
            DeploymentUnit(
                "robot-native",
                (RoleAssignment("robot", "robot-main"),),
                install_mode="native",
                install_root="/opt/elesim-robot",
                lifecycle="systemd",
            ),
        ),
    ).validate()

    runtime = host.runtime_units[0]
    robot = host.robot_units[0]
    assert _lifecycle_command(runtime, action="start")[-3:] == (
        "start",
        "pilot",
        "ui",
    )
    assert _lifecycle_command(runtime, action="build")[-3:] == (
        "build",
        "pilot",
        "ui",
    )
    assert _lifecycle_command(robot, action="start") == (
        "sudo",
        "-n",
        "systemctl",
        "start",
        "elesim-robot.service",
    )


def test_bundle_manifest_is_bounded_hashed_and_contains_no_payload() -> None:
    bundle = _bundle("server")

    manifest = bundle.manifest()

    assert manifest["host_id"] == "server"
    assert manifest["files"][0]["path"] == "enclaves/server/key.pem"
    assert manifest["files"][0]["mode"] == "0600"
    assert manifest["files"][0]["sha256"]
    assert b"private" not in bundle.manifest_bytes()


def test_role_scoped_bundle_keeps_only_selected_role_material() -> None:
    bundle = SecurityBundle(
        "lab",
        "jetson",
        "g2",
        (
            SecurityFile("keystore/public/identity_ca.cert.pem", b"public", 0o644),
            SecurityFile("public/legacy_identity_ca.cert.pem", b"legacy", 0o644),
            SecurityFile("keystore/enclaves/elesim/lab/pilot/main/key.pem", b"pilot"),
            SecurityFile("keystore/enclaves/elesim/lab/robot/main/key.pem", b"robot"),
            SecurityFile("roles/pilot/keystore/enclaves/elesim/lab/pilot/main/key.pem", b"pilot"),
            SecurityFile("roles/robot/keystore/enclaves/elesim/lab/robot/main/key.pem", b"robot"),
        ),
    ).validate()

    robot_view = bundle.for_roles(("robot",))

    assert [item.relative_path for item in robot_view.files] == [
        "keystore/public/identity_ca.cert.pem",
        "public/legacy_identity_ca.cert.pem",
        "keystore/enclaves/elesim/lab/robot/main/key.pem",
        "roles/robot/keystore/enclaves/elesim/lab/robot/main/key.pem",
    ]

    pilot_view = bundle.for_roles(("pilot",))
    assert [item.relative_path for item in pilot_view.files] == [
        "keystore/public/identity_ca.cert.pem",
        "public/legacy_identity_ca.cert.pem",
        "keystore/enclaves/elesim/lab/pilot/main/key.pem",
        "roles/pilot/keystore/enclaves/elesim/lab/pilot/main/key.pem",
    ]
    assert all("ui" not in item.relative_path for item in pilot_view.files)


def test_authority_export_directory_adapter_skips_the_old_manifest(
    tmp_path: Path,
) -> None:
    exported = tmp_path / "bundle"
    key = exported / "keystore/enclaves/elesim/lab/pilot/main/key.pem"
    key.parent.mkdir(parents=True)
    key.write_bytes(b"role-key")
    key.chmod(0o600)
    (exported / "manifest.json").write_text("{}", encoding="utf-8")

    bundle = SecurityBundle.from_directory(
        system_id="lab", host_id="laptop", generation="g2", root=exported
    )

    assert [file.relative_path for file in bundle.files] == [
        "keystore/enclaves/elesim/lab/pilot/main/key.pem"
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


def test_paramiko_session_drains_and_bounds_verbose_remote_output() -> None:
    class Channel:
        @staticmethod
        def recv_exit_status() -> int:
            return 0

    class Stream(io.BytesIO):
        channel = Channel()

    class Client:
        closed = False

        @staticmethod
        def exec_command(_command, timeout):
            assert timeout == 1800
            return None, Stream(b"out\n"), Stream(b"x" * (96 * 1024))

        def close(self) -> None:
            self.closed = True

    result = _ParamikoSession(Client(), command_timeout_s=2).run(
        ("docker", "compose", "build", "sim")
    )

    assert result.exit_status == 0
    assert result.stdout == "out\n"
    assert result.stderr.startswith("[earlier remote output truncated]\n")
    assert result.stderr.endswith("x" * (64 * 1024))


def test_paramiko_session_allows_slow_detached_compose_lifecycle() -> None:
    class Channel:
        @staticmethod
        def recv_exit_status() -> int:
            return 0

    class Stream(io.BytesIO):
        channel = Channel()

    class Client:
        @staticmethod
        def exec_command(_command, timeout):
            assert timeout == 300
            return None, Stream(), Stream()

    result = _ParamikoSession(Client(), command_timeout_s=2).run(
        (
            "docker",
            "compose",
            "-p",
            "elesim-runtime",
            "up",
            "-d",
            "--no-build",
        )
    )

    assert result.exit_status == 0


def test_managed_command_timeout_keeps_build_and_lifecycle_limits_separate() -> None:
    assert _command_timeout(("docker", "compose", "build", "sim"), 2) == 1800
    assert _command_timeout(("docker", "compose", "up", "-d"), 2) == 300
    assert _command_timeout(("/usr/local/bin/elesim-up", "--no-build", "sim"), 2) == 300
    assert _command_timeout(("/usr/local/bin/elesim-tailscale", "login"), 2) == 600
    assert (
        _command_timeout(
            ("/usr/local/bin/elesim-tailscale", "status", "--json"), 2
        )
        == 2
    )
    assert _command_timeout(("elesim-net", "show"), 2) == 2


def test_local_runtime_launcher_is_forwarded_to_the_host_helper(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    forwarded: list[tuple[tuple[str, ...], str, float]] = []

    def fake_helper(
        argv,
        *,
        socket_path: str,
        timeout_s: float,
        output=None,
    ) -> RemoteCommandResult:
        assert output is None
        forwarded.append((tuple(argv), socket_path, timeout_s))
        return RemoteCommandResult(0, "started\n", "")

    monkeypatch.setenv("ELESIM_HOST_HELPER_SOCKET", "/run/elesim-helper.sock")
    monkeypatch.setattr(secure_deployment, "_run_through_host_helper", fake_helper)

    result = _LocalSession(timeout_s=2).run(
        ("/opt/elesim/bin/elesim-up", "--no-build", "sim")
    )

    assert result.stdout == "started\n"
    assert forwarded == [
        (
            ("/opt/elesim/bin/elesim-up", "--no-build", "sim"),
            "/run/elesim-helper.sock",
            300,
        )
    ]


def test_local_viewer_cleanup_is_forwarded_to_the_host_helper(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    forwarded: list[tuple[tuple[str, ...], str, float]] = []

    def fake_helper(
        argv,
        *,
        socket_path: str,
        timeout_s: float,
        output=None,
    ) -> RemoteCommandResult:
        assert output is None
        forwarded.append((tuple(argv), socket_path, timeout_s))
        return RemoteCommandResult(0, "", "")

    monkeypatch.setenv("ELESIM_HOST_HELPER_SOCKET", "/run/elesim-helper.sock")
    monkeypatch.setattr(secure_deployment, "_run_through_host_helper", fake_helper)

    _LocalSession(timeout_s=2).run(
        ("/opt/elesim/bin/elesim-viewer-cleanup",)
    )

    assert forwarded == [
        (
            ("/opt/elesim/bin/elesim-viewer-cleanup",),
            "/run/elesim-helper.sock",
            2,
        )
    ]


def test_paramiko_session_streams_live_channel_output() -> None:
    class Channel:
        def __init__(self) -> None:
            self.stdout = [b"#1 load\n", b"#2 build\n"]
            self.stderr = [b"warning\n"]

        def recv_ready(self) -> bool:
            return bool(self.stdout)

        def recv(self, _size: int) -> bytes:
            return self.stdout.pop(0)

        def recv_stderr_ready(self) -> bool:
            return bool(self.stderr)

        def recv_stderr(self, _size: int) -> bytes:
            return self.stderr.pop(0)

        def exit_status_ready(self) -> bool:
            return not self.stdout and not self.stderr

        @staticmethod
        def recv_exit_status() -> int:
            return 0

    channel = Channel()

    class Stream(io.BytesIO):
        pass

    stdout = Stream()
    stdout.channel = channel
    stderr = Stream()
    stderr.channel = channel

    class Client:
        @staticmethod
        def exec_command(_command, timeout):
            assert timeout == 1800
            return None, stdout, stderr

        @staticmethod
        def close() -> None:
            pass

    output: list[tuple[str, str]] = []
    result = _ParamikoSession(Client(), command_timeout_s=2).run_streaming(
        ("docker", "compose", "build", "sim"),
        output=lambda stream, text: output.append((stream, text)),
    )

    assert output == [
        ("stdout", "#1 load\n"),
        ("stdout", "#2 build\n"),
        ("stderr", "warning\n"),
    ]
    assert result.stdout == "#1 load\n#2 build\n"
    assert result.stderr == "warning\n"


def test_paramiko_connector_uses_tailscale_ssh_auth_none_without_a_key(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    key_bytes = b"tailscale-server-key"
    fingerprint = ssh_sha256_fingerprint(key_bytes)

    class RawSocket:
        def __init__(self) -> None:
            self.closed = False

        def close(self) -> None:
            self.closed = True

    raw_socket = RawSocket()
    monkeypatch.setenv("ELESIM_TAILSCALE_PROXY", "1")
    monkeypatch.setenv(
        "ELESIM_TAILSCALE_PROXY_BIN", "/usr/local/bin/elesim-tailscale"
    )
    monkeypatch.setenv(
        "ELESIM_TAILSCALE_PROXY_SOCKET", "/var/run/tailscale/tailscaled.sock"
    )
    monkeypatch.setitem(
        sys.modules,
        "paramiko.proxy",
        SimpleNamespace(ProxyCommand=lambda _command: raw_socket),
    )

    class Key:
        def asbytes(self) -> bytes:
            return key_bytes

    class Transport:
        instances = []

        def __init__(self, connection) -> None:
            self.connection = connection
            self.authenticated = False
            self.username = None
            self.closed = False
            self.__class__.instances.append(self)

        def start_client(self, timeout) -> None:
            self.timeout = timeout

        def get_remote_server_key(self):
            return Key()

        def auth_none(self, username):
            self.username = username
            self.authenticated = True

        def is_authenticated(self) -> bool:
            return self.authenticated

        def set_keepalive(self, seconds: int) -> None:
            self.keepalive = seconds

        def close(self) -> None:
            self.closed = True
            self.connection.close()

    class Client:
        def __init__(self) -> None:
            self._transport = None
            self.closed = False

        def close(self) -> None:
            self.closed = True
            self._transport.close()

    monkeypatch.setitem(
        sys.modules,
        "paramiko",
        SimpleNamespace(Transport=Transport, SSHClient=Client),
    )
    endpoint = SshEndpoint(
        "100.64.0.20",
        22,
        "operator",
        "",
        fingerprint,
        auth_mode="tailscale",
    )

    session = ParamikoConnector(timeout_s=4).connect(endpoint)
    transport = Transport.instances[0]

    assert transport.username == "operator"
    assert transport.authenticated is True
    assert transport.auth_timeout == 4
    assert transport.keepalive == 15
    assert raw_socket.closed is False
    session.__exit__(None, None, None)
    assert transport.closed is True
    assert raw_socket.closed is True


def test_paramiko_connector_reports_tailscale_check_reauth_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import elesim_setup.secure_deployment as secure_deployment

    key_bytes = b"tailscale-server-key"
    raw_socket = SimpleNamespace(close=lambda: None)
    monkeypatch.setattr(
        secure_deployment.socket,
        "create_connection",
        lambda address, timeout: raw_socket,
    )

    class Key:
        def asbytes(self) -> bytes:
            return key_bytes

    class AuthenticationException(Exception):
        pass

    class Transport:
        def __init__(self, _connection) -> None:
            self.closed = False

        def start_client(self, timeout) -> None:
            pass

        def get_remote_server_key(self):
            return Key()

        def auth_none(self, username):
            raise AuthenticationException("rejected")

        def auth_password(self, username, password):
            raise AuthenticationException("rejected")

        def is_authenticated(self) -> bool:
            return False

        def close(self) -> None:
            self.closed = True

    monkeypatch.setitem(
        sys.modules,
        "paramiko",
        SimpleNamespace(
            AuthenticationException=AuthenticationException,
            Transport=Transport,
            SSHClient=lambda: object(),
        ),
    )
    endpoint = SshEndpoint(
        "100.64.0.20",
        22,
        "operator",
        "",
        ssh_sha256_fingerprint(key_bytes),
        auth_mode="tailscale",
    )

    with pytest.raises(RuntimeError, match="Tailscale SSH authentication failed.*action=check"):
        ParamikoConnector(timeout_s=4).connect(endpoint)


def test_paramiko_connector_does_not_treat_transport_failure_as_auth_fallback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import elesim_setup.secure_deployment as secure_deployment

    key_bytes = b"tailscale-server-key"
    raw_socket = SimpleNamespace(close=lambda: None)
    monkeypatch.setattr(
        secure_deployment.socket,
        "create_connection",
        lambda address, timeout: raw_socket,
    )

    class AuthenticationException(Exception):
        pass

    class Key:
        def asbytes(self) -> bytes:
            return key_bytes

    class Transport:
        password_attempted = False

        def __init__(self, _connection) -> None:
            pass

        def start_client(self, timeout) -> None:
            pass

        def get_remote_server_key(self):
            return Key()

        def auth_none(self, username):
            raise RuntimeError("transport EOF")

        def auth_password(self, username, password):
            self.__class__.password_attempted = True

        def close(self) -> None:
            pass

    monkeypatch.setitem(
        sys.modules,
        "paramiko",
        SimpleNamespace(
            AuthenticationException=AuthenticationException,
            Transport=Transport,
            SSHClient=lambda: object(),
        ),
    )
    endpoint = SshEndpoint(
        "100.64.0.20",
        22,
        "operator",
        "",
        ssh_sha256_fingerprint(key_bytes),
        auth_mode="tailscale",
    )

    with pytest.raises(RuntimeError, match="transport EOF"):
        ParamikoConnector(timeout_s=4).connect(endpoint)
    assert Transport.password_attempted is False


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
        if values[:2] == ("sha256sum", "--"):
            path = PurePosixPath(values[2])
            content = next(
                uploaded
                for uploaded_path, uploaded, _mode in self.uploads
                if uploaded_path == path
            )
            import hashlib

            return RemoteCommandResult(0, f"{hashlib.sha256(content).hexdigest()}  {path}\n")
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
        return RemoteCapabilities(True, True, False, True, "x86_64")

    def runtime_network_check(self, _session, _host) -> None:
        pass

    def runtime_launch_preflight(self, _session, _host) -> None:
        pass

    def snapshot(self, _session, host):
        return {"roles": list(host.roles)}

    def configure(self, _session, _host, _generation, _root) -> None:
        pass

    def restore(self, _session, _host, _configuration) -> None:
        pass

    def stop(self, _session, _host, _roles) -> None:
        pass

    def start(self, _session, _host, _roles) -> None:
        pass

    def build(self, _session, _host, _output) -> None:
        pass

    def launch(self, _session, _host) -> None:
        pass

    def status(self, _session, host):
        return {"running_roles": list(host.roles)}

    def verify(self, _session, _host, _generation, _running_roles) -> None:
        pass


def test_ssh_host_operations_stage_manifest_then_atomically_activate() -> None:
    host = _topology().host("server")
    topology = _topology()
    session = FakeSession()
    connector = FakeConnector(session)
    operations = SshHostOperations(
        connector, FakeLifecycle(), topology
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
    assert len(connector.endpoints) == 1
    assert connector.endpoints[0].host == "server.example"
    assert connector.endpoints[0].host != host.dds.address
    operations.close()


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
    pilot_root = Path(host.install_root) / "security/roles/pilot"
    ui_root = Path(host.install_root) / "security/roles/ui"
    assert (pilot_root / "enclaves/pilot/key.pem").read_bytes() == (
        b"g2-pilot"
    )
    assert (ui_root / "enclaves/ui/key.pem").read_bytes() == b"g2-ui"


def test_role_view_switch_and_clear_keep_stable_root_inode(tmp_path: Path) -> None:
    raw = _topology().to_dict()
    raw["hosts"][0]["install_root"] = str(tmp_path / "install")
    raw["hosts"][0]["bin_dir"] = str(tmp_path / "bin")
    topology = ConnectionTopology.from_dict(raw)
    host = topology.host("laptop")
    role_root = Path(host.install_root) / "security/roles/pilot"
    role_root.mkdir(parents=True)
    inode = role_root.stat().st_ino
    operations = LocalHostOperations(FakeLifecycle(), topology)

    operations.stage(host, _bundle("laptop", "g1"))
    operations.activate(host, "g1")
    previous = HostActivationState("g1", {"dds": {}, "network": {}})
    operations.stage(host, _bundle("laptop", "g2"))
    operations.activate(host, "g2")
    assert role_root.stat().st_ino == inode
    assert (role_root / "enclaves/pilot/key.pem").read_bytes() == (
        b"g2-pilot"
    )

    operations.rollback(host, previous)
    assert role_root.stat().st_ino == inode
    assert (role_root / "enclaves/pilot/key.pem").read_bytes() == (
        b"g1-pilot"
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
    (security / "roles/pilot").symlink_to(target, target_is_directory=True)
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
                        "roles": ["sim"],
                        "prefix": "/opt/elesim",
                        "bin_dir": "/usr/local/bin",
                        "install_mode": "container",
                        "dds": {},
                        "network": {
                            "turn_urls": [
                                "turn:100.64.0.2:3478?transport=udp"
                            ]
                        },
                        "turn": {
                            "mode": "managed",
                            "realm": "elesim.local",
                            "public_host": "100.64.0.2",
                            "secret_file": "/opt/elesim/secrets/turn.secret",
                        },
                    }
                ),
            )
        if values[:2] == ("uname", "-m"):
            return RemoteCommandResult(0, "x86_64\n")
        if values[:2] == ("test", "-f"):
            return RemoteCommandResult(0)
        if values[:2] == ("test", "-L"):
            return RemoteCommandResult(1)
        if values[-2:] == ("config", "--services"):
            return RemoteCommandResult(0, "sim coturn\n")
        return RemoteCommandResult(0, "26.1\n" if values[0] == "docker" else "")


class SidecarNetworkSession:
    def __init__(self, statuses: list[dict[str, str]]) -> None:
        self.statuses = list(statuses)
        self.events: list[str] = []

    def run(self, argv, *, check=True) -> RemoteCommandResult:
        values = tuple(argv)
        if values[-1:] == ("show",):
            self.events.append("show")
            return RemoteCommandResult(
                0,
                json.dumps(
                    {
                        "roles": ["sim"],
                        "prefix": "/opt/elesim",
                        "bin_dir": "/usr/local/bin",
                        "install_mode": "container",
                        "container_network": {"mode": "tailscale-sidecar"},
                    }
                ),
            )
        if values[-2:] == ("status", "--json"):
            self.events.append(f"status:{check}")
            if not self.statuses:
                raise AssertionError("unexpected extra Tailscale status call")
            return RemoteCommandResult(0, json.dumps(self.statuses.pop(0)))
        raise AssertionError(f"unexpected command: {values!r}")

    def run_streaming(self, argv, *, output, check=True) -> RemoteCommandResult:
        values = tuple(argv)
        if values[-1:] != ("login",):
            raise AssertionError(f"unexpected streaming command: {values!r}")
        self.events.append("login")
        output("stdout", "already authenticated\n")
        return RemoteCommandResult(0)


def test_runtime_network_preparation_skips_login_for_a_ready_sidecar() -> None:
    topology = _topology()
    host = topology.host("server")
    session = SidecarNetworkSession(
        [{"BackendState": "Running", "IPv4": "100.64.0.42"}]
    )
    output: list[tuple[str, str]] = []

    address = InstalledElesimLifecycle(topology).prepare_runtime_network(
        session, host, lambda stream, text: output.append((stream, text))
    )

    assert address == "100.64.0.42"
    assert session.events == ["show", "status:False"]
    assert output == []


def test_runtime_network_preparation_requires_explicit_sidecar_login() -> None:
    topology = _topology()
    host = topology.host("server")
    session = SidecarNetworkSession(
        [{"BackendState": "NeedsLogin", "IPv4": ""}]
    )
    output: list[tuple[str, str]] = []

    with pytest.raises(RuntimeError, match="elesim-tailscale login"):
        InstalledElesimLifecycle(topology).prepare_runtime_network(
            session, host, lambda stream, text: output.append((stream, text))
        )

    assert session.events == ["show", "status:False"]
    assert output == []


def test_concrete_lifecycle_preflight_and_managed_configuration_command() -> None:
    topology = _topology()
    host = topology.host("server")
    session = LifecycleSession()
    lifecycle = InstalledElesimLifecycle(topology)

    capabilities = lifecycle.preflight(
        session, host, PurePosixPath("/opt/elesim/security")
    )
    assert not any("namespace-check" in argv for argv, _check in session.commands)
    lifecycle.runtime_network_check(session, host)
    lifecycle.runtime_launch_preflight(session, host)
    lifecycle.configure(
        session, host, "g2", PurePosixPath("/opt/elesim/security")
    )
    lifecycle.stop(session, host, host.roles)
    lifecycle.start(session, host, host.roles)

    assert capabilities.docker
    assert capabilities.security_root_writable
    assert (
        (
            "/usr/local/bin/elesim-net",
            "namespace-check",
            "--dds-interface",
            "tailscale0",
            "--dds-address",
            "100.64.0.2",
            "--dds-peer",
            "100.64.0.1",
            "--dds-peer",
            "100.64.0.3",
        ),
        True,
    ) in session.commands
    assert (
        (("/usr/local/bin/elesim-net", "configuration-check"), True)
        in session.commands
    )
    configure = next(
        argv for argv, _check in session.commands if "configure" in argv
    )

    assert "--dds-security-provisioning" in configure
    assert configure[configure.index("--dds-security-generation") + 1] == "g2"
    assert configure[configure.index("--dds-security-bundle") + 1] == (
        "/opt/elesim/security/current/keystore"
    )
    assert configure[configure.index("--dds-enclave") + 1] == "/elesim/lab"
    assert configure[configure.index("--sim-id") + 1] == "sim-main"
    assert "--pilot-id" not in configure
    assert "--ui-id" not in configure
    assert "--robot-id" not in configure
    assert configure[configure.index("--turn-mode") + 1] == "managed"
    assert configure[configure.index("--turn-url") + 1] == (
        "turn:100.64.0.2:3478?transport=udp"
    )
    assert configure[configure.index("--turn-secret-file") + 1] == (
        "/opt/elesim/secrets/turn.secret"
    )
    compose_commands = [
        argv
        for argv, _check in session.commands
        if PurePosixPath(argv[0]).name == "elesim-compose"
    ]
    assert (
        "/usr/local/bin/elesim-compose",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "stop",
        "sim",
        "coturn",
    ) in compose_commands
    assert (
        "/usr/local/bin/elesim-compose",
        "-p",
        "elesim-runtime",
        "-f",
        "/opt/elesim/containers/compose.yaml",
        "start",
        "sim",
        "coturn",
    ) in compose_commands


def test_preflight_checks_writable_security_mount_not_read_only_install_prefix() -> None:
    topology = _topology()
    host = topology.host("server")

    class ManagerMountSession(LifecycleSession):
        def run(self, argv, *, check=True) -> RemoteCommandResult:
            values = tuple(argv)
            if values[:2] == ("test", "-w"):
                self.commands.append((values, check))
                # The manager sees the installation prefix through a read-only
                # home mount, while the exact security bind is writable.
                return RemoteCommandResult(
                    0 if values[2] == "/opt/elesim/security" else 1
                )
            return super().run(argv, check=check)

    session = ManagerMountSession()
    capabilities = InstalledElesimLifecycle(topology).preflight(
        session, host, PurePosixPath("/opt/elesim/security")
    )

    assert capabilities.security_root_writable
    assert ("test", "-w", "/opt/elesim/security") in {
        argv for argv, _check in session.commands
    }
    assert ("test", "-w", "/opt/elesim") not in {
        argv for argv, _check in session.commands
    }


def test_lifecycle_preflight_rejects_symlinked_managed_turn_secret() -> None:
    class SymlinkSession(LifecycleSession):
        def run(self, argv, *, check=True) -> RemoteCommandResult:
            if tuple(argv)[:2] == ("test", "-L"):
                return RemoteCommandResult(0)
            return super().run(argv, check=check)

    topology = _topology()
    with pytest.raises(RuntimeError, match="secret path is a symlink"):
        InstalledElesimLifecycle(topology).preflight(
            SymlinkSession(),
            topology.host("server"),
            PurePosixPath("/opt/elesim/security"),
        )


def test_lifecycle_preflight_rejects_symlinked_managed_turn_secret_ancestor() -> None:
    class AncestorSymlinkSession(LifecycleSession):
        def run(self, argv, *, check=True) -> RemoteCommandResult:
            values = tuple(argv)
            if values[:2] == ("test", "-L") and values[2] == "/opt/elesim/secrets":
                return RemoteCommandResult(0)
            return super().run(argv, check=check)

    topology = _topology()
    with pytest.raises(RuntimeError, match="secret path is a symlink"):
        InstalledElesimLifecycle(topology).preflight(
            AncestorSymlinkSession(),
            topology.host("server"),
            PurePosixPath("/opt/elesim/security"),
        )


def test_managed_turn_secret_must_stay_under_sim_install_root() -> None:
    host = _topology().host("server")
    state = {
        "network": {"turn_urls": ["turn:100.64.0.2:3478?transport=udp"]},
        "turn": {
            "mode": "managed",
            "realm": "elesim.local",
            "public_host": "100.64.0.2",
            "secret_file": "/etc/shadow",
        },
    }

    with pytest.raises(RuntimeError, match="under the Sim installation root"):
        _managed_turn_from_state(state, host)


def test_pending_managed_turn_uses_current_sim_address() -> None:
    host = _topology().host("server")
    state = {
        "network": {"turn_urls": []},
        "turn": {
            "mode": "managed",
            "realm": "elesim.local",
            "public_host": "",
            "secret_file": "/opt/elesim/secrets/turn.secret",
        },
    }

    assert _managed_turn_from_state(state, host) == {
        "turn_url": "turn:100.64.0.2:3478?transport=udp",
        "turn_realm": "elesim.local",
        "turn_public_host": "100.64.0.2",
        "turn_secret_file": "/opt/elesim/secrets/turn.secret",
    }


def test_runtime_doctor_requests_strict_peer_json() -> None:
    class DoctorSession:
        def __init__(self) -> None:
            self.commands: list[tuple[tuple[str, ...], bool]] = []

        def run(self, argv, *, check=True) -> RemoteCommandResult:
            command = tuple(argv)
            self.commands.append((command, check))
            return RemoteCommandResult(
                1,
                json.dumps({"ok": False, "results": []}),
                "peer is still pending\n",
            )

    topology = _topology()
    session = DoctorSession()
    report = InstalledElesimLifecycle(topology).runtime_doctor(
        session,
        topology.host("server"),
        ("pilot-main", "ui-main", ""),
        8.0,
    )

    assert report == {"ok": False, "results": []}
    assert session.commands == [
        (
            (
                "/usr/local/bin/elesim-net",
                "doctor",
                "--timeout",
                "8",
                "--json",
                "--strict-peers",
                "--readiness-only",
                "--expect-peer",
                "pilot-main",
                "--expect-peer",
                "ui-main",
            ),
            False,
        )
    ]


def test_runtime_doctor_explains_non_json_remote_output() -> None:
    class DoctorSession:
        @staticmethod
        def run(_argv, *, check=True) -> RemoteCommandResult:
            return RemoteCommandResult(2, "build progress\n", "docker failed\n")

    topology = _topology()
    with pytest.raises(RuntimeError, match="invalid JSON.*server"):
        InstalledElesimLifecycle(topology).runtime_doctor(
            DoctorSession(), topology.host("server"), (), 4.0
        )


def test_runtime_network_check_never_mixes_ssh_ports_into_dds_preflight() -> None:
    raw = _topology().to_dict()
    raw["hosts"][1]["ssh"].update(
        {"port": 22, "identity_file": "", "auth_mode": "tailscale"}
    )
    topology = ConnectionTopology.from_dict(raw)
    session = LifecycleSession()

    InstalledElesimLifecycle(topology).runtime_network_check(
        session, topology.host("laptop")
    )

    probe = next(
        argv for argv, _check in session.commands if "namespace-check" in argv
    )
    assert "--tcp-peer" not in probe
    assert "100.64.0.2" in probe

    raw["hosts"][1]["ssh"]["host"] = raw["hosts"][1]["dds"]["address"]
    shared = ConnectionTopology.from_dict(raw)
    session = LifecycleSession()
    InstalledElesimLifecycle(shared).runtime_network_check(
        session, shared.host("laptop")
    )
    probe = next(
        argv for argv, _check in session.commands if "namespace-check" in argv
    )
    assert "--tcp-peer" not in probe
    assert "100.64.0.2" in probe


def test_lifecycle_status_ignores_manager_service() -> None:
    class StatusSession:
        def run(self, argv, *, check=True) -> RemoteCommandResult:
            return RemoteCommandResult(0, "manager\n")

    topology = _topology()
    status = InstalledElesimLifecycle(topology).status(
        StatusSession(), topology.host("laptop")
    )

    assert status["state"] == "stopped"
    assert status["running_roles"] == []
    assert status["containers_present"] is False


def test_lifecycle_status_counts_managed_coturn_for_sim_readiness() -> None:
    class SimStatusSession:
        def __init__(self, services: str) -> None:
            self.services = services

        def run(self, _argv, *, check=True) -> RemoteCommandResult:
            return RemoteCommandResult(0, self.services)

    topology = _topology()
    lifecycle = InstalledElesimLifecycle(topology)
    without_relay = lifecycle.status(
        SimStatusSession("sim\n"), topology.host("server")
    )
    assert without_relay["state"] == "degraded"
    assert without_relay["containers_present"] is True
    assert "managed Coturn" in without_relay["detail"]

    with_relay = lifecycle.status(
        SimStatusSession("sim\ncoturn\n"), topology.host("server")
    )
    assert with_relay["state"] == "running"
    assert with_relay["containers_present"] is True


@pytest.mark.parametrize(
    "failed_tail",
    (
        ("ps", "--all", "--services"),
        ("ps", "--status", "running", "--services"),
        ("config", "--services"),
    ),
)
def test_lifecycle_status_surfaces_compose_query_failures(
    failed_tail: tuple[str, ...],
) -> None:
    class FailedStatusSession:
        @staticmethod
        def run(argv, *, check=True) -> RemoteCommandResult:
            values = tuple(argv)
            if values[-len(failed_tail) :] == failed_tail:
                return RemoteCommandResult(7, "", "compose query failed\n")
            return RemoteCommandResult(0, "pilot\nui\n")

    topology = _topology()
    with pytest.raises(RemoteCommandError, match="compose query failed") as captured:
        InstalledElesimLifecycle(topology).status(
            FailedStatusSession(), topology.host("laptop")
        )

    assert captured.value.argv[-len(failed_tail) :] == failed_tail
    assert captured.value.result.exit_status == 7


def test_simulation_only_configuration_does_not_emit_robot_endpoint() -> None:
    topology = ConnectionTopology(
        "lab_sim",
        "trusted-network",
        (
            ManagedHost(
                "laptop",
                True,
                DdsEndpoint("100.64.0.40", "tailscale0"),
                None,
                (
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
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
    assert configure[configure.index("--sim-id") + 1] == "sim-main"
    assert configure[configure.index("--turn-mode") + 1] == "none"
    assert "--clear-turn" in configure


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
            docker=bool(host.runtime_units),
            systemd=bool(host.robot_units),
            jetson=host.jetson,
            security_root_writable=True,
            architecture="x86_64",
        )

    def runtime_network_check(self, _host):
        return None

    def capture_state(self, host):
        self._event("current")
        return HostActivationState("g1", {"dds": {}}, host.roles)

    def stage(self, _host, _bundle):
        self._event("stage")

    def discard_generation(self, _host, _generation):
        self._event("discard")

    def stop(self, _host, _roles=None):
        self._event("stop")

    def activate(self, _host, _generation):
        self._event("activate")

    def configure_topology(self, _host):
        self._event("configure")

    def start(self, _host, _roles=None):
        self._event("start")

    def build(self, _host, _output):
        self._event("build")

    def launch(self, _host):
        self._event("launch")

    def status(self, host):
        return {"running_roles": list(host.roles)}

    def verify(self, _host, _generation, _running_roles=()):
        self._event("verify")

    def verify_topology(self, _host, _running_roles=()):
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


def test_issue_and_apply_preflights_before_issuing_generation() -> None:
    topology, events, operations, _bundles = _rollout_parts(
        fail_host="laptop", fail_once="preflight"
    )

    class Issuer:
        called = False

        def issue(self, _topology, _generation):
            self.called = True
            raise AssertionError("generation issuance must follow preflight")

    issuer = Issuer()
    with pytest.raises(RolloutError, match="during preflight"):
        GenerationRollout(topology, operations).issue_and_apply(issuer, "g2")

    assert issuer.called is False
    assert events == ["preflight:laptop"]


def test_issue_and_apply_captures_every_host_before_issuing_generation() -> None:
    topology, events, operations, bundles = _rollout_parts()

    class Issuer:
        def issue(self, _topology, _generation):
            assert events == [
                "preflight:laptop",
                "preflight:server",
                "preflight:robot",
                "current:laptop",
                "current:server",
                "current:robot",
            ]
            events.append("issue")
            return SimpleNamespace(
                generation="g2",
                bundles=bundles,
                activate_authority=lambda: None,
                rollback_authority=lambda: None,
            )

    result = GenerationRollout(topology, operations).issue_and_apply(Issuer(), "g2")

    assert result.generation == "g2"
    assert events[6] == "issue"


def test_simulation_only_trusted_rollout_accepts_one_compose_host() -> None:
    topology = ConnectionTopology(
        "lab_sim",
        "trusted-network",
        (
            ManagedHost(
                "laptop",
                True,
                DdsEndpoint("100.64.0.40", "tailscale0"),
                None,
                (
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
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


def test_initial_security_rollout_does_not_start_roles_that_were_stopped() -> None:
    topology, events, _operations, bundles = _rollout_parts()

    class StoppedOperations(FakeOperations):
        def capture_state(self, host):
            self._event("current")
            return HostActivationState(
                None,
                {
                    "roles": list(host.roles),
                    "prefix": host.install_root,
                    "bin_dir": host.bin_dir,
                    "install_mode": host.install_mode,
                    "dds": {},
                },
                (),
            )

    stopped_operations = {
        host.host_id: StoppedOperations(host.host_id, events)
        for host in topology.hosts
    }

    GenerationRollout(topology, stopped_operations).apply("g2", bundles)

    assert not any(event.startswith("stop:") for event in events)
    assert not any(event.startswith("start:") for event in events)
    assert [event for event in events if event.startswith("verify:")] == [
        "verify:laptop",
        "verify:server",
        "verify:robot",
    ]


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


def test_rollout_failure_restores_switched_hosts_when_every_role_was_stopped() -> None:
    topology, events, _operations, bundles = _rollout_parts()

    class StoppedOperations(FakeOperations):
        def capture_state(self, host):
            self._event("current")
            return HostActivationState("g1", {"dds": {}}, ())

    operations = {
        host.host_id: StoppedOperations(
            host.host_id,
            events,
            fail_once="verify" if host.host_id == "server" else "",
        )
        for host in topology.hosts
    }

    with pytest.raises(RolloutError, match="during verify"):
        GenerationRollout(topology, operations).apply("g2", bundles)

    assert [event for event in events if event.startswith("rollback:")] == [
        "rollback:robot",
        "rollback:server",
        "rollback:laptop",
    ]
    assert not any(event.startswith("start:") for event in events)


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
