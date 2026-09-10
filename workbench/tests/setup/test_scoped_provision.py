from __future__ import annotations

from pathlib import Path

from elesim_setup.connection_manager import (
    ConnectionTopology,
    DdsEndpoint,
    ManagedHost,
    RoleAssignment,
)
from elesim_setup.connections import ConnectionDeploymentRunner
from elesim_setup.secure_deployment import (
    InstalledElesimLifecycle,
    RemoteCommandResult,
    SecurityBundle,
    SecurityFile,
)
from elesim_setup.instance_identity import image_reference
from elesim_setup.releases import ReleaseManifest
from elesim_setup.state import InstallState


INSTALL = "01234567-89ab-cdef-0123-456789abcdef"


def _topology(prefix: Path) -> ConnectionTopology:
    return ConnectionTopology(
        system_id="lab",
        security_profile="sros2",
        hosts=(
            ManagedHost(
                host_id="operator",
                local=True,
                dds=DdsEndpoint("10.0.0.10", "eth0"),
                ssh=None,
                assignments=(
                    RoleAssignment("pilot", "pilot-main"),
                    RoleAssignment("sim", "sim-main"),
                    RoleAssignment("ui", "ui-main"),
                ),
                install_root=str(prefix),
                bin_dir=str(prefix / "bin"),
            ),
        ),
    ).validate()


def _release() -> ReleaseManifest:
    return ReleaseManifest(
        install_uuid=INSTALL,
        source_revision="git-" + "a" * 40,
        platform="linux/amd64",
        role_images={
            role: image_reference(INSTALL, role, "c" * 64)
            for role in ("pilot", "sim", "ui")
        },
        image_ids={role: "sha256:" + "b" * 64 for role in ("pilot", "sim", "ui")},
        build_fingerprints={role: "c" * 64 for role in ("pilot", "sim", "ui")},
        runtime_data_digest="d" * 64,
    ).validate()


def _bundle(generation: str) -> SecurityBundle:
    files: list[SecurityFile] = []
    for role, endpoint in (
        ("pilot", "pilot-main"),
        ("sim", "sim-main"),
        ("ui", "ui-main"),
    ):
        prefix = f"apps/{role}/keystore/"
        enclave = f"enclaves/elesim/lab/{role}/{endpoint.replace('-', '_')}"
        files.extend(
            (
                SecurityFile(prefix + "public/identity_ca.cert.pem", b"public"),
                SecurityFile(prefix + enclave + "/cert.pem", b"cert"),
                SecurityFile(prefix + enclave + "/key.pem", b"key"),
            )
        )
    return SecurityBundle("lab", "operator", generation, tuple(files)).validate()


def test_scoped_provision_prepares_network_then_registers_exact_instance(
    tmp_path: Path, monkeypatch
) -> None:
    prefix = tmp_path / "install"
    prefix.mkdir()
    (prefix / "install-ownership.json").write_text("owned", encoding="utf-8")
    topology = _topology(prefix)
    release = _release()
    state = InstallState(
        profile="custom",
        roles=("pilot", "sim", "ui"),
        prefix=str(prefix),
        bin_dir=str(prefix / "bin"),
        source_root=str(tmp_path),
    )
    events: list[str] = []

    class Docker:
        project = "elesim-runtime-0123456789abcdef"

    class Manifest:
        install_uuid = INSTALL
        prefix_path = prefix
        docker = Docker()

    class Authority:
        def __init__(self, _root: Path) -> None:
            pass

        @staticmethod
        def active():
            return None

    class Issued:
        generation = "g-20260909t000000000000z-abcdef123456"
        bundles = {"operator": _bundle(generation)}

        def activate_authority(self):
            events.append("authority-activate")

        def rollback_authority(self):
            events.append("authority-rollback")

    class Issuer:
        def __init__(self, _authority) -> None:
            pass

        def issue(self, _topology, generation):
            events.append("issue")
            assert generation == Issued.generation
            return Issued()

    class Operations:
        def prepare_runtime_network(self, host, output):
            assert host.host_id == "operator"
            events.append("network")
            output("stdout", "network ready\n")
            return None

        def register_scoped_instance(
            self, host, unit, instance, selected_release, bundle,
            *, replace_existing=False,
        ):
            events.append("register")
            assert host.host_id == "operator"
            assert unit.unit_id == "runtime"
            assert instance.system_id == "lab"
            assert instance.security_profile == "sros2"
            assert instance.security_generation == Issued.generation
            assert len(instance.endpoints) == 3
            assert selected_release == release
            assert bundle is not None
            assert bundle.generation == Issued.generation
            assert replace_existing is False

        def scoped_instance_state(self, *_args):
            return None

        def close(self):
            events.append("close")

    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_setup.connections.Sros2BundleIssuer", Issuer)
    monkeypatch.setattr(
        "elesim_setup.connections.new_generation_id",
        lambda: "g-20260909t000000000000z-abcdef123456",
    )
    monkeypatch.setattr("elesim_setup.connections.OwnershipManifest.load", lambda _path: Manifest())
    monkeypatch.setattr(
        "elesim_setup.connections.list_releases",
        lambda *_args, **_kwargs: (release,),
    )
    monkeypatch.setattr(
        ConnectionDeploymentRunner, "_local_install_scope", lambda _self: True
    )
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_state_for_local_scope",
        lambda _self, _uuid: state,
    )
    operations = Operations()
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        lambda _self, _topology: {"operator": operations},
    )

    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=prefix)
    runner(topology, "provision", lambda message: events.append(message))

    events = [
        event
        for event in events
        if event
        in {
            "network",
            "issue",
            "register",
            "authority-activate",
            "authority-rollback",
        }
    ]
    assert events[:4] == [
        "network",
        "issue",
        "register",
        "authority-activate",
    ]
    assert "authority-rollback" not in events


def test_scoped_first_provision_rejects_ambiguous_release_set(
    tmp_path: Path, monkeypatch
) -> None:
    prefix = tmp_path / "install"
    prefix.mkdir()
    (prefix / "install-ownership.json").write_text("owned", encoding="utf-8")
    topology = _topology(prefix)

    class Docker:
        project = "elesim-runtime-0123456789abcdef"

    class Manifest:
        install_uuid = INSTALL
        prefix_path = prefix
        docker = Docker()

    class Authority:
        def __init__(self, _root: Path) -> None:
            pass

        @staticmethod
        def active():
            return None

    monkeypatch.setattr("elesim_setup.connections.Sros2Authority", Authority)
    monkeypatch.setattr("elesim_setup.connections.OwnershipManifest.load", lambda _path: Manifest())
    monkeypatch.setattr(
        "elesim_setup.connections.list_releases",
        lambda *_args, **_kwargs: (_release(), _release()),
    )
    monkeypatch.setattr(ConnectionDeploymentRunner, "_local_install_scope", lambda _self: True)
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_state_for_local_scope",
        lambda _self, _uuid: InstallState(
            profile="custom",
            roles=("pilot", "sim", "ui"),
            prefix=str(prefix),
            bin_dir=str(prefix / "bin"),
            source_root=str(tmp_path),
        ),
    )
    monkeypatch.setattr(
        ConnectionDeploymentRunner,
        "_operations",
        lambda _self, _topology: {
            "operator": type(
                "Operations",
                (),
                {
                    "prepare_runtime_network": lambda _ops, _host, _output: None,
                    "close": lambda _ops: None,
                },
            )()
        },
    )

    runner = ConnectionDeploymentRunner(tmp_path / "authority", local_install_root=prefix)
    try:
        runner(topology, "provision", lambda _message: None)
    except ValueError as exc:
        assert "exactly one selected published" in str(exc)
    else:  # pragma: no cover - assertion clarity
        raise AssertionError("ambiguous releases must fail closed")


def test_scoped_preflight_reads_instance_security_boundary(tmp_path: Path) -> None:
    topology = _topology(tmp_path / "install")
    topology = ConnectionTopology(
        topology.system_id,
        topology.security_profile,
        (
            ManagedHost(
                "operator",
                True,
                DdsEndpoint("10.0.0.10", "eth0"),
                None,
                (RoleAssignment("pilot", "pilot-main"),),
                install_root=str(tmp_path / "install"),
                bin_dir=str(tmp_path / "install" / "bin"),
            ),
        ),
        topology.dds_graph,
    ).validate()

    class Session:
        def __init__(self) -> None:
            self.commands: list[tuple[str, ...]] = []

        def run(self, argv, *, check=True):
            values = tuple(str(value) for value in argv)
            self.commands.append(values)
            if values[0:2] == ("test", "-L"):
                return RemoteCommandResult(
                    0 if values[-1].endswith("/security/current") else 1
                )
            if values[0:2] == ("test", "-e"):
                return RemoteCommandResult(1 if "provisioning-required" in values[-1] else 0)
            if values[0:2] in {("test", "-x"), ("test", "-w"), ("test", "-f")}:
                return RemoteCommandResult(0)
            if values[0] == "cat":
                return RemoteCommandResult(
                    0,
                    '{"system_id":"lab","security_profile":"sros2",'
                    '"security_generation":"g1","endpoints":'
                    '[{"role":"pilot","endpoint_id":"pilot-main"}]}',
                )
            if values[0] == "readlink":
                return RemoteCommandResult(0, "generations/g1\n")
            if values and values[0].endswith("elesim-net") and values[1:] == ("show",):
                return RemoteCommandResult(
                    0,
                    '{"roles":["pilot"],"prefix":"'
                    + str(tmp_path / "install")
                    + '","bin_dir":"'
                    + str(tmp_path / "install" / "bin")
                    + '","install_mode":"container",'
                    '"dds":{}}',
                )
            if values == ("uname", "-m"):
                return RemoteCommandResult(0, "x86_64\n")
            return RemoteCommandResult(0)

    session = Session()
    lifecycle = InstalledElesimLifecycle(topology, scoped=True)
    capabilities = lifecycle.preflight(
        session,
        topology.local_host,
        Path("/tmp/install/security"),
    )
    assert capabilities.security_root_writable
    assert any(
        command
            == (
                "cat",
                str(tmp_path / "install" / "instances" / "lab" / "state.json"),
        )
        for command in session.commands
    )
    assert any(
        command
            == (
                "readlink",
                str(tmp_path / "install" / "instances" / "lab" / "security" / "current"),
        )
        for command in session.commands
    )
