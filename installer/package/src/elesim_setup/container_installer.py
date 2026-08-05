"""Generate a host-preserving Docker Compose installation for Elesim roles."""

from __future__ import annotations

import os
import secrets
import shlex
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import yaml

from .configuration import (
    dds_enclave,
    generate_role_configs,
    role_keystore_path,
    role_directory,
)
from .credentials import validate_external_turn_credentials
from .manager_lifecycle import manager_lifecycle_fragment
from .ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    DockerOwnership,
    HostUninstallerBundle,
    OwnershipManifest,
    OwnershipRefresh,
    install_host_uninstaller_bundle,
    ownership_install_uuid,
    prepare_ownership_refresh,
    write_ownership_manifest,
)
from .security_provisioning import (
    launch_guard,
    provisioning_required_path,
    sync_provisioning_required,
)
from .security_views import prepare_role_keystore_views
from .shell import operator_home, write_executable
from .state import InstallState


@dataclass(frozen=True)
class ContainerAction:
    title: str
    detail: str


GENERAL_COMPOSE_PROJECT = "elesim-runtime"
ROLE_CONTAINER_NAMES = {
    "pilot": "elesim-pilot",
    "ui": "elesim-ui",
    "sim": "elesim-sim",
}
# Detect containers created by the immediately preceding naming scheme so a
# second installation cannot leave an old role process running beside the new
# fixed name.
# Names emitted by the pre-pilot/sim installer.  They are only inspected during
# cleanup so an upgrade cannot leave an old process running beside the new one.
LEGACY_ROLE_CONTAINER_NAMES = ("elesim-controller", "elesim-simulator")
DOCKER_LOGGING = {
    "driver": "json-file",
    "options": {"max-size": "10m", "max-file": "4"},
}
RUNTIME_LOG_RETENTION = 5


def build_container_plan(state: InstallState) -> tuple[ContainerAction, ...]:
    state.validate()
    root = state.prefix_path / "containers"
    actions = [
        ContainerAction("호스트", "기존 Python/APT 환경은 변경하지 않음"),
        ContainerAction("Compose", f"Linux host-network project: {root / 'compose.yaml'}"),
    ]
    actions.extend(
        ContainerAction(role, f"격리 이미지 context: {root / 'build' / role}")
        for role in state.roles
    )
    actions.extend(
        (
            ContainerAction("도구", "elesim-setup/elesim-net 전용 tools image"),
            ContainerAction("명령", f"Compose 실행 래퍼: {state.bin_path}"),
            ContainerAction("시작", f"{state.bin_path / 'elesim-up'}"),
        )
    )
    return tuple(actions)


class ContainerInstaller:
    _ROS_BASE_IMAGE = "ros:humble-ros-base-jammy"

    def __init__(
        self,
        state: InstallState,
        *,
        state_path: Path | None = None,
        shell_bashrc: Path | None = None,
        dry_run: bool = False,
        log: Callable[[str], None] = print,
    ) -> None:
        self.state = state.validate()
        if self.state.install_mode != "container":
            raise ValueError("ContainerInstaller에는 install_mode=container가 필요합니다")
        self.state_path = (
            self.state.state_path
            if state_path is None
            else state_path.expanduser().resolve()
        )
        self.dry_run = bool(dry_run)
        self.log = log
        self._install_uuid = ""
        self.shell_bashrc = (
            None if shell_bashrc is None else shell_bashrc.expanduser().resolve()
        )

    @property
    def container_root(self) -> Path:
        return self.state.prefix_path / "containers"

    def run(self) -> None:
        self._validate_source()
        self.state.require_installable_dds()
        self._validate_external_turn_credentials()
        ownership_refresh = prepare_ownership_refresh(
            prefix=self.state.prefix_path,
            bin_dir=self.state.bin_path,
            edition="general",
            claimed_paths=self._claimed_paths(),
        )
        self._install_uuid = ownership_install_uuid(ownership_refresh)
        prefix_created = not os.path.lexists(self.state.prefix_path)
        bin_created = not os.path.lexists(self.state.bin_path)
        self.log("\n컨테이너 설치 계획")
        for action in build_container_plan(self.state):
            self.log(f"  [{action.title}] {action.detail}")
        self.log("")
        if self.dry_run:
            self.log("[DRY-RUN] 호스트나 Docker daemon을 변경하지 않았습니다.")
            return

        self.log("[1/6] 설치 디렉터리와 runtime data 준비")
        self.state.prefix_path.mkdir(parents=True, exist_ok=True)
        self.state.bin_path.mkdir(parents=True, exist_ok=True)
        security_root = self.state.prefix_path / "security"
        if security_root.is_symlink():
            raise ValueError(f"security root는 symlink일 수 없습니다: {security_root}")
        security_root.mkdir(mode=0o700, exist_ok=True)
        security_root.chmod(0o700)
        prepare_role_keystore_views(self.state)
        if self.state.dds.managed_security_pending:
            sync_provisioning_required(self.state)
        self._prepare_turn_secret()
        self._copy_runtime_data()
        generate_role_configs(self.state)
        self.log("[2/6] 역할별 image context 생성")
        for role in self.state.roles:
            self._write_role_context(role)
        self.log("[3/6] 설치/진단 tools context 생성")
        self._write_tools_context()
        self.log("[4/6] Compose 구성 생성")
        self._write_compose()
        self.log("[5/6] 실행 명령 생성")
        self._write_wrappers()
        self.log("[6/6] 설치 상태와 제거 소유권 저장")
        saved = self.state.save(self.state_path)
        if not self.state.dds.managed_security_pending:
            sync_provisioning_required(self.state)
        bundle = install_host_uninstaller_bundle(
            prefix=self.state.prefix_path,
            bin_dir=self.state.bin_path,
        )
        manifest = self._write_ownership_manifest(
            bundle=bundle,
            refresh=ownership_refresh,
            prefix_created=prefix_created,
            bin_created=bin_created,
        )
        self.log(f"[완료] 설치 상태: {saved}")
        self.log(f"[완료] 제거 소유권: {manifest.path}")
        self.log(f"[다음] 이미지 빌드 및 시작: {self.state.bin_path / 'elesim-up'}")

    def _claimed_paths(self) -> tuple[Path, ...]:
        claims = [
            self.container_root,
            self.state.prefix_path / "roles",
            self.state.prefix_path / "cache",
            self.state.prefix_path / "connections",
            self.state.prefix_path / "security",
            self.state.prefix_path / "secrets",
            self.state.prefix_path / "maintenance",
            *self._wrapper_paths(include_uninstaller=True),
        ]
        if _is_within(self.state_path, self.state.prefix_path):
            claims.append(self.state_path)
        return tuple(claims)

    def _write_ownership_manifest(
        self,
        *,
        bundle: HostUninstallerBundle,
        refresh: OwnershipRefresh | None,
        prefix_created: bool,
        bin_created: bool,
    ) -> OwnershipManifest:
        wrappers = self._wrapper_paths(include_uninstaller=True)
        inventory_roots: list[Path] = [
            self.container_root,
            self.state.prefix_path / "roles",
            bundle.root,
        ]
        external_paths: list[Path] = []
        if _is_within(self.state_path, self.state.prefix_path):
            inventory_roots.append(self.state_path)
        else:
            external_paths.append(self.state_path)
            self.log(
                "[ownership] prefix 밖의 custom state file은 보존합니다: "
                f"{self.state_path}"
            )
        if (
            self.state.dds.security_profile == "sros2"
            and self.state.dds.security_provisioning == "external"
            and self.state.dds.keystore_path is not None
        ):
            external_paths.append(self.state.dds.keystore_path)
        if self.state.turn.mode == "external" and self.state.turn.credential_path is not None:
            external_paths.append(self.state.turn.credential_path)
        containers = tuple(ROLE_CONTAINER_NAMES[role] for role in self.state.roles)
        if self.state.turn.managed:
            containers = (*containers, "elesim-coturn")
        # A crashed one-shot manager is still an exact Compose-owned object.
        containers = (*containers, "elesim-manager")
        docker = DockerOwnership(
            install_uuid=self._install_uuid,
            compose_file=str(self.container_root / "compose.yaml"),
            project=GENERAL_COMPOSE_PROJECT,
            containers=containers,
            local_images=(
                *(f"elesim/{role}:local" for role in self.state.roles),
                "elesim/tools:local",
            ),
        )
        created_roots = tuple(
            path
            for path, created in (
                (self.state.prefix_path, prefix_created),
                (self.state.bin_path, bin_created),
            )
            if created
        )
        return write_ownership_manifest(
            prefix=self.state.prefix_path,
            bin_dir=self.state.bin_path,
            edition="general",
            inventory_roots=inventory_roots,
            managed_roots=(
                self.container_root,
                self.state.prefix_path / "roles",
                self.state.prefix_path / "cache",
                self.state.prefix_path / "connections",
                self.state.prefix_path / "security",
                self.state.prefix_path / "secrets",
            ),
            created_roots=created_roots,
            wrapper_paths=wrappers,
            log_roots=(self.state.prefix_path / "logs",),
            authority_roots=(self.state.prefix_path / "authority",),
            external_paths=external_paths,
            shell_bashrc=self.shell_bashrc,
            docker=docker,
            install_uuid=self._install_uuid,
            refresh=refresh,
        )

    def _validate_source(self) -> None:
        root = self.state.source_path
        required = [
            root / "packages/protocol/pyproject.toml",
            root / "packages/elesim_interfaces/package.xml",
            root / "packages/elesim_interfaces/CMakeLists.txt",
            root / "packages/elesim_interfaces/msg/RgbdFrame.msg",
            root / "installer/package/pyproject.toml",
            root / "environment/containers/Dockerfile.app",
            root / "environment/containers/Dockerfile.tools",
            root / "environment/containers/robotpkg.asc",
        ]
        required.extend(
            root / role / "pyproject.toml"
            for role in self.state.roles
        )
        if "sim" in self.state.roles:
            required.append(root / "model/bundles/default/bundle.json")
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"컨테이너 설치 소스가 불완전합니다:\n{rendered}")

    def _copy_runtime_data(self) -> None:
        root = self.state.source_path
        for role in self.state.roles:
            target = role_directory(self.state, role)
            _copy_tree((root / role) / "config", target / "config")
            if role == "sim":
                _copy_tree(
                    root / "model/bundles/default",
                    target / "model/bundles/default",
                )

    def _write_role_context(self, role: str) -> None:
        root = self.state.source_path
        context = self.container_root / "build" / role
        if context.exists():
            shutil.rmtree(context)
        context.mkdir(parents=True)
        shutil.copy2(root / "environment/containers/Dockerfile.app", context / "Dockerfile")
        shutil.copy2(root / "environment/containers/robotpkg.asc", context / "robotpkg.asc")
        source = root / role
        shutil.copy2(source / "requirements.lock", context / "requirements.lock")
        _copy_source_tree(root / "packages/protocol", context / "protocol")
        _copy_source_tree(
            root / "packages/elesim_interfaces",
            context / "interfaces/elesim_interfaces",
        )
        _copy_source_tree(source, context / "application", ignore_config=True)
        (context / "entrypoint").write_text(_entrypoint(role), encoding="utf-8")
        (context / "entrypoint").chmod(0o755)

    def _write_tools_context(self) -> None:
        root = self.state.source_path
        context = self.container_root / "build/tools"
        if context.exists():
            shutil.rmtree(context)
        context.mkdir(parents=True)
        shutil.copy2(root / "environment/containers/Dockerfile.tools", context / "Dockerfile")
        setup = root / "installer/package"
        for name in ("requirements.lock",):
            shutil.copy2(setup / name, context / name)
        _copy_source_tree(root / "packages/protocol", context / "protocol")
        _copy_source_tree(
            root / "packages/elesim_interfaces",
            context / "interfaces/elesim_interfaces",
        )
        _copy_source_tree(setup, context / "setup")

    def _write_compose(self) -> None:
        services: dict[str, object] = {}
        for role in self.state.roles:
            services[role] = self._role_service(role)
        if self.state.turn.managed:
            services["coturn"] = self._coturn_service()
        services["tools"] = self._tools_service()
        services["manager"] = self._manager_service()
        payload = {"name": GENERAL_COMPOSE_PROJECT, "services": services}
        destination = self.container_root / "compose.yaml"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    def _role_service(self, role: str) -> dict[str, object]:
        context = self.container_root / "build" / role
        role_root = role_directory(self.state, role)
        service: dict[str, object] = {
            "image": f"elesim/{role}:local",
            "container_name": ROLE_CONTAINER_NAMES[role],
            "build": {
                "context": str(context),
                "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
                "args": {
                    "ROLE": role,
                    "BASE_IMAGE": self._ROS_BASE_IMAGE,
                    "COMPUTE_MODE": self.state.compute.gpu_mode,
                    "INSTALL_GO2_MPC": "1" if self.state.install_go2_mpc else "0",
                },
            },
            "network_mode": "host",
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "restart": "unless-stopped" if role != "ui" else "no",
            "logging": {
                "driver": DOCKER_LOGGING["driver"],
                "options": dict(DOCKER_LOGGING["options"]),
            },
            "environment": self._dds_environment(role),
            "volumes": [
                f"{role_root / 'config'}:/opt/elesim/config:ro",
                (
                    f"{role_keystore_path(self.state, role)}:"
                    f"{role_keystore_path(self.state, role)}:ro"
                ),
            ],
        }
        if role == "sim":
            service["volumes"].extend(  # type: ignore[union-attr]
                (
                    f"{role_root / 'model'}:/opt/elesim/model:ro",
                    f"{self.state.prefix_path / 'cache/genesis'}:/var/lib/elesim/.cache/genesis:rw",
                )
            )
            service["shm_size"] = "2gb"
            service["ipc"] = "host"
            if self.state.turn.managed:
                secret = self.state.turn.secret_path
                if secret is None:
                    raise ValueError("managed Coturn requires a TURN secret file")
                service["volumes"].append(  # type: ignore[union-attr]
                    f"{secret}:/run/secrets/turn.secret:ro"
                )
            elif self.state.turn.mode == "external":
                credentials = self.state.turn.credential_path
                if credentials is None:
                    raise ValueError(
                        "external TURN on Sim requires a credential file"
                    )
                service["volumes"].append(  # type: ignore[union-attr]
                    f"{credentials}:/run/secrets/turn.credentials.json:ro"
                )
        if role == "ui":
            service["environment"].update(  # type: ignore[union-attr]
                {
                    "DISPLAY": "${DISPLAY:-:0}",
                    "LIBGL_ALWAYS_SOFTWARE": "${ELESIM_UI_SOFTWARE_GL:-1}",
                }
            )
            service["volumes"].append("/tmp/.X11-unix:/tmp/.X11-unix:rw")  # type: ignore[union-attr]
            xauthority = os.environ.get("XAUTHORITY", "").strip()
            if xauthority and Path(xauthority).expanduser().is_file():
                authority_path = Path(xauthority).expanduser().resolve()
                service["environment"]["XAUTHORITY"] = str(authority_path)  # type: ignore[index]
                service["volumes"].append(  # type: ignore[union-attr]
                    f"{authority_path}:{authority_path}:ro"
                )
        if role in {"pilot", "sim"}:
            self._apply_compute(service)
        return service

    def _coturn_service(self) -> dict[str, object]:
        secret = self.state.turn.secret_path
        if secret is None:
            raise ValueError("managed Coturn requires a TURN secret file")
        command = (
            "exec turnserver -n --log-file=stdout --fingerprint "
            "--use-auth-secret --no-cli --no-multicast-peers --no-tls --no-dtls "
            "--min-port=49160 --max-port=49200 "
            '--realm="$$TURN_REALM" --external-ip="$$TURN_PUBLIC_IP" '
            '--static-auth-secret="$$(cat /run/secrets/turn.secret)"'
        )
        return {
            "image": "coturn/coturn:4.14.0-r0-alpine",
            "container_name": "elesim-coturn",
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "network_mode": "host",
            "restart": "unless-stopped",
            "logging": {
                "driver": DOCKER_LOGGING["driver"],
                "options": dict(DOCKER_LOGGING["options"]),
            },
            "entrypoint": ("/bin/sh", "-ec"),
            # Compose's scalar command form is shell-split. Pass one explicit
            # script argument so `sh -ec` receives the complete command.
            "command": (command,),
            "environment": {
                "TURN_REALM": self.state.turn.realm,
                "TURN_PUBLIC_IP": self.state.turn.public_host,
            },
            "volumes": (f"{secret}:/run/secrets/turn.secret:ro",),
            "tmpfs": ("/var/lib/coturn",),
            "depends_on": ("sim",),
        }

    def _apply_compute(self, service: dict[str, object]) -> None:
        environment = service["environment"]
        assert isinstance(environment, dict)
        if self.state.compute.gpu_mode == "cpu":
            environment["CUDA_VISIBLE_DEVICES"] = ""
            return
        environment["NVIDIA_DRIVER_CAPABILITIES"] = "compute,utility,graphics"
        if self.state.compute.gpu_mode == "specific":
            service["deploy"] = {
                "resources": {
                    "reservations": {
                        "devices": (
                            {
                                "driver": "nvidia",
                                "device_ids": (self.state.compute.gpu_device,),
                                "capabilities": ("gpu",),
                            },
                        )
                    }
                }
            }
        else:
            service["gpus"] = "all"
            # Compose null means "forward it when set, otherwise leave it unset".
            environment["CUDA_VISIBLE_DEVICES"] = None

    def _tools_service(self) -> dict[str, object]:
        context = self.container_root / "build/tools"
        dds_config = (
            role_directory(self.state, self.state.roles[0]) / "config" / "cyclonedds.xml"
        )
        volumes = [
            f"{self.state.prefix_path}:{self.state.prefix_path}:rw",
            f"{dds_config}:/opt/elesim/config/cyclonedds.xml:ro",
        ]
        if not _is_within(self.state.source_path, self.state.prefix_path):
            volumes.append(f"{self.state.source_path}:{self.state.source_path}:ro")
        if not _is_within(self.state_path, self.state.prefix_path):
            volumes.append(f"{self.state_path.parent}:{self.state_path.parent}:rw")
        keystore = self.state.dds.keystore_path
        if keystore is not None and not _is_within(keystore, self.state.prefix_path):
            volumes.append(f"{keystore}:{keystore}:ro")
        # The doctor reuses an installed role identity. Managed host bundles
        # deliberately contain no shared doctor/super-user enclave.
        environment = self._dds_environment(
            self.state.roles[0],
            enclave_override=True,
        )
        environment["HOME"] = "/tmp"
        environment["ELESIM_OPERATOR_HOME"] = str(operator_home())
        return {
            "image": "elesim/tools:local",
            "build": {
                "context": str(context),
                "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            },
            "network_mode": "host",
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "profiles": ("tools",),
            "user": f"{os.getuid()}:{os.getgid()}",
            "environment": environment,
            "volumes": volumes,
        }

    def _manager_service(self) -> dict[str, object]:
        context = self.container_root / "build/tools"
        home = operator_home()
        volumes = [
            f"{home}:{home}:ro",
            f"{self.state.prefix_path}:{self.state.prefix_path}:rw",
            "/var/run/docker.sock:/var/run/docker.sock:rw",
        ]
        if not _is_within(self.state_path, self.state.prefix_path):
            volumes.append(f"{self.state_path.parent}:{self.state_path.parent}:rw")
        if not _is_within(self.state.bin_path, home) and not _is_within(
            self.state.bin_path, self.state.prefix_path
        ):
            volumes.append(f"{self.state.bin_path}:{self.state.bin_path}:ro")
        return {
            "image": "elesim/tools:local",
            "build": {
                "context": str(context),
                "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            },
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "profiles": ("manager",),
            "user": f"{os.getuid()}:{os.getgid()}",
            "group_add": ("${ELESIM_DOCKER_GID:-0}",),
            "environment": {
                "HOME": str(home),
                "ELESIM_OPERATOR_HOME": str(home),
                "PYTHONUNBUFFERED": "1",
            },
            "volumes": volumes,
        }

    def _write_wrappers(self) -> None:
        compose = self.container_root / "compose.yaml"
        command = f"docker compose -f {shlex.quote(str(compose))}"
        guard = _compose_owner_guard(
            compose,
            project=GENERAL_COMPOSE_PROJECT,
            containers=(
                *ROLE_CONTAINER_NAMES.values(),
                *LEGACY_ROLE_CONTAINER_NAMES,
                "elesim-coturn",
                "elesim-manager",
            ),
        )
        application_guard = launch_guard(provisioning_required_path(self.state))
        wrappers: dict[str, tuple[str, bool]] = {
            "elesim-build": (f"{command} build", False),
            "elesim-up": (
                f"{command} up -d --build --remove-orphans",
                True,
            ),
            "elesim-setup": (
                f"{command} run --rm --build tools elesim-setup "
                f"--state {shlex.quote(str(self.state_path))}",
                False,
            ),
        }
        for role in self.state.roles:
            wrappers[f"elesim-{role}"] = (
                f"{command} up --remove-orphans {shlex.quote(role)}",
                True,
            )
        for name, (body, requires_provisioning) in wrappers.items():
            write_executable(
                self.state.bin_path / name,
                "#!/usr/bin/env bash\nset -euo pipefail\n"
                + (application_guard if requires_provisioning else "")
                + guard
                + "exec "
                + body
                + ' "$@"\n',
            )
        # ``elesim-net show`` is consumed as a machine-readable JSON document
        # by the connection manager.  Compose's ``run --build`` writes build
        # progress to stdout before the tool starts, which corrupts that
        # contract.  Build quietly as a separate command, then leave the
        # one-off container's stdout exclusively to ``elesim-net``.
        write_executable(
            self.state.bin_path / "elesim-net",
            "#!/usr/bin/env bash\nset -euo pipefail\n"
            + guard
            + f"{command} build --quiet tools >/dev/null\n"
            + f"exec {command} run --rm -T tools elesim-net "
            + f"--state {shlex.quote(str(self.state_path))}"
            + ' "$@"\n',
        )
        managed_services = (*self.state.roles,)
        if self.state.turn.managed:
            managed_services = (*managed_services, "coturn")
        write_executable(
            self.state.bin_path / "elesim-logs",
            _runtime_logs_wrapper(
                compose=compose,
                logs_root=self.state.prefix_path / "logs",
                services=managed_services,
                archive_enabled=self.state.runtime_text_logs.enabled,
                guard=guard,
            ),
        )
        write_executable(
            self.state.bin_path / "elesim-down",
            _runtime_down_wrapper(
                compose=compose,
                logs_root=self.state.prefix_path / "logs",
                services=managed_services,
                archive_enabled=self.state.runtime_text_logs.enabled,
                guard=guard,
            ),
        )
        write_executable(
            self.state.bin_path / "elesim-connections",
            _manager_wrapper(
                compose=compose,
                state_path=self.state.prefix_path / "connections/topology.json",
                authority_root=self.state.prefix_path / "authority",
                local_install_root=self.state.prefix_path,
                local_bin_dir=self.state.bin_path,
                install_uuid=self._install_uuid,
                guard=guard,
            ),
        )

    def _wrapper_paths(self, *, include_uninstaller: bool = False) -> tuple[Path, ...]:
        names = [
            "elesim-build",
            "elesim-up",
            "elesim-down",
            "elesim-logs",
            "elesim-setup",
            "elesim-net",
            "elesim-connections",
            *(f"elesim-{role}" for role in self.state.roles),
        ]
        if include_uninstaller:
            names.append("elesim-uninstall")
        return tuple(self.state.bin_path / name for name in names)

    def _dds_environment(
        self,
        role: str,
        *,
        enclave_override: bool = False,
    ) -> dict[str, object]:
        environment: dict[str, object] = {
            "PYTHONUNBUFFERED": "1",
            "ELESIM_SYSTEM_ID": self.state.dds.system_id,
            "ELESIM_DDS_DISCOVERY_MODE": self.state.dds.discovery_mode,
            "ELESIM_DDS_STATIC_PEERS": ",".join(self.state.dds.static_peers),
            "ELESIM_DDS_NETWORK_INTERFACE": self.state.dds.interface,
            "ELESIM_DDS_SECURITY_PROFILE": self.state.dds.security_profile,
            "ELESIM_DDS_VENDOR_CONFIG": "/opt/elesim/config/cyclonedds.xml",
            "ROS_DOMAIN_ID": str(self.state.dds.domain_id),
            "RMW_IMPLEMENTATION": self.state.dds.rmw_implementation,
            "ROS_LOCALHOST_ONLY": "0",
            "CYCLONEDDS_URI": "file:///opt/elesim/config/cyclonedds.xml",
        }
        if self.state.dds.security_profile == "sros2":
            environment.update(
                {
                    "ROS_SECURITY_ENABLE": "true",
                    "ROS_SECURITY_STRATEGY": "Enforce",
                    "ROS_SECURITY_KEYSTORE": str(
                        role_keystore_path(self.state, role)
                    ),
                    "ELESIM_DDS_ENCLAVE": dds_enclave(self.state, role),
                }
            )
            if enclave_override:
                environment["ROS_SECURITY_ENCLAVE_OVERRIDE"] = dds_enclave(
                    self.state,
                    role,
                )
        else:
            environment["ROS_SECURITY_ENABLE"] = "false"
        return environment

    def _prepare_turn_secret(self) -> None:
        if self.state.turn.mode == "external":
            if "sim" not in self.state.roles:
                return
            credentials = self.state.turn.credential_path
            if credentials is None:
                raise ValueError(
                    "external TURN on Sim requires a credential file"
                )
            credentials.chmod(0o600)
            return
        if not self.state.turn.managed:
            return
        secret = self.state.turn.secret_path
        if secret is None:
            raise ValueError("managed Coturn requires a TURN secret file")
        if secret.exists():
            if secret.is_symlink() or not secret.is_file():
                raise ValueError(f"TURN secret path is not a regular file: {secret}")
            secret.chmod(0o600)
            return
        secret.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=secret.parent,
            prefix=f".{secret.name}.",
            delete=False,
        ) as handle:
            handle.write(secrets.token_urlsafe(48) + "\n")
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        os.replace(temporary, secret)

    def _validate_external_turn_credentials(self) -> None:
        if (
            self.state.turn.mode != "external"
            or "sim" not in self.state.roles
        ):
            return
        credentials = self.state.turn.credential_path
        if credentials is None:
            raise ValueError(
                "external TURN on Sim requires a credential file"
            )
        validate_external_turn_credentials(
            credentials,
            urls=self.state.network.turn_urls,
        )


def _entrypoint(role: str) -> str:
    commands = {
        "pilot": (
            "elesim-pilot --config /opt/elesim/config/config.pc.yaml "
            "--runtime-config /opt/elesim/config/runtime.installed.yaml"
        ),
        "ui": "elesim-ui --config /opt/elesim/config/installed.yaml",
        "sim": (
            "elesim-sim --config /opt/elesim/config/app.installed.yaml "
            "--runtime-config /opt/elesim/config/runtime.installed.yaml "
            "--model-bundle /opt/elesim/model/bundles/default"
        ),
    }
    try:
        command = commands[role]
    except KeyError as exc:
        raise ValueError(f"unsupported container role: {role}") from exc
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "set +u\n"
        "source /opt/ros/humble/setup.bash\n"
        "source /opt/elesim/ros/install/setup.bash\n"
        "set -u\n"
        "exec "
        + command
        + ' "$@"\n'
    )


def _copy_source_tree(source: Path, destination: Path, *, ignore_config: bool = False) -> None:
    ignored = {"__pycache__", ".pytest_cache", "build", "dist"}
    source_root = source.resolve()

    def ignore(directory: str, names: list[str]) -> set[str]:
        matches = {
            name
            for name in names
            if name in ignored or name.endswith((".pyc", ".egg-info"))
        }
        # Deployment config is mounted separately at runtime.  Only omit that
        # top-level directory; packages such as src/elesim_sim/config are
        # application code and must remain in the install context.
        if ignore_config and Path(directory).resolve() == source_root:
            matches.add("config")
        return matches

    shutil.copytree(source, destination, ignore=ignore)


def _copy_tree(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination, dirs_exist_ok=True)


def _compose_owner_guard(
    compose: Path,
    *,
    project: str,
    containers: tuple[str, ...],
) -> str:
    rendered_containers = " ".join(shlex.quote(name) for name in containers)
    return (
        f"expected_compose={shlex.quote(str(compose))}\n"
        f"expected_project={shlex.quote(project)}\n"
        f"for container in {rendered_containers}; do\n"
        "  if ! docker container inspect \"$container\" >/dev/null 2>&1; then\n"
        "    continue\n"
        "  fi\n"
        "  metadata=\"$(docker container inspect --format "
        "'{{ index .Config.Labels \"com.docker.compose.project\" }}|"
        "{{ index .Config.Labels \"com.docker.compose.project.config_files\" }}' "
        "\"$container\")\"\n"
        "  actual_project=\"${metadata%%|*}\"\n"
        "  actual_compose=\"${metadata#*|}\"\n"
        "  if [[ \"$actual_project\" != \"$expected_project\" || "
        "\"$actual_compose\" != \"$expected_compose\" ]]; then\n"
        "    printf 'Elesim 고정 컨테이너 이름 충돌: %s\\n' \"$container\" >&2\n"
        "    printf '  기존 소유자: project=%s compose=%s\\n' "
        "\"$actual_project\" \"$actual_compose\" >&2\n"
        "    printf '  현재 설치: project=%s compose=%s\\n' "
        "\"$expected_project\" \"$expected_compose\" >&2\n"
        "    printf '기존 설치의 elesim-down으로 종료·제거한 뒤 다시 실행하십시오.\\n' >&2\n"
        "    exit 73\n"
        "  fi\n"
        "done\n"
    )


def _manager_wrapper(
    *,
    compose: Path,
    state_path: Path,
    authority_root: Path,
    local_install_root: Path,
    local_bin_dir: Path,
    install_uuid: str,
    guard: str,
) -> str:
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        + guard
        + "if [[ ! -S /var/run/docker.sock ]]; then\n"
        "  printf 'Docker socket을 찾을 수 없습니다: /var/run/docker.sock\\n' >&2\n"
        "  exit 2\n"
        "fi\n"
        "export ELESIM_DOCKER_GID=\"$(stat -c %g /var/run/docker.sock)\"\n"
        + manager_lifecycle_fragment(install_uuid)
        + "manager_port=8766\n"
        "manager_args=(\"$@\")\n"
        "for ((manager_index=0; manager_index<${#manager_args[@]}; manager_index++)); do\n"
        "  case \"${manager_args[$manager_index]}\" in\n"
        "    --port=*) manager_port=\"${manager_args[$manager_index]#--port=}\" ;;\n"
        "    --port)\n"
        "      if (( manager_index + 1 >= ${#manager_args[@]} )); then\n"
        "        printf '연결관리자 --port 값이 없습니다.\\n' >&2\n"
        "        exit 2\n"
        "      fi\n"
        "      manager_index=$((manager_index + 1))\n"
        "      manager_port=\"${manager_args[$manager_index]}\"\n"
        "      ;;\n"
        "  esac\n"
        "done\n"
        "if [[ ! $manager_port =~ ^[0-9]+$ || $manager_port -lt 1 || $manager_port -gt 65535 ]]; then\n"
        "  printf '연결관리자 port가 유효하지 않습니다: %s\\n' \"$manager_port\" >&2\n"
        "  exit 2\n"
        "fi\n"
        "manager_args+=(--host 0.0.0.0)\n"
        "tailscale_address=\"$(ip -4 -o addr show dev tailscale0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)\"\n"
        "manager_options=()\n"
        "manager_options+=( -e ELESIM_CONNECTION_PUBLISHED=1 )\n"
        "manager_options+=( -e \"ELESIM_TAILSCALE_ADDRESS=$tailscale_address\" )\n"
        "tailscale_bin=\"$(command -v tailscale 2>/dev/null || true)\"\n"
        "tailscale_socket=\"\"\n"
        "for candidate in /var/run/tailscale/tailscaled.sock /run/tailscale/tailscaled.sock; do\n"
        "  if [[ -S $candidate ]]; then tailscale_socket=$candidate; break; fi\n"
        "done\n"
        "if [[ -n $tailscale_bin && -n $tailscale_socket ]]; then\n"
        "  tailscale_gid=\"$(stat -c %g \"$tailscale_socket\" 2>/dev/null || true)\"\n"
        "  if [[ $tailscale_gid =~ ^[0-9]+$ ]]; then manager_options+=(--group-add \"$tailscale_gid\"); fi\n"
        "  manager_options+=(\n"
        "    -e ELESIM_TAILSCALE_PROXY=1\n"
        "    -e ELESIM_TAILSCALE_PROXY_BIN=/usr/local/bin/elesim-tailscale\n"
        "    -e ELESIM_TAILSCALE_PROXY_SOCKET=/var/run/tailscale/tailscaled.sock\n"
        "    -v \"$tailscale_bin:/usr/local/bin/elesim-tailscale:ro\"\n"
        "    -v \"$tailscale_socket:/var/run/tailscale/tailscaled.sock:rw\"\n"
        "  )\n"
        "fi\n"
        "if [[ -n ${SSH_AUTH_SOCK:-} && -S $SSH_AUTH_SOCK ]]; then\n"
        "  manager_options+=(\n"
        "    -e \"SSH_AUTH_SOCK=$SSH_AUTH_SOCK\"\n"
        "    -v \"$SSH_AUTH_SOCK:$SSH_AUTH_SOCK\"\n"
        "  )\n"
        "fi\n"
        "manager_started=1\n"
        "set +e\n"
        "docker compose -f "
        + shlex.quote(str(compose))
        + " run --rm --build --name elesim-manager --publish "
        + '"127.0.0.1:${manager_port}:${manager_port}" '
        + '"${manager_options[@]}" manager elesim-connections --state '
        + shlex.quote(str(state_path))
        + " --authority-root "
        + shlex.quote(str(authority_root))
        + " --local-install-root "
        + shlex.quote(str(local_install_root))
        + " --local-bin-dir "
        + shlex.quote(str(local_bin_dir))
        + ' "${manager_args[@]}"\n'
        + "manager_status=$?\n"
        + "exit \"$manager_status\"\n"
    )


def _runtime_archive_function(
    *,
    compose: Path,
    logs_root: Path,
    services: tuple[str, ...],
) -> str:
    rendered_services = " ".join(shlex.quote(service) for service in services)
    return (
        "archive_path_has_no_symlink_ancestor() {\n"
        "  local target=$1\n"
        "  local probe=$target\n"
        "  local resolved\n"
        "  while [[ ! -e \"$probe\" && ! -L \"$probe\" ]]; do\n"
        "    [[ $probe == / ]] && break\n"
        "    probe=${probe%/*}\n"
        "    [[ -n $probe ]] || probe=/\n"
        "  done\n"
        "  [[ ! -L \"$probe\" ]] || return 1\n"
        "  resolved=\"$(realpath -e -- \"$probe\")\" || return 1\n"
        "  [[ $resolved == \"$probe\" ]]\n"
        "}\n"
        "archive_runtime_logs() {\n"
        f"  local logs_root={shlex.quote(str(logs_root))}\n"
        '  local runs_root="$logs_root/runs"\n'
        '  if ! archive_path_has_no_symlink_ancestor "$logs_root"; then\n'
        "    printf '로그 archive 경로에 symlink가 포함될 수 없습니다: %s\\n' "
        '"$logs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  if ! mkdir -p -- "$runs_root"; then\n'
        "    printf '로그 archive 디렉터리를 만들 수 없습니다: %s\\n' "
        '"$runs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  if ! archive_path_has_no_symlink_ancestor "$logs_root" || '
        '! archive_path_has_no_symlink_ancestor "$runs_root" || '
        '[[ ! -d "$logs_root" || ! -d "$runs_root" ]]; then\n'
        "    printf '로그 archive 경로가 안전한 디렉터리가 아닙니다: %s\\n' "
        '"$runs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  if ! chmod 0700 -- "$logs_root" "$runs_root"; then\n'
        "    printf '로그 archive 디렉터리 권한을 설정할 수 없습니다: %s\\n' "
        '"$runs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        "  local timestamp\n"
        "  if ! timestamp=\"$(date -u +%Y%m%dT%H%M%S.%NZ)\"; then\n"
        "    printf 'UTC 로그 archive timestamp를 만들 수 없습니다.\\n' >&2\n"
        "    return 74\n"
        "  fi\n"
        '  local run_dir="$runs_root/$timestamp"\n'
        '  if [[ -e "$run_dir" || -L "$run_dir" ]] || '
        '! mkdir -- "$run_dir"; then\n'
        "    printf '고유한 로그 archive 디렉터리를 만들 수 없습니다: %s\\n' "
        '"$run_dir" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  if ! chmod 0700 -- "$run_dir"; then\n'
        "    printf '로그 archive 실행 디렉터리 권한을 설정할 수 없습니다: %s\\n' "
        '"$run_dir" >&2\n'
        "    return 74\n"
        "  fi\n"
        "  local archive_status=0\n"
        "  local service destination\n"
        f"  for service in {rendered_services}; do\n"
        '    destination="$run_dir/$service.log"\n'
        "    if ! docker compose -f "
        + shlex.quote(str(compose))
        + ' logs --no-color --timestamps "$service" '
        + '>"$destination" 2>&1; then\n'
        "      printf '서비스 로그 저장 실패: %s (상세: %s)\\n' "
        '"$service" "$destination" >&2\n'
        "      archive_status=74\n"
        "    fi\n"
        '    if ! chmod 0600 -- "$destination"; then\n'
        "      printf '로그 파일 권한 설정 실패: %s\\n' \"$destination\" >&2\n"
        "      archive_status=74\n"
        "    fi\n"
        "  done\n"
        "  local -a generations=()\n"
        "  local candidate name\n"
        "  shopt -s nullglob\n"
        '  for candidate in "$runs_root"/*; do\n'
        '    [[ -d "$candidate" && ! -L "$candidate" ]] || continue\n'
        '    name="${candidate##*/}"\n'
        "    [[ $name =~ ^[0-9]{8}T[0-9]{6}\\.[0-9]{9}Z$ ]] || continue\n"
        '    generations+=("$candidate")\n'
        "  done\n"
        f"  if (( ${{#generations[@]}} > {RUNTIME_LOG_RETENTION} )); then\n"
        "    mapfile -t generations < <(printf '%s\\n' "
        '"${generations[@]}" | LC_ALL=C sort)\n'
        f"    local remove_count=$((${{#generations[@]}} - {RUNTIME_LOG_RETENTION}))\n"
        "    local index\n"
        "    for ((index = 0; index < remove_count; index++)); do\n"
        '      candidate="${generations[index]}"\n'
        '      if [[ -L "$candidate" || ! -d "$candidate" || '
        '"$candidate" != "$runs_root/"* ]]; then\n'
        "        printf '안전하지 않은 archive 삭제 대상을 건너뜁니다: %s\\n' "
        '"$candidate" >&2\n'
        "        archive_status=74\n"
        "        continue\n"
        "      fi\n"
        '      if ! rm -rf -- "$candidate"; then\n'
        "        printf '오래된 로그 archive 삭제 실패: %s\\n' \"$candidate\" >&2\n"
        "        archive_status=74\n"
        "      fi\n"
        "    done\n"
        "  fi\n"
        "  printf '로그 archive: %s\\n' \"$run_dir\"\n"
        '  return "$archive_status"\n'
        "}\n"
    )


def _runtime_logs_wrapper(
    *,
    compose: Path,
    logs_root: Path,
    services: tuple[str, ...],
    archive_enabled: bool,
    guard: str,
) -> str:
    command = "docker compose -f " + shlex.quote(str(compose))
    archive = (
        _runtime_archive_function(
            compose=compose,
            logs_root=logs_root,
            services=services,
        )
        if archive_enabled
        else ""
    )
    save_action = (
        "  archive_runtime_logs\n"
        if archive_enabled
        else (
            "  printf '이 설치에서는 runtime text log archive가 비활성화되어 "
            "있습니다.\\n' >&2\n"
            "  exit 64\n"
        )
    )
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "umask 077\n"
        + guard
        + archive
        + "if (( $# == 0 )); then\n"
        + "  exec "
        + command
        + " logs -f\n"
        + "fi\n"
        + "if (( $# == 1 )) && [[ $1 == --save ]]; then\n"
        + save_action
        + "  exit $?\n"
        + "fi\n"
        + "printf '사용법: elesim-logs [--save]\\n' >&2\n"
        + "exit 64\n"
    )


def _runtime_down_wrapper(
    *,
    compose: Path,
    logs_root: Path,
    services: tuple[str, ...],
    archive_enabled: bool,
    guard: str,
) -> str:
    command = "docker compose -f " + shlex.quote(str(compose))
    if not archive_enabled:
        return (
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            + guard
            + "if (( $# != 0 )); then\n"
            + "  printf '사용법: elesim-down\\n' >&2\n"
            + "  exit 64\n"
            + "fi\n"
            + "exec "
            + command
            + " down --remove-orphans\n"
        )
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "umask 077\n"
        + guard
        + "if (( $# != 0 )); then\n"
        + "  printf '사용법: elesim-down\\n' >&2\n"
        + "  exit 64\n"
        + "fi\n"
        + _runtime_archive_function(
            compose=compose,
            logs_root=logs_root,
            services=services,
        )
        + "archive_status=0\n"
        + "archive_runtime_logs || archive_status=$?\n"
        + "down_status=0\n"
        + command
        + " down --remove-orphans || down_status=$?\n"
        + "if (( down_status != 0 )); then\n"
        + "  exit \"$down_status\"\n"
        + "fi\n"
        + "exit \"$archive_status\"\n"
    )


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


__all__ = ["ContainerAction", "ContainerInstaller", "build_container_plan"]
