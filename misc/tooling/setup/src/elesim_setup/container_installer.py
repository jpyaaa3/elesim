"""Generate a host-preserving Docker Compose installation for Elesim roles."""

from __future__ import annotations

import hashlib
import os
import secrets
import shlex
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping

import yaml

from .configuration import (
    dds_enclave,
    generate_role_configs,
    role_directory,
)
from .credentials import validate_external_turn_credentials
from .state import InstallState


@dataclass(frozen=True)
class ContainerAction:
    title: str
    detail: str


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

    @property
    def container_root(self) -> Path:
        return self.state.prefix_path / "containers"

    @property
    def installation_id(self) -> str:
        digest = hashlib.sha256(str(self.state.prefix_path).encode("utf-8")).hexdigest()
        return digest[:10]

    def run(self) -> None:
        self._validate_source()
        self.state.require_runnable_dds()
        self._validate_external_turn_credentials()
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
        self.log("[6/6] 설치 상태 저장")
        saved = self.state.save(self.state_path)
        self.log(f"[완료] 설치 상태: {saved}")
        self.log(f"[다음] 이미지 빌드 및 시작: {self.state.bin_path / 'elesim-up'}")

    def _validate_source(self) -> None:
        root = self.state.source_path
        required = [
            root / "packages/protocol/pyproject.toml",
            root / "packages/elesim_interfaces/package.xml",
            root / "packages/elesim_interfaces/CMakeLists.txt",
            root / "packages/elesim_interfaces/msg/RgbdFrame.msg",
            root / "misc/tooling/setup/pyproject.toml",
            root / "misc/infra/containers/Dockerfile.app",
            root / "misc/infra/containers/Dockerfile.tools",
            root / "misc/infra/containers/robotpkg.asc",
        ]
        required.extend(root / role / "pyproject.toml" for role in self.state.roles)
        if "simulator" in self.state.roles:
            required.append(root / "model/bundles/default/bundle.json")
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"컨테이너 설치 소스가 불완전합니다:\n{rendered}")

    def _copy_runtime_data(self) -> None:
        root = self.state.source_path
        for role in self.state.roles:
            target = role_directory(self.state, role)
            _copy_tree(root / role / "config", target / "config")
            if role == "simulator":
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
        shutil.copy2(root / "misc/infra/containers/Dockerfile.app", context / "Dockerfile")
        shutil.copy2(root / "misc/infra/containers/robotpkg.asc", context / "robotpkg.asc")
        shutil.copy2(root / role / "requirements.lock", context / "requirements.lock")
        _copy_source_tree(root / "packages/protocol", context / "protocol")
        _copy_source_tree(
            root / "packages/elesim_interfaces",
            context / "interfaces/elesim_interfaces",
        )
        _copy_source_tree(root / role, context / "application", ignore_config=True)
        (context / "entrypoint").write_text(_entrypoint(role), encoding="utf-8")
        (context / "entrypoint").chmod(0o755)

    def _write_tools_context(self) -> None:
        root = self.state.source_path
        context = self.container_root / "build/tools"
        if context.exists():
            shutil.rmtree(context)
        context.mkdir(parents=True)
        shutil.copy2(root / "misc/infra/containers/Dockerfile.tools", context / "Dockerfile")
        setup = root / "misc/tooling/setup"
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
        payload = {"name": f"elesim-{self.installation_id}", "services": services}
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
            "image": f"elesim-managed/{self.installation_id}-{role}:local",
            "build": {
                "context": str(context),
                "args": {
                    "ROLE": role,
                    "BASE_IMAGE": self._ROS_BASE_IMAGE,
                    "COMPUTE_MODE": self.state.compute.gpu_mode,
                    "INSTALL_GO2_MPC": "1" if self.state.install_go2_mpc else "0",
                },
            },
            "network_mode": "host",
            "restart": "unless-stopped" if role != "ui" else "no",
            "environment": self._dds_environment(role),
            "volumes": [f"{role_root / 'config'}:/opt/elesim/config:ro"],
        }
        if role == "simulator":
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
                        "external TURN on Simulator requires a credential file"
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
        if role in {"controller", "simulator"}:
            self._apply_compute(service)
        keystore = self.state.dds.keystore_path
        if keystore is not None:
            service["volumes"].append(  # type: ignore[union-attr]
                f"{keystore}:{keystore}:ro"
            )
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
        environment = self._dds_environment("doctor")
        environment["HOME"] = "/tmp"
        return {
            "image": "coturn/coturn:4.14.0-r0-alpine",
            "network_mode": "host",
            "restart": "unless-stopped",
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
            "depends_on": ("simulator",),
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
        environment = self._dds_environment("doctor")
        return {
            "image": f"elesim-managed/{self.installation_id}-tools:local",
            "build": {"context": str(context)},
            "network_mode": "host",
            "profiles": ("tools",),
            "user": f"{os.getuid()}:{os.getgid()}",
            "environment": environment,
            "volumes": volumes,
        }

    def _write_wrappers(self) -> None:
        compose = self.container_root / "compose.yaml"
        command = f"docker compose -f {shlex.quote(str(compose))}"
        wrappers: Mapping[str, str] = {
            "elesim-build": f"{command} build",
            "elesim-up": f"{command} up -d --build",
            "elesim-down": f"{command} down",
            "elesim-logs": f"{command} logs -f",
            "elesim-setup": (
                f"{command} run --rm --build tools elesim-setup "
                f"--state {shlex.quote(str(self.state_path))}"
            ),
            "elesim-net": (
                f"{command} run --rm --build tools elesim-net "
                f"--state {shlex.quote(str(self.state_path))}"
            ),
        }
        for role in self.state.roles:
            wrappers[f"elesim-{role}"] = f"{command} up {shlex.quote(role)}"
        for name, body in wrappers.items():
            _write_executable(
                self.state.bin_path / name,
                "#!/usr/bin/env bash\nset -euo pipefail\nexec " + body + ' "$@"\n',
            )

    def _dds_environment(self, role: str) -> dict[str, object]:
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
                    "ROS_SECURITY_KEYSTORE": self.state.dds.keystore,
                    "ROS_SECURITY_ENCLAVE_OVERRIDE": dds_enclave(
                        self.state,
                        role,
                    ),
                    "ELESIM_DDS_ENCLAVE": dds_enclave(self.state, role),
                }
            )
        else:
            environment["ROS_SECURITY_ENABLE"] = "false"
        return environment

    def _prepare_turn_secret(self) -> None:
        if self.state.turn.mode == "external":
            if "simulator" not in self.state.roles:
                return
            credentials = self.state.turn.credential_path
            if credentials is None:
                raise ValueError(
                    "external TURN on Simulator requires a credential file"
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
            or "simulator" not in self.state.roles
        ):
            return
        credentials = self.state.turn.credential_path
        if credentials is None:
            raise ValueError(
                "external TURN on Simulator requires a credential file"
            )
        validate_external_turn_credentials(
            credentials,
            urls=self.state.network.turn_urls,
        )


def _entrypoint(role: str) -> str:
    commands = {
        "controller": (
            "elesim-controller --config /opt/elesim/config/config.pc.yaml "
            "--runtime-config /opt/elesim/config/runtime.installed.yaml"
        ),
        "ui": "elesim-ui --config /opt/elesim/config/installed.yaml",
        "simulator": (
            "elesim-simulator --config /opt/elesim/config/app.installed.yaml "
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
        "source /opt/ros/humble/setup.bash\n"
        "source /opt/elesim/ros/install/setup.bash\n"
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
        # top-level directory; packages such as src/elesim_simulator/config are
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


def _write_executable(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(0o755)
    temporary.replace(path)


def _is_within(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


__all__ = ["ContainerAction", "ContainerInstaller", "build_container_plan"]
