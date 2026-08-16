"""Generate the single-container EleSim coding environment."""

from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import tempfile
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Mapping

import yaml

from .capabilities import HostCapabilities
from .configuration import write_cyclonedds_config
from .manager_lifecycle import (
    compose_owner_guard,
    host_helper_fragment,
    manager_lifecycle_fragment,
)
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
from .request import SetupRequest
from .shell import operator_home, write_executable
from .updater import render_update_wrapper


Log = Callable[[str], None]
_REQUIRED_PROJECTS = (
    ("packages/protocol", "pyproject.toml"),
    ("packages/elesim_interfaces", "package.xml"),
    ("packages/elesim_interfaces", "CMakeLists.txt"),
    ("pilot", "pyproject.toml"),
    ("ui", "pyproject.toml"),
    ("sim", "pyproject.toml"),
    ("robot", "pyproject.toml"),
)
DEVELOPER_COMPOSE_PROJECT = "elesim-runtime-dev"
_DEVELOPER_USERNAME = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")


def _resolve_developer_username() -> str:
    """Resolve a safe account label for the persistent developer container.

    The setup GUI runs in a disposable image which deliberately does not copy
    the host password database.  A container process can therefore have a
    numeric UID without a passwd entry, and libraries such as ``getpass``
    report ``No username found`` when their environment is also empty.  Keep
    the host-provided name when it is a valid Linux account label and use the
    image's deterministic ``dev`` account otherwise.
    """

    for variable in ("ELESIM_HOST_USER", "USER", "LOGNAME"):
        candidate = os.environ.get(variable, "").strip()
        if _DEVELOPER_USERNAME.fullmatch(candidate):
            return candidate
    return "dev"


@dataclass(frozen=True)
class DeveloperInstallState:
    schema_version: int
    workspace: str
    bin_dir: str
    repository: str
    ref: str
    gpu_mode: str
    gpu_device: str
    jaeger: bool
    dds: dict[str, object]


def _is_valid_developer_uninstall_tombstone(
    payload: object,
    *,
    install_uuid: str,
    workspace: Path,
) -> bool:
    if not isinstance(payload, Mapping):
        return False
    preserved_paths = payload.get("preserved_paths")
    if (
        type(payload.get("schema_version")) is not int
        or payload.get("schema_version") != 1
        or payload.get("install_uuid") != install_uuid
        or payload.get("edition") != "developer"
        or payload.get("prefix") != str(workspace)
        or type(payload.get("purged_logs")) is not bool
        or type(payload.get("purged_authority")) is not bool
        or not isinstance(preserved_paths, list)
        or any(
            not isinstance(value, str) or not Path(value).is_absolute()
            for value in preserved_paths
        )
    ):
        return False
    completed_at = payload.get("completed_at")
    if not isinstance(completed_at, str):
        return False
    try:
        completed = datetime.fromisoformat(completed_at)
    except ValueError:
        return False
    return completed.tzinfo is not None and completed.utcoffset() is not None


class DeveloperInstaller:
    def __init__(
        self,
        request: SetupRequest,
        *,
        capabilities: HostCapabilities | None = None,
        shell_bashrc: Path | None = None,
        dry_run: bool = False,
        log: Log = print,
    ) -> None:
        if request.edition != "developer":
            raise ValueError("DeveloperInstaller requires edition=developer")
        self.request = request
        self.capabilities = capabilities
        self.dry_run = bool(dry_run)
        self.log = log
        self.shell_bashrc = (
            None
            if shell_bashrc is None
            else Path(os.path.abspath(os.fspath(shell_bashrc.expanduser())))
        )
        self._install_uuid = ""
        self._build_root = self.generated_root / "build"

    @property
    def workspace(self) -> Path:
        return self.request.prefix

    @property
    def generated_root(self) -> Path:
        return self.workspace / ".elesim/development"

    @property
    def ownership_manifest_path(self) -> Path:
        return self.generated_root / "install-ownership.json"

    def run(self) -> None:
        ownership_refresh = prepare_ownership_refresh(
            prefix=self.workspace,
            bin_dir=self.request.bin_dir,
            edition="developer",
            manifest_path=self.ownership_manifest_path,
            claimed_paths=self._claimed_paths(),
        )
        self._install_uuid = ownership_install_uuid(ownership_refresh)
        prefix_created = not os.path.lexists(self.workspace)
        bin_created = not os.path.lexists(self.request.bin_dir)
        self.log("\n개발자 환경 생성 계획")
        self.log(f"  [workspace] {self.workspace}")
        self.log("  [container] privileged ROS2/Genesis full development image")
        self.log(f"  [jaeger] {'included' if self.request.jaeger else 'disabled'}")
        self.log("  [host] Python/APT/CUDA state is not modified")
        if self.dry_run:
            self.log("[DRY-RUN] workspace를 변경하지 않았습니다.")
            return
        self.log("[1/5] workspace 준비")
        self._prepare_workspace()
        self._build_root = self._prepare_build_root()
        self.log("[2/5] 개발 image context 생성")
        self._write_context()
        write_cyclonedds_config(
            self.generated_root / "cyclonedds.xml",
            self.request.dds,
        )
        self.log("[3/5] Compose 구성 생성")
        self._write_compose()
        self.log("[4/5] 실행 명령 생성")
        self._write_wrappers()
        self.log("[5/5] 설치 상태와 제거 소유권 저장")
        self._write_state()
        bundle = install_host_uninstaller_bundle(
            prefix=self.workspace,
            bin_dir=self.request.bin_dir,
            manifest_path=self.ownership_manifest_path,
            bundle_root=self.generated_root / "maintenance",
        )
        manifest = self._write_ownership_manifest(
            bundle=bundle,
            refresh=ownership_refresh,
            prefix_created=prefix_created,
            bin_created=bin_created,
        )
        self.log(f"[완료] 개발 Compose: {self.generated_root / 'compose.yaml'}")
        self.log(f"[완료] 제거 소유권: {manifest.path}")
        self.log(f"[다음] {self.request.bin_dir / 'elesim-up'}")

    def _write_ownership_manifest(
        self,
        *,
        bundle: HostUninstallerBundle,
        refresh: OwnershipRefresh | None,
        prefix_created: bool,
        bin_created: bool,
    ) -> OwnershipManifest:
        external_paths: list[Path] = []
        external_paths.extend(self._existing_uninstall_tombstones())
        if (
            self.request.dds.security_profile == "sros2"
            and self.request.dds.keystore_path is not None
        ):
            external_paths.append(self.request.dds.keystore_path)
        containers = ["elesim-dev", "elesim-manager"]
        if self.request.jaeger:
            containers.append("elesim-jaeger")
        created_roots = tuple(
            path
            for path, created in (
                (self.workspace, prefix_created),
                (self.request.bin_dir, bin_created),
            )
            if created
        )
        return write_ownership_manifest(
            prefix=self.workspace,
            bin_dir=self.request.bin_dir,
            edition="developer",
            inventory_roots=(bundle.root,),
            managed_roots=(
                self.generated_root,
                self.workspace / ".elesim/connections",
            ),
            created_roots=created_roots,
            wrapper_paths=self._wrapper_paths(include_uninstaller=True),
            authority_roots=(self.workspace / ".elesim/authority",),
            external_paths=external_paths,
            shell_bashrc=self.shell_bashrc,
            docker=DockerOwnership(
                install_uuid=self._install_uuid,
                compose_file=str(self.generated_root / "compose.yaml"),
                project=DEVELOPER_COMPOSE_PROJECT,
                containers=tuple(containers),
                local_images=("elesim/dev:local",),
            ),
            manifest_path=self.ownership_manifest_path,
            install_uuid=self._install_uuid,
            refresh=refresh,
        )

    def _claimed_paths(self) -> tuple[Path, ...]:
        claims = [
            self.workspace / ".elesim/connections",
            *self._wrapper_paths(include_uninstaller=True),
        ]
        root = self.generated_root
        if os.path.lexists(root):
            if root.is_symlink() or not root.is_dir():
                claims.append(root)
            else:
                preserved = set(self._existing_uninstall_tombstones())
                claims.extend(path for path in root.iterdir() if path not in preserved)
        return tuple(claims)

    def _existing_uninstall_tombstones(self) -> tuple[Path, ...]:
        root = self.generated_root
        if not root.is_dir() or root.is_symlink():
            return ()
        accepted: list[Path] = []
        for path in root.iterdir():
            if path.is_symlink() or not path.is_file():
                continue
            prefix = "uninstall-tombstone-"
            if not path.name.startswith(prefix) or not path.name.endswith(".json"):
                continue
            identifier = path.name[len(prefix) : -len(".json")]
            try:
                parsed_uuid = str(uuid.UUID(identifier))
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError, json.JSONDecodeError):
                continue
            if (
                parsed_uuid == identifier
                and _is_valid_developer_uninstall_tombstone(
                    payload,
                    install_uuid=identifier,
                    workspace=self.workspace,
                )
            ):
                accepted.append(path)
        return tuple(sorted(accepted))

    def _prepare_workspace(self) -> None:
        workspace = self.workspace
        if workspace.exists() and any(workspace.iterdir()):
            if not (workspace / ".git").is_dir() or not _valid_workspace(workspace):
                raise ValueError(
                    f"비어 있지 않은 경로는 기존 EleSim Git workspace여야 합니다: {workspace}"
                )
            self.log("[workspace] existing checkout reused without pull/reset")
            return
        workspace.parent.mkdir(parents=True, exist_ok=True)
        workspace.mkdir(exist_ok=True)
        staging = workspace / ".elesim-clone-staging"
        if staging.exists():
            shutil.rmtree(staging)
        self.log(
            f"[workspace] clone https://github.com/{self.request.repository}.git "
            f"ref={self.request.ref}"
        )
        from dulwich import porcelain

        try:
            porcelain.clone(
                f"https://github.com/{self.request.repository}.git",
                str(staging),
                checkout=True,
                branch=self.request.ref.encode("utf-8"),
            )
            if not _valid_workspace(staging):
                raise RuntimeError("cloned repository is not a complete EleSim workspace")
            for child in tuple(staging.iterdir()):
                child.replace(workspace / child.name)
            staging.rmdir()
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        if not _valid_workspace(workspace):
            raise RuntimeError("cloned repository could not be installed into the workspace")

    def _prepare_build_root(self) -> Path:
        """Choose a writable developer build context without following links."""

        legacy = self.generated_root / "build"
        fallback = self.generated_root / ".runtime-build"
        for candidate in (legacy, fallback):
            if candidate.is_symlink():
                raise ValueError(f"Developer image context는 symlink일 수 없습니다: {candidate}")
        candidates = (fallback, legacy) if fallback.exists() else (legacy, fallback)
        for index, candidate in enumerate(candidates):
            try:
                candidate.mkdir(mode=0o700, parents=True, exist_ok=True)
                candidate.chmod(0o700)
                if not os.access(candidate, os.W_OK | os.X_OK):
                    raise PermissionError(candidate)
                if index:
                    self.log(
                        "[build] 기존 Developer context에 쓸 수 없어 새 context를 사용합니다: "
                        f"{candidate}"
                    )
                return candidate
            except (PermissionError, FileExistsError, NotADirectoryError) as exc:
                if index == len(candidates) - 1:
                    raise PermissionError(
                        "Developer image context를 준비할 수 없습니다. "
                        f"경로의 권한/유형을 확인하십시오: {candidate}"
                    ) from exc
        raise RuntimeError("Developer image context 후보가 없습니다")

    def _write_context(self) -> None:
        source = self.request.source_root / "environment/development"
        required = (
            source / "Dockerfile",
            source / "requirements.lock",
            source / "entrypoint.sh",
            source / "dev-env.sh",
            self.request.source_root / "environment/containers/robotpkg.asc",
        )
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"개발 이미지 입력이 부족합니다:\n{rendered}")
        context = self._build_root
        _reset_developer_context(context)
        context.mkdir(parents=True)
        for name in (
            "Dockerfile",
            "requirements.lock",
            "entrypoint.sh",
            "dev-env.sh",
        ):
            shutil.copy2(source / name, context / name)
        shutil.copy2(
            self.request.source_root / "environment/containers/robotpkg.asc",
            context / "robotpkg.asc",
        )

    def _write_compose(self) -> None:
        home = self.generated_root / "home"
        cache = self.generated_root / "cache"
        context = self._build_root
        username = _resolve_developer_username()
        home.mkdir(parents=True, exist_ok=True)
        cache.mkdir(parents=True, exist_ok=True)
        environment: dict[str, object] = {
            "HOME": str(home),
            # Keep username discovery deterministic inside the numeric-UID
            # developer container.  The setup image has no host /etc/passwd,
            # so relying on getpass/pwd alone can yield "No username found".
            "USER": username,
            "LOGNAME": username,
            "ELESIM_HOST_USER": username,
            "ELESIM_WORKSPACE": str(self.workspace),
            "DISPLAY": "${DISPLAY:-:0}",
            "WAYLAND_DISPLAY": "${WAYLAND_DISPLAY:-}",
            "XDG_RUNTIME_DIR": "${XDG_RUNTIME_DIR:-}",
            "PYTHONUNBUFFERED": "1",
            "PYTHONNOUSERSITE": "1",
            "ELESIM_SYSTEM_ID": self.request.dds.system_id,
            "ELESIM_DDS_DISCOVERY_MODE": self.request.dds.discovery_mode,
            "ELESIM_DDS_STATIC_PEERS": ",".join(self.request.dds.static_peers),
            "ELESIM_DDS_NETWORK_INTERFACE": self.request.dds.interface,
            "ELESIM_DDS_SECURITY_PROFILE": self.request.dds.security_profile,
            "ELESIM_DDS_VENDOR_CONFIG": "/opt/elesim/config/cyclonedds.xml",
            "ROS_DOMAIN_ID": str(self.request.dds.domain_id),
            "RMW_IMPLEMENTATION": self.request.dds.rmw_implementation,
            "ROS_LOCALHOST_ONLY": "0",
            "CYCLONEDDS_URI": "file:///opt/elesim/config/cyclonedds.xml",
        }
        if self.request.dds.security_profile == "sros2":
            environment.update(
                {
                    "ROS_SECURITY_ENABLE": "true",
                    "ROS_SECURITY_STRATEGY": "Enforce",
                    "ROS_SECURITY_KEYSTORE": self.request.dds.keystore,
                    "ELESIM_DDS_ENCLAVE_BASE": self.request.dds.enclave,
                }
            )
        else:
            environment["ROS_SECURITY_ENABLE"] = "false"
        if self.request.jaeger:
            environment.update(
                {
                    "ELESIM_TRACE": "1",
                    "ELESIM_OTEL_ENDPOINT": "http://127.0.0.1:4318",
                    "ELESIM_OTEL_PROTOCOL": "http/protobuf",
                }
            )
        service: dict[str, object] = {
            "image": "elesim/dev:local",
            "container_name": "elesim-dev",
            "build": {
                "context": str(context),
                "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
                "args": {
                    "USERNAME": username,
                    "UID": str(os.getuid()),
                    "GID": str(os.getgid()),
                    "COMPUTE_MODE": self.request.compute.gpu_mode,
                },
            },
            "privileged": True,
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "network_mode": "host",
            "ipc": "host",
            "shm_size": "4gb",
            "working_dir": str(self.workspace),
            "environment": environment,
            "volumes": [
                f"{self.workspace}:{self.workspace}:rw",
                f"{home}:{home}:rw",
                f"{cache}:{cache}:rw",
                "/dev:/dev",
                "/tmp/.X11-unix:/tmp/.X11-unix:rw",
                (
                    f"{self.generated_root / 'cyclonedds.xml'}:"
                    "/opt/elesim/config/cyclonedds.xml:ro"
                ),
            ],
            "command": ("sleep", "infinity"),
            "restart": "unless-stopped",
        }
        if self.request.compute.gpu_mode == "cpu":
            environment["CUDA_VISIBLE_DEVICES"] = ""
        else:
            environment["NVIDIA_DRIVER_CAPABILITIES"] = "compute,utility,graphics"
            if self.request.compute.gpu_mode == "specific":
                service["deploy"] = {
                    "resources": {
                        "reservations": {
                            "devices": (
                                {
                                    "driver": "nvidia",
                                    "device_ids": (
                                        self.request.compute.gpu_device,
                                    ),
                                    "capabilities": ("gpu",),
                                },
                            )
                        }
                    }
                }
            else:
                service["gpus"] = "all"
                environment["CUDA_VISIBLE_DEVICES"] = None
        keystore = self.request.dds.keystore_path
        if keystore is not None:
            service["volumes"].append(f"{keystore}:{keystore}:ro")  # type: ignore[union-attr]
        if self.capabilities is not None and self.capabilities.wslg_available:
            service["volumes"].append("/mnt/wslg:/mnt/wslg:rw")  # type: ignore[union-attr]
            environment["WAYLAND_DISPLAY"] = "${WAYLAND_DISPLAY:-wayland-0}"
            environment["XDG_RUNTIME_DIR"] = (
                "${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
            )
            environment["PULSE_SERVER"] = (
                "${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
            )

        services: dict[str, object] = {"dev": service}
        services["manager"] = {
            "image": "elesim/dev:local",
            "build": service["build"],
            "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
            "profiles": ("manager",),
            "working_dir": str(self.workspace),
            "environment": {
                "HOME": str(home),
                "USER": username,
                "LOGNAME": username,
                "ELESIM_HOST_USER": username,
                "ELESIM_OPERATOR_HOME": str(operator_home()),
                "ELESIM_WORKSPACE": str(self.workspace),
                "DOCKER_CONFIG": "/tmp/elesim-docker-config",
                "PYTHONUNBUFFERED": "1",
                "PYTHONNOUSERSITE": "1",
            },
            "volumes": [
                f"{self.workspace}:{self.workspace}:rw",
                f"{home}:{home}:rw",
                f"{cache}:{cache}:rw",
                f"{operator_home()}:{operator_home()}:ro",
            ],
        }
        if self.request.jaeger:
            services["jaeger"] = {
                "image": "jaegertracing/jaeger:2.19.0",
                "container_name": "elesim-jaeger",
                "labels": {DOCKER_INSTALL_UUID_LABEL: self._install_uuid},
                "network_mode": "host",
                "profiles": ("observability",),
                "restart": "unless-stopped",
            }
        payload = {"name": DEVELOPER_COMPOSE_PROJECT, "services": services}
        destination = self.generated_root / "compose.yaml"
        destination.parent.mkdir(parents=True, exist_ok=True)
        if os.path.lexists(destination) and destination.is_symlink():
            raise ValueError(f"Developer Compose manifest는 symlink일 수 없습니다: {destination}")
        rendered = yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)
        with tempfile.NamedTemporaryFile(
            "w", encoding="utf-8", dir=destination.parent,
            prefix=f".{destination.name}.", delete=False,
        ) as handle:
            handle.write(rendered)
            temporary = Path(handle.name)
        try:
            os.replace(temporary, destination)
        finally:
            if temporary.exists():
                temporary.unlink()

    def _write_wrappers(self) -> None:
        compose = self.generated_root / "compose.yaml"
        command = f"docker compose -f {shlex.quote(str(compose))}"
        guard = compose_owner_guard(
            compose,
            project=DEVELOPER_COMPOSE_PROJECT,
            containers=("elesim-dev", "elesim-jaeger", "elesim-manager"),
        )
        wrappers: dict[str, str] = {
            "elesim-build": f"{command} build dev",
            "elesim-up": f"{command} up -d --build --remove-orphans dev",
            "elesim-logs": f"{command} --profile observability logs -f",
        }
        if self.request.jaeger:
            wrappers.update(
                {
                    "elesim-jaeger-up": (
                        f"{command} --profile observability up -d "
                        "--remove-orphans jaeger"
                    ),
                    "elesim-jaeger-down": (
                        f"{command} --profile observability stop jaeger"
                    ),
                }
            )
        for name, body in wrappers.items():
            write_executable(
                self.request.bin_dir / name,
                "#!/usr/bin/env bash\nset -euo pipefail\n"
                + guard
                + "exec "
                + body
                + ' "$@"\n',
            )
        write_executable(
            self.request.bin_dir / "elesim-down",
            _developer_down_wrapper(compose=compose, guard=guard),
        )
        write_executable(
            self.request.bin_dir / "elesim-dev",
            (
                "#!/usr/bin/env bash\n"
                "set -euo pipefail\n"
                + guard
                + f"{command} up -d --build --remove-orphans dev\n"
                "if [[ $# -eq 0 ]]; then\n"
                "  set -- bash\n"
                "fi\n"
                f"exec {command} exec dev /usr/local/bin/elesim-dev-env \"$@\"\n"
            ),
        )
        write_executable(
            self.request.bin_dir / "elesim-connections",
            _development_manager_wrapper(
                compose=compose,
                state_path=self.workspace / ".elesim/connections/topology.json",
                authority_root=self.workspace / ".elesim/authority",
                default_local_install_root=Path(
                    "~/.local/share/elesim"
                ).expanduser(),
                default_local_bin_dir=Path("~/.local/bin").expanduser(),
                operator_home=operator_home(),
                maintenance_root=self.generated_root / "maintenance",
                install_uuid=self._install_uuid,
                guard=guard,
                gpu_mode=self.request.compute.gpu_mode,
                gpu_device=self.request.compute.gpu_device,
            ),
        )
        write_executable(
            self.request.bin_dir / "elesim-update",
            render_update_wrapper(
                edition="developer",
                prefix=self.workspace,
                state_path=self.generated_root / "install-state.json",
                compose=compose,
                build_services=("dev",),
                preamble=guard,
                repository=self.request.repository,
                ref=self.request.ref,
                runtime_uid=os.getuid(),
            ),
        )

    def _wrapper_paths(self, *, include_uninstaller: bool = False) -> tuple[Path, ...]:
        names = [
            "elesim-build",
            "elesim-up",
            "elesim-down",
            "elesim-logs",
            "elesim-dev",
            "elesim-connections",
            "elesim-update",
        ]
        if self.request.jaeger:
            names.extend(("elesim-jaeger-up", "elesim-jaeger-down"))
        if include_uninstaller:
            names.append("elesim-uninstall")
        return tuple(self.request.bin_dir / name for name in names)

    def _write_state(self) -> None:
        state = DeveloperInstallState(
            schema_version=2,
            workspace=str(self.workspace),
            bin_dir=str(self.request.bin_dir),
            repository=self.request.repository,
            ref=self.request.ref,
            gpu_mode=self.request.compute.gpu_mode,
            gpu_device=self.request.compute.gpu_device,
            jaeger=self.request.jaeger,
            dds={
                "system_id": self.request.dds.system_id,
                "domain_id": self.request.dds.domain_id,
                "rmw_implementation": self.request.dds.rmw_implementation,
                "discovery_mode": self.request.dds.discovery_mode,
                "static_peers": list(self.request.dds.static_peers),
                "interface": self.request.dds.interface,
                "security_profile": self.request.dds.security_profile,
                "keystore": self.request.dds.keystore,
                "enclave": self.request.dds.enclave,
            },
        )
        destination = self.generated_root / "install-state.json"
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(asdict(state), ensure_ascii=False, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=".install-state.",
            delete=False,
        ) as handle:
            handle.write(payload)
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        temporary.replace(destination)


def _valid_workspace(workspace: Path) -> bool:
    return all(
        (workspace / project / marker).is_file()
        for project, marker in _REQUIRED_PROJECTS
    )


def _reset_developer_context(context: Path) -> None:
    if not os.path.lexists(context):
        return
    if context.is_symlink():
        raise ValueError(f"Developer image context는 symlink일 수 없습니다: {context}")
    if not context.is_dir():
        raise ValueError(f"Developer image context는 directory여야 합니다: {context}")
    try:
        shutil.rmtree(context)
    except PermissionError as exc:
        raise PermissionError(
            "기존 Developer image context를 교체할 권한이 없습니다: "
            f"{context}"
        ) from exc


def _developer_down_wrapper(*, compose: Path, guard: str) -> str:
    """Render the developer shutdown wrapper with optional manager purge."""

    command = "docker compose -f " + shlex.quote(str(compose))
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        + guard
        + "purge_requested=0\n"
        "if (( $# > 0 )) && [[ $1 == --purge ]]; then\n"
        "  purge_requested=1\n"
        "  shift\n"
        "fi\n"
        "if (( $# != 0 )); then\n"
        "  printf '사용법: elesim-down [--purge]\\n' >&2\n"
        "  exit 64\n"
        "fi\n"
        "down_status=0\n"
        "set +e\n"
        + command
        + " --profile observability down --remove-orphans\n"
        "down_status=$?\n"
        "set -e\n"
        "purge_status=0\n"
        "if (( purge_requested )) && docker container inspect elesim-manager >/dev/null 2>&1; then\n"
        "  docker rm -f elesim-manager >/dev/null || purge_status=$?\n"
        "fi\n"
        "if (( down_status != 0 )); then\n"
        "  exit \"$down_status\"\n"
        "fi\n"
        "if (( purge_status != 0 )); then\n"
        "  exit \"$purge_status\"\n"
        "fi\n"
    )


def _development_manager_wrapper(
    *,
    compose: Path,
    state_path: Path,
    authority_root: Path,
    default_local_install_root: Path,
    default_local_bin_dir: Path,
    operator_home: Path,
    maintenance_root: Path,
    install_uuid: str,
    guard: str,
    gpu_mode: str,
    gpu_device: str,
) -> str:
    if gpu_mode not in {"inherit", "specific", "cpu"}:
        raise ValueError(f"unsupported GPU mode: {gpu_mode!r}")
    if gpu_mode == "specific" and not gpu_device:
        raise ValueError("specific GPU mode requires a GPU device")
    if gpu_mode != "specific" and gpu_device:
        raise ValueError("GPU device is only valid for specific GPU mode")
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        + guard
        + "local_install_root=${ELESIM_LOCAL_INSTALL_ROOT:-"
        + shlex.quote(str(default_local_install_root))
        + "}\n"
        "local_bin_dir=${ELESIM_LOCAL_BIN_DIR:-"
        + shlex.quote(str(default_local_bin_dir))
        + "}\n"
        "if [[ $local_install_root != /* || ! -d $local_install_root ]]; then\n"
        "  printf '로컬 EleSim install root가 없거나 절대경로가 아닙니다: %s\\n' "
        "\"$local_install_root\" >&2\n"
        "  printf 'ELESIM_LOCAL_INSTALL_ROOT로 일반 설치 prefix를 지정하십시오.\\n' >&2\n"
        "  exit 2\n"
        "fi\n"
        "if [[ $local_bin_dir != /* || ! -d $local_bin_dir ]]; then\n"
        "  printf '로컬 EleSim bin dir가 없거나 절대경로가 아닙니다: %s\\n' "
        "\"$local_bin_dir\" >&2\n"
        "  printf 'ELESIM_LOCAL_BIN_DIR로 일반 설치 명령 디렉터리를 지정하십시오.\\n' >&2\n"
        "  exit 2\n"
        "fi\n"
        + manager_lifecycle_fragment(install_uuid)
        + "manager_compose_args=(-f "
        + shlex.quote(str(compose))
        + ")\n"
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
        "tailscale_interface=\"$(ip -o link show 2>/dev/null | awk -F': ' '$2 ~ /^tailscale[0-9]+$/ {print $2; exit}')\"\n"
        "tailscale_address=\"\"\n"
        "if [[ -n $tailscale_interface ]]; then\n"
        "  tailscale_address=\"$(ip -4 -o addr show dev \"$tailscale_interface\" 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)\"\n"
        "fi\n"
        "manager_options=(\n"
        "  -e ELESIM_CONNECTION_PUBLISHED=1\n"
        "  -e ELESIM_INSTALL_GPU_MODE="
        + shlex.quote(gpu_mode)
        + "\n"
        "  -e ELESIM_INSTALL_GPU_DEVICE="
        + shlex.quote(gpu_device)
        + "\n"
        "  -e \"ELESIM_TAILSCALE_ADDRESS=$tailscale_address\"\n"
        "  -e \"ELESIM_TAILSCALE_INTERFACE=$tailscale_interface\"\n"
        "  -v \"$local_install_root:$local_install_root:rw\"\n"
        "  -v \"$local_bin_dir:$local_bin_dir:ro\"\n"
        "  -e "
        + shlex.quote(f"ELESIM_OPERATOR_HOME={operator_home.resolve()}")
        + "\n"
        ")\n"
        + host_helper_fragment(
            maintenance_root=maintenance_root,
            compose_argument='"$local_install_root/containers/compose.yaml"',
            bin_dir_argument='"$local_bin_dir"',
            project="elesim-runtime",
        )
        + "if [[ -n ${SSH_AUTH_SOCK:-} && -S $SSH_AUTH_SOCK ]]; then\n"
        "  manager_options+=(\n"
        "    -e \"SSH_AUTH_SOCK=$SSH_AUTH_SOCK\"\n"
        "    -v \"$SSH_AUTH_SOCK:$SSH_AUTH_SOCK\"\n"
        "  )\n"
        "fi\n"
        "manager_started=1\n"
        "set +e\n"
        "docker compose \"${manager_compose_args[@]}\" run --rm --build --name elesim-manager --publish "
        + '"127.0.0.1:${manager_port}:${manager_port}" '
        + '"${manager_options[@]}" manager elesim-connections --state '
        + shlex.quote(str(state_path))
        + " --authority-root "
        + shlex.quote(str(authority_root))
        + " --local-install-root \"$local_install_root\""
        + " --local-bin-dir \"$local_bin_dir\""
        + ' "${manager_args[@]}"\n'
        + "manager_status=$?\n"
        + "exit \"$manager_status\"\n"
    )


__all__ = ["DeveloperInstallState", "DeveloperInstaller"]
