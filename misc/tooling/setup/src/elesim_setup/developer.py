"""Generate the single-container Elesim coding environment."""

from __future__ import annotations

import hashlib
import json
import os
import shlex
import shutil
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Mapping

import yaml

from .capabilities import HostCapabilities
from .configuration import write_cyclonedds_config
from .request import SetupRequest


Log = Callable[[str], None]
_REQUIRED_PROJECTS = (
    ("packages/protocol", "pyproject.toml"),
    ("packages/elesim_interfaces", "package.xml"),
    ("packages/elesim_interfaces", "CMakeLists.txt"),
    ("controller", "pyproject.toml"),
    ("ui", "pyproject.toml"),
    ("simulator", "pyproject.toml"),
    ("robot", "pyproject.toml"),
)


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


class DeveloperInstaller:
    def __init__(
        self,
        request: SetupRequest,
        *,
        capabilities: HostCapabilities | None = None,
        dry_run: bool = False,
        log: Log = print,
    ) -> None:
        if request.edition != "developer":
            raise ValueError("DeveloperInstaller requires edition=developer")
        self.request = request
        self.capabilities = capabilities
        self.dry_run = bool(dry_run)
        self.log = log

    @property
    def workspace(self) -> Path:
        return self.request.prefix

    @property
    def generated_root(self) -> Path:
        return self.workspace / ".elesim/development"

    @property
    def installation_id(self) -> str:
        digest = hashlib.sha256(str(self.workspace).encode("utf-8")).hexdigest()
        return digest[:10]

    def run(self) -> None:
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
        self.log("[5/5] 설치 상태 저장")
        self._write_state()
        self.log(f"[완료] 개발 Compose: {self.generated_root / 'compose.yaml'}")
        self.log(f"[다음] {self.request.bin_dir / 'elesim-up'}")

    def _prepare_workspace(self) -> None:
        workspace = self.workspace
        if workspace.exists() and any(workspace.iterdir()):
            if not (workspace / ".git").is_dir() or not _valid_workspace(workspace):
                raise ValueError(
                    f"비어 있지 않은 경로는 기존 Elesim Git workspace여야 합니다: {workspace}"
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
                raise RuntimeError("cloned repository is not a complete Elesim workspace")
            for child in tuple(staging.iterdir()):
                child.replace(workspace / child.name)
            staging.rmdir()
        except Exception:
            shutil.rmtree(staging, ignore_errors=True)
            raise
        if not _valid_workspace(workspace):
            raise RuntimeError("cloned repository could not be installed into the workspace")

    def _write_context(self) -> None:
        source = self.request.source_root / "misc/infra/development"
        required = (
            source / "Dockerfile",
            source / "requirements.lock",
            source / "entrypoint.sh",
            self.request.source_root / "misc/infra/containers/robotpkg.asc",
        )
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"개발 이미지 입력이 부족합니다:\n{rendered}")
        context = self.generated_root / "build"
        if context.exists():
            shutil.rmtree(context)
        context.mkdir(parents=True)
        for name in ("Dockerfile", "requirements.lock", "entrypoint.sh"):
            shutil.copy2(source / name, context / name)
        shutil.copy2(
            self.request.source_root / "misc/infra/containers/robotpkg.asc",
            context / "robotpkg.asc",
        )

    def _write_compose(self) -> None:
        home = self.generated_root / "home"
        cache = self.generated_root / "cache"
        context = self.generated_root / "build"
        home.mkdir(parents=True, exist_ok=True)
        cache.mkdir(parents=True, exist_ok=True)
        environment: dict[str, object] = {
            "HOME": str(home),
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
            "image": f"elesim-managed/{self.installation_id}-dev:local",
            "build": {
                "context": str(context),
                "args": {
                    "USERNAME": (
                        os.environ.get("ELESIM_HOST_USER", "").strip()
                        or os.environ.get("USER", "").strip()
                        or "dev"
                    ),
                    "UID": str(os.getuid()),
                    "GID": str(os.getgid()),
                },
            },
            "privileged": True,
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
        if self.request.jaeger:
            services["jaeger"] = {
                "image": "jaegertracing/jaeger:2.19.0",
                "network_mode": "host",
                "profiles": ("observability",),
                "restart": "unless-stopped",
            }
        payload = {"name": f"elesim-dev-{self.installation_id}", "services": services}
        destination = self.generated_root / "compose.yaml"
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    def _write_wrappers(self) -> None:
        compose = self.generated_root / "compose.yaml"
        command = f"docker compose -f {shlex.quote(str(compose))}"
        wrappers: dict[str, str] = {
            "elesim-build": f"{command} build dev",
            "elesim-up": f"{command} up -d --build dev",
            "elesim-down": f"{command} --profile observability down",
            "elesim-logs": f"{command} --profile observability logs -f",
            "elesim-dev": f"{command} run --rm --build dev bash",
        }
        if self.request.jaeger:
            wrappers.update(
                {
                    "elesim-jaeger-up": (
                        f"{command} --profile observability up -d jaeger"
                    ),
                    "elesim-jaeger-down": (
                        f"{command} --profile observability stop jaeger"
                    ),
                }
            )
        for name, body in wrappers.items():
            _write_executable(
                self.request.bin_dir / name,
                "#!/usr/bin/env bash\nset -euo pipefail\nexec " + body + ' "$@"\n',
            )

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


def _write_executable(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(0o755)
    temporary.replace(path)


__all__ = ["DeveloperInstallState", "DeveloperInstaller"]
