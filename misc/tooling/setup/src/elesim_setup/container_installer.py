"""Generate a host-preserving Docker Compose installation for Elesim roles."""

from __future__ import annotations

import hashlib
import os
import shlex
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping

import yaml

from .configuration import generate_role_configs, missing_credentials, role_directory
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
    _DEFAULT_BASE_IMAGE = "python:3.10-slim-bookworm"
    _SIMULATOR_BASE_IMAGE = "ros:humble-ros-base-jammy"

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
        missing = missing_credentials(self.state)
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"CURVE credential이 부족합니다:\n{rendered}")
        self.log("\n컨테이너 설치 계획")
        for action in build_container_plan(self.state):
            self.log(f"  [{action.title}] {action.detail}")
        self.log("")
        if self.dry_run:
            self.log("[DRY-RUN] 호스트나 Docker daemon을 변경하지 않았습니다.")
            return

        self.state.prefix_path.mkdir(parents=True, exist_ok=True)
        self.state.bin_path.mkdir(parents=True, exist_ok=True)
        self._copy_runtime_data()
        generate_role_configs(self.state)
        for role in self.state.roles:
            self._write_role_context(role)
        self._write_tools_context()
        self._write_compose()
        self._write_wrappers()
        saved = self.state.save(self.state_path)
        self.log(f"[완료] 설치 상태: {saved}")
        self.log(f"[다음] 이미지 빌드 및 시작: {self.state.bin_path / 'elesim-up'}")

    def _validate_source(self) -> None:
        root = self.state.source_path
        required = [
            root / "packages/protocol/pyproject.toml",
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
        for name in ("requirements.lock", "requirements-media.lock"):
            shutil.copy2(setup / name, context / name)
        _copy_source_tree(root / "packages/protocol", context / "protocol")
        _copy_source_tree(setup, context / "setup")

    def _write_compose(self) -> None:
        services: dict[str, object] = {}
        for role in self.state.roles:
            services[role] = self._role_service(role)
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
                    "BASE_IMAGE": (
                        self._SIMULATOR_BASE_IMAGE
                        if role == "simulator"
                        else self._DEFAULT_BASE_IMAGE
                    ),
                    "COMPUTE_MODE": self.state.compute.gpu_mode,
                    "INSTALL_GO2_MPC": "1" if self.state.install_go2_mpc else "0",
                },
            },
            "network_mode": "host",
            "restart": "unless-stopped" if role != "ui" else "no",
            "environment": {"PYTHONUNBUFFERED": "1"},
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
        credentials = self.state.security.root
        if credentials is not None:
            service["volumes"].append(  # type: ignore[union-attr]
                f"{credentials}:{credentials}:ro"
            )
        if role != "router" and "router" in self.state.roles:
            service["depends_on"] = ("router",)
        return service

    def _apply_compute(self, service: dict[str, object]) -> None:
        environment = service["environment"]
        assert isinstance(environment, dict)
        if self.state.compute.gpu_mode == "cpu":
            environment["CUDA_VISIBLE_DEVICES"] = ""
            return
        service["gpus"] = "all"
        environment["NVIDIA_DRIVER_CAPABILITIES"] = "compute,utility,graphics"
        if self.state.compute.gpu_mode == "specific":
            environment["CUDA_VISIBLE_DEVICES"] = self.state.compute.gpu_device
        else:
            # Compose null means "forward it when set, otherwise leave it unset".
            environment["CUDA_VISIBLE_DEVICES"] = None

    def _tools_service(self) -> dict[str, object]:
        context = self.container_root / "build/tools"
        volumes = [f"{self.state.prefix_path}:{self.state.prefix_path}:rw"]
        if not _is_within(self.state.source_path, self.state.prefix_path):
            volumes.append(f"{self.state.source_path}:{self.state.source_path}:ro")
        if not _is_within(self.state_path, self.state.prefix_path):
            volumes.append(f"{self.state_path.parent}:{self.state_path.parent}:rw")
        credentials = self.state.security.root
        if credentials is not None and not _is_within(credentials, self.state.prefix_path):
            volumes.append(f"{credentials}:{credentials}:ro")
        return {
            "image": f"elesim-managed/{self.installation_id}-tools:local",
            "build": {"context": str(context)},
            "network_mode": "host",
            "profiles": ("tools",),
            "user": f"{os.getuid()}:{os.getgid()}",
            "environment": {"HOME": "/tmp"},
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


def _entrypoint(role: str) -> str:
    commands = {
        "router": "elesim-router --config /opt/elesim/config/installed.yaml",
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
    return "#!/usr/bin/env bash\nset -euo pipefail\nexec " + command + ' "$@"\n'


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
