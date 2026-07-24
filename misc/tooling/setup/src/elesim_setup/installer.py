"""Role-isolated user installation without cross-deployment imports."""

from __future__ import annotations

import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence

from .configuration import dds_enclave, generate_role_configs, role_directory
from .credentials import validate_external_turn_credentials
from .state import InstallState


GO2_MPC_PACKAGE = "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git"


@dataclass(frozen=True)
class InstallAction:
    title: str
    detail: str


def build_install_plan(state: InstallState) -> tuple[InstallAction, ...]:
    state.validate()
    actions = [
        InstallAction("도구", f"설치/진단 전용 venv: {state.prefix_path / 'tools/venv'}"),
    ]
    for role in state.roles:
        actions.append(
            InstallAction(
                role,
                f"독립 venv + config: {role_directory(state, role)}",
            )
        )
    actions.extend(
        (
            InstallAction(
                "DDS",
                (
                    f"domain {state.dds.domain_id}, "
                    f"{state.dds.discovery_mode}, {state.dds.security_profile}"
                ),
            ),
            InstallAction("명령", f"실행 래퍼: {state.bin_path}"),
            InstallAction("상태", f"비밀값을 제외한 설치 상태: {state.state_path}"),
        )
    )
    if {"controller", "simulator"}.intersection(state.roles):
        detail = state.compute.gpu_mode
        if state.compute.gpu_mode == "specific":
            detail += f" ({state.compute.gpu_device})"
        actions.insert(-2, InstallAction("연산", f"GPU 정책: {detail}"))
    return tuple(actions)


class Installer:
    def __init__(
        self,
        state: InstallState,
        *,
        state_path: Path | None = None,
        dry_run: bool = False,
        log: Callable[[str], None] = print,
    ) -> None:
        self.state = state.validate()
        if self.state.install_mode != "native":
            raise ValueError("Installer에는 install_mode=native가 필요합니다")
        self.state_path = self.state.state_path if state_path is None else state_path.expanduser().resolve()
        self.dry_run = bool(dry_run)
        self.log = log

    def run(self) -> None:
        self._validate_source()
        self.state.require_runnable_dds()
        if self.state.turn.mode == "external" and "simulator" in self.state.roles:
            credentials = self.state.turn.credential_path
            if credentials is None:
                raise ValueError(
                    "external TURN on Simulator requires a credential file"
                )
            validate_external_turn_credentials(
                credentials,
                urls=self.state.network.turn_urls,
            )
        self._show_plan()
        if self.dry_run:
            self.log("[DRY-RUN] 파일이나 패키지를 변경하지 않았습니다.")
            return

        self.state.prefix_path.mkdir(parents=True, exist_ok=True)
        self.state.bin_path.mkdir(parents=True, exist_ok=True)
        self._install_ros_interfaces()
        self._install_tools()
        for role in self.state.roles:
            self._install_role(role)
        generate_role_configs(self.state)
        self._write_wrappers()
        state_path = self.state.save(self.state_path)
        self.log(f"[완료] 설치 상태: {state_path}")
        self.log(f"[다음] 연결 점검: {self.state.bin_path / 'elesim-net'} doctor")

    def _show_plan(self) -> None:
        self.log("\n설치 계획")
        for action in build_install_plan(self.state):
            self.log(f"  [{action.title}] {action.detail}")
        self.log("")

    def _validate_source(self) -> None:
        root = self.state.source_path
        required = [
            root / "packages/protocol/pyproject.toml",
            root / "packages/elesim_interfaces/package.xml",
            root / "packages/elesim_interfaces/CMakeLists.txt",
            root / "packages/elesim_interfaces/msg/RgbdFrame.msg",
            root / "misc/tooling/setup/pyproject.toml",
        ]
        required.extend(root / role / "pyproject.toml" for role in self.state.roles)
        if "simulator" in self.state.roles:
            required.append(root / "model/bundles/default/bundle.json")
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"설치 소스가 불완전합니다:\n{rendered}")
        if sys.version_info < (3, 10):
            raise RuntimeError("Elesim 설치에는 Python 3.10 이상이 필요합니다")
        if (
            "simulator" in self.state.roles
            and self.state.install_go2_mpc
            and shutil.which("git") is None
        ):
            raise RuntimeError(
                "Simulator의 go2-convex-mpc dependency 설치에는 git 명령이 필요합니다"
            )

    def _install_tools(self) -> None:
        root = self.state.source_path
        target = self.state.prefix_path / "tools"
        python = self._ensure_venv(target / "venv", system_site_packages=True)
        self.log("[도구] elesim-setup / elesim-net 설치")
        self._pip(python, "install", "--upgrade", "pip", "setuptools>=68", "wheel")
        self._pip(python, "install", "-r", str(root / "misc/tooling/setup/requirements.lock"))
        self._pip(
            python,
            "install",
            "--force-reinstall",
            "--no-deps",
            str(root / "packages/protocol"),
        )
        self._pip(
            python,
            "install",
            "--force-reinstall",
            "--no-deps",
            str(root / "misc/tooling/setup"),
        )
        self._pip(python, "check")

    def _install_role(self, role: str) -> None:
        root = self.state.source_path
        source = root / role
        target = role_directory(self.state, role)
        self.log(f"[{role}] 파일 배치")
        _copy_tree(source / "config", target / "config")
        if role == "simulator":
            _copy_tree(root / "model/bundles/default", target / "model/bundles/default")
        if role == "robot":
            _copy_tree(source / "systemd", target / "systemd")
            if (source / "install.sh").is_file():
                shutil.copy2(source / "install.sh", target / "install.sh")

        python = self._ensure_venv(
            target / "venv",
            system_site_packages=role == "robot",
        )
        self.log(f"[{role}] Python dependency 설치")
        self._pip(python, "install", "--upgrade", "pip", "setuptools>=68", "wheel")
        self._pip(python, "install", "-r", str(source / "requirements.lock"))
        if role == "simulator" and self.state.install_go2_mpc:
            self._pip(python, "install", GO2_MPC_PACKAGE)
        self._pip(
            python,
            "install",
            "--force-reinstall",
            "--no-deps",
            str(root / "packages/protocol"),
        )
        self._pip(
            python,
            "install",
            "--force-reinstall",
            "--no-deps",
            str(source),
        )
        self._pip(python, "check")

    def _install_ros_interfaces(self) -> None:
        root = self.state.source_path
        ros_root = self.state.prefix_path / "ros"
        command = (
            "source /opt/ros/humble/setup.bash && "
            "colcon --log-base "
            f"{shlex.quote(str(ros_root / 'log'))} build "
            "--base-paths "
            f"{shlex.quote(str(root / 'packages/elesim_interfaces'))} "
            "--build-base "
            f"{shlex.quote(str(ros_root / 'build'))} "
            "--install-base "
            f"{shlex.quote(str(ros_root / 'install'))} "
            "--symlink-install"
        )
        self.log("[ros] elesim_interfaces overlay build")
        self._run(("/bin/bash", "-lc", command))

    def _ensure_venv(
        self,
        path: Path,
        *,
        system_site_packages: bool = False,
    ) -> Path:
        python = path / "bin/python"
        if not python.is_file():
            self.log(f"[venv] {path}")
            arguments = [sys.executable, "-m", "venv"]
            if system_site_packages:
                arguments.append("--system-site-packages")
            arguments.append(str(path))
            self._run(tuple(arguments))
        return python

    def _pip(self, python: Path, *arguments: str) -> None:
        self._run(
            (
                str(python),
                "-m",
                "pip",
                "--disable-pip-version-check",
                *arguments,
            )
        )

    def _run(self, command: Sequence[str]) -> None:
        self.log("$ " + shlex.join(str(value) for value in command))
        subprocess.run(tuple(str(value) for value in command), check=True)

    def _write_wrappers(self) -> None:
        tool_venv = self.state.prefix_path / "tools/venv/bin"
        state_path = self.state_path
        _write_executable(
            self.state.bin_path / "elesim-setup",
            _exec_script(
                tool_venv / "elesim-setup",
                ("--state", str(state_path)),
                source_ros=self.state.prefix_path / "ros/install/setup.bash",
                environment=self._dds_environment("doctor"),
            ),
        )
        _write_executable(
            self.state.bin_path / "elesim-net",
            _exec_script(
                tool_venv / "elesim-net",
                ("--state", str(state_path)),
                source_ros=self.state.prefix_path / "ros/install/setup.bash",
                environment=self._dds_environment("doctor"),
            ),
        )
        for role in self.state.roles:
            executable, arguments = self._role_command(role)
            _write_executable(
                self.state.bin_path / f"elesim-{role}",
                _exec_script(
                    executable,
                    arguments,
                    environment={
                        **self._role_environment(role),
                        **self._dds_environment(role),
                    },
                    source_ros=self.state.prefix_path / "ros/install/setup.bash",
                    source_unitree=role == "robot",
                ),
            )

    def _dds_environment(self, role: str) -> Mapping[str, str]:
        config_role = role if role in self.state.roles else self.state.roles[0]
        environment = {
            "ELESIM_SYSTEM_ID": self.state.dds.system_id,
            "ELESIM_DDS_DISCOVERY_MODE": self.state.dds.discovery_mode,
            "ELESIM_DDS_STATIC_PEERS": ",".join(self.state.dds.static_peers),
            "ELESIM_DDS_NETWORK_INTERFACE": self.state.dds.interface,
            "ELESIM_DDS_SECURITY_PROFILE": self.state.dds.security_profile,
            "ROS_DOMAIN_ID": str(self.state.dds.domain_id),
            "RMW_IMPLEMENTATION": self.state.dds.rmw_implementation,
            "ROS_LOCALHOST_ONLY": "0",
            "CYCLONEDDS_URI": (
                "file://"
                + str(role_directory(self.state, config_role) / "config/cyclonedds.xml")
            ),
            "ELESIM_DDS_VENDOR_CONFIG": str(
                role_directory(self.state, config_role) / "config/cyclonedds.xml"
            ),
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

    def _role_environment(self, role: str) -> Mapping[str, str]:
        if role not in {"controller", "simulator"}:
            return {}
        if self.state.compute.gpu_mode == "specific":
            return {"CUDA_VISIBLE_DEVICES": self.state.compute.gpu_device}
        if self.state.compute.gpu_mode == "cpu":
            return {"CUDA_VISIBLE_DEVICES": ""}
        return {}

    def _role_command(self, role: str) -> tuple[Path, tuple[str, ...]]:
        root = role_directory(self.state, role)
        executable = root / f"venv/bin/elesim-{role}"
        config = root / "config"
        if role == "controller":
            return executable, (
                "--config",
                str(config / "config.pc.yaml"),
                "--runtime-config",
                str(config / "runtime.installed.yaml"),
            )
        if role == "ui":
            return executable, ("--config", str(config / "installed.yaml"))
        if role == "simulator":
            return executable, (
                "--config",
                str(config / "app.installed.yaml"),
                "--runtime-config",
                str(config / "runtime.installed.yaml"),
                "--model-bundle",
                str(root / "model/bundles/default"),
            )
        if role == "robot":
            return executable, ("--config", str(config / "installed.yaml"))
        raise ValueError(f"unknown role: {role}")


def preflight_notes(
    roles: Iterable[str],
    *,
    install_mode: str = "native",
) -> tuple[str, ...]:
    selected = set(roles)
    notes: list[str] = []
    if install_mode == "container":
        notes.append("호스트에는 Docker Engine과 Docker Compose plugin만 필요합니다.")
        if {"controller", "simulator"}.intersection(selected):
            notes.append("GPU 모드는 NVIDIA driver와 NVIDIA Container Toolkit이 필요합니다.")
        if "ui" in selected:
            notes.append("UI는 호스트 X11 display socket을 컨테이너에 전달합니다.")
        notes.append("컨테이너 설치는 호스트 APT/Python 환경을 변경하지 않습니다.")
        return tuple(notes)
    if "simulator" in selected:
        notes.append("Simulator는 git, Genesis가 지원하는 GPU driver와 graphics runtime이 별도로 필요합니다.")
    if "ui" in selected:
        notes.append("UI는 OpenGL/GLFW와 데스크톱 display 환경이 필요합니다.")
    if "robot" in selected:
        notes.append("Robot은 ROS2 Humble, unitree_ros2, RealSense와 serial 장치 권한이 필요합니다.")
    notes.append("설치기는 sudo나 방화벽 설정을 자동 실행하지 않습니다.")
    return tuple(notes)


def _copy_tree(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination, dirs_exist_ok=True)


def _exec_script(
    executable: Path,
    arguments: Sequence[str],
    *,
    environment: Mapping[str, str] | None = None,
    source_ros: Path | None = None,
    source_unitree: bool = False,
) -> str:
    command = shlex.join((str(executable), *(str(value) for value in arguments)))
    lines = ["#!/usr/bin/env bash", "set -euo pipefail"]
    if source_ros is not None:
        lines.extend(
            (
                "set +u",
                "source /opt/ros/humble/setup.bash",
            )
        )
        if source_unitree:
            lines.extend(
                (
                    'unitree_ws="${UNITREE_ROS2_WS:-$HOME/ros2_ws}"',
                    'if [[ -f "$unitree_ws/install/setup.bash" ]]; then',
                    '  source "$unitree_ws/install/setup.bash"',
                    "fi",
                )
            )
        lines.append(f"source {shlex.quote(str(source_ros))}")
        lines.append("set -u")
    for name, value in (environment or {}).items():
        lines.append(f"export {name}={shlex.quote(str(value))}")
    lines.append("exec " + command + ' "$@"')
    return "\n".join(lines) + "\n"


def _write_executable(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(0o755)
    temporary.replace(path)


__all__ = [
    "GO2_MPC_PACKAGE",
    "InstallAction",
    "Installer",
    "build_install_plan",
    "preflight_notes",
]
