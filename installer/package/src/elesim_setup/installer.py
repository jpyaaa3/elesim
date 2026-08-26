"""Role-isolated user installation without cross-deployment imports."""

from __future__ import annotations

import os
import pwd
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Mapping, Sequence

from .configuration import (
    RobotHostSettings,
    copy_role_config_tree,
    dds_enclave,
    generate_role_configs,
    role_keystore_path,
    role_directory,
)
from .credentials import validate_external_turn_credentials
from .ownership import (
    HostUninstallerBundle,
    OwnershipManifest,
    OwnershipRefresh,
    SystemdUnitOwnership,
    install_host_uninstaller_bundle,
    ownership_install_uuid,
    prepare_ownership_refresh,
    sha256_file,
    write_ownership_manifest,
)
from .security_provisioning import (
    launch_guard,
    provisioning_required_path,
    sync_provisioning_required,
)
from .security_views import prepare_role_keystore_views
from .runtime_status import render_native_status_wrapper
from .shell import write_executable
from .state import InstallState
from .updater import render_update_wrapper


GO2_MPC_PACKAGE = "git+https://github.com/elijah-waichong-chan/go2-convex-mpc.git@1c63c6a762779887ab0431fd60db681dede6cb32"
ROBOT_SYSTEMD_UNIT = "elesim-robot.service"
UNITREE_BRIDGE_SYSTEMD_UNIT = "elesim-unitree-bridge.service"
NATIVE_RUNTIME_LOG_RETENTION = 5
NATIVE_RUNTIME_LOG_BYTES = 10 * 1024 * 1024
UNITREE_BRIDGE_USER = "elesim-unitree"


@dataclass(frozen=True)
class InstallAction:
    title: str
    detail: str


@dataclass(frozen=True)
class NativeRobotHost:
    robot_user: str
    robot_home: Path
    bridge_user: str
    unitree_ros_workspace: Path
    unitree_interface: str
    unitree_domain_id: int

    @property
    def config_settings(self) -> RobotHostSettings:
        return RobotHostSettings(
            robot_user=self.robot_user,
            bridge_user=self.bridge_user,
            ros_workspace=self.unitree_ros_workspace,
            unitree_interface=self.unitree_interface,
            unitree_domain_id=self.unitree_domain_id,
        ).validate()


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
    if {"pilot", "sim"}.intersection(state.roles):
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
        shell_bashrc: Path | None = None,
        dry_run: bool = False,
        log: Callable[[str], None] = print,
    ) -> None:
        self.state = state.validate()
        if self.state.install_mode != "native":
            raise ValueError("Installer에는 install_mode=native가 필요합니다")
        self.state_path = self.state.state_path if state_path is None else state_path.expanduser().resolve()
        self.dry_run = bool(dry_run)
        self.log = log
        self._install_uuid = ""
        self.shell_bashrc = (
            None
            if shell_bashrc is None
            else Path(os.path.abspath(os.fspath(shell_bashrc.expanduser())))
        )
        self.robot_host = _resolve_native_robot_host()

    def run(self) -> None:
        self._validate_source()
        self.state.require_installable_dds()
        self._validate_robot_network_boundary()
        self._validate_unitree_workspace()
        if self.state.turn.mode == "external" and "sim" in self.state.roles:
            credentials = self.state.turn.credential_path
            if credentials is None:
                raise ValueError(
                    "external TURN on Sim requires a credential file"
                )
            validate_external_turn_credentials(
                credentials,
                urls=self.state.network.turn_urls,
            )
        ownership_refresh = prepare_ownership_refresh(
            prefix=self.state.prefix_path,
            bin_dir=self.state.bin_path,
            edition="general",
            claimed_paths=self._claimed_paths(),
        )
        self._install_uuid = ownership_install_uuid(ownership_refresh)
        prefix_created = not os.path.lexists(self.state.prefix_path)
        bin_created = not os.path.lexists(self.state.bin_path)
        self._show_plan()
        if self.dry_run:
            self.log("[DRY-RUN] 파일이나 패키지를 변경하지 않았습니다.")
            return

        self.state.prefix_path.mkdir(parents=True, exist_ok=True)
        self.state.bin_path.mkdir(parents=True, exist_ok=True)
        prepare_role_keystore_views(self.state)
        if self.state.dds.managed_security_pending:
            sync_provisioning_required(self.state)
        self._install_ros_interfaces()
        self._install_tools()
        for role in self.state.roles:
            self._install_role(role)
        generate_role_configs(
            self.state,
            robot_host=self.robot_host.config_settings,
        )
        self._write_wrappers()
        robot_services = self._write_robot_service_units()
        state_path = self.state.save(self.state_path)
        if not self.state.dds.managed_security_pending:
            sync_provisioning_required(self.state)
        bundle = install_host_uninstaller_bundle(
            prefix=self.state.prefix_path,
            bin_dir=self.state.bin_path,
        )
        manifest = self._write_ownership_manifest(
            bundle=bundle,
            services=robot_services,
            refresh=ownership_refresh,
            prefix_created=prefix_created,
            bin_created=bin_created,
        )
        self.log(f"[완료] 설치 상태: {state_path}")
        self.log(f"[완료] 제거 소유권: {manifest.path}")
        self.log(f"[다음] 연결 점검: {self.state.bin_path / 'elesim-net'} doctor")
        self._log_robot_service_registration(robot_services)

    def _claimed_paths(self) -> tuple[Path, ...]:
        claims = [
            self.state.prefix_path / "roles",
            self.state.prefix_path / "ros",
            self.state.prefix_path / "tools",
            self.state.prefix_path / "security",
            self.state.prefix_path / "connections",
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
        services: Sequence[Path],
        refresh: OwnershipRefresh | None,
        prefix_created: bool,
        bin_created: bool,
    ) -> OwnershipManifest:
        inventory_roots: list[Path] = [
            self.state.prefix_path / "roles",
            self.state.prefix_path / "ros",
            self.state.prefix_path / "tools",
            bundle.root,
        ]
        external_paths: list[Path] = []
        if _is_within(self.state_path, self.state.prefix_path):
            inventory_roots.append(self.state_path)
        else:
            external_paths.append(self.state_path)
        if (
            self.state.dds.security_profile == "sros2"
            and self.state.dds.security_provisioning == "external"
            and self.state.dds.keystore_path is not None
        ):
            external_paths.append(self.state.dds.keystore_path)
        service_paths = tuple(services)
        if len(service_paths) != 2:
            raise ValueError("native Robot install must own exactly two systemd units")
        units = tuple(
            SystemdUnitOwnership(
                name=service.name,
                destination=f"/etc/systemd/system/{service.name}",
                sha256=sha256_file(service),
            )
            for service in service_paths
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
                self.state.prefix_path / "roles",
                self.state.prefix_path / "ros",
                self.state.prefix_path / "tools",
                self.state.prefix_path / "connections",
                self.state.prefix_path / "security",
                self.state.prefix_path / "secrets",
            ),
            created_roots=created_roots,
            wrapper_paths=self._wrapper_paths(include_uninstaller=True),
            log_roots=(self.state.prefix_path / "logs",),
            authority_roots=(self.state.prefix_path / "authority",),
            external_paths=external_paths,
            shell_bashrc=self.shell_bashrc,
            systemd_units=units,
            install_uuid=self._install_uuid,
            refresh=refresh,
        )

    def _validate_unitree_workspace(self) -> None:
        if self.dry_run:
            return
        setup = self.robot_host.unitree_ros_workspace / "install/setup.bash"
        if not setup.is_file():
            raise FileNotFoundError(
                "Unitree ROS 2 workspace overlay가 없습니다: "
                f"{setup}. 설치 전에 UNITREE_ROS2_WS 또는 "
                "ELESIM_UNITREE_ROS2_WS를 실제 workspace로 지정하십시오."
            )

    def _validate_robot_network_boundary(self) -> None:
        elesim_interface = self.state.dds.interface.strip()
        if not elesim_interface:
            raise ValueError(
                "Robot 설치는 inter-host EleSim DDS interface를 명시해야 합니다"
            )
        if elesim_interface == self.robot_host.unitree_interface:
            raise ValueError(
                "EleSim DDS interface와 private Unitree interface가 같습니다. "
                "ELESIM_UNITREE_INTERFACE 또는 설치 DDS interface를 분리하십시오."
            )
        if self.state.dds.domain_id == self.robot_host.unitree_domain_id:
            raise ValueError(
                "EleSim ROS domain과 private Unitree domain이 같습니다. "
                "ELESIM_UNITREE_DOMAIN_ID 또는 설치 DDS domain을 분리하십시오."
            )

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
            root / "packages/elesim_interfaces/msg/EncodedRgbdFrame.msg",
            root / "installer/package/pyproject.toml",
        ]
        required.extend(
            root / role / "pyproject.toml"
            for role in self.state.roles
        )
        if "sim" in self.state.roles:
            required.append(root / "model/bundles/default/bundle.json")
            required.append(root / "model/bundles/d435/bundle.json")
        missing = [path for path in required if not path.is_file()]
        if missing:
            rendered = "\n".join(f"  - {path}" for path in missing)
            raise FileNotFoundError(f"설치 소스가 불완전합니다:\n{rendered}")
        if sys.version_info < (3, 10):
            raise RuntimeError("EleSim 설치에는 Python 3.10 이상이 필요합니다")
        if (
            "sim" in self.state.roles
            and self.state.install_go2_mpc
            and shutil.which("git") is None
        ):
            raise RuntimeError(
                "Sim의 go2-convex-mpc dependency 설치에는 git 명령이 필요합니다"
            )

    def _install_tools(self) -> None:
        root = self.state.source_path
        target = self.state.prefix_path / "tools"
        python = self._ensure_venv(target / "venv", system_site_packages=True)
        self.log("[도구] elesim-setup / elesim-net 설치")
        self._pip(
            python,
            "install",
            "--upgrade",
            "pip",
            "setuptools>=68,<80",
            "packaging>=24.2,<26",
            "wheel",
        )
        self._pip(python, "install", "-r", str(root / "installer/package/requirements.lock"))
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
            str(root / "installer/package"),
        )
        self._pip(python, "check")

    def _install_role(self, role: str) -> None:
        root = self.state.source_path
        source = root / role
        target = role_directory(self.state, role)
        self.log(f"[{role}] 파일 배치")
        copy_role_config_tree(source / "config", target / "config", role)
        if role == "sim":
            _copy_tree(root / "model/bundles/default", target / "model/bundles/default")
            _copy_tree(root / "model/bundles/d435", target / "model/bundles/d435")
        python = self._ensure_venv(
            target / "venv",
            system_site_packages=role == "robot",
        )
        self.log(f"[{role}] Python dependency 설치")
        self._pip(
            python,
            "install",
            "--upgrade",
            "pip",
            "setuptools>=68,<80",
            "packaging>=24.2,<26",
            "wheel",
        )
        self._pip(python, "install", "-r", str(source / "requirements.lock"))
        if role == "sim" and self.state.install_go2_mpc:
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
        environment = os.environ.copy()
        # colcon is supplied by the host ROS installation, but its Python
        # process also imports packaging/setuptools.  Jetson hosts commonly
        # have an old distro ``packaging`` beside a newer pip setuptools,
        # which breaks rosidl with canonicalize_version(...,
        # strip_trailing_zero=...).  The bootstrap venv carries a compatible
        # pair; prepend only that venv's site-packages and leave the host ROS
        # installation untouched.
        environment["PYTHONNOUSERSITE"] = "1"
        site_packages = (
            Path(sys.prefix)
            / "lib"
            / f"python{sys.version_info.major}.{sys.version_info.minor}"
            / "site-packages"
        )
        if site_packages.is_dir():
            inherited_pythonpath = environment.get("PYTHONPATH", "").strip()
            environment["PYTHONPATH"] = os.pathsep.join(
                value
                for value in (str(site_packages), inherited_pythonpath)
                if value
            )
        self._run(("/bin/bash", "-lc", command), env=environment)

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
        _ensure_python_pip(python)
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

    def _run(
        self,
        command: Sequence[str],
        *,
        env: Mapping[str, str] | None = None,
    ) -> None:
        self.log("$ " + shlex.join(str(value) for value in command))
        subprocess.run(
            tuple(str(value) for value in command),
            check=True,
            env=None if env is None else dict(env),
        )

    def _write_wrappers(self) -> None:
        tool_venv = self.state.prefix_path / "tools/venv/bin"
        state_path = self.state_path
        write_executable(
            self.state.bin_path / "elesim-setup",
            _exec_script(
                tool_venv / "elesim-setup",
                ("--state", str(state_path)),
                source_ros=self.state.prefix_path / "ros/install/setup.bash",
                environment=self._dds_environment(
                    "doctor",
                    enclave_override=True,
                ),
            ),
        )
        write_executable(
            self.state.bin_path / "elesim-net",
            _exec_script(
                tool_venv / "elesim-net",
                ("--state", str(state_path)),
                source_ros=self.state.prefix_path / "ros/install/setup.bash",
                environment=self._dds_environment(
                    "doctor",
                    enclave_override=True,
                ),
            ),
        )
        for role in self.state.roles:
            executable, arguments = self._role_command(role)
            write_executable(
                self.state.bin_path / f"elesim-{role}",
                _exec_script(
                    executable,
                    arguments,
                    environment={
                        **self._role_environment(role),
                        **self._dds_environment(role),
                    },
                    source_ros=self.state.prefix_path / "ros/install/setup.bash",
                    guard=launch_guard(provisioning_required_path(self.state)),
                ),
            )
        if self.state.roles == ("robot",):
            role_root = role_directory(self.state, "robot")
            write_executable(
                self.state.bin_path / "elesim-unitree-bridge",
                _exec_script(
                    role_root / "venv/bin/elesim-unitree-bridge",
                    ("--config", str(role_root / "config/installed.yaml")),
                    source_ros=self.state.prefix_path / "ros/install/setup.bash",
                    guard=launch_guard(provisioning_required_path(self.state)),
                ),
            )
            write_executable(
                self.state.bin_path / "elesim-up",
                _native_systemctl_wrapper("start"),
            )
            write_executable(
                self.state.bin_path / "elesim-logs",
                _native_logs_wrapper(
                    logs_root=self.state.prefix_path / "logs",
                    archive_enabled=self.state.runtime_text_logs.enabled,
                ),
            )
            write_executable(
                self.state.bin_path / "elesim-down",
                _native_down_wrapper(
                    logs_root=self.state.prefix_path / "logs",
                    archive_enabled=self.state.runtime_text_logs.enabled,
                ),
            )
            write_executable(
                self.state.bin_path / "elesim-status",
                render_native_status_wrapper(
                    robot_unit=ROBOT_SYSTEMD_UNIT,
                    bridge_unit=UNITREE_BRIDGE_SYSTEMD_UNIT,
                ),
            )
        write_executable(
            self.state.bin_path / "elesim-update",
            render_update_wrapper(
                edition="general",
                prefix=self.state.prefix_path,
                state_path=self.state_path,
                repository=self.state.source_repository,
                ref=self.state.source_ref,
                runtime_uid=os.getuid(),
            ),
        )

    def _wrapper_paths(self, *, include_uninstaller: bool = False) -> tuple[Path, ...]:
        names = [
            "elesim-setup",
            "elesim-net",
            "elesim-robot",
            "elesim-unitree-bridge",
            "elesim-up",
            "elesim-logs",
            "elesim-down",
            "elesim-status",
            "elesim-update",
        ]
        if include_uninstaller:
            names.append("elesim-uninstall")
        return tuple(self.state.bin_path / name for name in names)

    def _write_robot_service_units(self) -> tuple[Path, Path]:
        """Generate the two native units from this install's exact paths."""

        if self.state.roles != ("robot",):
            raise ValueError("native installer is reserved for a Robot-only host")
        role_root = role_directory(self.state, "robot")
        systemd_root = role_root / "systemd"
        robot_service = systemd_root / ROBOT_SYSTEMD_UNIT
        bridge_service = systemd_root / UNITREE_BRIDGE_SYSTEMD_UNIT
        robot_wrapper = self.state.bin_path / "elesim-robot"
        bridge_wrapper = self.state.bin_path / "elesim-unitree-bridge"
        marker = provisioning_required_path(self.state)
        robot_content = (
            "[Unit]\n"
            "Description=EleSim robot hardware endpoint\n"
            f"After=network-online.target {UNITREE_BRIDGE_SYSTEMD_UNIT}\n"
            "Wants=network-online.target\n"
            f"BindsTo={UNITREE_BRIDGE_SYSTEMD_UNIT}\n"
            f"ConditionPathExists=!{_systemd_quote(marker)}\n"
            "\n"
            "[Service]\n"
            "Type=simple\n"
            f"User={self.robot_host.robot_user}\n"
            f"SupplementaryGroups={self.robot_host.bridge_user}\n"
            f"Environment={_systemd_quote(f'HOME={self.robot_host.robot_home}')}\n"
            f"WorkingDirectory={_systemd_quote(role_root)}\n"
            f"ExecStart={_systemd_quote(robot_wrapper)}\n"
            "Restart=on-failure\n"
            "RestartPreventExitStatus=78\n"
            "RestartSec=2\n"
            "TimeoutStopSec=10\n"
            "\n"
            "[Install]\n"
            "WantedBy=multi-user.target\n"
        )
        bridge_content = (
            "[Unit]\n"
            "Description=EleSim local Unitree DDS bridge\n"
            "After=network-online.target\n"
            "Wants=network-online.target\n"
            f"PartOf={ROBOT_SYSTEMD_UNIT}\n"
            f"ConditionPathExists=!{_systemd_quote(marker)}\n"
            "\n"
            "[Service]\n"
            "Type=simple\n"
            f"User={self.robot_host.bridge_user}\n"
            f"Group={self.robot_host.bridge_user}\n"
            "UMask=0007\n"
            "RuntimeDirectory=elesim-unitree\n"
            "RuntimeDirectoryMode=0750\n"
            f"WorkingDirectory={_systemd_quote(role_root)}\n"
            'Environment="ROS_LOG_DIR=/run/elesim-unitree/ros-log"\n'
            f"ExecStart={_systemd_quote(bridge_wrapper)}\n"
            "Restart=on-failure\n"
            "RestartPreventExitStatus=78\n"
            "RestartSec=2\n"
            "TimeoutStopSec=10\n"
            "NoNewPrivileges=true\n"
            "PrivateTmp=true\n"
            "ProtectSystem=strict\n"
            "ProtectHome=read-only\n"
            "DevicePolicy=closed\n"
            "InaccessiblePaths=-/etc/elesim/keystore -/etc/elesim/security "
            f"-{self.state.prefix_path / 'security'} -/dev/ttyUSB0 -/dev/video0\n"
            "RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK\n"
            "\n"
            "[Install]\n"
            "WantedBy=multi-user.target\n"
        )
        _write_regular_file(robot_service, robot_content, mode=0o644)
        _write_regular_file(bridge_service, bridge_content, mode=0o644)
        return robot_service, bridge_service

    def _log_robot_service_registration(self, services: Sequence[Path]) -> None:
        robot_service, bridge_service = tuple(services)
        self.log("[Robot systemd] 설치기는 sudo/systemd 상태를 변경하지 않았습니다.")
        self.log(
            "$ sudo groupadd --force --system "
            + shlex.quote(self.robot_host.bridge_user)
        )
        self.log(
            "$ id -u "
            + shlex.quote(self.robot_host.bridge_user)
            + " >/dev/null 2>&1 || sudo useradd --system --gid "
            + shlex.quote(self.robot_host.bridge_user)
            + " --home-dir /nonexistent --shell /usr/sbin/nologin "
            + shlex.quote(self.robot_host.bridge_user)
        )
        self.log(
            "$ sudo usermod --append --groups "
            + shlex.quote(self.robot_host.bridge_user)
            + " "
            + shlex.quote(self.robot_host.robot_user)
        )
        access_roots = (
            role_directory(self.state, "robot"),
            self.state.prefix_path / "ros",
            self.robot_host.unitree_ros_workspace,
        )
        ancestors = _access_ancestors(
            (*access_roots, self.state.bin_path / "elesim-unitree-bridge")
        )
        if ancestors:
            self.log(
                "$ sudo setfacl -m u:"
                + self.robot_host.bridge_user
                + ":x -- "
                + " ".join(shlex.quote(str(path)) for path in ancestors)
            )
        self.log(
            "$ sudo setfacl -R -m u:"
            + self.robot_host.bridge_user
            + ":rX -- "
            + " ".join(shlex.quote(str(path)) for path in access_roots)
        )
        self.log(
            "$ sudo setfacl -m u:"
            + self.robot_host.bridge_user
            + ":rx -- "
            + shlex.quote(str(self.state.bin_path / "elesim-unitree-bridge"))
        )
        self.log(
            "$ sudo install -m 0644 -- "
            + shlex.quote(str(bridge_service))
            + f" /etc/systemd/system/{UNITREE_BRIDGE_SYSTEMD_UNIT}"
        )
        self.log(
            "$ sudo install -m 0644 -- "
            + shlex.quote(str(robot_service))
            + f" /etc/systemd/system/{ROBOT_SYSTEMD_UNIT}"
        )
        self.log("$ sudo systemctl daemon-reload")
        if self.state.dds.managed_security_pending:
            self.log(f"$ sudo systemctl enable {ROBOT_SYSTEMD_UNIT}")
            self.log(
                "[Robot systemd] elesim-connections provisioning이 끝나기 전에는 "
                "서비스를 시작하지 마십시오."
            )
        else:
            self.log(f"$ sudo systemctl enable --now {ROBOT_SYSTEMD_UNIT}")

    def _dds_environment(
        self,
        role: str,
        *,
        enclave_override: bool = False,
    ) -> Mapping[str, str]:
        config_role = role if role in self.state.roles else self.state.roles[0]
        identity_role = role if role in self.state.roles else self.state.roles[0]
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
                    "ROS_SECURITY_KEYSTORE": str(
                        role_keystore_path(self.state, identity_role)
                    ),
                    "ELESIM_DDS_ENCLAVE": dds_enclave(
                        self.state,
                        identity_role,
                    ),
                }
            )
            if enclave_override:
                environment["ROS_SECURITY_ENCLAVE_OVERRIDE"] = dds_enclave(
                    self.state,
                    identity_role,
                )
        else:
            environment["ROS_SECURITY_ENABLE"] = "false"
        return environment

    def _role_environment(self, role: str) -> Mapping[str, str]:
        if role not in {"pilot", "sim"}:
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
        if role == "pilot":
            return executable, (
                "--config",
                str(config / "config.yaml"),
                "--runtime-config",
                str(config / "runtime.installed.yaml"),
            )
        if role == "ui":
            return executable, ("--config", str(config / "installed.yaml"))
        if role == "sim":
            return executable, (
                "--config",
                str(config / "app.installed.yaml"),
                "--runtime-config",
                str(config / "runtime.installed.yaml"),
            )
        if role == "robot":
            return executable, ("--config", str(config / "installed.yaml"))
        raise ValueError(f"unknown role: {role}")


def preflight_notes(
    roles: Iterable[str],
    *,
    install_mode: str = "container",
) -> tuple[str, ...]:
    selected = set(roles)
    notes: list[str] = []
    if install_mode == "container":
        notes.append("호스트에는 Docker Engine과 Docker Compose plugin만 필요합니다.")
        if {"pilot", "sim"}.intersection(selected):
            notes.append("GPU 모드는 NVIDIA driver와 NVIDIA Container Toolkit이 필요합니다.")
        if "ui" in selected:
            notes.append("UI는 호스트 X11 display socket을 컨테이너에 전달합니다.")
        notes.append("컨테이너 설치는 호스트 APT/Python 환경을 변경하지 않습니다.")
        return tuple(notes)
    if "sim" in selected:
        notes.append("Sim는 git, Genesis가 지원하는 GPU driver와 graphics runtime이 별도로 필요합니다.")
    if "ui" in selected:
        notes.append("UI는 OpenGL/GLFW와 데스크톱 display 환경이 필요합니다.")
    if "robot" in selected:
        notes.append("Robot은 ROS2 Humble, unitree_ros2, RealSense와 serial 장치 권한이 필요합니다.")
    notes.append("설치기는 sudo나 방화벽 설정을 자동 실행하지 않습니다.")
    return tuple(notes)


def _resolve_native_robot_host() -> NativeRobotHost:
    configured_user = os.environ.get("ELESIM_HOST_USER", "").strip()
    account = None
    try:
        account = pwd.getpwuid(os.getuid())
    except KeyError:
        # Containers can run with a numeric UID that has no passwd entry.
        # Environment-provided identity and HOME remain valid fallbacks.
        account = None
    robot_user = configured_user or (account.pw_name if account is not None else "")
    configured_home = (
        os.environ.get("ELESIM_OPERATOR_HOME", "").strip()
        or os.environ.get("HOME", "").strip()
        or (account.pw_dir if account is not None else "")
    )
    if not configured_home:
        raise ValueError("native Robot 설치의 host home을 확인할 수 없습니다")
    robot_home = Path(configured_home).expanduser()
    if not robot_home.is_absolute():
        raise ValueError("native Robot host home은 절대 경로여야 합니다")
    workspace_value = os.environ.get("ELESIM_UNITREE_ROS2_WS", "").strip()
    if not workspace_value:
        workspace_value = os.environ.get("UNITREE_ROS2_WS", "").strip()
    unitree_workspace = (
        Path(workspace_value).expanduser()
        if workspace_value
        else robot_home / "ros2_ws"
    )
    unitree_interface = os.environ.get("ELESIM_UNITREE_INTERFACE", "eth0").strip()
    domain_value = os.environ.get("ELESIM_UNITREE_DOMAIN_ID", "1").strip()
    try:
        unitree_domain_id = int(domain_value)
    except ValueError as exc:
        raise ValueError("ELESIM_UNITREE_DOMAIN_ID는 0..232 정수여야 합니다") from exc
    host = NativeRobotHost(
        robot_user=robot_user,
        robot_home=robot_home.resolve(),
        bridge_user=UNITREE_BRIDGE_USER,
        unitree_ros_workspace=unitree_workspace.resolve(),
        unitree_interface=unitree_interface,
        unitree_domain_id=unitree_domain_id,
    )
    host.config_settings.validate()
    return host


def _access_ancestors(paths: Iterable[Path]) -> tuple[Path, ...]:
    """Return exact parent directories that a service account must traverse."""

    result: set[Path] = set()
    for raw in paths:
        path = raw.expanduser().resolve()
        parent = path.parent
        while parent != parent.parent:
            result.add(parent)
            parent = parent.parent
    return tuple(sorted(result, key=lambda value: (len(value.parts), str(value))))


def _is_within(path: Path, root: Path) -> bool:
    try:
        path.expanduser().resolve().relative_to(root.expanduser().resolve())
    except ValueError:
        return False
    return True


def _copy_tree(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise FileNotFoundError(source)
    _reject_source_symlinks(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, destination, dirs_exist_ok=True)


def _ensure_python_pip(python: Path) -> None:
    """Repair an interrupted native venv before dependency installation."""

    probe = subprocess.run(
        (str(python), "-m", "pip", "--version"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if probe.returncode == 0:
        return
    repair = subprocess.run(
        (str(python), "-m", "ensurepip", "--upgrade"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if repair.returncode != 0:
        detail = (repair.stderr or probe.stderr or "").strip()
        suffix = f" ({detail[-600:]})" if detail else ""
        raise RuntimeError(
            "native EleSim 가상환경에 pip가 없습니다. Python venv/ensurepip 패키지 "
            f"(예: Debian/Ubuntu의 python3-venv)를 설치하십시오{suffix}"
        )
    verify = subprocess.run(
        (str(python), "-m", "pip", "--version"),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if verify.returncode != 0:
        detail = (verify.stderr or "").strip()
        suffix = f" ({detail[-600:]})" if detail else ""
        raise RuntimeError(f"native venv pip 복구 후에도 실행할 수 없습니다{suffix}")


def _reject_source_symlinks(source: Path) -> None:
    if source.is_symlink():
        raise ValueError(f"설치 소스는 symlink일 수 없습니다: {source}")
    for directory, names, files in os.walk(source, followlinks=False):
        for name in (*names, *files):
            path = Path(directory) / name
            if path.is_symlink():
                raise ValueError(
                    "설치 소스 model tree 안의 symlink는 허용되지 않습니다: "
                    f"{path}"
                )


def _exec_script(
    executable: Path,
    arguments: Sequence[str],
    *,
    environment: Mapping[str, str] | None = None,
    source_ros: Path | None = None,
    guard: str = "",
) -> str:
    command = shlex.join((str(executable), *(str(value) for value in arguments)))
    lines = ["#!/usr/bin/env bash", "set -euo pipefail"]
    if guard:
        lines.extend(guard.rstrip("\n").splitlines())
    if source_ros is not None:
        lines.extend(
            (
                "set +u",
                "source /opt/ros/humble/setup.bash",
            )
        )
        lines.append(f"source {shlex.quote(str(source_ros))}")
        lines.append("set -u")
    for name, value in (environment or {}).items():
        lines.append(f"export {name}={shlex.quote(str(value))}")
    lines.append("exec " + command + ' "$@"')
    return "\n".join(lines) + "\n"


def _write_regular_file(path: Path, content: str, *, mode: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.chmod(mode)
    temporary.replace(path)


def _systemd_quote(value: object) -> str:
    text = str(value)
    if "\x00" in text or "\n" in text or "\r" in text:
        raise ValueError("systemd unit values must be single-line text")
    escaped = text.replace("\\", "\\\\").replace('"', '\\"').replace("%", "%%")
    return f'"{escaped}"'


def _native_systemctl_wrapper(action: str) -> str:
    if action not in {"start", "stop"}:
        raise ValueError(f"unsupported Robot systemd action: {action}")
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "if (( $# != 0 )); then\n"
        f"  printf '사용법: elesim-{'up' if action == 'start' else 'down'}\n' >&2\n"
        "  exit 64\n"
        "fi\n"
        f"exec sudo -n systemctl {action} {ROBOT_SYSTEMD_UNIT}\n"
    )


def _native_archive_function(logs_root: Path) -> str:
    """Render a bounded journald export for the two native Robot units."""

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
        '  if ! mkdir -p -- "$runs_root" || '
        '! archive_path_has_no_symlink_ancestor "$logs_root" || '
        '! archive_path_has_no_symlink_ancestor "$runs_root" || '
        '[[ ! -d "$logs_root" || ! -d "$runs_root" ]]; then\n'
        "    printf '로그 archive 디렉터리를 안전하게 만들 수 없습니다: %s\\n' "
        '"$runs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  if ! chmod 0700 -- "$logs_root" "$runs_root"; then\n'
        "    printf '로그 archive 디렉터리 권한 설정 실패: %s\\n' "
        '"$runs_root" >&2\n'
        "    return 74\n"
        "  fi\n"
        "  local timestamp\n"
        '  if ! timestamp="$(date -u +%Y%m%dT%H%M%S.%NZ)"; then\n'
        "    printf 'UTC 로그 archive timestamp 생성 실패.\\n' >&2\n"
        "    return 74\n"
        "  fi\n"
        '  local run_dir="$runs_root/$timestamp"\n'
        '  if [[ -e "$run_dir" || -L "$run_dir" ]] || '
        '! mkdir -- "$run_dir"; then\n'
        "    printf '고유 로그 archive 디렉터리 생성 실패: %s\\n' "
        '"$run_dir" >&2\n'
        "    return 74\n"
        "  fi\n"
        '  chmod 0700 -- "$run_dir" || return 74\n'
        '  local destination="$run_dir/robot.log"\n'
        "  local archive_status=0\n"
        "  if ! sudo -n journalctl --no-pager --output=short-iso-precise "
        f"--unit={ROBOT_SYSTEMD_UNIT} --unit={UNITREE_BRIDGE_SYSTEMD_UNIT} "
        f"2>&1 | tail -c {NATIVE_RUNTIME_LOG_BYTES} >\"$destination\"; then\n"
        "    printf 'Robot journald 로그 저장 실패: %s\\n' "
        '"$destination" >&2\n'
        "    archive_status=74\n"
        "  fi\n"
        '  chmod 0600 -- "$destination" || archive_status=74\n'
        "  local -a generations=()\n"
        "  local candidate name\n"
        "  shopt -s nullglob\n"
        '  for candidate in "$runs_root"/*; do\n'
        '    [[ -d "$candidate" && ! -L "$candidate" ]] || continue\n'
        '    name="${candidate##*/}"\n'
        "    [[ $name =~ ^[0-9]{8}T[0-9]{6}\\.[0-9]{9}Z$ ]] || continue\n"
        '    generations+=("$candidate")\n'
        "  done\n"
        f"  if (( ${{#generations[@]}} > {NATIVE_RUNTIME_LOG_RETENTION} )); then\n"
        "    mapfile -t generations < <(printf '%s\\n' "
        '"${generations[@]}" | LC_ALL=C sort)\n'
        f"    local remove_count=$((${{#generations[@]}} - {NATIVE_RUNTIME_LOG_RETENTION}))\n"
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
        '      rm -rf -- "$candidate" || archive_status=74\n'
        "    done\n"
        "  fi\n"
        "  printf '로그 archive: %s\\n' \"$run_dir\"\n"
        '  return "$archive_status"\n'
        "}\n"
    )


def _native_logs_wrapper(*, logs_root: Path, archive_enabled: bool) -> str:
    archive = _native_archive_function(logs_root) if archive_enabled else ""
    save = (
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
        + archive
        + "if (( $# == 0 )); then\n"
        + "  exec sudo -n journalctl --follow "
        + f"--unit={ROBOT_SYSTEMD_UNIT} --unit={UNITREE_BRIDGE_SYSTEMD_UNIT}\n"
        + "fi\n"
        + "if (( $# == 1 )) && [[ $1 == --save ]]; then\n"
        + save
        + "  exit $?\n"
        + "fi\n"
        + "printf '사용법: elesim-logs [--save]\\n' >&2\n"
        + "exit 64\n"
    )


def _native_down_wrapper(*, logs_root: Path, archive_enabled: bool) -> str:
    if not archive_enabled:
        return _native_systemctl_wrapper("stop")
    return (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "umask 077\n"
        "if (( $# != 0 )); then\n"
        "  printf '사용법: elesim-down\\n' >&2\n"
        "  exit 64\n"
        "fi\n"
        + _native_archive_function(logs_root)
        + "archive_status=0\n"
        + "archive_runtime_logs || archive_status=$?\n"
        + "stop_status=0\n"
        + f"sudo -n systemctl stop {ROBOT_SYSTEMD_UNIT} || stop_status=$?\n"
        + "if (( stop_status != 0 )); then\n"
        + '  exit "$stop_status"\n'
        + "fi\n"
        + 'exit "$archive_status"\n'
    )


__all__ = [
    "GO2_MPC_PACKAGE",
    "InstallAction",
    "Installer",
    "NativeRobotHost",
    "NATIVE_RUNTIME_LOG_BYTES",
    "NATIVE_RUNTIME_LOG_RETENTION",
    "ROBOT_SYSTEMD_UNIT",
    "UNITREE_BRIDGE_SYSTEMD_UNIT",
    "UNITREE_BRIDGE_USER",
    "build_install_plan",
    "preflight_notes",
]
