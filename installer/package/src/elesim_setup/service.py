"""One installation coordinator shared by terminal automation and the web UI."""

from __future__ import annotations

from typing import Callable

from .capabilities import HostCapabilities
from .container_installer import ContainerInstaller
from .developer import DeveloperInstaller
from .installer import Installer
from .request import SetupRequest
from .shell import operator_home, register_bash_path


Log = Callable[[str], None]


class SetupService:
    def __init__(
        self,
        capabilities: HostCapabilities,
        *,
        log: Log = print,
        dry_run: bool = False,
    ) -> None:
        self.capabilities = capabilities
        self.log = log
        self.dry_run = bool(dry_run)

    def run(self, request: SetupRequest) -> None:
        request.validate(self.capabilities)
        self.log(
            f"[setup] edition={request.edition} prefix={request.prefix} "
            f"gpu={request.compute.gpu_mode}"
        )
        shell_bashrc = operator_home() / ".bashrc" if request.register_path else None
        if request.edition == "developer":
            DeveloperInstaller(
                request,
                capabilities=self.capabilities,
                shell_bashrc=shell_bashrc,
                dry_run=self.dry_run,
                log=self.log,
            ).run()
        else:
            state = request.to_install_state()
            installer_type = (
                ContainerInstaller if state.install_mode == "container" else Installer
            )
            installer_type(
                state,
                state_path=state.state_path,
                shell_bashrc=shell_bashrc,
                dry_run=self.dry_run,
                log=self.log,
            ).run()
        if request.register_path and not self.dry_run:
            assert shell_bashrc is not None
            result = register_bash_path(request.bin_dir, bashrc=shell_bashrc)
            status = "updated" if result.changed else "already current"
            self.log(f"[shell] {result.bashrc} {status}")
            self.log("[shell] current terminal: source ~/.bashrc")



__all__ = ["SetupService"]
