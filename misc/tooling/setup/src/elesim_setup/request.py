"""Transport-neutral setup request shared by CLI automation and the web wizard."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .capabilities import HostCapabilities
from .profiles import normalize_roles
from .state import (
    ComputeSettings,
    InstallState,
    NetworkSettings,
    SecuritySettings,
    TurnSettings,
)


EDITIONS = frozenset({"general", "developer"})
CREDENTIAL_SOURCES = frozenset({"unused", "existing", "generate", "ssh"})


@dataclass(frozen=True)
class SshCredentialSource:
    host: str = ""
    port: int = 22
    user: str = ""
    remote_root: str = ""
    identity_file: str = ""
    accepted_fingerprint: str = ""

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any] | None) -> "SshCredentialSource":
        values = {} if raw is None else dict(raw)
        return cls(
            host=str(values.get("host", "")),
            port=int(values.get("port", 22)),
            user=str(values.get("user", "")),
            remote_root=str(values.get("remote_root", "")),
            identity_file=str(values.get("identity_file", "")),
            accepted_fingerprint=str(values.get("accepted_fingerprint", "")),
        )

    def validate(self) -> "SshCredentialSource":
        if not self.host.strip() or any(character.isspace() for character in self.host):
            raise ValueError("SSH credential host가 필요합니다")
        if not self.user.strip() or any(character.isspace() for character in self.user):
            raise ValueError("SSH credential user가 필요합니다")
        if not 1 <= int(self.port) <= 65535:
            raise ValueError("SSH port는 1..65535 범위여야 합니다")
        if not self.remote_root.strip():
            raise ValueError("원격 credential root가 필요합니다")
        if not self.accepted_fingerprint.strip():
            raise ValueError("승인된 SSH host fingerprint가 필요합니다")
        return self


@dataclass(frozen=True)
class SetupRequest:
    language: str
    edition: str
    roles: tuple[str, ...]
    prefix: Path
    bin_dir: Path
    source_root: Path
    compute: ComputeSettings
    network: NetworkSettings
    security: SecuritySettings
    turn: TurnSettings
    credential_source: str = "unused"
    ssh: SshCredentialSource = field(default_factory=SshCredentialSource)
    register_path: bool = False
    jaeger: bool = False
    repository: str = "jpyaaa3/elesim"
    ref: str = "main"

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "SetupRequest":
        turn_url = str(raw.get("turn_url", "")).strip()
        roles_raw = raw.get("roles", ())
        if not isinstance(roles_raw, (list, tuple)):
            raise ValueError("roles는 프로그램 이름 목록이어야 합니다")
        return cls(
            language=str(raw.get("language", "ko")),
            edition=str(raw.get("edition", "general")),
            roles=tuple(str(value) for value in roles_raw),
            prefix=_required_path(raw, "prefix"),
            bin_dir=_required_path(raw, "bin_dir"),
            source_root=_required_path(raw, "source_root"),
            compute=ComputeSettings(
                gpu_mode=str(raw.get("gpu_mode", "inherit")),
                gpu_device=str(raw.get("gpu_device", "")),
            ),
            network=NetworkSettings(
                router_host=str(raw.get("router_host", "127.0.0.1")),
                advertise_host=str(raw.get("advertise_host", "127.0.0.1")),
                router_port=int(raw.get("router_port", 5558)),
                rgbd_port=int(raw.get("rgbd_port", 5568)),
                turn_urls=(turn_url,) if turn_url else (),
                simulator_id=str(raw.get("simulator_id", "sim-default")),
                controller_id=str(raw.get("controller_id", "controller-main")),
            ),
            security=SecuritySettings(
                mode=str(raw.get("security_mode", "loopback")),
                credentials_root=str(raw.get("credentials_root", "")),
            ),
            turn=TurnSettings(
                mode=str(raw.get("turn_mode", "none")),
                realm=str(raw.get("turn_realm", "")),
                public_host=str(raw.get("turn_public_host", "")),
            ),
            credential_source=str(raw.get("credential_source", "unused")),
            ssh=SshCredentialSource.from_dict(
                raw.get("ssh") if isinstance(raw.get("ssh"), Mapping) else None
            ),
            register_path=bool(raw.get("register_path", False)),
            jaeger=bool(raw.get("jaeger", False)),
            repository=str(raw.get("repository", "jpyaaa3/elesim")),
            ref=str(raw.get("ref", "main")),
        )

    def validate(self, capabilities: HostCapabilities) -> "SetupRequest":
        if self.language not in {"ko", "en"}:
            raise ValueError(f"지원하지 않는 언어: {self.language!r}")
        if self.edition not in EDITIONS:
            raise ValueError(f"지원하지 않는 설치 종류: {self.edition!r}")
        if not str(self.prefix) or not str(self.bin_dir) or not str(self.source_root):
            raise ValueError("설치, bin, source 경로가 필요합니다")
        self.compute.validate()
        self.network.validate()
        self.security.validate()
        self.turn.validate()
        if self.credential_source not in CREDENTIAL_SOURCES:
            raise ValueError(f"지원하지 않는 credential source: {self.credential_source!r}")
        if self.edition == "developer":
            if not capabilities.developer_installable:
                raise ValueError("개발자 환경은 Ubuntu/WSL amd64에서만 지원합니다")
            if self.roles:
                raise ValueError("개발자 환경은 역할을 고르지 않고 전체 workspace를 설치합니다")
        else:
            roles = normalize_roles(self.roles)
            if "robot" in roles:
                if roles != ("robot",):
                    raise ValueError("Robot native 설치는 다른 역할과 분리한 단독 설치여야 합니다")
                if not capabilities.robot_installable:
                    raise ValueError("Robot 설치에는 감지된 Jetson/JetPack 호스트가 필요합니다")
        remote_curve = self.security.mode == "curve"
        if not remote_curve and self.credential_source != "unused":
            raise ValueError("credential source는 CURVE 보안 모드에서만 사용합니다")
        if remote_curve and self.credential_source == "unused":
            raise ValueError("CURVE 보안에는 credential source가 필요합니다")
        if (
            self.credential_source == "generate"
            and self.edition != "developer"
            and "router" not in self.roles
        ):
            raise ValueError("credential 생성은 Router 설치 호스트에서만 가능합니다")
        if self.credential_source == "ssh":
            self.ssh.validate()
        if self.turn.managed:
            if self.edition != "developer" and "router" not in self.roles:
                raise ValueError("managed Coturn은 Router 설치 호스트가 필요합니다")
            if self.security.mode != "curve":
                raise ValueError("managed Coturn은 CURVE 보안이 필요합니다")
        self._state().validate()
        return self

    def to_install_state(self) -> InstallState:
        if self.edition != "general":
            raise ValueError("developer request는 runtime InstallState로 변환할 수 없습니다")
        return self._state().validate()

    def _state(self) -> InstallState:
        roles = (
            normalize_roles(self.roles)
            if self.roles
            else ("router",) if self.edition == "developer" else ()
        )
        install_mode = "native" if roles == ("robot",) else "container"
        return InstallState(
            profile="custom",
            roles=roles,
            prefix=str(self.prefix),
            bin_dir=str(self.bin_dir),
            source_root=str(self.source_root),
            network=self.network,
            security=self.security,
            compute=self.compute,
            turn=self.turn,
            install_mode=install_mode,
        )


def _required_path(raw: Mapping[str, Any], name: str) -> Path:
    value = str(raw.get(name, "")).strip()
    if not value:
        raise ValueError(f"{name} 경로가 필요합니다")
    return Path(value).expanduser().resolve()


__all__ = [
    "CREDENTIAL_SOURCES",
    "EDITIONS",
    "SetupRequest",
    "SshCredentialSource",
]
