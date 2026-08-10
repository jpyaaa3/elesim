"""Fail-closed, host-only Elesim uninstaller.

Only Python's standard library is used here.  The command must remain usable
while it removes the generated tools image/venv that originally supplied it.
"""

from __future__ import annotations

import argparse
import json
import os
import stat
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping, Sequence

from .ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    DockerOwnership,
    OwnedPath,
    OwnershipError,
    OwnershipManifest,
    default_manifest_path,
    sha256_file,
)
from .shell import inspect_bash_path, unregister_bash_path


CommandRunner = Callable[[Sequence[str]], subprocess.CompletedProcess[str]]


class UninstallSafetyError(RuntimeError):
    """Raised before mutation when ownership cannot be proven."""


@dataclass(frozen=True)
class DockerObject:
    name: str
    object_id: str


@dataclass(frozen=True)
class UninstallPlan:
    manifest: OwnershipManifest
    manifest_sha256: str
    purge_logs: bool
    purge_authority: bool
    remove_paths: tuple[OwnedPath, ...]
    remove_roots: tuple[Path, ...]
    preserve_paths: tuple[Path, ...]
    containers: tuple[DockerObject, ...]
    images: tuple[DockerObject, ...]
    remove_shell_path: bool
    warnings: tuple[str, ...]
    tombstone: Path


def plan_uninstall(
    manifest_path: Path | None = None,
    *,
    purge_logs: bool = True,
    purge_authority: bool = True,
    runner: CommandRunner | None = None,
) -> UninstallPlan:
    """Validate every deletion boundary and return an immutable plan."""

    source = default_manifest_path() if manifest_path is None else _canonical(manifest_path)
    try:
        manifest = OwnershipManifest.load(source)
    except OwnershipError as exc:
        raise UninstallSafetyError(str(exc)) from exc
    manifest_digest = sha256_file(source)
    _validate_install_roots(manifest)
    _validate_owned_paths(manifest)
    _validate_wrappers(manifest)
    _validate_systemd(manifest, runner=runner)

    containers: tuple[DockerObject, ...] = ()
    images: tuple[DockerObject, ...] = ()
    if manifest.docker is not None:
        containers, images = _validate_docker(manifest.docker, runner=runner)

    warnings: list[str] = []
    remove_shell_path = False
    if manifest.shell is not None:
        shell_status = inspect_bash_path(
            Path(manifest.shell.bin_dir),
            bashrc=Path(manifest.shell.bashrc),
        )
        remove_shell_path = shell_status == "exact"
        if shell_status == "foreign":
            warnings.append(
                f"수정되었거나 다른 설치가 소유한 PATH block 보존: {manifest.shell.bashrc}"
            )

    preserve = [Path(value) for value in manifest.external_paths]
    if not purge_logs:
        preserve.extend(Path(value) for value in manifest.log_roots)
    if not purge_authority:
        preserve.extend(Path(value) for value in manifest.authority_roots)
    preserve_paths = _minimal_roots(preserve)

    remove_root_values = [Path(value) for value in manifest.managed_roots]
    if purge_logs:
        remove_root_values.extend(Path(value) for value in manifest.log_roots)
    if purge_authority:
        remove_root_values.extend(Path(value) for value in manifest.authority_roots)
    remove_roots = tuple(
        root
        for root in _minimal_roots(remove_root_values)
        if not _is_protected(root, preserve_paths)
    )
    remove_paths = tuple(
        entry
        for entry in manifest.owned_paths
        if not _is_protected(Path(entry.path), preserve_paths)
    )
    _validate_no_nested_mounts(
        manifest,
        remove_roots=remove_roots,
        remove_paths=remove_paths,
    )

    tombstone = _uninstall_state_root() / f"{manifest.install_uuid}.json"
    if _lexists(tombstone):
        raise UninstallSafetyError(f"uninstall tombstone이 이미 존재합니다: {tombstone}")
    return UninstallPlan(
        manifest=manifest,
        manifest_sha256=manifest_digest,
        purge_logs=bool(purge_logs),
        purge_authority=bool(purge_authority),
        remove_paths=remove_paths,
        remove_roots=remove_roots,
        preserve_paths=preserve_paths,
        containers=containers,
        images=images,
        remove_shell_path=remove_shell_path,
        warnings=tuple(warnings),
        tombstone=tombstone,
    )


def execute_uninstall(
    plan: UninstallPlan,
    *,
    confirm_prefix: str | None = None,
    runner: CommandRunner | None = None,
) -> Path:
    """Execute a prevalidated ownership plan.

    ``confirm_prefix`` remains an internal compatibility guard for callers
    that already supply it.  The host CLI deliberately needs no memorized
    confirmation: locating and validating the exact manifest is the safety
    boundary.
    """

    if confirm_prefix is not None and confirm_prefix != plan.manifest.prefix:
        raise UninstallSafetyError(
            "--confirm-prefix가 ownership manifest의 정확한 prefix와 다릅니다: "
            f"expected={plan.manifest.prefix}"
        )

    # Fail closed if the manifest or any ownership fact changed after --plan.
    current = plan_uninstall(
        plan.manifest.path,
        purge_logs=plan.purge_logs,
        purge_authority=plan.purge_authority,
        runner=runner,
    )
    if current.manifest_sha256 != plan.manifest_sha256:
        raise UninstallSafetyError("plan 이후 ownership manifest가 변경되었습니다")
    if (
        current.remove_paths != plan.remove_paths
        or current.remove_roots != plan.remove_roots
        or current.containers != plan.containers
        or current.images != plan.images
        or current.remove_shell_path != plan.remove_shell_path
    ):
        raise UninstallSafetyError("plan 이후 설치 소유권 상태가 변경되었습니다")

    if current.remove_shell_path and current.manifest.shell is not None:
        result = unregister_bash_path(
            Path(current.manifest.shell.bin_dir),
            bashrc=Path(current.manifest.shell.bashrc),
        )
        if not result.changed:
            raise UninstallSafetyError("검증 후 PATH block이 변경되어 제거하지 않았습니다")

    command_runner = _command_runner(runner)
    docker_ownership = current.manifest.docker
    for container in current.containers:
        if docker_ownership is None:
            raise UninstallSafetyError("Docker ownership disappeared after validation")
        result = command_runner(
            _docker_command(
                docker_ownership,
                ("docker", "container", "rm", "--force", container.object_id),
            )
        )
        _require_command(result, action=f"container 제거 {container.name}")
    for image in current.images:
        if docker_ownership is None:
            raise UninstallSafetyError("Docker ownership disappeared after validation")
        result = command_runner(
            _docker_command(
                docker_ownership,
                ("docker", "image", "rm", image.name),
            )
        )
        _require_command(result, action=f"local image 제거 {image.name}")

    filesystem_protection = (*current.preserve_paths, current.manifest.path)
    for root in sorted(current.remove_roots, key=lambda path: len(path.parts), reverse=True):
        _remove_tree(root, protected=filesystem_protection)

    files = [entry for entry in current.remove_paths if entry.kind != "directory"]
    directories = [entry for entry in current.remove_paths if entry.kind == "directory"]
    for entry in sorted(files, key=lambda value: len(Path(value.path).parts), reverse=True):
        path = Path(entry.path)
        if _lexists(path):
            path.unlink()
    for wrapper in current.manifest.wrappers:
        path = Path(wrapper.path)
        if _lexists(path) and not _is_protected(path, current.preserve_paths):
            path.unlink()
    for entry in sorted(
        directories,
        key=lambda value: len(Path(value.path).parts),
        reverse=True,
    ):
        _rmdir_if_empty(Path(entry.path))

    for root in sorted(
        (Path(value) for value in current.manifest.created_roots),
        key=lambda path: len(path.parts),
        reverse=True,
    ):
        if root != current.manifest.prefix_path:
            _rmdir_if_empty(root)

    # Prepare the tombstone first, then publish it only after unlinking the
    # manifest.  These are intentionally the last two ownership mutations.
    tombstone_payload = {
        "schema_version": 1,
        "install_uuid": current.manifest.install_uuid,
        "edition": current.manifest.edition,
        "prefix": current.manifest.prefix,
        "completed_at": datetime.now(timezone.utc).isoformat(),
        "purged_logs": current.purge_logs,
        "purged_authority": current.purge_authority,
        "preserved_paths": [str(path) for path in current.preserve_paths],
    }
    temporary = _write_tombstone_temporary(
        current.tombstone,
        tombstone_payload,
    )
    current.manifest.path.unlink()
    os.replace(temporary, current.tombstone)
    return current.tombstone


def render_plan(plan: UninstallPlan) -> str:
    lines = [
        "Elesim 제거 계획 (아직 변경하지 않음)",
        f"  install UUID: {plan.manifest.install_uuid}",
        f"  prefix: {plan.manifest.prefix}",
        f"  exact paths: {len(plan.remove_paths)}",
        f"  managed roots: {len(plan.remove_roots)}",
        f"  containers: {', '.join(value.name for value in plan.containers) or '-'}",
        f"  local images: {', '.join(value.name for value in plan.images) or '-'}",
        f"  logs: {'삭제' if plan.purge_logs else '보존'}",
        f"  operator Authority: {'삭제' if plan.purge_authority else '보존'}",
    ]
    if plan.preserve_paths:
        lines.append("  보존 경로:")
        lines.extend(f"    - {path}" for path in plan.preserve_paths)
    if plan.warnings:
        lines.append("  경고:")
        lines.extend(f"    - {warning}" for warning in plan.warnings)
    lines.append("실행 시 위 ownership 경계를 다시 검증한 뒤 즉시 제거합니다.")
    return "\n".join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="elesim-uninstall",
        description="ownership manifest 기반의 안전한 Elesim 제거",
    )
    parser.add_argument(
        "--manifest",
        default=str(default_manifest_path()),
        help="install-ownership.json 경로",
    )
    parser.add_argument(
        "--plan",
        action="store_true",
        help="검증된 제거 계획만 출력하고 변경하지 않음",
    )
    parser.add_argument(
        "--keep-logs",
        action="store_true",
        help="기본 삭제되는 runtime text logs를 보존",
    )
    parser.add_argument(
        "--keep-authority",
        action="store_true",
        help="기본 삭제되는 operator SROS2 Authority를 보존",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        plan = plan_uninstall(
            Path(args.manifest),
            purge_logs=not bool(args.keep_logs),
            purge_authority=not bool(args.keep_authority),
        )
        print(render_plan(plan))
        if args.plan:
            return 0
        tombstone = execute_uninstall(plan)
        print(f"Elesim 제거 완료. tombstone: {tombstone}")
        return 0
    except (OSError, UninstallSafetyError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2


def _validate_install_roots(manifest: OwnershipManifest) -> None:
    prefix = manifest.prefix_path
    if prefix.is_symlink() or not prefix.is_dir():
        raise UninstallSafetyError(f"prefix가 symlink이거나 directory가 아닙니다: {prefix}")
    if str(prefix.resolve(strict=True)) != manifest.prefix_realpath:
        raise UninstallSafetyError("prefix realpath가 설치 시점과 다릅니다")
    bin_dir = manifest.bin_path
    if _lexists(bin_dir):
        if bin_dir.is_symlink() or not bin_dir.is_dir():
            raise UninstallSafetyError(f"bin_dir가 symlink이거나 directory가 아닙니다: {bin_dir}")
        if str(bin_dir.resolve(strict=True)) != manifest.bin_dir_realpath:
            raise UninstallSafetyError("bin_dir realpath가 설치 시점과 다릅니다")
    _ensure_no_symlink_ancestors(manifest.path, boundary=prefix)


def _validate_owned_paths(manifest: OwnershipManifest) -> None:
    prefix = manifest.prefix_path
    bin_dir = manifest.bin_path
    for entry in manifest.owned_paths:
        path = Path(entry.path)
        boundary = prefix if _within_or_equal(path, prefix) else bin_dir
        _ensure_no_symlink_ancestors(path, boundary=boundary)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        actual = (
            "file"
            if stat.S_ISREG(mode)
            else "directory"
            if stat.S_ISDIR(mode)
            else "symlink"
            if stat.S_ISLNK(mode)
            else "other"
        )
        if actual != entry.kind:
            raise UninstallSafetyError(
                f"owned path 유형이 변경되었습니다: {path}: expected={entry.kind} actual={actual}"
            )
    for value in (
        *manifest.managed_roots,
        *manifest.log_roots,
        *manifest.authority_roots,
    ):
        path = Path(value)
        _ensure_no_symlink_ancestors(path, boundary=prefix)
        if _lexists(path) and (path.is_symlink() or not path.is_dir()):
            raise UninstallSafetyError(f"managed/preserved root가 안전한 directory가 아닙니다: {path}")


def _validate_wrappers(manifest: OwnershipManifest) -> None:
    for wrapper in manifest.wrappers:
        path = Path(wrapper.path)
        _ensure_no_symlink_ancestors(path, boundary=manifest.bin_path)
        if not _lexists(path):
            continue
        mode = path.lstat().st_mode
        if not stat.S_ISREG(mode) or stat.S_ISLNK(mode):
            raise UninstallSafetyError(f"wrapper가 일반 파일이 아닙니다: {path}")
        if sha256_file(path) != wrapper.sha256:
            raise UninstallSafetyError(
                f"wrapper가 설치 후 변경되었습니다. 삭제하지 않습니다: {path}"
            )


def _validate_systemd(
    manifest: OwnershipManifest,
    *,
    runner: CommandRunner | None,
) -> None:
    if not manifest.systemd_units:
        return
    command_runner = _command_runner(runner)
    for unit in manifest.systemd_units:
        destination = Path(unit.destination)
        result = command_runner(
            (
                "systemctl",
                "show",
                unit.name,
                "--property=LoadState",
                "--property=ActiveState",
                "--property=FragmentPath",
                "--no-pager",
            )
        )
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"systemd 상태를 확인할 수 없습니다: {unit.name}: {result.stderr.strip()}"
            )
        values = _key_values(result.stdout)
        load_state = values.get("LoadState", "not-found")
        fragment_text = values.get("FragmentPath", "").strip()
        fragment = None if not fragment_text else _canonical(Path(fragment_text))
        installed = _lexists(destination) or load_state != "not-found" or fragment is not None
        active = values.get("ActiveState", "inactive") not in {"inactive", "failed", "dead"}
        if installed or active:
            exact_copy = (
                _lexists(destination)
                and not destination.is_symlink()
                and destination.is_file()
                and sha256_file(destination) == unit.sha256
                and (fragment is None or fragment == destination)
            )
            if not exact_copy:
                raise UninstallSafetyError(
                    f"{unit.name}과 같은 이름의 foreign/변경된 systemd unit이 있습니다. "
                    "Elesim은 이 파일을 삭제하지 않습니다. FragmentPath와 unit 내용을 "
                    f"직접 확인해 충돌을 해결하십시오: fragment={fragment_text or '-'} "
                    f"expected={unit.destination}"
                )
            raise UninstallSafetyError(
                f"{unit.name}이 systemd에 설치되어 있거나 실행 중입니다. 먼저 정확히 다음을 실행하십시오:\n"
                f"  sudo systemctl disable --now {unit.name}\n"
                f"  sudo rm -- {unit.destination}\n"
                "  sudo systemctl daemon-reload"
            )


def _validate_no_nested_mounts(
    manifest: OwnershipManifest,
    *,
    remove_roots: Sequence[Path],
    remove_paths: Sequence[OwnedPath],
) -> None:
    mounts = _mount_points()
    prefix = manifest.prefix_path
    bin_dir = manifest.bin_path
    for mount in mounts:
        for root in remove_roots:
            if _within_or_equal(mount, root):
                raise UninstallSafetyError(
                    f"재귀 제거 경계 안에 mount/bind mount가 있습니다: root={root} "
                    f"mount={mount}. 먼저 unmount하십시오."
                )
        if mount in {prefix, bin_dir}:
            continue
        if not (
            _within_or_equal(mount, prefix)
            or _within_or_equal(mount, bin_dir)
        ):
            continue
        for entry in remove_paths:
            if _within_or_equal(Path(entry.path), mount):
                raise UninstallSafetyError(
                    "exact 제거 경로가 nested mount 안에 있습니다: "
                    f"path={entry.path} mount={mount}. 먼저 unmount하십시오."
                )
        for wrapper in manifest.wrappers:
            if _within_or_equal(Path(wrapper.path), mount):
                raise UninstallSafetyError(
                    "wrapper가 nested mount 안에 있습니다: "
                    f"path={wrapper.path} mount={mount}. 먼저 unmount하십시오."
                )


def _mount_points() -> tuple[Path, ...]:
    source = Path("/proc/self/mountinfo")
    try:
        lines = source.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise UninstallSafetyError(
            f"mount 경계를 확인할 수 없습니다: {source}: {exc}"
        ) from exc
    mounts: set[Path] = set()
    for line in lines:
        fields = line.split()
        if len(fields) < 6 or "-" not in fields:
            raise UninstallSafetyError("/proc/self/mountinfo 형식이 유효하지 않습니다")
        value = fields[4]
        for escaped, literal in (
            (r"\040", " "),
            (r"\011", "\t"),
            (r"\012", "\n"),
            (r"\134", "\\"),
        ):
            value = value.replace(escaped, literal)
        mounts.add(_canonical(Path(value)))
    return tuple(sorted(mounts, key=str))


def _validate_docker(
    ownership: DockerOwnership,
    *,
    runner: CommandRunner | None,
) -> tuple[tuple[DockerObject, ...], tuple[DockerObject, ...]]:
    raw_runner = _command_runner(runner)

    def command_runner(argv: Sequence[str]) -> subprocess.CompletedProcess[str]:
        return raw_runner(_docker_command(ownership, argv))

    info = command_runner(("docker", "info", "--format", "{{.ServerVersion}}"))
    if info.returncode != 0:
        raise UninstallSafetyError(
            "Docker daemon에 연결할 수 없어 container/image 소유권을 검증하지 못했습니다: "
            + info.stderr.strip()
        )
    if ownership.engine_id:
        identity = command_runner(("docker", "info", "--format", "{{.ID}}"))
        if identity.returncode != 0 or identity.stdout.strip() != ownership.engine_id:
            observed = identity.stdout.strip() or "unavailable"
            raise UninstallSafetyError(
                "설치 시 고정한 Docker Engine과 현재 daemon이 다릅니다: "
                f"expected={ownership.engine_id!r} actual={observed!r}"
            )
    listed_containers = command_runner(
        ("docker", "container", "ls", "--all", "--format", "{{.Names}}")
    )
    if listed_containers.returncode != 0:
        raise UninstallSafetyError(
            "Docker container 목록을 확인할 수 없습니다: "
            + listed_containers.stderr.strip()
        )
    container_names = {
        value.strip() for value in listed_containers.stdout.splitlines() if value.strip()
    }
    labeled_containers = command_runner(
        (
            "docker",
            "container",
            "ls",
            "--all",
            "--filter",
            f"label=com.docker.compose.project={ownership.project}",
            "--filter",
            f"label={DOCKER_INSTALL_UUID_LABEL}={ownership.install_uuid}",
            "--format",
            "{{.Names}}",
        )
    )
    if labeled_containers.returncode != 0:
        raise UninstallSafetyError(
            "Elesim ownership label container 목록을 확인할 수 없습니다: "
            + labeled_containers.stderr.strip()
        )
    labeled_names = {
        value.strip() for value in labeled_containers.stdout.splitlines() if value.strip()
    }
    unlisted = sorted(labeled_names - set(ownership.containers))
    if unlisted:
        raise UninstallSafetyError(
            "manifest에 없는 동일 설치 container가 실행/잔존합니다. 먼저 종료하십시오: "
            + ", ".join(unlisted)
        )

    containers: list[DockerObject] = []
    expected_compose = str(Path(ownership.compose_file).resolve(strict=False))
    for name in ownership.containers:
        if name not in container_names:
            continue
        result = command_runner(("docker", "container", "inspect", name))
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"목록에 있던 Docker container를 inspect할 수 없습니다: {name}: "
                + result.stderr.strip()
            )
        payload = _inspect_object(result.stdout, kind="container", name=name)
        labels = _labels(payload)
        project = labels.get("com.docker.compose.project", "")
        install_uuid = labels.get(DOCKER_INSTALL_UUID_LABEL, "")
        config_files = labels.get("com.docker.compose.project.config_files", "")
        configs = {
            str(Path(value.strip()).resolve(strict=False))
            for value in config_files.split(",")
            if value.strip()
        }
        if (
            project != ownership.project
            or install_uuid != ownership.install_uuid
            or expected_compose not in configs
        ):
            raise UninstallSafetyError(
                f"고정 container 이름이 다른 설치 소유입니다: {name}: "
                f"project={project!r} install_uuid={install_uuid!r} "
                f"compose={config_files!r}"
            )
        object_id = str(payload.get("Id", ""))
        if not object_id:
            raise UninstallSafetyError(f"Docker container ID가 비어 있습니다: {name}")
        containers.append(DockerObject(name=name, object_id=object_id))

    listed_images = command_runner(
        ("docker", "image", "ls", "--all", "--format", "{{.Repository}}:{{.Tag}}")
    )
    if listed_images.returncode != 0:
        raise UninstallSafetyError(
            "Docker image 목록을 확인할 수 없습니다: " + listed_images.stderr.strip()
        )
    image_names = {
        value.strip() for value in listed_images.stdout.splitlines() if value.strip()
    }
    images: list[DockerObject] = []
    for name in ownership.local_images:
        if name not in image_names:
            continue
        result = command_runner(("docker", "image", "inspect", name))
        if result.returncode != 0:
            raise UninstallSafetyError(
                f"목록에 있던 Docker image를 inspect할 수 없습니다: {name}: "
                + result.stderr.strip()
            )
        payload = _inspect_object(result.stdout, kind="image", name=name)
        labels = _labels(payload)
        project = labels.get("com.docker.compose.project", "")
        install_uuid = labels.get(DOCKER_INSTALL_UUID_LABEL, "")
        if project != ownership.project or install_uuid != ownership.install_uuid:
            raise UninstallSafetyError(
                f"local image 태그가 다른 설치 소유입니다: {name}: "
                f"project={project!r} install_uuid={install_uuid!r}"
            )
        object_id = str(payload.get("Id", ""))
        if not object_id:
            raise UninstallSafetyError(f"Docker image ID가 비어 있습니다: {name}")
        images.append(DockerObject(name=name, object_id=object_id))
    return tuple(containers), tuple(images)


def validate_docker_ownership(
    ownership: DockerOwnership,
    *,
    runner: CommandRunner | None = None,
) -> tuple[tuple[DockerObject, ...], tuple[DockerObject, ...]]:
    """Prove exact Docker labels/Compose boundaries without mutating objects."""

    return _validate_docker(ownership.validate(), runner=runner)


def _docker_command(
    ownership: DockerOwnership,
    argv: Sequence[str],
) -> tuple[str, ...]:
    values = tuple(str(value) for value in argv)
    if not values or values[0] != "docker":
        raise UninstallSafetyError("internal Docker command is malformed")
    if not ownership.context:
        return values
    return ("docker", "--context", ownership.context, *values[1:])


def _inspect_object(stdout: str, *, kind: str, name: str) -> Mapping[str, object]:
    try:
        value = json.loads(stdout)
        if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], Mapping):
            raise ValueError("expected one object")
        return value[0]
    except (json.JSONDecodeError, ValueError) as exc:
        raise UninstallSafetyError(f"Docker {kind} inspect 응답이 유효하지 않습니다: {name}") from exc


def _labels(payload: Mapping[str, object]) -> Mapping[str, str]:
    config = payload.get("Config", {})
    if not isinstance(config, Mapping):
        return {}
    labels = config.get("Labels", {})
    if not isinstance(labels, Mapping):
        return {}
    return {str(key): str(value) for key, value in labels.items() if value is not None}


def _remove_tree(root: Path, *, protected: tuple[Path, ...]) -> None:
    if not _lexists(root) or _is_protected(root, protected):
        return
    mode = root.lstat().st_mode
    if stat.S_ISLNK(mode) or stat.S_ISREG(mode):
        root.unlink()
        return
    if not stat.S_ISDIR(mode):
        raise UninstallSafetyError(f"지원하지 않는 managed path 유형: {root}")
    for entry in os.scandir(root):
        path = Path(entry.path)
        if _is_protected(path, protected):
            continue
        if entry.is_dir(follow_symlinks=False):
            _remove_tree(path, protected=protected)
        else:
            path.unlink()
    _rmdir_if_empty(root)


def _write_tombstone_temporary(
    destination: Path,
    payload: Mapping[str, object],
) -> Path:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=destination.parent,
        prefix=f".{destination.name}.",
        delete=False,
    ) as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
        temporary = Path(handle.name)
    temporary.chmod(0o600)
    return temporary


def _uninstall_state_root() -> Path:
    configured = os.environ.get("XDG_STATE_HOME", "").strip()
    base = Path(configured).expanduser() if configured else Path.home() / ".local/state"
    return _canonical(base / "elesim/uninstall")


def _rmdir_if_empty(path: Path) -> None:
    try:
        path.rmdir()
    except FileNotFoundError:
        return
    except OSError:
        # A non-empty directory contains data not covered by the manifest and
        # is deliberately preserved.
        return


def _ensure_no_symlink_ancestors(path: Path, *, boundary: Path) -> None:
    if not _within_or_equal(path, boundary):
        raise UninstallSafetyError(f"경로가 검증 boundary 밖입니다: {path}")
    current = path.parent
    while _within_or_equal(current, boundary):
        if _lexists(current) and current.is_symlink():
            raise UninstallSafetyError(f"경로 ancestor가 symlink입니다: {current}")
        if current == boundary:
            break
        current = current.parent


def _minimal_roots(paths: Sequence[Path]) -> tuple[Path, ...]:
    ordered = sorted({_canonical(path) for path in paths}, key=lambda path: len(path.parts))
    result: list[Path] = []
    for path in ordered:
        if not any(_within_or_equal(path, parent) for parent in result):
            result.append(path)
    return tuple(result)


def _is_protected(path: Path, roots: Sequence[Path]) -> bool:
    return any(_within_or_equal(path, root) for root in roots)


def _within_or_equal(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _canonical(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path.expanduser())))


def _lexists(path: Path) -> bool:
    return os.path.lexists(path)


def _key_values(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            result[key] = value
    return result


def _command_runner(runner: CommandRunner | None) -> CommandRunner:
    if runner is not None:
        return runner

    def run(command: Sequence[str]) -> subprocess.CompletedProcess[str]:
        try:
            return subprocess.run(
                tuple(command),
                check=False,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as exc:
            return subprocess.CompletedProcess(
                tuple(command),
                127,
                stdout="",
                stderr=str(exc),
            )

    return run


def _require_command(
    result: subprocess.CompletedProcess[str],
    *,
    action: str,
) -> None:
    if result.returncode != 0:
        raise UninstallSafetyError(f"{action} 실패: {result.stderr.strip()}")


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DockerObject",
    "UninstallPlan",
    "UninstallSafetyError",
    "execute_uninstall",
    "main",
    "plan_uninstall",
    "render_plan",
    "validate_docker_ownership",
]
