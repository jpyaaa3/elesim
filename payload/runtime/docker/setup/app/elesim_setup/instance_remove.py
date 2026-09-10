"""Host-only removal bridge for one scoped release instance.

The setup image deliberately has no Docker socket.  This small stdlib-only
bridge runs from the generated host dispatcher, holds the same install and
system locks as :class:`InstanceRuntime`, removes only exact containers whose
labels prove ownership, and then asks the tools image to commit the registry
transaction under a one-shot lease.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import secrets
import stat
import subprocess
import sys
from contextlib import ExitStack, contextmanager
from pathlib import Path
from typing import Any, Iterator, Mapping

from .instance_identity import container_name, project_name, service_key


_SYSTEM = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_ENDPOINT = re.compile(r"^[a-z][a-z0-9_-]{0,62}$")
_ROLES = frozenset({"pilot", "sim", "ui"})
_DOCKER_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


class InstanceRemovalError(RuntimeError):
    pass


def _lexical(path: Path) -> Path:
    return Path(os.path.abspath(os.path.expanduser(os.fspath(path))))


def _check_tree(path: Path, *, allow_missing: bool = True) -> None:
    path = _lexical(path)
    current = path
    while True:
        if current.is_symlink():
            raise InstanceRemovalError(f"refusing symlink path: {path}")
        if current == current.parent:
            break
        current = current.parent
    if not allow_missing and not path.exists():
        raise FileNotFoundError(path)


@contextmanager
def _open_lock(path: Path) -> Iterator[None]:
    _check_tree(path)
    if path.is_symlink():
        raise InstanceRemovalError(f"refusing symlink lock: {path}")
    try:
        fd = os.open(
            path,
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC,
            0o600,
        )
    except OSError as exc:
        raise InstanceRemovalError(f"cannot open instance lock: {path}") from exc
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise InstanceRemovalError(f"instance lock is not a singly-linked file: {path}")
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


@contextmanager
def _lease(prefix: Path, system: str, install_uuid: str) -> Iterator[str]:
    """Hold install/system locks and publish a private cross-container lease."""

    root = prefix / "instances"
    lock_root = root / ".locks"
    if lock_root.is_symlink() or not lock_root.is_dir():
        raise InstanceRemovalError("instance lock metadata is unavailable")
    install_lock = lock_root / "install.lock"
    system_lock = lock_root / f"{system}.lock"
    lease_path = lock_root / f"{system}.lease"
    with ExitStack() as stack:
        stack.enter_context(_open_lock(install_lock))
        stack.enter_context(_open_lock(system_lock))
        if lease_path.is_symlink() or lease_path.exists():
            raise InstanceRemovalError("an instance removal lease is already present")
        token = secrets.token_hex(32)
        payload = {
            "version": 1,
            "token": token,
            "system_id": system,
            "install_uuid": install_uuid,
        }
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW | os.O_CLOEXEC
        try:
            fd = os.open(lease_path, flags, 0o600)
        except OSError as exc:
            raise InstanceRemovalError("cannot create instance removal lease") from exc
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise InstanceRemovalError("instance removal lease is not a regular file")
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                fd = -1
                json.dump(payload, stream, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            lease_identity = (info.st_dev, info.st_ino)
            try:
                yield token
            finally:
                try:
                    current = lease_path.lstat()
                    if (current.st_dev, current.st_ino) == lease_identity:
                        lease_path.unlink()
                except FileNotFoundError:
                    pass
        finally:
            if fd != -1:
                os.close(fd)


def _docker_command(context: str, *arguments: str) -> tuple[str, ...]:
    if not isinstance(context, str) or not context or "\x00" in context or "\n" in context:
        raise InstanceRemovalError("Docker context is invalid")
    if not _DOCKER_NAME.fullmatch(context):
        raise InstanceRemovalError("Docker context is invalid")
    return ("docker", "--context", context, *arguments)


def _run(command: tuple[str, ...]) -> subprocess.CompletedProcess[str]:
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "").strip()
        raise InstanceRemovalError(
            "Docker instance removal operation failed"
            + (f": {detail[-400:]}" if detail else "")
        )
    return completed


def _inspect(context: str, name: str) -> Mapping[str, Any] | None:
    result = subprocess.run(
        _docker_command(context, "container", "inspect", "--format", "{{json .}}", name),
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode == 1 and not result.stdout.strip() and (
        not result.stderr.strip()
        or "no such object" in result.stderr.casefold()
        or "not found" in result.stderr.casefold()
    ):
        return None
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        raise InstanceRemovalError(
            "Docker container inspection failed"
            + (f": {detail[-400:]}" if detail else "")
        )
    try:
        payload = json.loads(result.stdout.strip())
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise InstanceRemovalError("Docker container inspection returned malformed JSON") from exc
    if not isinstance(payload, Mapping):
        raise InstanceRemovalError("Docker container inspection returned a non-object")
    return payload


def _read_target(prefix: Path, system: str) -> tuple[dict[str, str], ...]:
    state_path = prefix / "instances" / system / "state.json"
    _check_tree(state_path, allow_missing=False)
    if state_path.is_symlink() or not state_path.is_file():
        raise InstanceRemovalError("instance state is not a regular file")
    try:
        value = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise InstanceRemovalError("instance state is malformed") from exc
    if not isinstance(value, Mapping) or value.get("system_id") != system:
        raise InstanceRemovalError("instance state does not match the target system")
    endpoints = value.get("endpoints")
    if not isinstance(endpoints, list) or not endpoints:
        raise InstanceRemovalError("instance state has no endpoints")
    result: list[dict[str, str]] = []
    seen: set[str] = set()
    for endpoint in endpoints:
        if not isinstance(endpoint, Mapping):
            raise InstanceRemovalError("instance endpoint is malformed")
        role, endpoint_id = endpoint.get("role"), endpoint.get("endpoint_id")
        if (
            not isinstance(role, str)
            or role not in _ROLES
            or not isinstance(endpoint_id, str)
            or _ENDPOINT.fullmatch(endpoint_id) is None
        ):
            raise InstanceRemovalError("instance endpoint identity is malformed")
        service = service_key(system, endpoint_id)
        if service in seen:
            raise InstanceRemovalError("instance endpoint identity is duplicated")
        seen.add(service)
        result.append({"service": service, "role": role, "endpoint_id": endpoint_id})
    turn_raw = value.get("turn", {})
    if not isinstance(turn_raw, Mapping):
        raise InstanceRemovalError("instance TURN state is malformed")
    turn_mode = turn_raw.get("mode", "none")
    if not isinstance(turn_mode, str) or turn_mode not in {"none", "managed", "external"}:
        raise InstanceRemovalError("instance TURN mode is invalid")
    if turn_mode == "managed":
        if not any(item["role"] == "sim" for item in result):
            raise InstanceRemovalError("managed TURN requires a Sim endpoint")
        result.append(
            {
                "service": service_key(system, "coturn"),
                "role": "sim",
                "endpoint_id": "coturn",
            }
        )
    return tuple(result)


def _expected_container(install_uuid: str, service: str) -> str:
    return container_name(install_uuid, service)


def _verify_manifest(prefix: Path, install_uuid: str, project: str, context: str, engine: str, names: set[str]) -> None:
    path = prefix / "install-ownership.json"
    _check_tree(path, allow_missing=False)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise InstanceRemovalError("ownership manifest is malformed") from exc
    docker = payload.get("docker") if isinstance(payload, Mapping) else None
    if not isinstance(payload, Mapping) or not isinstance(docker, Mapping):
        raise InstanceRemovalError("ownership manifest has no Docker ownership")
    if (
        payload.get("prefix") != str(prefix)
        or payload.get("install_uuid") != install_uuid
        or docker.get("install_uuid") != install_uuid
        or docker.get("project") != project
        or docker.get("context") != context
        or docker.get("engine_id") != engine
    ):
        raise InstanceRemovalError("ownership manifest does not match this Docker backend")
    recorded = docker.get("containers")
    if not isinstance(recorded, list) or not names.issubset(set(recorded)):
        raise InstanceRemovalError("target containers are not recorded as install-owned")


def _verify_and_remove_containers(
    *, prefix: Path, system: str, install_uuid: str, project: str, context: str, engine: str,
    compose: Path, target: tuple[dict[str, str], ...],
) -> None:
    actual_engine = _run(_docker_command(context, "info", "--format", "{{.ID}}"))
    if actual_engine.stdout.strip() != engine:
        raise InstanceRemovalError("Docker Engine does not match the installed backend")
    names = {_expected_container(install_uuid, item["service"]) for item in target}
    _verify_manifest(prefix, install_uuid, project, context, engine, names)
    existing: list[str] = []
    aggregate = str(_lexical(compose))
    for item in target:
        service, name = item["service"], _expected_container(install_uuid, item["service"])
        inspected = _inspect(context, name)
        if inspected is None:
            continue
        actual_name = inspected.get("Name")
        config = inspected.get("Config")
        labels = config.get("Labels") if isinstance(config, Mapping) else None
        if actual_name != f"/{name}" or not isinstance(labels, Mapping):
            raise InstanceRemovalError(f"target container identity is not exact: {name}")
        if (
            labels.get("io.elesim.install_uuid") != install_uuid
            or labels.get("com.docker.compose.project") != project
            or labels.get("com.docker.compose.service") != service
            or labels.get("io.elesim.system_id") != system
            or labels.get("io.elesim.endpoint_id") != item["endpoint_id"]
            or labels.get("io.elesim.role") != item["role"]
            or (
                item["endpoint_id"] == "coturn"
                and labels.get("io.elesim.service_kind") != "coturn"
            )
            or aggregate
            not in {
                part.strip()
                for part in str(labels.get("com.docker.compose.project.config_files", "")).split(",")
            }
        ):
            raise InstanceRemovalError(f"target container is foreign or not instance-owned: {name}")
        existing.append(name)
    if existing:
        _run(_docker_command(context, "container", "rm", "-f", *existing))
        for name in existing:
            if _inspect(context, name) is not None:
                raise InstanceRemovalError(f"target container remains after removal: {name}")


def remove_instance(
    *, prefix: Path, state: Path, system: str, install_uuid: str,
    project: str, context: str, engine: str, compose: Path,
) -> int:
    prefix, state, compose = map(_lexical, (prefix, state, compose))
    if _SYSTEM.fullmatch(system) is None:
        raise InstanceRemovalError("system ID is invalid")
    if prefix == Path("/") or project != project_name(install_uuid):
        raise InstanceRemovalError("Docker project is not the scoped install project")
    _check_tree(prefix, allow_missing=False)
    _check_tree(state, allow_missing=False)
    _check_tree(compose, allow_missing=False)
    if state.is_symlink() or compose.is_symlink() or not compose.is_file():
        raise InstanceRemovalError("install state or Compose manifest is unsafe")
    aggregate = prefix / "containers" / "compose.instances.yaml"
    _check_tree(aggregate, allow_missing=False)
    if aggregate.is_symlink() or not aggregate.is_file():
        raise InstanceRemovalError("instance Compose manifest is unsafe")
    target = _read_target(prefix, system)
    with _lease(prefix, system, install_uuid) as token:
        _verify_and_remove_containers(
            prefix=prefix, system=system, install_uuid=install_uuid, project=project,
            context=context, engine=engine, compose=aggregate,
            target=target,
        )
        command = _docker_command(
            context, "compose", "--project-name", project, "--file", str(compose),
            "run", "--rm", "--no-deps", "--no-build", "tools", "elesim-setup",
            "--state", str(state), "instance", "remove", "--system", system,
            "--host-lease", token,
        )
        _run(command)
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="elesim-instance-remove")
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--system-id", required=True)
    parser.add_argument("--install-uuid", required=True)
    parser.add_argument("--project", required=True)
    parser.add_argument("--docker-context", required=True)
    parser.add_argument("--docker-engine-id", required=True)
    parser.add_argument("--compose", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        args = _parser().parse_args(argv)
        return remove_instance(
            prefix=args.prefix, state=args.state, system=args.system_id,
            install_uuid=args.install_uuid, project=args.project,
            context=args.docker_context, engine=args.docker_engine_id,
            compose=args.compose,
        )
    except (InstanceRemovalError, OSError, ValueError) as exc:
        print(f"instance removal refused: {exc}", file=sys.stderr)
        return 78


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
