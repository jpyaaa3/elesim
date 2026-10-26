"""List registered call signs and optionally inspect the pinned Docker runtime."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
from typing import Sequence

from .instance_identity import instance_container_name, is_scoped_project, named_image_parts, service_key
from .ownership import OwnershipManifest
from .releases import load_release, _reject_symlinked_ancestors


_ROLES = ("pilot", "sim", "ui")
_SYSTEM = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_ENDPOINT = re.compile(r"[a-z][a-z0-9_-]{0,62}\Z")
_KEY = re.compile(r"[0-9a-f]{64}\Z")
_MAX_STATE_BYTES = 1024 * 1024
_DOCKER_TIMEOUT_S = 2.0
_ACTIVE_COLOR = "\033[38;2;94;234;212m"
_COLOR_RESET = "\033[0m"


def _running_containers(owner: OwnershipManifest) -> set[tuple[str, str, str, str]] | None:
    """Return owned running endpoint identities, or None when status is unknown."""
    boundary = owner.docker
    if boundary is None or not boundary.context or not boundary.engine_id:
        return None
    command = ("docker", "--context", boundary.context)
    environment = dict(os.environ)
    environment.pop("DOCKER_HOST", None)
    environment["DOCKER_CONTEXT"] = boundary.context

    def query(*arguments: str) -> str:
        return subprocess.run(
            (*command, *arguments), capture_output=True, text=True, check=True,
            timeout=_DOCKER_TIMEOUT_S, env=environment,
        ).stdout

    fields = (".Names", ".State", '(.Label "io.elesim.install_uuid")',
              '(.Label "com.docker.compose.project")', '(.Label "com.docker.compose.service")',
              '(.Label "io.elesim.system_id")', '(.Label "io.elesim.role")',
              '(.Label "com.docker.compose.oneoff")')
    output_format = "[" + ",".join("{{json " + field + "}}" for field in fields) + "]"
    try:
        if query("info", "--format", "{{.ID}}").strip() != boundary.engine_id:
            return None
        output = query(
            "container", "ls", "--filter", "status=running",
            "--filter", f"label=io.elesim.install_uuid={owner.install_uuid}",
            "--filter", f"label=com.docker.compose.project={boundary.project}",
            "--format", output_format,
        )
        running = set()
        for line in output.splitlines():
            values = json.loads(line)
            if not isinstance(values, list) or len(values) != 8 or not all(isinstance(v, str) for v in values):
                return None
            name, state, install, project, service, system, role, oneoff = values
            if (state == "running" and install == owner.install_uuid
                    and project == boundary.project and oneoff.lower() != "true"):
                running.add((name, service, system, role))
        return running
    except (OSError, subprocess.SubprocessError, ValueError):
        # Listing remains usable with Docker stopped, missing or inaccessible.
        # Unknown is distinct from an observed empty set of running containers.
        return None


def _read_state(path: Path) -> dict:
    _reject_symlinked_ancestors(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise ValueError(f"instance state must be a regular, singly-linked file: {path}")
        if info.st_size > _MAX_STATE_BYTES:
            raise ValueError(f"instance state exceeds the maximum size: {path}")
        with os.fdopen(fd, "rb") as stream:
            fd = -1
            raw = stream.read(_MAX_STATE_BYTES + 1)
        if len(raw) > _MAX_STATE_BYTES:
            raise ValueError(f"instance state exceeds the maximum size: {path}")
    finally:
        if fd != -1:
            os.close(fd)
    value = json.loads(raw)
    if not isinstance(value, dict) or type(value.get("schema_version")) is not int or value["schema_version"] not in (2, 3):
        raise ValueError(f"unsupported instance state: {path}")
    if value.get("system_id") != path.parent.name:
        raise ValueError(f"instance system_id does not match its directory: {path}")
    key = value.get("release_key")
    if not isinstance(key, str) or not _KEY.fullmatch(key):
        raise ValueError(f"invalid instance release pin: {path}")
    endpoints = value.get("endpoints")
    if not isinstance(endpoints, list) or not endpoints:
        raise ValueError(f"instance endpoints must be a non-empty list: {path}")
    roles, ids = set(), set()
    for endpoint in endpoints:
        if not isinstance(endpoint, dict) or set(endpoint) != {"role", "endpoint_id"}:
            raise ValueError(f"invalid instance endpoint: {path}")
        role, identity = endpoint["role"], endpoint["endpoint_id"]
        if not isinstance(role, str) or role not in _ROLES or not isinstance(identity, str) or not _ENDPOINT.fullmatch(identity):
            raise ValueError(f"invalid instance endpoint: {path}")
        if role in roles or identity in ids:
            raise ValueError(f"duplicate instance endpoint role or identity: {path}")
        roles.add(role)
        ids.add(identity)
    return value


def list_instances(prefix: Path, *, include_activity: bool = False) -> tuple[dict, ...]:
    """Read exact local pins, including stopped instances and historical releases.

    Listing takes no lifecycle locks and creates no registry/lock files. Atomic
    registration snapshots are sufficient; concurrent removal may report an
    error, and the operator can repeat the read. include_activity adds a bounded
    read of the pinned Engine; it never starts Docker or changes containers.
    """
    prefix = Path(os.path.abspath(prefix.expanduser()))
    _reject_symlinked_ancestors(prefix)
    owner = OwnershipManifest.load(prefix / "install-ownership.json")
    if owner.prefix_path != prefix:
        raise ValueError("installation ownership prefix mismatch")
    if (owner.docker is None or owner.docker.project == "elesim-runtime"
            or not is_scoped_project(owner.install_uuid, owner.docker.project)):
        raise ValueError("this installation has no scoped instances")
    root = prefix / "instances"
    _reject_symlinked_ancestors(root)
    if not root.exists():
        return ()
    if not root.is_dir():
        raise ValueError("instance registry must be a directory")
    rows, releases, expected = [], {}, {}
    for child in sorted(root.iterdir(), key=lambda path: path.name):
        _reject_symlinked_ancestors(child)
        if not child.is_dir():
            raise ValueError(f"invalid instance directory: {child}")
        if child.name == ".locks":
            continue
        if not _SYSTEM.fullmatch(child.name):
            raise ValueError(f"invalid instance directory: {child}")
        state_path = child / "state.json"
        _reject_symlinked_ancestors(state_path)
        if not state_path.exists():
            retained = tuple(child.iterdir())
            if retained and all(not path.is_symlink() and path.is_dir() and path.name in {"security", "logs", "secrets"} for path in retained):
                continue
            raise ValueError(f"missing instance state: {state_path}")
        state = _read_state(state_path)
        key = state["release_key"]
        if key not in releases:
            release = load_release(prefix / "releases" / key)
            if release.install_uuid != owner.install_uuid:
                raise ValueError(f"{child.name}: pinned release belongs to another installation")
            releases[key] = release
        release = releases[key]
        assigned = {endpoint["role"] for endpoint in state["endpoints"]}
        callsigns = {}
        for role in _ROLES:
            if role not in assigned:
                continue
            if role not in release.role_images:
                raise ValueError(f"{child.name}: pinned release has no {role} image")
            parts = named_image_parts(release.role_images[role], role)
            callsigns[role] = parts[1] if parts else None
        rows.append({"system_id": child.name, "roles": callsigns})
        expected[child.name] = {
            (instance_container_name(owner.install_uuid, child.name, endpoint["role"], endpoint["endpoint_id"],
                                     install_name=owner.docker.install_name, naming=owner.docker.container_naming),
             service_key(child.name, endpoint["endpoint_id"]), child.name, endpoint["role"])
            for endpoint in state["endpoints"]
        }
    if include_activity and rows:
        running = _running_containers(owner)
        for row in rows:
            row["active"] = None if running is None else bool(expected[row["system_id"]] & running)
    return tuple(rows)


def format_instances(rows: Sequence[dict], *, color: bool = False) -> str:
    if not rows:
        return "No registered instances."
    groups = []
    for row in rows:
        active = row.get("active") is True
        lines = [row["system_id"] + (" (active)" if active else "")]
        lines.extend(f"  {role}: {name or '(no call sign)'}" for role, name in row["roles"].items())
        group = "\n".join(lines)
        groups.append(_ACTIVE_COLOR + group + _COLOR_RESET if color and active else group)
    return "\n\n".join(groups)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="elesim list", description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True, help=argparse.SUPPRESS)
    parser.add_argument("--json", action="store_true", help="print instance IDs, role call signs and activity as JSON")
    args = parser.parse_args(argv)
    try:
        rows = list_instances(args.prefix, include_activity=True)
    except (OSError, ValueError) as exc:
        print(f"Unable to list instances: {exc}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps({"instances": rows}))
    else:
        color = sys.stdout.isatty() and os.environ.get("TERM") != "dumb" and "NO_COLOR" not in os.environ
        print(format_instances(rows, color=color))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
