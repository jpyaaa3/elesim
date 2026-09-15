"""Host-only collection of unreferenced, install-owned release images."""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from .ownership import OwnershipManifest
from .releases import list_releases, release_key, _reject_symlinked_ancestors
from .operation_lock import _safe_lock


_ID = re.compile(r"sha256:[0-9a-f]{64}\Z")
_KEY = re.compile(r"[0-9a-f]{64}\Z")
_SYSTEM = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_NAMED_IMAGE = re.compile(
    r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:([a-z]{2,16}_[a-z]{2,16})-([a-z]{2,16}_[a-z]{2,16})$"
)


def _docker(context: str, *args: str) -> str:
    result = subprocess.run(("docker", "--context", context, *args),
                            capture_output=True, text=True, check=True)
    return result.stdout


def collect(prefix: Path, *, docker=_docker) -> tuple[str, ...]:
    """Caller holds the installation lock. Never delete by prefix or prune.

    Release manifests remain historical provenance, not a promise that an
    unregistered historical release's Docker image remains installed.
    """
    _reject_symlinked_ancestors(prefix)
    owner = OwnershipManifest.load(prefix / "install-ownership.json")
    boundary = owner.docker
    if owner.prefix_path != prefix or boundary is None:
        raise ValueError("image cleanup requires an owned container installation")
    project = boundary.project
    if project == "elesim-runtime" or not boundary.context or not boundary.engine_id:
        raise ValueError("image cleanup requires a pinned, install-scoped Docker backend")
    call = lambda *args: docker(boundary.context, *args)
    if call("info", "--format", "{{.ID}}").strip() != boundary.engine_id:
        raise ValueError("Docker Engine identity changed; refusing image cleanup")

    releases = {release_key(item): item for item in list_releases(prefix, install_uuid=owner.install_uuid)}
    _reject_symlinked_ancestors(Path(boundary.compose_file))
    current_tags = set(call("compose", "-p", project, "-f", boundary.compose_file,
                            "config", "--images").splitlines())
    current_ids = {}
    runtime_tags = {tag for item in releases.values() for tag in item.role_images.values()}
    for tag in current_tags & runtime_tags:
        current_ids[tag] = json.loads(call("image", "inspect", tag))[0]["Id"]
    current_releases = [item for item in releases.values()
                        if all(current_ids.get(tag) == item.image_ids[role]
                               for role, tag in item.role_images.items())]
    if not current_releases:
        raise ValueError("current build has no published release; refusing image cleanup")
    protected = {value for item in current_releases for value in item.image_ids.values()}
    instances = prefix / "instances"
    _reject_symlinked_ancestors(instances)
    for child in instances.iterdir():
        _reject_symlinked_ancestors(child)
        if child.name == ".locks":
            if not child.is_dir():
                raise ValueError("instance lock directory is invalid")
            # A managed transaction lease may span multiple host operations.
            # Conservatively defer all collection while one exists.
            if any(path.suffix in {".json", ".lease"} for path in child.iterdir()):
                raise ValueError("active instance transaction; image cleanup deferred")
            continue
        if not _SYSTEM.fullmatch(child.name) or not child.is_dir():
            raise ValueError("unknown instance registry entry; refusing image cleanup")
        state_path = child / "state.json"
        _reject_symlinked_ancestors(state_path)
        if not state_path.is_file() or state_path.stat().st_size > 1024 * 1024:
            raise ValueError("invalid instance state; refusing image cleanup")
        state = json.loads(state_path.read_text())
        if not isinstance(state, dict):
            raise ValueError("instance state must be an object")
        key = state.get("release_key")
        if (state.get("schema_version") not in (2, 3) or state.get("system_id") != child.name
                or not isinstance(key, str) or not _KEY.fullmatch(key) or key not in releases):
            raise ValueError("invalid instance release pin; refusing image cleanup")
        protected.update(releases[key].image_ids.values())

    # Inspect all containers, including stopped/foreign ones; label filtering
    # here would miss a different project consuming one of our images.
    for container in call("ps", "-aq", "--no-trunc").splitlines():
        if not _KEY.fullmatch(container):
            raise ValueError("invalid Docker container identity")
        records = json.loads(call("container", "inspect", container))
        protected.add(records[0]["Image"])

    owned_tags = set(boundary.local_images)
    release_ids = {image for item in releases.values() for image in item.image_ids.values()}
    candidates = set(release_ids)
    # Include old tools/dev images, but only with both inventory and labels.
    for image in call("image", "ls", "--quiet", "--no-trunc", "--filter",
                      f"label=io.elesim.install_uuid={owner.install_uuid}").splitlines():
        if not _ID.fullmatch(image):
            raise ValueError("invalid Docker image identity")
        candidates.add(image)
    available = set(call("image", "ls", "--quiet", "--no-trunc").splitlines())
    plans = []
    for image in sorted(candidates & available):
        records = json.loads(call("image", "inspect", image))
        record = records[0]
        labels = record.get("Config", {}).get("Labels") or {}
        tags = set(record.get("RepoTags") or ())
        if record.get("Id") != image:
            raise ValueError("Docker image identity changed")
        if image in protected or tags & current_tags:
            continue
        if not tags and image not in release_ids:
            continue  # An intermediate/cache image is not a published release.
        if (labels.get("io.elesim.install_uuid") != owner.install_uuid
                or labels.get("com.docker.compose.project") != project):
            raise ValueError("release image ownership mismatch; refusing cleanup")
        fingerprint = labels.get("io.elesim.build_fingerprint", "")
        if not isinstance(fingerprint, str) or not _KEY.fullmatch(fingerprint):
            raise ValueError("release image fingerprint is invalid")
        expected_old = re.compile(
            r"elesim/[a-z][a-z0-9_-]*:"
            + owner.install_uuid.replace("-", "")
            + "-"
            + fingerprint
            + r"\Z"
        )
        expected_named = re.compile(
            r"elesim/[a-z][a-z0-9_.-]*:"
            + re.escape(owner.docker.install_name)
            + r"-[a-z]{2,16}_[a-z]{2,16}\Z"
        ) if owner.docker.install_name else None
        if (record.get("RepoDigests") or len(tags) > 1 or not tags <= owned_tags
                or any(
                    not expected_old.fullmatch(tag)
                    and (expected_named is None or not expected_named.fullmatch(tag))
                    for tag in tags
                )):
            continue  # Preserve additional tags, including foreign aliases.
        plans.append((image, record))
    # No mutation until every registry/provenance check has completed. Docker's
    # non-force removal is the final fence against a newly created container.
    removed = []
    for image, expected in plans:
        if json.loads(call("image", "inspect", image))[0] != expected:
            raise ValueError("Docker image metadata changed before cleanup")
        call("image", "rm", image)
        removed.append(image)
        print(f"[cleanup] removed unreferenced image: {image}", flush=True)
    return tuple(removed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", type=Path, required=True)
    parser.add_argument("--lock-fd", type=int)
    args = parser.parse_args()
    prefix = Path(os.path.abspath(args.prefix.expanduser()))
    lock = prefix / "instances/.locks/install.lock"
    fd = None
    try:
        if args.lock_fd is not None:
            _reject_symlinked_ancestors(lock)
            actual, expected = os.fstat(args.lock_fd), lock.stat()
            if (actual.st_dev, actual.st_ino) != (expected.st_dev, expected.st_ino):
                raise ValueError("installation lock identity mismatch")
            fcntl.flock(args.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        else:
            fd = _safe_lock(lock)
        removed = collect(prefix)
        print(f"[cleanup] reclaimed {len(removed)} unreferenced image(s).")
        return 0
    except (OSError, ValueError, KeyError, TypeError, IndexError, subprocess.CalledProcessError) as exc:
        detail = str(exc)
        if isinstance(exc, subprocess.CalledProcessError) and exc.stderr:
            detail += ": " + exc.stderr.strip()[-600:]
        print(f"Image cleanup failed; the completed update/start was not rolled back: {detail}", file=sys.stderr)
        return 1
    finally:
        if fd is not None:
            os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
