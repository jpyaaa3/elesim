"""Host-only collection of unreferenced, install-owned release tags and images."""
from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from .ownership import OwnershipManifest
from .releases import ReleaseManifest, list_releases, release_key, _reject_symlinked_ancestors
from .operation_lock import _safe_lock
from .readable_names import reclaim_unused_image_names


_ID = re.compile(r"sha256:[0-9a-f]{64}\Z")
_KEY = re.compile(r"[0-9a-f]{64}\Z")
_SYSTEM = re.compile(r"[a-z][a-z0-9_]{0,62}\Z")
_NAMED_IMAGE = re.compile(
    r"^elesim/[a-z0-9][a-z0-9_.-]{0,127}:([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))-([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))$"
)


def _has_only_self_repo_digests(
    image_id: str,
    tags: set[str],
    repo_digests: object,
) -> bool:
    """Accept Docker's self-referential local digest, but no foreign one.

    Recent Docker/BuildKit daemons can attach ``repository@sha256:<image-id>``
    to a locally loaded image.  That is not an external ownership reference:
    it is another spelling of the exact local image.  A digest for another
    repository, or a digest that does not equal the inspected image ID, still
    means the cleanup boundary cannot prove that removing this image is safe.
    """

    if not repo_digests:
        return True
    if not isinstance(repo_digests, list) or not all(
        isinstance(value, str) for value in repo_digests
    ):
        return False
    image_digest = image_id.removeprefix("sha256:")
    repositories = {
        tag.rsplit(":", 1)[0]
        for tag in tags
        if ":" in tag
    }
    if not repositories:
        return False
    expected_digest = f"sha256:{image_digest}"
    for value in repo_digests:
        repository, separator, digest = value.rpartition("@")
        if (
            separator != "@"
            or not repository
            or digest != expected_digest
            or repository not in repositories
        ):
            return False
    return True


def _docker(context: str, *args: str) -> str:
    result = subprocess.run(("docker", "--context", context, *args),
                            capture_output=True, text=True, check=True)
    return result.stdout


def _container_references(call) -> dict[str, tuple[str, str | None]]:
    """Keep image identity separate from the name used to create a container."""
    references = {}
    for container in call("ps", "-aq", "--no-trunc").splitlines():
        if not _KEY.fullmatch(container):
            raise ValueError("invalid Docker container identity")
        records = json.loads(call("container", "inspect", container))
        record = records[0]
        image = record["Image"]
        if not isinstance(image, str) or not _ID.fullmatch(image):
            raise ValueError("invalid container image identity")
        config = record.get("Config") or {}
        reference = config.get("Image")
        if reference is not None and not isinstance(reference, str):
            raise ValueError("invalid container image reference")
        references[container] = (image, reference or None)
    return references


def available_releases(prefix: Path, *, docker=_docker) -> tuple[ReleaseManifest, ...]:
    """Return only releases whose exact role tags still name their image IDs."""
    _reject_symlinked_ancestors(prefix)
    owner = OwnershipManifest.load(prefix / "install-ownership.json")
    boundary = owner.docker
    if owner.prefix_path != prefix or boundary is None or not boundary.context or not boundary.engine_id:
        raise ValueError("available releases require an owned, pinned Docker installation")
    call = lambda *args: docker(boundary.context, *args)
    if call("info", "--format", "{{.ID}}").strip() != boundary.engine_id:
        raise ValueError("Docker Engine identity changed; refusing release lookup")
    found = []
    for release in list_releases(prefix, install_uuid=owner.install_uuid):
        for role, tag in release.role_images.items():
            try:
                records = json.loads(call("image", "inspect", tag))
            except subprocess.CalledProcessError as exc:
                if "No such image" not in (exc.stderr or "") and "No such object" not in (exc.stderr or ""):
                    raise
                break
            if (not isinstance(records, list) or len(records) != 1
                    or records[0].get("Id") != release.image_ids[role]):
                break
        else:
            found.append(release)
    return tuple(found)


def recorded_available_releases(prefix: Path) -> tuple[ReleaseManifest, ...]:
    """Read the host cleanup result from a GUI without a Docker socket."""
    _reject_symlinked_ancestors(prefix)
    owner = OwnershipManifest.load(prefix / "install-ownership.json")
    releases = list_releases(prefix, install_uuid=owner.install_uuid)
    path = prefix / "maintenance/available-releases.json"
    _reject_symlinked_ancestors(path)
    if not path.exists():
        return releases  # Older installations have no host-produced index yet.
    if not path.is_file() or path.stat().st_size > 1024 * 1024:
        raise ValueError("invalid available release index")
    data = json.loads(path.read_text())
    expected_engine = owner.docker.engine_id if owner.docker is not None else ""
    if (not isinstance(data, dict) or set(data) != {"schema_version", "install_uuid", "engine_id", "release_keys"}
            or type(data["schema_version"]) is not int or data["schema_version"] != 1
            or data["install_uuid"] != owner.install_uuid
            or data["engine_id"] != expected_engine or not isinstance(data["release_keys"], list)
            or not all(isinstance(key, str) and _KEY.fullmatch(key) for key in data["release_keys"])):
        raise ValueError("invalid available release index")
    keys = set(data["release_keys"])
    return tuple(item for item in releases if release_key(item) in keys)


def _record_available_releases(prefix: Path, releases: tuple[ReleaseManifest, ...], owner: OwnershipManifest) -> None:
    path = prefix / "maintenance/available-releases.json"
    _reject_symlinked_ancestors(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not path.is_file():
        raise ValueError("invalid available release index")
    payload = {
        "schema_version": 1,
        "install_uuid": owner.install_uuid,
        "engine_id": owner.docker.engine_id if owner.docker is not None else "",
        "release_keys": [release_key(item) for item in releases],
    }
    fd, temporary = tempfile.mkstemp(prefix=".available-releases-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            os.fchmod(stream.fileno(), 0o600)
            json.dump(payload, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def collect(prefix: Path, *, docker=_docker) -> tuple[str, ...]:
    """Caller holds the installation lock. Never delete by prefix or prune.

    Release manifests remain historical provenance, not a promise that an
    unregistered historical release's Docker image or tag remains installed.
    Return fully removed image IDs; retained images may lose unused tags.
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
    protected_tags = set(current_tags)
    pinned_release_keys = set()
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
        pinned_release_keys.add(key)
        protected.update(releases[key].image_ids.values())
        protected_tags.update(releases[key].role_images.values())

    # Inspect all containers, including stopped/foreign ones; label filtering
    # here would miss a different project consuming one of our images.
    container_references = _container_references(call)
    unknown_references = set()
    for image, reference in container_references.values():
        protected.add(image)
        if reference is None:
            unknown_references.add(image)
        else:
            protected_tags.add(reference)
            if ":" not in reference.rsplit("/", 1)[-1] and "@" not in reference:
                protected_tags.add(reference + ":latest")

    owned_tags = set(boundary.local_images)
    release_ids = {image for item in releases.values() for image in item.image_ids.values()}
    candidates = set(release_ids)
    # Include old tools images, but only with both inventory and labels.
    # Dev images may outlive a runtime update and are never auto-collected.
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
        if any(tag.startswith("elesim/dev:") for tag in tags):
            continue
        retained = image in protected or bool(tags & protected_tags)
        kept_tags = tags & protected_tags
        if image in unknown_references:
            kept_tags = set(tags)  # Incomplete container metadata cannot prove a tag is unused.
        elif retained and tags and not kept_tags:
            # Containers created by ID need the image, not every historical
            # name. Keep one stable anchor instead of removing its final tag.
            kept_tags.add(min(tags))
        references = tuple(sorted(tags - kept_tags)) if tags else (() if retained else (image,))
        if not references:
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
            + r"-[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6})\Z"
        ) if owner.docker.install_name else None
        if (
            not _has_only_self_repo_digests(
                record.get("Id", ""), tags, record.get("RepoDigests")
            )
            or not tags <= owned_tags
            or any(
                not expected_old.fullmatch(tag)
                and (expected_named is None or not expected_named.fullmatch(tag))
                for tag in tags
            )
        ):
            continue  # Preserve foreign or otherwise unowned aliases.
        plans.append((image, record, references, retained))
    # No mutation until every registry/provenance check has completed. Docker's
    # non-force removal is the final fence against a newly created container.
    if plans and _container_references(call) != container_references:
        raise ValueError("Docker container references changed before cleanup")
    removed = []
    for image, expected, references, retained in plans:
        if json.loads(call("image", "inspect", image))[0] != expected:
            raise ValueError("Docker image metadata changed before cleanup")
        # Remove exact validated tags without force. A retained image keeps
        # its referenced names or one anchor; only an unprotected image is
        # allowed to lose its final tag and image data.
        for reference in references:
            call("image", "rm", reference)
            if retained:
                print(f"[cleanup] removed unreferenced tag: {reference}", flush=True)
        if not retained:
            removed.append(image)
            print(f"[cleanup] removed unreferenced image: {image}", flush=True)
    if boundary.install_name:
        # Query the daemon after removals. A historical manifest is not an
        # ownership claim on a tag that no longer exists; the pinned instance
        # and all container references were already protected above.
        live_tags = set(call("image", "ls", "--format", "{{.Repository}}:{{.Tag}}").splitlines())
        prefix_tag = f"{boundary.install_name}-"
        live_aliases = {
            tag.rsplit("-", 1)[-1]
            for tag in live_tags
            if tag.startswith("elesim/") and tag.partition(":")[2].startswith(prefix_tag)
        }
        live_aliases.update(
            tag.rsplit("-", 1)[-1]
            for key in pinned_release_keys
            for tag in releases[key].role_images.values()
            if tag.startswith("elesim/") and tag.partition(":")[2].startswith(prefix_tag)
        )
        reclaimed = reclaim_unused_image_names(prefix / "containers/image-names.json", live_aliases)
        for alias in reclaimed:
            print(f"[cleanup] reclaimed image alias: {alias}", flush=True)
    _record_available_releases(prefix, available_releases(prefix, docker=docker), owner)
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
