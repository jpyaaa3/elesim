"""Read-only collection using existing install ownership and saved SSH pinning."""
from __future__ import annotations

import json
import os
from pathlib import Path, PurePosixPath
import subprocess


def fetch(*, prefix: Path, manager: str, topology: Path, host: str,
          archive: Path, destination: Path) -> Path:
    if not archive.is_absolute() or ".." in PurePosixPath(str(archive)).parts:
        raise ValueError("remote archive must be an absolute, normalized path")
    prefix = prefix.expanduser().resolve()
    owner = json.loads((prefix / "install-ownership.json").read_text())
    if Path(owner["prefix_realpath"]).resolve() != prefix:
        raise ValueError("installation ownership prefix mismatch")
    topology = topology.expanduser().resolve()
    if not topology.is_relative_to(prefix / "connections"):
        raise ValueError("use a saved topology from the selected installation")
    hosts = [item for item in json.loads(topology.read_text())["hosts"] if item["id"] == host]
    if len(hosts) != 1:
        raise ValueError("expected one host in the saved topology")
    endpoint = hosts[0]["ssh"]
    if not endpoint.get("pinned_fingerprint"):
        raise ValueError("SSH host has no saved fingerprint")
    pin = owner["docker"]
    if not pin.get("context") or not pin.get("engine_id"):
        raise ValueError("installation has no pinned Docker daemon")
    docker = ["docker", "--context", pin["context"]]
    env = os.environ.copy()
    env.pop("DOCKER_HOST", None)

    def read(args):
        return subprocess.run([*docker, *args], env=env, capture_output=True,
                              text=True, timeout=30, check=True).stdout
    if read(["info", "--format", "{{.ID}}"]).strip() != pin["engine_id"]:
        raise ValueError("Docker Engine differs from the installation pin")
    info = json.loads(read(["container", "inspect", manager]))[0]
    labels = info["Config"].get("Labels", {})
    if (labels.get("io.elesim.install_uuid") != owner["install_uuid"] or
            labels.get("com.docker.compose.project") != pin["project"] or not info["State"]["Running"]):
        raise ValueError("manager is not a running container owned by this installation")
    request = {"endpoint": endpoint, "archive": str(archive)}
    helper = Path(__file__).with_name("remote_fetch.py").read_text()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        subprocess.run([*docker, "exec", "-i", info["Id"], "python3", "-c", helper], env=env,
                       input=json.dumps(request).encode(), stdout=stream, timeout=300, check=True)
    return destination
