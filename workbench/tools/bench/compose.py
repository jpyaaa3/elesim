"""One-off diagnostics attached to an explicitly selected, installed Compose project.

No builds, pulls, registrations or normal-service lifecycle commands are issued.
The installed wrapper and pinned Engine remain the authority for Docker access.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import time
import uuid

from .catalog import ROOT, source_closure
from .evidence import write_json
from .runner import LocalExecutor


class ComposeExecutor(LocalExecutor):
    def __init__(self, prefix: Path, system: str, *, gpu: str | None = None):
        super().__init__(gpu=gpu)
        self.prefix = prefix.expanduser().resolve()
        self.system = system
        self.token = uuid.uuid4().hex
        self.resources = []
        self.services = {}
        self.validated = False

    def _docker(self, *args, missing_ok=False):
        result = subprocess.run([*self.docker, *args], text=True, capture_output=True, timeout=30,
                                env=self.docker_env, check=False)
        if result.returncode:
            if missing_ok and ("No such" in result.stderr or "not found" in result.stderr):
                return None
            raise RuntimeError(f"Docker {' '.join(args[:2])} failed: {result.stderr.strip()}")
        return result.stdout

    def _inspect(self, name):
        raw = self._docker("container", "inspect", name, missing_ok=True)
        return json.loads(raw)[0] if raw else None

    def _owned(self, item):
        labels = item["Config"].get("Labels", {})
        return (labels.get("io.elesim.install_uuid") == self.owner["install_uuid"] and
                labels.get("io.elesim.bench_id") == self.token and
                labels.get("com.docker.compose.project") == self.owner["docker"]["project"])

    def preflight(self, plan, run):
        self.run = run
        self.owner = json.loads((self.prefix / "install-ownership.json").read_text())
        uuid.UUID(self.owner["install_uuid"])
        if Path(self.owner["prefix_realpath"]).resolve() != self.prefix:
            raise ValueError("installation ownership prefix mismatch")
        wrapper = self.prefix / "bin/elesim-compose"
        if not wrapper.is_file():
            raise ValueError("current installation has no generated elesim-compose wrapper")
        pin = self.owner["docker"]
        if not pin.get("context") or not pin.get("engine_id") or not pin.get("project"):
            raise ValueError("installation has no pinned Docker identity")
        self.docker = ["docker", "--context", pin["context"]]
        self.docker_env = os.environ.copy()
        self.docker_env.pop("DOCKER_HOST", None)
        self.docker_env.pop("CUDA_VISIBLE_DEVICES", None)
        self.docker_env["DOCKER_CONTEXT"] = pin["context"]
        if self._docker("info", "--format", "{{.ID}}").strip() != pin["engine_id"]:
            raise ValueError("Docker Engine differs from the installation pin")
        self.base = [str(wrapper), "-f", str(self.prefix / "containers/compose.yaml"),
                     "-f", str(self.prefix / "containers/compose.instances.yaml")]
        config = subprocess.run([*self.base, "config", "--format", "json"], text=True,
                                capture_output=True, env=self.docker_env, timeout=30, check=True)
        resolved = json.loads(config.stdout)
        if resolved.get("name") != pin["project"]:
            raise ValueError("Compose project differs from installation ownership")
        roles = {stage.role for stage in plan.stages}
        if "workspace" in roles:
            raise ValueError("walking-gaze needs the generated development attachment; run the CLI there")
        for role in sorted(roles):
            matches = [(name, svc) for name, svc in resolved["services"].items()
                       if svc.get("labels", {}).get("io.elesim.system_id") == self.system
                       and svc.get("labels", {}).get("io.elesim.role") == role
                       and svc.get("labels", {}).get("io.elesim.install_uuid") == self.owner["install_uuid"]]
            if len(matches) != 1:
                raise ValueError(f"expected one owned {role} service in instance {self.system}")
            service, settings = matches[0]
            image = json.loads(self._docker("image", "inspect", settings["image"]))[0]
            if image["Config"].get("Labels", {}).get("io.elesim.install_uuid") != self.owner["install_uuid"]:
                raise ValueError("release image is not owned by this installation")
            self.services[role] = {"service": service, "image_id": image["Id"],
                                   "image_labels": image["Config"].get("Labels", {})}
        if any(stage.gpu for stage in plan.stages) and (not self.gpu or not self.gpu.startswith("GPU-")):
            raise ValueError("select exactly one physical GPU with --gpu GPU-<UUID>")
        (run / "control").mkdir()
        self.before = self._normal_inventory()
        self.validated = True
        return {"kind": "installed-compose", "install_uuid": self.owner["install_uuid"],
                "context": pin["context"], "engine_id": pin["engine_id"], "project": pin["project"],
                "instance": self.system, "gpu_uuid": self.gpu, "roles": self.services,
                "normal_containers_before": self.before, "bench_id": self.token,
                "isolation": "no network; stage-scoped read-only inputs; truth excluded from consumers"}

    def _normal_inventory(self):
        raw = self._docker("ps", "-aq", "--filter", "label=io.elesim.install_uuid=" + self.owner["install_uuid"])
        records = []
        for identity in raw.splitlines():
            item = self._inspect(identity)
            if item and not item["Config"].get("Labels", {}).get("io.elesim.bench_id"):
                records.append({"id": item["Id"], "image": item["Image"], "running": item["State"]["Running"]})
        return records

    def command(self, stage, plan, run, fingerprints):
        service = self.services[stage.role]
        # Do not mount the checkout/evidence tree: it can contain private truth.
        mounts = [
            {"type": "bind", "source": str(path),
             "target": "/bench/source/" + path.relative_to(ROOT).as_posix(), "read_only": True}
            for path in source_closure(stage.module, "workbench.research.support.worker")
        ] + [
            {"type": "bind", "source": str(run / "work/metadata"),
             "target": "/bench/run/metadata", "read_only": False},
        ]
        for name in stage.inputs:
            mounts.append({"type": "bind", "source": plan.inputs[name],
                           "target": plan.inputs[name], "read_only": True})
            for dependency in fingerprints.get(name, {}):
                if dependency.startswith("extends:"):
                    source = dependency.removeprefix("extends:")
                    if not any(m["target"] == source for m in mounts):
                        mounts.append({"type": "bind", "source": source, "target": source, "read_only": True})
        for relative in dict.fromkeys((*stage.reads, *stage.writes)):
            source = run / "work" / relative
            if not source.is_dir():
                raise ValueError(f"stage input not produced: {relative}")
            mounts.append({"type": "bind", "source": str(source), "target": f"/bench/run/{relative}",
                           "read_only": relative not in stage.writes})
        env = {"HOME": "/tmp", "PYTHONUNBUFFERED": "1", "PYTHONDONTWRITEBYTECODE": "1", "PYTHONPATH": "/bench/source",
               "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "2", "EGL_DEVICE_ID": "0",
               "XDG_CACHE_HOME": "/tmp/bench-cache", "DISPLAY": "",
               "NVIDIA_VISIBLE_DEVICES": self.gpu if stage.gpu else "void",
               "PYOPENGL_PLATFORM": "egl", "NVIDIA_DRIVER_CAPABILITIES": "compute,utility,graphics,video"}
        if stage.gpu:
            env["ELESIM_BENCH_GPU"] = self.gpu
        labels = {"io.elesim.install_uuid": self.owner["install_uuid"],
                  "io.elesim.bench_id": self.token, "io.elesim.role": stage.role}
        devices = [{"driver": "nvidia", "device_ids": [self.gpu], "capabilities": ["gpu"]}] if stage.gpu else []
        # Compose !override is required: ordinary merging would retain runtime
        # credentials/state mounts, networking and the installation's GPU set.
        fields = {"image": service["image_id"], "user": f"{os.getuid()}:{os.getgid()}",
                  "working_dir": "/bench/source", "network_mode": "none", "privileged": False,
                  "restart": "no", "read_only": True, "tmpfs": ["/tmp:exec,mode=1777"],
                  "security_opt": ["no-new-privileges:true"], "cap_drop": ["ALL"]}
        lines = ["services:", f"  {service['service']}:"]
        for key, value in fields.items():
            lines.append(f"    {key}: {json.dumps(value)}")
        for key, value in {"volumes": mounts, "environment": env, "labels": labels, "ports": [],
                           "depends_on": {}, "devices": [], "cap_add": []}.items():
            lines.append(f"    {key}: !override {json.dumps(value)}")
        lines += ["    networks: !reset []", "    gpus: !reset null", "    runtime: !reset null",
                  "    deploy:", "      resources:", "        reservations:",
                  f"          devices: !override {json.dumps(devices)}"]
        override = run / "control" / f"{stage.name}.yaml"
        override.write_text("\n".join(lines) + "\n")
        resolved = subprocess.run([*self.base, "-f", str(override), "config", "--format", "json"],
                                  env=self.docker_env, text=True, capture_output=True, timeout=30, check=True)
        final = json.loads(resolved.stdout)["services"][service["service"]]
        actual_mounts = {(v["source"], v["target"], v.get("read_only", False)) for v in final["volumes"]}
        expected_mounts = {(v["source"], v["target"], v["read_only"]) for v in mounts}
        if (actual_mounts != expected_mounts or final["image"] != service["image_id"] or
                final["network_mode"] != "none" or final.get("depends_on") or final.get("ports") or
                final["deploy"]["resources"]["reservations"].get("devices", []) != devices or
                "CUDA_VISIBLE_DEVICES" in final["environment"]):
            raise ValueError("resolved diagnostic Compose configuration does not match the plan")
        # Keep declared inputs at their original paths so relative YAML/model
        # references remain meaningful without mounting the whole checkout.
        bindings = {name: plan.inputs[name] for name in stage.inputs}
        bindings["run"] = "/bench/run"
        if "checkpoint" in fingerprints:
            bindings["checkpoint_sha256"] = fingerprints["checkpoint"]["."]["sha256"]
        name = f"elesim-bench-{self.token[:16]}-{stage.name}"
        self.resources.append({"name": name, "image": service["image_id"], "id": None,
                               "expected_mounts": mounts, "gpu_uuid": self.gpu if stage.gpu else None})
        write_json(run / "control/resources.json", {"bench_id": self.token, "resources": self.resources})
        worker = ["-m", "workbench.research.support.worker", "--metadata", f"/bench/run/metadata/{stage.name}.json",
                  "--module", stage.module, "--role", stage.role]
        if "config" in stage.inputs and stage.role in ("sim", "workspace"):
            worker += ["--simulation-config", bindings["config"]]
        command = [*self.base, "-f", str(override), "run", "--rm", "-T", "--no-deps", "--pull", "never",
                   "--name", name, "--entrypoint", "python3", service["service"],
                   *worker, "--",
                   *(a.format_map(bindings) for a in stage.args)]
        return command, self.docker_env

    def launch(self, command, env, log):
        process = super().launch(command, env, log)
        resource = self.resources[-1]
        deadline = time.monotonic() + 10
        while process.poll() is None and time.monotonic() < deadline:
            item = self._inspect(resource["name"])
            if item:
                if not self._owned(item) or item["Image"] != resource["image"]:
                    raise ValueError("diagnostic container identity mismatch")
                self._validate_container(item, resource)
                resource.update(id=item["Id"], mounts=item["Mounts"],
                                devices=item["HostConfig"].get("DeviceRequests"),
                                network=item["HostConfig"]["NetworkMode"])
                write_json(self.run / "control/resources.json", {"bench_id": self.token, "resources": self.resources})
                break
            time.sleep(.05)
        return process

    def _validate_container(self, item, resource):
        observed = {(m["Source"], m["Destination"], not m["RW"]) for m in item["Mounts"] if m["Type"] == "bind"}
        expected = {(m["source"], m["target"], m["read_only"]) for m in resource["expected_mounts"]}
        config = item["HostConfig"]
        requests = config.get("DeviceRequests") or []
        gpu = resource["gpu_uuid"]
        device_ok = (len(requests) == 1 and requests[0].get("DeviceIDs") == [gpu]) if gpu else not requests
        if (observed != expected or any(m["Type"] not in ("bind", "tmpfs") for m in item["Mounts"]) or
                config["NetworkMode"] != "none" or config.get("Privileged") or
                not config.get("ReadonlyRootfs") or not device_ok):
            raise ValueError("running diagnostic mounts/network/GPU differ from the validated plan")

    def cleanup(self):
        removed = []
        failures = []
        for resource in self.resources:
            try:
                item = self._inspect(resource["id"] or resource["name"])
                if item is None:
                    continue
                if not self._owned(item) or (resource["id"] and item["Id"] != resource["id"]):
                    raise ValueError("refusing cleanup of a foreign container")
                self._docker("stop", "--time", "5", item["Id"], missing_ok=True)
                self._docker("rm", item["Id"], missing_ok=True)
                removed.append(item["Id"])
            except Exception as exc:
                failures.append(str(exc))
        result = super().cleanup()
        result.update(own_containers_removed=removed)
        if self.validated:
            result["normal_containers_after"] = self._normal_inventory()
        if failures:
            raise ValueError("; ".join(failures))
        return result
