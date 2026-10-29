"""Load explicit trusted model factories, validating API and checkpoint hashes."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import importlib
import json
from pathlib import Path
import re

import yaml

from .contracts import MODEL_API_VERSION


METHODS = {"segmenter": "segment", "pose": "estimate", "pose_correction": "correct",
           "completion": "complete", "mesher": "mesh", "planner": "plan"}
_FACTORY = re.compile(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*\Z")


@dataclass(frozen=True)
class Model:
    stage: str
    implementation: object
    provenance: dict


def load_model(stage, spec, root: Path) -> Model:
    if stage not in METHODS or not isinstance(spec, dict) or set(spec) - {"factory", "options", "assets"}:
        raise ValueError(f"invalid {stage} model configuration")
    factory = spec.get("factory", "")
    if not isinstance(factory, str) or not _FACTORY.fullmatch(factory):
        raise ValueError(f"{stage} factory must be module:callable")
    options = spec.get("options", {})
    assets = spec.get("assets", {})
    if not isinstance(options, dict) or not isinstance(assets, dict) or len(assets) > 16:
        raise ValueError("model options/assets must be bounded objects")
    if len(json.dumps(options, allow_nan=False)) > 32768:
        raise ValueError("model options exceed budget")
    paths, hashes = {}, {}
    for name, asset in assets.items():
        if not isinstance(name, str) or not name.isidentifier() or not isinstance(asset, dict) or set(asset) != {"path", "sha256"}:
            raise ValueError("model assets require a name, path and sha256")
        path = Path(asset["path"]).expanduser()
        path = path if path.is_absolute() else root / path
        if not path.is_file():
            raise ValueError(f"missing {stage} model asset: {path}")
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            while part := handle.read(1024*1024):
                digest.update(part)
        if digest.hexdigest() != asset["sha256"]:
            raise ValueError(f"{stage} asset hash mismatch: {name}")
        paths[name], hashes[name] = str(path.resolve()), digest.hexdigest()
    module, name = factory.split(":")
    try:
        implementation = getattr(importlib.import_module(module), name)(options=dict(options), assets=dict(paths))
    except Exception as exc:
        raise RuntimeError(f"{stage} model factory {factory} could not load: {exc}") from exc
    api_version = getattr(implementation, "api_version", None)
    if type(api_version) is not int or api_version != MODEL_API_VERSION:
        raise ValueError(f"{stage} model API version must be {MODEL_API_VERSION}")
    if getattr(implementation, "stage", None) != stage or not callable(getattr(implementation, METHODS[stage], None)):
        raise ValueError(f"{stage} model does not implement {METHODS[stage]}")
    version = getattr(implementation, "version", None)
    if not isinstance(version, str) or not 1 <= len(version) <= 128:
        raise ValueError(f"{stage} model must report a bounded version")
    return Model(stage, implementation, {"factory": factory, "version": version, "assets": hashes,
        "options_sha256": hashlib.sha256(json.dumps(options, sort_keys=True, allow_nan=False).encode()).hexdigest()})


def load_models(path):
    path = Path(path).expanduser().resolve()
    if path.stat().st_size > 128*1024:
        raise ValueError("pipeline configuration exceeds budget")
    config = yaml.safe_load(path.read_text())
    if not isinstance(config, dict) or set(config) != {"schema_version", "models"} or type(config["schema_version"]) is not int or config["schema_version"] != 1:
        raise ValueError("pipeline configuration requires schema_version 1 and models")
    specs = config["models"]
    if not isinstance(specs, dict) or set(specs) - {*METHODS, "pose_corrections"}:
        raise ValueError("unknown reconstruction model stage")
    if "pose_correction" in specs:
        raise ValueError("use the ordered pose_corrections list")
    if not {"segmenter", "pose", "completion", "mesher"} <= set(specs):
        raise ValueError("segmenter, pose, completion and mesher models are required")
    corrections = specs.get("pose_corrections", [])
    if not isinstance(corrections, list) or len(corrections) > 4:
        raise ValueError("pose_corrections must be an ordered list of at most four models")
    models = {stage: load_model(stage, spec, path.parent)
              for stage, spec in specs.items() if stage != "pose_corrections"}
    return models, tuple(load_model("pose_correction", spec, path.parent) for spec in corrections)
