"""Pilot-owned catalog of versioned Robot-local IMU correction programs."""

from __future__ import annotations

import json
from pathlib import Path

from elesim_protocol import ImuModelDefinition


class ImuModelCatalog:
    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self._models = self._load()

    def _load(self) -> dict[str, ImuModelDefinition]:
        if not self.directory.is_dir() or self.directory.is_symlink():
            raise ValueError(f"Pilot IMU model directory is unavailable: {self.directory}")
        files = sorted(self.directory.glob("*.json"))
        if not 1 <= len(files) <= 16:
            raise ValueError("Pilot IMU model catalog must contain 1..16 JSON files")
        models: dict[str, ImuModelDefinition] = {}
        for path in files:
            if path.is_symlink() or not path.is_file() or path.stat().st_size > 16384:
                raise ValueError(f"invalid Pilot IMU model file: {path.name}")
            raw = json.loads(path.read_text(encoding="utf-8"))
            model = ImuModelDefinition.from_payload(raw, catalog_file=True)
            if path.stem != model.model_id or model.model_id in models:
                raise ValueError(f"IMU model filename/id mismatch: {path.name}")
            models[model.model_id] = model
        return models

    def list_models(self) -> list[dict[str, object]]:
        return [model.catalog_entry() for model in self._models.values()]

    def get(self, model_id: str) -> ImuModelDefinition:
        try:
            return self._models[model_id]
        except KeyError as exc:
            raise ValueError(f"unknown Pilot IMU model: {model_id}") from exc
