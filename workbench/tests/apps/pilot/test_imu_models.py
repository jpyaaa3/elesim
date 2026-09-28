from __future__ import annotations

import json
from pathlib import Path

import pytest

from elesim_pilot.imu_models import ImuModelCatalog
from elesim_protocol import ImuModelDefinition, ProtocolError


ROOT = Path(__file__).resolve().parents[4]


def test_pilot_catalog_loads_shipped_identity_formula() -> None:
    catalog = ImuModelCatalog(ROOT / "payload/config/pilot/imu_models")
    assert catalog.list_models() == [{
        "id": "identity", "version": 1,
        "label": "Identity (no IMU correction)", "requires_imu": False,
    }]
    model = catalog.get("identity")
    assert model.command_payload()["program"]["outputs"] == [0, 1, 2, 3]


def test_pilot_catalog_rejects_symlink_and_mismatched_file(tmp_path: Path) -> None:
    source = ROOT / "payload/config/pilot/imu_models/identity.json"
    (tmp_path / "identity.json").symlink_to(source)
    with pytest.raises(ValueError, match="invalid Pilot IMU model file"):
        ImuModelCatalog(tmp_path)
    (tmp_path / "identity.json").unlink()
    (tmp_path / "wrong.json").write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    with pytest.raises(ValueError, match="filename/id mismatch"):
        ImuModelCatalog(tmp_path)


def test_imu_model_schema_rejects_bad_expression_graph() -> None:
    model = json.loads((ROOT / "payload/config/pilot/imu_models/identity.json").read_text())
    model["program"]["nodes"][0] = {"op": "add", "a": 0, "b": 0}
    with pytest.raises(ProtocolError, match="node a"):
        ImuModelDefinition.from_payload(model, catalog_file=True)
    model["program"]["nodes"][0] = {"op": [], "index": 0}
    with pytest.raises(ProtocolError, match="unsupported IMU model operation"):
        ImuModelDefinition.from_payload(model, catalog_file=True)
