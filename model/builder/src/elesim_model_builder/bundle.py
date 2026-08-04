"""Build and validate immutable sim model bundles."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Sequence

from elesim_model_builder.go2_arm_merger import merge_go2_arm_urdf
from elesim_model_builder.json_builder import build_default_manifest
from elesim_model_builder.urdf_converter import convert_manifest_file


BUNDLE_SCHEMA_VERSION = 1
BUNDLE_TYPE = "elesim.sim-model"
BUNDLE_METADATA = "bundle.json"
ENTRYPOINTS = {
    "blueprint": "blueprint.json",
    "arm_urdf": "arm.urdf",
    "robot_urdf": "robot.urdf",
}


class BundleIntegrityError(ValueError):
    """Raised when a sim model bundle is incomplete or was modified."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_sha256(value: Mapping[str, Any]) -> str:
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _safe_relative_path(raw: object, *, context: str) -> Path:
    value = str(raw or "").strip().replace("\\", "/")
    path = Path(value)
    if not value or path.is_absolute() or ".." in path.parts or "://" in value:
        raise BundleIntegrityError(f"unsafe {context} path: {value!r}")
    return path


def _payload_files(bundle_dir: Path) -> list[Path]:
    return sorted(
        path
        for path in bundle_dir.rglob("*")
        if path.is_file() and path.name != BUNDLE_METADATA
    )


def _file_records(bundle_dir: Path) -> dict[str, dict[str, object]]:
    records: dict[str, dict[str, object]] = {}
    for path in _payload_files(bundle_dir):
        relative = path.relative_to(bundle_dir).as_posix()
        records[relative] = {"sha256": _sha256(path), "size": path.stat().st_size}
    return records


def _source_digest(asset_root: Path) -> str:
    records = {
        path.relative_to(asset_root).as_posix(): _sha256(path)
        for path in sorted(asset_root.rglob("*"))
        if path.is_file()
    }
    return _canonical_sha256(records)


def _write_metadata(bundle_dir: Path, *, source_asset_sha256: str, use_go2: bool) -> None:
    metadata: dict[str, Any] = {
        "schema_version": BUNDLE_SCHEMA_VERSION,
        "bundle_type": BUNDLE_TYPE,
        "entrypoints": dict(ENTRYPOINTS),
        "build": {
            "format_version": 1,
            "source_assets_sha256": source_asset_sha256,
            "use_go2": bool(use_go2),
        },
        "files": _file_records(bundle_dir),
    }
    metadata["bundle_id"] = _canonical_sha256(metadata)
    (bundle_dir / BUNDLE_METADATA).write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _publish_staged_bundle(stage: Path, output_dir: Path) -> None:
    backup = stage.parent / "previous-bundle"
    if output_dir.exists():
        output_dir.rename(backup)
    try:
        stage.rename(output_dir)
    except BaseException:
        if backup.exists() and not output_dir.exists():
            backup.rename(output_dir)
        raise
    if backup.exists():
        shutil.rmtree(backup)


def build_sim_bundle(
    *,
    asset_root: str | os.PathLike[str],
    output_dir: str | os.PathLike[str],
    use_go2: bool = True,
    use_hardware: bool = False,
    mount_xyz: Sequence[float] = (0.35, 0.0, 0.08),
) -> Path:
    """Build a complete bundle and atomically publish it to ``output_dir``."""

    assets = Path(asset_root).resolve()
    output = Path(output_dir).resolve()
    if not assets.is_dir():
        raise FileNotFoundError(f"model asset root not found: {assets}")
    output.parent.mkdir(parents=True, exist_ok=True)
    source_asset_sha256 = _source_digest(assets)

    with tempfile.TemporaryDirectory(prefix=".elesim-bundle-", dir=output.parent) as temp_dir:
        stage = Path(temp_dir) / "bundle"
        stage.mkdir()
        bundled_assets = stage / "assets"
        shutil.copytree(assets, bundled_assets)

        blueprint = Path(
            build_default_manifest(
                str(stage),
                use_hardware=bool(use_hardware),
                use_go2=bool(use_go2),
                output_name=ENTRYPOINTS["blueprint"],
                asset_root=str(bundled_assets),
            )
        )
        arm_urdf = stage / ENTRYPOINTS["arm_urdf"]
        robot_urdf = stage / ENTRYPOINTS["robot_urdf"]
        convert_manifest_file(str(blueprint), str(arm_urdf))
        if use_go2:
            merge_go2_arm_urdf(
                go2_urdf_path=bundled_assets / "go2/go2.urdf",
                arm_urdf_path=arm_urdf,
                out_urdf_path=robot_urdf,
                mount_xyz=mount_xyz,
            )
        else:
            shutil.copy2(arm_urdf, robot_urdf)

        _write_metadata(
            stage,
            source_asset_sha256=source_asset_sha256,
            use_go2=bool(use_go2),
        )
        validate_bundle(stage)
        _publish_staged_bundle(stage, output)
    return output


def _load_metadata(bundle_dir: Path) -> dict[str, Any]:
    metadata_path = bundle_dir / BUNDLE_METADATA
    if not metadata_path.is_file():
        raise BundleIntegrityError(f"missing {BUNDLE_METADATA}: {metadata_path}")
    try:
        value = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise BundleIntegrityError(f"invalid {BUNDLE_METADATA}: {exc}") from exc
    if not isinstance(value, dict):
        raise BundleIntegrityError(f"{BUNDLE_METADATA} must contain an object")
    return value


def _validate_metadata(metadata: dict[str, Any]) -> dict[str, dict[str, object]]:
    if metadata.get("schema_version") != BUNDLE_SCHEMA_VERSION:
        raise BundleIntegrityError(
            f"unsupported bundle schema_version: {metadata.get('schema_version')!r}"
        )
    if metadata.get("bundle_type") != BUNDLE_TYPE:
        raise BundleIntegrityError(f"unexpected bundle_type: {metadata.get('bundle_type')!r}")
    if metadata.get("entrypoints") != ENTRYPOINTS:
        raise BundleIntegrityError("bundle entrypoints do not match the sim contract")
    files = metadata.get("files")
    if not isinstance(files, dict) or not files:
        raise BundleIntegrityError("bundle files manifest is empty or invalid")
    identity = dict(metadata)
    bundle_id = identity.pop("bundle_id", None)
    if not isinstance(bundle_id, str) or bundle_id != _canonical_sha256(identity):
        raise BundleIntegrityError("bundle_id does not match metadata")
    return files


def _validate_files(bundle_dir: Path, records: Mapping[str, object]) -> None:
    expected: set[str] = set()
    for raw_relative, raw_record in records.items():
        relative = _safe_relative_path(raw_relative, context="manifest file")
        name = relative.as_posix()
        expected.add(name)
        if not isinstance(raw_record, dict):
            raise BundleIntegrityError(f"invalid file record: {name}")
        path = bundle_dir / relative
        if not path.is_file():
            raise BundleIntegrityError(f"missing payload file: {name}")
        expected_hash = raw_record.get("sha256")
        if not isinstance(expected_hash, str) or _sha256(path) != expected_hash:
            raise BundleIntegrityError(f"hash mismatch: {name}")
        expected_size = raw_record.get("size")
        if not isinstance(expected_size, int) or path.stat().st_size != expected_size:
            raise BundleIntegrityError(f"size mismatch: {name}")

    actual = {
        path.relative_to(bundle_dir).as_posix()
        for path in _payload_files(bundle_dir)
    }
    untracked = sorted(actual - expected)
    missing_records = sorted(expected - actual)
    if untracked:
        raise BundleIntegrityError("untracked files: " + ", ".join(untracked))
    if missing_records:
        raise BundleIntegrityError("missing files: " + ", ".join(missing_records))


def _validate_reference(bundle_dir: Path, raw: object, *, context: str) -> None:
    relative = _safe_relative_path(raw, context=context)
    if not (bundle_dir / relative).is_file():
        raise BundleIntegrityError(f"missing {context}: {relative.as_posix()}")


def _validate_blueprint(bundle_dir: Path, relative: str) -> None:
    path = bundle_dir / relative
    try:
        blueprint = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise BundleIntegrityError(f"invalid blueprint: {exc}") from exc
    parts = blueprint.get("parts") if isinstance(blueprint, dict) else None
    if not isinstance(parts, list) or not parts:
        raise BundleIntegrityError("blueprint parts are empty or invalid")
    for index, part in enumerate(parts):
        assets = part.get("assets") if isinstance(part, dict) else None
        if not isinstance(assets, dict):
            raise BundleIntegrityError(f"blueprint part {index} has no assets object")
        for kind in ("mesh", "frame", "physics"):
            _validate_reference(
                bundle_dir,
                assets.get(kind),
                context=f"blueprint part {index} {kind}",
            )


def _validate_urdf(bundle_dir: Path, relative: str) -> None:
    path = bundle_dir / relative
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise BundleIntegrityError(f"invalid URDF {relative}: {exc}") from exc
    if root.tag != "robot":
        raise BundleIntegrityError(f"URDF root is not robot: {relative}")
    for index, mesh in enumerate(root.iter("mesh")):
        _validate_reference(
            bundle_dir,
            mesh.attrib.get("filename"),
            context=f"{relative} mesh {index}",
        )


def validate_bundle(bundle_dir: str | os.PathLike[str]) -> dict[str, Any]:
    """Validate metadata, payload hashes and every model reference."""

    bundle = Path(bundle_dir).resolve()
    if not bundle.is_dir():
        raise BundleIntegrityError(f"bundle directory not found: {bundle}")
    metadata = _load_metadata(bundle)
    records = _validate_metadata(metadata)
    _validate_files(bundle, records)
    entrypoints = metadata["entrypoints"]
    _validate_blueprint(bundle, entrypoints["blueprint"])
    _validate_urdf(bundle, entrypoints["arm_urdf"])
    _validate_urdf(bundle, entrypoints["robot_urdf"])
    return metadata


__all__ = [
    "BUNDLE_METADATA",
    "BUNDLE_SCHEMA_VERSION",
    "BUNDLE_TYPE",
    "BundleIntegrityError",
    "build_sim_bundle",
    "validate_bundle",
]
