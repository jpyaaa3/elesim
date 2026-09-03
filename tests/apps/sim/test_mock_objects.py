from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from elesim_sim.simulation.mock_objects import (
    MockObjectArtifact,
    MockObjectCatalog,
    MockObjectError,
    MockObjectLimits,
    project_artifact_xz,
)


REPO_ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / "AGENTS.md").is_file())
DEMO_BOX = REPO_ROOT / "payload" / "data" / "models" / "objects" / "demo_box.obj"


def _cube() -> str:
    return """\
# A cube with duplicate XZ points at the top and bottom.
v -1 0 -1
v  1 0 -1
v  1 0  1
v -1 0  1
v -1 2 -1
v  1 2 -1
v  1 2  1
v -1 2  1
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 2 6 7 3
f 3 7 8 4
f 4 8 5 1
"""


def _write(root: Path, name: str, contents: str | bytes) -> Path:
    path = root / name
    if isinstance(contents, bytes):
        path.write_bytes(contents)
    else:
        path.write_text(contents, encoding="utf-8")
    return path


def test_valid_cube_is_immutable_and_described_by_content_hash(tmp_path: Path) -> None:
    path = _write(tmp_path, "cube.obj", _cube())

    artifact = MockObjectCatalog(tmp_path).load("cube.obj")

    assert isinstance(artifact, MockObjectArtifact)
    assert artifact.asset_id == "cube"
    assert artifact.units == "m"
    assert artifact.vertex_count == 8
    assert artifact.face_count == 6
    assert artifact.polygon_xz == ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    assert artifact.sha256 == digest
    assert artifact.revision == f"sha256:{digest}"
    assert artifact.descriptor()["sha256"] == digest
    with pytest.raises(AttributeError):
        artifact.asset_id = "other"  # type: ignore[misc]


def test_world_xz_projection_uses_the_same_xyz_euler_pose_as_genesis(tmp_path: Path) -> None:
    artifact = MockObjectCatalog(tmp_path).load_asset(
        _write(tmp_path, "cube.obj", _cube()).name
    )

    assert project_artifact_xz(artifact, (0.0, 0.0, 0.0)) == artifact.polygon_xz
    # A 90-degree X roll moves the cube's local Y extent into world Z.
    assert project_artifact_xz(artifact, (90.0, 0.0, 0.0)) == (
        (-1.0, 0.0),
        (1.0, 0.0),
        (1.0, 2.0),
        (-1.0, 2.0),
    )


@pytest.mark.parametrize("name", ["../cube.obj", "nested/cube.obj", "/tmp/cube.obj", "cube.txt"])
def test_catalog_rejects_absolute_traversal_and_non_obj_names(tmp_path: Path, name: str) -> None:
    _write(tmp_path, "cube.obj", _cube())

    with pytest.raises(MockObjectError, match=r"basename|\.obj|unsafe"):
        MockObjectCatalog(tmp_path).load(name)


def test_catalog_rejects_symlinked_root_and_asset(tmp_path: Path) -> None:
    source = tmp_path / "source"
    source.mkdir()
    _write(source, "cube.obj", _cube())
    root_link = tmp_path / "root-link"
    asset_link = tmp_path / "alias.obj"
    try:
        root_link.symlink_to(source, target_is_directory=True)
        asset_link.symlink_to(source / "cube.obj")
    except (OSError, NotImplementedError):
        pytest.skip("symlinks unavailable")

    with pytest.raises(MockObjectError, match="symlink"):
        MockObjectCatalog(root_link)
    with pytest.raises(MockObjectError, match="symlink"):
        MockObjectCatalog(tmp_path).load("alias.obj")


def test_nonfinite_vertex_is_rejected(tmp_path: Path) -> None:
    contents = "v 0 0 0\nv nan 0 0\nv 0 0 1\nf 1 2 3\n"
    _write(tmp_path, "bad.obj", contents)

    with pytest.raises(MockObjectError, match="non-finite"):
        MockObjectCatalog(tmp_path).load("bad.obj")


@pytest.mark.parametrize("face", ["f 1 2 4", "f 1 0 2", "f 1 nope 2"])
def test_invalid_face_index_is_rejected(tmp_path: Path, face: str) -> None:
    _write(tmp_path, "bad.obj", f"v 0 0 0\nv 1 0 0\nv 0 0 1\n{face}\n")

    with pytest.raises(MockObjectError, match="face index"):
        MockObjectCatalog(tmp_path).load("bad.obj")


def test_file_vertex_face_and_line_bounds_are_enforced(tmp_path: Path) -> None:
    _write(tmp_path, "large.obj", _cube())
    with pytest.raises(MockObjectError, match="file exceeds"):
        MockObjectCatalog(tmp_path, limits=MockObjectLimits(max_file_bytes=8)).load("large.obj")

    _write(tmp_path, "many.obj", "\n".join("v 0 0 0" for _ in range(3)) + "\nf 1 2 3\n")
    with pytest.raises(MockObjectError, match="vertex count"):
        MockObjectCatalog(tmp_path, limits=MockObjectLimits(max_vertices=2)).load("many.obj")

    _write(tmp_path, "faces.obj", "v 0 0 0\nv 1 0 0\nv 0 0 1\nf 1 2 3\nf 1 3 2\n")
    with pytest.raises(MockObjectError, match="face count"):
        MockObjectCatalog(tmp_path, limits=MockObjectLimits(max_faces=1)).load("faces.obj")

    _write(tmp_path, "line.obj", "v 0 0 0 # " + "x" * 100 + "\nv 1 0 0\nv 0 0 1\nf 1 2 3\n")
    with pytest.raises(MockObjectError, match="line"):
        MockObjectCatalog(tmp_path, limits=MockObjectLimits(max_line_bytes=16)).load("line.obj")


def test_hull_is_deterministic_and_polygon_point_bound_is_enforced(tmp_path: Path) -> None:
    first = "v 0 0 0\nv 1 0 0\nv 0 0 1\nf 1 2 3\n"
    second = "v 0 0 1\nv 0 0 0\nv 1 0 0\nf 3 1 2\n"
    _write(tmp_path, "a.obj", first)
    _write(tmp_path, "b.obj", second)
    a = MockObjectCatalog(tmp_path).load("a")
    b = MockObjectCatalog(tmp_path).load("b")
    assert a.polygon_xz == b.polygon_xz
    assert a.sha256 != b.sha256

    with pytest.raises(MockObjectError, match="polygon"):
        MockObjectCatalog(
            tmp_path,
            limits=MockObjectLimits(max_polygon_points=2),
        ).load("a.obj")


def test_degenerate_projection_is_rejected(tmp_path: Path) -> None:
    _write(tmp_path, "flat.obj", "v 0 0 0\nv 1 1 0\nv 2 2 0\nf 1 2 3\n")

    with pytest.raises(MockObjectError, match="degenerate|area"):
        MockObjectCatalog(tmp_path).load("flat.obj")


def test_invalid_utf8_is_rejected(tmp_path: Path) -> None:
    _write(tmp_path, "bad.obj", b"v \xff 0 0\nv 1 0 0\nv 0 0 1\nf 1 2 3\n")

    with pytest.raises(MockObjectError, match="UTF-8"):
        MockObjectCatalog(tmp_path).load("bad.obj")


def test_assets_lists_only_direct_obj_files(tmp_path: Path) -> None:
    _write(tmp_path, "b.obj", _cube())
    _write(tmp_path, "a.txt", "ignored")
    (tmp_path / "nested").mkdir()
    _write(tmp_path / "nested", "nested.obj", _cube())

    assert MockObjectCatalog(tmp_path).assets() == ("b.obj",)


def test_catalog_rejects_asset_names_that_cannot_cross_protocol(tmp_path: Path) -> None:
    _write(tmp_path, "bad name.obj", _cube())
    with pytest.raises(MockObjectError, match="invalid asset id"):
        MockObjectCatalog(tmp_path).assets()


def test_catalog_freezes_the_artifact_used_by_the_built_scene(tmp_path: Path) -> None:
    path = _write(tmp_path, "cube.obj", _cube())
    catalog = MockObjectCatalog(tmp_path)
    first = catalog.load("cube")
    path.write_text(_cube().replace("v -1 0 -1", "v -2 0 -1"), encoding="utf-8")

    assert catalog.load("cube.obj") is first


def test_builtin_demo_box_faces_point_outward_for_genesis_backface_culling() -> None:
    artifact = MockObjectCatalog(DEMO_BOX.parent).load(DEMO_BOX.name)
    signed_volume = 0.0
    for face in artifact.faces:
        anchor = artifact.vertices[face[0]]
        for index in range(1, len(face) - 1):
            left = artifact.vertices[face[index]]
            right = artifact.vertices[face[index + 1]]
            cross = (
                left[1] * right[2] - left[2] * right[1],
                left[2] * right[0] - left[0] * right[2],
                left[0] * right[1] - left[1] * right[0],
            )
            signed_volume += sum(a * b for a, b in zip(anchor, cross)) / 6.0

    assert signed_volume > 0.0
