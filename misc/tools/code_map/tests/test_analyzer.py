from __future__ import annotations

import json
import subprocess
from pathlib import Path

from misc.tools.code_map.analyzer import _parse, analyze_repository


def _node(snapshot, kind: str, name: str):
    return next(node for node in snapshot.nodes if node.kind == kind and node.name == name)


def test_ast_extracts_alias_nested_inheritance_decorators_callbacks_and_async():
    source = '''
import asyncio as aio
from collections import deque as Queue

def decorator(fn):
    return fn

class Base: pass

@decorator
class Child(Base):
    @decorator
    async def work(self):
        task = aio.create_task(self.finish())
        task.add_done_callback(self.on_done)
        return await task

    def finish(self):
        return Queue()

    def on_done(self, task):
        return task
'''
    nodes, raw_edges = _parse("pilot/src/thing.py", source, "modified")

    assert {node.kind for node in nodes} >= {"module", "class", "method"}
    child = next(node for node in nodes if node.kind == "class" and node.name == "Child")
    work = next(node for node in nodes if node.kind == "method" and node.name == "work")
    assert child.detail["bases"] == ["Base"]
    assert "decorator" in child.detail["decorators"]
    assert work.detail["async"] is True
    assert {edge["kind"] for edge in raw_edges} >= {"import", "inherits", "decorator", "async-task", "callback", "call"}
    assert any(edge["target_name"] == "aio.create_task" and edge["kind"] == "async-task" for edge in raw_edges)
    assert any(edge["target_name"] == "self.on_done" and edge["kind"] == "callback" for edge in raw_edges)


def test_parse_syntax_error_is_explicit_unparsed_node():
    nodes, edges = _parse("sim/broken.py", "def broken(:\n", "modified")
    assert not edges
    assert len(nodes) == 1
    assert nodes[0].kind == "unparsed"
    assert "error" in nodes[0].detail


def _git_repo(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    root.mkdir()
    subprocess.run(("git", "init", "-q"), cwd=root, check=True)
    subprocess.run(("git", "config", "user.email", "test@example.invalid"), cwd=root, check=True)
    subprocess.run(("git", "config", "user.name", "Code Map Test"), cwd=root, check=True)
    return root


def _commit(root: Path, message: str) -> None:
    subprocess.run(("git", "add", "-A"), cwd=root, check=True)
    subprocess.run(("git", "commit", "-qm", message), cwd=root, check=True)


def test_repository_includes_untracked_deleted_and_head_diff_changes(tmp_path: Path):
    root = _git_repo(tmp_path)
    (root / "pilot.py").write_text("def original():\n    return 1\n", encoding="utf-8")
    (root / "deleted.py").write_text("def gone():\n    return 0\n", encoding="utf-8")
    _commit(root, "baseline")
    (root / "pilot.py").write_text("def original():\n    return 2\n", encoding="utf-8")
    (root / "deleted.py").unlink()
    (root / "untracked.py").write_text("def fresh():\n    return 3\n", encoding="utf-8")

    snapshot = analyze_repository(root, use_cache=False)
    by_path = {node.path: node for node in snapshot.nodes if node.kind in {"module", "unparsed"}}
    assert by_path["pilot.py"].change == "modified"
    assert by_path["untracked.py"].change == "added"
    assert by_path["deleted.py"].change == "deleted"
    assert snapshot.stats["changed"] >= 3


def test_cache_roundtrip_preserves_snapshot_shape(tmp_path: Path):
    root = _git_repo(tmp_path)
    (root / "module.py").write_text("def f():\n    return 'ok'\n", encoding="utf-8")
    _commit(root, "baseline")
    first = analyze_repository(root, use_cache=False)
    second = analyze_repository(root, use_cache=True)
    assert first.digest == second.digest
    assert first.git_head == second.git_head
    assert [node.id for node in first.nodes] == [node.id for node in second.nodes]
    assert [edge.id for edge in first.edges] == [edge.id for edge in second.edges]
    payload = json.loads((root / ".elesim/analysis/code-map/snapshot.json").read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert all("id" in edge for edge in payload["edges"])
