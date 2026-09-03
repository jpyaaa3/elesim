from __future__ import annotations

import json
import gzip
import subprocess
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from misc.tools.code_map.server import (
    MAX_FLOW_EXPANSIONS,
    MAX_SOURCE_BYTES,
    CodeMapServer,
    _inside,
    _flow_graph,
    _source,
    _ui_map,
)
from misc.tools.code_map.model import Edge, Node, Snapshot


def test_source_is_contained_utf8_and_bounded(tmp_path: Path):
    root = tmp_path / "repo"
    root.mkdir()
    source = root / "ok.py"
    source.write_text("\n".join(f"line_{i}" for i in range(600)), encoding="utf-8")
    assert _source(root, "ok.py", 300)["start"] == 220
    assert _source(root, "ok.py", 300)["end"] <= 600
    with pytest.raises(ValueError):
        _inside(root, "../outside.py")
    outside = tmp_path / "outside.py"
    outside.write_text("secret", encoding="utf-8")
    (root / "link.py").symlink_to(outside)
    with pytest.raises(ValueError):
        _inside(root, "link.py")
    huge = root / "huge.py"
    huge.write_bytes(b"x" * (MAX_SOURCE_BYTES + 1))
    with pytest.raises(ValueError, match="exceeds"):
        _source(root, "huge.py", 1)


def _request(server: CodeMapServer, path: str, token: str | None = None, method: str = "GET"):
    url = f"http://127.0.0.1:{server.server_address[1]}{path}"
    request = urllib.request.Request(url, method=method)
    if token is not None:
        request.add_header("X-Code-Map-Token", token)
    return urllib.request.urlopen(request, timeout=2)


def test_server_requires_token_and_is_read_only(tmp_path: Path):
    root = tmp_path / "repo"
    root.mkdir()
    (root / "module.py").write_text("def f():\n    return 1\n", encoding="utf-8")
    (root / "web").mkdir()
    subprocess.run(("git", "init", "-q"), cwd=root, check=True)
    subprocess.run(("git", "add", "module.py"), cwd=root, check=True)
    subprocess.run(
        ("git", "-c", "user.name=Code Map", "-c", "user.email=code-map@example.invalid", "commit", "-qm", "baseline"),
        cwd=root,
        check=True,
    )
    server = CodeMapServer(("127.0.0.1", 0), root, "secret-token")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with pytest.raises(urllib.error.HTTPError) as error:
            _request(server, "/api/snapshot")
        assert error.value.code == 403
        with _request(server, "/api/source?path=module.py&line=1", "secret-token") as response:
            payload = json.load(response)
        assert payload["path"] == "module.py"
        with _request(server, "/api/ui-map", "secret-token") as response:
            ui_map = json.load(response)
        assert ui_map["surfaces"] == []
        assert ui_map["stats"]["controls"] == 0
        with pytest.raises(urllib.error.HTTPError) as error:
            _request(server, "/api/snapshot", "secret-token", "POST")
        assert error.value.code == 405
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_rejects_non_loopback_bind(tmp_path: Path):
    with pytest.raises(ValueError, match="loopback"):
        CodeMapServer(("0.0.0.0", 0), tmp_path, "token")


def test_json_api_uses_gzip_for_large_payload(tmp_path: Path):
    root = tmp_path / "repo"
    root.mkdir()
    subprocess.run(("git", "init", "-q"), cwd=root, check=True)
    (root / "module.py").write_text("\n".join(f"def f_{i}(): return {i}" for i in range(80)), encoding="utf-8")
    subprocess.run(("git", "add", "module.py"), cwd=root, check=True)
    subprocess.run(
        ("git", "-c", "user.name=Code Map", "-c", "user.email=code-map@example.invalid", "commit", "-qm", "baseline"),
        cwd=root,
        check=True,
    )
    server = CodeMapServer(("127.0.0.1", 0), root, "secret-token")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        url = f"http://127.0.0.1:{server.server_address[1]}/api/snapshot"
        request = urllib.request.Request(url, headers={"X-Code-Map-Token": "secret-token", "Accept-Encoding": "gzip"})
        with urllib.request.urlopen(request, timeout=2) as response:
            assert response.headers["Content-Encoding"] == "gzip"
            payload = json.loads(gzip.decompress(response.read()))
        assert payload["stats"]["nodes"] >= 80
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_flow_graph_exposes_directional_layers_and_control_edges():
    nodes = [
        Node("ui:a", "function", "a", "a", "ui/a.py", "ui"),
        Node("pilot:b", "function", "b", "b", "pilot/b.py", "pilot"),
        Node("sim:c", "function", "c", "c", "sim/c.py", "sim"),
    ]
    edges = [
        Edge("ui:a", "pilot:b", "call"),
        Edge("pilot:b", "sim:c", "call"),
        Edge("sim:c", "pilot:b", "callback"),
        Edge("ui:a", "sim:c", "import"),
    ]
    flow = {
        "id": "flow:test",
        "title": "test",
        "entry_nodes": ["ui:a"],
        "nodes": ["ui:a", "pilot:b", "sim:c"],
    }
    snapshot = Snapshot("digest", "head", "now", nodes, edges, [], {}, flows=[flow])
    payload = _flow_graph(snapshot, "flow:test", "both", 20, "full")
    by_id = {node["id"]: node for node in payload["nodes"]}
    assert [by_id[node_id]["flow_depth"] for node_id in ("ui:a", "pilot:b", "sim:c")] == [0, 1, 2]
    assert by_id["ui:a"]["flow_entry"] is True
    by_edge = {(edge["source"], edge["target"]): edge for edge in payload["edges"]}
    assert by_edge[("ui:a", "pilot:b")]["flow_edge"] is True
    assert by_edge[("ui:a", "sim:c")]["flow_edge"] is False
    assert by_edge[("sim:c", "pilot:b")]["flow_backedge"] is True
    assert payload["flow"]["direction"] == "both"
    upstream = _flow_graph(
        Snapshot("digest", "head", "now", nodes, edges, [], {}, flows=[{
            **flow,
            "entry_nodes": ["sim:c"],
        }]),
        "flow:test",
        "upstream",
        20,
        "full",
    )
    upstream_by_id = {node["id"]: node for node in upstream["nodes"]}
    assert upstream["flow"]["layout"] == "directed"
    assert upstream_by_id["sim:c"]["flow_depth"] == 0
    assert upstream_by_id["pilot:b"]["flow_depth"] == -1
    assert upstream_by_id["ui:a"]["flow_depth"] == -2
    assert upstream_by_id["ui:a"]["flow_order"] < upstream_by_id["pilot:b"]["flow_order"]


def test_flow_spine_collapses_helpers_and_expands_without_losing_boundaries():
    nodes = [
        Node("ui:entry", "function", "entry", "entry", "ui/a.py", "ui"),
        Node("ui:branch", "function", "branch", "branch", "ui/a.py", "ui", detail={"branches": [{"line": 2}]}),
        Node("ui:helper-a", "function", "helper_a", "helper_a", "ui/a.py", "ui"),
        Node("ui:helper-b", "function", "helper_b", "helper_b", "ui/a.py", "ui"),
        Node("semantic:dds", "semantic", "DDS", "dds", "", "contract"),
    ]
    edges = [
        Edge("ui:entry", "ui:branch", "callback"),
        Edge("ui:branch", "ui:helper-a", "call"),
        Edge("ui:branch", "ui:helper-b", "call"),
        Edge("ui:branch", "semantic:dds", "contract"),
        Edge("ui:helper-a", "external:value", "data-write", confidence="unresolved"),
    ]
    flow = {"id": "flow:collapse", "title": "collapse", "entry_nodes": ["ui:entry"], "nodes": [node.id for node in nodes]}
    snapshot = Snapshot("digest", "head", "now", nodes, edges, [], {}, flows=[flow])

    spine = _flow_graph(snapshot, flow["id"], view="spine")
    assert {node["id"] for node in spine["nodes"]} >= {"ui:entry", "ui:branch", "semantic:dds"}
    placeholders = [node for node in spine["nodes"] if node["kind"] == "collapsed"]
    assert len(placeholders) == 1
    assert placeholders[0]["detail"]["collapsed_count"] == 2
    assert spine["flow"]["collapsed"] == 2
    assert any(edge["kind"] == "collapsed" for edge in spine["edges"])

    expanded = _flow_graph(snapshot, flow["id"], view="spine", expand={placeholders[0]["id"]})
    assert {"ui:helper-a", "ui:helper-b"} <= {node["id"] for node in expanded["nodes"]}
    assert expanded["flow"]["collapsed"] == 0
    full = _flow_graph(snapshot, flow["id"], view="full")
    assert not any(node["kind"] == "collapsed" for node in full["nodes"])


def test_flow_projection_rejects_invalid_view_and_expansion():
    node = Node("ui:entry", "function", "entry", "entry", "ui/a.py", "ui")
    flow = {"id": "flow:validation", "title": "validation", "entry_nodes": [node.id], "nodes": [node.id]}
    snapshot = Snapshot("digest", "head", "now", [node], [], [], {}, flows=[flow])
    with pytest.raises(ValueError, match="view"):
        _flow_graph(snapshot, flow["id"], view="unknown")
    with pytest.raises(ValueError, match="unknown flow expansion"):
        _flow_graph(snapshot, flow["id"], expand={"collapsed:missing"})
    with pytest.raises(ValueError, match="unknown flow expansion"):
        _flow_graph(snapshot, flow["id"], expand={node.id})
    with pytest.raises(ValueError, match="at most"):
        _flow_graph(snapshot, flow["id"], expand={f"x:{index}" for index in range(MAX_FLOW_EXPANSIONS + 1)})


def test_flow_spine_preserves_ast_branch_loop_and_exception_edges():
    nodes = [
        Node("entry", "function", "entry", "entry", "ui/a.py", "ui"),
        Node("branch", "function", "branch", "branch", "ui/a.py", "ui"),
        Node("loop", "function", "loop", "loop", "ui/a.py", "ui"),
        Node("error", "external", "RuntimeError", "RuntimeError", "", "external"),
    ]
    edges = [
        Edge("entry", "branch", "call", detail={"control": [{"kind": "branch", "line": 3}]}),
        Edge("branch", "loop", "call", detail={"control": [{"kind": "loop", "line": 4}]}),
        Edge("loop", "error", "exception"),
    ]
    flow = {"id": "flow:control", "title": "control", "entry_nodes": ["entry"], "nodes": [node.id for node in nodes]}
    payload = _flow_graph(Snapshot("d", "h", "n", nodes, edges, [], {}, flows=[flow]), flow["id"])
    by_edge = {(edge["source"], edge["target"]): edge for edge in payload["edges"]}
    assert by_edge[("entry", "branch")]["flow_branch"] is True
    assert by_edge[("branch", "loop")]["flow_loop"] is True
    assert {"entry", "branch", "loop", "error"} <= {node["id"] for node in payload["nodes"]}


def test_ui_map_links_mock_controls_to_action_workflows():
    widget = {
        "kind": "button",
        "label": "Run {mode}##run",
        "id": "run",
        "expression": "f'Run {mode}##run'",
        "dynamic": True,
        "control": [{"kind": "branch", "line": 8, "arm": "body"}],
        "line": 9,
    }
    node = Node(
        "ui:panel",
        "method",
        "draw",
        "elesim_ui.panels.control_4dof.draw",
        "payload/runtime/docker/ui/app/elesim_ui/panels/control_4dof.py",
        "ui",
        line=5,
        detail={"ui_widgets": [widget]},
    )
    flow = {
        "id": "ui-action:run",
        "title": "Run",
        "family": "operator",
        "kind": "action",
        "detail": {"widget": widget, "template": node.id},
    }
    payload = _ui_map(Snapshot("d", "h", "n", [node], [], [], {}, flows=[flow]))
    assert payload["stats"] == {"surfaces": 1, "controls": 1, "dynamic": 1}
    assert payload["surfaces"][0]["title"] == "4-DOF Control"
    control = payload["surfaces"][0]["sections"][0]["controls"][0]
    assert control["workflow_id"] == flow["id"]
    assert control["conditional"] is True
    assert control["line"] == 9


def test_web_assets_are_local_and_expose_required_controls():
    web = Path(__file__).parents[1] / "web"
    html = (web / "index.html").read_text(encoding="utf-8")
    script = (web / "app.js").read_text(encoding="utf-8")
    assert "https://" not in html
    assert {"workflow", "flow-view", "collapse-all", "hideTests", "mock-ui-toggle", "mock-ui", "mock-ui-content", "depth", "source", "diff"} <= {
        marker.split('"', 1)[0] for marker in html.split('id="')[1:]
    }
    assert 'id="hideTests" type="checkbox" checked' in html
    assert "/api/snapshot" in script
    assert "/api/events" in script
    assert "/api/traces" not in script
    assert "/api/ui-map" in script
    assert 'id="groups"' in html
    assert "function roleLayout" in script
    assert "function fitGraph" in script
    assert "function zoomAt" in script
    assert "screenX - worldX * nextScale" in script
    assert "flow-arrow" in html
    assert '"elk.direction": "DOWN"' in script
    assert '"elk.port.side": "NORTH"' in script
    assert '"elk.port.side": "SOUTH"' in script
    assert 'class: "port input", cx: 0, cy: -NODE_HEIGHT / 2' in script
    assert 'class: "port output", cx: 0, cy: NODE_HEIGHT / 2' in script
    assert " V ${middleY} H ${target.x} V ${endY}" in script
    assert "sideX" in script
    assert "flow_depth" in script
    assert "node.kind === \"collapsed\"" in script
    assert "function isTestNode" in script
    assert "function renderMockUi" in script
    assert "function activateMockWorkflow" in script
    assert "정적 탐색 전용 · 실제 명령은 실행되지 않음" in html
    assert "selectedFlow ? withoutTests(graph)" in script
    assert "suppressClick = Boolean(dragging?.moved)" in script
    assert "`S${Number(node.flow_depth" in script
