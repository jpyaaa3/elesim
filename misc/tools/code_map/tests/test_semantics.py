from __future__ import annotations

from misc.tools.code_map.model import Edge, Node
from misc.tools.code_map.semantics import build_flow_catalog, build_workflows, semantic_nodes_and_edges


def _n(name: str, detail: str = "") -> Node:
    return Node(f"pilot:x.py:{name}", "function", name, name, "pilot/x.py", "pilot", detail={"text": detail})


def test_semantic_groups_and_workflows_are_declared_and_covered():
    nodes = [
        _n("publish_rgbd", "dds peer_envelope descriptor heartbeat queue"),
        _n("operator_intent", "operator_result"),
        _n("select_target", "lease motion_command ack"),
        _n("simulation_command", "queued genesis orbit"),
        _n("webrtc_session", "signal ice frame"),
        _n("hardware_write", "validate feedback deadman"),
    ]
    semantic, edges = semantic_nodes_and_edges(nodes)
    assert {node.id for node in semantic} >= {
        "semantic:dds",
        "semantic:authority",
        "semantic:webrtc",
        "semantic:hardware",
        "semantic:genesis",
    }
    assert all(edge.evidence == "declared" and edge.kind == "contract" for edge in edges)
    workflows = {item["id"]: item for item in build_workflows(nodes)}
    assert workflows["startup"]["coverage"] > 0
    assert workflows["operator"]["coverage"] == 1
    assert workflows["motion"]["coverage"] == 1
    assert workflows["simulation"]["coverage"] == 1
    assert workflows["webrtc"]["coverage"] == 1
    assert workflows["robot"]["coverage"] == 1


def test_flow_catalog_creates_action_and_system_slices():
    action = Node(
        "ui:panel.py:draw", "function", "draw", "panel.draw", "ui/panel.py", "ui",
        detail={"ui_widgets": [{"kind": "button", "id": "run", "label": "Run"}]},
    )
    system = _n("publish_rgbd", "dds rgbd publish perception")
    edge = Edge(action.id, system.id, "call")
    flows = build_flow_catalog([action, system], [edge])
    actions = [flow for flow in flows if flow["kind"] == "action"]
    assert actions and actions[0]["detail"]["widget"]["id"] == "run"
    assert system.id in actions[0]["nodes"]


def test_flow_catalog_keeps_one_entry_per_widget_and_bounded_slices():
    widgets = [
        {"kind": "button", "id": "run", "label": "Run##run"},
        {"kind": "slider_float", "id": "speed", "label": "Speed"},
    ]
    action = Node(
        "ui:panel.py:draw", "function", "draw", "panel.draw", "ui/panel.py", "ui",
        detail={"ui_widgets": widgets},
    )
    flows = build_flow_catalog([action], [])
    actions = [flow for flow in flows if flow["kind"] == "action"]
    assert len(actions) == len(widgets)
    assert len({flow["id"] for flow in actions}) == len(actions)
    assert all(len(flow["nodes"]) <= 96 for flow in actions)
