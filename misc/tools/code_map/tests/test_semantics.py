from __future__ import annotations

from misc.tools.code_map.model import Node
from misc.tools.code_map.semantics import build_workflows, semantic_nodes_and_edges


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
