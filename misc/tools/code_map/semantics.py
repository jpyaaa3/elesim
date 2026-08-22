"""EleSim-specific semantic labels derived from syntax, never imports."""

from __future__ import annotations

from collections import defaultdict
from typing import Iterable

from .model import Edge, Node


SEMANTIC_GROUPS = {
    "dds": ("dds", "peer_envelope", "descriptor", "heartbeat", "rgbd"),
    "authority": ("lease", "session", "deadman", "estop", "sequence"),
    "webrtc": ("webrtc", "offer", "answer", "ice", "srtp", "frame"),
    "hardware": ("dynamixel", "unitree", "serial", "device", "write", "feedback"),
    "genesis": ("genesis", "simulation_command", "orbit", "camera"),
}

WORKFLOW_RULES = (
    ("startup", "Startup descriptor → heartbeat → bounded queue", ("descriptor", "heartbeat", "startup", "queue")),
    ("operator", "UI operator intent → Pilot result", ("operator_intent", "operator_result")),
    ("motion", "Pilot target → motion lease → command/ack", ("select_target", "lease", "motion_command", "ack")),
    (
        "simulation",
        "UI simulation command → Sim queue → Genesis apply",
        ("simulation_command", "queued", "genesis", "orbit"),
    ),
    ("rgbd", "RGB-D capture → DDS publish → perception", ("rgbd", "publish", "subscriber", "perception")),
    ("webrtc", "WebRTC session → signaling → ICE → first frame", ("webrtc", "session", "signal", "ice", "frame")),
    ("robot", "Robot validation → hardware write → feedback/deadman", ("validate", "write", "feedback", "deadman")),
)


def semantic_nodes_and_edges(nodes: Iterable[Node]) -> tuple[list[Node], list[Edge]]:
    semantic_nodes: dict[str, Node] = {}
    edges: list[Edge] = []
    for node in nodes:
        if node.kind not in {"module", "class", "function", "method"}:
            continue
        haystack = " ".join((node.name, node.qualname, str(node.detail))).lower()
        for group, terms in SEMANTIC_GROUPS.items():
            hits = sorted({term for term in terms if term in haystack})
            if not hits:
                continue
            target = f"semantic:{group}"
            semantic_nodes.setdefault(
                target,
                Node(target, "semantic", group.upper(), group, "", "contract", detail={"terms": list(terms)}),
            )
            edges.append(
                Edge(node.id, target, "contract", "declared", "inferred", node.path, node.line, {"matches": hits})
            )
    return list(semantic_nodes.values()), edges


def build_workflows(nodes: Iterable[Node]) -> list[dict[str, object]]:
    searchable = [node for node in nodes if node.path]
    workflows: list[dict[str, object]] = []
    for workflow_id, title, terms in WORKFLOW_RULES:
        matches: dict[str, list[str]] = defaultdict(list)
        for node in searchable:
            haystack = " ".join((node.name, node.qualname, str(node.detail))).lower()
            for term in terms:
                if term in haystack:
                    matches[term].append(node.id)
        ordered = [node_id for term in terms for node_id in matches.get(term, [])[:12]]
        workflows.append(
            {
                "id": workflow_id,
                "title": title,
                "terms": list(terms),
                "nodes": list(dict.fromkeys(ordered)),
                "coverage": sum(bool(matches.get(term)) for term in terms) / len(terms),
            }
        )
    return workflows
