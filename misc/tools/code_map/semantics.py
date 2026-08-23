"""EleSim-specific semantic labels derived from syntax, never imports."""

from __future__ import annotations

from collections import defaultdict, deque
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

SYSTEM_FLOW_RULES = (
    ("dds-startup", "DDS startup descriptor and bounded queue", "dds", ("descriptor", "heartbeat", "queue")),
    ("authority-motion", "Motion target, lease and command acknowledgement", "authority", ("target", "lease", "command", "ack")),
    ("simulation-session", "Simulation session and Genesis command", "simulation", ("simulation", "genesis", "orbit")),
    ("rgbd-perception", "RGB-D publication and perception consumer", "media", ("rgbd", "publish", "perception")),
    ("webrtc-media", "WebRTC signaling and first frame", "media", ("webrtc", "signal", "ice", "frame")),
    ("robot-safety", "Robot validation, hardware write and deadman", "safety", ("validate", "write", "feedback", "deadman")),
    ("lifecycle", "Process lifecycle and recovery", "lifecycle", ("startup", "shutdown", "restart", "recovery")),
)


def _haystack(node: Node) -> str:
    return " ".join((node.name, node.qualname, str(node.detail))).lower()


def _reachable(
    entry: str,
    nodes: dict[str, Node],
    edges: Iterable[Edge],
    budget: int = 96,
    adjacency: dict[str, list[str]] | None = None,
) -> list[str]:
    if adjacency is None:
        adjacency = defaultdict(list)
        for edge in edges:
            if edge.source in nodes and edge.target in nodes:
                adjacency[edge.source].append(edge.target)
    queue = deque([entry])
    seen: list[str] = []
    visited: set[str] = set()
    while queue and len(seen) < budget:
        current = queue.popleft()
        if current in visited:
            continue
        visited.add(current)
        seen.append(current)
        queue.extend(target for target in adjacency.get(current, []) if target not in visited)
    return seen


def _phase_roles(node_ids: Iterable[str], nodes: dict[str, Node]) -> list[str]:
    return list(dict.fromkeys(nodes[node_id].role for node_id in node_ids if node_id in nodes))


def _flow(
    flow_id: str,
    title: str,
    family: str,
    kind: str,
    trigger: str,
    entry: str,
    nodes: dict[str, Node],
    edges: list[Edge],
    detail: dict[str, object] | None = None,
    adjacency: dict[str, list[str]] | None = None,
) -> dict[str, object]:
    selected = _reachable(entry, nodes, edges, adjacency=adjacency)
    gaps = sorted(
        {
            str(edge.detail.get("resolution_gap") or edge.target)
            for edge in edges
            if edge.source in selected and (edge.confidence == "unresolved" or edge.detail.get("resolution_gap"))
        }
    )
    return {
        "id": flow_id,
        "title": title,
        "family": family,
        "kind": kind,
        "trigger": trigger,
        "entry_nodes": [entry],
        "nodes": selected,
        "phases": _phase_roles(selected, nodes),
        "coverage": 0.0 if gaps else 1.0,
        "gaps": gaps,
        "detail": detail or {},
    }


def build_flow_catalog(nodes: Iterable[Node], edges: Iterable[Edge]) -> list[dict[str, object]]:
    """Build micro-flow entries from current syntax and protocol vocabulary.

    This deliberately stores only entry metadata and a bounded initial slice. The
    graph itself remains the worktree-derived source of truth and can be expanded
    by the server without maintaining a second hand-written graph.
    """
    node_list = [node for node in nodes if node.path or node.kind in {"external", "semantic"}]
    by_id = {node.id: node for node in node_list}
    edge_list = list(edges)
    adjacency: dict[str, list[str]] = defaultdict(list)
    adjacency_rank: dict[tuple[str, str], tuple[int, str]] = {}
    edge_rank = {
        "call": 0,
        "callback": 1,
        "async-task": 1,
        "thread": 1,
        "entrypoint": 2,
        "contains": 3,
        "contract": 4,
        "data-write": 5,
        "import": 6,
        "inherits": 7,
    }
    for edge in edge_list:
        if edge.source in by_id and edge.target in by_id:
            adjacency[edge.source].append(edge.target)
            rank = (
                edge_rank.get(edge.kind, 8),
                "1" if edge.target.startswith("external:") else "0",
            )
            previous = adjacency_rank.get((edge.source, edge.target))
            if previous is None or rank < previous:
                adjacency_rank[(edge.source, edge.target)] = rank
    for source, targets in adjacency.items():
        adjacency[source] = sorted(
            set(targets),
            key=lambda target: (*adjacency_rank.get((source, target), (8, "0")), target),
        )
    result: list[dict[str, object]] = []
    used: set[str] = set()
    family_counts: dict[str, int] = defaultdict(int)

    for node in node_list:
        if node.role != "ui" or node.kind not in {"function", "method"}:
            continue
        widgets = node.detail.get("ui_widgets", [])
        if not isinstance(widgets, list):
            continue
        for index, widget in enumerate(widgets):
            if not isinstance(widget, dict):
                continue
            widget_id = str(widget.get("id") or widget.get("kind") or "widget")
            flow_id = f"ui-action:{node.id}:{widget_id}:{index}"
            if flow_id in used:
                continue
            used.add(flow_id)
            raw_label = str(widget.get("label") or widget.get("expression") or widget_id)
            label = raw_label.split("##", 1)[0].strip() or widget_id
            result.append(
                _flow(
                    flow_id,
                    f"{label} · {node.name}",
                    "operator",
                    "action",
                    f"ui.{widget.get('kind', 'widget')}",
                    node.id,
                    by_id,
                    edge_list,
                    {"widget": widget, "template": node.id, "operator_calls": node.detail.get("operator_calls", [])},
                    adjacency,
                )
            )

    # Non-UI entries expose autonomous and protocol paths that have no button.
    for node in node_list:
        if node.kind not in {"function", "method", "entrypoint"} or node.role not in {"pilot", "sim", "robot", "packages"}:
            continue
        haystack = _haystack(node)
        for family_id, title, family, terms in SYSTEM_FLOW_RULES:
            hits = [term for term in terms if term in haystack]
            if not hits:
                continue
            if family_counts[family_id] >= 24:
                continue
            flow_id = f"system:{family_id}:{node.id}"
            if flow_id in used:
                continue
            used.add(flow_id)
            result.append(
                _flow(
                    flow_id,
                    f"{title} · {node.name}",
                    family,
                    "system",
                    ".".join(hits),
                    node.id,
                    by_id,
                    edge_list,
                    {"matches": hits},
                    adjacency,
                )
            )
            family_counts[family_id] += 1

    # Keep the seven high-level overview cards as navigational families.
    for workflow in build_workflows(node_list):
        matches = [node_id for node_id in workflow.get("nodes", []) if node_id in by_id]
        if not matches:
            continue
        flow_id = f"overview:{workflow['id']}"
        used.add(flow_id)
        selected = list(dict.fromkeys(item for node_id in matches for item in _reachable(node_id, by_id, edge_list, 160, adjacency)))
        result.append(
            {
                **workflow,
                "id": flow_id,
                "family": workflow["id"],
                "kind": "overview",
                "trigger": "semantic terms",
                "entry_nodes": matches[:8],
                "nodes": selected[:160],
                "phases": _phase_roles(selected, by_id),
                "gaps": [],
                "detail": {"legacy_id": workflow["id"]},
            }
        )

    result.sort(key=lambda item: (str(item.get("family", "")), str(item.get("title", "")), str(item.get("id", ""))))
    return result


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
