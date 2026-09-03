"""Loopback-only HTTP surface for the code map."""

from __future__ import annotations

import gzip
import hashlib
import hmac
import json
import mimetypes
import secrets
import subprocess
import time
import urllib.parse
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .analyzer import analyze_repository
from .model import Edge


MAX_SOURCE_BYTES = 128 * 1024
MAX_SOURCE_LINES = 500
_FLOW_EDGE_KINDS = frozenset(
    {
        "call",
        "callback",
        "async-task",
        "thread",
        "entrypoint",
        "contract",
        "data-write",
        "state-write",
        "exception",
        "return",
    }
)
_FLOW_VIEWS = frozenset({"overview", "spine", "full"})
_BOUNDARY_EDGE_KINDS = frozenset({"callback", "async-task", "thread", "entrypoint"})
_DETAIL_EDGE_KINDS = frozenset({"data-write", "state-write", "return"})
_RUNTIME_ROLES = frozenset({"pilot", "sim", "ui", "robot"})
MAX_FLOW_EXPANSIONS = 32
CSP = (
    "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; "
    "img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'"
)


def _inside(root: Path, relative: str) -> Path:
    if not relative or Path(relative).is_absolute():
        raise ValueError("a repository-relative path is required")
    candidate = root / relative
    if candidate.is_symlink():
        raise ValueError("symlink source paths are not readable")
    unresolved = candidate.resolve(strict=False)
    if root != unresolved and root not in unresolved.parents:
        raise ValueError("path escapes repository")
    try:
        resolved = candidate.resolve(strict=True)
    except FileNotFoundError as exc:
        raise ValueError("path does not exist") from exc
    if root != resolved and root not in resolved.parents:
        raise ValueError("path escapes repository")
    if not resolved.is_file():
        raise ValueError("path is not a file")
    return resolved


def _source(root: Path, path: str, line: int) -> dict[str, Any]:
    candidate = _inside(root, path)
    raw = candidate.read_bytes()
    if len(raw) > MAX_SOURCE_BYTES:
        raise ValueError(f"source exceeds {MAX_SOURCE_BYTES} bytes")
    text = raw.decode("utf-8")
    lines = text.splitlines()
    center = max(1, line)
    start = max(1, center - 80)
    end = min(len(lines), start + MAX_SOURCE_LINES - 1)
    return {"path": path, "start": start, "end": end, "text": "\n".join(lines[start - 1 : end])}


def _diff(root: Path, path: str) -> str:
    _inside(root, path)
    result = subprocess.run(
        ("git", "diff", "--no-ext-diff", "--unified=3", "HEAD", "--", path),
        cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if result.returncode:
        raise ValueError(result.stderr.strip() or "git diff failed")
    return result.stdout[:MAX_SOURCE_BYTES]


def _flow_layers(
    entry_ids: list[str],
    ordered_ids: list[str],
    edges: list[Any],
    *,
    direction: str,
) -> tuple[dict[str, int], dict[str, int]]:
    """Return deterministic directed-flow depth and order metadata.

    ELK can make a graph readable, but it cannot infer which edges are the
    execution spine when a snapshot also contains imports and inheritance.
    Prefer control/data edges for the layer calculation and fall back to the
    complete subgraph when a flow has no such edges.  Upstream views receive
    negative depths so the original source-to-sink direction still reads
    top-to-bottom.
    """

    selected = set(ordered_ids)
    candidates = [
        edge
        for edge in edges
        if edge.source in selected
        and edge.target in selected
        and (edge.kind in _FLOW_EDGE_KINDS or edge.kind == "collapsed")
    ]
    if not candidates:
        candidates = [
            edge
            for edge in edges
            if edge.source in selected and edge.target in selected
        ]
    adjacency: dict[str, list[str]] = {}
    for edge in candidates:
        source, target = (
            (edge.target, edge.source)
            if direction == "upstream"
            else (edge.source, edge.target)
        )
        adjacency.setdefault(source, []).append(target)
    for source in tuple(adjacency):
        adjacency[source] = sorted(set(adjacency[source]))

    depths: dict[str, int] = {}
    order: dict[str, int] = {}
    queue: list[tuple[str, int]] = [
        (entry, 0) for entry in entry_ids if entry in selected
    ]
    while queue:
        current, depth = queue.pop(0)
        if current in depths:
            continue
        depths[current] = depth
        order[current] = len(order)
        queue.extend(
            (target, depth + 1)
            for target in adjacency.get(current, ())
            if target not in depths
        )

    if direction == "upstream":
        depths = {node_id: -depth for node_id, depth in depths.items()}
        # The traversal starts at the selected sink, but the rendered graph
        # still reads source-to-sink from top to bottom.  Re-number steps in
        # that visual order so labels do not count backwards along the wire.
        order = {
            node_id: index
            for index, node_id in enumerate(
                sorted(depths, key=lambda node_id: (depths[node_id], order[node_id], node_id))
            )
        }

    if depths:
        fallback = max(depths.values()) + 1 if direction != "upstream" else min(depths.values()) - 1
    else:
        fallback = 0
    for node_id in ordered_ids:
        if node_id not in depths:
            depths[node_id] = fallback
            fallback += 1 if direction != "upstream" else -1
        if node_id not in order:
            order[node_id] = len(order)
    return depths, order


def _node_phase(node: Any) -> str:
    haystack = " ".join((str(node.name), str(node.qualname), str(node.detail))).lower()
    rules = (
        ("initialize", ("init", "startup", "discover", "connect", "open", "create")),
        ("input", ("receive", "capture", "read", "subscriber", "intent", "request")),
        ("output", ("publish", "send", "write", "reply", "result", "ack")),
        ("cleanup", ("close", "shutdown", "cleanup", "stop", "release", "revoke")),
    )
    return next((phase for phase, terms in rules if any(term in haystack for term in terms)), "process")


def _collapsed_id(flow_id: str, members: set[str]) -> str:
    digest = hashlib.sha256("\0".join(sorted(members)).encode()).hexdigest()[:16]
    return f"collapsed:{flow_id}:{digest}"


def _edge_controls(edge: Any) -> set[str]:
    detail = edge.detail if isinstance(edge.detail, dict) else {}
    controls = detail.get("control", [])
    return {
        str(item.get("kind"))
        for item in controls
        if isinstance(item, dict) and item.get("kind")
    }


def _ui_surface_title(path: str) -> str:
    stem = Path(path).stem.replace("_", " ").strip()
    aliases = {
        "control 4dof": "4-DOF Control",
        "control panel": "Control Panel",
        "go2": "GO2",
        "ik": "IK",
        "live visual status": "Live Visual Status",
        "sag": "Sag Compensation",
        "sim view": "Simulation",
    }
    return aliases.get(stem, stem.title() or "UI")


def _ui_map(snapshot: Any) -> dict[str, Any]:
    """Return a read-only interaction twin derived from action-flow metadata."""

    by_id = {node.id: node for node in snapshot.nodes}
    surfaces: dict[str, dict[str, Any]] = {}
    sections: dict[tuple[str, str], dict[str, Any]] = {}
    controls = 0
    dynamic = 0
    for flow in snapshot.flows:
        if flow.get("kind") != "action":
            continue
        detail = flow.get("detail", {})
        widget = detail.get("widget", {}) if isinstance(detail, dict) else {}
        template = detail.get("template") if isinstance(detail, dict) else None
        node = by_id.get(template)
        if node is None or not isinstance(widget, dict) or node.role != "ui":
            continue
        path_parts = Path(node.path).parts
        if "test" in path_parts or "tests" in path_parts:
            continue
        surface = surfaces.setdefault(
            node.path,
            {
                "id": f"ui-surface:{node.path}",
                "title": _ui_surface_title(node.path),
                "path": node.path,
                "helper": Path(node.path).name == "helpers.py",
                "sections": [],
            },
        )
        section_key = (node.path, node.id)
        section = sections.get(section_key)
        if section is None:
            section = {
                "id": f"ui-section:{node.id}",
                "title": node.name,
                "qualname": node.qualname,
                "line": node.line,
                "controls": [],
            }
            sections[section_key] = section
            surface["sections"].append(section)
        raw_label = str(widget.get("label") or "")
        expression = str(widget.get("expression") or "")
        widget_id = str(widget.get("id") or widget.get("kind") or "control")
        label = raw_label.split("##", 1)[0].strip()
        if not label:
            label = " ".join(widget_id.replace("_", " ").replace("-", " ").split()) if raw_label else expression or widget_id
        is_dynamic = bool(widget.get("dynamic"))
        control = {
            "id": f"ui-control:{flow['id']}",
            "workflow_id": flow["id"],
            "family": flow.get("family", "operator"),
            "kind": str(widget.get("kind") or "button"),
            "label": label,
            "widget_id": widget_id,
            "expression": expression,
            "dynamic": is_dynamic,
            "conditional": bool(widget.get("control")),
            "condition": widget.get("control", []),
            "path": node.path,
            "qualname": node.qualname,
            "line": int(widget.get("line") or node.line),
        }
        section["controls"].append(control)
        controls += 1
        dynamic += int(is_dynamic)
    payload = sorted(surfaces.values(), key=lambda item: (item["helper"], item["title"], item["path"]))
    for surface in payload:
        surface["sections"].sort(key=lambda item: (item["line"], item["title"]))
        for section in surface["sections"]:
            section["controls"].sort(key=lambda item: (item["line"], item["id"]))
    return {
        "schema_version": snapshot.schema_version,
        "surfaces": payload,
        "stats": {"surfaces": len(payload), "controls": controls, "dynamic": dynamic},
    }


def _flow_projection(
    flow_id: str,
    ordered_ids: list[str],
    edges: list[Any],
    by_id: dict[str, Any],
    entry_ids: set[str],
    view: str,
    expand: set[str],
) -> tuple[list[str], list[Any], list[dict[str, Any]]]:
    """Project a flow into a reversible, progressively disclosed graph."""

    selected = set(ordered_ids)
    selected_edges = [edge for edge in edges if edge.source in selected and edge.target in selected]
    if view == "full":
        if expand:
            raise ValueError("expand is not used by the full flow view")
        return ordered_ids, selected_edges, []

    control = [edge for edge in selected_edges if edge.kind in _FLOW_EDGE_KINDS and edge.kind not in _DETAIL_EDGE_KINDS]
    boundary_nodes: set[str] = set(entry_ids)
    for edge in control:
        source = by_id.get(edge.source)
        target = by_id.get(edge.target)
        crosses_runtime_role = (
            source
            and target
            and source.role in _RUNTIME_ROLES
            and target.role in _RUNTIME_ROLES
            and source.role != target.role
        )
        if edge.kind in _BOUNDARY_EDGE_KINDS or crosses_runtime_role:
            boundary_nodes.update((edge.source, edge.target))
        if view == "spine" and (edge.kind == "exception" or _edge_controls(edge) & {"branch", "loop"}):
            boundary_nodes.update((edge.source, edge.target))

    visible = set(boundary_nodes)
    for node_id in ordered_ids:
        node = by_id.get(node_id)
        if node is None:
            continue
        if node.kind in {"semantic", "entrypoint"}:
            visible.add(node_id)

    hidden = selected - visible
    undirected: dict[str, set[str]] = {node_id: set() for node_id in hidden}
    for edge in selected_edges:
        if edge.source in hidden and edge.target in hidden:
            undirected[edge.source].add(edge.target)
            undirected[edge.target].add(edge.source)
    components: list[set[str]] = []
    while hidden:
        root = min(hidden)
        component: set[str] = set()
        frontier = [root]
        while frontier:
            current = frontier.pop()
            if current in component:
                continue
            component.add(current)
            frontier.extend(sorted(undirected.get(current, ()), reverse=True))
        hidden -= component
        components.append(component)

    # Sibling leaf/helper components reached through the same visible boundary
    # are one disclosure unit.  This is the case that otherwise turns one
    # coordinator with many small callees into dozens of placeholder leaves.
    grouped: dict[tuple[tuple[str, ...], tuple[tuple[str, str], ...]], set[str]] = {}
    for component in components:
        roles = tuple(sorted({by_id[node_id].role for node_id in component if node_id in by_id}))
        boundary = tuple(sorted({
            ("in", edge.source) if edge.target in component else ("out", edge.target)
            for edge in selected_edges
            if (edge.source in component) != (edge.target in component)
        }))
        grouped.setdefault((roles, boundary), set()).update(component)
    components = list(grouped.values())

    component_ids = {_collapsed_id(flow_id, component) for component in components}
    unknown_expansions = expand - component_ids
    if unknown_expansions:
        raise ValueError("unknown flow expansion")

    collapsed: list[dict[str, Any]] = []
    expanded_members: set[str] = set()
    for component in components:
        placeholder_id = _collapsed_id(flow_id, component)
        if placeholder_id in expand:
            expanded_members.update(component)
            continue
        roles = sorted({by_id[node_id].role for node_id in component if node_id in by_id})
        collapsed.append({
            "id": placeholder_id,
            "kind": "collapsed",
            "name": f"{len(component)}개 세부 호출",
            "qualname": "collapsed flow segment",
            "path": "",
            "role": roles[0] if len(roles) == 1 else "flow",
            "line": 1,
            "end_line": 1,
            "change": "unchanged",
            "detail": {
                "collapsed_count": len(component),
                "members": sorted(component),
                "roles": roles,
                "expand_id": placeholder_id,
            },
            "members": component,
        })

    visible.update(expanded_members)
    output_edges: list[Any] = [
        edge for edge in selected_edges if edge.source in visible and edge.target in visible
    ]
    for placeholder in collapsed:
        members = placeholder["members"]
        boundary_edges: dict[tuple[str, str, str], Any] = {}
        for edge in selected_edges:
            source_inside, target_inside = edge.source in members, edge.target in members
            if source_inside == target_inside:
                continue
            source = placeholder["id"] if source_inside else edge.source
            target = placeholder["id"] if target_inside else edge.target
            if source not in visible and source != placeholder["id"]:
                continue
            if target not in visible and target != placeholder["id"]:
                continue
            boundary_edges.setdefault((source, target, edge.kind), edge)
        placeholder["boundary_edges"] = [
            Edge(
                source,
                target,
                "collapsed",
                edge.evidence,
                "inferred",
                edge.path,
                edge.line,
                {"collapsed_count": len(members), "original_kind": edge.kind},
            )
            for (source, target, _kind), edge in boundary_edges.items()
        ]
    collapsed_payload = [{key: value for key, value in item.items() if key not in {"members", "boundary_edges"}} for item in collapsed]
    synthetic_edges = [edge for item in collapsed for edge in item["boundary_edges"]]
    visible_order = [node_id for node_id in ordered_ids if node_id in visible]
    return visible_order, [*output_edges, *synthetic_edges], collapsed_payload


def _flow_graph(
    snapshot: Any,
    flow_id: str,
    direction: str = "both",
    budget: int = 500,
    view: str = "spine",
    expand: set[str] | None = None,
) -> dict[str, Any]:
    if view not in _FLOW_VIEWS:
        raise ValueError("view must be overview, spine or full")
    expand = set(expand or ())
    if len(expand) > MAX_FLOW_EXPANSIONS:
        raise ValueError(f"at most {MAX_FLOW_EXPANSIONS} flow expansions are allowed")
    flow = next((item for item in snapshot.flows if item.get("id") == flow_id), None)
    if flow is None:
        raise ValueError("unknown flow")
    ordered_flow_ids = list(dict.fromkeys(str(value) for value in flow.get("nodes", [])))
    node_ids = set(ordered_flow_ids)
    entry_ids = set(str(value) for value in flow.get("entry_nodes", []))
    if direction in {"upstream", "downstream"}:
        adjacency: dict[str, set[str]] = {}
        candidates = [
            edge
            for edge in snapshot.edges
            if edge.source in node_ids
            and edge.target in node_ids
            and edge.kind in _FLOW_EDGE_KINDS
        ]
        if not candidates:
            candidates = [
                edge
                for edge in snapshot.edges
                if edge.source in node_ids and edge.target in node_ids
            ]
        for edge in candidates:
            adjacency.setdefault(edge.source if direction == "downstream" else edge.target, set()).add(
                edge.target if direction == "downstream" else edge.source
            )
        frontier = [entry for entry in ordered_flow_ids if entry in entry_ids]
        frontier.extend(entry for entry in entry_ids if entry not in frontier)
        node_ids = set()
        traversal_order: list[str] = []
        while frontier and len(node_ids) < budget:
            current = frontier.pop(0)
            if current in node_ids:
                continue
            node_ids.add(current)
            traversal_order.append(current)
            frontier.extend(sorted(adjacency.get(current, set())))
    if direction in {"upstream", "downstream"}:
        ordered_ids = traversal_order
    else:
        ordered_ids = ordered_flow_ids
    ordered_ids = ordered_ids[:budget]
    by_id = {node.id: node for node in snapshot.nodes}
    projected_ids, edges, collapsed = _flow_projection(
        flow_id, ordered_ids, snapshot.edges, by_id, entry_ids, view, expand
    )
    node_ids = set(projected_ids)
    node_ids.update(node["id"] for node in collapsed)
    depth_by_id, order_by_id = _flow_layers(
        [str(value) for value in flow.get("entry_nodes", [])],
        [*projected_ids, *(node["id"] for node in collapsed)],
        edges,
        direction=direction,
    )
    nodes = []
    for node_id in projected_ids:
        node = by_id.get(node_id)
        if node is None:
            continue
        raw = dict(node.__dict__)
        raw["flow_depth"] = depth_by_id.get(node_id, 0)
        raw["flow_order"] = order_by_id.get(node_id, 0)
        raw["flow_entry"] = node_id in entry_ids
        raw["flow_phase"] = _node_phase(node)
        nodes.append(raw)
    for raw in collapsed:
        raw["flow_depth"] = depth_by_id.get(raw["id"], 0)
        raw["flow_order"] = order_by_id.get(raw["id"], 0)
        raw["flow_entry"] = False
        raw["flow_phase"] = "detail"
        nodes.append(raw)
    edges = edges[: budget * 4]
    flow_edges = [edge for edge in edges if edge.kind in _FLOW_EDGE_KINDS or edge.kind == "collapsed"]
    incoming = {node_id: 0 for node_id in node_ids}
    for edge in flow_edges:
        incoming[edge.target] = incoming.get(edge.target, 0) + 1
    edge_payload = []
    for edge in edges:
        raw = {**edge.__dict__, "id": getattr(edge, "id", f"{edge.source}|{edge.kind}|{edge.target}")}
        raw["flow_edge"] = edge.kind in _FLOW_EDGE_KINDS or edge.kind == "collapsed"
        if raw["flow_edge"]:
            controls = _edge_controls(edge)
            raw["flow_branch"] = "branch" in controls
            raw["flow_loop"] = "loop" in controls
            raw["flow_merge"] = incoming.get(edge.target, 0) > 1
            raw["flow_backedge"] = edge.source == edge.target or (
                depth_by_id.get(edge.target, 0) < depth_by_id.get(edge.source, 0)
            )
        edge_payload.append(raw)
    flow_payload = dict(flow)
    flow_payload["direction"] = direction
    flow_payload["layout"] = "directed"
    flow_payload["view"] = view
    flow_payload["collapsed"] = sum(node["detail"]["collapsed_count"] for node in collapsed)
    return {
        "flow": flow_payload,
        "nodes": nodes,
        "edges": edge_payload,
    }


class CodeMapServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], root: Path, token: str) -> None:
        if address[0] not in {"127.0.0.1", "::1", "localhost"}:
            raise ValueError("code map must bind to loopback")
        self.root = root.resolve()
        self.token = token
        self.web_root = Path(__file__).with_name("web")
        super().__init__(address, CodeMapHandler)


class CodeMapHandler(BaseHTTPRequestHandler):
    server: CodeMapServer

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"[code-map] {self.address_string()} {fmt % args}")

    def _query(self) -> dict[str, list[str]]:
        return urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)

    def _authorized(self) -> bool:
        query_token = self._query().get("token", [""])[0]
        header_token = self.headers.get("X-Code-Map-Token", "")
        cookie = SimpleCookie(self.headers.get("Cookie", ""))
        cookie_token = cookie.get("elesim_code_map")
        supplied = query_token or header_token or (cookie_token.value if cookie_token else "")
        return hmac.compare_digest(supplied, self.server.token)

    def _headers(
        self,
        status: int,
        content_type: str,
        length: int | None = None,
        *,
        cookie: bool = False,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", CSP)
        if cookie:
            self.send_header("Set-Cookie", f"elesim_code_map={self.server.token}; HttpOnly; SameSite=Strict; Path=/")
        if length is not None:
            self.send_header("Content-Length", str(length))
        self.end_headers()

    def _json(self, payload: Any, status: int = HTTPStatus.OK) -> None:
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
        compressed = "gzip" in self.headers.get("Accept-Encoding", "").lower() and len(raw) > 1024
        if compressed:
            raw = gzip.compress(raw, compresslevel=5)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", CSP)
        if compressed:
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _error(self, status: int, message: str) -> None:
        self._json({"error": message}, status)

    def do_POST(self) -> None:
        self._error(HTTPStatus.METHOD_NOT_ALLOWED, "read-only server")

    do_PUT = do_POST
    do_PATCH = do_POST
    do_DELETE = do_POST

    def do_GET(self) -> None:
        split = urllib.parse.urlsplit(self.path)
        if not self._authorized():
            self._error(HTTPStatus.FORBIDDEN, "invalid code-map token")
            return
        try:
            if split.path == "/api/snapshot":
                self._json(analyze_repository(self.server.root).as_dict())
            elif split.path == "/api/flows":
                snapshot = analyze_repository(self.server.root)
                self._json({
                    "schema_version": snapshot.schema_version,
                    "flows": snapshot.flows,
                    "stats": {"count": len(snapshot.flows), "actions": sum(item.get("kind") == "action" for item in snapshot.flows)},
                })
            elif split.path == "/api/ui-map":
                self._json(_ui_map(analyze_repository(self.server.root)))
            elif split.path == "/api/flow":
                query = self._query()
                flow_id = query.get("id", [""])[0]
                direction = query.get("direction", ["both"])[0]
                if direction not in {"both", "downstream", "upstream"}:
                    raise ValueError("direction must be both, downstream or upstream")
                budget = min(1000, max(1, int(query.get("budget", ["500"])[0])))
                view = query.get("view", ["spine"])[0]
                expand = {value for raw in query.get("expand", []) for value in raw.split(",") if value}
                self._json(_flow_graph(analyze_repository(self.server.root), flow_id, direction, budget, view, expand))
            elif split.path == "/api/source":
                query = self._query()
                path = query.get("path", [""])[0]
                line = int(query.get("line", ["1"])[0])
                self._json(_source(self.server.root, path, line))
            elif split.path == "/api/diff":
                path = self._query().get("path", [""])[0]
                self._json({"path": path, "text": _diff(self.server.root, path)})
            elif split.path == "/api/events":
                self._events()
            else:
                self._static(split.path, cookie=bool(self._query().get("token")))
        except (ValueError, OSError, UnicodeError) as exc:
            self._error(HTTPStatus.BAD_REQUEST, str(exc))

    def _events(self) -> None:
        self._headers(HTTPStatus.OK, "text/event-stream; charset=utf-8")
        digest = ""
        for _ in range(30):
            snapshot = analyze_repository(self.server.root)
            if snapshot.digest != digest:
                digest = snapshot.digest
                self.wfile.write(f"event: snapshot\ndata: {digest}\n\n".encode())
                self.wfile.flush()
            time.sleep(2)

    def _static(self, url_path: str, *, cookie: bool) -> None:
        relative = "index.html" if url_path in {"", "/"} else url_path.lstrip("/")
        candidate = _inside(self.server.web_root, relative)
        raw = candidate.read_bytes()
        content_type = mimetypes.guess_type(candidate.name)[0] or "application/octet-stream"
        self._headers(HTTPStatus.OK, content_type, len(raw), cookie=cookie)
        self.wfile.write(raw)


def serve(
    root: Path,
    host: str = "127.0.0.1",
    port: int = 0,
    token: str = "",
) -> None:
    actual_token = token or secrets.token_urlsafe(24)
    with CodeMapServer((host, port), root, actual_token) as server:
        actual_port = server.server_address[1]
        print(f"[code-map] http://{host}:{actual_port}/?token={actual_token}", flush=True)
        server.serve_forever()
