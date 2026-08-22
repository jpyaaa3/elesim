"""AST-only analyzer for the current EleSim worktree."""

from __future__ import annotations

import ast
import hashlib
import json
import re
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from .model import Edge, Node, SCHEMA_VERSION, Snapshot
from .semantics import build_workflows, semantic_nodes_and_edges


ROLE_ROOTS = {"pilot", "sim", "ui", "robot", "packages", "installer", "model", "misc"}
CALLBACK_CALLS = {"submit", "create_task", "add_done_callback", "call_soon", "call_later", "run_in_executor"}


def _git(root: Path, *args: str, check: bool = True) -> str:
    result = subprocess.run(
        ("git", *args), cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
    )
    if check and result.returncode:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout


def _role(path: str) -> str:
    first = path.split("/", 1)[0]
    return first if first in ROLE_ROOTS else "root"


def _module_name(path: str) -> str:
    parts = list(Path(path).with_suffix("").parts)
    if "src" in parts:
        parts = parts[parts.index("src") + 1 :]
    if parts and parts[-1] == "__init__":
        parts.pop()
    return ".".join(parts)


def _node_id(role: str, path: str, qualname: str) -> str:
    return f"{role}:{path}:{qualname}"


def _name(expr: ast.AST) -> str:
    if isinstance(expr, ast.Name):
        return expr.id
    if isinstance(expr, ast.Attribute):
        parent = _name(expr.value)
        return f"{parent}.{expr.attr}" if parent else expr.attr
    if isinstance(expr, ast.Call):
        return _name(expr.func)
    if isinstance(expr, ast.Subscript):
        return _name(expr.value)
    return ""


def _literal_strings(tree: ast.AST, limit: int = 16) -> list[str]:
    values: list[str] = []
    for item in ast.walk(tree):
        if isinstance(item, ast.Constant) and isinstance(item.value, str):
            value = item.value.strip()
            if value and len(value) <= 96:
                values.append(value)
        if len(values) >= limit:
            break
    return list(dict.fromkeys(values))


class _Visitor(ast.NodeVisitor):
    def __init__(self, path: str, change: str) -> None:
        self.path = path
        self.role = _role(path)
        self.module = _module_name(path)
        self.change = change
        self.stack: list[str] = []
        self.owner_stack: list[str] = []
        self.nodes: list[Node] = []
        self.raw_edges: list[dict[str, Any]] = []
        self.imports: dict[str, str] = {}

    def current_id(self) -> str:
        qualname = ".".join([self.module, *self.stack]) if self.stack else self.module
        return _node_id(self.role, self.path, qualname)

    def add_edge(self, target: str, kind: str, line: int, confidence: str = "unresolved") -> None:
        if target:
            self.raw_edges.append(
                {
                    "source": self.current_id(),
                    "target_name": target,
                    "kind": kind,
                    "line": line,
                    "confidence": confidence,
                }
            )

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            local = alias.asname or alias.name.split(".")[0]
            self.imports[local] = alias.name
            self.add_edge(alias.name, "import", node.lineno, "exact")

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        base = "." * node.level + (node.module or "")
        for alias in node.names:
            target = f"{base}.{alias.name}".strip(".")
            self.imports[alias.asname or alias.name] = target
            self.add_edge(target, "import", node.lineno, "exact")

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        self._visit_definition(node, "class")

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        self._visit_definition(node, "method" if self.owner_stack else "function")

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        self._visit_definition(node, "method" if self.owner_stack else "function", is_async=True)

    def _visit_definition(self, node: ast.AST, kind: str, is_async: bool = False) -> None:
        name = str(getattr(node, "name"))
        self.stack.append(name)
        qualname = ".".join([self.module, *self.stack])
        decorators = [_name(item) for item in getattr(node, "decorator_list", [])]
        detail = {"decorators": decorators, "async": is_async}
        if kind == "class":
            detail["bases"] = [_name(item) for item in getattr(node, "bases", [])]
        else:
            args = getattr(node, "args")
            detail["parameters"] = [arg.arg for arg in (*args.posonlyargs, *args.args, *args.kwonlyargs)]
            detail["strings"] = _literal_strings(node)
        self.nodes.append(
            Node(
                _node_id(self.role, self.path, qualname), kind, name, qualname, self.path, self.role,
                getattr(node, "lineno", 1),
                getattr(node, "end_lineno", getattr(node, "lineno", 1)),
                self.change,
                detail,
            )
        )
        for decorator in decorators:
            self.add_edge(decorator, "decorator", getattr(node, "lineno", 1))
        if kind == "class":
            for base in detail["bases"]:
                self.add_edge(base, "inherits", getattr(node, "lineno", 1))
            self.owner_stack.append(name)
        for child in getattr(node, "body", []):
            self.visit(child)
        if kind == "class":
            self.owner_stack.pop()
        self.stack.pop()

    def visit_Call(self, node: ast.Call) -> None:
        called = _name(node.func)
        kind = "call"
        leaf = called.rsplit(".", 1)[-1]
        if leaf in CALLBACK_CALLS:
            kind = "async-task" if leaf in {"submit", "create_task", "run_in_executor"} else "callback"
            self.add_edge(called, kind, node.lineno)
            candidates = [*node.args, *(keyword.value for keyword in node.keywords if keyword.arg == "target")]
            for candidate in candidates[:2]:
                target = _name(candidate)
                if target:
                    self.add_edge(target, kind, node.lineno)
        elif leaf in {"Thread", "Process"}:
            for keyword in node.keywords:
                if keyword.arg == "target":
                    self.add_edge(_name(keyword.value), "thread", node.lineno)
        self.add_edge(called, "call", node.lineno)
        self.generic_visit(node)


def _parse(path: str, source: str, change: str) -> tuple[list[Node], list[dict[str, Any]]]:
    role = _role(path)
    module = _module_name(path)
    module_id = _node_id(role, path, module)
    try:
        tree = ast.parse(source, filename=path)
    except (SyntaxError, ValueError) as exc:
        unparsed = Node(
            module_id,
            "unparsed",
            Path(path).name,
            module,
            path,
            role,
            change=change,
            detail={"error": str(exc)},
        )
        return [unparsed], []
    visitor = _Visitor(path, change)
    visitor.nodes.append(
        Node(module_id, "module", Path(path).name, module, path, role, 1, max(1, source.count("\n") + 1), change)
    )
    visitor.visit(tree)
    return visitor.nodes, visitor.raw_edges


def _changes(root: Path) -> dict[str, str]:
    mapping = {"A": "added", "M": "modified", "D": "deleted", "R": "modified", "C": "added"}
    result: dict[str, str] = {}
    for line in _git(root, "diff", "--name-status", "HEAD", "--", "*.py").splitlines():
        fields = line.split("\t")
        if len(fields) >= 2:
            result[fields[-1]] = mapping.get(fields[0][0], "modified")
    tracked = set(_git(root, "ls-files", "--", "*.py").splitlines())
    for path in _git(root, "ls-files", "--others", "--exclude-standard", "--", "*.py").splitlines():
        if path not in tracked:
            result[path] = "added"
    return result


def _resolve_edges(nodes: list[Node], raw: Iterable[dict[str, Any]]) -> list[Edge]:
    by_qualname = {node.qualname: node.id for node in nodes}
    by_short: dict[str, list[str]] = {}
    module_ids = {node.qualname: node.id for node in nodes if node.kind == "module"}
    for node in nodes:
        by_short.setdefault(node.name, []).append(node.id)
    edges: list[Edge] = []
    for item in raw:
        target_name = item["target_name"]
        target = by_qualname.get(target_name) or module_ids.get(target_name)
        confidence = item["confidence"]
        if target is None:
            short = target_name.rsplit(".", 1)[-1]
            candidates = by_short.get(short, [])
            if len(candidates) == 1:
                target, confidence = candidates[0], "inferred"
            else:
                target = f"external:{target_name}"
                confidence = "unresolved"
        source_role = item["source"].split(":", 1)[0]
        target_role = target.split(":", 1)[0] if ":" in target and not target.startswith("external:") else ""
        deployment_roles = {"pilot", "sim", "ui", "robot"}
        violation = (
            item["kind"] == "import"
            and confidence == "exact"
            and source_role in deployment_roles
            and target_role in deployment_roles
            and source_role != target_role
        )
        if item["kind"] == "call" and confidence == "unresolved":
            continue
        edges.append(
            Edge(
                item["source"], target, item["kind"], "static", confidence,
                path=_path_from_id(item["source"]), line=item["line"],
                detail={"boundary_violation": violation} if violation else {},
            )
        )
    unique: dict[str, Edge] = {edge.id: edge for edge in edges if edge.source != edge.target}
    return list(unique.values())


def _path_from_id(node_id: str) -> str:
    parts = node_id.split(":", 2)
    return parts[1] if len(parts) == 3 else ""


def _head(root: Path) -> str:
    return _git(root, "rev-parse", "HEAD").strip()


def _structure(nodes: list[Node]) -> tuple[list[Node], list[Edge]]:
    additions: dict[str, Node] = {}
    edges: list[Edge] = []
    for node in nodes:
        if not node.path or node.kind != "module":
            continue
        role_id = f"role:{node.role}"
        package = node.qualname.split(".", 1)[0] or node.role
        package_id = f"package:{node.role}:{package}"
        additions.setdefault(role_id, Node(role_id, "role", node.role, node.role, "", node.role))
        additions.setdefault(package_id, Node(package_id, "package", package, package, "", node.role))
        edges.append(Edge(role_id, package_id, "contains", "declared", "exact"))
        edges.append(Edge(package_id, node.id, "contains", "declared", "exact"))
    return list(additions.values()), list({edge.id: edge for edge in edges}.values())


def _entrypoints(root: Path, nodes: list[Node]) -> tuple[list[Node], list[Edge]]:
    by_qualname = {node.qualname: node.id for node in nodes}
    additions: list[Node] = []
    edges: list[Edge] = []
    for pyproject in sorted(root.glob("**/pyproject.toml")):
        if any(part in {".git", ".elesim", "dist", "build"} for part in pyproject.parts):
            continue
        try:
            text = pyproject.read_text(encoding="utf-8")
        except (OSError, UnicodeError):
            continue
        relative = pyproject.relative_to(root).as_posix()
        role = _role(relative)
        scripts: dict[str, str] = {}
        in_scripts = False
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("["):
                in_scripts = stripped == "[project.scripts]"
                continue
            if not in_scripts or not stripped or stripped.startswith("#"):
                continue
            match = re.match(r'([A-Za-z0-9_.-]+)\s*=\s*["\']([^"\']+)["\']\s*$', stripped)
            if match:
                scripts[match.group(1)] = match.group(2)
        for command, reference in scripts.items():
            target_name = str(reference).replace(":", ".")
            entry_id = f"entrypoint:{relative}:{command}"
            additions.append(
                Node(entry_id, "entrypoint", command, command, relative, role, detail={"target": reference})
            )
            target = by_qualname.get(target_name)
            if target is None:
                matches = [node.id for node in nodes if node.qualname.endswith(target_name)]
                target = matches[0] if len(matches) == 1 else f"external:{target_name}"
            confidence = "unresolved" if target.startswith("external:") else "exact"
            edges.append(Edge(entry_id, target, "entrypoint", "declared", confidence, relative))
    return additions, edges


def _mark_orphans(nodes: list[Node], edges: list[Edge]) -> None:
    incoming_kinds = {"call", "callback", "thread", "async-task", "entrypoint"}
    incoming = {edge.target for edge in edges if edge.kind in incoming_kinds}
    for node in nodes:
        if node.kind not in {"function", "class"} or node.id in incoming:
            continue
        if "/tests/" in f"/{node.path}" or node.name.startswith("_") or node.detail.get("decorators"):
            continue
        node.detail["orphan_candidate"] = True


def analyze_repository(root: Path, *, use_cache: bool = True) -> Snapshot:
    root = root.resolve()
    changes = _changes(root)
    listed = _git(root, "ls-files", "--cached", "--others", "--exclude-standard", "--", "*.py")
    paths = sorted(set(listed.splitlines()))
    git_head = _head(root)
    digest_builder = hashlib.sha256(f"schema:{SCHEMA_VERSION}\nhead:{git_head}\n".encode())
    sources: list[tuple[str, str, str]] = []
    for path in paths:
        candidate = root / path
        if not candidate.is_file() or candidate.is_symlink():
            continue
        try:
            source = candidate.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            source = ""
        change = changes.get(path, "unchanged")
        digest_builder.update(path.encode())
        digest_builder.update(b"\0")
        digest_builder.update(source.encode())
        sources.append((path, source, change))
    digest = digest_builder.hexdigest()
    cache_dir = root / ".elesim" / "analysis" / "code-map"
    cache_file = cache_dir / "snapshot.json"
    if use_cache and cache_file.is_file():
        try:
            cached = json.loads(cache_file.read_text(encoding="utf-8"))
            if cached.get("digest") == digest and cached.get("schema_version") == SCHEMA_VERSION:
                return Snapshot(
                    digest=cached["digest"],
                    git_head=cached["git_head"],
                    generated_at=cached["generated_at"],
                    nodes=[Node(**node) for node in cached["nodes"]],
                    edges=[
                        Edge(**{key: value for key, value in edge.items() if key != "id"})
                        for edge in cached["edges"]
                    ],
                    workflows=cached["workflows"],
                    stats=cached["stats"],
                )
        except (OSError, ValueError, TypeError, KeyError):
            pass
    nodes: list[Node] = []
    raw_edges: list[dict[str, Any]] = []
    for path, source, change in sources:
        parsed_nodes, parsed_edges = _parse(path, source, change)
        nodes.extend(parsed_nodes)
        raw_edges.extend(parsed_edges)
    for path, change in changes.items():
        if change == "deleted" and path.endswith(".py"):
            module = _module_name(path)
            nodes.append(
                Node(
                    _node_id(_role(path), path, module),
                    "module",
                    Path(path).name,
                    module,
                    path,
                    _role(path),
                    change="deleted",
                )
            )
    structure_nodes, structure_edges = _structure(nodes)
    nodes.extend(structure_nodes)
    entry_nodes, entry_edges = _entrypoints(root, nodes)
    nodes.extend(entry_nodes)
    semantic_nodes, semantic_edges = semantic_nodes_and_edges(nodes)
    nodes.extend(semantic_nodes)
    edges = _resolve_edges(nodes, raw_edges) + structure_edges + entry_edges + semantic_edges
    _mark_orphans(nodes, edges)
    snapshot = Snapshot(
        digest=digest,
        git_head=git_head,
        generated_at=datetime.now(timezone.utc).isoformat(),
        nodes=nodes,
        edges=edges,
        workflows=build_workflows(nodes),
        stats={
            "files": len(sources), "nodes": len(nodes), "edges": len(edges),
            "unparsed": sum(node.kind == "unparsed" for node in nodes),
            "changed": sum(node.change != "unchanged" for node in nodes),
        },
    )
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file.write_text(json.dumps(snapshot.as_dict(), ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return snapshot
