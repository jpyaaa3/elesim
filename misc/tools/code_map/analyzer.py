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
from .semantics import build_flow_catalog, build_workflows, semantic_nodes_and_edges


ROLE_ROOTS = {"pilot", "sim", "ui", "robot", "packages", "installer", "model", "misc", "tests"}
CALLBACK_CALLS = {"submit", "create_task", "add_done_callback", "call_soon", "call_later", "run_in_executor"}
WIDGET_CALLS = {
    "button", "small_button", "arrow_button", "checkbox", "radio_button", "selectable",
    "slider_float", "slider_int", "drag_float", "drag_int", "input_text", "input_float",
    "input_int", "combo", "begin_combo", "menu_item", "invisible_button",
}


def _git(root: Path, *args: str, check: bool = True) -> str:
    result = subprocess.run(
        ("git", *args), cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
    )
    if check and result.returncode:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout


def _role(path: str) -> str:
    parts = Path(path).parts
    if len(parts) >= 4 and parts[:3] == ("payload", "runtime", "docker"):
        return parts[3] if parts[3] in {"pilot", "sim", "ui"} else "root"
    if len(parts) >= 4 and parts[:3] == ("payload", "runtime", "native"):
        return parts[3] if parts[3] == "robot" else "root"
    if len(parts) >= 3 and parts[:3] == ("payload", "runtime", "common"):
        return "packages"
    if len(parts) >= 3 and parts[:2] == ("tests", "roles"):
        return parts[2] if parts[2] in {"pilot", "sim", "ui", "robot"} else "root"
    if len(parts) >= 2 and parts[:2] == ("tests", "protocol"):
        return "packages"
    first = parts[0] if parts else ""
    return first if first in ROLE_ROOTS else "root"


def _module_name(path: str) -> str:
    parts = list(Path(path).with_suffix("").parts)
    if len(parts) >= 5 and parts[:3] == ["payload", "runtime", "docker"]:
        parts = parts[5:] if parts[4] == "app" else parts[4:]
    elif len(parts) >= 5 and parts[:3] == ["payload", "runtime", "native"]:
        parts = parts[5:] if parts[4] == "app" else parts[4:]
    elif len(parts) >= 4 and parts[:3] == ["payload", "runtime", "common"]:
        parts = parts[4:] if parts[3] == "protocol" else parts[3:]
    elif "src" in parts:
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


def _annotation(node: ast.AST | None) -> str:
    if node is None:
        return ""
    try:
        return ast.unparse(node)
    except (AttributeError, TypeError):
        return ""


def _target_names(node: ast.AST) -> list[str]:
    if isinstance(node, (ast.Tuple, ast.List)):
        result: list[str] = []
        for item in node.elts:
            result.extend(_target_names(item))
        return result
    name = _name(node)
    return [name] if name else []


def _literal_value(node: ast.AST | None) -> Any:
    if node is None:
        return None
    if isinstance(node, ast.Constant) and isinstance(node.value, (str, int, float, bool, type(None))):
        return node.value
    if isinstance(node, (ast.Tuple, ast.List, ast.Set)):
        values = [_literal_value(item) for item in node.elts]
        return values if all(value is not None for value in values) else None
    return ""


def _argument_defaults(args: ast.arguments) -> list[tuple[ast.arg, ast.AST | None]]:
    positional = [*args.posonlyargs, *args.args]
    missing = len(positional) - len(args.defaults)
    result = [(argument, None) for argument in positional[:missing]]
    result.extend(zip(positional[missing:], args.defaults))
    result.extend((argument, default) for argument, default in zip(args.kwonlyargs, args.kw_defaults))
    return result


def _widget_id(label: str, kind: str) -> str:
    value = label.split("##", 1)[1] if "##" in label else label
    value = re.sub(r"[^A-Za-z0-9_.:-]+", "-", value.strip()).strip("-")
    return value or kind


def _widget_label(node: ast.AST | None) -> tuple[str, bool]:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value, False
    if isinstance(node, ast.JoinedStr):
        parts: list[str] = []
        for value in node.values:
            if isinstance(value, ast.Constant) and isinstance(value.value, str):
                parts.append(value.value)
            elif isinstance(value, ast.FormattedValue):
                parts.append("{" + (_annotation(value.value) or "value") + "}")
        return "".join(parts), True
    if isinstance(node, ast.IfExp):
        body, _ = _widget_label(node.body)
        otherwise, _ = _widget_label(node.orelse)
        labels = [value.split("##", 1)[0].strip() for value in (body, otherwise)]
        return " / ".join(value for value in labels if value), True
    return "", True


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
        self.control_stack: list[dict[str, Any]] = []

    def current_id(self) -> str:
        qualname = ".".join([self.module, *self.stack]) if self.stack else self.module
        return _node_id(self.role, self.path, qualname)

    def add_edge(self, target: str, kind: str, line: int, confidence: str = "unresolved") -> None:
        self.add_edge_detail(target, kind, line, confidence=confidence)

    def add_edge_detail(
        self,
        target: str,
        kind: str,
        line: int,
        confidence: str = "unresolved",
        detail: dict[str, Any] | None = None,
    ) -> None:
        if target:
            self.raw_edges.append(
                {
                    "source": self.current_id(),
                    "target_name": target,
                    "kind": kind,
                    "line": line,
                    "confidence": confidence,
                    "detail": detail or {},
                }
            )

    def current_detail(self) -> dict[str, Any] | None:
        current = self.current_id()
        for node in reversed(self.nodes):
            if node.id == current:
                return node.detail
        return None

    def record(self, key: str, value: Any) -> None:
        detail = self.current_detail()
        if detail is None:
            return
        detail.setdefault(key, []).append(value)

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
            all_args = (*args.posonlyargs, *args.args, *args.kwonlyargs)
            detail["parameters"] = [arg.arg for arg in all_args]
            detail["parameter_ports"] = [
                {
                    "name": arg.arg,
                    "annotation": _annotation(arg.annotation),
                    "default": _literal_value(default),
                    "kind": "kwonly" if arg in args.kwonlyargs else "positional",
                }
                for arg, default in _argument_defaults(args)
            ]
            detail["return_annotation"] = _annotation(getattr(node, "returns", None))
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
        call_detail = {
            "arguments": [_name(argument) or _literal_value(argument) for argument in node.args],
            "keywords": {
                keyword.arg or "**": _name(keyword.value) or _literal_value(keyword.value)
                for keyword in node.keywords
            },
            "control": [dict(item) for item in self.control_stack],
        }
        self.add_edge_detail(called, "call", node.lineno, detail=call_detail)
        if called.startswith("imgui.") and leaf in WIDGET_CALLS:
            label_node = node.args[0] if node.args else None
            label, dynamic = _widget_label(label_node)
            self.record(
                "ui_widgets",
                {
                    "kind": leaf,
                    "label": str(label or ""),
                    "id": _widget_id(str(label or ""), leaf),
                    "expression": _annotation(label_node),
                    "dynamic": dynamic,
                    "control": [dict(item) for item in self.control_stack],
                    "line": node.lineno,
                },
            )
        if called and (
            called.startswith("service.")
            or called.startswith("panel.service.")
            or ".service." in called
        ):
            self.record("operator_calls", called.rsplit(".", 1)[-1])
        self.generic_visit(node)

    def visit_Return(self, node: ast.Return) -> None:
        self.record(
            "return_sites",
            {
                "line": node.lineno,
                "value": _name(node.value) if node.value is not None else "",
                "literal": _literal_value(node.value) if node.value is not None else None,
                "control": [dict(item) for item in self.control_stack],
            },
        )
        self.generic_visit(node)

    def visit_Raise(self, node: ast.Raise) -> None:
        value = _name(node.exc) if node.exc is not None else ""
        self.record(
            "raise_sites",
            {"line": node.lineno, "value": value, "control": [dict(item) for item in self.control_stack]},
        )
        self.add_edge_detail(
            value or "raise",
            "exception",
            node.lineno,
            detail={"control": [dict(item) for item in self.control_stack]},
        )
        self.generic_visit(node)

    def visit_Assign(self, node: ast.Assign) -> None:
        names = [name for target in node.targets for name in _target_names(target)]
        control = [dict(item) for item in self.control_stack]
        self.record("assignments", {"line": node.lineno, "targets": names, "value": _name(node.value), "control": control})
        for name in names:
            self.add_edge_detail(name, "data-write", node.lineno, confidence="inferred", detail={"targets": names, "control": control})
        self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> None:
        names = _target_names(node.target)
        control = [dict(item) for item in self.control_stack]
        self.record("assignments", {"line": node.lineno, "targets": names, "value": _name(node.value), "control": control})
        for name in names:
            self.add_edge_detail(name, "data-write", node.lineno, confidence="inferred", detail={"targets": names, "control": control})
        self.generic_visit(node)

    def visit_If(self, node: ast.If) -> None:
        self.record("branches", {"line": node.lineno, "test": _name(node.test) or _literal_value(node.test), "control": [dict(item) for item in self.control_stack]})
        self.visit(node.test)
        self.control_stack.append({"kind": "branch", "line": node.lineno, "arm": "body"})
        for child in node.body:
            self.visit(child)
        self.control_stack.pop()
        if node.orelse:
            self.control_stack.append({"kind": "branch", "line": node.lineno, "arm": "else"})
            for child in node.orelse:
                self.visit(child)
            self.control_stack.pop()

    def visit_For(self, node: ast.For) -> None:
        self.record("loops", {"kind": "for", "line": node.lineno, "target": _name(node.target), "control": [dict(item) for item in self.control_stack]})
        self.visit(node.target)
        self.visit(node.iter)
        self.control_stack.append({"kind": "loop", "line": node.lineno})
        for child in node.body:
            self.visit(child)
        self.control_stack.pop()
        for child in node.orelse:
            self.visit(child)

    visit_AsyncFor = visit_For

    def visit_While(self, node: ast.While) -> None:
        self.record("loops", {"kind": "while", "line": node.lineno, "test": _name(node.test), "control": [dict(item) for item in self.control_stack]})
        self.visit(node.test)
        self.control_stack.append({"kind": "loop", "line": node.lineno})
        for child in node.body:
            self.visit(child)
        self.control_stack.pop()
        for child in node.orelse:
            self.visit(child)


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
        detail = dict(item.get("detail", {}))
        if confidence == "unresolved":
            detail["resolution_gap"] = target_name
        if violation:
            detail["boundary_violation"] = True
        edges.append(
            Edge(
                item["source"], target, item["kind"], "static", confidence,
                path=_path_from_id(item["source"]), line=item["line"],
                detail=detail,
            )
        )
    unique: dict[str, Edge] = {edge.id: edge for edge in edges if edge.source != edge.target}
    return list(unique.values())


def _path_from_id(node_id: str) -> str:
    parts = node_id.split(":", 2)
    return parts[1] if len(parts) == 3 else ""


def _external_nodes(edges: Iterable[Edge]) -> list[Node]:
    targets = sorted({edge.target for edge in edges if edge.target.startswith("external:")})
    return [
        Node(
            target,
            "external",
            target.removeprefix("external:"),
            target.removeprefix("external:"),
            "",
            "boundary",
            detail={"resolution_gap": True},
        )
        for target in targets
    ]


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
                    flows=cached.get("flows", []),
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
    nodes.extend(_external_nodes(edges))
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
        flows=build_flow_catalog(nodes, edges),
    )
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_file.write_text(json.dumps(snapshot.as_dict(), ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    return snapshot
