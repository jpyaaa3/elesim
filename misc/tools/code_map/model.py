"""Serializable code-map model."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


SCHEMA_VERSION = 1


@dataclass
class Node:
    id: str
    kind: str
    name: str
    qualname: str
    path: str
    role: str
    line: int = 1
    end_line: int = 1
    change: str = "unchanged"
    detail: dict[str, Any] = field(default_factory=dict)


@dataclass
class Edge:
    source: str
    target: str
    kind: str
    evidence: str = "static"
    confidence: str = "exact"
    path: str = ""
    line: int = 0
    detail: dict[str, Any] = field(default_factory=dict)

    @property
    def id(self) -> str:
        return f"{self.source}|{self.kind}|{self.target}|{self.path}:{self.line}"


@dataclass
class Snapshot:
    digest: str
    git_head: str
    generated_at: str
    nodes: list[Node]
    edges: list[Edge]
    workflows: list[dict[str, Any]]
    stats: dict[str, Any]
    schema_version: int = SCHEMA_VERSION

    def as_dict(self) -> dict[str, Any]:
        data = asdict(self)
        for edge, raw in zip(self.edges, data["edges"]):
            raw["id"] = edge.id
        return data
