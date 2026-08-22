"""Loopback-only HTTP surface for the code map."""

from __future__ import annotations

import hmac
import gzip
import json
import mimetypes
import secrets
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .analyzer import analyze_repository


MAX_SOURCE_BYTES = 128 * 1024
MAX_SOURCE_LINES = 500
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


def _jaeger_spans(base_url: str) -> list[dict[str, Any]]:
    parsed = urllib.parse.urlparse(base_url)
    if parsed.scheme not in {"http", "https"} or parsed.hostname not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError("Jaeger URL must be loopback HTTP(S)")
    url = base_url.rstrip("/") + "/api/v3/traces?limit=100&lookback=1h"
    try:
        with urllib.request.urlopen(url, timeout=2.0) as response:
            payload = json.load(response)
    except (OSError, urllib.error.URLError, ValueError) as exc:
        return [{"error": str(exc), "source": url}]
    spans: list[dict[str, Any]] = []

    def walk(value: Any) -> None:
        if isinstance(value, dict):
            name = value.get("name") or value.get("operationName")
            span_id = value.get("spanId") or value.get("spanID")
            if name and span_id:
                attrs = value.get("attributes") or value.get("tags") or {}
                spans.append({"name": str(name), "span_id": str(span_id), "attributes": attrs})
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(payload)
    return spans[:2000]


class CodeMapServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], root: Path, token: str, jaeger_url: str) -> None:
        if address[0] not in {"127.0.0.1", "::1", "localhost"}:
            raise ValueError("code map must bind to loopback")
        self.root = root.resolve()
        self.token = token
        self.jaeger_url = jaeger_url
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
            elif split.path == "/api/source":
                query = self._query()
                path = query.get("path", [""])[0]
                line = int(query.get("line", ["1"])[0])
                self._json(_source(self.server.root, path, line))
            elif split.path == "/api/diff":
                path = self._query().get("path", [""])[0]
                self._json({"path": path, "text": _diff(self.server.root, path)})
            elif split.path == "/api/traces":
                self._json({"spans": _jaeger_spans(self.server.jaeger_url)})
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
    jaeger_url: str = "http://127.0.0.1:16686",
) -> None:
    actual_token = token or secrets.token_urlsafe(24)
    with CodeMapServer((host, port), root, actual_token, jaeger_url) as server:
        actual_port = server.server_address[1]
        print(f"[code-map] http://{host}:{actual_port}/?token={actual_token}", flush=True)
        server.serve_forever()
