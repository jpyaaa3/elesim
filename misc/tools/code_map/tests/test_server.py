from __future__ import annotations

import json
import gzip
import io
import subprocess
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from misc.tools.code_map.server import (
    MAX_SOURCE_BYTES,
    CodeMapServer,
    _inside,
    _jaeger_spans,
    _source,
)


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
    server = CodeMapServer(("127.0.0.1", 0), root, "secret-token", "http://127.0.0.1:9")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        with pytest.raises(urllib.error.HTTPError) as error:
            _request(server, "/api/snapshot")
        assert error.value.code == 403
        with _request(server, "/api/source?path=module.py&line=1", "secret-token") as response:
            payload = json.load(response)
        assert payload["path"] == "module.py"
        with pytest.raises(urllib.error.HTTPError) as error:
            _request(server, "/api/snapshot", "secret-token", "POST")
        assert error.value.code == 405
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_server_rejects_non_loopback_bind(tmp_path: Path):
    with pytest.raises(ValueError, match="loopback"):
        CodeMapServer(("0.0.0.0", 0), tmp_path, "token", "http://127.0.0.1:16686")


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
    server = CodeMapServer(("127.0.0.1", 0), root, "secret-token", "http://127.0.0.1:9")
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


def test_jaeger_overlay_is_loopback_and_normalizes_spans(monkeypatch):
    payload = {"traces": [{"spans": [{"spanId": "abc", "name": "publish_rgbd", "attributes": {"role": "sim"}}]}]}

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *unused):
            self.close()

    monkeypatch.setattr(urllib.request, "urlopen", lambda *args, **kwargs: Response(json.dumps(payload).encode()))
    assert _jaeger_spans("http://127.0.0.1:16686") == [
        {"name": "publish_rgbd", "span_id": "abc", "attributes": {"role": "sim"}}
    ]
    with pytest.raises(ValueError, match="loopback"):
        _jaeger_spans("https://jaeger.example.invalid")


def test_web_assets_are_local_and_expose_required_controls():
    web = Path(__file__).parents[1] / "web"
    html = (web / "index.html").read_text(encoding="utf-8")
    script = (web / "app.js").read_text(encoding="utf-8")
    assert "https://" not in html
    assert {"workflow", "depth", "traces", "source", "diff"} <= {
        marker.split('"', 1)[0] for marker in html.split('id="')[1:]
    }
    assert "/api/snapshot" in script
    assert "/api/events" in script
    assert "/api/traces" in script
    assert 'id="groups"' in html
    assert "function roleLayout" in script
    assert "function fitGraph" in script
    assert "function zoomAt" in script
    assert "screenX - worldX * nextScale" in script
