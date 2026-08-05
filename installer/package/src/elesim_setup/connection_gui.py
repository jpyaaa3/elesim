"""Loopback-only browser connection manager for an Elesim DDS graph.

The browser edits only the non-secret :class:`ConnectionTopology` document.
Provisioning and rollout work is injected so this HTTP boundary never needs to
know an authority private key, an SSH password, or deployment internals.
"""

from __future__ import annotations

import hmac
import ipaddress
import json
import os
import re
import secrets
import threading
import time
import urllib.parse
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Mapping

from .connection_manager import (
    CONNECTION_SCHEMA_VERSION,
    ConnectionTopology,
    TOPOLOGY_MODES,
    TwoHostPreflight,
)


ConnectionRunner = Callable[
    [ConnectionTopology, str, Callable[[str], None]],
    None,
]
StatusProvider = Callable[[ConnectionTopology], Mapping[str, object]]
FingerprintProbe = Callable[[str, int], str]

_MAX_BODY_BYTES = 1_048_576
_MAX_JOB_LOGS = 2_048
_MAX_LOG_LINE = 4_096
_JOB_ACTIONS = frozenset(
    {"provision", "deploy", "rotate", "start", "stop", "restart", "check"}
)
_SSH_FINGERPRINT = re.compile(r"^SHA256:[A-Za-z0-9+/]{43}$")
_PEM_LOG = re.compile(r"-----BEGIN|-----END", re.IGNORECASE)
_SECRET_ASSIGNMENT = re.compile(
    r"\b(password|passphrase|private[_ -]?key|secret|credential|token)"
    r"\s*([:=])\s*(?:\"[^\"]*\"|'[^']*'|\S+)",
    re.IGNORECASE,
)
_BEARER_VALUE = re.compile(r"\bbearer\s+\S+", re.IGNORECASE)


class ConnectionJobCancelled(RuntimeError):
    """Raised at a cooperative log boundary after cancellation was requested."""


def connection_web_root() -> Path:
    return Path(__file__).resolve().parent / "connection_web"


@dataclass
class ConnectionJob:
    status: str = "idle"
    action: str = ""
    logs: list[str] = field(default_factory=list)
    error: str = ""
    started_at: float | None = None
    finished_at: float | None = None

    def snapshot(self) -> dict[str, object]:
        return {
            "status": self.status,
            "action": self.action,
            "logs": list(self.logs),
            "error": self.error,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
        }


class ConnectionManagerApplication:
    """Validate, persist, and deploy one operator-owned connection topology."""

    def __init__(
        self,
        *,
        state_path: Path,
        token: str,
        runner: ConnectionRunner,
        status_provider: StatusProvider | None = None,
        fingerprint_probe: FingerprintProbe | None = None,
        local_install_root: Path | None = None,
        local_bin_dir: Path | None = None,
    ) -> None:
        self.state_path = state_path.expanduser()
        self.token = str(token)
        if not self.token:
            raise ValueError("a non-empty connection-manager token is required")
        self.runner = runner
        self.status_provider = status_provider
        self.fingerprint_probe = fingerprint_probe or _default_fingerprint_probe
        self.local_install_root = (
            "" if local_install_root is None else str(local_install_root.expanduser().resolve())
        )
        self.local_bin_dir = (
            "" if local_bin_dir is None else str(local_bin_dir.expanduser().resolve())
        )
        self.job = ConnectionJob()
        self._job_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._cancel_event = threading.Event()
        self._job_thread: threading.Thread | None = None

    def context(self) -> dict[str, object]:
        topology = self.load_topology(required=False)
        from .network import detect_tailscale

        tailscale = detect_tailscale().to_dict()
        return {
            "schema_version": CONNECTION_SCHEMA_VERSION,
            "topology_modes": sorted(TOPOLOGY_MODES),
            "topology_exists": topology is not None,
            "topology": None if topology is None else topology.to_dict(),
            "derived_static_peers": self._derived_peers(topology),
            "local_defaults": {
                "install_root": self.local_install_root,
                "bin_dir": self.local_bin_dir,
            },
            "tailscale": tailscale,
        }

    def load_topology(self, *, required: bool = True) -> ConnectionTopology | None:
        with self._state_lock:
            if not self.state_path.exists() and not self.state_path.is_symlink():
                if required:
                    raise FileNotFoundError(
                        "save a valid connection topology before starting deployment"
                    )
                return None
            return ConnectionTopology.load(self.state_path)

    def validate_topology(self, payload: Mapping[str, Any]) -> dict[str, object]:
        topology = ConnectionTopology.from_dict(payload)
        return self._topology_response(topology, saved=False)

    def validate_preflight(self, payload: Mapping[str, Any]) -> dict[str, object]:
        """Validate two mutable host endpoints without touching saved topology.

        ``probe_ssh`` is an explicit, read-only host-key reachability check. It
        never stores the returned fingerprint and does not turn this temporary
        endpoint document into a deployable topology.
        """

        raw = dict(payload)
        probe_ssh = raw.pop("probe_ssh", False)
        if not isinstance(probe_ssh, bool):
            raise ValueError("preflight probe_ssh must be boolean")
        preflight = TwoHostPreflight.from_dict(raw)
        ssh_checks: dict[str, dict[str, object]] = {}
        for host in preflight.hosts:
            if host.ssh is None:
                continue
            check: dict[str, object] = {
                "host": host.ssh.host,
                "port": host.ssh.port,
                "user": host.ssh.user,
                "checked": False,
            }
            if probe_ssh:
                result = self.probe_fingerprint(
                    {"host": host.ssh.host, "port": host.ssh.port}
                )
                check["checked"] = True
                check["fingerprint"] = result["fingerprint"]
            ssh_checks[host.host_id] = check
        return {
            "valid": True,
            "preflight": preflight.to_dict(),
            "derived_static_peers": {
                host.host_id: list(preflight.discovery_peers(host.host_id))
                for host in preflight.hosts
            },
            "ssh_checks": ssh_checks,
        }

    def save_topology(self, payload: Mapping[str, Any]) -> dict[str, object]:
        topology = ConnectionTopology.from_dict(payload)
        with self._job_lock:
            if self.job.status in {"running", "cancelling"}:
                raise RuntimeError(
                    "배포 중에는 연결 토폴로지를 변경할 수 없습니다"
                )
            with self._state_lock:
                destination = topology.save(self.state_path)
        response = self._topology_response(topology, saved=True)
        response["mode"] = f"{destination.stat().st_mode & 0o777:04o}"
        return response

    def probe_fingerprint(self, payload: Mapping[str, Any]) -> dict[str, object]:
        keys = {str(key) for key in payload}
        if keys != {"host", "port"}:
            raise ValueError("SSH probe requires exactly host and port")
        host = payload["host"]
        port = payload["port"]
        if not isinstance(host, str) or not host.strip():
            raise ValueError("SSH host must be a non-empty hostname or IP")
        if (
            isinstance(port, bool)
            or not isinstance(port, int)
            or not 1 <= port <= 65535
        ):
            raise ValueError("SSH port must be in 1..65535")
        fingerprint = self.fingerprint_probe(host.strip(), port)
        if not isinstance(fingerprint, str) or not _SSH_FINGERPRINT.fullmatch(
            fingerprint
        ):
            raise RuntimeError("SSH probe returned an invalid host-key fingerprint")
        return {"fingerprint": fingerprint}

    def start_job(self, action: str) -> dict[str, object]:
        if action not in _JOB_ACTIONS:
            raise ValueError(f"unsupported connection-manager action: {action!r}")
        with self._job_lock:
            if self.job.status in {"running", "cancelling"}:
                raise RuntimeError("a connection-manager job is already running")
            topology = self.load_topology(required=True)
            assert topology is not None
            if (
                action in {"provision", "rotate"}
                and topology.security_profile != "sros2"
            ):
                raise ValueError(f"{action} requires the sros2 security profile")
            self._cancel_event.clear()
            self.job = ConnectionJob(
                status="running",
                action=action,
                started_at=time.time(),
            )
        thread = threading.Thread(
            target=self._run_job,
            args=(topology, action),
            name=f"elesim-connection-{action}",
            daemon=False,
        )
        self._job_thread = thread
        thread.start()
        return self.job_snapshot()

    def cancel_job(self) -> dict[str, object]:
        with self._job_lock:
            if self.job.status not in {"running", "cancelling"}:
                raise RuntimeError("no connection-manager job is running")
            self._cancel_event.set()
            self.job.status = "cancelling"
            return self.job.snapshot()

    def job_snapshot(self) -> dict[str, object]:
        with self._job_lock:
            return self.job.snapshot()

    def runtime_status(self) -> dict[str, object]:
        """Read-only lifecycle status, kept separate from DDS discovery state."""

        topology = self.load_topology(required=False)
        if topology is None:
            return {"available": False, "reason": "topology is not saved", "hosts": []}
        if self.status_provider is None:
            return {
                "available": False,
                "reason": "runtime status provider is not configured",
                "hosts": [],
            }
        result = dict(self.status_provider(topology))
        result.setdefault("available", True)
        return result

    def request_shutdown(self) -> None:
        with self._job_lock:
            if self.job.status in {"running", "cancelling"}:
                raise RuntimeError(
                    "배포가 실행 중입니다. 먼저 취소하고 rollback 완료를 기다리십시오"
                )

    def cancel_and_wait(self) -> None:
        with self._job_lock:
            thread = self._job_thread
            if self.job.status in {"running", "cancelling"}:
                self._cancel_event.set()
                self.job.status = "cancelling"
        if thread is not None and thread.is_alive():
            thread.join()

    def _run_job(self, topology: ConnectionTopology, action: str) -> None:
        def log(message: str) -> None:
            if self._cancel_event.is_set():
                raise ConnectionJobCancelled("connection-manager job cancelled")
            safe = self._safe_status_text(message)
            with self._job_lock:
                self.job.logs.append(safe)
                if len(self.job.logs) > _MAX_JOB_LOGS:
                    del self.job.logs[: len(self.job.logs) - _MAX_JOB_LOGS]
            if self._cancel_event.is_set():
                raise ConnectionJobCancelled("connection-manager job cancelled")

        try:
            self.runner(topology, action, log)
        except ConnectionJobCancelled:
            with self._job_lock:
                self.job.status = "cancelled"
                self.job.finished_at = time.time()
            return
        except Exception as exc:  # The browser reports a bounded, redacted failure.
            with self._job_lock:
                self.job.status = "failed"
                self.job.error = self._safe_status_text(exc)
                self.job.finished_at = time.time()
            return
        with self._job_lock:
            self.job.status = "completed"
            self.job.finished_at = time.time()

    def _safe_status_text(self, value: object) -> str:
        text = str(value).replace("\x00", "").replace("\r", " ")
        text = text[:_MAX_LOG_LINE]
        if self.token:
            text = text.replace(self.token, "[redacted]")
        if _PEM_LOG.search(text):
            return "[redacted sensitive job output]"
        text = _SECRET_ASSIGNMENT.sub(r"\1\2[redacted]", text)
        text = _BEARER_VALUE.sub("Bearer [redacted]", text)
        return text

    @staticmethod
    def _derived_peers(
        topology: ConnectionTopology | None,
    ) -> dict[str, list[str]]:
        if topology is None:
            return {}
        return {
            host.host_id: list(topology.discovery_peers(host.host_id))
            for host in topology.hosts
        }

    def _topology_response(
        self,
        topology: ConnectionTopology,
        *,
        saved: bool,
    ) -> dict[str, object]:
        return {
            "valid": True,
            "saved": saved,
            "topology": topology.to_dict(),
            "derived_static_peers": self._derived_peers(topology),
        }


class ConnectionManagerServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple[str, int],
        application: ConnectionManagerApplication,
        *,
        allow_container_wildcard: bool = False,
    ) -> None:
        _require_loopback(
            address[0], allow_container_wildcard=allow_container_wildcard
        )
        self.application = application
        super().__init__(address, ConnectionManagerRequestHandler)


class ConnectionManagerRequestHandler(BaseHTTPRequestHandler):
    server: ConnectionManagerServer

    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path.startswith("/api/"):
            if not self._authorized():
                return
            routes: dict[str, Callable[[], Mapping[str, object]]] = {
                "/api/context": self.server.application.context,
                "/api/topology": self.server.application.context,
                "/api/job": self.server.application.job_snapshot,
                "/api/runtime": self.server.application.runtime_status,
                "/api/status": self.server.application.runtime_status,
            }
            function = routes.get(parsed.path)
            if function is None:
                self._json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)
                return
            self._call(function)
            return
        self._static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            return
        path = urllib.parse.urlsplit(self.path).path
        if path == "/api/validate":
            self._call(
                lambda: self.server.application.validate_topology(self._body())
            )
            return
        if path == "/api/preflight":
            self._call(
                lambda: self.server.application.validate_preflight(self._body())
            )
            return
        if path == "/api/save":
            self._call(lambda: self.server.application.save_topology(self._body()))
            return
        if path == "/api/ssh/fingerprint":
            self._call(
                lambda: self.server.application.probe_fingerprint(self._body())
            )
            return
        if path.startswith("/api/job/"):
            action = path.removeprefix("/api/job/")

            def start() -> Mapping[str, object]:
                self._require_empty_body()
                return self.server.application.start_job(action)

            self._call(start, status=HTTPStatus.ACCEPTED)
            return
        if path == "/api/cancel":
            self._call(self._cancel_with_empty_body)
            return
        if path == "/api/shutdown":
            def shutdown() -> Mapping[str, object]:
                self._require_empty_body()
                self.server.application.request_shutdown()
                threading.Thread(target=self.server.shutdown, daemon=True).start()
                return {"status": "closing"}

            self._call(shutdown)
            return
        self._json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: object) -> None:
        return

    def _authorized(self) -> bool:
        supplied = self.headers.get("X-Elesim-Token", "")
        if hmac.compare_digest(supplied, self.server.application.token):
            return True
        self._json({"error": "unauthorized"}, status=HTTPStatus.UNAUTHORIZED)
        return False

    def _body(self) -> Mapping[str, Any]:
        if self.headers.get("Transfer-Encoding"):
            raise ValueError("streaming request bodies are not accepted")
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if not 0 <= length <= _MAX_BODY_BYTES:
            raise ValueError("request body is too large")
        if length and self.headers.get_content_type() != "application/json":
            raise ValueError("request body must use application/json")
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8")) if raw else {}
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError("request body is not valid UTF-8 JSON") from exc
        if not isinstance(payload, Mapping):
            raise ValueError("JSON body must be an object")
        return payload

    def _require_empty_body(self) -> None:
        if self._body():
            raise ValueError("this action does not accept request fields")

    def _cancel_with_empty_body(self) -> Mapping[str, object]:
        self._require_empty_body()
        return self.server.application.cancel_job()

    def _call(
        self,
        function: Callable[[], Mapping[str, object]],
        *,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        try:
            payload = function()
        except (
            FileNotFoundError,
            OSError,
            PermissionError,
            RuntimeError,
            ValueError,
        ) as exc:
            message = self.server.application._safe_status_text(exc)
            self._json({"error": message}, status=HTTPStatus.BAD_REQUEST)
            return
        self._json(payload, status=status)

    def _json(
        self,
        payload: Mapping[str, object],
        *,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._security_headers(api=True)
        self.end_headers()
        self.wfile.write(body)

    def _static(self, request_path: str) -> None:
        relative = request_path.lstrip("/") or "index.html"
        allowed = {
            "index.html": "text/html; charset=utf-8",
            "app.js": "text/javascript; charset=utf-8",
            "style.css": "text/css; charset=utf-8",
            "i18n.json": "application/json; charset=utf-8",
        }
        content_type = allowed.get(relative)
        if content_type is None:
            self._json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)
            return
        path = connection_web_root() / relative
        if not path.is_file():
            self._json({"error": "not found"}, status=HTTPStatus.NOT_FOUND)
            return
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self._security_headers(api=False)
        self.end_headers()
        self.wfile.write(body)

    def _security_headers(self, *, api: bool) -> None:
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Permissions-Policy",
            "camera=(), microphone=(), geolocation=()",
        )
        policy = "default-src 'none'"
        if not api:
            policy = (
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "connect-src 'self'; img-src 'self' data:; base-uri 'none'; "
                "form-action 'none'; frame-ancestors 'none'"
            )
        self.send_header("Content-Security-Policy", policy)


def _require_loopback(host: str, *, allow_container_wildcard: bool = False) -> None:
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError(
            "connection manager must bind to a literal loopback address"
        ) from exc
    if not address.is_loopback and not (
        allow_container_wildcard and address.is_unspecified
    ):
        raise ValueError("connection manager must bind to loopback only")


def _default_fingerprint_probe(host: str, port: int) -> str:
    # Keep the non-network topology editor importable with the standard library;
    # Paramiko/protocol credential helpers are needed only when a probe is run.
    from .credentials import probe_ssh_fingerprint

    return probe_ssh_fingerprint(host, port)


def run_connection_gui(
    *,
    state_path: Path,
    runner: ConnectionRunner,
    status_provider: StatusProvider | None = None,
    host: str = "127.0.0.1",
    port: int = 8766,
    token: str = "",
    fingerprint_probe: FingerprintProbe | None = None,
    local_install_root: Path | None = None,
    local_bin_dir: Path | None = None,
) -> int:
    session_token = token or secrets.token_urlsafe(32)
    application = ConnectionManagerApplication(
        state_path=state_path,
        token=session_token,
        runner=runner,
        status_provider=status_provider,
        fingerprint_probe=fingerprint_probe,
        local_install_root=local_install_root,
        local_bin_dir=local_bin_dir,
    )
    server = ConnectionManagerServer(
        (host, int(port)),
        application,
        allow_container_wildcard=os.environ.get("ELESIM_CONNECTION_PUBLISHED") == "1",
    )
    actual_host, actual_port = server.server_address[:2]
    display_host = "127.0.0.1" if actual_host in {"0.0.0.0", "::"} else actual_host
    url = f"http://{display_host}:{actual_port}/?token={session_token}"
    print(
        f"[connection-manager] {url}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        application.cancel_and_wait()
        server.server_close()
    return 0


__all__ = [
    "ConnectionJob",
    "ConnectionJobCancelled",
    "ConnectionManagerApplication",
    "ConnectionManagerServer",
    "ConnectionRunner",
    "StatusProvider",
    "run_connection_gui",
    "connection_web_root",
]
