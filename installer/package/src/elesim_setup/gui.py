"""Loopback-only browser wizard for Elesim installation."""

from __future__ import annotations

import hmac
import json
import secrets
import shlex
import threading
import time
import urllib.parse
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .capabilities import HostCapabilities, detect_host_capabilities
from .credentials import (
    probe_ssh_fingerprint,
    validate_external_turn_credentials,
)
from .request import SetupRequest
from .ownership import OwnershipManifest, sha256_file


InstallRunner = Callable[[SetupRequest, Callable[[str], None]], None]
_MAX_BODY_BYTES = 1_048_576


class InstallCancelled(RuntimeError):
    pass


def web_root() -> Path:
    return Path(__file__).resolve().parent / "web"


@dataclass
class InstallJob:
    status: str = "idle"
    logs: list[str] = field(default_factory=list)
    error: str = ""
    started_at: float | None = None
    finished_at: float | None = None

    def snapshot(self) -> dict[str, object]:
        return {
            "status": self.status,
            "logs": list(self.logs),
            "error": self.error,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
        }


class WizardApplication:
    def __init__(
        self,
        *,
        source_root: Path,
        invocation_dir: Path,
        capabilities: HostCapabilities,
        repository: str,
        ref: str,
        token: str,
        runner: InstallRunner,
        allowed_roots: Sequence[Path] | None = None,
    ) -> None:
        self.source_root = source_root.expanduser().resolve()
        self.invocation_dir = invocation_dir.expanduser().resolve()
        self.capabilities = capabilities
        self.repository = repository
        self.ref = ref
        self.token = token
        home = Path.home().resolve()
        roots = allowed_roots or (home, self.invocation_dir)
        self.allowed_roots = tuple(dict.fromkeys(path.expanduser().resolve() for path in roots))
        self.runner = runner
        self.job = InstallJob()
        self._job_lock = threading.Lock()
        self._cancel_event = threading.Event()

    def context(self) -> dict[str, object]:
        return {
            "repository": self.repository,
            "ref": self.ref,
            "defaults": {
                "prefix": str(self.invocation_dir),
                "bin_dir": str(self.invocation_dir / "bin"),
                "dds_system_id": "elesim",
                "dds_domain_id": 0,
                "dds_rmw_implementation": "rmw_cyclonedds_cpp",
                "dds_discovery_mode": "multicast",
                "dds_static_peers": "",
                "dds_interface": "",
                "dds_security_profile": "sros2",
                "dds_security_provisioning": "managed",
                "dds_keystore": "",
                "dds_enclave": "",
            },
            "capabilities": self.capabilities.to_dict(),
            "allowed_roots": [str(path) for path in self.allowed_roots],
        }

    def list_directories(
        self,
        requested: Path,
        *,
        include_files: bool = False,
    ) -> dict[str, object]:
        directory = requested.expanduser().resolve()
        self._require_allowed(directory)
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        children: list[dict[str, str]] = []
        try:
            entries = sorted(directory.iterdir(), key=lambda path: path.name.casefold())
        except OSError as exc:
            raise PermissionError(str(exc)) from exc
        for child in entries:
            try:
                resolved = child.resolve()
                self._require_allowed(resolved)
                if child.is_dir():
                    children.append(
                        {"name": child.name, "path": str(resolved), "kind": "directory"}
                    )
                elif include_files and child.is_file():
                    children.append(
                        {"name": child.name, "path": str(resolved), "kind": "file"}
                    )
            except (OSError, PermissionError):
                continue
        parent = directory.parent.resolve()
        return {
            "path": str(directory),
            "parent": str(parent) if parent != directory and self._is_allowed(parent) else "",
            "directories": children,
        }

    def build_request(self, payload: Mapping[str, Any]) -> SetupRequest:
        trusted = dict(payload)
        trusted["source_root"] = str(self.source_root)
        trusted["repository"] = self.repository
        trusted["ref"] = self.ref
        request = SetupRequest.from_dict(trusted)
        self._require_allowed(request.prefix)
        self._require_allowed(request.bin_dir)
        keystore = request.dds.keystore_path
        if keystore is not None:
            self._require_allowed(keystore)
        turn_secret = request.turn.secret_path
        if turn_secret is not None:
            self._require_allowed(turn_secret)
        turn_credentials = request.turn.credential_path
        if turn_credentials is not None:
            self._require_allowed(turn_credentials)
            validate_external_turn_credentials(
                turn_credentials,
                urls=request.network.turn_urls,
            )
        identity = request.ssh.identity_file.strip()
        if identity:
            identity_path = Path(identity).expanduser().resolve()
            self._require_allowed(identity_path)
            if not identity_path.is_file():
                raise FileNotFoundError(identity_path)
        return request.validate(self.capabilities)

    def validate_request(self, payload: Mapping[str, Any]) -> dict[str, object]:
        request = self.build_request(payload)
        return {
            "edition": request.edition,
            "roles": list(request.roles),
            "prefix": str(request.prefix),
            "bin_dir": str(request.bin_dir),
            "gpu_mode": request.compute.gpu_mode,
            "security_profile": request.dds.security_profile,
            "security_provisioning": request.dds.security_provisioning,
            "turn_mode": request.turn.mode,
            "runtime_text_logs": request.runtime_text_logs.enabled,
            "register_path": request.register_path,
            "jaeger": request.jaeger,
        }

    def uninstall_guide(self, payload: Mapping[str, Any]) -> dict[str, object]:
        """Validate an installed ownership record and render host-only commands.

        The disposable bootstrap GUI intentionally has neither the Docker
        socket nor a host command channel.  It can therefore guide a clean
        uninstall without turning a loopback web token into deletion authority.
        The generated host command performs the complete pre-mutation checks
        again immediately before removal.
        """

        prefix_value = payload.get("prefix")
        if not isinstance(prefix_value, str) or not prefix_value.strip():
            raise ValueError("제거할 설치 prefix가 필요합니다")
        prefix = Path(prefix_value).expanduser().resolve()
        self._require_allowed(prefix)
        manifest_candidates = (
            prefix / "install-ownership.json",
            prefix / ".elesim/development/install-ownership.json",
        )
        for candidate in manifest_candidates:
            self._require_allowed(candidate)
        present = tuple(
            candidate
            for candidate in manifest_candidates
            if candidate.exists() or candidate.is_symlink()
        )
        if len(present) != 1:
            rendered = ", ".join(str(path) for path in manifest_candidates)
            raise ValueError(
                "선택한 prefix에서 ownership manifest를 정확히 하나 찾을 수 "
                f"없습니다: {rendered}"
            )
        manifest_path = present[0]
        manifest = OwnershipManifest.load(manifest_path)
        if manifest.prefix_path != prefix:
            raise ValueError(
                "선택한 prefix와 ownership manifest의 prefix가 다릅니다: "
                f"{manifest.prefix}"
            )
        uninstaller = manifest.bin_path / "elesim-uninstall"
        self._require_allowed(uninstaller)
        wrapper = next(
            (entry for entry in manifest.wrappers if entry.path == str(uninstaller)),
            None,
        )
        if (
            wrapper is None
            or uninstaller.is_symlink()
            or not uninstaller.is_file()
            or sha256_file(uninstaller) != wrapper.sha256
        ):
            raise ValueError(
                "ownership manifest가 현재 elesim-uninstall wrapper를 소유하지 "
                "않거나 파일이 변경되었습니다"
            )
        flags: list[str] = []
        keep_logs = payload.get("keep_logs")
        if keep_logs is True:
            flags.append("--keep-logs")
        elif keep_logs is not None and keep_logs is not False:
            raise ValueError("keep_logs는 boolean이어야 합니다")
        keep_authority = payload.get("keep_authority")
        if keep_authority is True:
            flags.append("--keep-authority")
        elif keep_authority is not None and keep_authority is not False:
            raise ValueError("keep_authority는 boolean이어야 합니다")
        base = (
            str(uninstaller),
            "--manifest",
            str(manifest.path),
            *flags,
        )
        return {
            "install_uuid": manifest.install_uuid,
            "prefix": manifest.prefix,
            "plan_command": shlex.join((*base, "--plan")),
            "execute_command": shlex.join(base),
            "preserves_logs": "--keep-logs" in flags,
            "preserves_authority": "--keep-authority" in flags,
        }

    def start_install(self, payload: Mapping[str, Any]) -> dict[str, object]:
        request = self.build_request(payload)
        with self._job_lock:
            if self.job.status in {"running", "cancelling"}:
                raise RuntimeError("an installation is already running")
            self._cancel_event.clear()
            self.job = InstallJob(status="running", started_at=time.time())
        thread = threading.Thread(
            target=self._run_install,
            args=(request,),
            name="elesim-setup-install",
            daemon=True,
        )
        thread.start()
        return self.job.snapshot()

    def job_snapshot(self) -> dict[str, object]:
        with self._job_lock:
            return self.job.snapshot()

    def cancel_install(self) -> dict[str, object]:
        with self._job_lock:
            if self.job.status not in {"running", "cancelling"}:
                raise RuntimeError("no installation is running")
            self._cancel_event.set()
            self.job.status = "cancelling"
            return self.job.snapshot()

    def _run_install(self, request: SetupRequest) -> None:
        def log(message: str) -> None:
            with self._job_lock:
                self.job.logs.append(str(message))
            if self._cancel_event.is_set():
                raise InstallCancelled("installation cancelled by user")

        try:
            self.runner(request, log)
            if self._cancel_event.is_set():
                raise InstallCancelled("installation cancelled by user")
        except InstallCancelled:
            with self._job_lock:
                self.job.status = "cancelled"
                self.job.finished_at = time.time()
            return
        except Exception as exc:  # The UI must report installer failures verbatim.
            with self._job_lock:
                self.job.status = "failed"
                self.job.error = str(exc)
                self.job.finished_at = time.time()
            return
        with self._job_lock:
            self.job.status = "completed"
            self.job.finished_at = time.time()

    def _require_allowed(self, path: Path) -> None:
        if not self._is_allowed(path):
            raise PermissionError(f"path is outside installer roots: {path}")

    def _is_allowed(self, path: Path) -> bool:
        resolved = path.resolve()
        for root in self.allowed_roots:
            try:
                resolved.relative_to(root)
                return True
            except ValueError:
                continue
        return False


class WizardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], application: WizardApplication):
        self.application = application
        super().__init__(address, WizardRequestHandler)


class WizardRequestHandler(BaseHTTPRequestHandler):
    server: WizardServer

    def do_GET(self) -> None:  # noqa: N802
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path.startswith("/api/"):
            if not self._authorized():
                return
            if parsed.path == "/api/context":
                self._json(self.server.application.context())
                return
            if parsed.path == "/api/job":
                self._json(self.server.application.job_snapshot())
                return
            if parsed.path == "/api/directories":
                query = urllib.parse.parse_qs(parsed.query)
                value = query.get("path", [str(self.server.application.invocation_dir)])[0]
                include_files = query.get("files", ["0"])[0] == "1"
                self._call(
                    lambda: self.server.application.list_directories(
                        Path(value),
                        include_files=include_files,
                    )
                )
                return
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802
        if not self._authorized():
            return
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path == "/api/validate":
            self._call(lambda: self.server.application.validate_request(self._body()))
            return
        if parsed.path == "/api/install":
            self._call(
                lambda: self.server.application.start_install(self._body()),
                status=HTTPStatus.ACCEPTED,
            )
            return
        if parsed.path == "/api/uninstall/guide":
            self._call(
                lambda: self.server.application.uninstall_guide(self._body())
            )
            return
        if parsed.path == "/api/cancel":
            self._call(self.server.application.cancel_install)
            return
        if parsed.path == "/api/ssh/fingerprint":
            def fingerprint() -> Mapping[str, object]:
                body = self._body()
                return {
                    "fingerprint": probe_ssh_fingerprint(
                        str(body.get("host", "")),
                        int(body.get("port", 22)),
                    )
                }

            self._call(fingerprint)
            return
        if parsed.path == "/api/shutdown":
            self._json({"status": "closing"})
            threading.Thread(target=self.server.shutdown, daemon=True).start()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def log_message(self, format: str, *args: object) -> None:
        return

    def _authorized(self) -> bool:
        supplied = self.headers.get("X-Elesim-Token", "")
        if hmac.compare_digest(supplied, self.server.application.token):
            return True
        self._json({"error": "unauthorized"}, status=HTTPStatus.UNAUTHORIZED)
        return False

    def _body(self) -> Mapping[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if not 0 <= length <= _MAX_BODY_BYTES:
            raise ValueError("request body is too large")
        raw = self.rfile.read(length)
        payload = json.loads(raw.decode("utf-8")) if raw else {}
        if not isinstance(payload, Mapping):
            raise ValueError("JSON body must be an object")
        return payload

    def _call(
        self,
        function: Callable[[], Mapping[str, object]],
        *,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        try:
            payload = function()
        except (FileNotFoundError, OSError, PermissionError, RuntimeError, ValueError) as exc:
            self._json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
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
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'none'")
        self.end_headers()
        self.wfile.write(body)

    def _static(self, request_path: str) -> None:
        relative = request_path.lstrip("/") or "index.html"
        if relative not in {
            "index.html",
            "app.js",
            "style.css",
            "i18n.json",
            "fonts/NotoSansCJKkr-Regular.otf",
        }:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        path = web_root() / relative
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_types = {
            ".html": "text/html; charset=utf-8",
            ".js": "text/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".otf": "font/otf",
        }
        body = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_types.get(path.suffix, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "font-src 'self'; connect-src 'self'; img-src 'self' data:; "
            "base-uri 'none'; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(body)


def run_gui(
    *,
    source_root: Path,
    invocation_dir: Path,
    repository: str,
    ref: str,
    runner: InstallRunner,
    host: str = "127.0.0.1",
    port: int = 8765,
    token: str = "",
    capabilities: HostCapabilities | None = None,
) -> int:
    session_token = token or secrets.token_urlsafe(32)
    application = WizardApplication(
        source_root=source_root,
        invocation_dir=invocation_dir,
        capabilities=capabilities or detect_host_capabilities(),
        repository=repository,
        ref=ref,
        token=session_token,
        runner=runner,
    )
    server = WizardServer((host, int(port)), application)
    actual_host, actual_port = server.server_address[:2]
    display_host = "127.0.0.1" if actual_host in {"0.0.0.0", "::"} else actual_host
    print(
        f"[setup-gui] http://{display_host}:{actual_port}/?token={session_token}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        return 130
    finally:
        server.server_close()
    return 0


__all__ = [
    "InstallCancelled",
    "InstallJob",
    "WizardApplication",
    "WizardServer",
    "run_gui",
    "web_root",
]
