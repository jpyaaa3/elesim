"""Stdlib-only build transcript and bounded terminal progress display."""
from __future__ import annotations

import argparse
import codecs
from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import secrets
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unicodedata


def _open_log(root: Path):
    """Create a private file without following any directory symlink."""
    root = Path(os.path.abspath(root.expanduser()))
    descriptor = os.open(root.anchor, os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in root.parts[1:]:
            try:
                os.mkdir(part, 0o700, dir_fd=descriptor)
            except FileExistsError:
                pass
            child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
                            dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        info = os.fstat(descriptor)
        if info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise PermissionError(f"build log directory must be private and owned by this user: {root}")
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + secrets.token_hex(6) + ".log"
        fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                     0o600, dir_fd=descriptor)
        return root / name, os.fdopen(fd, "wb")
    finally:
        os.close(descriptor)


_ESCAPES = re.compile(r"\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)|\x1b\[[0-?]*[ -/]*[@-~]")
_IMAGE_OUTPUT = re.compile(
    r"(?:naming to|writing image|exporting to)\s+(?:docker\.io/)?"
    r"(elesim/[a-z][a-z0-9_.-]*:[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6})-"
    r"[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))(?:\s|$)",
    re.IGNORECASE,
)
_IMAGE_SUMMARY_OUTPUT = re.compile(
    r"Image\s+(?:docker\.io/)?"
    r"(elesim/[a-z][a-z0-9_.-]*:[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6})-"
    r"[a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))\s+Built\b",
    re.IGNORECASE,
)
_IMAGE_REFERENCE = re.compile(
    r"^(elesim/[a-z][a-z0-9_.-]*):"
    r"([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))-"
    r"([a-z]{2,16}(?:_[a-z]{2,16}|[0-9]{0,6}))$",
    re.IGNORECASE,
)
_DOCKER_CONFIG_MAX_BYTES = 1024 * 1024


def _is_windows_credential_helper(value: object) -> bool:
    """Return whether a Docker credential helper is a Windows executable.

    WSL commonly inherits Docker Desktop's ``desktop.exe`` helper from the
    Windows-side Docker config. The Linux Docker CLI resolves that name to a
    ``docker-credential-*.exe`` executable and then fails with ``Exec format
    error`` before BuildKit can pull a public base image.
    """

    return isinstance(value, str) and value.strip().lower().endswith(".exe")


def _prepare_docker_environment(
    environment: dict[str, str],
) -> tuple[dict[str, str], Path | None]:
    """Use a temporary config when the inherited Docker config is WSL-hostile.

    Keep the caller's config untouched and retain ordinary ``auths`` entries
    plus non-Windows per-registry helpers. A build only needs a disposable
    config because the broken helper is used for registry pulls; local image
    inspection/tagging remains on the same Docker daemon and is unaffected.
    """

    configured_root = environment.get("DOCKER_CONFIG", "").strip()
    config_root = (
        Path(configured_root).expanduser()
        if configured_root
        else Path.home() / ".docker"
    )
    config_path = config_root / "config.json"
    try:
        if not config_path.is_file() or config_path.stat().st_size > _DOCKER_CONFIG_MAX_BYTES:
            return environment, None
        with config_path.open("r", encoding="utf-8") as stream:
            config = json.load(stream)
    except (OSError, UnicodeError, ValueError):
        # Let Docker report malformed/unreadable configurations in the usual
        # way. This helper must not turn an unrelated config error into a
        # different failure before the build process starts.
        return environment, None
    if not isinstance(config, dict):
        return environment, None

    sanitized = dict(config)
    changed = False
    if _is_windows_credential_helper(sanitized.get("credsStore")):
        sanitized.pop("credsStore", None)
        changed = True
    helpers = sanitized.get("credHelpers")
    if isinstance(helpers, dict):
        filtered_helpers = {
            registry: helper
            for registry, helper in helpers.items()
            if not _is_windows_credential_helper(helper)
        }
        if len(filtered_helpers) != len(helpers):
            changed = True
            if filtered_helpers:
                sanitized["credHelpers"] = filtered_helpers
            else:
                sanitized.pop("credHelpers", None)
    if not changed:
        return environment, None

    temporary_root = Path(tempfile.mkdtemp(prefix="elesim-docker-config-"))
    temporary_config = temporary_root / "config.json"
    try:
        temporary_config.write_text(
            json.dumps(sanitized, ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
        temporary_root.chmod(0o700)
        temporary_config.chmod(0o600)
    except BaseException:
        shutil.rmtree(temporary_root, ignore_errors=True)
        raise
    prepared = dict(environment)
    prepared["DOCKER_CONFIG"] = str(temporary_root)
    return prepared, temporary_root


def _image_output_match(value: str):
    """Recognize BuildKit exports and Compose's compact ``Image ... Built`` line."""
    return _IMAGE_OUTPUT.search(value) or _IMAGE_SUMMARY_OUTPUT.search(value)


def _display_text(value: str) -> str:
    return "".join(c for c in _ESCAPES.sub("", value) if c.isprintable())


def _muted(value: str, enabled: bool) -> str:
    """Render secondary transcript metadata in terminal gray only."""
    return f"\x1b[90m{value}\x1b[0m" if enabled else value


def _fit_row(value: str, width: int) -> str:
    end = 0
    for end, char in enumerate(value, 1):
        width -= 0 if unicodedata.combining(char) else (2 if unicodedata.east_asian_width(char) in "WF" else 1)
        if width < 0:
            return value[:end - 1]
    return value[:end]


def _write_image_report(path: Path, images: dict[str, tuple[str, str]]) -> None:
    """Persist the compact image summary for the caller's final report."""
    path = Path(path)
    if not path.is_absolute() or path.parent.is_symlink():
        raise ValueError("image report path must be an absolute non-symlink path")
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".build-images-", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            names = {value[0] for value in images.values()}
            if len(names) == 1:
                stream.write(f"Installation name={next(iter(names))}\n")
            order = {f"elesim/{role}": index for index, role in enumerate(
                ("tools", "sim", "pilot", "ui", "robot")
            )}
            for repository in sorted(images, key=lambda name: (order.get(name, len(order)), name)):
                stream.write(f"{repository}={images[repository][1]}\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _parse_expected_images(values: list[str]) -> dict[str, tuple[str, str]]:
    """Parse the complete image set selected by the generated Compose file.

    BuildKit output only mentions services that were rebuilt.  The updater
    supplies the target references separately so a report also includes
    images that were safely retagged and reused.
    """
    images: dict[str, tuple[str, str]] = {}
    for value in values:
        match = _IMAGE_REFERENCE.fullmatch(value.strip())
        if match is None:
            raise ValueError(f"invalid expected image reference: {value!r}")
        repository = match.group(1).lower()
        identity = (match.group(2).lower(), match.group(3).lower())
        previous = images.get(repository)
        if previous is not None and previous != identity:
            raise ValueError(f"duplicate expected image repository: {repository}")
        images[repository] = identity
    return images


def run(command: list[str], log_dir: Path, mode: str = "auto", *,
        title: str = "Runtime image build", notice_prefixes: tuple[str, ...] = (),
        hidden_prefixes: tuple[str, ...] = (), result_file: Path | None = None) -> int:
    log_path, transcript = _open_log(log_dir)
    tty = sys.stdout.isatty() and sys.stderr.isatty() and os.environ.get("TERM") != "dumb"
    verbose = mode == "verbose" or (mode == "auto" and not tty)
    animate = tty and not verbose and mode != "plain"
    tail: deque[str] = deque(maxlen=12)
    built_images: dict[str, tuple[str, str]] = {}
    line_count = shown_count = 0
    pending = ""
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    started = last_draw = time.monotonic()
    cancelled = 0
    cancel_time = 0.0
    process = None
    temporary_docker_config: Path | None = None
    handlers = {}

    def stop(signum, _frame):
        nonlocal cancelled, cancel_time
        if not cancelled:
            cancelled, cancel_time = signum, time.monotonic()
        if process is not None:
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass

    def show(chunk: bytes):
        nonlocal pending, line_count, shown_count
        transcript.write(chunk)
        transcript.flush()
        if verbose:
            sys.stdout.buffer.write(chunk)
            sys.stdout.buffer.flush()
        text = pending + decoder.decode(chunk)
        lines = text.replace("\r", "\n").split("\n")
        pending = lines.pop()[-1024:]
        for line in lines:
            line_count += 1
            safe = _display_text(line[-1024:])
            image_match = _image_output_match(safe)
            if image_match:
                image = image_match.group(1).lower()
                repository, tag = image.split(":", 1)
                install_name, alias = tag.split("-", 1)
                built_images[repository] = (install_name, alias)
                if not verbose:
                    continue
            if not verbose and safe.lstrip().startswith(hidden_prefixes):
                continue
            if not verbose and safe.lstrip().startswith(notice_prefixes):
                if animate:
                    print("\r\x1b[2K", end="", file=sys.stderr)
                print(f"  │ {safe}", file=sys.stderr, flush=True)
                shown_count += 1
            elif safe.strip():
                tail.append(safe)

    try:
        title = _display_text(title)
        heading = title
        print(f"{heading}\n{_muted(f'  └ Full log: {log_path}', tty)}",
              file=sys.stderr, flush=True)
        environment = {**os.environ, "ELESIM_PROGRESS_ACTIVE": "1", "PYTHONUNBUFFERED": "1"}
        environment, temporary_docker_config = _prepare_docker_environment(environment)
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=environment)
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, stop)
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while selector.get_map() or process.poll() is None:
                for key, _ in selector.select(0.2):
                    chunk = os.read(key.fd, 65536)
                    if chunk:
                        show(chunk)
                    else:
                        selector.unregister(key.fileobj)
                now = time.monotonic()
                if cancelled and now - cancel_time >= 3:
                    stop(signal.SIGKILL, None)
                if animate and now - last_draw >= 0.2:
                    elapsed = int(now - started)
                    latest = _display_text(pending) or (tail[-1] if tail else "Waiting for build output")
                    line = f"  │ {elapsed // 60:02d}:{elapsed % 60:02d}  {latest}"
                    width = max(1, shutil.get_terminal_size().columns - 1)
                    # One live row, never append the entire build stream.
                    print("\r\x1b[2K" + _fit_row(line, width), end="", file=sys.stderr, flush=True)
                    last_draw = now
            status = process.wait()
        pending += decoder.decode(b"", final=True)
        if pending:
            safe = _display_text(pending)
            image_match = _image_output_match(safe)
            if image_match:
                image = image_match.group(1).lower()
                repository, tag = image.split(":", 1)
                install_name, alias = tag.split("-", 1)
                built_images[repository] = (install_name, alias)
                if not verbose:
                    safe = ""
            if not verbose and safe.lstrip().startswith(hidden_prefixes):
                pass
            elif not verbose and safe.lstrip().startswith(notice_prefixes):
                if animate:
                    print("\r\x1b[2K", end="", file=sys.stderr)
                print(f"  │ {safe}", file=sys.stderr)
                shown_count += 1
            else:
                tail.append(safe)
            line_count += 1
        if animate:
            print("\r\x1b[2K", end="", file=sys.stderr)
        status = 128 + cancelled if cancelled else (128 - status if status < 0 else status)
        if not verbose:
            preview = list(tail) if status else ([] if notice_prefixes else list(tail)[-3:])
            for line in preview:
                preview_line = f"  │ {_fit_row(line, max(40, shutil.get_terminal_size().columns - 4))}"
                print(_muted(preview_line, tty), file=sys.stderr)
            print(_muted("  │ ...", tty), file=sys.stderr)
        if status == 0 and built_images and result_file is not None:
            _write_image_report(result_file, built_images)
        label = "Completed" if status == 0 else f"Failed (exit {status})"
        print(_muted(
            f"  └ {label} in {time.monotonic() - started:.1f}s",
            tty), file=sys.stderr, flush=True)
        return status
    finally:
        if process is not None:
            if process.poll() is None:
                stop(signal.SIGTERM, None)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    stop(signal.SIGKILL, None)
                    process.wait()
            process.stdout.close()
        for sig, handler in handlers.items():
            signal.signal(sig, handler)
        if temporary_docker_config is not None:
            shutil.rmtree(temporary_docker_config, ignore_errors=True)
        transcript.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--title", default="Runtime image build")
    parser.add_argument("--notice-prefix", action="append", default=[])
    parser.add_argument("--hide-prefix", action="append", default=[])
    parser.add_argument("--result-file", type=Path)
    parser.add_argument(
        "--write-report", action="store_true",
        help="write a complete image report without running a build command",
    )
    parser.add_argument(
        "--expected-image", action="append", default=[],
        help="image reference to include in a complete report",
    )
    parser.add_argument("--mode", choices=("auto", "compact", "plain", "verbose"),
                        default="verbose" if os.environ.get("ELESIM_VERBOSE") == "1"
                        else os.environ.get("ELESIM_BUILD_PROGRESS", "auto"))
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.write_report:
        if args.result_file is None:
            parser.error("--write-report requires --result-file")
        try:
            _write_image_report(args.result_file, _parse_expected_images(args.expected_image))
        except (OSError, ValueError) as exc:
            print(f"Build progress error: {exc}", file=sys.stderr)
            return 74
        return 0
    if args.log_dir is None:
        parser.error("--log-dir is required when running a build")
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a build command is required")
    try:
        mode = "verbose" if os.environ.get("ELESIM_VERBOSE") == "1" else args.mode
        return run(command, args.log_dir, mode, title=args.title,
                   notice_prefixes=tuple(args.notice_prefix),
                   hidden_prefixes=tuple(args.hide_prefix), result_file=args.result_file)
    except OSError as exc:
        print(f"Build progress error: {exc}", file=sys.stderr)
        return 74


if __name__ == "__main__":
    raise SystemExit(main())
