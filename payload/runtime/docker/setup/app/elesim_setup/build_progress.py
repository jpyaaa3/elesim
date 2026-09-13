"""Stdlib-only build transcript and bounded terminal progress display."""
from __future__ import annotations

import argparse
import codecs
from collections import deque
from datetime import datetime, timezone
import os
from pathlib import Path
import re
import secrets
import selectors
import shutil
import signal
import subprocess
import sys
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


def run(command: list[str], log_dir: Path, mode: str = "auto", *,
        title: str = "Runtime image build", notice_prefixes: tuple[str, ...] = ()) -> int:
    log_path, transcript = _open_log(log_dir)
    tty = sys.stdout.isatty() and sys.stderr.isatty() and os.environ.get("TERM") != "dumb"
    verbose = mode == "verbose" or (mode == "auto" and not tty)
    animate = tty and not verbose and mode != "plain"
    tail: deque[str] = deque(maxlen=12)
    line_count = shown_count = 0
    pending = ""
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    started = last_draw = time.monotonic()
    cancelled = 0
    cancel_time = 0.0
    process = None
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
            if not verbose and safe.lstrip().startswith(notice_prefixes):
                if animate:
                    print("\r\x1b[2K", end="", file=sys.stderr)
                print(f"  │ {safe}", file=sys.stderr, flush=True)
                shown_count += 1
            elif safe.strip():
                tail.append(safe)

    try:
        title = _display_text(title)
        print(f"• {title}\n{_muted(f'  └ Full log: {log_path}', tty)}",
              file=sys.stderr, flush=True)
        environment = {**os.environ, "ELESIM_PROGRESS_ACTIVE": "1", "PYTHONUNBUFFERED": "1"}
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
            if not verbose and safe.lstrip().startswith(notice_prefixes):
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
                print(f"  │ {_fit_row(line, max(40, shutil.get_terminal_size().columns - 4))}", file=sys.stderr)
            omitted = max(0, line_count - shown_count - len(preview))
            print(_muted("  │ ...", tty), file=sys.stderr)
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
        transcript.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-dir", required=True, type=Path)
    parser.add_argument("--title", default="Runtime image build")
    parser.add_argument("--notice-prefix", action="append", default=[])
    parser.add_argument("--mode", choices=("auto", "compact", "plain", "verbose"),
                        default="verbose" if os.environ.get("ELESIM_VERBOSE") == "1"
                        else os.environ.get("ELESIM_BUILD_PROGRESS", "auto"))
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a build command is required")
    try:
        mode = "verbose" if os.environ.get("ELESIM_VERBOSE") == "1" else args.mode
        return run(command, args.log_dir, mode, title=args.title,
                   notice_prefixes=tuple(args.notice_prefix))
    except OSError as exc:
        print(f"Build progress error: {exc}", file=sys.stderr)
        return 74


if __name__ == "__main__":
    raise SystemExit(main())
