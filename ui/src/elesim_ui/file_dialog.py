from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Optional

_GLFW_WINDOW: Any = None


def set_glfw_window(window: Any) -> None:
    """Register the GLFW window so native dialogs can take focus on macOS."""
    global _GLFW_WINDOW
    _GLFW_WINDOW = window


def _resolve_initial_dir(initial_dir: str) -> str:
    path = Path(str(initial_dir or "").strip()).expanduser()
    if path.is_file():
        path = path.parent
    if path.is_dir():
        return str(path.resolve())
    cwd = Path.cwd()
    return str(cwd) if cwd.is_dir() else os.getcwd()


def _applescript_escape(value: str) -> str:
    return str(value).replace("\\", "\\\\").replace('"', '\\"')


def _path_matches_extensions(path: str, extensions: tuple[str, ...]) -> bool:
    if not extensions:
        return True
    suffix = Path(path).suffix.lower()
    allowed = {str(ext).strip().lower() for ext in extensions if str(ext).strip()}
    allowed = {ext if ext.startswith(".") else f".{ext}" for ext in allowed}
    return suffix in allowed


def _suspend_glfw_window() -> None:
    window = _GLFW_WINDOW
    if window is None:
        return
    try:
        import glfw

        glfw.iconify_window(window)
        for _ in range(3):
            glfw.poll_events()
    except Exception:
        pass


def _resume_glfw_window() -> None:
    window = _GLFW_WINDOW
    if window is None:
        return
    try:
        import glfw

        glfw.restore_window(window)
        glfw.focus_window(window)
        for _ in range(3):
            glfw.poll_events()
    except Exception:
        pass


def _browse_open_file_macos(
    *,
    title: str,
    initial_dir: str,
    extensions: tuple[str, ...],
) -> str | None:
    initial = _resolve_initial_dir(initial_dir)
    prompt = _applescript_escape(title or "Select file")
    default_location = _applescript_escape(initial)
    # Do not use AppleScript "of type" filters here: bare extensions like "json"
    # often hide every file on recent macOS builds.
    script = f"""
tell application "System Events" to activate
try
    set picked to choose file with prompt "{prompt}" default location (POSIX file "{default_location}")
    return POSIX path of picked
on error errMsg number errNum
    if errNum is -128 then
        return ""
    end if
    error errMsg number errNum
end try
""".strip()
    _suspend_glfw_window()
    try:
        proc = subprocess.run(
            ["osascript", "-e", script],
            check=False,
            capture_output=True,
            text=True,
            timeout=120.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"[ui] file dialog failed: {exc}")
        return None
    finally:
        _resume_glfw_window()

    if proc.returncode != 0:
        err = str(proc.stderr or proc.stdout or "").strip()
        if err:
            print(f"[ui] file dialog error: {err}")
        return None
    selected = str(proc.stdout or "").strip()
    if not selected:
        return None
    if not _path_matches_extensions(selected, extensions):
        print(f"[ui] file dialog: rejected non-matching extension: {selected}")
        return None
    return selected


def _browse_open_file_tk(
    *,
    title: str,
    initial_dir: str,
    extensions: tuple[str, ...],
) -> str | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except Exception:
        return None

    initial = _resolve_initial_dir(initial_dir)
    filetypes: list[tuple[str, str]] = []
    for ext in extensions:
        pat = str(ext).strip()
        if not pat:
            continue
        if not pat.startswith("*"):
            pat = f"*{pat}" if pat.startswith(".") else f"*.{pat}"
        label = pat.replace("*", "").upper().strip(".") or "Files"
        filetypes.append((f"{label} files", pat))
    if not filetypes:
        filetypes = [("All files", "*.*")]

    root = tk.Tk()
    root.withdraw()
    try:
        root.update_idletasks()
        selected = filedialog.askopenfilename(
            title=str(title or "Select file"),
            initialdir=initial,
            filetypes=[*filetypes, ("All files", "*.*")],
        )
    finally:
        root.destroy()
    selected = str(selected or "").strip()
    if not selected:
        return None
    if not _path_matches_extensions(selected, extensions):
        return None
    return selected


def browse_open_file_path(
    *,
    title: str,
    initial_dir: str,
    extensions: tuple[str, ...] = (".json",),
) -> str | None:
    """Open a native file picker without initializing Tk on macOS."""
    if sys.platform == "darwin":
        return _browse_open_file_macos(title=title, initial_dir=initial_dir, extensions=extensions)
    return _browse_open_file_tk(title=title, initial_dir=initial_dir, extensions=extensions)
