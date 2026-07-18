from __future__ import annotations

import json
import os
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional


@dataclass
class RunContext:
    run_id: str
    arm_preset: str = "neutral"
    go2_motion: str = ""
    gaze_mode: str = "off"
    requested_gaze_mode: str = ""
    actual_gaze_mode: str = ""
    preview_enable: bool = False
    preview_used_ratio: float = 0.0
    preview_fallback_ratio: float = 0.0
    preview_type: str = ""
    gait_period_s: float = 0.0
    preview_horizon_s: float = 0.0
    gait_template_path: str = ""
    pitch_trim_config: dict[str, float] = field(default_factory=dict)
    notes: str = ""
    git_commit: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def git_commit_hash() -> str:
        try:
            out = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=2.0,
            )
            return str(out).strip()
        except Exception:
            return ""

    @classmethod
    def from_cli(
        cls,
        *,
        run_id: str,
        arm_preset: str = "neutral",
        go2_motion: str = "",
        gaze_mode: str = "off",
        pitch_trim_config: Optional[dict[str, float]] = None,
        notes: str = "",
        **extra: Any,
    ) -> RunContext:
        return cls(
            run_id=str(run_id),
            arm_preset=str(arm_preset),
            go2_motion=str(go2_motion),
            gaze_mode=str(gaze_mode),
            requested_gaze_mode=str(gaze_mode),
            actual_gaze_mode=str(gaze_mode),
            pitch_trim_config=dict(pitch_trim_config or {}),
            notes=str(notes),
            git_commit=cls.git_commit_hash(),
            extra=dict(extra),
        )

    def with_preview_stats(
        self,
        *,
        preview_enable: bool,
        preview_used_ratio: float,
        preview_fallback_ratio: float,
        preview_type: str = "",
        gait_period_s: float = 0.0,
        preview_horizon_s: float = 0.0,
        gait_template_path: str = "",
    ) -> RunContext:
        requested = str(self.requested_gaze_mode or self.gaze_mode)
        if requested == "pitch_preview":
            actual = "pitch_preview" if float(preview_used_ratio) >= 0.5 else "uv"
        else:
            actual = requested
        return RunContext(
            run_id=self.run_id,
            arm_preset=self.arm_preset,
            go2_motion=self.go2_motion,
            gaze_mode=self.gaze_mode,
            requested_gaze_mode=requested,
            actual_gaze_mode=actual,
            preview_enable=bool(preview_enable),
            preview_used_ratio=float(preview_used_ratio),
            preview_fallback_ratio=float(preview_fallback_ratio),
            preview_type=str(preview_type),
            gait_period_s=float(gait_period_s),
            preview_horizon_s=float(preview_horizon_s),
            gait_template_path=str(gait_template_path),
            pitch_trim_config=dict(self.pitch_trim_config),
            notes=self.notes,
            git_commit=self.git_commit,
            extra=dict(self.extra),
        )

    def finalize_meta(self, log_dir: str | Path, **kwargs: Any) -> Path:
        ctx = self
        if kwargs:
            payload = asdict(self)
            payload.update(kwargs)
            path = Path(log_dir) / f"{self.run_id}_meta.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            return path
        return self.write_meta(log_dir)

    def validate_env_run_id(self, *, strict: bool = True) -> bool:
        env_rid = os.environ.get("ELESIM_RUN_ID", "").strip()
        if not env_rid:
            if strict:
                raise ValueError(
                    "ELESIM_RUN_ID is not set in environment; start sim_agent.py with matching run id"
                )
            return False
        if env_rid != str(self.run_id):
            msg = f"run_id mismatch: CLI={self.run_id!r} env ELESIM_RUN_ID={env_rid!r}"
            if strict:
                raise ValueError(msg)
            print(f"[run_context] warning: {msg}")
            return False
        return True

    def write_meta(self, log_dir: str | Path) -> Path:
        path = Path(log_dir) / f"{self.run_id}_meta.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = asdict(self)
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return path
