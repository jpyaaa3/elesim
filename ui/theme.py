from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class UiFontSpec:
    title_px: float = 18.0
    content_px: float = 16.0
    misc_px: float = 14.0
    title_fallback_scale: float = 1.10


FONT_SPEC = UiFontSpec()
INTER_DIR = REPO_ROOT / "fonts" / "inter"
CONTENT_FONT_CANDIDATES = (
    INTER_DIR / "Inter-Regular.ttf",
    Path("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf"),
    Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"),
)
TITLE_FONT = INTER_DIR / "Inter-SemiBold.ttf"
