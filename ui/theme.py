from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


UI_ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class UiFontSpec:
    title_px: float = 22.0
    content_px: float = 18.0
    misc_px: float = 16.0
    title_fallback_scale: float = 1.15


FONT_SPEC = UiFontSpec()
INTER_DIR = UI_ROOT / "fonts" / "inter"
FONT_DIR = UI_ROOT / "fonts"
NOTO_CJK_DIR = FONT_DIR / "NotoSansCJK"
CONTENT_FONT_CANDIDATES = (
    NOTO_CJK_DIR / "NotoSansCJKkr-Regular.otf",
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJKkr-Regular.otf"),
    Path("/usr/share/fonts/truetype/noto/NotoSansKR-Regular.ttf"),
    Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf"),
    INTER_DIR / "Inter-Regular.ttf",
    Path("/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf"),
    Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
    Path("/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf"),
)
TITLE_FONT_CANDIDATES = (
    NOTO_CJK_DIR / "NotoSansCJKkr-Regular.otf",
    INTER_DIR / "Inter-SemiBold.ttf",
)
TITLE_FONT = next((p for p in TITLE_FONT_CANDIDATES if p.exists()), INTER_DIR / "Inter-SemiBold.ttf")


def add_font_with_korean_ranges(fonts, path: Path, size_px: float):
    """Load a font with Korean glyph ranges when the imgui binding exposes them."""
    glyph_ranges = None
    get_korean_ranges = getattr(fonts, "get_glyph_ranges_korean", None)
    if callable(get_korean_ranges):
        try:
            glyph_ranges = get_korean_ranges()
        except Exception:
            glyph_ranges = None
    if glyph_ranges is not None:
        try:
            return fonts.add_font_from_file_ttf(str(path), float(size_px), glyph_ranges=glyph_ranges)
        except TypeError:
            try:
                return fonts.add_font_from_file_ttf(str(path), float(size_px), None, glyph_ranges)
            except TypeError:
                pass
    return fonts.add_font_from_file_ttf(str(path), float(size_px))
