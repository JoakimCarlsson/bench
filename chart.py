"""Draw the results as dark-surface SVG charts for the README.

    python chart.py               reads results/latest.json

Writes results/overview.svg, the average slowdown of each
language against the fastest one per kernel, and
results/kernels.svg, a heatmap of every kernel's slowdown.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT / "results"
LANGS = ["c", "cpp", "zig", "rust"]
LABEL = {"c": "C", "cpp": "C++", "zig": "Zig", "rust": "Rust"}
FONT = "system-ui, -apple-system, 'Segoe UI', Helvetica, Arial, sans-serif"
CAP = 2.0


@dataclass(frozen=True)
class Theme:
    """Surface, ink and ramp of the chart colour scheme."""

    surface: str
    text_primary: str
    text_secondary: str
    grid: str
    bar: str
    ramp: tuple[str, ...]


THEME = Theme(
    surface="#1a1a19",
    text_primary="#ffffff",
    text_secondary="#c3c2b7",
    grid="#383835",
    bar="#3987e5",
    ramp=("#232a33", "#0d366b", "#184f95", "#256abf", "#3987e5", "#6da7ec", "#9ec5f4", "#cde2fb"),
)


def medians(path: Path) -> dict[str, dict[str, float]]:
    """Median of the per-run medians, in milliseconds, by kernel and language."""
    data = json.loads(path.read_text(encoding="utf-8"))
    return {
        case: {lang: statistics.median(s["median_ns"] for s in per[lang]) / 1e6 for lang in LANGS}
        for case, per in data["results"].items()
    }


def relative(times: dict[str, dict[str, float]]) -> dict[str, dict[str, float]]:
    """Each language's time divided by the fastest language's, per kernel."""
    return {case: {lang: t / min(per.values()) for lang, t in per.items()} for case, per in times.items()}


def geomean(values: list[float]) -> float:
    """Geometric mean."""
    return math.exp(sum(math.log(v) for v in values) / len(values))


def hex_rgb(color: str) -> tuple[int, int, int]:
    """A `#rrggbb` colour as integers."""
    return int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)


def luminance(color: str) -> float:
    """WCAG relative luminance."""

    def channel(c: int) -> float:
        s = c / 255
        return s / 12.92 if s <= 0.03928 else ((s + 0.055) / 1.055) ** 2.4

    r, g, b = hex_rgb(color)
    return 0.2126 * channel(r) + 0.7152 * channel(g) + 0.0722 * channel(b)


def ramp_color(theme: Theme, ratio: float) -> str:
    """Ramp step for a slowdown ratio, 1.0 at the first step and CAP or more at the last."""
    t = (min(ratio, CAP) - 1.0) / (CAP - 1.0)
    return theme.ramp[round(t * (len(theme.ramp) - 1))]


def ink_on(fill: str) -> str:
    """Primary text colour that reads on a fill: dark ink on light fills, white on dark."""
    return "#0b0b0b" if luminance(fill) > 0.32 else "#ffffff"


def text(x: float, y: float, content: str, fill: str, size: int = 13, anchor: str = "start", weight: int = 400) -> str:
    """An SVG text element."""
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" font-size="{size}" font-weight="{weight}" '
        f'text-anchor="{anchor}" font-family="{FONT}">{content}</text>'
    )


def svg(width: int, height: int, theme: Theme, body: list[str], title: str) -> str:
    """A complete SVG document on the theme's surface."""
    return "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
            f"<title>{title}</title>",
            f'<rect width="{width}" height="{height}" rx="8" fill="{theme.surface}"/>',
            *body,
            "</svg>",
            "",
        ]
    )


def bar_path(x: float, y: float, length: float, thickness: float, radius: float = 4.0) -> str:
    """A horizontal bar square at the baseline and rounded at its data end."""
    r = min(radius, length, thickness / 2)
    return (
        f'<path d="M{x:.1f},{y:.1f} h{length - r:.1f} a{r},{r} 0 0 1 {r},{r} '
        f'v{thickness - 2 * r:.1f} a{r},{r} 0 0 1 {-r},{r} h{-(length - r):.1f} z"'
    )


def overview(rel: dict[str, dict[str, float]], theme: Theme) -> str:
    """Average slowdown of each language against the fastest per kernel, as bars."""
    slow = {lang: (geomean([per[lang] for per in rel.values()]) - 1.0) * 100 for lang in LANGS}
    wins = {lang: sum(1 for per in rel.values() if per[lang] == min(per.values())) for lang in LANGS}
    kernels = len(rel)
    width, left, right = 720, 150, 90
    top, row, thickness = 78, 44, 22
    height = top + row * len(LANGS) + 40
    plot = width - left - right
    scale_max = max(10.0, math.ceil(max(slow.values()) / 10) * 10)
    body = [
        text(24, 32, "Average slowdown against the fastest language", theme.text_primary, 17, weight=600),
        text(24, 54, f"Geometric mean over {kernels} kernels; 0% would mean fastest on every kernel", theme.text_secondary, 13),
    ]
    for tick in range(0, int(scale_max) + 1, 10 if scale_max <= 50 else 20):
        x = left + plot * tick / scale_max
        body.append(f'<line x1="{x:.1f}" y1="{top - 6}" x2="{x:.1f}" y2="{top + row * len(LANGS) - 10}" stroke="{theme.grid}" stroke-width="1"/>')
        body.append(text(x, top + row * len(LANGS) + 8, f"{tick}%", theme.text_secondary, 12, "middle"))
    for i, lang in enumerate(sorted(LANGS, key=lambda l: slow[l])):
        y = top + i * row
        body.append(text(24, y + 15, LABEL[lang], theme.text_primary, 14, weight=600))
        body.append(text(64, y + 15, f"fastest on {wins[lang]}", theme.text_secondary, 12))
        length = max(plot * slow[lang] / scale_max, 2.0)
        body.append(bar_path(left, y, length, thickness) + f' fill="{theme.bar}"/>')
        body.append(text(left + length + 8, y + 16, f"{slow[lang]:.1f}%", theme.text_primary, 13, weight=600))
    return svg(width, height, theme, body, "Average slowdown against the fastest language")


def heatmap(rel: dict[str, dict[str, float]], times: dict[str, dict[str, float]], theme: Theme) -> str:
    """Every kernel's slowdown per language, as a heatmap with the times written in."""
    cases = list(rel)
    left, cell_w, cell_h, gap = 130, 120, 30, 2
    top = 104
    width = left + len(LANGS) * (cell_w + gap) + 24
    height = top + len(cases) * (cell_h + gap) + 70
    body = [
        text(24, 32, "Time per kernel, relative to the fastest language", theme.text_primary, 17, weight=600),
        text(24, 54, "Each cell: median milliseconds and slowdown. Darker is slower; the scale stops at 2x.", theme.text_secondary, 13),
    ]
    for j, lang in enumerate(LANGS):
        x = left + j * (cell_w + gap) + cell_w / 2
        body.append(text(x, top - 14, LABEL[lang], theme.text_primary, 14, "middle", 600))
    for i, case in enumerate(cases):
        y = top + i * (cell_h + gap)
        body.append(text(left - 12, y + cell_h / 2 + 5, case, theme.text_primary, 13, "end"))
        for j, lang in enumerate(LANGS):
            x = left + j * (cell_w + gap)
            ratio = rel[case][lang]
            fill = ramp_color(theme, ratio)
            ink = ink_on(fill)
            label = "fastest" if ratio == 1.0 else f"{ratio:.2f}x"
            body.append(f'<rect x="{x}" y="{y}" width="{cell_w}" height="{cell_h}" rx="4" fill="{fill}"><title>{case}, {LABEL[lang]}: {times[case][lang]:.2f} ms, {ratio:.2f}x</title></rect>')
            body.append(text(x + 10, y + cell_h / 2 + 5, f"{times[case][lang]:.1f} ms", ink, 12))
            body.append(text(x + cell_w - 10, y + cell_h / 2 + 5, label, ink, 12, "end", 600))
    legend_y = top + len(cases) * (cell_h + gap) + 24
    steps = len(theme.ramp)
    swatch = 40
    body.append(text(left - 12, legend_y + 13, "slowdown", theme.text_secondary, 12, "end"))
    for k, color in enumerate(theme.ramp):
        x = left + k * (swatch + gap)
        body.append(f'<rect x="{x}" y="{legend_y}" width="{swatch}" height="18" rx="3" fill="{color}"/>')
        if k in (0, steps - 1):
            label = "1x" if k == 0 else f"{CAP:.0f}x or more"
            body.append(text(x + (0 if k == 0 else swatch), legend_y + 36, label, theme.text_secondary, 12, "start" if k == 0 else "end"))
    return svg(width, height, theme, body, "Time per kernel, relative to the fastest language")


def main() -> int:
    """Read the latest results and write the two SVGs."""
    times = medians(RESULTS / "latest.json")
    rel = relative(times)
    (RESULTS / "overview.svg").write_text(overview(rel, THEME), encoding="utf-8")
    (RESULTS / "kernels.svg").write_text(heatmap(rel, times, THEME), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
