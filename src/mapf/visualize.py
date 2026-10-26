"""Text and animated-SVG rendering of MAPF solutions."""

from __future__ import annotations

from .grid import Instance, Path_
from .validate import position

_SYMBOLS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
_COLORS = [
    "#e6194b",
    "#3cb44b",
    "#4363d8",
    "#f58231",
    "#911eb4",
    "#42d4f4",
    "#f032e6",
    "#bfef45",
    "#fabed4",
    "#469990",
    "#dcbeff",
    "#9a6324",
    "#800000",
    "#808000",
]


def render_ascii(instance: Instance, paths: list[Path_], t: int) -> str:
    g = instance.grid
    canvas = [["#" if (r, c) in g.obstacles else "." for c in range(g.cols)] for r in range(g.rows)]
    for goal in instance.goals:
        canvas[goal[0]][goal[1]] = "*"
    for a, p in enumerate(paths):
        r, c = position(p, t)
        canvas[r][c] = _SYMBOLS[a % len(_SYMBOLS)]
    return "\n".join("".join(row) for row in canvas)


def render_ascii_all(instance: Instance, paths: list[Path_]) -> str:
    horizon = max(len(p) for p in paths)
    return "\n\n".join(f"t={t}\n{render_ascii(instance, paths, t)}" for t in range(horizon))


def render_svg(instance: Instance, paths: list[Path_], cell: int = 30, step: float = 0.5) -> str:
    """Animated SVG (SMIL); ``step`` is seconds per time step."""
    g = instance.grid
    w, h = g.cols * cell, g.rows * cell
    horizon = max(len(p) for p in paths) - 1
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        f'<rect width="{w}" height="{h}" fill="#fafafa"/>',
    ]
    for r, c in g.obstacles:
        out.append(f'<rect x="{c * cell}" y="{r * cell}" width="{cell}" height="{cell}" fill="#333"/>')
    for a, goal in enumerate(instance.goals):
        col = _COLORS[a % len(_COLORS)]
        out.append(
            f'<rect x="{goal[1] * cell + 4}" y="{goal[0] * cell + 4}" width="{cell - 8}" height="{cell - 8}" '
            f'fill="none" stroke="{col}" stroke-width="2" stroke-dasharray="3"/>'
        )
    dur = max(horizon, 1) * step
    for a, p in enumerate(paths):
        col = _COLORS[a % len(_COLORS)]
        xs = ";".join(str((position(p, t)[1] + 0.5) * cell) for t in range(horizon + 1))
        ys = ";".join(str((position(p, t)[0] + 0.5) * cell) for t in range(horizon + 1))
        cx0, cy0 = (p[0][1] + 0.5) * cell, (p[0][0] + 0.5) * cell
        out.append(
            f'<circle r="{cell * 0.32}" fill="{col}" cx="{cx0}" cy="{cy0}">'
            f'<animate attributeName="cx" values="{xs}" dur="{dur}s" repeatCount="indefinite"/>'
            f'<animate attributeName="cy" values="{ys}" dur="{dur}s" repeatCount="indefinite"/></circle>'
        )
    out.append("</svg>")
    return "\n".join(out)
