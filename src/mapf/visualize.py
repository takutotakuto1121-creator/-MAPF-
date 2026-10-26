"""Text and animated-SVG rendering of MAPF solutions."""

from __future__ import annotations

import math

from .graph import Graph, Instance, Node, NodePath
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


def _on_lattice(graph: Graph) -> bool:
    return len(graph.pos) == len(graph) and all(
        float(x).is_integer() and float(y).is_integer() for x, y in graph.pos.values()
    )


def render_ascii(instance: Instance, paths: list[NodePath], t: int) -> str:
    """Draw time step ``t``. Graphs with integer ``pos`` on every node are drawn on a character
    lattice (``#`` = no node; edges are not drawn); other graphs are listed as ``agent: node``."""
    graph = instance.graph
    if not _on_lattice(graph):
        return "\n".join(
            f"agent {a}: {position(p, t)}  (goal {g})"
            for a, (p, g) in enumerate(zip(paths, instance.goals, strict=True))
        )
    xs = [int(x) for x, _ in graph.pos.values()]
    ys = [int(y) for _, y in graph.pos.values()]
    x0, y0 = min(xs), min(ys)
    canvas = [["#"] * (max(xs) - x0 + 1) for _ in range(max(ys) - y0 + 1)]

    def put(node: Node, ch: str) -> None:
        x, y = graph.pos[node]
        canvas[int(y) - y0][int(x) - x0] = ch

    for n in graph.nodes:
        put(n, ".")
    for goal in instance.goals:
        put(goal, "*")
    for a, p in enumerate(paths):
        put(position(p, t), _SYMBOLS[a % len(_SYMBOLS)])
    return "\n".join("".join(row) for row in canvas)


def render_ascii_all(instance: Instance, paths: list[NodePath]) -> str:
    horizon = max(len(p) for p in paths)
    return "\n\n".join(f"t={t}\n{render_ascii(instance, paths, t)}" for t in range(horizon))


def _layout(graph: Graph) -> dict[Node, tuple[float, float]]:
    """Node coordinates in 'cell' units; circular layout when positions are missing."""
    if len(graph.pos) == len(graph):
        return {n: (float(x), float(y)) for n, (x, y) in graph.pos.items()}
    radius = max(1.5, len(graph) / (2 * math.pi))
    return {
        n: (radius * math.cos(2 * math.pi * i / len(graph)), radius * math.sin(2 * math.pi * i / len(graph)))
        for i, n in enumerate(graph.nodes)
    }


def render_svg(instance: Instance, paths: list[NodePath], cell: int = 30, step: float = 0.5) -> str:
    """Animated SVG (SMIL); ``step`` is seconds per time step."""
    graph = instance.graph
    coords = _layout(graph)
    x0 = min(x for x, _ in coords.values())
    y0 = min(y for _, y in coords.values())
    margin = 0.7

    def xy(n: Node) -> tuple[float, float]:
        x, y = coords[n]
        return round((x - x0 + margin) * cell, 2), round((y - y0 + margin) * cell, 2)

    w = round((max(x for x, _ in coords.values()) - x0 + 2 * margin) * cell)
    h = round((max(y for _, y in coords.values()) - y0 + 2 * margin) * cell)
    horizon = max(len(p) for p in paths) - 1
    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        f'<rect width="{w}" height="{h}" fill="#fafafa"/>',
    ]
    for u, v in graph.edges:
        (ux, uy), (vx, vy) = xy(u), xy(v)
        out.append(f'<line x1="{ux}" y1="{uy}" x2="{vx}" y2="{vy}" stroke="#bbb" stroke-width="2"/>')
    for n in graph.nodes:
        nx, ny = xy(n)
        out.append(f'<circle cx="{nx}" cy="{ny}" r="{cell * 0.14}" fill="#999"/>')
    if len(graph) <= 40:  # labels only when they stay readable
        for n in graph.nodes:
            nx, ny = xy(n)
            out.append(
                f'<text x="{nx}" y="{ny - cell * 0.22}" font-size="{cell * 0.3}" fill="#666" '
                f'text-anchor="middle" font-family="sans-serif">{n}</text>'
            )
    for a, goal in enumerate(instance.goals):
        gx, gy = xy(goal)
        r = cell * 0.4
        out.append(
            f'<rect x="{gx - r}" y="{gy - r}" width="{2 * r}" height="{2 * r}" fill="none" '
            f'stroke="{_COLORS[a % len(_COLORS)]}" stroke-width="2" stroke-dasharray="3"/>'
        )
    dur = max(horizon, 1) * step
    for a, p in enumerate(paths):
        col = _COLORS[a % len(_COLORS)]
        pts = [xy(position(p, t)) for t in range(horizon + 1)]
        xs = ";".join(str(x) for x, _ in pts)
        ys = ";".join(str(y) for _, y in pts)
        out.append(
            f'<circle r="{cell * 0.32}" fill="{col}" cx="{pts[0][0]}" cy="{pts[0][1]}">'
            f'<animate attributeName="cx" values="{xs}" dur="{dur}s" repeatCount="indefinite"/>'
            f'<animate attributeName="cy" values="{ys}" dur="{dur}s" repeatCount="indefinite"/></circle>'
        )
    out.append("</svg>")
    return "\n".join(out)
