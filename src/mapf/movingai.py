"""Loaders for the MovingAI MAPF benchmark format (https://movingai.com/benchmarks/mapf/).

Grid maps become 4-connected graphs (node id = index of the free cell in row-major order,
``pos = (col, row)``).
"""

from __future__ import annotations

from pathlib import Path

from .graph import Graph, Instance

PASSABLE = ".GS"  # ground, grass, swamp; everything else (@ O T W) is blocked


def load_map(path: str | Path) -> Graph:
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.strip() == "map") + 1
    except StopIteration:
        raise ValueError(f"{path}: missing 'map' header line") from None
    rows = [ln for ln in lines[start:] if ln]
    return Graph.from_grid(["".join("." if ch in PASSABLE else "#" for ch in row) for row in rows])


def load_scen(map_path: str | Path, scen_path: str | Path, num_agents: int) -> Instance:
    """Build an instance from the first ``num_agents`` usable scenario lines.

    Scenario lines are ``bucket map width height start_x start_y goal_x goal_y optimal``
    (x = column, y = row). Lines that reuse an earlier start or goal, or touch a wall, are skipped.
    """
    graph = load_map(map_path)
    at = {(int(x), int(y)): n for n, (x, y) in graph.pos.items()}
    starts, goals = [], []
    for line in Path(scen_path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 9 or parts[0] == "version":
            continue
        sx, sy, gx, gy = map(int, parts[4:8])
        s, g = at.get((sx, sy)), at.get((gx, gy))
        if s is None or g is None or s in starts or g in goals:
            continue
        starts.append(s)
        goals.append(g)
        if len(starts) == num_agents:
            break
    if len(starts) < num_agents:
        raise ValueError(f"scenario only has {len(starts)} usable agents, {num_agents} requested")
    return Instance(graph, tuple(starts), tuple(goals))
