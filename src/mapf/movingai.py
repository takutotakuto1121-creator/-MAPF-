"""Loaders for the MovingAI MAPF benchmark format (https://movingai.com/benchmarks/mapf/)."""

from __future__ import annotations

from pathlib import Path

from .grid import Grid, Instance

PASSABLE = ".GS"  # ground, grass, swamp; everything else (@ O T W) is blocked


def load_map(path: str | Path) -> Grid:
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.strip() == "map") + 1
    except StopIteration:
        raise ValueError(f"{path}: missing 'map' header line") from None
    rows = [ln for ln in lines[start:] if ln]
    return Grid.from_lines(["".join("." if ch in PASSABLE else "#" for ch in row) for row in rows])


def load_scen(map_path: str | Path, scen_path: str | Path, num_agents: int) -> Instance:
    """Build an instance from the first ``num_agents`` usable scenario lines.

    Scenario lines are ``bucket map width height start_x start_y goal_x goal_y optimal``
    (x = column, y = row). Lines that reuse an earlier start or goal are skipped.
    """
    grid = load_map(map_path)
    starts, goals = [], []
    for line in Path(scen_path).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 9 or parts[0] == "version":
            continue
        sx, sy, gx, gy = map(int, parts[4:8])
        s, g = (sy, sx), (gy, gx)
        if s in starts or g in goals or not grid.is_free(s) or not grid.is_free(g):
            continue
        starts.append(s)
        goals.append(g)
        if len(starts) == num_agents:
            break
    if len(starts) < num_agents:
        raise ValueError(f"scenario only has {len(starts)} usable agents, {num_agents} requested")
    return Instance(grid, tuple(starts), tuple(goals))
