"""Grid map, MAPF instance and solution types.

Coordinates are ``(row, col)``; ``(0, 0)`` is the top-left cell.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

Loc = tuple[int, int]
Path_ = list[Loc]

MOVES: tuple[Loc, ...] = ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1))  # wait + 4-neighbourhood


@dataclass(frozen=True)
class Grid:
    rows: int
    cols: int
    obstacles: frozenset[Loc]

    @classmethod
    def from_lines(cls, lines: list[str]) -> Grid:
        if not lines:
            raise ValueError("empty map")
        cols = len(lines[0])
        if any(len(line) != cols for line in lines):
            raise ValueError("map rows must have equal length")
        obstacles = frozenset(
            (r, c) for r, line in enumerate(lines) for c, ch in enumerate(line) if ch in "#@T"
        )
        return cls(len(lines), cols, obstacles)

    def in_bounds(self, loc: Loc) -> bool:
        return 0 <= loc[0] < self.rows and 0 <= loc[1] < self.cols

    def is_free(self, loc: Loc) -> bool:
        return self.in_bounds(loc) and loc not in self.obstacles

    def free_cells(self) -> list[Loc]:
        return [(r, c) for r in range(self.rows) for c in range(self.cols) if (r, c) not in self.obstacles]

    def neighbors(self, loc: Loc) -> list[Loc]:
        """Cells reachable in one step from ``loc`` (including ``loc`` itself = wait)."""
        out = []
        for dr, dc in MOVES:
            nxt = (loc[0] + dr, loc[1] + dc)
            if self.is_free(nxt):
                out.append(nxt)
        return out

    def distances_from(self, goal: Loc) -> dict[Loc, int]:
        """BFS shortest-path distance from every reachable cell to ``goal``."""
        dist = {goal: 0}
        queue = deque([goal])
        while queue:
            cur = queue.popleft()
            for nxt in self.neighbors(cur):
                if nxt not in dist:
                    dist[nxt] = dist[cur] + 1
                    queue.append(nxt)
        return dist


@dataclass(frozen=True)
class Instance:
    grid: Grid
    starts: tuple[Loc, ...]
    goals: tuple[Loc, ...]

    def __post_init__(self) -> None:
        if len(self.starts) != len(self.goals):
            raise ValueError("starts and goals must have the same length")
        for name, locs in (("start", self.starts), ("goal", self.goals)):
            if len(set(locs)) != len(locs):
                raise ValueError(f"duplicate {name} locations")
            for loc in locs:
                if not self.grid.is_free(loc):
                    raise ValueError(f"{name} {loc} is not a free cell")

    @property
    def num_agents(self) -> int:
        return len(self.starts)

    @classmethod
    def load(cls, path: str | Path) -> Instance:
        """Load a scenario file.

        Format: map lines (``.`` free, ``#`` obstacle), a line ``---``, then one
        ``start_row start_col goal_row goal_col`` line per agent. ``//`` starts a comment.
        """
        text = Path(path).read_text(encoding="utf-8")
        map_lines: list[str] = []
        agent_lines: list[str] = []
        target = map_lines
        for raw in text.splitlines():
            line = raw.split("//")[0].rstrip()
            if not line.strip():
                continue
            if line.strip() == "---":
                target = agent_lines
                continue
            target.append(line.strip())
        grid = Grid.from_lines(map_lines)
        starts, goals = [], []
        for line in agent_lines:
            sr, sc, gr, gc = map(int, line.split())
            starts.append((sr, sc))
            goals.append((gr, gc))
        return cls(grid, tuple(starts), tuple(goals))

    @classmethod
    def random(cls, grid: Grid, num_agents: int, seed: int | None = None) -> Instance:
        """Random starts/goals; every agent's goal is reachable from its start."""
        rng = random.Random(seed)
        cells = grid.free_cells()
        if num_agents > len(cells):
            raise ValueError("more agents than free cells")
        for _ in range(1000):
            starts = rng.sample(cells, num_agents)
            goals = rng.sample(cells, num_agents)
            if all(g in grid.distances_from(s) for s, g in zip(starts, goals, strict=True)):
                return cls(grid, tuple(starts), tuple(goals))
        raise RuntimeError("could not generate a solvable-looking instance")


def path_cost(path: Path_) -> int:
    """Time step at which the agent reaches its final cell for good."""
    end = len(path) - 1
    while end > 0 and path[end - 1] == path[-1]:
        end -= 1
    return end


@dataclass
class Solution:
    paths: list[Path_]
    expanded: int = 0
    runtime: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def sum_of_costs(self) -> int:
        """Sum over agents of the time step at which each agent reaches its goal for good."""
        return sum(path_cost(p) for p in self.paths)

    @property
    def makespan(self) -> int:
        return max(len(p) for p in self.paths) - 1
