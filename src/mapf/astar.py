"""Space-time A* for a single agent under constraints."""
from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from .grid import Grid, Loc, Path_


@dataclass
class ConstraintTable:
    """Constraints for one agent.

    vertex: (loc, t)            -- may not occupy ``loc`` at time ``t``
    edge:   (prev, loc, t)      -- may not move ``prev -> loc`` arriving at time ``t``
    parked: loc -> t_from       -- ``loc`` is occupied by another agent for all t >= t_from
    """

    vertex: set[tuple[Loc, int]] = field(default_factory=set)
    edge: set[tuple[Loc, Loc, int]] = field(default_factory=set)
    parked: dict[Loc, int] = field(default_factory=dict)

    def blocked(self, prev: Loc, loc: Loc, t: int) -> bool:
        if (loc, t) in self.vertex:
            return True
        if prev != loc and (prev, loc, t) in self.edge:
            return True
        return loc in self.parked and t >= self.parked[loc]

    def last_vertex_time(self, loc: Loc) -> int:
        """Latest time a vertex constraint applies to ``loc`` (-1 if none)."""
        return max((t for (l, t) in self.vertex if l == loc), default=-1)

    def max_time(self) -> int:
        times = [t for _, t in self.vertex] + [t for *_, t in self.edge] + list(self.parked.values())
        return max(times, default=0)


def space_time_astar(
    grid: Grid,
    start: Loc,
    goal: Loc,
    h: dict[Loc, int],
    table: ConstraintTable | None = None,
    max_t: int | None = None,
) -> Path_ | None:
    """Shortest constraint-respecting path from ``start`` to ``goal``.

    The agent may wait. The goal only counts as reached once no later constraint
    forbids staying there. Returns ``None`` if no path exists within ``max_t``.
    """
    table = table or ConstraintTable()
    if start not in h or goal in table.parked:
        return None
    if max_t is None:
        max_t = table.max_time() + len(grid.free_cells())
    earliest_goal = table.last_vertex_time(goal) + 1
    if table.blocked(start, start, 0):
        return None

    counter = 0
    open_heap: list[tuple[int, int, int, int, Loc]] = [(h[start], h[start], 0, counter, start)]
    parent: dict[tuple[Loc, int], tuple[Loc, int] | None] = {(start, 0): None}
    closed: set[tuple[Loc, int]] = set()

    while open_heap:
        _, _, t, _, loc = heapq.heappop(open_heap)
        if (loc, t) in closed:
            continue
        closed.add((loc, t))
        if loc == goal and t >= earliest_goal:
            path: Path_ = []
            node: tuple[Loc, int] | None = (loc, t)
            while node is not None:
                path.append(node[0])
                node = parent[node]
            return path[::-1]
        if t >= max_t:
            continue
        for nxt in grid.neighbors(loc):
            if nxt not in h or table.blocked(loc, nxt, t + 1) or (nxt, t + 1) in closed:
                continue
            if (nxt, t + 1) not in parent:
                parent[(nxt, t + 1)] = (loc, t)
            counter += 1
            g = t + 1
            heapq.heappush(open_heap, (g + h[nxt], h[nxt], g, counter, nxt))
    return None
