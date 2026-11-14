"""Enhanced CBS (Barer et al. 2014): bounded-suboptimal CBS using focal search on both levels.

The returned sum of costs is at most ``w`` times the optimum. ``w = 1`` is optimal; larger
``w`` trades quality for speed by preferring nodes and paths with fewer conflicts.
"""

from __future__ import annotations

import heapq
import itertools
import time
from collections import Counter

from .astar import ConstraintTable
from .cbs import Constraint, _table_for
from .graph import Graph, Instance, Node, NodePath, Solution, path_cost
from .validate import count_conflicts, find_conflict

INF = float("inf")


class FocalList:
    """Open list with a focal sublist.

    Each item has ``low`` (a lower bound on the optimum below it), ``adm`` (its own cost) and
    ``key`` (the secondary ordering). ``pop`` returns the item with the smallest ``key`` among
    those with ``adm <= w * min(low)``; if none qualifies, the item with the smallest ``low``.
    """

    def __init__(self, w: float) -> None:
        self.w = w
        self._ids = itertools.count()
        self._items: dict[int, object] = {}
        self._open: list[tuple[float, int]] = []
        self._pending: list[tuple[float, object, int]] = []
        self._focal: list[tuple[object, float, int]] = []

    def __len__(self) -> int:
        return len(self._items)

    def push(self, item: object, low: float, adm: float, key: object) -> None:
        i = next(self._ids)
        self._items[i] = item
        heapq.heappush(self._open, (low, i))
        if adm <= self.w * self.min_low():
            heapq.heappush(self._focal, (key, adm, i))
        else:
            heapq.heappush(self._pending, (adm, key, i))

    def min_low(self) -> float:
        while self._open and self._open[0][1] not in self._items:
            heapq.heappop(self._open)
        return self._open[0][0] if self._open else INF

    def pop(self) -> object:
        bound = self.w * self.min_low()
        while self._pending and self._pending[0][0] <= bound:
            adm, key, i = heapq.heappop(self._pending)
            if i in self._items:
                heapq.heappush(self._focal, (key, adm, i))
        while self._focal:
            _, _, i = heapq.heappop(self._focal)
            if i in self._items:
                return self._items.pop(i)
        while self._open:  # nothing in focal: fall back to the best lower bound
            _, i = heapq.heappop(self._open)
            if i in self._items:
                return self._items.pop(i)
        raise IndexError("pop from empty FocalList")


class PathIndex:
    """Occupancy of the other agents' paths, to count conflicts a candidate move would cause."""

    def __init__(self, paths: list[NodePath | None], skip: int | None = None) -> None:
        self.vertex: Counter = Counter()
        self.edge: Counter = Counter()
        self.parked: dict[Node, list[int]] = {}
        for a, p in enumerate(paths):
            if p is None or a == skip:
                continue
            for t, loc in enumerate(p):
                self.vertex[(loc, t)] += 1
                if t > 0 and p[t - 1] != loc:
                    self.edge[(p[t - 1], loc, t)] += 1
            self.parked.setdefault(p[-1], []).append(len(p))

    def conflicts(self, prev: Node, loc: Node, t: int) -> int:
        c = self.vertex.get((loc, t), 0)
        if prev != loc:
            c += self.edge.get((loc, prev, t), 0)
        parked = self.parked.get(loc)
        if parked:
            c += sum(1 for start in parked if t >= start)
        return c


def focal_astar(
    graph: Graph,
    start: Node,
    goal: Node,
    h: dict[Node, int],
    table: ConstraintTable,
    index: PathIndex,
    w: float,
    max_t: int | None = None,
) -> tuple[NodePath, int] | None:
    """Space-time A* choosing, among nodes with ``f <= w * f_min``, the one with fewest conflicts.

    Returns ``(path, f_min)`` where ``f_min`` lower-bounds this agent's optimal cost.
    """
    if start not in h or goal in table.parked or table.blocked(start, start, 0):
        return None
    if max_t is None:
        max_t = table.max_time() + len(graph)
    earliest_goal = table.last_vertex_time(goal) + 1
    fl = FocalList(w)
    fl.push((start, 0, 0), h[start], h[start], (0, h[start]))
    parent: dict[tuple[Node, int], Node | None] = {(start, 0): None}
    closed: set[tuple[Node, int]] = set()
    while len(fl):
        f_min = fl.min_low()
        loc, t, conf = fl.pop()  # type: ignore[misc]
        if (loc, t) in closed:
            continue
        closed.add((loc, t))
        if loc == goal and t >= earliest_goal:
            path: NodePath = [loc]
            cur = loc
            for k in range(t, 0, -1):
                cur = parent[(cur, k)]  # type: ignore[assignment]
                path.append(cur)
            return path[::-1], int(f_min)
        if t >= max_t:
            continue
        for nxt in graph.moves(loc):
            if nxt not in h or (nxt, t + 1) in parent or table.blocked(loc, nxt, t + 1):
                continue
            parent[(nxt, t + 1)] = loc
            c = conf + index.conflicts(loc, nxt, t + 1)
            f = t + 1 + h[nxt]
            fl.push((nxt, t + 1, c), f, f, (c, f))
    return None


def ecbs(
    instance: Instance, w: float = 1.5, max_nodes: int = 100_000, timeout: float | None = None
) -> Solution | None:
    """Return a solution with sum of costs <= ``w`` * optimal, or ``None`` (unsolvable or out of budget)."""
    if w < 1:
        raise ValueError("w must be >= 1")
    t0 = time.perf_counter()
    graph, n = instance.graph, instance.num_agents
    heuristics = [graph.distances_from(g) for g in instance.goals]

    paths: list[NodePath | None] = [None] * n
    lbs = [0] * n
    for a in range(n):
        res = focal_astar(
            graph, instance.starts[a], instance.goals[a], heuristics[a],
            ConstraintTable(), PathIndex(paths), w,
        )  # fmt: skip
        if res is None:
            return None
        paths[a], lbs[a] = res

    def make_node(constraints, paths, lbs):
        cost = sum(path_cost(p) for p in paths)
        return (constraints, paths, lbs, cost, count_conflicts(paths))

    root = make_node((), paths, lbs)
    open_list = FocalList(w)
    open_list.push(root, sum(lbs), root[3], (root[4], root[3]))
    expanded = 0
    while len(open_list):
        if expanded >= max_nodes or (timeout is not None and time.perf_counter() - t0 > timeout):
            return None
        constraints, cur_paths, cur_lbs, _, _ = open_list.pop()  # type: ignore[misc]
        expanded += 1
        conflict = find_conflict(cur_paths)
        if conflict is None:
            return Solution(
                cur_paths, expanded, time.perf_counter() - t0, {"w": w, "lower_bound": sum(cur_lbs)}
            )
        if conflict.prev is None:
            new = [
                Constraint(conflict.a1, conflict.loc, conflict.t),
                Constraint(conflict.a2, conflict.loc, conflict.t),
            ]
        else:
            new = [
                Constraint(conflict.a1, conflict.loc, conflict.t, conflict.prev),
                Constraint(conflict.a2, conflict.prev, conflict.t, conflict.loc),
            ]
        for c in new:
            child_constraints = constraints + (c,)
            res = focal_astar(
                graph, instance.starts[c.agent], instance.goals[c.agent], heuristics[c.agent],
                _table_for(c.agent, child_constraints), PathIndex(cur_paths, skip=c.agent), w,
            )  # fmt: skip
            if res is None:
                continue
            child_paths = list(cur_paths)
            child_paths[c.agent] = res[0]
            child_lbs = list(cur_lbs)
            child_lbs[c.agent] = max(child_lbs[c.agent], res[1])
            node = make_node(child_constraints, child_paths, child_lbs)
            open_list.push(node, sum(child_lbs), node[3], (node[4], node[3]))
    return None
