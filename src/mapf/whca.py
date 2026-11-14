"""WHCA* (Silver 2005): Windowed Hierarchical Cooperative A*.

Prioritized planning over a short time window, repeated as the agents move: in each phase the
agents plan ``window`` steps ahead in priority order against each other's reservations, the
first ``step`` steps are executed, and everybody re-plans. Fast and memory-friendly, but
incomplete (agents can livelock), so it gives up after ``max_steps``.
"""

from __future__ import annotations

import heapq
import itertools
import random
import time

from .astar import ConstraintTable
from .graph import Graph, Instance, Node, NodePath, Solution


def windowed_astar(
    graph: Graph,
    start: Node,
    goal: Node,
    h: dict[Node, int],
    table: ConstraintTable,
    window: int,
) -> NodePath | None:
    """Best path of exactly ``window`` steps from ``start`` (cost + distance-to-goal at the end).

    Waiting on the goal is free so agents are not pulled away from it needlessly.
    """
    counter = itertools.count()
    heap = [(h[start], 0, next(counter), start, 0)]
    best_g = {(start, 0): 0}
    parent: dict[tuple[Node, int], Node | None] = {(start, 0): None}
    while heap:
        _, g, _, loc, t = heapq.heappop(heap)
        if best_g.get((loc, t), g) < g:
            continue
        if t == window:
            path = [loc]
            for k in range(t, 0, -1):
                loc = parent[(loc, k)]  # type: ignore[assignment]
                path.append(loc)
            return path[::-1]
        for nxt in graph.moves(loc):
            if nxt not in h or table.blocked(loc, nxt, t + 1):
                continue
            ng = g + (0 if loc == goal and nxt == goal else 1)
            if ng < best_g.get((nxt, t + 1), float("inf")):
                best_g[(nxt, t + 1)] = ng
                parent[(nxt, t + 1)] = loc
                heapq.heappush(heap, (ng + h[nxt], ng, next(counter), nxt, t + 1))
    return None


def whca(
    instance: Instance,
    window: int = 8,
    step: int = 4,
    max_steps: int = 1000,
    timeout: float | None = None,
    seed: int = 0,
) -> Solution | None:
    if not 1 <= step <= window:
        raise ValueError("need 1 <= step <= window")
    t0 = time.perf_counter()
    rng = random.Random(seed)
    graph, n = instance.graph, instance.num_agents
    goals = instance.goals
    dist = [graph.distances_from(g) for g in goals]
    if any(instance.starts[a] not in dist[a] for a in range(n)):
        return None
    cur = list(instance.starts)
    paths = [[s] for s in cur]
    bumped: list[int] = []  # agents that failed to plan: they go first in the next attempt

    while len(paths[0]) - 1 < max_steps:
        if all(cur[a] == goals[a] for a in range(n)):
            return Solution(paths, 0, time.perf_counter() - t0, {"window": window, "step": step})
        if timeout is not None and time.perf_counter() - t0 > timeout:
            return None
        # agents still travelling plan first; the ones parked on their goal yield if needed
        rest = [a for a in range(n) if a not in bumped]
        rng.shuffle(rest)
        order = bumped + sorted(rest, key=lambda a: (cur[a] == goals[a], -dist[a][cur[a]]))
        table = ConstraintTable()
        plans: dict[int, NodePath] = {}
        failed = None
        for a in order:
            plan = windowed_astar(graph, cur[a], goals[a], dist[a], table, window)
            if plan is None:
                failed = a
                break
            plans[a] = plan
            for t, loc in enumerate(plan):
                table.vertex.add((loc, t))
                if t > 0:
                    table.edge.add((loc, plan[t - 1], t))
        if failed is not None:
            if bumped and bumped[0] == failed:
                return None  # even with top priority it cannot move: give up
            bumped = [failed] + [a for a in bumped if a != failed]
            continue
        bumped = []
        for t in range(1, step + 1):
            for a in range(n):
                cur[a] = plans[a][t]
                paths[a].append(cur[a])
            if all(cur[a] == goals[a] for a in range(n)):
                break
    return None
