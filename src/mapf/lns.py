"""MAPF-LNS (Li et al. 2021): large neighbourhood search that improves a Prioritized Planning solution.

Repeatedly removes a few agents, re-plans them against all the others, and keeps the result
if the sum of costs does not get worse. Anytime: the longer it runs the better the solution.
"""

from __future__ import annotations

import random
import time
from collections import deque

from .astar import ConstraintTable
from .graph import Instance, Node, Solution, path_cost
from .prioritized import add_path, plan_in_order, prioritized_restarts


def _ball(graph, center: Node, size: int) -> set[Node]:
    """The ``size`` nodes closest to ``center`` (BFS order)."""
    seen = {center}
    queue = deque([center])
    while queue and len(seen) < size:
        cur = queue.popleft()
        for nxt in graph.neighbors(cur):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return seen


def lns(
    instance: Instance,
    time_limit: float = 1.0,
    neighborhood: int = 8,
    max_iterations: int | None = None,
    seed: int = 0,
) -> Solution | None:
    """Improve a PP solution for ``time_limit`` seconds. ``None`` if no initial solution exists."""
    t0 = time.perf_counter()
    rng = random.Random(seed)
    graph, n = instance.graph, instance.num_agents
    init = prioritized_restarts(instance, tries=10, seed=seed)
    if init is None:
        return None
    heuristics = [graph.distances_from(g) for g in instance.goals]
    paths = list(init.paths)
    costs = [path_cost(p) for p in paths]
    k = min(neighborhood, n)

    def pick() -> list[int]:
        """Choose agents to remove using one of three destroy heuristics."""
        kind = rng.choice(("random", "agent", "map"))
        if kind == "random" or n <= k:
            return rng.sample(range(n), k)
        if kind == "agent":  # the agent with the biggest delay plus the agents starting nearest to it
            delays = [costs[a] - heuristics[a][instance.starts[a]] for a in range(n)]
            a0 = max(range(n), key=lambda a: (delays[a], rng.random()))
            ball = _ball(graph, instance.starts[a0], 4 * k)
            near = [a for a in range(n) if instance.starts[a] in ball and a != a0]
            rng.shuffle(near)
            return [a0, *near[: k - 1]]
        # map: agents whose paths cross a random region of the graph
        centre = rng.choice(paths[rng.randrange(n)])
        ball = _ball(graph, centre, 4 * k)
        hit = [a for a in range(n) if any(loc in ball for loc in paths[a])]
        rng.shuffle(hit)
        return hit[:k] if len(hit) >= 2 else rng.sample(range(n), k)

    iterations = accepted = 0
    while time.perf_counter() - t0 < time_limit and (max_iterations is None or iterations < max_iterations):
        iterations += 1
        removed = pick()
        gone = set(removed)
        table = ConstraintTable()
        for a in range(n):
            if a not in gone:
                add_path(table, paths[a])
        order = removed[:]
        rng.shuffle(order)
        new = plan_in_order(instance, order, heuristics, table)
        if new is None:
            continue
        if sum(path_cost(new[a]) for a in removed) <= sum(costs[a] for a in removed):
            for a in removed:
                paths[a], costs[a] = new[a], path_cost(new[a])
            accepted += 1
    return Solution(
        paths,
        iterations,
        time.perf_counter() - t0,
        {"initial_cost": init.sum_of_costs, "accepted": accepted},
    )
