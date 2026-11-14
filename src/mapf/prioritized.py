"""Prioritized Planning (Silver 2005): fast, but incomplete and suboptimal."""

from __future__ import annotations

import random
import time

from .astar import ConstraintTable, space_time_astar
from .graph import Instance, Node, NodePath, Solution


def add_path(table: ConstraintTable, path: NodePath) -> None:
    """Make ``path`` an obstacle: its vertices, swap moves, and the node it parks on afterwards."""
    for t, loc in enumerate(path):
        table.vertex.add((loc, t))
        if t > 0:
            table.edge.add((loc, path[t - 1], t))  # block the swap move
    table.parked[path[-1]] = len(path) - 1


def plan_in_order(
    instance: Instance,
    order: list[int],
    heuristics: list[dict[Node, int]],
    table: ConstraintTable,
) -> dict[int, NodePath] | None:
    """Plan the agents in ``order`` one by one against ``table`` (which is extended in place)."""
    paths: dict[int, NodePath] = {}
    for a in order:
        path = space_time_astar(instance.graph, instance.starts[a], instance.goals[a], heuristics[a], table)
        if path is None:
            return None
        paths[a] = path
        add_path(table, path)
    return paths


def prioritized_planning(instance: Instance, order: list[int] | None = None) -> Solution | None:
    """Plan agents one by one; later agents treat earlier paths as moving obstacles.

    ``order`` is the priority order (default: agents with longer shortest paths first).
    Returns ``None`` if some agent cannot be planned for the given order.
    """
    t0 = time.perf_counter()
    heuristics = [instance.graph.distances_from(g) for g in instance.goals]
    if order is None:
        order = sorted(range(instance.num_agents), key=lambda a: -heuristics[a].get(instance.starts[a], 0))
    planned = plan_in_order(instance, order, heuristics, ConstraintTable())
    if planned is None:
        return None
    paths = [planned[a] for a in range(instance.num_agents)]
    return Solution(paths, runtime=time.perf_counter() - t0, extra={"order": order})


def prioritized_restarts(
    instance: Instance, tries: int = 30, seed: int = 0, timeout: float | None = None
) -> Solution | None:
    """Prioritized Planning with random restarts: keep the cheapest solution over several priority orders.

    The first try uses the default order (longest shortest-path first), the rest are random.
    """
    t0 = time.perf_counter()
    rng = random.Random(seed)
    best: Solution | None = None
    for k in range(tries):
        if timeout is not None and time.perf_counter() - t0 > timeout:
            break
        order = None
        if k > 0:
            order = list(range(instance.num_agents))
            rng.shuffle(order)
        sol = prioritized_planning(instance, order)
        if sol is not None and (best is None or sol.sum_of_costs < best.sum_of_costs):
            best = sol
            best.extra["try"] = k
    if best is not None:
        best.runtime = time.perf_counter() - t0
    return best
