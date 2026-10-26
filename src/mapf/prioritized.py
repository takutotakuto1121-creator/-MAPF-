"""Prioritized Planning (Silver 2005): fast, but incomplete and suboptimal."""

from __future__ import annotations

import time

from .astar import ConstraintTable, space_time_astar
from .graph import Instance, Solution


def prioritized_planning(instance: Instance, order: list[int] | None = None) -> Solution | None:
    """Plan agents one by one; later agents treat earlier paths as moving obstacles.

    ``order`` is the priority order (default: agents with longer shortest paths first).
    Returns ``None`` if some agent cannot be planned for the given order.
    """
    t0 = time.perf_counter()
    graph = instance.graph
    heuristics = [graph.distances_from(g) for g in instance.goals]
    if order is None:
        order = sorted(
            range(instance.num_agents),
            key=lambda a: -heuristics[a].get(instance.starts[a], 0),
        )
    paths: list = [None] * instance.num_agents
    table = ConstraintTable()  # shared: every planned path becomes an obstacle for the rest
    for a in order:
        path = space_time_astar(graph, instance.starts[a], instance.goals[a], heuristics[a], table)
        if path is None:
            return None
        paths[a] = path
        for t, loc in enumerate(path):
            table.vertex.add((loc, t))
            if t > 0:
                table.edge.add((loc, path[t - 1], t))  # block the swap move
        table.parked[path[-1]] = len(path) - 1
    return Solution(paths, runtime=time.perf_counter() - t0, extra={"order": order})
