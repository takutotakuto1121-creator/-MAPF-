"""Conflict-Based Search (Sharon et al. 2015): complete and optimal for sum of costs."""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

from .astar import ConstraintTable, space_time_astar
from .graph import Instance, Node, NodePath, Solution, path_cost
from .validate import find_conflict


@dataclass(frozen=True)
class Constraint:
    agent: int
    loc: Node
    t: int
    prev: Node | None = None  # set => edge constraint (prev -> loc arriving at t)


@dataclass(order=True)
class _Node:
    cost: int
    seq: int
    constraints: tuple[Constraint, ...] = field(compare=False)
    paths: list[NodePath] = field(compare=False)


def _table_for(agent: int, constraints: tuple[Constraint, ...]) -> ConstraintTable:
    table = ConstraintTable()
    for c in constraints:
        if c.agent != agent:
            continue
        if c.prev is None:
            table.vertex.add((c.loc, c.t))
        else:
            table.edge.add((c.prev, c.loc, c.t))
    return table


def cbs(instance: Instance, max_nodes: int = 100_000, timeout: float | None = None) -> Solution | None:
    """Return a sum-of-costs-optimal solution, or ``None`` if none was found.

    ``None`` means unsolvable *or* the node/time budget ran out.
    """
    t0 = time.perf_counter()
    graph = instance.graph
    heuristics = [graph.distances_from(g) for g in instance.goals]

    root_paths = []
    for a in range(instance.num_agents):
        p = space_time_astar(graph, instance.starts[a], instance.goals[a], heuristics[a])
        if p is None:
            return None
        root_paths.append(p)

    seq = 0
    root = _Node(sum(map(path_cost, root_paths)), seq, (), root_paths)
    open_heap = [root]
    expanded = 0
    while open_heap:
        if expanded >= max_nodes or (timeout is not None and time.perf_counter() - t0 > timeout):
            return None
        node = heapq.heappop(open_heap)
        expanded += 1
        conflict = find_conflict(node.paths)
        if conflict is None:
            return Solution(node.paths, expanded=expanded, runtime=time.perf_counter() - t0)

        if conflict.prev is None:
            new = [
                Constraint(conflict.a1, conflict.loc, conflict.t),
                Constraint(conflict.a2, conflict.loc, conflict.t),
            ]
        else:  # edge swap: a1 goes prev->loc, a2 goes loc->prev, both arriving at t
            new = [
                Constraint(conflict.a1, conflict.loc, conflict.t, conflict.prev),
                Constraint(conflict.a2, conflict.prev, conflict.t, conflict.loc),
            ]
        for c in new:
            constraints = node.constraints + (c,)
            path = space_time_astar(
                graph,
                instance.starts[c.agent],
                instance.goals[c.agent],
                heuristics[c.agent],
                _table_for(c.agent, constraints),
            )
            if path is None:
                continue
            paths = list(node.paths)
            paths[c.agent] = path
            seq += 1
            heapq.heappush(open_heap, _Node(sum(map(path_cost, paths)), seq, constraints, paths))
    return None
