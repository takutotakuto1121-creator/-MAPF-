"""Independence Detection (Standley 2010) on top of CBS.

Every agent starts in its own group and is planned alone. While paths of different groups
conflict, those groups are merged and re-planned together with CBS. Stays optimal for sum of
costs, and is much faster than plain CBS when agents interact only locally.
"""

from __future__ import annotations

import time

from .cbs import cbs
from .graph import Instance, Solution
from .validate import find_conflict


def id_cbs(instance: Instance, max_nodes: int = 100_000, timeout: float | None = None) -> Solution | None:
    t0 = time.perf_counter()
    paths: list = [None] * instance.num_agents
    group_of = list(range(instance.num_agents))
    groups: dict[int, list[int]] = {a: [a] for a in range(instance.num_agents)}
    expanded = 0

    def solve(members: list[int]) -> bool:
        nonlocal expanded
        sub = Instance(
            instance.graph,
            tuple(instance.starts[a] for a in members),
            tuple(instance.goals[a] for a in members),
        )
        remaining = None if timeout is None else timeout - (time.perf_counter() - t0)
        if remaining is not None and remaining <= 0:
            return False
        sol = cbs(sub, max_nodes=max_nodes, timeout=remaining)
        if sol is None:
            return False
        expanded += sol.expanded
        for a, p in zip(members, sol.paths, strict=True):
            paths[a] = p
        return True

    for a in range(instance.num_agents):
        if not solve([a]):
            return None
    while True:
        c = find_conflict(paths)
        if c is None:
            largest = max(len(m) for m in groups.values())
            return Solution(paths, expanded, time.perf_counter() - t0, {"largest_group": largest})
        g1, g2 = group_of[c.a1], group_of[c.a2]
        merged = sorted(groups.pop(g1) + groups.pop(g2))
        groups[g1] = merged
        for a in merged:
            group_of[a] = g1
        if not solve(merged):
            return None
