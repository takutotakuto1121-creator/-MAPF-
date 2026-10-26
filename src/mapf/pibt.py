"""PIBT (Okumura et al. 2022): Priority Inheritance with Backtracking.

Very fast one-step-at-a-time planner that scales to hundreds of agents. Reaching all
goals is not guaranteed on arbitrary maps (it is on e.g. biconnected graphs with free
cells), so the solver gives up after ``max_steps``.
"""

from __future__ import annotations

import random
import time

from .grid import Instance, Loc, Solution


def pibt(instance: Instance, max_steps: int = 1000, seed: int = 0) -> Solution | None:
    t0 = time.perf_counter()
    rng = random.Random(seed)
    grid = instance.grid
    n = instance.num_agents
    dist = [grid.distances_from(g) for g in instance.goals]
    if any(instance.starts[a] not in dist[a] for a in range(n)):
        return None

    cur: list[Loc] = list(instance.starts)
    paths = [[s] for s in cur]
    priority = [rng.random() for _ in range(n)]  # tie-breaker in [0, 1), grows by 1 per waiting step

    occ_now: dict[Loc, int] = {}
    occ_next: dict[Loc, int] = {}
    nxt: list[Loc | None] = [None] * n

    def plan(a: int) -> bool:
        """Try to give agent ``a`` a cell for the next step, pushing others if needed."""
        cands = grid.neighbors(cur[a])
        rng.shuffle(cands)
        cands.sort(key=lambda v: dist[a][v])
        for v in cands:
            if v in occ_next:
                continue
            u = occ_now.get(v)
            if u is not None and u != a and nxt[u] == cur[a]:
                continue  # would swap places with u
            occ_next[v], nxt[a] = a, v
            if u is not None and u != a and nxt[u] is None and not plan(u):
                del occ_next[v]
                nxt[a] = None
                continue
            return True
        return False

    for _ in range(max_steps):
        if all(cur[a] == instance.goals[a] for a in range(n)):
            return Solution(paths, runtime=time.perf_counter() - t0, extra={"steps": len(paths) - 1})

        occ_now.clear()
        occ_now.update({loc: a for a, loc in enumerate(cur)})
        occ_next.clear()
        nxt[:] = [None] * n

        for a in sorted(range(n), key=lambda a: -priority[a]):
            if nxt[a] is None and not plan(a):
                occ_next[cur[a]], nxt[a] = a, cur[a]  # nobody can have claimed it: stay

        for a in range(n):
            cur[a] = nxt[a]  # type: ignore[assignment]
            paths[a].append(cur[a])
            priority[a] = priority[a] % 1 if cur[a] == instance.goals[a] else priority[a] + 1
    return None
