"""LaCAM (Okumura 2023): lazy constraints addition search. Complete and very scalable.

A depth-first search over joint configurations. Each configuration is expanded lazily: its
successor configurations are produced one at a time by PIBT under a growing set of
"agent i must go to node v" constraints, so the branching factor is never materialised.
"""

from __future__ import annotations

import random
import time
from collections import deque
from dataclasses import dataclass, field

from .graph import Instance, Node, Solution


@dataclass
class _Node:
    config: tuple
    parent: _Node | None
    priorities: list[float]
    order: list[int]
    tree: deque = field(default_factory=lambda: deque([((), ())]))  # pending (agents, nodes) constraints


def lacam(
    instance: Instance, max_nodes: int = 1_000_000, timeout: float | None = None, seed: int = 0
) -> Solution | None:
    """Find *a* solution (not an optimal one). ``None`` means unsolvable or budget exhausted."""
    t0 = time.perf_counter()
    rng = random.Random(seed)
    graph, n = instance.graph, instance.num_agents
    goals = tuple(instance.goals)
    dist = [graph.distances_from(g) for g in goals]
    if any(instance.starts[a] not in dist[a] for a in range(n)):
        return None
    top = max((dist[a][instance.starts[a]] for a in range(n)), default=0) + 1
    init_prio = [dist[a][instance.starts[a]] / top + rng.random() * 1e-3 for a in range(n)]

    def sorted_order(prio: list[float]) -> list[int]:
        return sorted(range(n), key=lambda a: -prio[a])

    def generate(node: _Node, who: tuple, where: tuple) -> tuple | None:
        """One successor configuration of ``node`` honouring the constraints, or ``None``."""
        cur = node.config
        occ_now = {loc: a for a, loc in enumerate(cur)}
        occ_next: dict[Node, int] = {}
        nxt: list[Node | None] = [None] * n

        def plan(a: int) -> bool:
            cands = graph.moves(cur[a])
            rng.shuffle(cands)
            cands.sort(key=lambda v: dist[a][v])
            for v in cands:
                if v in occ_next:
                    continue
                u = occ_now.get(v)
                if u is not None and u != a and nxt[u] == cur[a]:
                    continue  # swap
                occ_next[v], nxt[a] = a, v
                if u is not None and u != a and nxt[u] is None and not plan(u):
                    del occ_next[v]
                    nxt[a] = None
                    continue
                return True
            return False

        for a, v in zip(who, where, strict=True):
            u = occ_now.get(v)
            if v in occ_next or (u is not None and u != a and nxt[u] == cur[a]):
                return None
            occ_next[v], nxt[a] = a, v
        for a, v in zip(who, where, strict=True):
            u = occ_now.get(v)
            if u is not None and u != a and nxt[u] is None and not plan(u):
                return None
        for a in node.order:
            if nxt[a] is None and not plan(a):
                if cur[a] in occ_next:
                    return None
                occ_next[cur[a]], nxt[a] = a, cur[a]
        return tuple(nxt)  # type: ignore[arg-type]

    start = tuple(instance.starts)
    root = _Node(start, None, init_prio, sorted_order(init_prio))
    explored = {start: root}
    stack = [root]
    iterations = 0
    while stack:
        iterations += 1
        if iterations > max_nodes or (timeout is not None and time.perf_counter() - t0 > timeout):
            return None
        node = stack[-1]
        if node.config == goals:
            configs = []
            cur: _Node | None = node
            while cur is not None:
                configs.append(cur.config)
                cur = cur.parent
            configs.reverse()
            paths = [[c[a] for c in configs] for a in range(n)]
            return Solution(paths, iterations, time.perf_counter() - t0, {"configs": len(explored)})
        if not node.tree:
            stack.pop()
            continue
        who, where = node.tree.popleft()
        if len(who) < n:  # grow the constraint tree by one more agent
            a = node.order[len(who)]
            moves = graph.moves(node.config[a])
            rng.shuffle(moves)
            for v in moves:
                node.tree.append((who + (a,), where + (v,)))
        config = generate(node, who, where)
        if config is None or config == node.config:
            continue
        if config in explored:
            stack.append(explored[config])
            continue
        prio = [p % 1 if config[a] == goals[a] else p + 1 for a, p in enumerate(node.priorities)]
        child = _Node(config, node, prio, sorted_order(prio))
        explored[config] = child
        stack.append(child)
    return None
