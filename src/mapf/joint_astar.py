"""A* in the joint configuration space: optimal for sum of costs, but only for tiny instances.

Useful as a ground-truth oracle for the other solvers (about <= 5 agents on small graphs).
"""

from __future__ import annotations

import heapq
import itertools
import time

from .graph import Instance, Solution


def joint_astar(
    instance: Instance, max_expansions: int = 2_000_000, timeout: float | None = None
) -> Solution | None:
    """Sum-of-costs-optimal solution by exhaustive joint search, or ``None`` if none was found.

    The cost of an agent is the time it reaches its goal for the last time. To charge that
    exactly, a state also stores per agent how many steps it has waited on its goal since
    arriving; that debt is paid if the agent later leaves the goal.
    """
    t0 = time.perf_counter()
    graph, n = instance.graph, instance.num_agents
    goals = instance.goals
    dist = [graph.distances_from(g) for g in goals]
    if any(instance.starts[a] not in dist[a] for a in range(n)):
        return None

    def h(pos: tuple) -> int:
        return sum(dist[a][pos[a]] for a in range(n))

    def successors(pos: tuple, wait: tuple):
        """Yield (new_pos, new_wait, step_cost) for every conflict-free joint move."""

        def rec(a: int, chosen: list, cost: int, new_wait: list):
            if a == n:
                yield tuple(chosen), tuple(new_wait), cost
                return
            for v in graph.moves(pos[a]):
                if v in chosen:
                    continue  # vertex conflict
                if any(chosen[b] == pos[a] and pos[b] == v for b in range(a)):
                    continue  # swap with an earlier agent
                at_goal = pos[a] == goals[a]
                if at_goal and v == pos[a]:
                    step, w = 0, wait[a] + 1  # waiting on the goal is free until the agent leaves it
                elif at_goal:
                    step, w = wait[a] + 1, 0  # leaving: pay for the steps waited plus this one
                else:
                    step, w = 1, 0
                chosen.append(v)
                new_wait.append(w)
                yield from rec(a + 1, chosen, cost + step, new_wait)
                chosen.pop()
                new_wait.pop()

        yield from rec(0, [], 0, [])

    start = (tuple(instance.starts), (0,) * n)
    goal_pos = tuple(goals)
    counter = itertools.count()
    heap = [(h(start[0]), 0, next(counter), start)]
    best_g = {start: 0}
    parent: dict = {start: None}
    expanded = 0
    while heap:
        _, g, _, state = heapq.heappop(heap)
        if best_g.get(state, g) < g:
            continue
        if state[0] == goal_pos:
            states = []
            while state is not None:
                states.append(state[0])
                state = parent[state]
            states.reverse()
            paths = [[s[a] for s in states] for a in range(n)]
            return Solution(paths, expanded=expanded, runtime=time.perf_counter() - t0)
        expanded += 1
        if expanded > max_expansions or (timeout is not None and time.perf_counter() - t0 > timeout):
            return None
        for new_pos, new_wait, cost in successors(*state):
            nxt = (new_pos, new_wait)
            ng = g + cost
            if ng < best_g.get(nxt, float("inf")):
                best_g[nxt] = ng
                parent[nxt] = state
                heapq.heappush(heap, (ng + h(new_pos), ng, next(counter), nxt))
    return None
