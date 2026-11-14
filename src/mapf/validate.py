"""Conflict detection and solution validation.

After its path ends an agent stays at its final node (and keeps blocking it).
"""

from __future__ import annotations

from dataclasses import dataclass

from .graph import Instance, Node, NodePath


@dataclass(frozen=True)
class Conflict:
    a1: int
    a2: int
    loc: Node  # vertex conflict: shared node; edge conflict: a1's destination
    t: int  # time step at which the conflict occurs (arrival time for edge conflicts)
    prev: Node | None = None  # edge conflict only: a1's origin (a2 moves loc -> prev)


def position(path: NodePath, t: int) -> Node:
    return path[min(t, len(path) - 1)]


def _conflicts_at(paths: list[NodePath], t: int):
    """Yield every conflict at time ``t`` as ``Conflict`` objects (vertex conflicts first)."""
    now = [position(p, t) for p in paths]
    seen: dict[Node, int] = {}
    for a, loc in enumerate(now):
        if loc in seen:
            yield Conflict(seen[loc], a, loc, t)
        else:
            seen[loc] = a
    if t > 0:
        before = [position(p, t - 1) for p in paths]
        moved = {(before[a], now[a]): a for a in range(len(paths)) if before[a] != now[a]}
        for a in range(len(paths)):
            if before[a] != now[a]:
                b = moved.get((now[a], before[a]))
                if b is not None and a < b:
                    yield Conflict(a, b, now[a], t, prev=before[a])


def find_conflict(paths: list[NodePath]) -> Conflict | None:
    """Return the earliest conflict among ``paths`` or ``None``."""
    for t in range(max(len(p) for p in paths)):
        for c in _conflicts_at(paths, t):
            return c
    return None


def count_conflicts(paths: list[NodePath]) -> int:
    """Number of conflicting (agent pair, time) occurrences; 0 iff the paths are conflict-free."""
    total = 0
    for t in range(max(len(p) for p in paths)):
        total += sum(1 for _ in _conflicts_at(paths, t))
    return total


def validate(instance: Instance, paths: list[NodePath]) -> list[str]:
    """Return a list of human-readable problems; empty means the solution is valid."""
    errors: list[str] = []
    if len(paths) != instance.num_agents:
        return [f"expected {instance.num_agents} paths, got {len(paths)}"]
    for a, p in enumerate(paths):
        if not p:
            errors.append(f"agent {a}: empty path")
            continue
        if p[0] != instance.starts[a]:
            errors.append(f"agent {a}: path starts at {p[0]}, expected {instance.starts[a]}")
        if p[-1] != instance.goals[a]:
            errors.append(f"agent {a}: path ends at {p[-1]}, expected {instance.goals[a]}")
        for t, loc in enumerate(p):
            if loc not in instance.graph:
                errors.append(f"agent {a}: t={t} at unknown node {loc!r}")
            elif t > 0 and loc != p[t - 1] and not instance.graph.has_edge(p[t - 1], loc):
                errors.append(f"agent {a}: t={t} illegal move {p[t - 1]!r} -> {loc!r} (no such edge)")
    if not errors:
        c = find_conflict(paths)
        if c is not None:
            kind = "edge" if c.prev is not None else "vertex"
            errors.append(f"{kind} conflict between agents {c.a1} and {c.a2} at {c.loc}, t={c.t}")
    return errors
