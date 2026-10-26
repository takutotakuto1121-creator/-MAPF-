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


def find_conflict(paths: list[NodePath]) -> Conflict | None:
    """Return the earliest conflict among ``paths`` or ``None``."""
    horizon = max(len(p) for p in paths)
    for t in range(horizon):
        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                if position(paths[i], t) == position(paths[j], t):
                    return Conflict(i, j, position(paths[i], t), t)
                if t > 0:
                    pi0, pi1 = position(paths[i], t - 1), position(paths[i], t)
                    pj0, pj1 = position(paths[j], t - 1), position(paths[j], t)
                    if pi0 == pj1 and pi1 == pj0 and pi0 != pi1:
                        return Conflict(i, j, pi1, t, prev=pi0)
    return None


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
