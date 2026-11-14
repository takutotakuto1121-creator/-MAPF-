"""Registry of all solvers behind one interface: ``solve(name, instance, timeout, seed, **options)``."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from .cbs import cbs
from .ecbs import ecbs
from .graph import Instance, Solution
from .idcbs import id_cbs
from .joint_astar import joint_astar
from .lacam import lacam
from .lns import lns
from .pibt import pibt
from .prioritized import prioritized_planning, prioritized_restarts
from .whca import whca

Solver = Callable[..., Solution | None]


@dataclass(frozen=True)
class Algorithm:
    name: str
    fn: Solver  # fn(instance, timeout, seed, **options)
    summary: str
    optimal: bool  # sum-of-costs optimal when it succeeds
    complete: bool  # finds a solution whenever one exists (given enough time)


ALGORITHMS: dict[str, Algorithm] = {
    a.name: a
    for a in [
        Algorithm(
            "astar",
            lambda i, timeout, seed, **o: joint_astar(i, timeout=timeout, **o),
            "joint-space A*; optimal but only for a handful of agents",
            True,
            True,
        ),
        Algorithm(
            "cbs",
            lambda i, timeout, seed, **o: cbs(i, timeout=timeout, **o),
            "Conflict-Based Search; optimal",
            True,
            True,
        ),
        Algorithm(
            "id_cbs",
            lambda i, timeout, seed, **o: id_cbs(i, timeout=timeout, **o),
            "Independence Detection + CBS; optimal, faster when agents interact locally",
            True,
            True,
        ),
        Algorithm(
            "ecbs",
            lambda i, timeout, seed, **o: ecbs(i, timeout=timeout, **o),
            "Enhanced CBS; sum of costs <= w * optimal (option w, default 1.5)",
            False,
            True,
        ),
        Algorithm(
            "pp",
            lambda i, timeout, seed, **o: prioritized_planning(i, **o),
            "Prioritized Planning; fast, incomplete",
            False,
            False,
        ),
        Algorithm(
            "pp_restarts",
            lambda i, timeout, seed, **o: prioritized_restarts(i, timeout=timeout, seed=seed, **o),
            "Prioritized Planning with random restarts; keeps the best of several orders",
            False,
            False,
        ),
        Algorithm(
            "whca",
            lambda i, timeout, seed, **o: whca(i, timeout=timeout, seed=seed, **o),
            "Windowed Hierarchical Cooperative A*; windowed prioritized planning, incomplete",
            False,
            False,
        ),
        Algorithm(
            "pibt",
            lambda i, timeout, seed, **o: pibt(i, seed=seed, **o),
            "Priority Inheritance with Backtracking; very fast, incomplete",
            False,
            False,
        ),
        Algorithm(
            "lacam",
            lambda i, timeout, seed, **o: lacam(i, timeout=timeout, seed=seed, **o),
            "LaCAM; complete and scalable, solutions are not optimal",
            False,
            True,
        ),
        Algorithm(
            "lns",
            lambda i, timeout, seed, **o: lns(
                i, seed=seed, **({"time_limit": min(timeout, 1.0)} if timeout else {}) | o
            ),
            "Large Neighbourhood Search improving a PP solution (option time_limit, default 1 s)",
            False,
            False,
        ),
    ]
}


def solve(
    name: str, instance: Instance, timeout: float | None = None, seed: int = 0, **options
) -> Solution | None:
    """Run the algorithm ``name``; ``None`` if it found no solution within its budget."""
    try:
        algo = ALGORITHMS[name]
    except KeyError:
        raise ValueError(f"unknown algorithm {name!r}; choose from {', '.join(ALGORITHMS)}") from None
    return algo.fn(instance, timeout, seed, **options)
