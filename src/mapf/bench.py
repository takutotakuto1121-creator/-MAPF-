"""Benchmark the solvers on random instances: ``python -m mapf.bench``."""

from __future__ import annotations

import argparse
import statistics

from .cbs import cbs
from .convert import load_graph
from .graph import Instance
from .pibt import pibt
from .prioritized import prioritized_planning
from .validate import validate

SOLVERS = {
    "cbs": lambda inst, timeout: cbs(inst, timeout=timeout),
    "pp": lambda inst, timeout: prioritized_planning(inst),
    "pibt": lambda inst, timeout: pibt(inst),
}


def run(map_path: str, agent_counts: list[int], trials: int, timeout: float, algos: list[str]) -> str:
    """Markdown table. Cost/makespan are averaged over instances that *every* algorithm solved."""
    graph = load_graph(map_path)
    rows = [
        "| agents | algo | solved | avg SoC | avg makespan | avg time [s] |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    for n in agent_counts:
        instances = [Instance.random(graph, n, seed) for seed in range(trials)]
        results: dict[str, list] = {}
        for algo in algos:
            sols = []
            for inst in instances:
                sol = SOLVERS[algo](inst, timeout)
                if sol is not None:
                    assert not validate(inst, sol.paths), f"{algo} produced an invalid solution"
                sols.append(sol)
            results[algo] = sols
        common = [i for i in range(trials) if all(results[a][i] is not None for a in algos)]
        for algo in algos:
            solved = sum(s is not None for s in results[algo])
            if common:
                picked = [results[algo][i] for i in common]
                soc = statistics.mean(s.sum_of_costs for s in picked)
                mk = statistics.mean(s.makespan for s in picked)
                tm = statistics.mean(s.runtime for s in picked)
                rows.append(f"| {n} | {algo} | {solved}/{trials} | {soc:.1f} | {mk:.1f} | {tm:.4f} |")
            else:
                rows.append(f"| {n} | {algo} | {solved}/{trials} | - | - | - |")
    return "\n".join(rows)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default="maps/room.json")
    ap.add_argument("-n", "--agents", type=int, nargs="+", default=[4, 8, 12])
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=2.0, help="per-instance CBS timeout [s]")
    ap.add_argument("--algos", nargs="+", choices=SOLVERS, default=list(SOLVERS))
    args = ap.parse_args(argv)
    print(f"map: {args.map}, trials per setting: {args.trials}")
    print("(SoC / makespan / time are averaged over instances solved by all algorithms)\n")
    print(run(args.map, args.agents, args.trials, args.timeout, args.algos))


if __name__ == "__main__":
    main()
