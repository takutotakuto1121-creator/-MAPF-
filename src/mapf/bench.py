"""Benchmark the solvers on random instances: ``python -m mapf.bench``."""

from __future__ import annotations

import argparse
import statistics

from .algorithms import ALGORITHMS, solve
from .convert import load_graph
from .graph import Instance
from .validate import validate

DEFAULT_ALGOS = [a for a in ALGORITHMS if a != "astar"]  # joint A* only suits a handful of agents


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
                sol = solve(algo, inst, timeout)
                if sol is not None:
                    assert not validate(inst, sol.paths), f"{algo} produced an invalid solution"
                sols.append(sol)
            results[algo] = sols
        common = [i for i in range(trials) if all(results[a][i] is not None for a in algos)]
        for algo in algos:
            solved = sum(s is not None for s in results[algo])
            idx, mark = common, ""
            if not common:  # nothing solved by everyone: fall back to each algorithm's own instances
                idx, mark = [i for i in range(trials) if results[algo][i] is not None], "*"
            if idx:
                picked = [results[algo][i] for i in idx]
                soc = statistics.mean(s.sum_of_costs for s in picked)
                mk = statistics.mean(s.makespan for s in picked)
                tm = statistics.mean(s.runtime for s in picked)
                cells = f"{soc:.1f}{mark} | {mk:.1f}{mark} | {tm:.4f}{mark}"
                rows.append(f"| {n} | {algo} | {solved}/{trials} | {cells} |")
            else:
                rows.append(f"| {n} | {algo} | {solved}/{trials} | - | - | - |")
    return "\n".join(rows)


def run_scenario(path: str, timeout: float, algos: list[str]) -> str:
    """One scenario file, every algorithm: a Markdown table."""
    inst = Instance.load(path)
    rows = ["| algo | sum of costs | makespan | time [s] |", "|---|---:|---:|---:|"]
    for algo in algos:
        sol = solve(algo, inst, timeout)
        if sol is None:
            rows.append(f"| {algo} | no solution within {timeout:g} s | - | - |")
            continue
        assert not validate(inst, sol.paths), f"{algo} produced an invalid solution"
        rows.append(f"| {algo} | {sol.sum_of_costs} | {sol.makespan} | {sol.runtime:.3f} |")
    return f"{path}: {inst.num_agents} agents, {len(inst.graph)} nodes\n\n" + "\n".join(rows)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default="maps/room.json")
    ap.add_argument("-n", "--agents", type=int, nargs="+", default=[4, 8, 12])
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--timeout", type=float, default=2.0, help="per-instance CBS timeout [s]")
    ap.add_argument("--algos", nargs="+", choices=ALGORITHMS, default=DEFAULT_ALGOS)
    ap.add_argument(
        "--scenario", help="run every algorithm on this scenario file instead of random instances"
    )
    args = ap.parse_args(argv)
    if args.scenario:
        print(run_scenario(args.scenario, args.timeout, args.algos))
        return
    print(f"map: {args.map}, trials per setting: {args.trials}")
    print("(SoC / makespan / time are averaged over instances solved by all algorithms;")
    print(" * = nobody solved them all, so over the instances that algorithm solved)\n")
    print(run(args.map, args.agents, args.trials, args.timeout, args.algos))


if __name__ == "__main__":
    main()
