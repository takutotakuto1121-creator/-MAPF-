from __future__ import annotations

import argparse
import sys

from . import cbs, pibt, prioritized_planning, validate
from .convert import load_graph
from .graph import Instance
from .movingai import load_scen
from .visualize import render_ascii_all, render_svg

ALGOS = {
    "cbs": lambda inst, timeout: cbs(inst, timeout=timeout),
    "pp": lambda inst, timeout: prioritized_planning(inst),
    "pibt": lambda inst, timeout: pibt(inst),
}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="mapf", description="Multi-Agent Path Finding solver")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("scenario", nargs="?", help="scenario JSON file (graph + agents)")
    src.add_argument(
        "--random", metavar="MAP", help="graph JSON (or MovingAI .map / text grid); place random agents on it"
    )
    src.add_argument("--movingai", nargs=2, metavar=("MAP", "SCEN"), help="MovingAI .map and .scen files")
    ap.add_argument("-n", "--agents", type=int, default=5, help="number of random agents")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-a", "--algo", choices=ALGOS, default="cbs")
    ap.add_argument("--timeout", type=float, default=None, help="CBS time limit [s]")
    ap.add_argument("--svg", metavar="FILE", help="write animated SVG")
    ap.add_argument("--ascii", action="store_true", help="print every time step")
    args = ap.parse_args(argv)

    if args.random:
        instance = Instance.random(load_graph(args.random), args.agents, args.seed)
    elif args.movingai:
        instance = load_scen(args.movingai[0], args.movingai[1], args.agents)
    else:
        instance = Instance.load(args.scenario)

    sol = ALGOS[args.algo](instance, args.timeout)
    if sol is None:
        print("no solution found (unsolvable or search budget exceeded)", file=sys.stderr)
        return 1
    errors = validate(instance, sol.paths)
    if errors:
        print("INVALID SOLUTION:\n" + "\n".join(errors), file=sys.stderr)
        return 2
    print(
        f"{args.algo}: agents={instance.num_agents} sum_of_costs={sol.sum_of_costs} "
        f"makespan={sol.makespan} expanded={sol.expanded} time={sol.runtime:.3f}s"
    )
    if args.ascii:
        print(render_ascii_all(instance, sol.paths))
    if args.svg:
        with open(args.svg, "w", encoding="utf-8") as f:
            f.write(render_svg(instance, sol.paths))
        print(f"wrote {args.svg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
