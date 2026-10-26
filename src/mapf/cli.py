from __future__ import annotations

import argparse
import sys

from . import cbs, prioritized_planning, validate
from .grid import Grid, Instance
from .visualize import render_ascii_all, render_svg

ALGOS = {"cbs": cbs, "pp": prioritized_planning}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="mapf", description="Multi-Agent Path Finding solver")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("scenario", nargs="?", help="scenario file (map, '---', agent lines)")
    src.add_argument("--random", metavar="MAP", help="map file; generate random agents on it")
    ap.add_argument("-n", "--agents", type=int, default=5, help="number of random agents")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("-a", "--algo", choices=ALGOS, default="cbs")
    ap.add_argument("--svg", metavar="FILE", help="write animated SVG")
    ap.add_argument("--ascii", action="store_true", help="print every time step")
    args = ap.parse_args(argv)

    if args.random:
        lines = [l.strip() for l in open(args.random, encoding="utf-8") if l.strip() and l.strip() != "---"]
        lines = [l for l in lines if set(l) <= set(".#@T")]
        instance = Instance.random(Grid.from_lines(lines), args.agents, args.seed)
    else:
        instance = Instance.load(args.scenario)

    sol = ALGOS[args.algo](instance)
    if sol is None:
        print("no solution found (unsolvable or search budget exceeded)", file=sys.stderr)
        return 1
    errors = validate(instance, sol.paths)
    if errors:
        print("INVALID SOLUTION:\n" + "\n".join(errors), file=sys.stderr)
        return 2
    print(f"{args.algo}: agents={instance.num_agents} sum_of_costs={sol.sum_of_costs} "
          f"makespan={sol.makespan} expanded={sol.expanded} time={sol.runtime:.3f}s")
    if args.ascii:
        print(render_ascii_all(instance, sol.paths))
    if args.svg:
        with open(args.svg, "w", encoding="utf-8") as f:
            f.write(render_svg(instance, sol.paths))
        print(f"wrote {args.svg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
