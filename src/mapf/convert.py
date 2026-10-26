"""Convert grid maps to graph JSON: ``python -m mapf.convert IN.(map|txt) OUT.json``.

``.map`` is the MovingAI format; anything else is plain text with ``.`` free and ``#`` wall.
"""

from __future__ import annotations

import argparse

from .graph import Graph
from .movingai import load_map


def load_graph(path: str) -> Graph:
    """Load a graph from JSON (graph or scenario file), MovingAI ``.map`` or plain text grid."""
    if path.endswith(".json"):
        return Graph.load(path)
    if path.endswith(".map"):
        return load_map(path)
    with open(path, encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.startswith("//")]
    return Graph.from_grid(lines)


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input")
    ap.add_argument("output")
    args = ap.parse_args(argv)
    graph = load_graph(args.input)
    graph.save(args.output)
    print(f"wrote {args.output}: {len(graph)} nodes, {len(graph.edges)} edges")


if __name__ == "__main__":
    main()
