"""Graph, MAPF instance and solution types.

A map is an undirected graph. Agents occupy one node per time step and, in one step,
either wait or move along one edge. Node ids are JSON scalars (``int`` or ``str``).
Optional 2-D positions (``pos``) are only used for drawing.
"""

from __future__ import annotations

import json
import random
from collections import deque
from collections.abc import Hashable, Iterable
from dataclasses import dataclass, field
from pathlib import Path

Node = Hashable
NodePath = list[Node]


class Graph:
    def __init__(
        self,
        nodes: Iterable[Node],
        edges: Iterable[tuple[Node, Node]],
        pos: dict[Node, tuple[float, float]] | None = None,
    ) -> None:
        self.nodes: tuple[Node, ...] = tuple(nodes)
        if len(set(self.nodes)) != len(self.nodes):
            raise ValueError("duplicate node ids")
        adj: dict[Node, list[Node]] = {n: [] for n in self.nodes}
        seen: set[frozenset] = set()
        edge_list: list[tuple[Node, Node]] = []
        for u, v in edges:
            if u not in adj or v not in adj:
                raise ValueError(f"edge ({u!r}, {v!r}) references an unknown node")
            if u == v:
                raise ValueError(f"self-loop on node {u!r}")
            key = frozenset((u, v))
            if key in seen:
                continue
            seen.add(key)
            adj[u].append(v)
            adj[v].append(u)
            edge_list.append((u, v))
        self._adj = {n: tuple(vs) for n, vs in adj.items()}
        self.edges: tuple[tuple[Node, Node], ...] = tuple(edge_list)
        self.pos: dict[Node, tuple[float, float]] = dict(pos or {})
        unknown = set(self.pos) - set(self.nodes)
        if unknown:
            raise ValueError(f"positions given for unknown nodes: {sorted(map(str, unknown))}")

    def __contains__(self, node: Node) -> bool:
        return node in self._adj

    def __len__(self) -> int:
        return len(self.nodes)

    def neighbors(self, node: Node) -> tuple[Node, ...]:
        """Nodes adjacent to ``node`` (not including ``node`` itself)."""
        return self._adj[node]

    def moves(self, node: Node) -> list[Node]:
        """Nodes an agent at ``node`` can occupy next step: ``node`` itself (wait) and its neighbours."""
        return [node, *self._adj[node]]

    def has_edge(self, u: Node, v: Node) -> bool:
        return u in self._adj and v in self._adj[u]

    def distances_from(self, goal: Node) -> dict[Node, int]:
        """BFS hop distance from every node that can reach ``goal``."""
        dist = {goal: 0}
        queue = deque([goal])
        while queue:
            cur = queue.popleft()
            for nxt in self._adj[cur]:
                if nxt not in dist:
                    dist[nxt] = dist[cur] + 1
                    queue.append(nxt)
        return dist

    # --- construction -------------------------------------------------------------

    @classmethod
    def from_grid(cls, lines: list[str], blocked: str = "#@T") -> Graph:
        """4-connected grid graph. Free cells get ids 0, 1, 2, ... in row-major order
        and ``pos = (col, row)``; characters in ``blocked`` are walls."""
        if not lines:
            raise ValueError("empty map")
        if any(len(line) != len(lines[0]) for line in lines):
            raise ValueError("map rows must have equal length")
        ids: dict[tuple[int, int], int] = {}
        for r, line in enumerate(lines):
            for c, ch in enumerate(line):
                if ch not in blocked:
                    ids[(r, c)] = len(ids)
        edges = []
        for (r, c), i in ids.items():
            for nb in ((r + 1, c), (r, c + 1)):
                if nb in ids:
                    edges.append((i, ids[nb]))
        pos = {i: (c, r) for (r, c), i in ids.items()}
        return cls(ids.values(), edges, pos)

    @classmethod
    def from_dict(cls, data: dict) -> Graph:
        """Build from the JSON form: ``{"nodes": [...], "edges": [[u, v], ...]}``.

        A node is either an id or ``{"id": ..., "pos": [x, y]}``.
        """
        nodes, pos = [], {}
        for item in data["nodes"]:
            if isinstance(item, dict):
                nodes.append(item["id"])
                if "pos" in item:
                    pos[item["id"]] = (item["pos"][0], item["pos"][1])
            else:
                nodes.append(item)
        return cls(nodes, [(u, v) for u, v in data["edges"]], pos)

    def to_dict(self) -> dict:
        nodes = [{"id": n, "pos": list(self.pos[n])} if n in self.pos else n for n in self.nodes]
        return {"nodes": nodes, "edges": [list(e) for e in self.edges]}

    @classmethod
    def load(cls, path: str | Path) -> Graph:
        """Read a graph JSON file, or the ``"graph"`` part of a scenario file."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data["graph"] if "graph" in data else data)

    def save(self, path: str | Path) -> None:
        Path(path).write_text(dump_json(self.to_dict()), encoding="utf-8")

    @classmethod
    def random_connected(cls, num_nodes: int, extra_edges: int = 0, seed: int | None = None) -> Graph:
        """Random connected graph: a random spanning tree plus ``extra_edges`` more edges."""
        rng = random.Random(seed)
        edges = {(rng.randrange(i), i) for i in range(1, num_nodes)}
        max_edges = num_nodes * (num_nodes - 1) // 2
        target = min(len(edges) + extra_edges, max_edges)
        while len(edges) < target:
            u, v = sorted(rng.sample(range(num_nodes), 2))
            edges.add((u, v))
        return cls(range(num_nodes), sorted(edges))


def dump_json(data, indent: int = 0, width: int = 78) -> str:
    """Readable JSON: containers that fit on one line stay inline, others get one item per line."""
    flat = json.dumps(data, ensure_ascii=False)
    if len(flat) + indent <= width or not isinstance(data, (dict, list)):
        return flat
    pad = " " * (indent + 2)
    if isinstance(data, dict):
        body = [f"{pad}{json.dumps(k)}: {dump_json(v, indent + 2, width)}" for k, v in data.items()]
        return "{\n" + ",\n".join(body) + "\n" + " " * indent + "}"
    items = [dump_json(v, indent + 2, width) for v in data]
    if all("\n" not in it and len(it) <= 24 for it in items):  # short items: pack several per line
        lines, cur = [], ""
        for it in items:
            if cur and len(pad) + len(cur) + len(it) + 2 > width:
                lines.append(cur)
                cur = ""
            cur += (", " if cur else "") + it
        lines.append(cur)
        items = lines
    body = [pad + it for it in items]
    return "[\n" + ",\n".join(body) + "\n" + " " * indent + "]"


@dataclass(frozen=True)
class Instance:
    graph: Graph
    starts: tuple[Node, ...]
    goals: tuple[Node, ...]

    def __post_init__(self) -> None:
        if len(self.starts) != len(self.goals):
            raise ValueError("starts and goals must have the same length")
        for name, nodes in (("start", self.starts), ("goal", self.goals)):
            if len(set(nodes)) != len(nodes):
                raise ValueError(f"duplicate {name} nodes")
            for n in nodes:
                if n not in self.graph:
                    raise ValueError(f"{name} {n!r} is not a node of the graph")

    @property
    def num_agents(self) -> int:
        return len(self.starts)

    @classmethod
    def from_dict(cls, data: dict) -> Instance:
        agents = data["agents"]
        return cls(
            Graph.from_dict(data["graph"]),
            tuple(a["start"] for a in agents),
            tuple(a["goal"] for a in agents),
        )

    def to_dict(self) -> dict:
        return {
            "graph": self.graph.to_dict(),
            "agents": [{"start": s, "goal": g} for s, g in zip(self.starts, self.goals, strict=True)],
        }

    @classmethod
    def load(cls, path: str | Path) -> Instance:
        """Read a scenario JSON file: ``{"graph": {...}, "agents": [{"start": .., "goal": ..}, ...]}``."""
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def save(self, path: str | Path) -> None:
        Path(path).write_text(dump_json(self.to_dict()), encoding="utf-8")

    @classmethod
    def random(cls, graph: Graph, num_agents: int, seed: int | None = None) -> Instance:
        """Random distinct starts and distinct goals, each goal reachable from its start."""
        rng = random.Random(seed)
        if num_agents > len(graph):
            raise ValueError("more agents than nodes")
        for _ in range(1000):
            starts = rng.sample(graph.nodes, num_agents)
            goals = rng.sample(graph.nodes, num_agents)
            if all(g in graph.distances_from(s) for s, g in zip(starts, goals, strict=True)):
                return cls(graph, tuple(starts), tuple(goals))
        raise RuntimeError("could not generate an instance with reachable goals")


def path_cost(path: NodePath) -> int:
    """Time step at which the agent reaches its final node for good."""
    end = len(path) - 1
    while end > 0 and path[end - 1] == path[-1]:
        end -= 1
    return end


@dataclass
class Solution:
    paths: list[NodePath]
    expanded: int = 0
    runtime: float = 0.0
    extra: dict = field(default_factory=dict)

    @property
    def sum_of_costs(self) -> int:
        return sum(path_cost(p) for p in self.paths)

    @property
    def makespan(self) -> int:
        return max(len(p) for p in self.paths) - 1
