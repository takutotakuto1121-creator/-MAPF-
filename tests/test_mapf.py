import json

import pytest

from mapf import (
    Graph,
    Instance,
    cbs,
    find_conflict,
    load_map,
    load_scen,
    pibt,
    prioritized_planning,
    validate,
)
from mapf.astar import ConstraintTable, space_time_astar
from mapf.convert import load_graph


def grid(n):
    return Graph.from_grid(["." * n] * n)


def line(n):
    return Graph(range(n), [(i, i + 1) for i in range(n - 1)])


def test_astar_respects_vertex_constraint():
    g = line(3)
    h = g.distances_from(2)
    assert space_time_astar(g, 0, 2, h) == [0, 1, 2]
    table = ConstraintTable(vertex={(1, 1)})
    assert space_time_astar(g, 0, 2, h, table) == [0, 0, 1, 2]


def test_astar_waits_for_goal_constraint_to_expire():
    g = line(3)
    table = ConstraintTable(vertex={(1, 3)})
    path = space_time_astar(g, 0, 1, g.distances_from(1), table)
    assert len(path) - 1 >= 4 and path[-1] == 1


def test_swap_in_open_grid_is_valid_and_optimal():
    g = grid(3)  # ids are row-major: 0..2 is the top row
    inst = Instance(g, (0, 2), (2, 0))
    sol = cbs(inst)
    assert validate(inst, sol.paths) == []
    assert sol.sum_of_costs == 6  # one detours around the other: 2 + 4


def test_cbs_optimal_vs_prioritized():
    inst = Instance.load("maps/crossing.json")
    c, p = cbs(inst), prioritized_planning(inst)
    assert validate(inst, c.paths) == []
    assert validate(inst, p.paths) == []
    assert c.sum_of_costs <= p.sum_of_costs


def test_corridor_swap_uses_bay():
    inst = Instance.load("maps/corridor.json")
    assert validate(inst, cbs(inst).paths) == []


def test_unsolvable_head_on_corridor_returns_none():
    inst = Instance(line(5), (0, 4), (4, 0))
    assert cbs(inst, max_nodes=200) is None


@pytest.mark.parametrize("path", ["maps/ring.json", "maps/petersen.json"])
@pytest.mark.parametrize("algo", [cbs, prioritized_planning, pibt])
def test_non_grid_graphs(path, algo):
    inst = Instance.load(path)
    sol = algo(inst)
    assert sol is not None
    assert validate(inst, sol.paths) == []


def test_string_node_ids():
    inst = Instance.load("maps/ring.json")
    sol = cbs(inst)
    assert all(isinstance(n, str) for p in sol.paths for n in p)


def test_validate_detects_problems():
    inst = Instance(grid(3), (0, 2), (2, 0))
    through = [[0, 1, 2], [2, 1, 0]]  # both on node 1 at t=1
    assert any("conflict" in e for e in validate(inst, through))
    teleport = [[0, 8, 2], [2, 2, 0]]
    assert validate(inst, teleport)
    unknown = [[0, 99, 2], [2, 2, 0]]
    assert any("unknown node" in e for e in validate(inst, unknown))
    non_edge = [[0, 2], [2, 0]]  # 0-2 is not an edge in a 3x3 grid
    assert any("no such edge" in e for e in validate(inst, non_edge))
    swap = [[0, 1], [1, 0]]
    assert find_conflict(swap).prev is not None


def test_goal_blocking_after_arrival_is_conflict():
    # agent 1 passes through agent 0's goal after agent 0 has parked there
    paths = [[0, 1], [3, 3, 1]]
    assert find_conflict(paths) is not None


def test_graph_validation():
    with pytest.raises(ValueError, match="unknown node"):
        Graph([0, 1], [(0, 2)])
    with pytest.raises(ValueError, match="duplicate"):
        Graph([0, 0], [])
    with pytest.raises(ValueError, match="self-loop"):
        Graph([0], [(0, 0)])
    with pytest.raises(ValueError, match="not a node"):
        Instance(line(3), (0,), (7,))
    with pytest.raises(ValueError, match="duplicate start"):
        Instance(line(3), (0, 0), (1, 2))


def test_graph_json_roundtrip(tmp_path):
    g = Graph(["a", "b", "c"], [("a", "b"), ("b", "c")], {"a": (0, 0), "b": (1, 0)})
    g2 = Graph.from_dict(json.loads(json.dumps(g.to_dict())))
    assert g2.nodes == g.nodes and g2.edges == g.edges and g2.pos == g.pos
    inst = Instance(g, ("a",), ("c",))
    inst.save(tmp_path / "s.json")
    back = Instance.load(tmp_path / "s.json")
    assert (back.starts, back.goals, back.graph.edges) == (inst.starts, inst.goals, g.edges)
    assert Graph.load(tmp_path / "s.json").nodes == g.nodes  # graph out of a scenario file


def test_json_format_documented_in_readme_parses():
    data = {
        "graph": {"nodes": [0, {"id": 1, "pos": [1, 0]}, 2], "edges": [[0, 1], [1, 2]]},
        "agents": [{"start": 0, "goal": 2}],
    }
    inst = Instance.from_dict(data)
    assert inst.graph.pos == {1: (1, 0)}
    assert cbs(inst).paths == [[0, 1, 2]]


def test_random_connected_graph_is_connected():
    for seed in range(5):
        g = Graph.random_connected(30, extra_edges=10, seed=seed)
        assert len(g.distances_from(0)) == 30
        inst = Instance.random(g, 4, seed)
        sol = cbs(inst, max_nodes=20_000)
        assert sol is not None and validate(inst, sol.paths) == []


def test_pibt_solves_open_grid_and_is_valid():
    g = grid(6)
    for seed in range(10):
        inst = Instance.random(g, 10, seed)
        sol = pibt(inst, seed=seed)
        assert sol is not None
        assert validate(inst, sol.paths) == []


def test_movingai_loader():
    graph = load_map("tests/data/tiny.map")
    assert len(graph) == 30 - 5  # 3 '@' and 2 'T' blocked
    inst = load_scen("tests/data/tiny.map", "tests/data/tiny.scen", 3)
    assert graph.pos[inst.starts[0]] == (0, 0) and graph.pos[inst.goals[0]] == (5, 4)  # (x, y)
    assert validate(inst, cbs(inst).paths) == []


def test_movingai_skips_blocked_agents():
    # the 4th scenario line starts on an obstacle, so only 3 agents are usable
    with pytest.raises(ValueError):
        load_scen("tests/data/tiny.map", "tests/data/tiny.scen", 4)


def test_convert_grid_text(tmp_path):
    from mapf.convert import main

    txt = tmp_path / "m.txt"
    txt.write_text("// comment\n..#\n...\n")
    out = tmp_path / "m.json"
    main([str(txt), str(out)])
    g = load_graph(str(out))
    assert len(g) == 5 and len(g.edges) == 5


def test_ascii_rendering():
    from mapf.visualize import render_ascii

    inst = Instance.load("maps/corridor.json")
    frame = render_ascii(inst, cbs(inst).paths, 0)
    assert frame.splitlines()[1].startswith("0") and "#" in frame


def test_cli_end_to_end(tmp_path):
    from mapf.cli import main

    svg = tmp_path / "out.svg"
    for scenario in ("maps/crossing.json", "maps/ring.json", "maps/petersen.json"):
        for algo in ("cbs", "pp", "pibt"):
            assert main([scenario, "-a", algo, "--svg", str(svg)]) == 0
            assert svg.read_text().startswith("<svg")
    assert main(["--random", "maps/room.json", "-n", "5"]) == 0
    assert main(["--movingai", "tests/data/tiny.map", "tests/data/tiny.scen", "-n", "3"]) == 0
