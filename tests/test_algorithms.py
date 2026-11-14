import pytest

from mapf import (
    ALGORITHMS,
    Graph,
    Instance,
    cbs,
    count_conflicts,
    ecbs,
    id_cbs,
    joint_astar,
    lacam,
    lns,
    prioritized_planning,
    prioritized_restarts,
    solve,
    validate,
    whca,
)
from mapf.ecbs import FocalList

SMALL = ["maps/crossing.json", "maps/corridor.json", "maps/ring.json", "maps/petersen.json"]


def small_instances():
    grid = Graph.from_grid(["...", "...", "..."])
    for seed in range(12):
        yield Instance.random(grid, 3, seed)
    tree_plus = Graph.random_connected(8, extra_edges=3, seed=1)
    for seed in range(8):
        yield Instance.random(tree_plus, 3, seed)


def test_optimal_solvers_agree_with_exhaustive_search():
    for inst in small_instances():
        costs = {
            "astar": joint_astar(inst).sum_of_costs,
            "cbs": cbs(inst).sum_of_costs,
            "id_cbs": id_cbs(inst).sum_of_costs,
            "ecbs1": ecbs(inst, w=1.0).sum_of_costs,
        }
        assert len(set(costs.values())) == 1, costs


@pytest.mark.parametrize("w", [1.0, 1.3, 2.0])
def test_ecbs_respects_suboptimality_bound(w):
    grid = Graph.from_grid(["....", ".##.", "....", "...."])
    for seed in range(12):
        inst = Instance.random(grid, 5, seed)
        opt = cbs(inst, timeout=10)
        if opt is None:
            continue
        sol = ecbs(inst, w=w)
        assert validate(inst, sol.paths) == []
        assert opt.sum_of_costs <= sol.sum_of_costs <= w * opt.sum_of_costs + 1e-9
        assert sol.sum_of_costs <= w * sol.extra["lower_bound"] + 1e-9


def test_ecbs_rejects_w_below_one():
    with pytest.raises(ValueError):
        ecbs(Instance.load("maps/crossing.json"), w=0.5)


def test_id_cbs_keeps_independent_agents_apart():
    inst = Instance(Graph.from_grid(["." * 10, "#" * 10, "." * 10]), (0, 10), (9, 19))
    sol = id_cbs(inst)
    assert sol.extra["largest_group"] == 1 and sol.expanded == 2


@pytest.mark.parametrize("name", ALGORITHMS)
@pytest.mark.parametrize("path", SMALL)
def test_every_algorithm_returns_valid_solutions(name, path):
    inst = Instance.load(path)
    sol = solve(name, inst, timeout=3, **({"time_limit": 0.1} if name == "lns" else {}))
    if ALGORITHMS[name].complete:
        assert sol is not None
    if sol is not None:
        assert validate(inst, sol.paths) == []
        if ALGORITHMS[name].optimal:
            assert sol.sum_of_costs == cbs(inst).sum_of_costs


def test_lacam_is_complete_where_pibt_style_planners_get_stuck():
    inst = Instance.load("maps/corridor.json")  # the swap needs a detour into the bay
    sol = lacam(inst)
    assert sol is not None and validate(inst, sol.paths) == []


def test_lacam_gives_up_on_unsolvable_instance():
    inst = Instance(Graph(range(5), [(i, i + 1) for i in range(4)]), (0, 4), (4, 0))
    assert lacam(inst, max_nodes=2000) is None


def test_lns_never_worse_than_start_and_reproducible():
    inst = Instance.load("maps/warehouse.json")
    a = lns(inst, time_limit=60, seed=3, max_iterations=40)
    b = lns(inst, time_limit=60, seed=3, max_iterations=40)
    assert validate(inst, a.paths) == []
    assert a.sum_of_costs <= a.extra["initial_cost"]
    assert a.paths == b.paths


def test_restarts_not_worse_than_single_pp():
    inst = Instance.load("maps/warehouse.json")
    assert prioritized_restarts(inst, tries=5).sum_of_costs <= prioritized_planning(inst).sum_of_costs


def test_whca_validates_options_and_solves_warehouse():
    with pytest.raises(ValueError):
        whca(Instance.load("maps/ring.json"), window=2, step=5)
    inst = Instance.load("maps/warehouse.json")
    sol = whca(inst, window=10, step=5)
    assert sol is not None and validate(inst, sol.paths) == []


@pytest.mark.parametrize("name", ["ecbs", "lacam", "pp_restarts", "whca", "lns"])
def test_scalable_algorithms_solve_40_agent_warehouse(name):
    inst = Instance.load("maps/warehouse.json")
    sol = solve(name, inst, timeout=20)
    assert sol is not None and validate(inst, sol.paths) == []
    if name == "ecbs":
        assert sol.sum_of_costs <= 1.5 * sol.extra["lower_bound"] + 1e-9


def test_count_conflicts():
    assert count_conflicts([[0, 1], [3, 4]]) == 0
    assert count_conflicts([[0, 1, 2], [2, 1, 0]]) == 1  # vertex conflict at t=1
    assert count_conflicts([[0, 1], [1, 0]]) == 1  # swap
    assert count_conflicts([[0, 1], [2, 1]]) == 1  # both end on node 1 (and then stay)


def test_focal_list_prefers_key_inside_the_bound_and_falls_back_to_best_low():
    fl = FocalList(w=2.0)
    fl.push("cheap-conflicted", low=10, adm=10, key=(5, 10))
    fl.push("pricey-clean", low=12, adm=19, key=(0, 19))  # within 2 * 10
    fl.push("too-expensive", low=11, adm=30, key=(0, 30))  # outside the bound
    assert fl.pop() == "pricey-clean"
    assert fl.pop() == "cheap-conflicted"
    assert fl.pop() == "too-expensive"  # focal is empty: falls back to the open list
    assert len(fl) == 0


def test_registry_rejects_unknown_algorithm():
    with pytest.raises(ValueError, match="unknown algorithm"):
        solve("nope", Instance.load("maps/ring.json"))


def test_cli_lists_algorithms_and_passes_options(capsys):
    from mapf.cli import main

    assert main(["--list-algos"]) == 0
    assert "lacam" in capsys.readouterr().out
    assert main(["maps/crossing.json", "-a", "ecbs", "-o", "w=1.2"]) == 0
    assert main(["maps/crossing.json", "-a", "lns", "-o", "time_limit=0.05", "-o", "neighborhood=4"]) == 0
    with pytest.raises(SystemExit):
        main([])
