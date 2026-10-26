import pytest

from mapf import Grid, Instance, cbs, find_conflict, prioritized_planning, validate
from mapf.astar import ConstraintTable, space_time_astar


def open_grid(n):
    return Grid.from_lines(["." * n] * n)


def test_astar_respects_vertex_constraint():
    g = Grid.from_lines(["..."])
    h = g.distances_from((0, 2))
    assert space_time_astar(g, (0, 0), (0, 2), h) == [(0, 0), (0, 1), (0, 2)]
    table = ConstraintTable(vertex={((0, 1), 1)})
    path = space_time_astar(g, (0, 0), (0, 2), h, table)
    assert path == [(0, 0), (0, 0), (0, 1), (0, 2)]


def test_astar_waits_for_goal_constraint_to_expire():
    g = Grid.from_lines(["..."])
    h = g.distances_from((0, 1))
    table = ConstraintTable(vertex={((0, 1), 3)})
    path = space_time_astar(g, (0, 0), (0, 1), h, table)
    assert len(path) - 1 >= 4 and path[-1] == (0, 1)


def test_swap_in_open_grid_is_valid_and_optimal():
    inst = Instance(open_grid(3), ((0, 0), (0, 2)), ((0, 2), (0, 0)))
    sol = cbs(inst)
    assert validate(inst, sol.paths) == []
    assert sol.sum_of_costs == 6  # one detours around the other: 2 + 4


def test_cbs_optimal_vs_prioritized():
    inst = Instance.load("maps/crossing.txt")
    c, p = cbs(inst), prioritized_planning(inst)
    assert validate(inst, c.paths) == []
    assert validate(inst, p.paths) == []
    assert c.sum_of_costs <= p.sum_of_costs


def test_corridor_swap_uses_bay():
    inst = Instance.load("maps/corridor.txt")
    sol = cbs(inst)
    assert validate(inst, sol.paths) == []


def test_unsolvable_head_on_corridor_returns_none():
    inst = Instance(Grid.from_lines(["....."]), ((0, 0), (0, 4)), ((0, 4), (0, 0)))
    assert cbs(inst, max_nodes=200) is None


def test_validate_detects_conflicts():
    inst = Instance(open_grid(3), ((0, 0), (0, 2)), ((0, 2), (0, 0)))
    bad = [[(0, 0), (0, 1), (0, 2)], [(0, 2), (0, 1), (0, 0)]]
    assert any("conflict" in e for e in validate(inst, bad))
    swap = [[(0, 0), (0, 1)], [(0, 1), (0, 0)]]
    assert find_conflict(swap).prev is not None


def test_goal_blocking_after_arrival_is_conflict():
    # agent 1 passes through agent 0's goal after agent 0 has parked there
    paths = [[(0, 0), (0, 1)], [(1, 1), (1, 1), (0, 1)]]
    assert find_conflict(paths) is not None


@pytest.mark.parametrize("seed", range(10))
def test_random_instances_cbs_valid_and_not_worse_than_pp(seed):
    grid = Grid.from_lines(open("maps/room.txt").read().split("//")[1].split("\n")[1:9])
    inst = Instance.random(grid, 4, seed)
    c = cbs(inst, max_nodes=20_000)
    assert c is not None
    assert validate(inst, c.paths) == []
    p = prioritized_planning(inst)
    if p is not None:
        assert validate(inst, p.paths) == []
        assert c.sum_of_costs <= p.sum_of_costs
