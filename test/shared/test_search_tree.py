"""The free search's trees, rebuilt from pybullet_planning's own birrt on a small maze through `draw_fn`."""

from __future__ import annotations

import numpy as np
from pybullet_planning.motion_planners.rrt_connect import birrt

from husky_assembly_teleop.shared.assembly_planning import search_tree
from husky_assembly_teleop.shared.assembly_planning.search_tree import GOAL_TREE, RUNS, START_TREE, SearchTree

START, GOAL = (1.0, 5.0), (9.0, 5.0)
STEP = 0.25


def search(tree: SearchTree, wall_top: float, iterations: int, seed: int = 3) -> list | None:
    """Run birrt from START to GOAL around a wall at x 4.5-5.5 up to `wall_top` (10 closes it), drawing into `tree`."""
    rng = np.random.default_rng(seed)

    def extend(q1, q2):
        q1, q2 = np.asarray(q1), np.asarray(q2)
        steps = max(int(np.ceil(np.linalg.norm(q2 - q1) / STEP)), 1)
        return [tuple(q1 + (q2 - q1) * k / steps) for k in range(1, steps + 1)]

    return birrt(START, GOAL, lambda a, b: float(np.linalg.norm(np.subtract(a, b))),
                 lambda: tuple(rng.uniform(0.0, 10.0, 2)), extend,
                 lambda q, **_options: 4.5 <= q[0] <= 5.5 and q[1] <= wall_top, max_iterations=iterations,
                 max_time=10.0,
                 draw_fn=tree.draw_fn)


def recording(iterations: int) -> tuple[SearchTree, list, list]:
    """A tree that logs into a list, and the configs it located, begun on the maze for runs of `iterations`."""
    lines, located = [], []
    tree = SearchTree(lines.append)

    def locate(config):
        located.append(tuple(config))
        return np.array([[config[0], config[1], 0.0]])

    tree.begin("try 1/3", START, GOAL, locate, iterations)
    return tree, lines, located


def check_shape(view) -> None:
    """Each tree hangs from its own root, every node from an earlier node of its tree."""
    assert view.configs.shape == (len(view.trees), 2) and view.points.shape == (len(view.trees), 1, 3)
    roots = np.flatnonzero(view.parents < 0)
    assert sorted(view.trees[roots]) == [START_TREE, GOAL_TREE]
    for tree, root in ((START_TREE, START), (GOAL_TREE, GOAL)):
        assert np.allclose(view.configs[roots[view.trees[roots] == tree][0]], root)
    children = np.flatnonzero(view.parents >= 0)
    assert np.all(view.parents[children] < children)
    assert np.all(view.trees[view.parents[children]] == view.trees[children])


def test_a_found_path_is_recorded_whole_and_each_node_located_once(monkeypatch):
    """Around the wall, every iteration recorded: the last view holds both trees, and a node is located once."""
    monkeypatch.setattr(search_tree, "RECORD_PERIOD", 0.0)
    tree, lines, located = recording(40)
    path = search(tree, wall_top=8.0, iterations=40)
    tree.end(found=path is not None)
    view = tree.view
    # * Each run that ended without a path was logged; the last one found it.
    assert path is not None and view.over and view.found and len(lines) == view.run - 1
    check_shape(view)
    assert len(located) == len(set(located)) and len(located) >= len(view.trees)
    # ? The nodes where the trees met are the bigger tree's newest: birrt draws only as many as the smaller one has.
    assert view.gap is not None
    assert view.line().startswith(f"try 1/3 · run {view.run}/{RUNS} · it ") and view.line().endswith("nodes · found")


def test_each_birrt_run_on_a_closed_wall_is_logged_and_the_last_one_stays_drawn(monkeypatch):
    """A wall with no way round: every run fails and is logged; a fast search records only each run's last iteration."""
    monkeypatch.setattr(search_tree, "RECORD_PERIOD", 1e9)
    tree, lines, _located = recording(6)
    published, publish = [], tree._publish
    tree._publish = lambda **options: published.append(publish(**options)) or published[-1]
    assert search(tree, wall_top=10.0, iterations=6) is None
    tree.end(found=False)
    view = tree.view
    assert view.over and not view.found and view.run == RUNS
    assert [line.split(" ended:")[0] for line in lines] == [f"try 1/3: birrt run {run}/{RUNS}" for run in
                                                            range(1, RUNS + 1)]
    assert all(" · it 6/6 · " in line for line in lines)
    assert [(each.run, each.iteration) for each in published] == [(run, 6) for run in range(1, RUNS + 1)]
    check_shape(view)
    distance, start, goal = view.gap
    assert distance > 1.0 and view.configs[start][0] < 4.5 < 5.5 < view.configs[goal][0]


def test_a_new_try_starts_empty_and_clear_forgets_it():
    """`begin` publishes an empty view at once, so the last try's trees go; `clear` leaves nothing to draw."""
    tree, _lines, _located = recording(40)
    search(tree, wall_top=8.0, iterations=40)
    tree.begin("try 2/3", START, GOAL, lambda config: np.zeros((1, 3)), 40)
    assert len(tree.view.trees) == 0 and tree.view.gap is None and tree.view.label == "try 2/3"
    tree.clear()
    assert tree.view is None


def test_the_checks_of_the_iteration_under_way_are_kept_until_the_next_one_begins():
    """Each check comes as it is made; a sample starts a new iteration with none, and the search's end clears them."""
    tree, _lines, _located = recording(40)
    tree.draw_fn((3.0, 3.0), [])
    tree.checked(np.array([[1.0, 2.0, 0.0]]), False)
    tree.checked(np.array([[4.6, 2.0, 0.0]]), True)
    points, collided = tree.probes()
    assert points.shape == (2, 1, 3) and list(collided) == [False, True]
    tree.draw_fn((7.0, 3.0), [])
    assert len(tree.probes()[1]) == 0
    tree.checked(np.array([[1.0, 2.0, 0.0]]), False)
    tree.end(found=False)
    assert len(tree.probes()[1]) == 0
