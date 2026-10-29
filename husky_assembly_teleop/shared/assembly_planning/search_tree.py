"""
A search's two trees while pybullet_planning's birrt grows them, rebuilt from its `draw_fn`, for drawing and progress.

- The worker calls `begin` before each search, hands it `draw_fn`, and calls `end` after it. birrt calls `draw_fn` on
  the worker with each iteration's sample, then every node of both trees again (`rrt_connect`). An iteration's nodes
  are published whole when the next sample comes: `view` is always one iteration.
- * Cheap for fast searches: an iteration is recorded only RECORD_PERIOD after the last one published (and the last of
  each run always). Since birrt draws every node again each iteration, a skipped one loses nothing.
- ! birrt draws as many nodes of the bigger tree as the smaller one has: the bigger tree's newest nodes show later.
- Each run (of `runs`) takes `max_iterations` samples unless it runs out of time: counting samples tells the run; the
  run before is logged.
- An iteration of tamp's search can take many seconds (a check of the whole cell takes ~0.1 s), so every configuration
  it checks is kept too, as it is checked (`checked`, `probes`): the extension being tried now, and where it collided.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from functools import cached_property
from typing import Callable, Hashable, Sequence

import numpy as np
from pybullet_planning.motion_planners.utils import RRT_RESTARTS
from scipy.spatial import cKDTree

#: birrt's runs of RRT-Connect by default: pybullet_planning's restarts and the first.
RUNS = RRT_RESTARTS + 1
#: Seconds between recorded iterations; a slower search records every one.
RECORD_PERIOD = 0.5
#: A node this close to the start or the goal, radians in every joint, is that tree's root (tamp's own values).
ROOT_TOLERANCE = 1e-6
#: The trees: grown from the start, and from the goal.
START_TREE, GOAL_TREE = 0, 1


@dataclass(frozen=True, eq=False)
class TreeView:
    """One iteration of the search, as published; views compare by identity.

    Attributes:
        label: The try, e.g. "try 2/3".
        run: Which of birrt's runs, from 1.
        runs: Runs birrt may make.
        iteration: The run's iteration, from 1.
        max_iterations: Iterations a run may take.
        configs: (N, joints) each node's joint values.
        points: (N, arms, 3) each node's tool0 positions in the world, one per moving arm.
        parents: (N,) each node's parent, -1 for a root.
        trees: (N,) START_TREE or GOAL_TREE.
        over: The search ended.
        found: It ended with a path.
    """

    label: str
    run: int
    runs: int
    iteration: int
    max_iterations: int
    configs: np.ndarray
    points: np.ndarray
    parents: np.ndarray
    trees: np.ndarray
    over: bool = False
    found: bool = False

    def counts(self) -> tuple[int, int]:
        """Nodes in the start's tree and in the goal's."""
        return int(np.sum(self.trees == START_TREE)), int(np.sum(self.trees == GOAL_TREE))

    @cached_property
    def gap(self) -> tuple[float, int, int] | None:
        """The closest nodes of the two trees: their joint-space distance (rad) and indices; None if a tree is empty.

        ? birrt's own distance wraps no joint either: a UR joint turns at most ±2π, so this is the distance it uses.
        """
        start, goal = np.flatnonzero(self.trees == START_TREE), np.flatnonzero(self.trees == GOAL_TREE)
        if not len(start) or not len(goal):
            return None
        distances, nearest = cKDTree(self.configs[goal]).query(self.configs[start])
        best = int(np.argmin(distances))
        return float(distances[best]), int(start[best]), int(goal[nearest[best]])

    def line(self) -> str:
        """The progress in one line, e.g. "try 1/3 · run 2/3 · it 31/50 · gap 0.84 rad · 812+640 nodes"."""
        gap = "" if self.gap is None else f" · gap {self.gap[0]:.2f} rad"
        ended = " · found" if self.found else " · ended" if self.over else ""
        return (f"{self.label} · run {self.run}/{self.runs} · it {self.iteration}/{self.max_iterations}{gap} · "
                f"{'+'.join(str(count) for count in self.counts())} nodes{ended}")


class SearchTree:
    """Collects birrt's trees on the worker (`draw_fn`); `view` gives the main thread the last whole iteration."""

    def __init__(self, log: Callable[[str], None] = lambda _line: None):
        """Nothing searched yet.

        Args:
            log: Gets a line as each birrt run ends. Called on the worker.
        """
        self._log = log
        #: The last published iteration, or None: replaced whole, never changed, so the main thread reads it as is.
        self.view: TreeView | None = None
        self.begin("", (), (), lambda _config: np.zeros((0, 3)), 1)
        self.view = None

    def begin(self, label: str, start: Sequence[float], goal: Sequence[float],
              locate: Callable[[Sequence[float]], np.ndarray], max_iterations: int, runs: int = RUNS) -> None:
        """Start recording one search; the drawing is cleared. Worker thread.

        Args:
            label: The try, e.g. "try 2/3".
            start: The start's joint values, in the order the search's nodes have.
            goal: The goal's joint values, the same order.
            locate: A node's tool0 positions in the world, (arms, 3); called once per node, between the search's checks.
            max_iterations: Iterations a run may take.
            runs: Runs birrt may make: its restarts and the first.
        """
        self._label, self._locate, self._max_iterations, self._runs = label, locate, max(max_iterations, 1), runs
        self._start, self._goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        #: Each node's tool0 positions by its joint values: a node is located once, however often it is drawn.
        self._located: dict[Hashable, np.ndarray] = {}
        self._run, self._iteration, self._samples = 1, 1, 0
        self._recording, self._published_at = False, time.monotonic()
        self._new_iteration()
        self.view = self._publish()

    def checked(self, points: np.ndarray, collided: bool) -> None:
        """A configuration tamp just checked: its tool0 positions (arms, 3), and whether it collided. Worker thread."""
        self._probes.append((points, collided))

    def probes(self) -> tuple[np.ndarray, np.ndarray]:
        """The configurations checked since this iteration began: (P, arms, 3) tool0 positions, and (P,) collided.

        * A copy, for the main thread: the worker only appends to the list, or replaces it.
        """
        probes = list(self._probes)
        if not probes:
            return np.zeros((0, 1, 3)), np.zeros(0, dtype=bool)
        return np.asarray([points for points, _ in probes]), np.asarray([hit for _, hit in probes], dtype=bool)

    def draw_fn(self, config: Sequence[float], segment: Sequence[Sequence[float]], *_valid: bool) -> None:
        """The hook birrt calls with a sample (no segment, not a root), a root, or a node and its edge. Worker."""
        if not len(segment):
            tree = self._root_of(config)
            if tree is None:
                self._sample()
            elif self._recording:
                self._add(config, -1, tree)
            return
        if not self._recording:
            return
        # * Keyed by tree too: the node where the trees meet has the same joint values in both.
        key = tuple(segment[1])
        parent = self._index.get((START_TREE, key), self._index.get((GOAL_TREE, key)))
        if parent is not None:
            self._add(config, parent, self._trees[parent])

    def clear(self) -> None:
        """Forget the last search: nothing to draw until the next `begin`."""
        self.view = None

    def end(self, found: bool) -> None:
        """Publish the last iteration, and say how the last run ended unless a path was found. Worker thread."""
        self._probes = []
        if self._configs:
            self.view = self._publish(over=True, found=found)
        elif self.view is not None:
            self.view = replace(self.view, over=True, found=found)
        if not found and self.view is not None and len(self.view.trees):
            self._log(f"{self._label}: birrt run {self.view.run}/{self._runs} ended: {self.view.line()}")

    # --- --- --- --- --- ON THE WORKER --- --- --- --- ---

    def _root_of(self, config: Sequence[float]) -> int | None:
        """START_TREE or GOAL_TREE if `config` is the start or the goal, else None (a sample)."""
        values = np.asarray(config, dtype=float)
        for tree, root in ((START_TREE, self._start), (GOAL_TREE, self._goal)):
            if values.shape == root.shape and float(np.max(np.abs(values - root), initial=0.0)) <= ROOT_TOLERANCE:
                return tree
        return None

    def _sample(self) -> None:
        """A new iteration begins: publish the one before if it was recorded, log a run that ended, pick this one."""
        if self._recording and self._configs:
            self.view = self._publish()
            self._published_at = time.monotonic()
        self._samples += 1
        run, iteration = divmod(self._samples - 1, self._max_iterations)
        if run + 1 != self._run and self.view is not None and len(self.view.trees):
            self._log(f"{self._label}: birrt run {self._run}/{self._runs} ended: {self.view.line()}")
        self._run, self._iteration = run + 1, iteration + 1
        self._recording = (self._iteration == self._max_iterations
                           or time.monotonic() - self._published_at >= RECORD_PERIOD)
        self._new_iteration()

    def _add(self, config: Sequence[float], parent: int, tree: int) -> None:
        """Add a node to this iteration, unless its tree has it; located once, however often it is drawn."""
        key = tuple(config)
        if (tree, key) in self._index:
            return
        points = self._located.get(key)
        if points is None:
            points = self._located[key] = np.asarray(self._locate(config), dtype=float)
        self._index[(tree, key)] = len(self._configs)
        self._configs.append(config)
        self._points.append(points)
        self._parents.append(parent)
        self._trees.append(tree)

    def _new_iteration(self) -> None:
        """Empty lists for the next iteration's nodes and checks."""
        self._probes: list[tuple[np.ndarray, bool]] = []
        #: Each node's index by (its tree, its joint values).
        self._index: dict[tuple[int, Hashable], int] = {}
        self._configs: list[Sequence[float]] = []
        self._points: list[np.ndarray] = []
        self._parents: list[int] = []
        self._trees: list[int] = []

    def _publish(self, over: bool = False, found: bool = False) -> TreeView:
        """This iteration's nodes as a view."""
        joints = len(self._start)
        configs = np.asarray(self._configs, dtype=float).reshape(len(self._configs), joints)
        points = np.asarray(self._points, dtype=float) if self._points else np.zeros((0, 1, 3))
        return TreeView(self._label, self._run, self._runs, self._iteration, self._max_iterations, configs, points,
                        np.asarray(self._parents, dtype=int), np.asarray(self._trees, dtype=int), over, found)
