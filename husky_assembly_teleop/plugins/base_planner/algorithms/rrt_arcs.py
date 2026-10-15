"""
RRT-Connect over the base's floor pose (x, y, yaw), then arcs for its corners, checked for a range of turning centres.

The path leaves the start and reaches the target driving straight, and its corners become arcs where they fit. It is
checked as the follower drives it with its model of the robot (crl_husky config/base_models), the true turning centre
anywhere in a range of xICR (`path.driven`).
"""

from __future__ import annotations

import itertools
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from crl_husky.base_model import load_model

from ...planning.search import TIME_LIMIT, PlanResult, connect
from ..path import (ARC_STEP, BasePath, densify, driven, move_margins, round_corners, steer, steer_cost, steer_points,
                    timed_path)
from ..world import COLLISION_MARGIN, PlanningWorld
from .base import Algorithm, Debug

#: Where the turning centre may be along the body, metres, as crl_husky's xICR (negative: ahead of base_footprint);
#: around Alice's fitted -0.13 m (config/base_models in crl_husky).
XICR_RANGE = (-0.20, -0.10)
#: Largest gap between the xICR values a path is checked with, metres.
XICR_STEP = 0.025
#: The robot model the onboard follower runs with (its `model:=`), from crl_husky config/base_models.
FOLLOWER_MODEL = "alice_tiles"
#: Arc radii tried in place of a turn on the spot, widest first, metres.
ARC_RADII = (2.0, 1.0, 0.5)
#: Straight drive off the start and onto the target, metres, so the robot does not turn on the spot beside them.
LEAD = 0.5
#: Samples are drawn from the box around start and goal, grown by this much per side, metres.
SAMPLE_PADDING = 3.0
#: Collision-checking resolution along a move: metres driven, radians turned.
POSITION_STEP = 0.05
YAW_STEP = 0.05

#: Debug layers, back to front: name and colour (r, g, b).
SAMPLED = "samples", (173, 181, 189)
EXPLORED = "moves tried, as far as clear", (134, 142, 150)
BLOCKED = "first blocked pose of a move", (224, 49, 49)
CORNERS = "search's corners", (247, 103, 7)
LEADS = "leads", (174, 62, 201)
ARC_REFUSED = "arcs refused", (255, 135, 135)
ARC_KEPT = "arcs kept", (47, 158, 68)
DRIVEN = "path as driven, per xICR", (21, 170, 191)
FAILED_AT = "where a check failed", (201, 42, 42)


@dataclass(frozen=True)
class Settings:
    """The RRT's settings for one search.

    Attributes:
        x_icr: Lowest and highest xICR the robot may have, metres.
        follower_model: The robot model the onboard follower runs with (`model:=`), as in crl_husky config/base_models.
        lead: Straight drive off the start and onto the target, metres; 0 for none.
        arcs: Whether corners become arcs.
        margin: Keep at least this far from everything else, metres.
    """

    x_icr: tuple[float, float] = XICR_RANGE
    follower_model: str = FOLLOWER_MODEL
    lead: float = LEAD
    arcs: bool = True
    margin: float = COLLISION_MARGIN


class RrtArcs(Algorithm):
    """RRT-Connect with turn-drive-turn moves, smoothed, corners rounded into arcs, checked for a range of xICR."""

    label = "RRT + arcs"

    def __init__(self):
        """No widgets until `add_settings`."""
        self._widgets = None

    def add_settings(self, gui) -> None:
        """Add the xICR range, follower model, lead, arcs and margin."""
        models = follower_models()
        self._widgets = (
            gui.add_multi_slider("xICR m", min=-0.30, max=0.10, step=0.01, initial_value=XICR_RANGE,
                                 hint="Where the turning centre may really be, as crl_husky's xICR (negative: ahead "
                                      "of base_footprint); the path is checked for all of them"),
            gui.add_dropdown("Follower model", options=models,
                             initial_value=FOLLOWER_MODEL if FOLLOWER_MODEL in models else models[0],
                             hint="The robot model the onboard follower runs with (its model:=), from crl_husky "
                                  "config/base_models; the path is checked as that follower drives it"),
            gui.add_slider("Lead m", min=0.0, max=1.5, step=0.05, initial_value=LEAD,
                           hint="Straight drive off the start and onto the target, where there is room"),
            gui.add_checkbox("Arcs", initial_value=True, hint="Round corners into arcs where they fit"),
            gui.add_slider("Margin m", min=0.0, max=0.3, step=0.01, initial_value=COLLISION_MARGIN,
                           hint="Keep at least this far from the other robots and bodies"))

    def settings(self) -> Settings:
        """The widgets' values, or the defaults."""
        if self._widgets is None:
            return Settings()
        x_icr, model, lead, arcs, margin = (widget.value for widget in self._widgets)
        return Settings(tuple(float(value) for value in x_icr), str(model), float(lead), bool(arcs), float(margin))

    def plan(self, world, own, start, goal, abort, settings, debug) -> PlanResult[BasePath]:
        """See `plan`."""
        return plan(world, own, start, goal, abort, settings, debug)


def plan(world: PlanningWorld, own: str, start, goal, abort: threading.Event, settings: Settings = Settings(),
         debug: Debug | None = None) -> PlanResult[BasePath]:
    """Plan a collision-free base path from start to goal. Runs on the worker thread.

    Args:
        world: The planning world, already synced.
        own: The robot to plan for, by id.
        start: (x, y, yaw) where it is.
        goal: (x, y, yaw) where it should end up.
        abort: Set from the main thread to end the search early.
        settings: The search's settings.
        debug: Where to record the search's inner workings; None records nothing.

    Returns:
        PlanResult[BasePath]: The path, or why there is none.
    """
    debug = debug or Debug()
    for layer in (SAMPLED, EXPLORED, BLOCKED, CORNERS, LEADS, ARC_REFUSED, ARC_KEPT, DRIVEN, FAILED_AT):
        debug.layer(*layer)
    started = phase = time.time()
    start, goal = tuple(map(float, start)), tuple(map(float, goal))
    margin = settings.margin

    def lap(text: str) -> None:
        """Note how long the phase since the last lap took."""
        nonlocal phase
        debug.note(f"{text} ({time.time() - phase:.2f} s)")
        phase = time.time()

    # * Report a blocked start or target explicitly; the search would only fail.
    for name, pose in (("start", start), ("target", goal)):
        other = world.hit_by(own, pose, margin)
        if other is not None:
            return PlanResult(None, f"{name} is in collision with {other}", time.time() - started)

    try:
        x_model = load_model(settings.follower_model)["xICR"]
    except (FileNotFoundError, ValueError) as error:
        return PlanResult(None, f"follower model: {error}", time.time() - started)
    low_icr, high_icr = min(settings.x_icr), max(settings.x_icr)
    icrs = np.linspace(low_icr, high_icr, max(int(math.ceil((high_icr - low_icr) / XICR_STEP)), 1) + 1)
    # The worst case, for the search: the farthest the turning centre may be from base_footprint.
    farthest = max(abs(low_icr), abs(high_icr))
    # Where the last failed `clear` hit, as driven with which xICR, and what.
    failure: list = []

    def clear(poses) -> bool:
        """Whether driving through `poses` (yaw unwrapped) stays clear, for every xICR in the range."""
        for icr in icrs:
            for pose in driven(poses, icr, x_model, POSITION_STEP, YAW_STEP):
                if abort.is_set():
                    return False
                other = world.hit_by(own, pose, margin)
                if other is not None:
                    failure[:] = [pose, icr, other]
                    return False
        return True

    def cancelled() -> PlanResult[BasePath]:
        return PlanResult(None, "cancelled", time.time() - started)

    def found(poses, direct: bool = False) -> PlanResult[BasePath]:
        for icr in icrs:
            debug.path(DRIVEN[0], driven(poses, icr, x_model, POSITION_STEP, YAW_STEP), DRIVEN[1])
        lap(f"found: {len(poses)} waypoints, checked for {len(icrs)} xICR values, {low_icr:+.3f} to {high_icr:+.3f} m, "
            f"follower model {settings.follower_model} (xICR {x_model:+.3f} m)")
        # `densify` gives the poses after the first: put the start back, or the plan starts past it.
        return PlanResult(timed_path([poses[0]] + densify(poses, ARC_STEP, math.inf)), seconds=time.time() - started,
                          direct=direct)

    # * The straight move, if it is clear.
    direct = [start] + steer(start, goal)
    if clear(direct):
        return found(direct, direct=True)
    if abort.is_set():
        return cancelled()
    lap(f"straight move blocked by {failure[2]}")
    debug.points(FAILED_AT[0], [failure[0]], FAILED_AT[1])

    # * Leave the start and reach the target driving straight, towards the other one first, where the far end of
    # * that drive has room to turn.
    def lead(pose, sign: float):
        heading = np.array([math.cos(pose[2]), math.sin(pose[2])])
        ahead = 1.0 if np.dot(heading, np.subtract(goal[:2], start[:2])) >= 0.0 else -1.0
        for side in (ahead, -ahead) if settings.lead > 0.0 else ():
            x, y = np.array(pose[:2]) + sign * side * settings.lead * heading
            end = (float(x), float(y), pose[2])
            roomy = world.hit_by(own, end, margin + 2.0 * farthest) is None
            if roomy and clear([pose, end] if sign > 0 else [end, pose]):
                debug.path(LEADS[0], [pose, end], LEADS[1])
                return end
        return pose

    depart, arrive = lead(start, 1.0), lead(goal, -1.0)
    if abort.is_set():
        return cancelled()
    lap(f"leads: {'off the start' if depart != start else 'none off the start'}, "
        f"{'onto the target' if arrive != goal else 'none onto the target'}")

    low = np.minimum(start[:2], goal[:2]) - SAMPLE_PADDING
    high = np.maximum(start[:2], goal[:2]) + SAMPLE_PADDING
    sampled: list = []

    def sample():
        x, y = np.random.uniform(low, high)
        sampled.append((x, y))
        return (float(x), float(y), float(np.random.uniform(-math.pi, math.pi)))

    # Extra margin of each pose the search has stepped through, for whichever move stepped through it.
    extra: dict[tuple, float] = {}
    # For debug: every move tried, and every pose checked (whether it hit).
    moves: list = []
    checked: dict[tuple, bool] = {}

    def extend(q1, q2):
        points = steer_points(q1, q2, POSITION_STEP, YAW_STEP)
        for point, extra_margin in zip(points, move_margins(q1, points, farthest)):
            extra[point] = max(extra.get(point, 0.0), extra_margin)
        if debug.on:
            moves.append((q1, points))
        return points

    def collides(q) -> bool:
        hit = world.hit_by(own, q, margin + extra.get(q, 0.0)) is not None
        if debug.on:
            checked[q] = hit
        return hit

    def tried_arc(stretch) -> bool:
        ok = clear(stretch)
        name, color = ARC_KEPT if ok else ARC_REFUSED
        debug.path(name, stretch, color)
        return ok

    def rounded(poses):
        """The corners rounded, except the one before the drive onto the target: keep that straight."""
        if not settings.arcs:
            return poses
        if arrive == goal:
            return round_corners(poses, ARC_RADII, tried_arc)
        return _unwrapped(round_corners(poses[:-1], ARC_RADII, tried_arc) + poses[-1:])

    # * Join start, search and target with `steer`, round the corners, and check the whole path as it will be driven.
    # * Search again while time is left: the search checks its moves one by one, not the corners where they join.
    try:
        for attempt in itertools.count(1):
            left = TIME_LIMIT - (time.time() - started)
            if left <= 0.0:
                return PlanResult(None, f"no path found in {TIME_LIMIT:.0f} s", time.time() - started)
            corners = connect(depart, arrive, steer_cost, sample, extend, collides, abort, left)
            lap(f"search {attempt}: {'no path' if corners.states is None else f'{len(corners.states)} corners'}, "
                f"{len(moves)} moves and {len(checked)} poses checked so far")
            if corners.states is None:
                generic = corners.reason.startswith("no path found")
                reason = f"no path found in {TIME_LIMIT:.0f} s" if generic and not abort.is_set() else corners.reason
                return PlanResult(None, reason, time.time() - started)
            poses = [start] + ([depart] if depart != start else [])
            for q1, q2 in zip(corners.states[:-1], corners.states[1:]):
                poses += steer(q1, q2)
            poses = _unwrapped(poses + ([goal] if arrive != goal else []))
            debug.path(CORNERS[0], poses, CORNERS[1])
            for candidate in (rounded(poses), poses):
                if clear(candidate):
                    return found(candidate)
            if abort.is_set():
                return cancelled()
            lap(f"search {attempt}: path as driven hits {failure[2]} with xICR {failure[1]:+.3f} m; searching again")
            debug.points(FAILED_AT[0], [failure[0]], FAILED_AT[1])
    finally:
        _draw_search(debug, sampled, moves, checked)


def follower_models() -> list[str]:
    """Names of the robot models in crl_husky config/base_models, "ideal" first."""
    try:
        folder = Path(get_package_share_directory("crl_husky")) / "config" / "base_models"
    except LookupError:
        return ["ideal"]
    return sorted((path.stem for path in folder.glob("*.yaml")), key=lambda name: (name != "ideal", name))


def _draw_search(debug: Debug, sampled, moves, checked) -> None:
    """Draw the samples, and each move tried as far as it was checked clear, with the pose that stopped it."""
    if not debug.on:
        return
    debug.points(SAMPLED[0], sampled, SAMPLED[1])
    explored, blocked = [], []
    for q1, points in moves:
        last = q1
        for point in points:
            hit = checked.get(point)
            if hit is None:
                break  # never checked: smoothing only measured it
            if hit:
                blocked.append(point)
                break
            last = point
        if last is not q1:
            explored.append((q1[:2], last[:2]))
    debug.lines(EXPLORED[0], explored, EXPLORED[1])
    debug.points(BLOCKED[0], blocked, BLOCKED[1])


def _unwrapped(poses) -> list[tuple[float, float, float]]:
    """(x, y, yaw) poses with yaw unwrapped, so no step turns the long way."""
    poses = np.array(poses, dtype=float)
    poses[:, 2] = np.unwrap(poses[:, 2])
    return [tuple(pose) for pose in poses]
