"""
The takes as Sweeps and Record show them: a chip per take, and each take's tool0 arc at the robot with its points.

Sweeps colours them by the collision check; Record by what was measured, and each measured point red while its
sample's marker error is over MARKER_ERROR_LIMIT. A take recorded with another tool body or before a base refine counts
as out of date: to record again.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING, Iterable

import numpy as np
import trimesh

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose

from ....ui.quaternion import quaternion_to_wxyz
from ....ui.style import BUSY, CHIP_LINE_STYLE, FAIL, INFO, LEVEL_COLORS, NONE, OK, block, chip, hover
from ....world.checks import BAD
from ..analysis.checks import (LEFT_OUT_LIMITS, left_out_reason, marker_error_detail, marker_error_level,
                               sample_marker_error)
from ..robot.sampling import Sample
from ..robot.sweeps import named

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ..run import Run

#: Joint-space points drawn per sweep step for a take's arc.
ARC_SUBSTEPS = 4
#: Arc colours by take state.
ARC_COLORS = {"unchecked": (150, 150, 150), "clear": (40, 160, 70), "collides": (220, 60, 60),
              "recorded": (30, 110, 220)}
#: A measured point whose sample's marker error is red.
MARKER_RED = (220, 60, 60)
#: Waypoint spheres: radius (metres) and opacity, still to measure and measured.
POINT_RADIUS, POINT_OPACITY = 0.008, 0.45
MEASURED_RADIUS, MEASURED_OPACITY = 0.013, 0.9


def place_on(frame, pose: Pose | None, placed: tuple | None) -> tuple:
    """Show `frame` at `pose`, or hide it for None; only if that changed since `placed`. Returns what it shows now."""
    shown = (True, pose.position, pose.orientation) if pose is not None else (False,)
    if shown != placed:
        frame.visible = pose is not None
        if pose is not None:
            frame.position, frame.wxyz = pose.position, quaternion_to_wxyz(pose.orientation)
    return shown


def take_label(index: int, sweep) -> str:
    """A take's label in the take dropdowns, e.g. "Take 3 · J0"."""
    return f"Take {index + 1} · J{sweep.joint}"


def take_index(label: str) -> int | None:
    """The take index a dropdown label names, or None for no take."""
    parts = label.split()
    return int(parts[1]) - 1 if len(parts) > 1 and parts[0] == "Take" else None


#: The width of a chip row's name, px: every row's chips start at the same place.
ROW_NAME_WIDTH = 68


def chip_row(name: str, chips: str, wrap: bool = False) -> str:
    """A row of chips after a small grey name in a column of its own: one line high, more chips scroll sideways.

    Args:
        name: The row's name; "" leaves the column empty.
        chips: The chips' HTML.
        wrap: Let the chips wrap onto more lines instead, for rows whose chips are fixed while a step shows.
    """
    label = (f'<span title="{escape(name, quote=True)}" style="flex:0 0 {ROW_NAME_WIDTH}px;font-size:0.85em;'
             f'color:{NONE};white-space:nowrap;overflow:hidden;text-overflow:ellipsis">{escape(name)}</span>')
    body = chips if wrap else f'{CHIP_LINE_STYLE}<div class="chip-line">{chips}</div>'
    row = f'<div style="display:flex;align-items:baseline">{label}<div style="flex:1;min-width:0">{body}</div></div>'
    return f'<div style="margin-left:8px">{block(row)}</div>'


def marker_chip(text: str, errors: list[dict | None]) -> str:
    """A chip with the largest marker error of these samples, coloured by it (`marker_error_level`); the worst on hover.

    Its hover names the worst sample by its number among `errors` when there are several.
    """
    largest = [sample_marker_error(error) for error in errors]
    known = [index for index, error in enumerate(largest) if error is not None]
    worst = max(known, key=lambda index: largest[index], default=None)
    value = f"{largest[worst] * 1e3:.2f} mm" if worst is not None else "?"
    if worst is None:
        detail = "marker error unknown"
    elif len(errors) == 1:
        detail = f"marker error: {marker_error_detail(errors[worst])}"
    else:
        detail = f"worst of {len(errors)}: sample {worst + 1}, {marker_error_detail(errors[worst])}"
    return chip(f"{text} · {value}", LEVEL_COLORS[marker_error_level(largest[worst] if worst is not None else None)],
                detail)


def take_state(run: Run, index: int) -> str:
    """A take's state: unchecked, collides, clear, out of date (recorded, but not current) or recorded."""
    if index in run.takes:
        return "recorded" if run.take_current(run.takes[index]) else "out of date"
    if index not in run.collisions:
        return "unchecked"
    return "collides" if run.collisions[index] else "clear"


class TakesView:
    """Draws the run's takes, and the one recording now as its samples come.

    Attributes:
        in_progress: The take being recorded and its samples so far, or None.
        backwards: Whether that take runs from its last waypoint to its first.
    """

    def __init__(self):
        """Nothing drawn yet."""
        self.in_progress: tuple[int, list[Sample]] | None = None
        self.backwards = False
        #: The run the arcs are for, each take's tool0 path in base_footprint, and what each take was drawn as.
        self._run: Run | None = None
        self._arcs: list[np.ndarray] = []
        self._drawn: dict[int, tuple] = {}
        self._frame = None
        self._shown: tuple | None = None

    def chips(self, run: Run, indices: Iterable[int], by_collisions: bool) -> str:
        """One row of chips per joint: by collision check, or grey to record and blue recorded.

        By measurement, a take left out of Solve is amber, and the one recording busy.
        """
        colors = ({"unchecked": NONE, "clear": OK, "collides": FAIL, "recorded": INFO, "out of date": NONE}
                  if by_collisions else
                  {"unchecked": NONE, "clear": NONE, "collides": NONE, "recorded": INFO, "out of date": NONE})
        rows = ""
        for joint in (0, 1):
            chips = ""
            for index in (i for i in indices if run.sweeps.sweeps[i].joint == joint):
                state = take_state(run, index)
                detail, color = self._hover(run, index, state, by_collisions), colors[state]
                if state == "recorded" and not run.takes[index].good:
                    color = BUSY
                if self.in_progress is not None and self.in_progress[0] == index:
                    color, detail = BUSY, "recording"
                chips += chip(f"{index + 1} J{joint}", color, detail)
            rows += chip_row("takes" if not rows else "", chips, wrap=True) if chips else ""
        return rows

    @staticmethod
    def _hover(run: Run, index: int, state: str, by_collisions: bool) -> str:
        """A take chip's hover text: what its state means, and what to do."""
        if state == "recorded":
            recorded = run.takes[index]
            reason = left_out_reason(recorded.circle)
            if reason is not None:
                return f"left out of Solve: {reason} ({LEFT_OUT_LIMITS}): record it again"
            return f"recorded, Solve uses it: radius {recorded.circle.radius * 1e3:.0f} mm"
        if state == "out of date":
            return "recorded with another tool body or before a base refine: record it again"
        if state == "collides":
            # * The planner's reason is logged in full when Check collisions finds it.
            return hover(f"collides where the base stands, skipped: {run.collisions[index]}")
        if state == "unchecked":
            return "not checked for collisions yet: Check collisions in Sweeps"
        return "clear where the base stands: Record can take it" if by_collisions else "to record: pick it in Take"

    def marker_chips(self, run: Run, indices: Iterable[int]) -> str:
        """A row with a chip per measured take: its samples' largest marker error, the worst sample's on hover."""
        chips = ""
        for index in indices:
            errors = self._marker_errors(run, index)
            if errors:
                chips += marker_chip(str(index + 1), errors)
        return chip_row("markers", chips)

    def _marker_errors(self, run: Run, index: int) -> list[dict | None]:
        """The marker errors of a take's measured points, in their order; none for a take not measured."""
        if take_state(run, index) == "recorded":
            return list(run.takes[index].marker_errors)
        if self.in_progress is not None and self.in_progress[0] == index:
            return [{"base": sample.base_error, "tool": sample.tool_error} for sample in self.in_progress[1]]
        return []

    def _compute_arcs(self, run: Run) -> None:
        """Each take's tool0 path in base_footprint, densified so a sweep draws as an arc."""
        arcs = []
        for sweep in run.sweeps.sweeps:
            waypoints = sweep.waypoints()
            dense = [a + (b - a) * k / ARC_SUBSTEPS for a, b in zip(waypoints[:-1], waypoints[1:])
                     for k in range(ARC_SUBSTEPS)] + [waypoints[-1]]
            arcs.append(np.array([link_pose(run.urdf, Pose(), named(run.arm, q), f"{run.arm}_tool0").position
                                  for q in dense]))
        self._run, self._arcs, self._drawn = run, arcs, {}

    def _points(self, run: Run, index: int) -> tuple[np.ndarray, np.ndarray]:
        """A take's points in base_footprint: waypoints still to measure (tool0), and measured ones (the tool body)."""
        waypoints = self._arcs[index][::ARC_SUBSTEPS]
        if take_state(run, index) == "recorded":
            recorded = run.takes[index]
            # * Measured in the base rigid body; base_footprint is the body with the applied calibration.
            to_footprint = invert(recorded.calibration)
            measured = [compose(to_footprint, Pose(tuple(point))).position for point in recorded.take.points]
            return waypoints[:0], np.array(measured)
        if self.in_progress is not None and self.in_progress[0] == index:
            samples = list(self.in_progress[1])
            measured = [compose(invert(sample.footprint), sample.tool).position for sample in samples]
            ahead = waypoints[::-1] if self.backwards else waypoints
            return ahead[len(samples):], np.array(measured).reshape(-1, 3)
        return waypoints, waypoints[:0]

    def hide(self, ctx: PluginContext) -> None:
        """Hide every take."""
        if self._frame is not None and self._shown != (False,):
            self._frame.visible, self._shown = False, (False,)

    def draw(self, ctx: PluginContext, run: Run, by_collisions: bool) -> None:
        """Draw each take's tool0 path at the robot and its points, coloured by its state.

        Only a take whose state or points changed is drawn again, so the others never flicker.
        """
        root = f"{ctx.view.scene_root}/takes"
        if run is not self._run:
            # * Another run: drop the old takes' nodes with their frame.
            if self._frame is not None:
                self._frame.remove()
                self._frame = self._shown = None
            self._compute_arcs(run)
        if self._frame is None:
            self._frame = ctx.view.scene.add_frame(root, show_axes=False, visible=False)
        self._shown = place_on(self._frame, ctx.kinematics.base_pose(run.serial), self._shown)
        spheres = None
        for index, arc in enumerate(self._arcs):
            state = take_state(run, index)
            progress = len(self.in_progress[1]) if self.in_progress and self.in_progress[0] == index else None
            key = (state, progress, id(run.takes.get(index)), by_collisions)
            if self._drawn.get(index) == key:
                continue
            self._drawn[index] = key
            # * Sweeps colours by the collision check; Record keeps colour for what was measured: grey planned, blue
            # * sampled, and a colliding take, which cannot be recorded, not drawn.
            color = ARC_COLORS.get(state, ARC_COLORS["clear"]) if by_collisions else ARC_COLORS[
                "recorded" if state == "recorded" else "unchecked"]
            segments = np.stack([arc[:-1], arc[1:]], axis=1)
            ctx.view.scene.add_line_segments(f"{root}/{index}", segments, color, thickness=0.003,
                                             visible=by_collisions or state != "collides")
            spheres = spheres or [trimesh.creation.icosphere(subdivisions=1, radius=r)
                                  for r in (POINT_RADIUS, MEASURED_RADIUS)]
            # * Waypoints still to measure are small and faint; measured points larger and solid, filled in live.
            for name, points, sphere, opacity in zip(("pending", "measured"), self._points(run, index), spheres,
                                                     (POINT_OPACITY, MEASURED_OPACITY)):
                if not len(points) or (not by_collisions and state == "collides"):
                    ctx.view.scene.add_frame(f"{root}/{index}_{name}", show_axes=False, visible=False)
                    continue
                if name == "pending":
                    colors = color
                else:
                    levels = [marker_error_level(sample_marker_error(error)) for error in
                              self._marker_errors(run, index)]
                    levels += [None] * (len(points) - len(levels))
                    colors = np.array([MARKER_RED if level == BAD else ARC_COLORS["recorded"] for level in
                                       levels[:len(points)]])
                ctx.view.scene.add_batched_meshes_simple(
                    f"{root}/{index}_{name}", sphere.vertices, sphere.faces,
                    batched_wxyzs=np.tile((1.0, 0.0, 0.0, 0.0), (len(points), 1)), batched_positions=points,
                    batched_colors=colors,
                    batched_opacities=np.full(len(points), opacity), cast_shadow=False, receive_shadow=False)
