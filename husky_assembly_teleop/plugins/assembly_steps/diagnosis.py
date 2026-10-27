"""
The step's diagnostic drawings in the 3D view: the tool0 check, and the tool0 path of an insert or retreat plan.

Rebuilt on each call under "<plugin root>/diagnosis"; shown while the panel is in use (`RecentUse`), like the ghosts.
Main thread only.
"""

from __future__ import annotations

import math
from typing import Mapping, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from ...plugin_api.context import PluginContext
from ...shared.assembly_planning.route_result import pose_error
from ...shared.execution.servo import flange_side
from ...ui.scene_view import add_labelled_frame

#: Axes lengths, metres: the authored target, the live tool0, a plan's start and end, a waypoint along the path.
TARGET_AXES, LIVE_AXES, PLAN_AXES, WAYPOINT_AXES = 0.15, 0.08, 0.06, 0.02
#: Colours, RGB 0-255: the authored (Rhino) frames, the live or planned frames, the gap lines, the tool0 path.
RHINO_COLOR, LIVE_COLOR, GAP_COLOR, PATH_COLOR = (40, 40, 220), (220, 40, 40), (0, 0, 0), (0, 180, 0)
#: Small axes on about this many waypoints of a path.
WAYPOINT_AXES_COUNT = 10


class StepDiagnosis:
    """The diagnostic nodes of the current step."""

    def __init__(self, ctx: PluginContext):
        """Nothing drawn yet."""
        self._scene, self._root_path = ctx.view.scene, f"{ctx.view.scene_root}/diagnosis"
        self._roots: dict[str, object] = {}
        self._visible = True

    def _root(self, what: str):
        """A fresh frame node for drawing `what` ("tool0" or "path"), the old one removed."""
        self.clear(what)
        root = self._scene.add_frame(f"{self._root_path}/{what}", show_axes=False, visible=self._visible)
        self._roots[what] = root
        return root

    def show_tool0(self, live: Mapping[str, Pose], targets: Mapping[str, Pose]) -> None:
        """Per flange: the authored target frame (blue), the live frame (red), and a line between their origins.

        Args:
            live: Flange id -> its live world pose (mocap base + FK).
            targets: Flange id -> its authored target.
        """
        root = self._root("tool0")
        for index, (flange, target) in enumerate(targets.items()):
            side = flange_side(flange)
            add_labelled_frame(self._scene, f"{root.name}/{index}/target", target, f"{side} target (Rhino)",
                               TARGET_AXES, RHINO_COLOR)
            add_labelled_frame(self._scene, f"{root.name}/{index}/live", live[flange], f"{side} live (mocap + FK)",
                               LIVE_AXES, LIVE_COLOR)
            self._scene.add_line_segments(f"{root.name}/{index}/gap", points=np.array(
                [[target.position, live[flange].position]], dtype=np.float32), colors=GAP_COLOR, thickness=2.0,
                thickness_units="screen")

    def show_path(self, paths: Mapping[str, Sequence[Pose]], rhino_start: Mapping[str, Pose],
                  rhino_target: Mapping[str, Pose]) -> None:
        """Per arm: the Rhino start and target frames, the plan's start and end frames, gap lines, the tool0 path.

        Args:
            paths: Arm name -> its tool0 at every row of the plan (`tool0_world_path`).
            rhino_start: Arm name -> the authored pose the movement starts from.
            rhino_target: Arm name -> the authored pose it ends at.
        """
        root = self._root("path")
        for arm, path in paths.items():
            if not path:
                continue
            base = f"{root.name}/{arm}"
            for what, rhino, planned in (("start", rhino_start.get(arm), path[0]),
                                         ("end", rhino_target.get(arm), path[-1])):
                add_labelled_frame(self._scene, f"{base}/plan_{what}", planned, f"{arm} plan {what}", PLAN_AXES,
                                   LIVE_COLOR)
                if rhino is not None:
                    add_labelled_frame(self._scene, f"{base}/rhino_{what}", rhino, f"{arm} Rhino {what}", TARGET_AXES
                                       if what == "end" else TARGET_AXES * 0.8, RHINO_COLOR)
                    self._scene.add_line_segments(f"{base}/gap_{what}", points=np.array(
                        [[rhino.position, planned.position]], dtype=np.float32), colors=GAP_COLOR, thickness=1.0,
                        thickness_units="screen")
            points = np.array([pose.position for pose in path], dtype=np.float32)
            if len(points) > 1:
                self._scene.add_line_segments(f"{base}/path", points=np.stack([points[:-1], points[1:]], axis=1),
                                              colors=PATH_COLOR, thickness=3.0, thickness_units="screen")
            every = max(1, len(path) // WAYPOINT_AXES_COUNT)
            for index in range(0, len(path), every):
                pose = path[index]
                self._scene.add_frame(f"{base}/waypoint_{index}", position=tuple(float(v) for v in pose.position),
                                      wxyz=_wxyz(pose), axes_length=WAYPOINT_AXES, axes_radius=WAYPOINT_AXES / 20.0,
                                      show_axes=True)

    def clear(self, what: str | None = None) -> None:
        """Remove the drawing of `what`, or every drawing."""
        for key in [what] if what is not None else list(self._roots):
            root = self._roots.pop(key, None)
            if root is not None:
                root.remove()

    @property
    def visible(self) -> bool:
        """bool: Whether the drawings show."""
        return self._visible

    @visible.setter
    def visible(self, value: bool) -> None:
        if value != self._visible:
            self._visible = value
            for root in self._roots.values():
                root.visible = value


def tool0_line(errors: Mapping[str, dict]) -> str:
    """One line of the tool0 check: "tool0 live minus target: left 1.23 mm (dx/dy/dz) axes x/y/z deg | right …".

    Args:
        errors: Side -> `tool0_errors`' entry.
    """
    parts = []
    for side, error in errors.items():
        dx, dy, dz = error["pos_err_mm"]
        ax, ay, az = error["rot_err_deg"]
        parts.append(f"{side} {error['pos_norm_mm']:.2f} mm ({dx:+.2f}/{dy:+.2f}/{dz:+.2f}) "
                     f"axes {ax:.2f}/{ay:.2f}/{az:.2f}°")
    return "tool0 live minus target: " + " | ".join(parts)


def path_line(movement: str, paths: Mapping[str, Sequence[Pose]], rhino_start: Mapping[str, Pose],
              rhino_target: Mapping[str, Pose]) -> str:
    """One line on a plan's tool0 path: how far its start and end sit from the Rhino start and target, per arm."""
    parts = []
    for arm, path in paths.items():
        if not path:
            continue
        gaps = []
        ends = (("start", rhino_start.get(arm), path[0]), ("end", rhino_target.get(arm), path[-1]))
        for what, rhino, planned in ends:
            if rhino is not None:
                distance, turn = pose_error(planned, rhino)
                gaps.append(f"Rhino {what}→plan {what} {distance * 1e3:.2f} mm/{math.degrees(turn):.3f}°")
        parts.append(f"{arm}: " + ", ".join(gaps))
    count = max((len(path) for path in paths.values()), default=0)
    return f"[visual diagnosis] {movement} " + " | ".join(parts) + f"; {count} waypoints"


def _wxyz(pose: Pose) -> tuple[float, float, float, float]:
    """A pose's quaternion as viser wants it, (w, x, y, z)."""
    x, y, z, w = (float(v) for v in pose.orientation)
    return w, x, y, z
