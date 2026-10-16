"""
Bar marker fitting: how far a bar the mocap sees is from where the process plans it.

Pick a bar of the loaded process. On every mocap frame its CAD markers are fitted to the raw markers near where the
bar is expected (`world.markers`, `world/marker_fit.fit_near`), without Motive's labels or rigid bodies: near the
last fit, else near the planned bar. The planned bar is the process body in the live scene (`process/bars/<id>`): a
built bar at its assembly pose, a held one on the real robot's flange.
The view shows the real bar (blue, the process bar and its mounted halves at the fitted pose) next to the planned
one, and with "Show markers" every raw marker (green), the fitted markers (blue: markers, axis, a line per residual)
and the planned markers (grey). Record writes a take of the fit, its residuals and the deviation to the Drive
(`data_experiment/mocap_bars/`).

- ! Marker layouts come only from the process (`BodySpec.markers` of the halves mounted on a bar); old compas_fab
  exports converted on load have none.
- ! Markers of no Motive asset reach the cloud only with "Unlabeled Markers" on in Motive's streaming settings.
- * The real bar and its halves are drawn only, in this plugin's view: not scene bodies, so they never collide (the
  planned ones already do), and hidden while there is no fit.

Run with:  -p plugins:="['mocap_bars']"  (loads `process`)
"""

from __future__ import annotations

import asyncio
import json
from html import escape
from typing import TYPE_CHECKING

import numpy as np
import viser

from ..drive import drive_folder, folder_name, stamped_name
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..plugin_api.trace import Signal
from ..ui.quaternion import quaternion_to_wxyz
from ..ui.scene_view import add_shape
from ..ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_SENSOR, block, chip, note, section, values
from ..world.checks import STALE_AFTER
from ..world.marker_fit import BarDeviation, MarkerFit, bar_deviation, bar_markers, fit_near
from ..robot_interface.mocap_state import MarkerCloud
from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from ..shared.process_contract import ProcessContract

#: Takes go into a folder per experiment under this one, below the Drive root; create it in Drive first.
RECORDING_FOLDER = "data_experiment/mocap_bars"
#: Bar dropdown entry while the process has no bar with markers.
NO_BAR = "–"
#: Colours: raw markers, the fitted bar, the planned markers.
MEASURED_COLOR = (40, 190, 80)
FIT_COLOR = (40, 120, 230)
PLANNED_COLOR = (150, 150, 150)
#: The real bar's colour and its mounted halves', (r, g, b) from 0 to 1.
FIT_COLOR_UNIT = tuple(c / 255 for c in FIT_COLOR)
HALF_COLOR_UNIT = (0.1, 0.27, 0.6)
#: Size of the drawn markers, metres.
MARKER_POINT_SIZE = 0.02


def fit_first(model: np.ndarray, points: np.ndarray, priors: list[tuple[str, np.ndarray]],
              radius: float) -> tuple[MarkerFit, str] | None:
    """The fit near the first prior that gives one, and that prior's name; None if none does. Any thread.

    Args:
        model: The bar's CAD markers in its frame, shape (m, 3).
        points: Every raw marker, shape (n, 3).
        priors: (name, 4x4 bar pose) to search near, in order.
        radius: Search radius, metres.
    """
    for name, prior in priors:
        fit = fit_near(model, points, prior, radius=radius)
        if fit is not None:
            return fit, name
    return None


@register
class MocapBarsPlugin(HuskyPlugin):
    """Fits a process bar's markers to the raw mocap markers near it and measures its deviation from the plan."""

    name = "mocap_bars"
    requires = ("process",)
    experimental = True

    def __init__(self):
        """Start with nothing fitted."""
        #: Bar id -> (marker names, CAD markers in the bar frame), from the loaded process.
        self._layouts: dict[str, tuple[tuple[str, ...], np.ndarray]] = {}
        #: The process object the layouts were read from.
        self._layouts_of = None
        self._bar_id: str | None = None
        #: Latest fit, the cloud it was made from and what placed its search, and the planned bar pose this tick.
        self._fit: MarkerFit | None = None
        self._fitted_cloud: MarkerCloud | None = None
        self._fitted_near = ""
        self._fitting: asyncio.Task | None = None
        self._planned: Pose | None = None
        self._deviation: BarDeviation | None = None
        #: What the scene last showed, so marker nodes are only resent when they change.
        self._drawn: tuple | None = None
        #: Nodes of the real bar and its halves, and the bar and process they were built from.
        self._bar_nodes: list[viser.SceneNodeHandle] = []
        self._bar_built: tuple | None = None
        self._take: asyncio.Task | None = None
        self._message = ""

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel and the marker nodes, and find the Drive folder for takes.

        Args:
            ctx: This plugin's context.
        """
        try:
            self._folder, problem = drive_folder(ctx.config.drive_root, RECORDING_FOLDER), ""
        except FileNotFoundError as error:
            self._folder, problem = None, str(error)
            ctx.log_warn(f"takes cannot be recorded: {problem}")
        with ctx.view.ui() as gui:
            gui.add_html(section("bar", SECTION_SENSOR))
            self._bar = gui.add_dropdown("Bar", options=[NO_BAR],
                                         hint="Bars of the loaded process with 3 or more markers on their halves.")
            self._radius = gui.add_slider("Search (cm)", min=2, max=30, step=1, initial_value=8,
                                          hint="Markers farther than this from where the bar is expected are "
                                               "ignored. Raise it when the bar is far from the plan.")
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            self._show_markers = gui.add_checkbox("Show markers", initial_value=True)

            gui.add_html(section("record", SECTION_CTRL))
            self._experiment = gui.add_text("Experiment", initial_value="bar_accuracy",
                                            hint=f"Takes go to {RECORDING_FOLDER}/<experiment>/ on the Drive.")
            self._seconds = gui.add_slider("Seconds", min=1, max=60, step=1, initial_value=5)
            record = gui.add_button("Record take", disabled=bool(problem),
                                    hint=problem or "Record the fit and its deviation from the plan, once per tick.")
            self._result: viser.GuiHtmlHandle = gui.add_html("")

        scene, root = ctx.view.scene, ctx.view.scene_root
        self._bar_frame = scene.add_frame(f"{root}/real_bar", show_axes=False, visible=False)
        # * Always float32: the default float16 rounds world positions to millimetres.
        self._measured_cloud = scene.add_point_cloud(f"{root}/measured", points=np.zeros((0, 3)), colors=MEASURED_COLOR,
                                                     point_size=MARKER_POINT_SIZE, point_shape="circle",
                                                     precision="float32", visible=False)
        self._fit_cloud = scene.add_point_cloud(f"{root}/fit/markers", points=np.zeros((0, 3)), colors=FIT_COLOR,
                                                point_size=MARKER_POINT_SIZE, point_shape="circle",
                                                precision="float32", visible=False)
        self._fit_lines = scene.add_line_segments(f"{root}/fit/lines", points=np.zeros((1, 2, 3)), colors=FIT_COLOR,
                                                  thickness=0.003, visible=False)
        self._planned_cloud = scene.add_point_cloud(f"{root}/planned", points=np.zeros((0, 3)), colors=PLANNED_COLOR,
                                                    point_size=MARKER_POINT_SIZE, point_shape="circle",
                                                    precision="float32", visible=False)

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        self._bar.on_update(ctx.defer_value("choose bar", self._choose))
        record.on_click(ctx.defer("record take", lambda: self._start_take(ctx)))

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _choose(self, bar_id: str) -> None:
        """Fit this bar from now on."""
        self._bar_id = bar_id if bar_id in self._layouts else None
        self._fit, self._fitted_cloud = None, None

    def _start_take(self, ctx: PluginContext) -> None:
        """Start recording a take, unless one runs or no bar is chosen."""
        if self._take is not None and not self._take.done():
            self._message = "a take is already recording"
        elif self._bar_id is None:
            self._message = "choose a bar first"
        else:
            self._take = ctx.spawn("record take", self._record_take(ctx))

    async def _record_take(self, ctx: PluginContext) -> None:
        """Record the fit for the chosen seconds, then save it with the bar's layout and the plan's context."""
        bar_id, seconds = self._bar_id, float(self._seconds.value)
        names, model = self._layouts[bar_id]
        experiment = folder_name(self._experiment.value, "take")
        self._message = f"recording {bar_id} for {seconds:g} s"
        with ctx.record(*self._signals(model)) as take:
            await ctx.sleep(seconds)
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        description = {"bar": bar_id, "search_radius": self._radius.value / 100,
                       "process": str(selection.process.folder) if selection is not None else None,
                       "step": selection.movement.id if selection is not None else None, "seconds": seconds,
                       "frame": "world (mocap, Z-up), metres; quaternions (x, y, z, w); deviation in the planned "
                                "bar's frame, its axis z"}
        path = self._folder / experiment / f"{stamped_name(bar_id.split('/')[-1])}.npz"
        extra = {"model_markers": model, "model_marker_names": np.array(names),
                 "description": np.array(json.dumps(description))}
        await ctx.run_in_thread(take.save, path, extra)
        fitted = int(np.isfinite(take.values("fit_rms")).sum())
        self._message = f"saved {len(take)} samples ({fitted} fitted) to {path}"
        ctx.log_info(self._message)

    def _signals(self, model: np.ndarray) -> list[Signal]:
        """What a take records each tick: the fit, its quality, the planned pose and the deviation."""
        def fit() -> MarkerFit | None:
            return self._fit

        def planned() -> Pose | None:
            return self._planned

        def deviation() -> BarDeviation | None:
            return self._deviation

        return [
            Signal("fit_position", lambda: fit().pose.position if fit() else None, ("x", "y", "z"), "m"),
            Signal("fit_orientation", lambda: fit().pose.orientation if fit() else None, ("qx", "qy", "qz", "qw")),
            Signal("fit_rms", lambda: fit().rms if fit() else None, (), "m"),
            Signal("fit_max", lambda: float(fit().residuals.max()) if fit() else None, (), "m"),
            Signal("fit_count", lambda: len(fit().residuals) if fit() else None),
            Signal("planned_position", lambda: planned().position if planned() else None, ("x", "y", "z"), "m"),
            Signal("planned_orientation", lambda: planned().orientation if planned() else None,
                   ("qx", "qy", "qz", "qw")),
            Signal("deviation", lambda: (deviation().across, deviation().along) if deviation() else None,
                   ("across", "along"), "m"),
            Signal("tilt", lambda: deviation().tilt if deviation() else None, (), "rad"),
            Signal("matched_markers", lambda: self._matched(len(model)), tuple(
                f"{index}.{axis}" for index in range(len(model)) for axis in "xyz"), "m"),
        ]

    def _matched(self, count: int) -> np.ndarray | None:
        """The measured marker matched to each CAD marker, flattened, NaN where none was; None without a fit."""
        fit = self._fit
        if fit is None:
            return None
        rows = np.full((count, 3), np.nan)
        rows[fit.model_index] = fit.points
        return rows.reshape(-1)

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Follow the loaded process, refit on each new marker cloud, and compare with the planned bar.

        Args:
            ctx: This plugin's context.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of = loaded
            self._layouts = bar_markers(loaded) if loaded is not None else {}
            self._bar.options = list(self._layouts) or [NO_BAR]
            self._bar_id = next(iter(self._layouts), None)
            self._bar.value = self._bar_id or NO_BAR
            self._fit, self._fitted_cloud = None, None
        ids = selection.ids if selection is not None else None
        live_id = ids.get(self._bar_id) if ids is not None and self._bar_id else None
        self._planned = ctx.scene.snapshot.world_poses.get(live_id) if live_id else None
        cloud = ctx.world.markers
        if self._bar_id is None or cloud is None:
            self._fit = None
        elif cloud is not self._fitted_cloud and (self._fitting is None or self._fitting.done()):
            # * On a worker thread: a crowded cloud takes tens of milliseconds. Clouds arriving meanwhile are skipped.
            self._fitted_cloud = cloud
            self._fitting = ctx.spawn("fit bar", self._refit(ctx, self._bar_id, cloud.positions))
        self._deviation = (bar_deviation(self._fit.pose, self._planned)
                           if self._fit is not None and self._planned is not None else None)

    async def _refit(self, ctx: PluginContext, bar_id: str, points: np.ndarray) -> None:
        """Fit the bar to the markers near its last fit, else near the planned bar."""
        priors = [("last fit", self._fit.world_from_model)] if self._fit is not None else []
        if self._planned is not None:
            priors.append(("plan", self._planned.matrix()))
        found = await ctx.run_in_thread(fit_first, self._layouts[bar_id][1], points, priors, self._radius.value / 100)
        # ! Another bar may have been chosen meanwhile.
        if bar_id == self._bar_id:
            self._fit, self._fitted_near = found if found is not None else (None, "")

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the marker stream, the fit and the deviation, and move the bar and the marker nodes.

        Args:
            ctx: This plugin's context.
        """
        cloud = ctx.world.markers
        if cloud is None:
            state = chip("no markers", NONE, "nothing on /mocap/markers yet: is the mocap relay running?")
        elif ctx.now() - cloud.arrival_time > STALE_AFTER:
            state = chip("markers stale", FAIL, f"no marker cloud for {ctx.now() - cloud.arrival_time:.1f} s")
        else:
            state = chip(f"{len(cloud.positions)} markers", OK)
        self._status.content = block(state + values(*self._status_lines()))
        running = self._take is not None and not self._take.done()
        self._result.content = (chip("recording", BUSY) if running else "") + (
            note(escape(self._message)) if self._message else "")
        self._draw_bar(ctx)
        self._draw_markers(cloud if self._show_markers.value else None)

    def _status_lines(self) -> list[str]:
        """The fit's error and the deviation from the plan, one per line."""
        if not self._layouts:
            return ["no bar with markers in the loaded process"]
        fit, bar_id = self._fit, self._bar_id
        layout = self._layouts.get(bar_id, ((), np.zeros((0, 3))))[1]
        if fit is None and self._planned is None:
            return [f"{bar_id}: no fit, and no planned pose to search near: is the bar in the selected step?"]
        if fit is None:
            return [f"{bar_id}: no fit; needs 3+ of its {len(layout)} markers within the search of the plan"]
        lines = [f"fit {len(fit.residuals)}/{len(layout)} near {self._fitted_near}  rms {fit.rms * 1e3:.2f}  max "
                 f"{fit.residuals.max() * 1e3:.2f} mm"]
        deviation = self._deviation
        lines.append("no planned pose: is the bar in the selected step?" if deviation is None else
                     f"across {deviation.across * 1e3:.1f}  along {deviation.along * 1e3:+.1f} mm  "
                     f"tilt {np.degrees(deviation.tilt):.2f}°")
        return lines

    def _draw_bar(self, ctx: PluginContext) -> None:
        """Show the chosen bar and its mounted halves at the fitted pose, rebuilt when the bar or process changes."""
        process = self._layouts_of
        if self._bar_built is None or self._bar_built[0] != self._bar_id or self._bar_built[1] is not process:
            self._bar_built = (self._bar_id, process)
            for node in self._bar_nodes:
                node.remove()
            self._bar_nodes = []
            bar = process.bodies.get(self._bar_id) if process is not None and self._bar_id else None
            if bar is not None:
                scene, root = ctx.view.scene, self._bar_frame.name
                self._bar_nodes = [add_shape(scene, f"{root}/{index}", shape, FIT_COLOR_UNIT)
                                   for index, shape in enumerate(bar.geometry.visual)]
                # * Mounted halves stay where they sit on their bar, so the assembly poses give the offset.
                for half_id in sorted(process.bodies):
                    half = process.bodies[half_id]
                    if half.mount != self._bar_id:
                        continue
                    offset = compose(invert(bar.assembly_pose), half.assembly_pose)
                    frame = scene.add_frame(f"{root}/{half_id.replace('/', '_')}", show_axes=False,
                                            position=offset.position, wxyz=quaternion_to_wxyz(offset.orientation))
                    for index, shape in enumerate(half.geometry.visual):
                        add_shape(scene, f"{frame.name}/{index}", shape, HALF_COLOR_UNIT)
                    self._bar_nodes.append(frame)
        fit = self._fit
        self._bar_frame.visible = fit is not None and bool(self._bar_nodes)
        if self._bar_frame.visible:
            pose = fit.pose
            self._bar_frame.position = pose.position
            self._bar_frame.wxyz = quaternion_to_wxyz(pose.orientation)

    def _draw_markers(self, cloud: MarkerCloud | None) -> None:
        """Show the raw markers, the fitted and the planned bar's markers, resending them only when they changed.

        Args:
            cloud: The markers to show, or None to hide them all.
        """
        # ? Identity, not id(): a freed sample's id can be reused by the next one.
        shown = (cloud, self._fit, self._planned, self._bar_id)
        if self._drawn is not None and all(a is b for a, b in zip(self._drawn, shown)):
            return
        self._drawn = shown
        self._measured_cloud.visible = cloud is not None and len(cloud.positions) > 0
        if self._measured_cloud.visible:
            self._measured_cloud.points = cloud.positions.astype(np.float32)
        layout = self._layouts[self._bar_id][1] if self._bar_id in self._layouts else None
        self._planned_cloud.visible = layout is not None and self._planned is not None and cloud is not None
        if self._planned_cloud.visible:
            matrix = self._planned.matrix()
            self._planned_cloud.points = (layout @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32)
        fit = self._fit
        self._fit_cloud.visible = self._fit_lines.visible = fit is not None and cloud is not None
        if not self._fit_cloud.visible:
            return
        self._fit_cloud.points = fit.transform(layout).astype(np.float32)
        # * The bar axis over the markers' span, then one line from each matched marker to its fitted CAD marker.
        span = layout[:, 2].min(), layout[:, 2].max()
        axis = fit.transform(np.array([[0.0, 0.0, span[0]], [0.0, 0.0, span[1]]]))
        residuals = np.stack([fit.points, fit.transform(layout[fit.model_index])], axis=1)
        self._fit_lines.points = np.concatenate([axis[None], residuals]).astype(np.float32)
        self._fit_lines.colors = np.full((len(residuals) + 1, 2, 3), FIT_COLOR, dtype=np.uint8)
