"""
Bar marker fitting: how far a bar the mocap sees is from where the process plans it.

Pick a bar of the loaded process and the Motive rigid body made of its markers, and press Track. The bar's CAD
markers are fitted to the measured ones without Motive's labels (`world/marker_layouts.fit_bar`, the old monitor's
matcher) at most ten times a second: matched around the last good fit while it is fresh, else around the planned
pose, else by a full search. The view shows the real bar (blue, the process bar and its mounted halves at the fitted
pose) next to the planned one, and with "Show markers" the measured markers (green the trusted ones the fit used,
orange filled in by Motive), the fitted markers (blue: markers, axis, a line per residual) and the planned markers
(grey). The planned bar is the process body in the live scene (`process/bars/<id>`): a built bar at its assembly
pose, a held one on the real robot's flange. Record writes a take of the fit, its residuals and the deviation to the
Drive (`data_experiment/mocap_bars/`): a run folder per take with experiment.json and recording.npz
(`shared/recordings/runs.py`).

- ! Marker layouts come only from the process (`BodySpec.markers` of the halves mounted on a bar); old compas_fab
  exports converted on load have none.
- ! The fit runs on the main thread: a full search takes ~20 ms, the match around a hint ~1 ms.
- * Markers dropped for their size or Motive residual are neither fitted nor drawn; a failed fit's reason counts them.
- * The real bar and its halves are drawn only, in this plugin's view: not scene bodies, so they never collide (the
  planned ones already do), and hidden while there is no fit.

Run with:  -p plugins:="['mocap_bars']"  (loads `process`)
"""

from __future__ import annotations

import asyncio
from html import escape
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import viser

from ..drive import drive_folder, folder_name
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..plugin_api.trace import Signal
from ..robot_interface.mocap_state import RawMarkers
from ..shared.recordings.runs import run_folder, save_recording, write_description
from ..ui.quaternion import quaternion_to_wxyz
from ..ui.scene_view import add_shape
from ..ui.style import BUSY, NONE, SECTION_CTRL, SECTION_SENSOR, block, check_chip, chip, note, section, values
from ..world.bar_fit import BarFit
from ..world.bar_pose import transform_points
from ..world.marker_fit import BarDeviation, bar_deviation
from ..world.marker_layouts import BarLayout, Sensed, bar_layouts, fit_bar, sensed_from_raw
from ..world.measured import TrackedObject
from ..world.mocap import mocap_check
from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from ..shared.process_contract import ProcessContract

#: Name of the tracked rigid body in `world.tracked_objects` and on the health panel.
TRACKED_NAME = "fitted_bar"
#: Takes go into a folder per experiment under this one, below the Drive root; create it in Drive first.
RECORDING_FOLDER = "data_experiment/mocap_bars"
#: Bar dropdown entry while the process has no bar with markers.
NO_BAR = "–"
#: Fits run at most this often, seconds (the old "follow live (10 Hz)").
FIT_PERIOD = 0.1
#: A sample older than this is not fitted, seconds: is Motive streaming?
MAX_MARKER_AGE = 0.5
#: The last good fit guides the next one while it is younger than this, seconds; after that the planned pose does.
HINT_MAX_AGE = 1.0
#: Colours: measured markers, markers filled in by Motive, the fitted bar, the planned markers.
MEASURED_COLOR = (40, 190, 80)
FILLED_COLOR = (250, 140, 30)
FIT_COLOR = (40, 120, 230)
PLANNED_COLOR = (150, 150, 150)
#: The real bar's colour and its mounted halves', (r, g, b) from 0 to 1.
FIT_COLOR_UNIT = tuple(c / 255 for c in FIT_COLOR)
HALF_COLOR_UNIT = (0.1, 0.27, 0.6)
#: Size of the drawn markers, metres.
MARKER_POINT_SIZE = 0.02


def matched_pairs(fit: BarFit) -> np.ndarray:
    """The fit's matches as an (n, 2) int array of (sensed index, design index)."""
    return np.array(list(fit.correspondences.items()), dtype=int).reshape(-1, 2)


@register
class MocapBarsPlugin(HuskyPlugin):
    """Fits a process bar's markers to a tracked Motive rigid body and measures its deviation from the plan."""

    name = "mocap_bars"
    icon = viser.Icon.RULER_2
    requires = ("process",)
    experimental = True

    def __init__(self):
        """Start with nothing tracked or fitted."""
        self._object: TrackedObject | None = None
        #: Bar id -> its marker layout, from the loaded process.
        self._layouts: dict[str, BarLayout] = {}
        #: The process object the layouts were read from.
        self._layouts_of = None
        self._bar_id: str | None = None
        #: The sample last fitted and its mocap message time; the fit and its pose when it succeeded, else why not.
        self._sensed: Sensed | None = None
        self._fitted_update: float | None = None
        self._fit: BarFit | None = None
        self._pose: Pose | None = None
        self._reason = ""
        #: When the next fit may run, and the last good fit's (time, 4x4 pose): the hint the next fit starts from.
        self._next_fit_at = 0.0
        self._last_ok: tuple[float, np.ndarray] | None = None
        #: The planned bar pose this tick, and the fit's deviation from it.
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
            self._mocap_id = gui.add_number("Mocap ID", initial_value=0, min=0, step=1,
                                            hint="The Motive rigid body made of this bar's markers.")
            track = gui.add_button("Track")
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
        track.on_click(ctx.defer("track bar", lambda: self._track(ctx)))
        record.on_click(ctx.defer("record take", lambda: self._start_take(ctx)))

    def teardown(self, ctx: PluginContext) -> None:
        """Stop tracking the rigid body.

        Args:
            ctx: This plugin's context.
        """
        ctx.untrack_object(TRACKED_NAME)

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _choose(self, bar_id: str) -> None:
        """Fit this bar from now on."""
        self._bar_id = bar_id if bar_id in self._layouts else None
        self._forget_fit()

    def _track(self, ctx: PluginContext) -> None:
        """Track the rigid body in "Mocap ID", replacing the previous one."""
        mocap_id = int(self._mocap_id.value)
        ctx.untrack_object(TRACKED_NAME)
        # * No geometry: the process draws the bar; this is a frame and its markers.
        self._object = ctx.track_object(TRACKED_NAME, mocap_id, label=f"bar markers ({mocap_id})")
        self._forget_fit()
        self._message = f"tracking mocap id {mocap_id}"
        ctx.log_info(self._message)

    def _forget_fit(self) -> None:
        """Drop the fit and its hint: a new bar, rigid body or process starts from a full search."""
        self._sensed = self._fitted_update = self._fit = self._pose = self._last_ok = None
        self._reason = ""

    def _start_take(self, ctx: PluginContext) -> None:
        """Start recording a take, unless one runs or nothing is fitted yet."""
        if self._take is not None and not self._take.done():
            self._message = "a take is already recording"
        elif self._bar_id is None or self._object is None:
            self._message = "choose a bar and track its rigid body first"
        else:
            self._take = ctx.spawn("record take", self._record_take(ctx), survives_soft_stop=True)

    async def _record_take(self, ctx: PluginContext) -> None:
        """Record the fit for the chosen seconds, then save it with the bar's layout and the plan's context."""
        bar_id, seconds = self._bar_id, float(self._seconds.value)
        layout = self._layouts[bar_id]
        experiment = folder_name(self._experiment.value, "take")
        self._message = f"recording {bar_id} for {seconds:g} s"
        with ctx.record(*self._signals(layout.points)) as take:
            await ctx.sleep(seconds)
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        description = {"bar": bar_id, "mocap_id": self._object.mocap_id if self._object else None,
                       "process": str(selection.process.folder) if selection is not None else None,
                       "step": selection.movement.id if selection is not None else None, "seconds": seconds,
                       "frame": "world (mocap, Z-up), metres; quaternions (x, y, z, w); deviation in the planned "
                                "bar's frame, its axis z"}
        extra = {"model_markers": layout.points, "model_marker_names": np.array(layout.names)}
        file_format = ctx.config.recording_format

        def save() -> Path:
            folder = run_folder(self._folder, experiment, bar_id.split("/")[-1])
            write_description(folder, description)
            return save_recording(folder, take, None, extra, file_format)

        path = await ctx.run_in_thread(save)
        fitted = int(np.isfinite(take.values("fit_rms")).sum())
        self._message = f"saved {len(take)} samples ({fitted} fitted) to {path}"
        ctx.log_info(self._message)

    def _signals(self, model: np.ndarray) -> list[Signal]:
        """What a take records each tick: the fit, its quality, the planned pose and the deviation."""
        def fit() -> BarFit | None:
            return self._fit

        def pose() -> Pose | None:
            return self._pose

        def planned() -> Pose | None:
            return self._planned

        def deviation() -> BarDeviation | None:
            return self._deviation

        return [
            Signal("fit_position", lambda: pose().position if pose() else None, ("x", "y", "z"), "m"),
            Signal("fit_orientation", lambda: pose().orientation if pose() else None, ("qx", "qy", "qz", "qw")),
            Signal("fit_rms", lambda: fit().rms_m if fit() else None, (), "m"),
            Signal("fit_max", lambda: fit().max_residual_m if fit() else None, (), "m"),
            Signal("fit_count", lambda: fit().n_matched if fit() else None),
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
        fit, sensed = self._fit, self._sensed
        if fit is None or sensed is None:
            return None
        pairs = matched_pairs(fit)
        rows = np.full((count, 3), np.nan)
        rows[pairs[:, 1]] = sensed.points[pairs[:, 0]]
        return rows.reshape(-1)

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Follow the loaded process, refit on a new mocap message at most every FIT_PERIOD, and compare with the plan.

        Args:
            ctx: This plugin's context.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of = loaded
            self._layouts = bar_layouts(loaded) if loaded is not None else {}
            self._bar.options = list(self._layouts) or [NO_BAR]
            self._bar_id = next(iter(self._layouts), None)
            self._bar.value = self._bar_id or NO_BAR
            self._forget_fit()
        ids = selection.ids if selection is not None else None
        live_id = ids.get(self._bar_id) if ids is not None and self._bar_id else None
        self._planned = ctx.scene.snapshot.world_poses.get(live_id) if live_id else None
        obj, now = self._object, ctx.now()
        if self._bar_id is None or obj is None or obj.markers is None:
            self._fit = self._pose = None
        elif obj.last_update_time != self._fitted_update and now >= self._next_fit_at:
            self._fitted_update, self._next_fit_at = obj.last_update_time, now + FIT_PERIOD
            self._refit(obj.markers, self._layouts[self._bar_id], now)
        self._deviation = (bar_deviation(self._pose, self._planned)
                           if self._pose is not None and self._planned is not None else None)

    def _refit(self, markers: RawMarkers, layout: BarLayout, now: float) -> None:
        """Fit the layout to this sample, matched around the last good fit while fresh, else around the planned pose."""
        if self._last_ok is not None and now - self._last_ok[0] <= HINT_MAX_AGE:
            hint = self._last_ok[1]
        else:
            hint = self._planned.matrix() if self._planned is not None else None
        self._sensed = sensed_from_raw(markers, now)
        fit = fit_bar(self._sensed, layout, max_age=MAX_MARKER_AGE, hint=hint)
        self._fit = fit if fit.ok else None
        self._pose = Pose.from_matrix(fit.T_world_from_bar) if fit.ok else None
        self._reason = "" if fit.ok else fit.reason
        if fit.ok:
            self._last_ok = (now, fit.T_world_from_bar)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the tracking state, the fit and the deviation, and move the marker nodes.

        Args:
            ctx: This plugin's context.
        """
        obj = self._object
        if obj is None:
            state = chip("not tracking", NONE, "choose the bar's Motive rigid body and press Track")
        else:
            state = check_chip(mocap_check(f"mocap {obj.mocap_id}", obj.mocap_id, obj, ctx.now()))
        self._status.content = block(state + values(*self._status_lines()))
        running = self._take is not None and not self._take.done()
        self._result.content = (chip("recording", BUSY) if running else "") + (
            note(escape(self._message)) if self._message else "")
        self._draw_bar(ctx)
        self._draw_markers()

    def _status_lines(self) -> list[str]:
        """Marker counts, the fit's error and the deviation from the plan: always 3 lines, "—" when missing."""
        lines = self._readout()
        return lines + ["—"] * (3 - len(lines))

    def _readout(self) -> list[str]:
        """Up to 3 lines: marker counts, the fit's error or why it failed, and the deviation from the plan."""
        if not self._layouts:
            return ["no bar with markers in the loaded process"]
        obj, fit, bar_id = self._object, self._fit, self._bar_id
        if obj is None or obj.markers is None:
            count = len(self._layouts[bar_id].points) if bar_id in self._layouts else 0
            return [f"{bar_id}: {count} CAD markers; no mocap message yet"]
        lines = [f"markers {len(obj.markers.ids)}, measured {int(obj.markers.measured.sum())}"]
        if fit is None:
            lines.append(f"{bar_id}: {self._reason or 'no fit yet'}")
            return lines
        lines.append(f"fit {fit.n_matched}/{fit.n_design}  rms {fit.rms_m * 1e3:.2f}  max "
                     f"{fit.max_residual_m * 1e3:.2f} mm")
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
        pose = self._pose
        self._bar_frame.visible = pose is not None and bool(self._bar_nodes)
        if self._bar_frame.visible:
            self._bar_frame.position = pose.position
            self._bar_frame.wxyz = quaternion_to_wxyz(pose.orientation)

    def _draw_markers(self) -> None:
        """Show the measured markers, the fitted and the planned bar, resending them only when they changed."""
        obj = self._object
        markers = obj.markers if obj is not None and self._show_markers.value else None
        sensed = self._sensed if markers is not None else None
        # ? Identity, not id(): a freed sample's id can be reused by the next one.
        shown = (markers, sensed, self._fit, self._planned, self._bar_id)
        if self._drawn is not None and all(a is b for a, b in zip(self._drawn, shown)):
            return
        self._drawn = shown
        # * Green: the trusted markers the last fit used; orange: the ones Motive filled in from its own pose.
        self._measured_cloud.visible = sensed is not None and len(markers.ids) > 0
        if self._measured_cloud.visible:
            filled = markers.positions[~markers.measured]
            self._measured_cloud.points = np.concatenate([sensed.points, filled]).astype(np.float32)
            self._measured_cloud.colors = np.array([MEASURED_COLOR] * len(sensed.points) + [FILLED_COLOR] * len(filled),
                                                   dtype=np.uint8).reshape(-1, 3)
        layout = self._layouts[self._bar_id].points if self._bar_id in self._layouts else None
        self._planned_cloud.visible = layout is not None and self._planned is not None and markers is not None
        if self._planned_cloud.visible:
            self._planned_cloud.points = transform_points(self._planned.matrix(), layout).astype(np.float32)
        fit = self._fit
        self._fit_cloud.visible = self._fit_lines.visible = fit is not None and markers is not None
        if not self._fit_cloud.visible:
            return
        world_from_bar = fit.T_world_from_bar
        self._fit_cloud.points = transform_points(world_from_bar, layout).astype(np.float32)
        # * The bar axis over the markers' span, then one line from each matched marker to its fitted CAD marker.
        span = layout[:, 2].min(), layout[:, 2].max()
        axis = transform_points(world_from_bar, np.array([[0.0, 0.0, span[0]], [0.0, 0.0, span[1]]]))
        pairs = matched_pairs(fit)
        residuals = np.stack([sensed.points[pairs[:, 0]], transform_points(world_from_bar, layout[pairs[:, 1]])],
                             axis=1)
        self._fit_lines.points = np.concatenate([axis[None], residuals]).astype(np.float32)
        self._fit_lines.colors = np.full((len(residuals) + 1, 2, 3), FIT_COLOR, dtype=np.uint8)
