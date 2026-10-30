"""
Bar marker fitting: where every bar the mocap sees is, against where the process plans it.

Every bar of the loaded process that carries markers is tracked as its Motive asset (bar Bxx streams as asset 10xx;
the Mocap ID box re-tracks the chosen bar under another id). Each bar's CAD markers are fitted to the measured ones
without Motive's labels (`world/marker_layouts.fit_bar`, the old monitor's matcher) at most ten times a second:
matched around the last good fit while it is fresh, else around the planned pose, else by the full search, of which
at most one runs per tick across the bars. "Ghost bars" draws every fitted bar (the process bar and its mounted
halves, see-through, the old monitor's MoCap cyan) at its fitted pose; "Show markers" draws the chosen bar's
measured markers (green the trusted ones the fit used, orange filled in by Motive), its fitted markers (blue:
markers, axis, a line per residual) and its planned markers and line (grey, pink). The stats block below the buttons
reads the chosen bar's fit the old monitor's way: each marker's residuals, each plate's ball pair, the spacing
between plates, the bow, and the deviation from the planned pose; "Measure + log" and "Log live" print the old
`[mocap bar]` lines. The planned bar is the process body in the live scene (`process/bars/<id>`): a built bar at its
assembly pose, a held one on the real robot's flange. Record writes a take of the chosen bar's fit to the Drive
(`data_experiment/mocap_bars/`): a run folder per take with experiment.json and recording.npz
(`shared/recordings/runs.py`). Other plugins read the fits through `MocapBarsContract`.

- ! Marker layouts come only from the process (`BodySpec.markers` of the halves mounted on a bar); old compas_fab
  exports converted on load have none.
- ! The fits run on the main thread: a full search takes ~20 ms (one per tick at most), the match around a hint ~1 ms
  per bar.
- * Markers dropped for their size or Motive residual are neither fitted nor drawn; a failed fit's reason counts them.
- * The ghosts are drawn only, in this plugin's view: not scene bodies, so they never collide (the planned ones
  already do), and hidden while there is no fit.

Run with:  -p plugins:="['mocap_bars']"  (loads `process`)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import TYPE_CHECKING, Mapping

import numpy as np
import viser

from ..drive import drive_folder, folder_name
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..plugin_api.trace import Signal
from ..shared.mocap_bars_contract import BarMeasurement
from ..shared.process_scene.colors import BAR_LINE_COLOR, BAR_LINE_WIDTH
from ..shared.recordings.runs import run_folder, save_recording, write_description
from ..ui.quaternion import quaternion_to_wxyz
from ..ui.scene_view import add_shape
from ..ui.style import BUSY, NONE, SECTION_CTRL, SECTION_SENSOR, block, check_chip, chip, note, section, values
from ..world.bar_fit import BarFit
from ..world.bar_pose import PoseDeviation, bar_ends, pose_deviation, transform_points
from ..world.marker_fit import (MATCHED, MISSING, MarkerRow, PlateGeometry, bar_deviation, bar_report, marker_rows,
                                plate_geometry)
from ..world.marker_layouts import BarLayout, Sensed, bar_asset_id, bar_layouts, fit_bar, sensed_from_raw
from ..world.measured import TrackedObject
from ..world.mocap import mocap_check
from scaffolding_assembly_core.geometry import Pose, compose, invert

if TYPE_CHECKING:
    from ..shared.process_contract import ProcessContract

#: Name of a tracked bar in `world.tracked_objects` and on the health panel: "<TRACKED_NAME>/<B20>".
TRACKED_NAME = "fitted_bar"
#: Takes go into a folder per experiment under this one, below the Drive root; create it in Drive first.
RECORDING_FOLDER = "data_experiment/mocap_bars"
#: Bar dropdown entry while the process has no bar with markers.
NO_BAR = "–"
#: Fits run at most this often per bar, seconds (the old "follow live (10 Hz)").
FIT_PERIOD = 0.1
#: A sample older than this is not fitted, seconds: is Motive streaming?
MAX_MARKER_AGE = 0.5
#: The last good fit guides the next one while it is younger than this, seconds; after that the planned pose does.
HINT_MAX_AGE = 1.0
#: A bar whose full search failed waits this long before the next, seconds: a wrong asset must not search every turn.
SEARCH_RETRY = 1.0
#: "Log live" prints the chosen bar's report this often, seconds.
LOG_PERIOD = 5.0
#: Colours: measured markers, markers filled in by Motive, the fitted markers, the planned markers.
MEASURED_COLOR = (40, 190, 80)
FILLED_COLOR = (250, 140, 30)
FIT_COLOR = (40, 120, 230)
PLANNED_COLOR = (150, 150, 150)
#: The ghost bars' colour, the old monitor's MoCap cyan (r, g, b, alpha from 0 to 1).
MOCAP_COLOR = (0.0, 0.8, 0.9, 0.5)
#: Size of the drawn markers, metres.
MARKER_POINT_SIZE = 0.02


def matched_pairs(fit: BarFit) -> np.ndarray:
    """The fit's matches as an (n, 2) int array of (sensed index, design index)."""
    return np.array(list(fit.correspondences.items()), dtype=int).reshape(-1, 2)


def short(bar: str) -> str:
    """A bar's name without its folder: "bars/B20" -> "B20"."""
    return bar.rsplit("/", 1)[-1]


@dataclass
class _Followed:
    """One bar followed live: its layout and tracked asset, the last sample and fit, the hint for the next fit.

    Attributes:
        layout: The bar's designed markers.
        obj: Its tracked Motive asset.
        sensed: The sample last fitted, or None.
        fit: The last fit, ok or not; None before any.
        pose: The fitted pose when `fit` is ok, else None.
        fitted_update: The mocap message time of the sample last fitted (ok or by the full search).
        next_fit_at: When the next fit may run.
        last_ok: (time, 4x4 pose) of the last good fit: the hint the next fit starts from while fresh.
        planned: Where the process places the bar this tick, or None.
        deviation: The fit against `planned`, or None.
        measured_at: Capture time of the sample last fitted.
        rows: (marker rows, plate geometry) of the last good fit, computed once for the stats block; None otherwise.
    """

    layout: BarLayout
    obj: TrackedObject
    sensed: Sensed | None = None
    fit: BarFit | None = None
    pose: Pose | None = None
    fitted_update: float | None = None
    next_fit_at: float = 0.0
    last_ok: tuple[float, np.ndarray] | None = None
    planned: Pose | None = None
    deviation: PoseDeviation | None = None
    measured_at: float = 0.0
    rows: tuple[list[MarkerRow], PlateGeometry] | None = None


@register
class MocapBarsPlugin(HuskyPlugin):
    """Fits every process bar's markers to its tracked Motive asset and measures its deviation from the plan."""

    name = "mocap_bars"
    icon = viser.Icon.RULER_2
    requires = ("process",)
    experimental = True

    def __init__(self):
        """Start with nothing tracked or fitted."""
        #: Bar id -> what is followed of it, every bar with a layout, in the process's order.
        self._bars: dict[str, _Followed] = {}
        #: The process the bars were read from.
        self._layouts_of = None
        self._bar_id: str | None = None
        #: Index into the bars of the one whose turn the full search is.
        self._turn = 0
        #: Rows the stats block always has, so it never changes height: set per process.
        self._stats_rows = 3
        #: What the scene last showed of the chosen bar, so marker nodes are only resent when they change.
        self._drawn: tuple | None = None
        #: Bar id -> its ghost's frame node, and the process the ghosts were built from.
        self._ghosts: dict[str, viser.FrameHandle] = {}
        self._ghosts_of = None
        self._take: asyncio.Task | None = None
        self._message = ""
        self._next_log_at = 0.0
        self._now = 0.0

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
                                         hint="The bar the markers, the stats and a take are of; every bar of the "
                                              "loaded process with 3 or more markers on its halves is tracked.")
            self._mocap_id = gui.add_number("Mocap ID", initial_value=0, min=0, step=1,
                                            hint="The Motive rigid body made of this bar's markers (Bxx -> 10xx).")
            track = gui.add_button("Track", hint="Track the chosen bar as the Mocap ID instead.")
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            self._show_markers = gui.add_checkbox("Show markers", initial_value=True)
            self._ghost_bars = gui.add_checkbox("Ghost bars", initial_value=True,
                                                hint="Every fitted bar drawn see-through at its MoCap pose.")
            self._log_live = gui.add_checkbox("Log live", initial_value=False,
                                              hint=f"Print the chosen bar's fit report every {LOG_PERIOD:.0f} s.")
            measure = gui.add_button("Measure + log", hint="Fit the chosen bar afresh and print its report.")

            gui.add_html(section("record", SECTION_CTRL))
            self._experiment = gui.add_text("Experiment", initial_value="bar_accuracy",
                                            hint=f"Takes go to {RECORDING_FOLDER}/<experiment>/ on the Drive.")
            self._seconds = gui.add_slider("Seconds", min=1, max=60, step=1, initial_value=5)
            record = gui.add_button("Record take", disabled=bool(problem),
                                    hint=problem or "Record the fit and its deviation from the plan, once per tick.")
            self._result: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("stats", SECTION_SENSOR))
            self._stats: viser.GuiHtmlHandle = gui.add_html("")

        scene, root = ctx.view.scene, ctx.view.scene_root
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
        # * The planned bar's centre line and name, the pink of the process panel's Rhino bar lines.
        self._planned_line = scene.add_line_segments(f"{root}/planned_line", points=np.zeros((1, 2, 3)),
                                                     colors=BAR_LINE_COLOR, thickness=BAR_LINE_WIDTH,
                                                     thickness_units="screen", visible=False)
        self._planned_label = scene.add_label(f"{root}/planned_label", "", visible=False)

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        self._bar.on_update(ctx.defer_value("choose bar", self._choose))
        track.on_click(ctx.defer("track bar", lambda: self._track(ctx)))
        measure.on_click(ctx.defer("measure bar", lambda: self._measure_and_log(ctx)))
        record.on_click(ctx.defer("record take", lambda: self._start_take(ctx)))

    def teardown(self, ctx: PluginContext) -> None:
        """Stop tracking every bar.

        Args:
            ctx: This plugin's context.
        """
        for bar in list(self._bars):
            ctx.untrack_object(f"{TRACKED_NAME}/{short(bar)}")
        self._bars = {}

    # --- --- --- --- --- THE CONTRACT --- --- --- --- ---

    @property
    def fits(self) -> Mapping[str, BarMeasurement]:
        """Mapping[str, BarMeasurement]: Bar id -> its latest measurement, every tracked bar that had a sample."""
        return {bar: self._measurement(bar, state) for bar, state in self._bars.items() if state.fit is not None}

    def measure_now(self, hints: Mapping[str, Pose] | None = None) -> dict[str, BarMeasurement]:
        """Fit every tracked bar afresh from its newest sample (around its hint, else the full search); kept."""
        now = self._now
        for bar, state in self._bars.items():
            if state.obj.markers is None:
                continue
            hint = hints.get(bar) if hints else None
            self._refit(state, now, None if hint is None else hint.matrix(), full_search=True)
        return {bar: self._measurement(bar, state) for bar, state in self._bars.items() if state.fit is not None}

    def _measurement(self, bar: str, state: _Followed) -> BarMeasurement:
        """The bar's latest fit as the contract hands it on."""
        return BarMeasurement(bar, state.obj.mocap_id, state.layout, state.fit, state.pose, state.sensed,
                              state.sensed.source if state.sensed is not None else "", state.measured_at,
                              state.fit.tracked, state.planned, state.deviation)

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _choose(self, bar_id: str) -> None:
        """Show this bar's markers and stats from now on; the Mocap ID box shows its asset."""
        self._bar_id = bar_id if bar_id in self._bars else None
        if self._bar_id is not None:
            self._mocap_id.value = self._bars[self._bar_id].obj.mocap_id
        self._drawn = None

    def _track(self, ctx: PluginContext) -> None:
        """Track the chosen bar as the rigid body in "Mocap ID" instead of its own asset."""
        state = self._bars.get(self._bar_id)
        if state is None:
            self._message = "choose a bar first"
            return
        mocap_id = int(self._mocap_id.value)
        state.obj = self._track_bar(ctx, self._bar_id, mocap_id)
        self._forget_fit(state)
        self._message = f"{short(self._bar_id)}: tracking mocap id {mocap_id}"
        ctx.log_info(self._message)

    def _track_bar(self, ctx: PluginContext, bar: str, mocap_id: int) -> TrackedObject:
        """Track `bar` as `mocap_id`, replacing an earlier tracking of it."""
        name = f"{TRACKED_NAME}/{short(bar)}"
        ctx.untrack_object(name)
        # * No geometry: the process draws the bar; this is a frame and its markers.
        return ctx.track_object(name, mocap_id, label=f"bar {short(bar)} markers ({mocap_id})")

    def _follow_all(self, ctx: PluginContext, loaded) -> None:
        """Track every bar of the loaded process that has a marker layout, each as its own Motive asset."""
        for bar in list(self._bars):
            ctx.untrack_object(f"{TRACKED_NAME}/{short(bar)}")
        self._bars = {}
        layouts = bar_layouts(loaded) if loaded is not None else {}
        for bar, layout in layouts.items():
            mocap_id = bar_asset_id(bar)
            if mocap_id is None:
                ctx.log_warn(f"{bar} has no Motive asset id by the Bxx -> 10xx rule: not tracked")
                continue
            for warning in layout.warnings:
                ctx.log_warn(warning)
            self._bars[bar] = _Followed(layout, self._track_bar(ctx, bar, mocap_id))
        self._turn = 0
        markers = max((len(layout.points) for layout in layouts.values()), default=0)
        plates = max((len({name.rsplit("/", 1)[0] for name in layout.names}) for layout in layouts.values()),
                     default=0)
        # * A summary, a row per marker, the strays, a row per plate and per spacing, the deviation.
        self._stats_rows = 3 + markers + max(2 * plates - 1, 0)
        self._bar.options = list(self._bars) or [NO_BAR]
        self._bar_id = next(iter(self._bars), None)
        self._bar.value = self._bar_id or NO_BAR
        if self._bar_id is not None:
            self._mocap_id.value = self._bars[self._bar_id].obj.mocap_id
        self._drawn = None
        if self._bars:
            ctx.log_info("tracking " + ", ".join(f"{short(bar)} as asset {state.obj.mocap_id}"
                                                 for bar, state in self._bars.items()))

    @staticmethod
    def _forget_fit(state: _Followed) -> None:
        """Drop a bar's fit and its hint: a new rigid body starts from a full search."""
        state.sensed = state.fitted_update = state.fit = state.pose = state.last_ok = state.rows = None
        state.next_fit_at = 0.0

    def _measure_and_log(self, ctx: PluginContext) -> None:
        """Fit the chosen bar afresh (around its planned pose, else the full search) and print its report."""
        state = self._bars.get(self._bar_id)
        if state is None or state.obj.markers is None:
            self._message = "no mocap message of the chosen bar yet"
            return
        self._refit(state, self._now, state.planned.matrix() if state.planned is not None else None,
                    full_search=True)
        self._log_report(ctx, self._bar_id, state)

    def _log_report(self, ctx: PluginContext, bar: str, state: _Followed) -> None:
        """The old monitor's three `[mocap bar]` lines of a bar, a warning when its fit failed."""
        if state.fit is None:
            return
        planned = state.planned.matrix() if state.planned is not None else None
        say = ctx.log_info if state.fit.ok else ctx.log_warn
        for line in bar_report(bar, state.fit, state.sensed, state.layout, planned, state.deviation):
            say(line)

    def _start_take(self, ctx: PluginContext) -> None:
        """Start recording a take, unless one runs or nothing is fitted yet."""
        if self._take is not None and not self._take.done():
            self._message = "a take is already recording"
        elif self._bar_id is None:
            self._message = "choose a bar first"
        else:
            self._take = ctx.spawn("record take", self._record_take(ctx), survives_soft_stop=True)

    async def _record_take(self, ctx: PluginContext) -> None:
        """Record the chosen bar's fit for the chosen seconds, then save it with its layout and the plan's context."""
        bar_id, seconds = self._bar_id, float(self._seconds.value)
        state = self._bars[bar_id]
        experiment = folder_name(self._experiment.value, "take")
        self._message = f"recording {bar_id} for {seconds:g} s"
        with ctx.record(*self._signals(state)) as take:
            await ctx.sleep(seconds)
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        description = {"bar": bar_id, "mocap_id": state.obj.mocap_id,
                       "process": str(selection.process.folder) if selection is not None else None,
                       "step": selection.movement.id if selection is not None else None, "seconds": seconds,
                       "frame": "world (mocap, Z-up), metres; quaternions (x, y, z, w); deviation in the planned "
                                "bar's frame, its axis z"}
        extra = {"model_markers": state.layout.points, "model_marker_names": np.array(state.layout.names)}
        file_format = ctx.config.recording_format

        def save() -> Path:
            folder = run_folder(self._folder, experiment, short(bar_id))
            write_description(folder, description)
            return save_recording(folder, take, None, extra, file_format)

        path = await ctx.run_in_thread(save)
        fitted = int(np.isfinite(take.values("fit_rms")).sum())
        self._message = f"saved {len(take)} samples ({fitted} fitted) to {path}"
        ctx.log_info(self._message)

    def _signals(self, state: _Followed) -> list[Signal]:
        """What a take records each tick: the fit, its quality, the planned pose and the deviation."""
        model = state.layout.points

        def fit() -> BarFit | None:
            return state.fit if state.fit is not None and state.fit.ok else None

        def planned_deviation():
            return (bar_deviation(state.pose, state.planned)
                    if state.pose is not None and state.planned is not None else None)

        return [
            Signal("fit_position", lambda: state.pose.position if state.pose else None, ("x", "y", "z"), "m"),
            Signal("fit_orientation", lambda: state.pose.orientation if state.pose else None,
                   ("qx", "qy", "qz", "qw")),
            Signal("fit_rms", lambda: fit().rms_m if fit() else None, (), "m"),
            Signal("fit_max", lambda: fit().max_residual_m if fit() else None, (), "m"),
            Signal("fit_count", lambda: fit().n_matched if fit() else None),
            Signal("planned_position", lambda: state.planned.position if state.planned else None, ("x", "y", "z"),
                   "m"),
            Signal("planned_orientation", lambda: state.planned.orientation if state.planned else None,
                   ("qx", "qy", "qz", "qw")),
            Signal("deviation", lambda: (planned_deviation().across, planned_deviation().along)
                   if planned_deviation() else None, ("across", "along"), "m"),
            Signal("tilt", lambda: planned_deviation().tilt if planned_deviation() else None, (), "rad"),
            Signal("matched_markers", lambda: self._matched(state), tuple(
                f"{index}.{axis}" for index in range(len(model)) for axis in "xyz"), "m"),
        ]

    @staticmethod
    def _matched(state: _Followed) -> np.ndarray | None:
        """The measured marker matched to each CAD marker, flattened, NaN where none was; None without a fit."""
        fit, sensed = state.fit, state.sensed
        if fit is None or not fit.ok or sensed is None:
            return None
        pairs = matched_pairs(fit)
        rows = np.full((len(state.layout.points), 3), np.nan)
        rows[pairs[:, 1]] = sensed.points[pairs[:, 0]]
        return rows.reshape(-1)

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Follow the loaded process, refit each bar on a new mocap message at most every FIT_PERIOD, compare.

        Args:
            ctx: This plugin's context.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of = loaded
            self._follow_all(ctx, loaded)
        ids = selection.ids if selection is not None else None
        now = self._now = ctx.now()
        searches = []
        for bar, state in self._bars.items():
            live_id = ids.get(bar) if ids is not None else None
            state.planned = ctx.scene.snapshot.world_poses.get(live_id) if live_id else None
            obj = state.obj
            if obj.markers is None:
                state.fit = state.pose = state.rows = None
            elif obj.last_update_time != state.fitted_update and now >= state.next_fit_at:
                if state.last_ok is not None and now - state.last_ok[0] <= HINT_MAX_AGE:
                    hint = state.last_ok[1]
                else:
                    hint = state.planned.matrix() if state.planned is not None else None
                # * The quick match around the hint; a bar it misses waits for its turn at the full search.
                if hint is None or not self._refit(state, now, hint, full_search=False):
                    searches.append(bar)
            state.deviation = (pose_deviation(state.pose.matrix(), state.planned.matrix())
                               if state.pose is not None and state.planned is not None else None)
        if searches:
            self._search_one(searches, now)
        chosen = self._bars.get(self._bar_id)
        if chosen is not None and self._log_live.value and now >= self._next_log_at and chosen.fit is not None:
            self._next_log_at = now + LOG_PERIOD
            self._log_report(ctx, self._bar_id, chosen)

    def _search_one(self, bars: list[str], now: float) -> None:
        """The full search for one of `bars`, the first at or after the turn; the others wait for the next tick."""
        order = list(self._bars)
        indices = sorted(order.index(bar) for bar in bars)
        index = next((i for i in indices if i >= self._turn), indices[0])
        state = self._bars[order[index]]
        self._refit(state, now, None, full_search=True)
        self._turn = (index + 1) % len(order)
        state.deviation = (pose_deviation(state.pose.matrix(), state.planned.matrix())
                           if state.pose is not None and state.planned is not None else None)

    def _refit(self, state: _Followed, now: float, hint: np.ndarray | None, full_search: bool) -> bool:
        """Fit a bar's layout to its newest sample; True when the fit is ok.

        A good fit, or a full search's failure (kept for its reason, the bar then waits SEARCH_RETRY), replaces the
        bar's fit and sample; a hint match that misses with the search withheld leaves the last fit as it was.
        """
        obj = state.obj
        sensed = sensed_from_raw(obj.markers, now)
        fit = fit_bar(sensed, state.layout, max_age=MAX_MARKER_AGE, hint=hint, full_search=full_search)
        if not fit.ok and not full_search:
            return False
        state.sensed, state.fit, state.rows = sensed, fit, None
        state.measured_at = obj.markers.capture_time if obj.markers.capture_time is not None else (obj.last_update_time
                                                                                                   or now)
        state.fitted_update = obj.last_update_time
        if fit.ok:
            state.pose = Pose.from_matrix(fit.T_world_from_bar)
            state.last_ok = (now, fit.T_world_from_bar)
            state.next_fit_at = now + FIT_PERIOD
            return True
        state.pose, state.next_fit_at = None, now + SEARCH_RETRY
        return False

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the tracking state, the fits and the deviations, and move the marker and ghost nodes.

        Args:
            ctx: This plugin's context.
        """
        state = self._bars.get(self._bar_id)
        if state is None:
            chips = chip("not tracking", NONE, "no bar with markers in the loaded process")
        else:
            chips = check_chip(mocap_check(f"mocap {state.obj.mocap_id}", state.obj.mocap_id, state.obj, ctx.now()))
        self._status.content = block(chips + values(*self._status_lines()))
        running = self._take is not None and not self._take.done()
        self._result.content = (chip("recording", BUSY) if running else "") + (
            note(escape(self._message)) if self._message else "")
        self._draw_ghosts(ctx)
        self._draw_markers()
        self._stats.content = self._stats_block()

    def _status_lines(self) -> list[str]:
        """What is tracked, the chosen bar's markers and fit, its deviation: always 3 lines, "—" when missing."""
        lines = self._readout()
        return lines + ["—"] * (3 - len(lines))

    def _readout(self) -> list[str]:
        """Up to 3 lines: the tracked bars, the chosen bar's counts and fit (or why it failed), its deviation."""
        if not self._bars:
            return ["no bar with markers in the loaded process"]
        lines = ["tracking " + " ".join(f"{short(bar)}:{state.obj.mocap_id}" for bar, state in self._bars.items())]
        state, bar_id = self._bars.get(self._bar_id), self._bar_id
        if state is None:
            return lines
        obj, fit = state.obj, state.fit
        if obj.markers is None:
            lines.append(f"{short(bar_id)}: {len(state.layout.points)} CAD markers; no mocap message yet")
            return lines
        if fit is None or not fit.ok:
            lines.append(f"{short(bar_id)}: markers {len(obj.markers.ids)}, measured "
                         f"{int(obj.markers.measured.sum())}; {fit.reason if fit is not None else 'no fit yet'}")
            return lines
        lines.append(f"{short(bar_id)}: markers {len(obj.markers.ids)}, measured {int(obj.markers.measured.sum())}; "
                     f"fit {fit.n_matched}/{fit.n_design}  rms {fit.rms_m * 1e3:.2f}  max "
                     f"{fit.max_residual_m * 1e3:.2f} mm")
        deviation = state.deviation
        lines.append("no planned pose: is the bar in the selected step?" if deviation is None else
                     f"vs planned  dpos {deviation.dpos_mm:.1f} mm  dang {deviation.dang_deg:.2f}°  "
                     f"dx {deviation.dpos_bar_mm[0]:+.1f} dy {deviation.dpos_bar_mm[1]:+.1f} "
                     f"dz(along) {deviation.dpos_bar_mm[2]:+.1f} mm")
        return lines

    def _stats_block(self) -> str:
        """The chosen bar's fit read the old monitor's way, always `_stats_rows` rows (padded with "—")."""
        rows = self._stats_lines()
        return block(values(*(rows + ["—"] * (self._stats_rows - len(rows)))[:self._stats_rows]))

    def _stats_lines(self) -> list[str]:
        """The summary, one row per design marker, the strays, the plates and spacings, the deviation."""
        state = self._bars.get(self._bar_id)
        if state is None or state.fit is None or state.sensed is None:
            return ["no fit yet"]
        fit, sensed, layout = state.fit, state.sensed, state.layout
        lines = [f"source {sensed.source}  age {sensed.age * 1e3:.0f} ms  fit {fit.n_matched}/{fit.n_design}  "
                 + (f"rms {fit.rms_m * 1e3:.2f}  max {(fit.max_residual_m or 0.0) * 1e3:.2f} mm  " if fit.rms_m
                    is not None else "")
                 + f"ambiguous {'yes' if fit.ambiguous else 'no'}  dropped {sensed.dropped}  occluded "
                 f"{sensed.occluded}" + ("" if fit.ok else f"  FAILED: {fit.reason}")]
        if not fit.ok:
            return lines
        if state.rows is None:
            state.rows = (marker_rows(fit, sensed, layout), plate_geometry(fit, sensed, layout))
        markers, geometry = state.rows
        strays = 0
        for row in markers:
            if row.state == MATCHED:
                motive = "—" if row.motive_mm is None else f"{row.motive_mm:.1f}"
                lines.append(f"{_plain(row.name):<22} motive {motive:>4}  fit {row.fit_mm:4.1f}  matched")
            elif row.state == MISSING:
                lines.append(f"{_plain(row.name):<22} missing")
            else:
                strays += 1
        lines.append(f"stray sensed {strays}")
        for plate in geometry.plates:
            if plate.length_mm is None:
                lines.append(f"{_plain(plate.plate):<18} {plate.n_seen} of 2 balls seen")
                continue
            bow = "" if plate.bend_mm is None else f"  bow {plate.bend_mm:.1f} mm"
            lines.append(f"{_plain(plate.plate):<18} pair {plate.length_mm:.1f} ({plate.length_err_mm:+.1f}) mm  "
                         f"roll {plate.roll_err_deg:+.1f}°{bow}")
        for spacing in geometry.spacing:
            lines.append(f"{_plain(spacing.from_plate)} → {_plain(spacing.to_plate):<12} {spacing.mm:.1f} "
                         f"({spacing.err_mm:+.1f}) mm")
        deviation = state.deviation
        lines.append("no planned pose: is the bar in the selected step?" if deviation is None else
                     f"vs planned  dpos {deviation.dpos_mm:.1f} mm  dang {deviation.dang_deg:.2f}°  "
                     f"dx {deviation.dpos_bar_mm[0]:+.1f} dy {deviation.dpos_bar_mm[1]:+.1f} "
                     f"dz(along) {deviation.dpos_bar_mm[2]:+.1f} mm")
        return lines

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Every fitted bar's body and mounted halves at its fitted pose, while "Ghost bars" is ticked."""
        process = self._layouts_of
        if self._ghosts_of is not process:
            self._ghosts_of = process
            for frame in self._ghosts.values():
                frame.remove()
            self._ghosts = {}
        for bar, state in self._bars.items():
            frame = self._ghosts.get(bar)
            show = self._ghost_bars.value and state.pose is not None
            if frame is None:
                if not show:
                    continue
                frame = self._ghosts[bar] = self._build_ghost(ctx, process, bar)
            frame.visible = show
            if show:
                frame.position = state.pose.position
                frame.wxyz = quaternion_to_wxyz(state.pose.orientation)

    def _build_ghost(self, ctx: PluginContext, process, bar_id: str) -> viser.FrameHandle:
        """A frame holding the bar's visual shapes and its mounted halves', see-through, at the origin."""
        scene = ctx.view.scene
        frame = scene.add_frame(f"{ctx.view.scene_root}/ghosts/{short(bar_id)}", show_axes=False, visible=False)
        bar = process.bodies.get(bar_id) if process is not None else None
        if bar is None:
            return frame
        for index, shape in enumerate(bar.geometry.visual):
            add_shape(scene, f"{frame.name}/{index}", shape, MOCAP_COLOR)
        # * Mounted halves stay where they sit on their bar, so the assembly poses give the offset.
        for half_id in sorted(process.bodies):
            half = process.bodies[half_id]
            if half.mount != bar_id:
                continue
            offset = compose(invert(bar.assembly_pose), half.assembly_pose)
            half_frame = scene.add_frame(f"{frame.name}/{half_id.replace('/', '_')}", show_axes=False,
                                         position=offset.position, wxyz=quaternion_to_wxyz(offset.orientation))
            for index, shape in enumerate(half.geometry.visual):
                add_shape(scene, f"{half_frame.name}/{index}", shape, MOCAP_COLOR)
        return frame

    def _draw_markers(self) -> None:
        """Show the chosen bar's measured markers, its fitted and planned markers, resent only when they changed."""
        state = self._bars.get(self._bar_id)
        obj = state.obj if state is not None else None
        markers = obj.markers if obj is not None and self._show_markers.value else None
        sensed = state.sensed if markers is not None else None
        fit = state.fit if state is not None and state.fit is not None and state.fit.ok else None
        planned = state.planned if state is not None else None
        # ? Identity, not id(): a freed sample's id can be reused by the next one.
        shown = (markers, sensed, fit, planned, self._bar_id)
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
        layout = state.layout.points if state is not None else None
        self._planned_cloud.visible = layout is not None and planned is not None and markers is not None
        self._planned_line.visible = self._planned_label.visible = self._planned_cloud.visible
        if self._planned_cloud.visible:
            T = planned.matrix()
            self._planned_cloud.points = transform_points(T, layout).astype(np.float32)
            self._planned_line.points = bar_ends(T, state.layout.length).astype(np.float32).reshape(1, 2, 3)
            self._planned_label.text = f"{short(self._bar_id)} Rhino"
            self._planned_label.position = tuple(float(v) for v in T[:3, 3])
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


def _plain(name: str) -> str:
    """A marker or plate name without the "joints/" folder."""
    return name[len("joints/"):] if name.startswith("joints/") else name
