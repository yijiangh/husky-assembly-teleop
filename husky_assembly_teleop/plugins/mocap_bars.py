"""
Bar marker fitting: where every bar the mocap sees is, against where the process plans it.

Every bar of the loaded process that carries markers is fitted to the relay's raw markers near where it should be
(`shared/bars/layouts.fit_in_cloud`): the markers within SEARCH_RADIUS of its markers at its planned pose, Motive's
labels and assets never used (Motive mixes up bars that look alike). The planned pose is the process body in the live
scene (`process/bars/<id>`): a built bar at its assembly pose, a held one on the real robot's flange; a bar the live
scene lacks is looked for at its assembly pose. Each bar is fitted at most ten times a second: matched around the last
good fit while it is fresh (a marker may be hidden), else around the planned pose (every marker), else by the full
search over its nearby markers, of which at most one runs per tick across the bars and only for bars the live scene
has. Every bar is a body of the live scene, "mocap_bars/<B20>": the process bar and
its mounted halves, see-through in the old monitor's MoCap cyan, at its fitted pose, shown while its fit is fresh; the
observer records them every tick. "Show markers" draws the chosen bar's nearby markers
(green the ones the fit matched, amber the others), its fitted markers (blue: markers, axis, a line per residual) and
its planned markers and line (grey, pink); "Raw markers" every marker of the cloud. The stats block below the buttons
reads the chosen bar's fit the old monitor's way: each marker's residuals, each plate's ball pair, the spacing between
plates, the bow, and the deviation from the planned pose; "Measure + log" and "Log live" print the old `[mocap bar]`
lines. "Arrows" draws the as-built report's arrows on the chosen bar or every fitted bar, from where it should be to
its fit: the gap at its start, middle and end, the planned (faint) and fitted axes with the turn about each, both
"Magnify" times larger (labels true). The fit settings section sets the search radius and the timings below for this
plugin's fits (the contract's too, so the as-built record's), not servo's. Record writes a take of the chosen bar's
fit to the Drive (`data_experiment/mocap_bars/`): a run folder per take with experiment.json and recording.npz
(`shared/recordings/runs.py`). Other plugins read the fits through `MocapBarsContract`.

- ! Marker layouts come only from the process (`BodySpec.markers` of the halves mounted on a bar); old compas_fab
  exports converted on load have none.
- ! A bar farther than SEARCH_RADIUS from where it should be is not found: the fit is a check of the plan, not a
  search of the lab.
- ! The fits run on the main thread: a full search takes ~20 ms (one per tick at most), the match around a hint ~1 ms
  per bar.
- * Markers dropped for their size or Motive residual are neither fitted nor drawn; a failed fit's reason counts them.
- ! The bodies have visual shapes only: they never collide (the planned bars already do) and are never obstacles.

Run with:  plugins:="mocap_bars"  (loads `process`)
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import TYPE_CHECKING, Mapping

import numpy as np
import viser

from ..colors import BLUE, FITTED, GRAPE, GREEN, GREY, INK, LIGHT_GREY, MEASURED, RED, WARN, rgb, rgb01
from ..drive import drive_folder, environment, folder_name
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..plugin_api.trace import Signal
from ..shared.mocap_bars_contract import MOCAP_TAB, BarMeasurement
from ..shared.process_scene.colors import BAR_LINE_COLOR, BAR_LINE_WIDTH
from ..shared.recordings.runs import run_folder, save_recording, write_description
from ..shared.bars.layouts import SEARCH_RADIUS, BarLayout, bar_layouts, drawn_bar, fit_in_cloud
from ..shared.bars.pose import PoseDeviation, bar_ends, pose_deviation
from ..shared.bars.readings import (MATCHED, MISSING, MarkerRow, PlateGeometry, bar_report, marker_rows,
                                    plate_geometry)
from ..ui.style import (BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_SENSOR, block, chip, chip_line, note, section,
                        values)
from ..world.marker_match import BarFit, Sensed, transform_points
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.scene import Body

if TYPE_CHECKING:
    from ..robot_interface.mocap_state import MarkerCloud
    from ..shared.process_contract import ProcessContract

#: Takes go into a folder per experiment under this one, below the Drive root; create it in Drive first.
RECORDING_FOLDER = "data_experiment/mocap_bars"
#: Bar dropdown entry while the process has no bar with markers.
NO_BAR = "–"
#: Defaults of the panel's fit settings. Fits run at most this often per bar, seconds (the old "follow live
#: (10 Hz)").
FIT_PERIOD = 0.1
#: A sample older than this is not fitted, seconds: is Motive streaming?
MAX_MARKER_AGE = 0.5
#: The last good fit guides the next one while it is younger than this, seconds; after that the planned pose does.
HINT_MAX_AGE = 1.0
#: A bar whose full search failed waits this long before the next, seconds: a bar not there must not search every turn.
SEARCH_RETRY = 1.0
#: "Log live" prints the chosen bar's report this often, seconds.
LOG_PERIOD = 5.0
#: "Raw markers" redraws the whole cloud at most this often, seconds.
CLOUD_PERIOD = 0.2
#: Colours: the nearby markers the fit matched, the other nearby markers, the fitted markers, the planned markers,
#: the whole cloud.
MEASURED_COLOR = rgb(GREEN)
STRAY_COLOR = rgb(WARN)
FIT_COLOR = rgb(FITTED)
PLANNED_COLOR = rgb(GREY)
CLOUD_COLOR = rgb(GREY)
#: The fitted bars' colour, the old monitor's MoCap cyan (r, g, b, alpha from 0 to 1).
MOCAP_COLOR = rgb01(MEASURED, 0.5)
#: Size of the drawn markers, metres; the whole cloud's smaller, so the chosen bar's stand out.
MARKER_POINT_SIZE = 0.02
CLOUD_POINT_SIZE = 0.012
#: "Arrows": none, the chosen bar's, or every fitted bar's.
ARROW_MODES = ("off", "chosen bar", "all bars")
#: The arrows of a gap or a turn are drawn this many times larger by default (the as-built viewer's MAGNIFY): a
#: millimetre on a 2 m bar cannot be seen. Their labels are the true values.
MAGNIFY = 20
#: Length of the axes drawn at a bar's start and radius of the turn arcs around them, metres.
AXIS_LENGTH = 0.15
ARC_RADIUS = 0.03
#: The arrows are redrawn at most this often, seconds.
ARROW_PERIOD = 0.2
#: Colours of the arrows: the gaps (the as-built viewer's ink), the turns (its purple), the planned axes (faint), the
#: fitted x / y / z axes.
GAP_COLOR = rgb(INK)
TURN_COLOR = rgb(GRAPE)
PLANNED_AXIS_COLOR = rgb(LIGHT_GREY)
AXIS_COLORS = (rgb(RED), rgb(GREEN), rgb(BLUE))


def matched_pairs(fit: BarFit) -> np.ndarray:
    """The fit's matches as an (n, 2) int array of (sensed index, design index)."""
    return np.array(list(fit.correspondences.items()), dtype=int).reshape(-1, 2)


def short(bar: str) -> str:
    """A bar's name without its folder: "bars/B20" -> "B20"."""
    return bar.rsplit("/", 1)[-1]


@dataclass
class Arrows:
    """What the arrows of one or more bars draw: line segments and their colours, and labels.

    Attributes:
        segments: (2, 3) world points per segment, metres.
        colors: An rgb colour per segment.
        labels: (text, world position) per label.
    """

    segments: list[np.ndarray]
    colors: list[tuple[int, int, int]]
    labels: list[tuple[str, np.ndarray]]


def _arrow_head(tip: np.ndarray, direction: np.ndarray, size: float) -> list[np.ndarray]:
    """The two short strokes of an arrow head at `tip`, pointing along `direction`, `size` long."""
    forward = direction / np.linalg.norm(direction)
    side = np.cross(forward, (0.0, 0.0, 1.0))
    if np.linalg.norm(side) < 1e-6:
        side = np.cross(forward, (1.0, 0.0, 0.0))
    side /= np.linalg.norm(side)
    back = tip - forward * size
    return [np.array([tip, back + side * 0.4 * size]), np.array([tip, back - side * 0.4 * size])]


def deviation_arrows(planned: np.ndarray, fitted: np.ndarray, length: float, magnify: float = MAGNIFY,
                     name: str = "") -> Arrows:
    """The as-built viewer's arrows for one bar: its gaps and turns from planned to fitted, `magnify` times larger.

    An arrow from each of the planned bar's start, middle and end towards the fitted one's (at least 1 cm long), its
    true distance at the tip; the planned axes at the start (faint) and the fitted ones (x / y / z), each fitted
    axis's tip labelled with the turn about it, and a curved arrow sweeping `magnify` times that turn about it.

    Args:
        planned: 4x4 planned bar pose (origin at the middle, z along the bar).
        fitted: 4x4 fitted bar pose.
        length: Bar length, metres.
        magnify: How many times larger the gaps and turns are drawn.
        name: Put before the middle gap's label ("B20"), or nothing.
    """
    out = Arrows([], [], [])

    def add(segment: np.ndarray, color: tuple[int, int, int]) -> None:
        out.segments.append(segment)
        out.colors.append(color)

    half = length / 2.0
    along = np.array([[0.0, 0.0, -half], [0.0, 0.0, 0.0], [0.0, 0.0, half]])
    targets, reals = transform_points(planned, along), transform_points(fitted, along)
    for index, (target, real) in enumerate(zip(targets, reals)):
        gap = real - target
        size = float(np.linalg.norm(gap))
        text = f"{size * 1e3:.2f} mm"
        if index == 1 and name:
            text = f"{name} {text}"
        if size < 1e-7:
            out.labels.append((text, target))
            continue
        shown = max(0.01, magnify * size)
        tip = target + gap / size * shown
        add(np.array([target, tip]), GAP_COLOR)
        for stroke in _arrow_head(tip, gap, min(0.015, 0.3 * shown)):
            add(stroke, GAP_COLOR)
        out.labels.append((text, tip))
    turn = pose_deviation(fitted, planned).rotvec_bar_deg
    origin, fitted_origin = targets[0], reals[0]
    axes, fitted_axes = planned[:3, :3].T, fitted[:3, :3].T
    # * Which way each arc starts from its axis: about x towards y, about y towards x, about z towards y.
    start_axis = (1, 0, 1)
    for i in range(3):
        add(np.array([origin, origin + axes[i] * AXIS_LENGTH]), PLANNED_AXIS_COLOR)
        add(np.array([fitted_origin, fitted_origin + fitted_axes[i] * AXIS_LENGTH]), AXIS_COLORS[i])
        # * + 0.0: no "-0.00°".
        out.labels.append((f"about {'XYZ'[i]}: {round(float(turn[i]), 2) + 0.0:+.2f}°",
                           fitted_origin + fitted_axes[i] * AXIS_LENGTH))
        # At most a near full turn, however large the magnified angle; none when too small to see.
        sweep = float(np.clip(magnify * np.radians(turn[i]), -5.8, 5.8))
        if abs(sweep) <= 0.05:
            continue
        centre, k, v = origin + axes[i] * AXIS_LENGTH * 0.8, axes[i], axes[start_axis[i]]
        steps = max(4, int(np.ceil(abs(sweep) / 0.05)))
        angles = np.linspace(0.0, sweep, steps + 1)[:, None]
        # * Rodrigues for v across k: v cos a + (k x v) sin a.
        points = centre + ARC_RADIUS * (v * np.cos(angles) + np.cross(k, v) * np.sin(angles))
        for a, b in zip(points[:-1], points[1:]):
            add(np.array([a, b]), TURN_COLOR)
        for stroke in _arrow_head(points[-1], points[-1] - points[-2], min(0.012, 0.5 * ARC_RADIUS * abs(sweep))):
            add(stroke, TURN_COLOR)
    return out


@dataclass
class _Followed:
    """One bar followed live: its layout, the last sample and fit, the hint for the next fit.

    Attributes:
        layout: The bar's designed markers.
        assembled: Its assembly pose, 4x4: where it is looked for while the live scene lacks it.
        sensed: The markers near it last fitted, or None.
        fit: The last fit, ok or not; None before any.
        pose: The fitted pose when `fit` is ok, else None.
        fitted_cloud: The marker cloud last fitted (ok or by the full search), so each is fitted once.
        next_fit_at: When the next fit may run.
        last_ok: (time, 4x4 pose) of the last good fit: the hint the next fit starts from while fresh.
        planned: Where the process places the bar this tick, or None.
        deviation: The fit against `planned`, or None.
        measured_at: Capture time of the sample last fitted.
        rows: (marker rows, plate geometry) of the last good fit, computed once for the stats block; None otherwise.
    """

    layout: BarLayout
    assembled: np.ndarray
    sensed: Sensed | None = None
    fit: BarFit | None = None
    pose: Pose | None = None
    fitted_cloud: MarkerCloud | None = None
    next_fit_at: float = 0.0
    last_ok: tuple[float, np.ndarray] | None = None
    planned: Pose | None = None
    deviation: PoseDeviation | None = None
    measured_at: float = 0.0
    rows: tuple[list[MarkerRow], PlateGeometry] | None = None


@register
class MocapBarsPlugin(HuskyPlugin):
    """Fits every process bar to the raw markers near where it should be and measures its deviation from the plan."""

    name = "mocap_bars"
    tab_path = (MOCAP_TAB, "Bars")
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
        #: The cloud "Raw markers" last drew (None: hidden), and when.
        self._drawn_cloud: MarkerCloud | None = None
        self._cloud_drawn_at = -np.inf
        self._take: asyncio.Task | None = None
        self._message = ""
        self._next_log_at = 0.0
        #: The relay the cloud comes from, or None (tests).
        self._relay = None
        self._now = 0.0
        #: The settings' sliders (None before setup: the module's defaults hold).
        self._radius_mm = self._fit_period = self._marker_age = self._hint_age = self._search_retry = None
        #: What the arrows last drew (bars, poses, mode, magnify), when, and their label nodes, reused.
        self._arrows_drawn: tuple | None = None
        self._arrows_drawn_at = -np.inf
        self._arrow_labels: list = []

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
                                              "loaded process with 3 or more markers on its halves is fitted.")
            # * The chip on its own line, the fit below it: the fit's numbers change on every refit and would drop the
            #   chip's hover.
            self._status_chip: viser.GuiHtmlHandle = gui.add_html(chip_line(""))
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            self._show_markers = gui.add_checkbox("Show markers", initial_value=True)
            self._raw_markers = gui.add_checkbox("Raw markers", initial_value=False,
                                                 hint="Every marker the cameras see, small and grey.")
            self._log_live = gui.add_checkbox("Log live", initial_value=False,
                                              hint=f"Print the chosen bar's fit report every {LOG_PERIOD:.0f} s.")
            measure = gui.add_button("Measure + log", hint="Fit the chosen bar afresh and print its report.")
            self._arrows = gui.add_dropdown("Arrows", options=list(ARROW_MODES), initial_value=ARROW_MODES[1],
                                            hint="The as-built report's arrows from where the bar should be to its "
                                                 "fit: the gap at its start, middle and end and the turn about each "
                                                 "axis, magnified; the labels are the true values.")
            self._magnify = gui.add_slider("Magnify", min=1, max=50, step=1, initial_value=MAGNIFY,
                                           hint="How many times larger the arrows draw the gaps and turns.")

            gui.add_html(section("fit settings", SECTION_CTRL))
            self._radius_mm = gui.add_slider("Search mm", min=10, max=150, step=5, initial_value=SEARCH_RADIUS * 1e3,
                                             hint="Markers farther than this from every marker of a bar where it "
                                                  "should be are not its own: a bar farther off is not found. Servo "
                                                  "keeps its own 50 mm.")
            self._fit_period = gui.add_slider("Fit every s", min=0.05, max=1.0, step=0.05, initial_value=FIT_PERIOD,
                                              hint="Each bar is refitted at most this often.")
            self._marker_age = gui.add_slider("Marker age s", min=0.1, max=2.0, step=0.1,
                                              initial_value=MAX_MARKER_AGE,
                                              hint="Older markers are not fitted, and a bar fitted to them hides.")
            self._hint_age = gui.add_slider("Hint age s", min=0.2, max=5.0, step=0.1, initial_value=HINT_MAX_AGE,
                                            hint="A good fit younger than this is where the next fit starts (a "
                                                 "hidden marker allowed); after that, where the bar should be.")
            self._search_retry = gui.add_slider("Retry search s", min=0.2, max=10.0, step=0.2,
                                                initial_value=SEARCH_RETRY,
                                                hint="A bar whose full search failed waits this long for the next.")

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
        self._cloud_points = scene.add_point_cloud(f"{root}/cloud", points=np.zeros((0, 3)), colors=CLOUD_COLOR,
                                                   point_size=CLOUD_POINT_SIZE, point_shape="circle",
                                                   precision="float32", visible=False)
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
        self._scene, self._root = scene, root
        # * Every bar's arrows in one node: gaps, axes, arcs and their heads.
        self._arrow_lines = scene.add_line_segments(f"{root}/arrows", points=np.zeros((1, 2, 3)), colors=GAP_COLOR,
                                                    thickness=2.5, thickness_units="screen", visible=False)

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        self._bar.on_update(ctx.defer_value("choose bar", self._choose))
        measure.on_click(ctx.defer("measure bar", lambda: self._measure_and_log(ctx)))
        record.on_click(ctx.defer("record take", lambda: self._start_take(ctx)))

    # --- --- --- --- --- THE CONTRACT --- --- --- --- ---

    @property
    def fits(self) -> Mapping[str, BarMeasurement]:
        """Mapping[str, BarMeasurement]: Bar id -> its latest measurement, every bar fitted since the last cloud."""
        return {bar: self._measurement(bar, state) for bar, state in self._bars.items() if state.fit is not None}

    def measure_now(self, hints: Mapping[str, Pose] | None = None) -> dict[str, BarMeasurement]:
        """Fit every bar afresh from the newest cloud, near its hint (else where it should be); kept.

        The full search runs only for a bar in the live scene, as in `update`.
        """
        cloud, now = self._cloud, self._now
        if cloud is None:
            return {}
        for bar, state in self._bars.items():
            hint = hints.get(bar) if hints else None
            self._refit(state, cloud, now, None if hint is None else hint.matrix(),
                        full_search=state.planned is not None)
        return {bar: self._measurement(bar, state) for bar, state in self._bars.items() if state.fit is not None}

    def _measurement(self, bar: str, state: _Followed) -> BarMeasurement:
        """The bar's latest fit as the contract hands it on."""
        return BarMeasurement(bar, state.layout, state.fit, state.pose, state.sensed,
                              state.sensed.source if state.sensed is not None else "", state.measured_at,
                              state.fit.tracked, state.planned, state.deviation)

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _choose(self, bar_id: str) -> None:
        """Show this bar's markers and stats from now on."""
        self._bar_id = bar_id if bar_id in self._bars else None
        self._drawn = None

    def _follow_all(self, ctx: PluginContext, loaded) -> None:
        """Fit every bar of the loaded process that has a marker layout from now on, each a scene body, hidden."""
        self._bars = {}
        ctx.scene.remove_prefix(f"{self.name}/")
        layouts = bar_layouts(loaded) if loaded is not None else {}
        for bar, layout in layouts.items():
            for warning in layout.warnings:
                ctx.log_warn(warning)
            self._bars[bar] = _Followed(layout, loaded.bodies[bar].assembly_pose.matrix())
            ctx.scene.put(Body(self._body_id(bar), drawn_bar(loaded, bar), Pose(), label=f"{short(bar)} (MoCap)",
                               color=MOCAP_COLOR, enabled=False), drawn_only=True)
        self._turn = 0
        markers = max((len(layout.points) for layout in layouts.values()), default=0)
        plates = max((len({name.rsplit("/", 1)[0] for name in layout.names}) for layout in layouts.values()),
                     default=0)
        # * A summary, a row per marker, the strays, a row per plate and per spacing, the deviation.
        self._stats_rows = 3 + markers + max(2 * plates - 1, 0)
        self._bar.options = list(self._bars) or [NO_BAR]
        self._bar_id = next(iter(self._bars), None)
        self._bar.value = self._bar_id or NO_BAR
        self._drawn = None
        if self._bars:
            ctx.log_info(f"fitting {', '.join(short(bar) for bar in self._bars)} to the raw markers within "
                         f"{self.search_radius * 1e3:.0f} mm of where each should be")

    def _measure_and_log(self, ctx: PluginContext) -> None:
        """Fit the chosen bar afresh (around where it should be, else the full search) and print its report."""
        state = self._bars.get(self._bar_id)
        if state is None or self._cloud is None:
            self._message = "no marker cloud from the relay yet"
            return
        self._refit(state, self._cloud, self._now, None, full_search=True)
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
        relay = getattr(ctx.world, "mocap_relay", None)
        radius_mm = self.search_radius * 1e3
        description = {"bar": bar_id, "markers": f"the relay's raw markers within {radius_mm:.0f} mm of where the bar "
                                                 f"should be",
                       **environment(relay is not None and relay.state.simulated),
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

        def planned_deviation() -> PoseDeviation | None:
            return state.deviation

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
            Signal("deviation", lambda: (planned_deviation().across_mm / 1e3, planned_deviation().along_mm / 1e3)
                   if planned_deviation() else None, ("across", "along"), "m"),
            Signal("tilt", lambda: np.radians(planned_deviation().tilt_deg) if planned_deviation() else None, (),
                   "rad"),
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

    @property
    def _cloud(self) -> MarkerCloud | None:
        """The relay's newest marker cloud, or None without a relay or before one."""
        return self._relay.state.marker_cloud if self._relay is not None else None

    @staticmethod
    def _setting(slider, default: float) -> float:
        """A settings slider's value, or the module's default before setup."""
        return float(slider.value) if slider is not None else default

    @property
    def search_radius(self) -> float:
        """float: How far from where a bar's markers should be its markers are looked for, metres."""
        return self._setting(self._radius_mm, SEARCH_RADIUS * 1e3) / 1e3

    @property
    def max_marker_age(self) -> float:
        """float: Markers older than this are not fitted and their bar hides, seconds."""
        return self._setting(self._marker_age, MAX_MARKER_AGE)

    def update(self, ctx: PluginContext) -> None:
        """Follow the loaded process, refit each bar on a new marker cloud at most every FIT_PERIOD, compare.

        Args:
            ctx: This plugin's context.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        loaded = selection.process if selection is not None else None
        if loaded is not self._layouts_of:
            self._layouts_of = loaded
            self._follow_all(ctx, loaded)
        self._relay = getattr(ctx.world, "mocap_relay", None)
        cloud = self._cloud
        ids = selection.ids if selection is not None else None
        snapshot = ctx.scene.snapshot
        now = self._now = ctx.now()
        searches = []
        for bar, state in self._bars.items():
            live_id = ids.get(bar) if ids is not None else None
            # * Only while its body is shown: an unbuilt bar sits disabled at its seat in the live scene.
            body = snapshot.bodies.get(live_id) if live_id else None
            state.planned = snapshot.world_poses.get(live_id) if body is not None and body.enabled else None
            if cloud is None:
                state.fit = state.pose = state.rows = state.sensed = None
            elif cloud is not state.fitted_cloud and now >= state.next_fit_at:
                fresh = state.last_ok is not None and now - state.last_ok[0] <= self._setting(self._hint_age,
                                                                                              HINT_MAX_AGE)
                # * The quick match around the last fit (a marker may be hidden), else where the bar should be (all
                #   markers); a bar it misses waits for its turn at the full search.
                self._refit(state, cloud, now, state.last_ok[1] if fresh else None, full_search=False,
                            partial_hint=fresh)
                if state.fitted_cloud is not cloud:
                    searches.append(bar)
            state.deviation = (pose_deviation(state.pose.matrix(), state.planned.matrix())
                               if state.pose is not None and state.planned is not None else None)
        if searches:
            self._search_one(searches, cloud, now)
        self._place_bodies(ctx, now)
        chosen = self._bars.get(self._bar_id)
        if chosen is not None and self._log_live.value and now >= self._next_log_at and chosen.fit is not None:
            self._next_log_at = now + LOG_PERIOD
            self._log_report(ctx, self._bar_id, chosen)

    def _body_id(self, bar: str) -> str:
        """A bar's scene body: "mocap_bars/B20"."""
        return f"{self.name}/{short(bar)}"

    def _place_bodies(self, ctx: PluginContext, now: float) -> None:
        """Move each bar's body to its fit; shown only while the fit is ok and its markers under MAX_MARKER_AGE old."""
        for bar, state in self._bars.items():
            body = ctx.scene.bodies.get(self._body_id(bar))
            if body is None:
                continue
            body.enabled = state.pose is not None and now - state.measured_at <= self.max_marker_age
            if body.enabled:
                body.placement = state.pose

    def _search_one(self, bars: list[str], cloud: MarkerCloud, now: float) -> None:
        """The full search for one of `bars`, the first at or after the turn; the others wait for the next tick."""
        order = list(self._bars)
        indices = sorted(order.index(bar) for bar in bars)
        index = next((i for i in indices if i >= self._turn), indices[0])
        state = self._bars[order[index]]
        self._refit(state, cloud, now, None, full_search=True)
        self._turn = (index + 1) % len(order)
        state.deviation = (pose_deviation(state.pose.matrix(), state.planned.matrix())
                           if state.pose is not None and state.planned is not None else None)

    def _refit(self, state: _Followed, cloud: MarkerCloud, now: float, hint: np.ndarray | None,
               full_search: bool, partial_hint: bool = False) -> bool:
        """Fit a bar's layout to the cloud's markers near where it should be; True when the fit is ok.

        Where it should be is its planned pose, else its assembly pose; `hint` (else that) is matched around first,
        needing every marker unless `partial_hint` (a fresh good fit). A good fit, or a failure kept for its reason
        (the bar then waits SEARCH_RETRY), replaces the bar's fit and sample. A hint match that misses with the search
        withheld leaves the last fit as it was for the full search's turn, but only for a bar shown in the live scene
        with 3 or more markers near it: with fewer no fit is possible, and a bar not shown (not built yet; its
        neighbours' markers sit there) never gets the ~20 ms search.
        """
        where = state.planned.matrix() if state.planned is not None else state.assembled
        radius, max_age = self.search_radius, self.max_marker_age
        sensed, fit = fit_in_cloud(cloud, state.layout, where, now, radius=radius, hint=hint, max_age=max_age,
                                   full_search=full_search, hint_all_markers=not partial_hint)
        if not fit.ok and not full_search:
            if len(sensed.points) < 3:
                # * The full search says why at once.
                sensed, fit = fit_in_cloud(cloud, state.layout, where, now, radius=radius, max_age=max_age)
            elif state.planned is not None:
                return False
            else:
                fit.reason = (f"not shown in the step's scene (not built yet?): matched only around its assembly "
                              f"pose, every marker needed ({len(sensed.points)} markers near)")
        state.sensed, state.fit, state.rows = sensed, fit, None
        state.measured_at = cloud.capture_time
        state.fitted_cloud = cloud
        if fit.ok:
            state.pose = Pose.from_matrix(fit.T_world_from_bar)
            state.last_ok = (now, fit.T_world_from_bar)
            state.next_fit_at = now + self._setting(self._fit_period, FIT_PERIOD)
            return True
        state.pose, state.next_fit_at = None, now + self._setting(self._search_retry, SEARCH_RETRY)
        return False

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Show the markers' state, the fits and the deviations, and move the marker nodes.

        Args:
            ctx: This plugin's context.
        """
        self._status_chip.content = chip_line(self._cloud_chip(ctx.now()))
        self._status.content = block(values(*self._status_lines()))
        running = self._take is not None and not self._take.done()
        recording = chip("recording", BUSY, "A take is being recorded; it is saved to the Drive when its seconds are "
                                            "up.")
        self._result.content = (recording if running else "") + (
            note(escape(self._message)) if self._message else "")
        self._draw_markers()
        self._draw_cloud(ctx.now())
        self._draw_arrows(ctx.now())
        self._stats.content = self._stats_block()

    def _cloud_chip(self, now: float) -> str:
        """The marker cloud's chip: grey before one, red while stale, green while fresh; its age only in the hover."""
        if not self._bars:
            return chip("no bars", NONE, "no bar with markers in the loaded process")
        cloud = self._cloud
        if cloud is None:
            return chip("no markers", FAIL if self._relay is not None else NONE,
                        "no marker cloud from the relay yet: is it running, and new enough to send ~/markers?")
        age = self.max_marker_age
        if now - cloud.capture_time > age:
            return chip("markers stale", FAIL, f"the newest marker cloud is over {age:g} s old: is Motive streaming?")
        return chip("markers", OK, "the relay's raw markers arrive")

    def _status_lines(self) -> list[str]:
        """The bars fitted, the chosen bar's markers and fit, its deviation: always 3 lines, "—" when missing."""
        lines = self._readout()
        return lines + ["—"] * (3 - len(lines))

    def _readout(self) -> list[str]:
        """Up to 3 lines: the bars fitted, the chosen bar's counts and fit (or why it failed), its deviation."""
        if not self._bars:
            return ["no bar with markers in the loaded process"]
        fitted = [short(bar) for bar, state in self._bars.items() if state.pose is not None]
        lines = [f"fitted {len(fitted)} of {len(self._bars)}: " + (" ".join(fitted) or "none")]
        state, bar_id = self._bars.get(self._bar_id), self._bar_id
        if state is None:
            return lines
        sensed, fit = state.sensed, state.fit
        if sensed is None:
            lines.append(f"{short(bar_id)}: {len(state.layout.points)} CAD markers; no marker cloud yet")
            return lines
        if fit is None or not fit.ok:
            lines.append(f"{short(bar_id)}: {len(sensed.points)} markers near; "
                         f"{fit.reason if fit is not None else 'no fit yet'}")
            return lines
        lines.append(f"{short(bar_id)}: {len(sensed.points)} markers near; fit {fit.n_matched}/{fit.n_design}  rms "
                     f"{fit.rms_m * 1e3:.2f}  max {fit.max_residual_m * 1e3:.2f} mm")
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

    def _draw_markers(self) -> None:
        """Show the chosen bar's nearby markers, its fitted and planned markers, resent only when they changed."""
        state = self._bars.get(self._bar_id)
        sensed = state.sensed if state is not None and self._show_markers.value else None
        fit = state.fit if sensed is not None and state.fit is not None and state.fit.ok else None
        planned = state.planned if state is not None else None
        # ? Identity, not id(): a freed sample's id can be reused by the next one.
        shown = (sensed, fit, planned, self._bar_id)
        if self._drawn is not None and all(a is b for a, b in zip(self._drawn, shown)):
            return
        self._drawn = shown
        # * MEASURED: the markers the fit matched; WARN: the other markers near the bar (strays, another bar's).
        self._measured_cloud.visible = sensed is not None and len(sensed.points) > 0
        if self._measured_cloud.visible:
            matched = np.zeros(len(sensed.points), dtype=bool)
            if fit is not None:
                matched[list(fit.correspondences)] = True
            self._measured_cloud.points = sensed.points.astype(np.float32)
            self._measured_cloud.colors = np.where(matched[:, None], MEASURED_COLOR, STRAY_COLOR).astype(np.uint8)
        layout = state.layout.points if state is not None else None
        self._planned_cloud.visible = layout is not None and planned is not None and sensed is not None
        self._planned_line.visible = self._planned_label.visible = self._planned_cloud.visible
        if self._planned_cloud.visible:
            T = planned.matrix()
            self._planned_cloud.points = transform_points(T, layout).astype(np.float32)
            self._planned_line.points = bar_ends(T, state.layout.length).astype(np.float32).reshape(1, 2, 3)
            self._planned_label.text = f"{short(self._bar_id)} Rhino"
            self._planned_label.position = tuple(float(v) for v in T[:3, 3])
        self._fit_cloud.visible = self._fit_lines.visible = fit is not None
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

    def _draw_cloud(self, now: float) -> None:
        """Every marker of the newest cloud while "Raw markers" is ticked, at most every CLOUD_PERIOD."""
        cloud = self._cloud if self._raw_markers.value else None
        if cloud is self._drawn_cloud or (cloud is not None and self._drawn_cloud is not None
                                          and now - self._cloud_drawn_at < CLOUD_PERIOD):
            return
        self._drawn_cloud, self._cloud_drawn_at = cloud, now
        self._cloud_points.visible = cloud is not None
        if cloud is not None:
            self._cloud_points.points = cloud.positions.astype(np.float32)

    def _draw_arrows(self, now: float) -> None:
        """The chosen bar's (or every bar's) deviation arrows, for bars with a fresh fit and a planned pose.

        Redrawn when the mode or magnification changed at once, else at most every ARROW_PERIOD while a pose changed.
        """
        mode, magnify = self._arrows.value, float(self._magnify.value)
        names = [] if mode == ARROW_MODES[0] else [self._bar_id] if mode == ARROW_MODES[1] else list(self._bars)
        shown = []
        for bar in names:
            state = self._bars.get(bar)
            if (state is not None and state.pose is not None and state.planned is not None
                    and now - state.measured_at <= self.max_marker_age):
                shown.append((bar, state.pose, state.planned))
        key = (mode, magnify, tuple(shown))
        last = self._arrows_drawn
        if last is not None:
            same = last[:2] == key[:2] and len(last[2]) == len(shown) and all(
                a[0] == b[0] and a[1] is b[1] and a[2] is b[2] for a, b in zip(last[2], shown))
            if same or (last[:2] == key[:2] and now - self._arrows_drawn_at < ARROW_PERIOD):
                return
        self._arrows_drawn, self._arrows_drawn_at = key, now
        arrows = Arrows([], [], [])
        for bar, pose, planned in shown:
            one = deviation_arrows(planned.matrix(), pose.matrix(), self._bars[bar].layout.length, magnify,
                                   short(bar) if mode == ARROW_MODES[2] else "")
            arrows.segments += one.segments
            arrows.colors += one.colors
            arrows.labels += one.labels
        self._arrow_lines.visible = bool(arrows.segments)
        if arrows.segments:
            self._arrow_lines.points = np.array(arrows.segments, dtype=np.float32)
            self._arrow_lines.colors = np.repeat(np.array(arrows.colors, dtype=np.uint8)[:, None], 2, axis=1)
        for index, (text, position) in enumerate(arrows.labels):
            where = tuple(float(v) for v in position)
            if index == len(self._arrow_labels):
                self._arrow_labels.append(self._scene.add_label(f"{self._root}/arrow_labels/{index}", text=text,
                                                                position=where))
            label = self._arrow_labels[index]
            label.text, label.position, label.visible = text, where, True
        for label in self._arrow_labels[len(arrows.labels):]:
            label.visible = False


def _plain(name: str) -> str:
    """A marker or plate name without the "joints/" folder."""
    return name[len("joints/"):] if name.startswith("joints/") else name
