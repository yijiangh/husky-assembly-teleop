"""
A mocap probe: track one rigid body, show its live pose, and record points with it, e.g. obstacle corners.

Points are numbered per label ("table_1", ...) and shown in the 3D view. Delete and
Export act on the selected points, so one session can give one file per obstacle.
"Load points" adds the points of an exported file, unselected; a name already taken is renumbered.
A big Status tab, readable from across the room, sits in the "Big status" folder at the bottom, closed at start;
the coverage map and cameras sit in the "Debug" folder above it, also closed, and the floor calibration in its own
"Floor calibration" folder, closed at start.
The phone's Mocap tab has the same big status over two tabs: Points (Label, Record point, Map coverage, a top-down
map while mapping with the coverage map, obstacles, robots, points and probe, and the last points) and Floor.
Its delete-all buttons (every point, or the floor calibration's) and Export floor calibration act on a second tap.
An export with a warning (a mark off, an anchor not read) goes on a second press within FORCE_SECONDS, the button
orange meanwhile, on the laptop too.

Floor calibration: "Floor point" records a "floor" sample, "Mark" one of the marks on the floor (origin, mark1..3),
"Anchor" the tip near one of Motive's anchor markers: a hint, the anchor being that marker, read with the probe away
(`_anchors_now`). An anchor stored in the lab frame file is found near its stored place by itself; only one not found
there, or a new one, needs pointing at. The plane, the lab frame on it and their
quality are solved raw on each redraw (`world/floor_calibration.py`). "Export floor calibration" saves it into today's
calibration session folder the relay announces, marked real or sim by the relay's mode. The section shows the frame
the relay holds, in its states, with "Use refit" and "Apply floor calibration" (refused while anything moves).
Its "poses off" lines check the frame in use by the marks alone, beside the relay's own estimate (`_check_lines`).
"Show lab marks and anchors" shows the lab frame file's marks as rings on the floor and its anchors as see-through
spheres, sized by their cutoffs, where the frame being fitted puts them (else the relay's frame); and each anchor read
as Motive's marker, with its distance from its stored place and its noise (`_draw_read_anchors`).
Deleting or loading floor samples changes the calibration.
? The plane is where the rigid body's origin was: with a ball tip and Motive's pivot at its centre, one radius up.

Cameras: "Show cameras" draws Motive's cameras where its calibration puts them, each a wireframe of its view with
its name, its view's edges as faint rays down to the plane at "At height" and a cross where its view's middle meets
it, from the relay's announcement (`world.mocap_relay.state.cameras`); a fake Motive's in a simulation.
"Views on the floor" colours the coverage map's area by how many cameras see each point of that plane, with each
region's outline and count (`world/camera_coverage.py`: the views cut with the plane, their polygons intersected).

Coverage map: with "Map coverage" ticked, walk the probe around; every tick keeps a sample of the probe's position
and marker error (none when not tracked). The map on the floor is kernel-smoothed from them and coloured by the mean
marker error (`world/coverage_map.py`): green, deeper the lower, up to the mocap chip's amber threshold, then amber
to deep orange, red past calibration's limit; greyer where the probe was often lost, grey where never tracked, and
nothing where no samples are near. Export and Load keep the samples across runs; loading then mapping adds to them.

! The recorded point is the rigid body's origin: set the pivot in Motive to the probe tip.
! Recording is refused while the mocap chip is red; amber records, with its marker error.
! A lost probe has no position: its misses count at its last fix, so walk slowly through gaps.

Run with:  -p plugins:="['mocap_probe']"
"""

from __future__ import annotations

import asyncio
import itertools
import json
import time
from dataclasses import asdict
from html import escape
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import shapely
import viser
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ..drive import drive_folder, environment, folder_name, stamped_name, started_at
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..robot_interface.mocap_definitions import MocapAnchor, MocapCamera, camera_changes, cameras_json
from ..robot_interface.mocap_relay import (FRAME_ERROR, FRAME_HELD, FRAME_NONE, FRAME_REFIT_PENDING, FRAME_UNCHECKED,
                                           FRAME_UNTRUSTED, FloorCalibrationState)
from ..shared.execution.moving import moving_reason
from ..shared.guide.panel import set_changed
from ..ui.checklist import CheckList
from ..ui.phone import big_buttons, phone_text
from ..ui.style import (BUSY, FAIL, INFO, LEVEL_COLORS, NONE, OK, SECTION_CTRL, SECTION_SENSOR, block, check_chip,
                        chip, chip_line, note, note_line, numbers, section, values)
from ..ui.top_down import Dot, FloorGrid, render_top_down, scene_footprints
from ..world.camera_coverage import (aim_point, camera_fov, coverage_faces, footprint, outlines, plane_point,
                                     rectangle, segments, triangles, view_edges, view_rotation)
from ..world.checks import BAD, GOOD, WARN, Check
from ..world.coverage_map import (COVERAGE_BIN, COVERAGE_ORIGIN, COVERAGE_SIZE, CoverageMap,
                                  coverage_color, map_colors)
from ..world.floor_calibration import (ANCHOR_LABEL, FLOOR_LABEL, PLANE_MIN_POINTS, PLANE_STD_GOOD, FloorSolution,
                                       Sample, anchor_name, floor_calibration_json, from_raw, pointed_anchors,
                                       samples_from_json, solve_floor, to_raw)
from ..world.lab import DEFAULT_LAB, replace_text
from ..world.lab_frame import MARK_WARN, MARKS, lab_frame_file, read_lab_frame
from ..world.measured import TrackedObject
from ..world.mocap import MARKER_ERROR_LIMIT, MARKER_ERROR_WARN, mocap_check
from ..world.mocap_frame import FLOOR_PREFIX, camera_label, find_anchors, moved_camera, volume_moved

#: Name of the probe in `world.tracked_objects` and on the health panel.
PROBE_NAME = "probe"
#: Rigid-body id of the lab's probe, tracked from startup.
DEFAULT_MOCAP_ID = 1863
#: Label used when the Label field is empty.
DEFAULT_LABEL = "p"
#: Where "Export" writes the probe's measurements (points, coverage), below the Drive root; floor calibrations go to
#: the relay's calibration sessions.
EXPORT_FOLDER = Path("data_experiment") / "mocap_probe"
#: Colour of unselected recorded points in the 3D view.
SAMPLE_COLOR = (230, 60, 150)
#: Colour of the selected recorded points.
SELECTED_COLOR = (250, 200, 30)
#: Points listed on the phone, newest last.
PHONE_POINTS = 5
#: Seconds between redraws of the phone's top-down map; each redraw is sent as a PNG over the robot WiFi.
MAP_PERIOD = 0.5
#: The probe on the map: white while tracked, grey without a fix.
PROBE_DOT_COLOR, PROBE_LOST_COLOR = (255, 255, 255), (150, 150, 150)
#: How long the big status panel flashes blue after a point is recorded, seconds.
FLASH_SECONDS = 0.8
#: Blue of that flash.
FLASH_COLOR = "#1c7ed6"
#: Seconds a phone button that acts on its second tap (delete all, export) waits for it.
CONFIRM_SECONDS = 3.0
#: Seconds Export floor calibration waits for the second press that exports in spite of a warning, orange meanwhile.
FORCE_SECONDS = 2.0
#: The big status panel's word for each mocap check level.
BIG_WORDS = {GOOD: "TRACKED", WARN: "NOISY", BAD: "NO FIX"}
#: Metres between the phone map's grid lines.
MAP_LINES = 1.0
#: Height of the coverage map above the floor, metres, so it does not flicker with the grid; and how opaque it is.
COVERAGE_Z, COVERAGE_OPACITY = 0.004, 0.9
#: File name prefix of exported coverage maps.
COVERAGE_PREFIX = "mocap_coverage"
#: The marks' buttons and the label each records.
MARK_BUTTONS = dict(zip(("Origin", "Mark 1", "Mark 2", "Mark 3"), MARKS))
#: Anchor records the tip; the anchor is the unlabeled marker nearest it, within ANCHOR_REACH, metres, in the relay's
#: newest reading (under MARKERS_FRESH seconds old), one reading for all anchors (`MocapProbePlugin._anchors_now`).
#: An anchor stored in the lab frame and not pointed at is the marker nearest its stored place, as close.
#: ! A reading counts for an anchor only when it came MARKERS_AWAY seconds after the probe was last within PROBE_NEAR
#:   metres of it, longer than the relay averages one (2 s): the probe and the hand hide or shift the marker.
ANCHOR_REACH, MARKERS_FRESH = 0.10, 5.0
PROBE_NEAR, MARKERS_AWAY = 0.5, 3.0
#: How a stored anchor not found near its stored place ends, why it is not read: the anchors line asks for it. A tip
#: within POINT_REACH metres of its stored place is named after it, though farther than ANCHOR_REACH.
POINT_AT_IT, POINT_REACH = "point at it", 0.5
#: Anchors per row of the floor readout: a row "A2 3.3 moved · A3 1.1 · A4 0.9" fits a phone's width.
ANCHORS_PER_ROW = 3
#: How far each camera's wireframe reaches, metres, and its colour.
FRUSTUM_DEPTH = 0.5
FRUSTUM_COLOR = (40, 90, 200)
#: Seen by this many cameras or more is green on the floor; fewer go through yellow to red (one), none grey.
CAMERAS_FULL = 6
#: The views' regions: how opaque, and how far above their plane, metres (over the coverage map's tiles).
VIEWS_OPACITY, VIEWS_LIFT = 0.6, 0.006
#: Their outlines' colour and width, and the count labels' height, metres; faces smaller than this, m², get none.
OUTLINE_COLOR, OUTLINE_WIDTH, COUNT_HEIGHT, COUNT_MIN_AREA = (50, 50, 50), 0.01, 0.18, 0.05
#: A view's edges: faint grey rays from the camera to the plane, this long, metres, where they never meet it.
RAY_COLOR, RAY_WIDTH, RAY_LENGTH = (185, 185, 185), 0.004, 15.0
#: The lab frame file's marks and anchors in the 3D view (Show lab marks and anchors), sized by the cutoffs: each mark
#: a ring on the floor of MARK_WARN radius; each anchor a faint sphere of ANCHOR_REACH (found within) around a denser
#: one of MARK_WARN (moved beyond). Line width, metres; colours; opacities.
STORED_MARK_COLOR, STORED_LINE = (250, 140, 0), 0.0005
STORED_ANCHOR_COLOR, REACH_OPACITY, WARN_OPACITY = (40, 170, 160), 0.12, 0.5
#: An anchor's spheres once read: green with its marker within MARK_WARN of them, red beyond; orange while it is not
#: found near its stored place (the chips' colours).
WITHIN_COLOR, BEYOND_COLOR, NOT_FOUND_COLOR = (tuple(int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
                                               for hex_color in (OK, FAIL, BUSY))
#: Each anchor read: Motive's marker as a solid sphere this wide, metres, labelled with its distance from the stored
#: place drawn and its noise, taken from the relay's look for the marker within NOISE_MATCH metres of it.
READ_RADIUS, READ_COLOR, NOISE_MATCH = 0.001, (20, 90, 200), 0.005
#: Where a view's middle meets the plane: a cross this wide, metres, in the frustums' colour.
AIM_SIZE, AIM_WIDTH = 0.3, 0.015


def _error_mm(error: tuple[float, float] | None) -> str:
    """A (mean, std) marker error in metres as "0.82 ± 0.10 mm", or "–"."""
    return f"{error[0] * 1e3:.2f} ± {error[1] * 1e3:.2f} mm" if error else "–"


def relay_frame(ctx: PluginContext) -> FloorCalibrationState | None:
    """The floor calibration the relay last announced, or None before it did or without a relay."""
    relay = ctx.world.mocap_relay
    return relay.state.frame if relay is not None else None


def lab_from_raw(ctx: PluginContext) -> tuple | None:
    """What the relay applies to every raw pose now, ((x, y, z), (qx, qy, qz, qw)), or None for nothing."""
    frame = relay_frame(ctx)
    return frame.lab_from_raw if frame is not None else None


def few(names: Sequence[str], shown: int = 3) -> str:
    """Up to `shown` names, then how many more, e.g. "A1, A2, A3 +2"."""
    return ", ".join(names[:shown]) + (f" +{len(names) - shown}" if len(names) > shown else "")


def frame_chips(frame: FloorCalibrationState | None, anchors_missing: Sequence[str] = ()) -> str:
    """The relay's frame state as a chip in its own words (`doc/mocap_relay.md`), and a newer floor calibration's."""
    if frame is None:
        return chip("frame ?", NONE, "the mocap relay announced no floor calibration: not running")
    missing = f" ({few(anchors_missing)} missing now)" if anchors_missing else ""
    if frame.status == FRAME_NONE:
        html = chip("no floor calibration", NONE, "poses are raw (Motive's, Z-up), not in the lab frame: Export floor "
                                                  "calibration, then Apply floor calibration")
    elif frame.status == FRAME_ERROR:
        html = chip("frame: Drive not read", FAIL, frame.error or "the relay could not read a floor calibration")
    elif frame.status == FRAME_UNCHECKED:
        html = chip("frame unchecked", BUSY, f"{frame.floor_calibration} applied; no anchor or truss-camera fit yet, "
                                             f"so poses may be off.\nCheck Motive sees the anchors.")
    elif frame.status == FRAME_HELD:
        html = chip("frame held", OK, frame.floor_calibration + ("; refit in use" if frame.refit else "")
                    + ".\nHow far off: the 'poses off' line of Floor calibration.")
    elif frame.status == FRAME_REFIT_PENDING:
        html = chip("refit pending", BUSY, "Motive's frame moved: poses are off until Use refit.\nHow far: the "
                                           "'poses off' line of Floor calibration.")
    elif frame.status == FRAME_UNTRUSTED:
        html = chip("frame untrusted", FAIL, f"No fit agrees: poses may be off.\nCheck in Motive that the anchors are "
                                             f"seen{missing} and no truss camera was bumped.")
    else:
        html = chip(f"frame {frame.status}", NONE, "a state this monitor does not know: update it")
    if frame.error and frame.status != FRAME_ERROR:
        html += chip("Drive not read", FAIL, f"{frame.error}; the frame in use is kept")
    if frame.newer:
        moves = f" moves points up to {frame.newer_moves * 1e3:.1f} mm" if frame.newer_moves is not None else ""
        html += chip("newer floor calibration", BUSY, f"{frame.newer}{moves}: not applied until Apply floor "
                                                      f"calibration")
    return html


def redraw(drawn: tuple | None, cameras: dict, height: float) -> bool:
    """Whether cameras drawn as (cameras, height) need drawing again: others, moved (`camera_changes`), another height.

    Motive's continuous calibration moves them a little on every announcement: that is not worth a redraw.
    """
    return drawn is None or drawn[1] != height or drawn[0].keys() != cameras.keys() or bool(
        camera_changes(drawn[0], cameras))


def camera_frustum(scene: viser.SceneApi, name: str, camera: MocapCamera) -> list:
    """Draw one camera as a wireframe of its view, FRUSTUM_DEPTH deep, named the lab's way; return the handles."""
    horizontal, vertical = np.radians(camera_fov(camera.name))
    x, y, z, w = view_rotation(camera).as_quat()
    frustum = scene.add_camera_frustum(f"{name}/frustum", fov=float(vertical),
                                       aspect=float(np.tan(horizontal / 2) / np.tan(vertical / 2)),
                                       scale=FRUSTUM_DEPTH, color=FRUSTUM_COLOR, wxyz=(w, x, y, z),
                                       position=camera.position, thickness=0.006)
    label = scene.add_label(f"{name}/label", camera_label(camera.name), position=camera.position)
    return [frustum, label]


def ring(center_xy: Sequence[float], radius: float, sides: int = 24) -> np.ndarray:
    """A circle on the floor (z 0) around `center_xy`, as line segments, (sides, 2, 3)."""
    angles = np.linspace(0.0, 2.0 * np.pi, sides + 1)
    points = np.c_[center_xy[0] + radius * np.cos(angles), center_xy[1] + radius * np.sin(angles), np.zeros(sides + 1)]
    return np.stack([points[:-1], points[1:]], axis=1)


@register
class MocapProbePlugin(HuskyPlugin):
    """Tracks a mocap probe, shows its pose, and records, lists and exports points."""

    name = "mocap_probe"
    hidden = True
    tab_path = ("Mocap probe", "Points")
    icon = viser.Icon.CROSSHAIR

    def __init__(self):
        """Start with no probe tracked and no samples."""
        self._probe: TrackedObject | None = None
        self.samples: list[Sample] = []
        self._next_id = 1
        # Last number given per label, and per anchor as "anchor <name>"; never lowered, so names stay unique after a
        # delete.
        self._label_counts: dict[str, int] = {}
        # Each sample's sphere and label in the 3D view, by id.
        self._sample_handles: dict[int, tuple[viser.IcosphereHandle, viser.LabelHandle]] = {}
        # Sample ids whose sphere is drawn in SELECTED_COLOR.
        self._highlighted: set[int] = set()
        # Result of the last action on points (track, record, delete, export, coverage), and on the floor calibration.
        self._message = self._floor_message = ""
        self._message_color = self._floor_message_color = NONE
        # ROS time until which the big status panel shows the blue flash.
        self._flash_until = 0.0
        # Per button that acts on its second tap ("all", "floor", "export"): the ROS time that tap is due by, and
        # whether it exports in spite of a warning.
        self._confirm: dict[str, tuple[float, bool]] = {}
        # The lab frame file, the floor (x, y) it stores per mark and the (x, y, z) per anchor.
        self._lab_frame_path: Path | None = None
        self._ideal_marks: dict[str, np.ndarray] = {}
        self._ideal_anchors: dict[str, np.ndarray] = {}
        # ROS time the probe was last within PROBE_NEAR of each anchor pointed at: its marker is read only after.
        self._probe_near: dict[str, float] = {}
        # The relay's lab_from_raw last seen, and when it announced it (its clock): markers taken before are not used.
        self._lab_from_raw: tuple | None = None
        self._lab_from_raw_since: float | None = None
        # Coverage map, its image on the floor, and how many samples it showed when drawn at what ROS time.
        self.coverage = CoverageMap()
        self._coverage_image: viser.ImageHandle | None = None
        self._coverage_drawn, self._coverage_time = 0, -np.inf
        # `last_update_time` of the last probe message counted, so each message counts once.
        self._counted_update: float | None = None
        # ROS time the phone's map was last drawn, and the task drawing it.
        self._map_time = -np.inf
        self._map_task: asyncio.Task | None = None
        # The cameras and height drawn (the relay's announcement, compared by identity), and their frustums, labels,
        # rays and aim crosses.
        self._drawn_cameras: tuple | None = None
        self._camera_handles: list = []
        # The cameras and height the views on the floor were cut for, their regions' meshes and area by count.
        self._views_for: tuple | None = None
        self._views_handles: list = []
        self._views_areas: dict[int, float] = {}
        # Each anchor read, drawn: its marker's sphere, its label and the label's text.
        self._read: dict[str, tuple] = {}
        # Each stored anchor's two spheres, and the colour they were given last.
        self._anchor_spheres: dict[str, list] = {}
        self._anchor_tints: dict[str, tuple] = {}
        # Each stored anchor's label in the 3D view, and its text.
        self._anchor_labels: dict[str, tuple] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build the widgets and start tracking DEFAULT_MOCAP_ID.

        Args:
            ctx: This plugin's context.
        """
        self._lab_frame_path = lab_frame_file(ctx.config.data_directory, ctx.config.lab or DEFAULT_LAB)
        self._read_lab_frame(ctx)
        with ctx.view.ui() as gui:
            gui.add_html(section("probe", SECTION_SENSOR))
            self._mocap_id = gui.add_number("Mocap ID", initial_value=DEFAULT_MOCAP_ID, min=0, step=1)
            track = gui.add_button("Track", hint="Subscribe to this rigid body; replaces the previous one.")
            self._status: viser.GuiHtmlHandle = gui.add_html("")

            gui.add_html(section("record", SECTION_CTRL))
            self._label = gui.add_text("Label", initial_value="",
                                       hint=f"Name of the next point, numbered automatically: "
                                            f"table -> table_1, table_2, ... Empty gives {DEFAULT_LABEL}_<n>.")
            record = gui.add_button("Record point")

            # * Actions on the selection sit above the list, which grows at the bottom.
            gui.add_html(section("selected points", SECTION_CTRL))
            self._file_name = gui.add_text("File name", initial_value="probe_points",
                                           hint=f"Saved on the Drive as {EXPORT_FOLDER}/<date>_<time>_<name>.json")
            actions = gui.add_button_group("Selected", ["Delete", "Export"])
            load_points = gui.add_upload_button("Load points", mime_type="application/json", icon=viser.Icon.UPLOAD,
                                                hint="Add the points of an exported file, unselected.")
            self._result: viser.GuiHtmlHandle = gui.add_html("")
            self._list = CheckList(ctx, gui, "Samples", empty_text="no samples yet")

            # * Its own folder, closed at start: the relay's frame chips and Use refit sit in it too.
            with gui.add_folder("Floor calibration", expand_by_default=False):
                probe_floor = gui.add_button(
                    "Floor point", icon=viser.Icon.ARROW_BAR_TO_DOWN,
                    hint=f"Record a '{FLOOR_LABEL}' sample, fitted into the floor plane. To drop a bad one, select "
                         f"only it in the list above and Delete.")
                probe_mark = gui.add_button_group(
                    "Mark", list(MARK_BUTTONS),
                    hint=f"Record the marked point under the tip; it fits the plane too. The lab frame is fitted to "
                         f"the marks in {self._lab_frame_path.name}; probing again averages.")
                probe_anchor = gui.add_button(
                    "Anchor", icon=viser.Icon.ANCHOR,
                    hint=f"Tip on an anchor marker, press, then move the probe {PROBE_NEAR:g} m away. Only anchors "
                         f"the anchors line asks for, or a new one.")
                gui.add_html(self._marks_html())
                self._show_stored = gui.add_checkbox(
                    "Show lab marks and anchors", initial_value=False,
                    hint=f"Show {self._lab_frame_path.name}'s marks (rings) and anchors (spheres: "
                         f"{ANCHOR_REACH * 100:g} cm search, {MARK_WARN * 1e3:g} mm moved) where the fit puts them.")
                export_floor = gui.add_button(
                    "Export floor calibration", icon=viser.Icon.FILE_EXPORT,
                    hint=f"Save the floor calibration to the Drive sessions folder the relay reads. With a warning, "
                         f"press again within {FORCE_SECONDS:g} s to export anyway.")
                # * The relay's floor calibration on a line always there, then its two buttons, always shown: nothing
                # * above moves.
                self._frame_chips: viser.GuiHtmlHandle = gui.add_html(chip_line(""))
                use_refit = gui.add_button(
                    "Use refit", icon=viser.Icon.REFRESH, disabled=True,
                    hint="Motive's frame moved: put poses back in the lab frame by the anchors' fit (else the truss "
                         "cameras'). Refused while a robot moves.")
                apply_floor = gui.add_button("Apply floor calibration", icon=viser.Icon.CHECK, disabled=True,
                                             hint="A newer floor calibration is on the Drive (an export): the relay "
                                                  "applies it, and every pose moves by the difference. Refused while "
                                                  "any robot moves.")
                self._floor_status: viser.GuiHtmlHandle = gui.add_html("")
                # * The floor section's last result, wrapped: the phone has no hover to show a cut line.
                self._floor_result: viser.GuiHtmlHandle = gui.add_html("")
                self._use_refit, self._apply_floor = use_refit, apply_floor

            # * Rarely used tools in a folder closed at start, so the points' Export is not mistaken for the
            #   coverage map's; above Big status, which holds no button.
            with gui.add_folder("Debug", expand_by_default=False):
                gui.add_html(section("coverage map", SECTION_SENSOR))
                self._mapping = gui.add_checkbox("Map coverage", initial_value=False,
                                                 hint=f"Record every probe message and map the mean marker error: "
                                                      f"green ≤ {MARKER_ERROR_WARN * 1e3:g} mm, orange ≤ "
                                                      f"{MARKER_ERROR_LIMIT * 1e3:g} mm, red above, grey never "
                                                      f"tracked.")
                self._show_map = gui.add_checkbox("Show map", initial_value=True)
                coverage_actions = gui.add_button_group("Coverage", ["Export", "Clear"])
                load = gui.add_upload_button("Load coverage", mime_type="application/json", icon=viser.Icon.UPLOAD,
                                             hint="Replace the map with an exported one; mapping then adds to it.")
                self._coverage_status: viser.GuiHtmlHandle = gui.add_html("")

                gui.add_html(section("cameras", SECTION_SENSOR))
                self._show_cameras = gui.add_checkbox("Show cameras", initial_value=False,
                                                      hint="Motive's cameras where its calibration puts them, each "
                                                           "with a wireframe of its view, as the mocap relay "
                                                           "announces them.")
                self._show_views = gui.add_checkbox(
                    "Views on the floor", initial_value=False,
                    hint=f"How many cameras see each floor point at the height below: grey none, red one, green "
                         f"{CAMERAS_FULL}+. Range and occlusion ignored.")
                self._views_height = gui.add_slider("At height m", initial_value=0.0, min=-0.5, max=4.0, step=0.05,
                                                    hint="Metres above the floor: the plane the views are cut with, "
                                                         "e.g. where the markers are.")
                self._cameras_status: viser.GuiHtmlHandle = gui.add_html("")

            # * The big status tab in a folder, closed at start; last, so opening it moves no button.
            with gui.add_folder("Big status", expand_by_default=False), \
                    gui.add_tab_group().add_tab("Status", viser.Icon.EYE):
                self._big: viser.GuiHtmlHandle = gui.add_html("")

        self._floor_axes = ctx.view.scene.add_frame(f"{ctx.view.scene_root}/lab_frame", axes_length=0.3,
                                                    axes_radius=0.006, visible=False)
        # * The stored marks and anchors, in the lab frame below one node that `_draw_stored` places.
        root = f"{ctx.view.scene_root}/stored"
        self._stored = ctx.view.scene.add_frame(root, show_axes=False, visible=False)
        for name, xy in self._ideal_marks.items():
            ctx.view.scene.add_line_segments(f"{root}/{name}/ring", ring(xy, MARK_WARN), STORED_MARK_COLOR,
                                             thickness=STORED_LINE)
            ctx.view.scene.add_label(f"{root}/{name}/label", name, position=(*xy, 0.0))
        for name, xyz in self._ideal_anchors.items():
            sizes = (("reach", ANCHOR_REACH, REACH_OPACITY), ("warn", MARK_WARN, WARN_OPACITY))
            self._anchor_spheres[name] = [
                ctx.view.scene.add_icosphere(f"{root}/{name}/{kind}", radius=radius, color=STORED_ANCHOR_COLOR,
                                             opacity=opacity, position=tuple(xyz)) for kind, radius, opacity in sizes]
            self._anchor_labels[name] = (ctx.view.scene.add_label(f"{root}/{name}/label", name, position=tuple(xyz)),
                                         name)

        # * The phone: record and calibrate while walking with the probe, under the same big status.
        #   Tracking, the points' export and the list stay on the laptop.
        with ctx.view.phone("Mocap", viser.Icon.CROSSHAIR) as phone:
            big = big_buttons([viser.Icon.MAP_PIN, viser.Icon.ARROW_BAR_TO_DOWN, viser.Icon.CURRENT_LOCATION], 64, 28)
            phone.add_html(f"<style>{big}</style>")
            self._phone_big = phone.add_html("")
            views = phone.add_tab_group()
            with views.add_tab("Points", icon=viser.Icon.MAP_PIN):
                self._phone_label = phone.add_text("Label", initial_value="")
                phone_record = phone.add_button("Record point", color="green", icon=viser.Icon.MAP_PIN)
                self._phone_result = phone.add_html("")
                phone_delete_all = phone.add_button("Delete all points", color="gray", icon=viser.Icon.TRASH)
                self._phone_mapping = phone.add_checkbox("Map coverage", initial_value=False)
                # * Shown and drawn only while mapping: it costs a PNG over the robot WiFi twice a second.
                self._phone_map = phone.add_image(render_top_down(*self._map_inputs(None)), format="png",
                                                  visible=False)
                self._phone_points = phone.add_html("")
            with views.add_tab("Floor", icon=viser.Icon.ARROW_BAR_TO_DOWN):
                phone_floor = phone.add_button("Floor point", color="green", icon=viser.Icon.ARROW_BAR_TO_DOWN)
                phone_mark = phone.add_button_group("Mark", list(MARK_BUTTONS))
                phone_anchor = phone.add_button("Anchor", icon=viser.Icon.ANCHOR)
                phone.add_html(phone_text(self._marks_html()))
                self._phone_floor_status = phone.add_html("")
                phone_export_floor = phone.add_button("Export floor calibration", icon=viser.Icon.FILE_EXPORT)
                phone_delete_floor = phone.add_button("Delete floor points", color="gray", icon=viser.Icon.TRASH)
                self._phone_floor_result = phone.add_html("")

        # * Buttons that act on their second tap, with their label and colour; `draw` turns them red while they wait
        # * for that tap, orange while a second press exports in spite of a warning.
        self._confirms = {"all": [(phone_delete_all, "Delete all points", "gray")],
                          "floor": [(phone_delete_floor, "Delete floor points", "gray")],
                          "export": [(export_floor, "Export floor calibration", None),
                                     (phone_export_floor, "Export floor calibration", None)]}

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        track.on_click(ctx.defer("track probe", lambda: self._track(ctx)))
        record.on_click(ctx.defer("record point", lambda: self._record(ctx, self._label.value)))
        phone_record.on_click(ctx.defer("record point (phone)", lambda: self._record(ctx, self._phone_label.value)))
        probe_floor.on_click(ctx.defer("floor point", lambda: self._record(ctx, FLOOR_LABEL)))
        probe_mark.on_click(ctx.defer_value("mark", lambda clicked: self._record(ctx, MARK_BUTTONS[clicked])))
        phone_mark.on_click(ctx.defer_value("mark (phone)", lambda clicked: self._record(ctx, MARK_BUTTONS[clicked])))
        phone_floor.on_click(ctx.defer("floor point (phone)", lambda: self._record(ctx, FLOOR_LABEL)))
        probe_anchor.on_click(ctx.defer("anchor", lambda: self._record_anchor(ctx)))
        phone_anchor.on_click(ctx.defer("anchor (phone)", lambda: self._record_anchor(ctx)))
        export_floor.on_click(ctx.defer("export floor calibration", lambda: self._export_floor(ctx)))
        use_refit.on_click(ctx.defer("use refit", lambda: self._use_refit_now(ctx)))
        apply_floor.on_click(ctx.defer("apply floor calibration", lambda: self._apply_floor_now(ctx)))
        phone_delete_all.on_click(ctx.defer("delete all points (phone)", lambda: self._delete_all(ctx, "all")))
        phone_delete_floor.on_click(ctx.defer("delete floor points (phone)", lambda: self._delete_all(ctx, "floor")))
        phone_export_floor.on_click(ctx.defer("export floor calibration (phone)",
                                              lambda: self._export_floor(ctx, phone=True)))
        # * One map switch: the phone's checkbox sets the desktop one, whose callback runs `_set_mapping`.
        self._phone_mapping.on_update(ctx.defer_value("map coverage (phone)",
                                                      lambda on: setattr(self._mapping, "value", on)))
        actions.on_click(ctx.defer_value("selected points", lambda clicked: (
            self._delete(ctx) if clicked == "Delete" else self._export(ctx))))
        load_points.on_upload(ctx.defer_value("load points", lambda file: self._load_points(ctx, file)))
        self._mapping.on_update(ctx.defer_value("map coverage", lambda on: self._set_mapping(ctx, on)))
        coverage_actions.on_click(ctx.defer_value("coverage", lambda clicked: (
            self._export_coverage(ctx) if clicked == "Export" else self._clear_coverage(ctx))))
        load.on_upload(ctx.defer_value("load coverage", lambda file: self._load_coverage(ctx, file)))

        self._track(ctx)

    def teardown(self, ctx: PluginContext) -> None:
        """Stop tracking the probe.

        Args:
            ctx: This plugin's context.
        """
        ctx.untrack_object(PROBE_NAME)

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _track(self, ctx: PluginContext) -> None:
        """Track the rigid body in "Mocap ID", replacing any previous probe.

        Args:
            ctx: This plugin's context.
        """
        mocap_id = int(self._mocap_id.value)
        ctx.untrack_object(PROBE_NAME)
        # * No geometry: a frame, not an obstacle.
        self._probe = ctx.track_object(PROBE_NAME, mocap_id, label="mocap probe")
        self._show(f"tracking mocap id {mocap_id}")
        ctx.log_info(self._message)

    def _record(self, ctx: PluginContext, label: str) -> None:
        """Record the probe position as a new, selected sample, unless the mocap check is red.

        Args:
            ctx: This plugin's context.
            label: The Label field of the page that asked, desktop or phone.
        """
        check = self._check(ctx)
        label = label.strip() or DEFAULT_LABEL
        floor = label == FLOOR_LABEL or label in MARKS
        if check is None or check.level == BAD:
            # ! Never record a stale or invalid pose: it would silently give a wrong corner.
            self._report(ctx, f"not recorded: {check.detail if check else 'probe not tracked'}", floor, warn=True)
            return
        sample = self._tip_sample(ctx, self._next_name(label, label), label)
        self._report(ctx, f"recorded {sample.name} at {numbers(sample.position, 3, 8, 4)} m", floor, color=OK)

    def _tip_sample(self, ctx: PluginContext, name: str, label: str) -> Sample:
        """Add the probe tip as a new, selected sample and flash the status; the probe must be tracked."""
        sample = Sample(id=0, name=name, label=label,
                        position=self._probe.position.tolist(), orientation=self._probe.orientation.tolist(),
                        mocap_id=self._probe.mocap_id, marker_error=self._probe.marker_error, stamp=ctx.now(),
                        raw_position=to_raw(lab_from_raw(ctx), self._probe.position).tolist())
        self._add_sample(ctx, sample, selected=True)
        self._flash_until = ctx.now() + FLASH_SECONDS
        return sample

    def _record_anchor(self, ctx: PluginContext) -> None:
        """Record the probe tip as an anchor sample, a hint where its marker is, unless the mocap check is red.

        Named after an anchor already pointed at within ANCHOR_REACH (it averages), else the lab frame's anchor stored
        nearest the tip within ANCHOR_REACH, else the one the anchors line asks for nearest it within POINT_REACH, else
        the next free "A<n>".
        """
        check = self._check(ctx)
        if check is None or check.level == BAD:
            self._report(ctx, f"no anchor: {check.detail if check else 'probe not tracked'}", floor=True, warn=True)
            return
        name = self._anchor_name_for(ctx, self._probe.position)
        sample = self._tip_sample(ctx, self._next_name(name, f"{ANCHOR_LABEL} {name}"), ANCHOR_LABEL)
        self._probe_near[name] = ctx.now()
        self._report(ctx, f"anchor {name}: tip at {numbers(sample.position, 3, 8, 4)} m; its marker is read once the "
                          f"probe is {PROBE_NEAR:g} m away", floor=True)

    def _next_name(self, prefix: str, key: str) -> str:
        """`prefix` with the next number counted under `key`: a label, or "anchor <name>" for an anchor's samples."""
        self._label_counts[key] = self._label_counts.get(key, 0) + 1
        return f"{prefix}_{self._label_counts[key]}"

    def _anchor_name_for(self, ctx: PluginContext, position: np.ndarray) -> str:
        """The name of the anchor at `position` (the lab frame the relay publishes): see `_record_anchor`."""
        for sample in self.samples:
            if sample.label == ANCHOR_LABEL and np.linalg.norm(np.subtract(sample.position, position)) <= ANCHOR_REACH:
                return anchor_name(sample)
        if self._ideal_anchors:
            name, ideal = min(self._ideal_anchors.items(), key=lambda item: np.linalg.norm(item[1] - position))
            if np.linalg.norm(ideal - position) <= ANCHOR_REACH:
                return name
        asked = [name for name, why in self._anchors_now(ctx)[1].items() if why.endswith(POINT_AT_IT)]
        nearest = min(asked, key=lambda name: np.linalg.norm(self._ideal_anchors[name] - position), default=None)
        if nearest is not None and np.linalg.norm(self._ideal_anchors[nearest] - position) <= POINT_REACH:
            return nearest
        taken = set(self._ideal_anchors) | {anchor_name(s) for s in self.samples if s.label == ANCHOR_LABEL}
        return next(f"A{n}" for n in itertools.count(1) if f"A{n}" not in taken)

    def _load_points(self, ctx: PluginContext, file: viser.UploadedFile) -> None:
        """Add the samples of an exported file, unselected; names already taken get the next number of their label.

        Args:
            ctx: This plugin's context.
            file: The uploaded JSON file.
        """
        try:
            loaded = samples_from_json(json.loads(file.content))
        except ValueError as error:  # * json.JSONDecodeError is a ValueError
            self._show(f"not loaded {file.name}: {error}", FAIL)
            ctx.log_warn(self._message)
            return
        taken = {sample.name for sample in self.samples}
        renamed = 0
        for sample in loaded:
            # * An anchor sample is numbered per anchor ("A3_2"), any other per label ("table_3").
            prefix = anchor_name(sample) if sample.label == ANCHOR_LABEL else sample.label
            key = f"{ANCHOR_LABEL} {prefix}" if sample.label == ANCHOR_LABEL else prefix
            # * Keep the numbering going: the next recorded table becomes table_4.
            head, _, number = sample.name.rpartition("_")
            if head == prefix and number.isdigit():
                self._label_counts[key] = max(self._label_counts.get(key, 0), int(number))
            if sample.name in taken:
                sample.name = self._next_name(prefix, key)
                renamed += 1
            taken.add(sample.name)
            self._add_sample(ctx, sample, selected=False)
        self._show(f"loaded {len(loaded)} points from {file.name}" + (f", {renamed} renamed" if renamed else ""), OK)
        ctx.log_info(self._message)

    def _delete(self, ctx: PluginContext) -> None:
        """Delete the selected samples.

        Args:
            ctx: This plugin's context.
        """
        doomed = self._selected_samples()
        if not doomed:
            self._show("nothing selected to delete")
            return
        self._remove_samples(ctx, doomed)

    def _delete_all(self, ctx: PluginContext, which: str) -> None:
        """Delete every sample, or only the floor calibration's, on the second tap within CONFIRM_SECONDS.

        Args:
            ctx: This plugin's context.
            which: "all", or "floor" for the floor calibration's samples (floor points, marks, anchors).
        """
        doomed, floor = self._deletable(which), which == "floor"
        if not doomed:
            self._report(ctx, "no points to delete", floor, log=False)
        elif not self._second_tap(ctx, which):
            self._report(ctx, f"tap {self._confirms[which][0][1]} again within {CONFIRM_SECONDS:g} s to delete "
                              f"{len(doomed)} points", floor, log=False, color=BUSY)
        else:
            self._remove_samples(ctx, doomed, floor)

    def _second_tap(self, ctx: PluginContext, which: str, seconds: float = CONFIRM_SECONDS,
                    anyway: bool = False) -> bool:
        """Whether a tap on a button that acts on its second tap (`_confirms`) is that one; else arm it for `seconds`.

        `anyway`: the tap exports in spite of a warning; a tap armed without one does not count for it.
        """
        due, armed_anyway = self._confirm.pop(which, (-np.inf, False))
        if ctx.now() <= due and (armed_anyway or not anyway):
            return True
        self._confirm[which] = (ctx.now() + seconds, anyway)
        return False

    def _remove_samples(self, ctx: PluginContext, doomed: list[Sample], floor: bool = False) -> None:
        """Remove `doomed` from the samples, the list and the 3D view; say so in the floor section if `floor`."""
        for sample in doomed:
            self.samples.remove(sample)
            self._list.remove(sample.id)
            self._highlighted.discard(sample.id)
            for handle in self._sample_handles.pop(sample.id):
                handle.remove()
        self._report(ctx, f"deleted {', '.join(sample.name for sample in doomed)}", floor)

    def _export(self, ctx: PluginContext) -> None:
        """Write the selected samples to a timestamped JSON file in the Drive's EXPORT_FOLDER.

        Args:
            ctx: This plugin's context.
        """
        chosen = self._selected_samples()
        if not chosen:
            self._show("nothing selected to export")
            return
        path = self._export_path(ctx, folder_name(self._file_name.value, "probe_points"))
        if path is None:
            return
        data = {
            "frame": "the lab frame, as the relay publishes poses (raw_position: raw, Motive's Z-up, the relay's floor "
                     "calibration taken off); metres; quaternions are (x, y, z, w)",
            "exported_at": started_at(),
            **environment(self._simulated(ctx)),
            "samples": [asdict(sample) for sample in chosen],
        }
        path.write_text(json.dumps(data, indent=2))
        self._show(f"exported {len(chosen)} samples to {path}", OK)
        ctx.log_info(self._message)

    def _export_floor(self, ctx: PluginContext, phone: bool = False) -> None:
        """Write the floor calibration and its samples to a new file in today's session the relay reads, whole.

        Args:
            ctx: This plugin's context.
            phone: Pressed on the phone: it writes on the second tap within CONFIRM_SECONDS.
        """
        solution = self._solve(ctx)
        problem = self._floor_problem(ctx, solution)
        if problem:
            self._report(ctx, f"not exported: {problem}", floor=True, warn=True)
            return
        relay, frame = ctx.world.mocap_relay, relay_frame(ctx)
        path = self._floor_path(ctx, frame)
        if path is None:
            return
        warning = self._floor_warning(ctx, solution)
        if warning and not self._second_tap(ctx, "export", FORCE_SECONDS, anyway=True):
            self._report(ctx, f"not exported: {warning}; press again within {FORCE_SECONDS:g} s to export anyway",
                         floor=True, warn=True, color=BUSY)
            return
        if phone and not warning and not self._second_tap(ctx, "export"):
            self._report(ctx, f"tap {self._confirms['export'][1][1]} again within {CONFIRM_SECONDS:g} s to export",
                         floor=True, log=False, color=BUSY)
            return
        data = {**floor_calibration_json(solution, self._lab_frame_path.name), **environment(self._simulated(ctx))}
        # * The cameras raw, like the samples and anchors: the relay's floor calibration taken off.
        back = invert(Pose(*frame.lab_from_raw)) if frame is not None and frame.lab_from_raw else Pose()
        cameras = {name: moved_camera(back, camera) for name, camera in relay.state.cameras.items()}
        data["cameras"] = json.loads(cameras_json(list(cameras.values()), 0.0))["cameras"]
        for name, entry in data["marks"].items():
            entry["stored_xy"] = self._ideal_marks[name].tolist() if name in self._ideal_marks else None
        # * Kept for later: an old floor calibration out of date is the usual reason to calibrate again.
        data["relay_frame"] = ({"status": frame.status, "floor_calibration": frame.floor_calibration,
                                "pending": frame.pending, "error": frame.error} if frame else None)
        path.parent.mkdir(exist_ok=True)
        # * Swapped in whole: the relay reads the newest file and must never find half of one.
        replace_text(path, json.dumps(data, indent=2))
        self._confirm.pop("export", None)
        # * The relay applies it only on command (Apply floor calibration): whatever was measured against an
        #   out-of-date frame is off by its move.
        replaced = (f"; it replaces {frame.floor_calibration or 'the old floor calibration'}, whose poses were off "
                    f"({frame.pending or frame.error or frame.status})" if frame and frame.poses_off else "")
        (ctx.log_warn if replaced else ctx.log_info)(
            f"exported the floor calibration ({len(solution.samples)} points, {len(solution.anchors)} anchors, "
            f"{len(cameras)} cameras) to {path}; {self._anchors_line(ctx, solution)}; {self._apart_line(solution)}; "
            f"{'; '.join(self._check_lines(ctx, solution))}{replaced}"
            + (f"; exported in spite of: {warning}" if warning else ""))
        self._show(f"exported {path.name}" + (" with a warning" if warning else "")
                   + "; press Apply floor calibration to use it", BUSY if warning else OK, floor=True)

    def _floor_problem(self, ctx: PluginContext, solution: FloorSolution) -> str:
        """Why the floor calibration cannot be exported, or "": the frame unplaced, or no cameras from the relay."""
        if solution.transform is None:
            return solution.problem
        relay = ctx.world.mocap_relay
        if relay is None or not relay.state.cameras:
            # * The relay follows Motive's frame by the cameras stored with it, and passes over a file without any.
            return ("no cameras from the mocap relay (not running, or Motive not answering): the relay ignores a floor "
                    "calibration without them")
        return ""

    def _floor_warning(self, ctx: PluginContext, solution: FloorSolution) -> str:
        """What is wrong with a floor calibration that can still be exported, or "": a mark off, anchors not read."""
        warnings = []
        if solution.worst is not None and solution.worst[1] > MARK_WARN:
            name, off = solution.worst
            warnings.append(f"{name} lands {off * 1e3:.1f} mm from where it is stored: probe it again (hit badly?), "
                            f"delete its samples, or check the mark")
        unread = {}
        for name, why in self._anchors_now(ctx)[1].items():
            unread.setdefault(why, []).append(name)
        if unread:
            listed = "; ".join(f"{', '.join(names)}: {why}" for why, names in unread.items())
            warnings.append(f"anchors not read ({listed}): each is the unlabeled marker nearest where it was pointed "
                            f"at or is stored, all read at once with the probe away; move the probe away, point at "
                            f"it, check Motive, or delete its samples")
        return "; ".join(warnings)

    def _floor_path(self, ctx: PluginContext, frame: FloorCalibrationState | None) -> Path | None:
        """A new floor calibration file in today's session of the sessions folder the relay reads; None (and why)."""
        problem = "the mocap relay announced no floor calibration folder (not running, or an older relay)"
        if frame is not None and frame.folder:
            try:
                folder = drive_folder(ctx.config.drive_root, Path(frame.folder)) / time.strftime("%Y%m%d")
                return folder / f"{stamped_name(FLOOR_PREFIX)}.json"
            except FileNotFoundError as error:
                problem = str(error)
        self._report(ctx, f"not exported: {problem}", floor=True, warn=True)
        return None

    def _simulated(self, ctx: PluginContext) -> bool:
        """Whether the relay reads a simulation (`sim:=true`): then whatever the probe measures is simulated."""
        relay = ctx.world.mocap_relay
        return bool(relay is not None and relay.state.simulated)

    def _use_refit_now(self, ctx: PluginContext) -> None:
        """Ask the relay to use its pending refit, unless anything moves: every pose it publishes changes."""
        frame = relay_frame(ctx)
        if frame is None or frame.status != FRAME_REFIT_PENDING:
            self._report(ctx, f"no refit pending: the frame is {frame.status if frame else 'not announced'}",
                         floor=True)
            return
        self._ask_relay(ctx, "use refit", "refit not used", ctx.world.mocap_relay.use_refit)

    def _apply_floor_now(self, ctx: PluginContext) -> None:
        """Ask the relay to apply the newer floor calibration, unless anything moves: every pose it publishes moves."""
        frame = relay_frame(ctx)
        if frame is None or not frame.newer:
            self._report(ctx, "no newer floor calibration to apply: the relay applies the newest on the Drive",
                         floor=True)
            return
        self._ask_relay(ctx, "apply floor calibration", "floor calibration not applied",
                        ctx.world.mocap_relay.reload_floor_calibration)

    def _ask_relay(self, ctx: PluginContext, what: str, refused: str, call) -> None:
        """Call one of the relay's services that move every pose (`call`), unless anything moves; report its answer."""
        why = moving_reason(ctx.world, ctx.now())
        future = None if why else call()
        if why:
            self._report(ctx, f"{refused}: {why}; every pose changes, so only while nothing moves", floor=True,
                         warn=True)
        elif future is None:
            self._report(ctx, f"{refused}: the mocap relay does not answer", floor=True, warn=True)
        else:
            ctx.spawn(what, self._answered(ctx, what, future))

    async def _answered(self, ctx: PluginContext, what: str, future) -> None:
        """Report the relay's answer to Use refit or Apply floor calibration."""
        response = await ctx.ros(future)
        self._show(response.message, OK if response.success else FAIL, floor=True)
        (ctx.log_info if response.success else ctx.log_warn)(f"{what}: {response.message}")

    def _set_mapping(self, ctx: PluginContext, on: bool) -> None:
        """Start or stop counting probe messages into the coverage map; shows the map on start."""
        # Unchanged values fire no callback, so this ends the round trip with the phone's checkbox.
        self._phone_mapping.value = on
        self._phone_map.visible = on
        self._map_time = -np.inf  # draw at once, not from the last time it was mapping
        if on:
            self._show_map.value = True
        self._counted_update = None
        self._show("coverage mapping " + ("started" if on else "stopped"))
        ctx.log_info(self._message)

    def _export_path(self, ctx: PluginContext, name: str) -> Path | None:
        """A new file "<date>_<time>_<name>.json" in the Drive's EXPORT_FOLDER; None (and why) if that is missing."""
        try:
            return drive_folder(ctx.config.drive_root, EXPORT_FOLDER) / f"{stamped_name(name)}.json"
        except FileNotFoundError as error:
            self._show(f"not exported: {error}", FAIL)
            ctx.log_warn(self._message)
            return None

    def _export_coverage(self, ctx: PluginContext) -> None:
        """Write the coverage map to a timestamped JSON file in the Drive's EXPORT_FOLDER."""
        if not self.coverage.count:
            self._show("coverage map is empty; tick Map coverage and walk the probe around")
            return
        path = self._export_path(ctx, COVERAGE_PREFIX)
        if path is None:
            return
        data = {"frame": "the lab frame, as the relay publishes poses; metres; a sample per monitor tick while mapping",
                "exported_at": started_at(), **environment(self._simulated(ctx)),
                "mocap_id": self._probe.mocap_id if self._probe else None,
                "error": "the probe's mean marker error, metres; null when not tracked",
                **self.coverage.to_json()}
        # * Compact: thousands of rows.
        path.write_text(json.dumps(data))
        self._show(f"exported coverage of {self.coverage.count} samples to {path}", OK)
        ctx.log_info(self._message)

    def _clear_coverage(self, ctx: PluginContext) -> None:
        """Empty the coverage map."""
        self.coverage = CoverageMap()
        self._coverage_drawn, self._coverage_time = -1, -np.inf
        self._show("coverage map cleared")
        ctx.log_info(self._message)

    def _load_coverage(self, ctx: PluginContext, file: viser.UploadedFile) -> None:
        """Replace the coverage map with an exported one."""
        try:
            coverage = CoverageMap.from_json(json.loads(file.content))
        except (ValueError, KeyError, TypeError, IndexError) as error:
            self._show(f"not loaded {file.name}: {error}", FAIL)
            ctx.log_warn(self._message)
            return
        self.coverage = coverage
        self._coverage_drawn, self._coverage_time = -1, -np.inf
        self._show_map.value = True
        self._show(f"loaded coverage of {coverage.count} samples from {file.name}", OK)
        ctx.log_info(self._message)

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Note the anchors the probe is near, and count the latest probe message in the coverage map while mapping.

        Args:
            ctx: This plugin's context.
        """
        probe = self._probe
        if probe is not None and probe.position is not None:
            here = to_raw(lab_from_raw(ctx), probe.position)
            for name, hint in self._anchor_hints(ctx).items():
                if np.linalg.norm(hint - here) < PROBE_NEAR:
                    self._probe_near[name] = ctx.now()
        if not self._mapping.value or probe is None or probe.position is None:
            return
        if probe.last_update_time is None or probe.last_update_time == self._counted_update:
            return
        self._counted_update = probe.last_update_time
        tracked = bool(probe.tracked and probe.tracking_valid)
        # ? A tracked message without a marker error counts as not tracked; the relay always sends one.
        error = probe.marker_error if tracked else None
        self.coverage.add(probe.position, error, ctx.now())

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _report(self, ctx: PluginContext, text: str, floor: bool = False, warn: bool = False,
                log: bool = True, color: str | None = None) -> None:
        """Show `text` below the floor section's buttons (`floor`) or the selected points'; log it if `log`.

        Args:
            ctx: This plugin's context.
            text: The line.
            floor: Below the floor section's buttons, else the selected points'.
            warn: A failure: red, logged as a warning.
            log: Log it.
            color: The line's colour instead, e.g. BUSY for a question.
        """
        self._show(text, color or (FAIL if warn else NONE), floor)
        if log:
            (ctx.log_warn if warn else ctx.log_info)(text)

    def _show(self, text: str, color: str = NONE, floor: bool = False) -> None:
        """Show `text` in `color` (FAIL only for a failure) below the floor section's buttons or the points'."""
        if floor:
            self._floor_message, self._floor_message_color = text, color
        else:
            self._message, self._message_color = text, color

    def _selected_samples(self) -> list[Sample]:
        """The samples ticked in the list, in recording order."""
        return [sample for sample in self.samples if self._list.is_selected(sample.id)]

    def _deletable(self, which: str) -> list[Sample]:
        """What a phone delete-all button deletes: every sample for "all", the floor calibration's for "floor"."""
        return [sample for sample in self.samples
                if which == "all" or sample.label in (FLOOR_LABEL, ANCHOR_LABEL) or sample.label in MARKS]

    def _marks_html(self) -> str:
        """Where the stored marks and anchors are, in small gray lines; read once, so they never change."""
        if not self._ideal_marks and not self._ideal_anchors:
            return note(f"no marks or anchors stored in {escape(self._lab_frame_path.name)}")
        # * Under 40 characters, so a line fits a phone's width.
        return block(values(*(f"{name:<6} x {xy[0]:+6.3f} y {xy[1]:+6.3f} m" for name, xy in self._ideal_marks.items()),
                            *(f"{name:<6} x {xyz[0]:+6.3f} y {xyz[1]:+6.3f} z {xyz[2]:+6.3f} m"
                              for name, xyz in self._ideal_anchors.items()), dim=True))

    def _read_lab_frame(self, ctx: PluginContext) -> None:
        """Read the stored marks and anchors; none (and why) if the file cannot be read."""
        try:
            lab_frame = read_lab_frame(self._lab_frame_path)
            self._ideal_marks, self._ideal_anchors = lab_frame.marks, lab_frame.anchors
        except (ValueError, KeyError, TypeError) as error:  # * json.JSONDecodeError is a ValueError
            self._ideal_marks, self._ideal_anchors = {}, {}
            self._report(ctx, f"lab frame not read: {error}", floor=True, warn=True)

    def _solve(self, ctx: PluginContext) -> FloorSolution:
        """The floor calibration from the samples, the lab frame's stored marks and anchors, and the anchors now."""
        return solve_floor(self.samples, self._ideal_marks, self._ideal_anchors, self._anchors_now(ctx)[0])

    def _anchor_hints(self, ctx: PluginContext) -> dict[str, np.ndarray]:
        """Where each anchor's marker is looked for, raw: its tips' mean where pointed at, else its stored place.

        A stored place counts only while the relay applies a floor calibration: it is in the lab frame.
        """
        transform = lab_from_raw(ctx)
        stored = {name: to_raw(transform, xyz) for name, xyz in self._ideal_anchors.items()} if transform else {}
        return {**stored, **pointed_anchors(self.samples)}

    def _anchors_now(self, ctx: PluginContext) -> tuple[dict[str, np.ndarray], dict[str, str]]:
        """Each anchor looked for (`_anchor_hints`) where the relay's newest reading has its marker, raw; why not, else.

        The marker is the one nearest the hint within ANCHOR_REACH, in a reading that came MARKERS_AWAY after the probe
        was last near it; none while the relay's markers are not usable (`_unlabeled_markers`).
        """
        hints, transform = self._anchor_hints(ctx), lab_from_raw(ctx)
        pointed = pointed_anchors(self.samples)
        markers, stale = self._unlabeled_markers(ctx)
        markers = [to_raw(transform, marker) for marker in markers]
        found = find_anchors({name: MocapAnchor(name, tuple(position)) for name, position in hints.items()}, markers,
                             ANCHOR_REACH)
        relay = ctx.world.mocap_relay
        arrived = relay.state.markers_time if relay is not None and relay.state.markers_time is not None else -np.inf
        missing = {}
        for name in hints:
            # * A stored anchor not found where it is stored is asked for even with the probe near: it is being pointed.
            if name not in pointed and name not in found and not stale:
                missing[name] = (f"no unlabeled marker within {ANCHOR_REACH * 100:.0f} cm of its stored place: "
                                 f"{POINT_AT_IT}")
            elif arrived - self._probe_near.get(name, -np.inf) <= MARKERS_AWAY:
                missing[name] = "probe near"
            elif name not in found:
                missing[name] = stale or f"no unlabeled marker within {ANCHOR_REACH * 100:.0f} cm"
        return {name: markers[index] for name, index in found.items() if name not in missing}, missing

    def _unlabeled_markers(self, ctx: PluginContext) -> tuple[np.ndarray, str]:
        """The relay's newest unlabeled markers, (n, 3) in the frame it publishes; none, and why, while not usable.

        Not usable: older than MARKERS_FRESH, or taken before the relay's lab_from_raw last changed (Use refit, Apply
        floor calibration): taken back to raw by the new one, they would be off by the change.
        """
        relay = ctx.world.mocap_relay
        state = relay.state if relay is not None else None
        frame = state.frame if state is not None else None
        lab_from_raw = frame.lab_from_raw if frame is not None else None
        if lab_from_raw != self._lab_from_raw:
            self._lab_from_raw, self._lab_from_raw_since = lab_from_raw, frame.time if frame is not None else None
        none = np.empty((0, 3))
        if state is None or state.markers_time is None or ctx.now() - state.markers_time > MARKERS_FRESH:
            return none, ("no unlabeled markers from the mocap relay (is it running, and Unlabeled Markers on in "
                          "Motive's streaming settings?)")
        since, stamp = self._lab_from_raw_since, state.markers_stamp
        if since is not None and stamp is not None and stamp < since:
            return none, "the relay's markers are from before its floor calibration changed: try again in a moment"
        return np.asarray(state.unlabeled_markers, dtype=float).reshape(-1, 3), ""

    def _anchors_line(self, ctx: PluginContext, solution: FloorSolution) -> str:
        """The anchors on one line, for the log (`_anchor_entries`)."""
        return "anchors mm: " + (" · ".join(self._anchor_entries(ctx, solution)) or "–")

    def _anchor_entries(self, ctx: PluginContext, solution: FloorSolution) -> list[str]:
        """Each anchor looked for: mm off its stored place in 3D, or "moved", "new", "point" or "–".

        "moved": left out of the fit; "new": not stored; "point": not found near its stored place; "–": not read.
        """
        missing = self._anchors_now(ctx)[1]
        names = list(self._anchor_hints(ctx))
        offsets = {**solution.offsets, **solution.moved}

        def shown(name: str) -> str:
            """One anchor's entry."""
            if name not in solution.anchors:
                return "point" if missing.get(name, "").endswith(POINT_AT_IT) else "–"
            if name not in offsets:
                return "new"
            return f"{offsets[name] * 1e3:.1f}" + (" moved" if name in solution.moved else "")
        return [f"{name} {shown(name)}" for name in names]

    def _anchor_rows(self, entries: list[str]) -> list[str]:
        """The anchors' readout lines: a heading, then ANCHORS_PER_ROW entries a row.

        ! As many rows as the stored anchors need, set at setup: an anchor not stored lengthens a row, never adds one.
        """
        rows = max(1, -(-len(self._ideal_anchors) // ANCHORS_PER_ROW))
        per_row = max(ANCHORS_PER_ROW, -(-len(entries) // rows))
        lines = [" · ".join(entries[row * per_row:(row + 1) * per_row]) for row in range(rows)]
        # * Indented, which also keeps an empty row one line high.
        return ["anchors mm:", *(f"  {line or '–'}" if row == 0 else f"  {line}" for row, line in enumerate(lines))]

    @staticmethod
    def _apart_line(solution: FloorSolution) -> str:
        """How far the marks alone and the anchors alone place the lab apart."""
        return f"marks vs anchors: {'–' if solution.apart is None else f'{solution.apart * 1e3:.1f}'} mm apart"

    @staticmethod
    def _check_lines(ctx: PluginContext, solution: FloorSolution) -> list[str]:
        """How far poses are off the lab frame now at the lab volume's worst corner, by the marks and by the relay.

        Both against the frame the relay applies: the marks alone (`FloorSolution.marks_off`), the relay's trusted fit
        from its last look (`MocapRelayState.lab_from_raw_now`), and how far those two place the lab apart.
        """
        relay, applied = ctx.world.mocap_relay, lab_from_raw(ctx)
        state = relay.state if relay is not None else None
        fresh = state is not None and state.metrics_time is not None and ctx.now() - state.metrics_time <= MARKERS_FRESH
        estimate = state.lab_from_raw_now if fresh else None
        applied = Pose(*applied) if applied else None

        def mm(metres: float | None) -> str:
            """Metres as mm, or "–"."""
            return "–" if metres is None else f"{metres * 1e3:.1f}"

        by_marks = solution.marks_off(applied) if applied else None
        by_relay = volume_moved(compose(estimate, invert(applied))) if estimate and applied else None
        apart = solution.marks_off(estimate) if estimate else None
        return [f"poses off mm: marks {mm(by_marks)} · relay {mm(by_relay)}", f"marks vs relay: {mm(apart)} mm apart"]

    def _floor_html(self, solution: FloorSolution, height: float | None, anchors: list[str],
                    check: list[str], not_found: Sequence[str] = ()) -> str:
        """The floor calibration's chips and readout lines, always all there so the buttons never move.

        ! Keep each line under 40 characters, so it fits a phone's width.

        Args:
            solution: The floor calibration from the current samples.
            height: The probe's height above the plane, metres, or None without a plane or a fix.
            anchors: The anchors' entries (`_anchor_entries`).
            check: The poses off lines (`_check_lines`).
            not_found: The stored anchors not found near their stored place (`_not_found`): an amber chip.
        """
        plane, fit, worst = solution.plane, solution.fit, solution.worst
        if plane is None:
            state = chip("no fit", NONE, "probe at least four floor points, spread over the area")
        elif not plane.is_usable():
            state = chip("fit unusable", FAIL, "the points sit in one spot or on a line, or not on the floor")
        else:
            tilt = f"tilt {np.degrees(plane.tilt):.3f}° from Motive's ground plane"
            state = chip("fit good", OK, tilt) if plane.is_good() else chip(
                "fit noisy", BUSY, f"needs {PLANE_MIN_POINTS} points and the normal within "
                                   f"{np.degrees(PLANE_STD_GOOD):g}° per axis: add points farther apart; {tilt}")
        if solution.transform is None:
            state += chip("no frame", NONE, solution.problem)
        elif fit is None:
            state += chip("frame not fixed", BUSY, f"No marks or anchors in {self._lab_frame_path.name}: the origin "
                                                   f"mark and Motive's x place the frame.\nWrite them into the file "
                                                   f"to fix it.")
        elif worst[1] > MARK_WARN:
            state += chip(f"{worst[0]} off", BUSY, f"{worst[1] * 1e3:.1f} mm from where it is stored: probe it again, "
                                                   f"delete its samples, or check the mark; Export twice exports "
                                                   f"anyway")
        else:
            moved = list(solution.moved)
            state += chip(f"fixed {len(fit.residuals)}/{len(self._ideal_marks) + len(self._ideal_anchors)}",
                          BUSY if moved else OK,
                          f"Fitted to the marks and anchors of {self._lab_frame_path.name}"
                          + (f"; {len(moved)} left out as moved ({few(moved)})" if moved else ""))
        if not_found:
            state += chip(f"{not_found[0]} not found" if len(not_found) == 1 else f"{len(not_found)} anchors not found",
                          BUSY, f"{few(not_found)}: no marker within {ANCHOR_REACH * 100:.0f} cm of the stored place."
                                f"\nPoint the probe at it, press Anchor, move it away.")
        # * Per mark: mm off its stored place, "new" when probed but not stored, "–" when not probed.
        marks = " · ".join(f"{name.removeprefix('mark')} " + ("–" if name not in solution.measured else
                                                              f"{fit.residuals[name] * 1e3:.1f}"
                                                              if fit and name in fit.residuals else "new")
                           for name in MARKS)
        if plane is None:
            floor = ["floor: – pts · rms – · max – mm", "floor normal: ±– per axis"]
        else:
            normal_std = f"{np.degrees(plane.normal_std):.3f}°" if np.isfinite(plane.normal_std) else "∞"
            floor = [f"floor: {len(plane.residuals)} pts · rms {plane.rms * 1e3:.1f} · max "
                     f"{plane.max_error * 1e3:.1f} mm", f"floor normal: ±{normal_std} per axis"]
        probe = f"probe above floor: {numbers(None if height is None else (height * 1e3,), 1, 6, 1)} mm"
        return chip_line(state) + block(values(*floor, "marks mm:", f"  {marks}", *self._anchor_rows(anchors),
                                               self._apart_line(solution), *check, probe))

    def _check(self, ctx: PluginContext) -> Check | None:
        """The probe's mocap check (same as the health panel's), or None when nothing is tracked."""
        if self._probe is None:
            return None
        return mocap_check(f"mocap {self._probe.mocap_id}", self._probe.mocap_id, self._probe, ctx.now())

    def _add_sample(self, ctx: PluginContext, sample: Sample, selected: bool) -> None:
        """Give `sample` the next id and add it to the samples, the list and the 3D view."""
        sample.id = self._next_id
        self._next_id += 1
        self.samples.append(sample)
        self._add_marker(ctx, sample)
        self._list.add(sample.id, sample.name, hint=f"{numbers(sample.position, 3, 8, 4)} m", selected=selected)

    def _add_marker(self, ctx: PluginContext, sample: Sample) -> None:
        """Show a recorded sample in the 3D view: a sphere and its name."""
        path = f"{ctx.view.scene_root}/samples/{sample.id}"
        sphere = ctx.view.scene.add_icosphere(f"{path}/point", radius=0.008, color=SAMPLE_COLOR,
                                              position=tuple(sample.position))
        label = ctx.view.scene.add_label(f"{path}/label", sample.name, position=tuple(sample.position))
        self._sample_handles[sample.id] = (sphere, label)

    def _draw_coverage(self, ctx: PluginContext) -> None:
        """Show the smoothed map on the floor, drawn again at most every MAP_PERIOD while samples come in."""
        if self._coverage_drawn != self.coverage.count and ctx.now() - self._coverage_time >= MAP_PERIOD:
            smoothed = self.coverage.smoothed()
            alpha = np.where(smoothed.seen, round(COVERAGE_OPACITY * 255), 0).astype(np.uint8)[..., None]
            # * viser draws the image's first row at the low y edge: rows are y bins, columns x bins.
            image = np.concatenate([map_colors(smoothed), alpha], axis=-1).transpose(1, 0, 2)
            if self._coverage_image is None:
                self._coverage_image = ctx.view.scene.add_image(
                    f"{ctx.view.scene_root}/coverage", image, *COVERAGE_SIZE, format="png",
                    position=(COVERAGE_ORIGIN[0] + COVERAGE_SIZE[0] / 2, COVERAGE_ORIGIN[1] + COVERAGE_SIZE[1] / 2,
                              COVERAGE_Z), cast_shadow=False, receive_shadow=False)
            else:
                self._coverage_image.image = image
            self._coverage_drawn, self._coverage_time = self.coverage.count, ctx.now()
        if self._coverage_image is not None:
            set_changed(self._coverage_image, visible=self._show_map.value)

    def _coverage_html(self) -> str:
        """The coverage section's summary: samples, area mapped, overall rate and error, and the map under the probe."""
        coverage, probe = self.coverage, self._probe
        rate = coverage.tracked_rate()
        smoothed = coverage.smoothed()
        here = smoothed.at(*probe.position[:2]) if probe is not None and probe.position is not None else None
        state = chip("mapping", INFO) if self._mapping.value else chip("off", NONE)
        return block(state + values(
            f"{coverage.count} samples, {smoothed.seen.sum() * COVERAGE_BIN ** 2:.0f} m², "
            f"tracked {'–' if rate is None else f'{rate:.0%}'}", f"err {_error_mm(coverage.total_error())}",
            "" if here is None else f"here tracked {here[0]:.0%}, err "
                                    f"{'–' if np.isnan(here[1]) else f'{here[1] * 1e3:.2f} mm'}"))

    def _map_inputs(self, ctx: PluginContext | None) -> tuple[FloorGrid, np.ndarray, Iterable, list[Dot]]:
        """What the phone's map shows: coverage cells, every footprint in the scene, the recorded points and the probe.

        Args:
            ctx: This plugin's context; None leaves out the scene and the probe (before the first tick).

        Returns:
            The arguments of `render_top_down`; the footprints are computed lazily, so on its thread.
        """
        colors = map_colors(self.coverage.smoothed())
        dots = [Dot(*sample.position[:2], SAMPLE_COLOR) for sample in self.samples]
        probe = self._probe
        if ctx is not None and probe is not None and probe.position is not None:
            check = self._check(ctx)
            lost = check is None or check.level == BAD
            dots.append(Dot(*probe.position[:2], PROBE_LOST_COLOR if lost else PROBE_DOT_COLOR, radius=7,
                            outline=(0, 0, 0)))
        footprints = () if ctx is None else scene_footprints(ctx.scene.snapshot)
        lines = tuple(round(size / MAP_LINES) for size in COVERAGE_SIZE)
        return FloorGrid(COVERAGE_ORIGIN, MAP_LINES, lines), colors, footprints, dots

    async def _draw_map(self, ctx: PluginContext) -> None:
        """Draw the phone's map on a worker thread, then show it; a robot's first footprints take about a second."""
        try:
            picture = await ctx.run_in_thread(render_top_down, *self._map_inputs(ctx))
            # * Assigning sends the PNG; skip it when nothing moved.
            if not np.array_equal(picture, self._phone_map.image):
                self._phone_map.image = picture
        finally:
            self._map_task = None

    def _big_status(self, ctx: PluginContext, check: Check | None, min_height: str = "40vh") -> str:
        """The big status panel's HTML: one colour and one word readable at a distance, x y z and the marker error.

        ! Any text change re-renders it: the only live numbers are x y z, rounded to the millimetre, and the error.

        Args:
            ctx: This plugin's context.
            check: The probe's mocap check, or None when nothing is tracked.
            min_height: The block's least height; lower on the phone, so Record point stays on screen.
        """
        # * No check detail here (the side panel's chip has it): only the recorded point's name.
        # ! The name line and the x y z and error rows are always there, so the buttons below never move.
        detail = ""
        if ctx.now() < self._flash_until:
            color, word, detail = FLASH_COLOR, "RECORDED", self.samples[-1].name if self.samples else ""
        elif check is None:
            color, word = NONE, "NOT TRACKING"
        else:
            color, word = LEVEL_COLORS[check.level], BIG_WORDS[check.level]
        detail = (f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">'
                  f'{escape(detail) if detail else "&nbsp;"}</div>')
        probe = self._probe
        # * Fixed width and sign, so the digits stand still while the probe moves; dim when the fix is stale or absent.
        xyz = None if probe is None or probe.position is None else probe.position
        dim = "opacity:.5;" if xyz is None or check is None or check.level == BAD else ""
        lines = [f"{axis:<3} {numbers(None if xyz is None else (xyz[i],), 1, 7, 3)} m" for i, axis in enumerate("xyz")]
        error = "—".rjust(7) if xyz is None or probe.marker_error is None else f"{probe.marker_error * 1e3:7.2f}"
        rows = "".join(f"<div>{line}</div>" for line in [*lines, f"err {error} mm"])
        # * A left-aligned block, centred: the rows line up although the units differ in length.
        position = (f'<div style="font:700 clamp(20px,5vw,64px)/1.2 monospace;white-space:pre;text-align:left;'
                    f'display:inline-block;margin-top:8px;{dim}">{rows}</div>')
        # ? vh units, so the box grows with a floated, enlarged panel.
        return (f'<div style="background:{color};color:#fff;border-radius:8px;min-height:{min_height};'
                f'display:flex;flex-direction:column;align-items:center;justify-content:center;'
                f'text-align:center;padding:12px;margin:0 8px 8px">'
                f'<div style="font-size:clamp(32px,9vw,120px);font-weight:800;line-height:1.1">{word}</div>'
                f'{detail}'
                f'{position}'
                f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">{len(self.samples)} points'
                f'{" · mapping coverage" if self._mapping.value else ""}</div>'
                f'</div>')

    def _draw_floor(self, ctx: PluginContext, check: Check | None) -> None:
        """Refit the floor to the current samples, show its quality and move the lab frame's axes and stored points."""
        solution = self._solve(ctx)
        plane, probe = solution.plane, self._probe
        height = None
        if plane is not None and probe is not None and probe.position is not None and check and check.level != BAD:
            height = plane.height(to_raw(lab_from_raw(ctx), probe.position))
        not_found = self._not_found(ctx)
        html = self._floor_html(solution, height, self._anchor_entries(ctx, solution),
                                self._check_lines(ctx, solution), not_found)
        self._floor_status.content = html
        self._phone_floor_status.content = phone_text(html)
        # * The axes show where the export puts the lab frame; viser sends only the values that changed.
        transform = solution.transform
        self._floor_axes.visible = transform is not None
        applied = lab_from_raw(ctx)
        # * The scene is in the lab frame the relay applies now: its lab_from_raw, then the new raw_from_floor.
        if transform is not None:
            shown = (Pose(*applied).matrix() if applied else np.eye(4)) @ np.linalg.inv(transform)
            self._floor_axes.position = shown[:3, 3]
            self._floor_axes.wxyz = np.roll(Rotation.from_matrix(shown[:3, :3]).as_quat(), 1)
        else:
            shown = np.eye(4) if applied else None
        # * The stored marks and anchors where the frame being fitted puts them, else where the relay's frame does;
        #   never without either frame.
        self._stored.visible = shown is not None and self._show_stored.value
        if self._stored.visible:
            self._stored.position = shown[:3, 3]
            self._stored.wxyz = np.roll(Rotation.from_matrix(shown[:3, :3]).as_quat(), 1)
        self._draw_read_anchors(ctx, solution, shown if self._stored.visible else None, not_found)

    def _not_found(self, ctx: PluginContext) -> list[str]:
        """The stored anchors not found near their stored place, not pointed at: the probe has to show where."""
        return [name for name, why in self._anchors_now(ctx)[1].items() if why.endswith(POINT_AT_IT)]

    def _draw_read_anchors(self, ctx: PluginContext, solution: FloorSolution, shown: np.ndarray | None,
                           not_found: list[str]) -> None:
        """Each anchor read, while the stored ones show (`shown`, lab to the scene): Motive's marker, labelled.

        The label: how far the marker is from its stored place as drawn, and the relay's noise of it. The stored
        anchor's spheres turn green with the marker within MARK_WARN of them, red beyond, orange while `not_found`,
        its label then saying so.
        """
        applied, wanted, tints = lab_from_raw(ctx), {}, {name: NOT_FOUND_COLOR for name in not_found}
        for name, raw in solution.anchors.items() if shown is not None else ():
            position, text = from_raw(applied, raw), name
            if name in self._ideal_anchors:
                stored = (shown @ np.r_[self._ideal_anchors[name], 1.0])[:3]
                off = np.linalg.norm(position - stored)
                text += f" {off * 1e3:.1f} mm"
                tints[name] = WITHIN_COLOR if off <= MARK_WARN else BEYOND_COLOR
            noise = self._marker_noise(ctx, raw)
            wanted[name] = (position, text + ("" if noise is None else f" · noise {noise:.2f} mm"))
        for name in set(self._read) - set(wanted):
            for handle in self._read.pop(name)[:2]:
                handle.remove()
        root = f"{ctx.view.scene_root}/read_anchors/{{}}"
        for name, (position, text) in wanted.items():
            sphere, label, shown_text = self._read.get(name) or (ctx.view.scene.add_icosphere(
                f"{root.format(name)}/sphere", radius=READ_RADIUS, color=READ_COLOR), None, None)
            sphere.position = position
            # * A label's text cannot change: a new one when it does.
            if text != shown_text:
                if label is not None:
                    label.remove()
                label = ctx.view.scene.add_label(f"{root.format(name)}/label", text, position=tuple(position))
            label.position = tuple(position)
            self._read[name] = (sphere, label, text)
        for name, spheres in self._anchor_spheres.items():
            tint = tints.get(name, STORED_ANCHOR_COLOR)
            if self._anchor_tints.get(name) != tint:
                for sphere in spheres:
                    sphere.color = tint
                self._anchor_tints[name] = tint
        for name, (label, shown_text) in self._anchor_labels.items():
            text = f"{name} not found: point at it" if name in not_found else name
            if text != shown_text:
                position = label.position
                label.remove()
                label = ctx.view.scene.add_label(f"{ctx.view.scene_root}/stored/{name}/label", text, position=position)
                self._anchor_labels[name] = (label, text)

    @staticmethod
    def _marker_noise(ctx: PluginContext, raw: np.ndarray) -> float | None:
        """The relay's noise of the marker at `raw` in its last look, mm; None without one within NOISE_MATCH."""
        relay = ctx.world.mocap_relay
        looked = relay.state.marker_noise if relay is not None else ()
        if not looked:
            return None
        distance, noise = min(((np.linalg.norm(np.subtract(position, raw)), noise) for position, noise in looked),
                              key=lambda item: item[0])
        return noise if distance <= NOISE_MATCH else None

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def _draw_frame(self, ctx: PluginContext) -> None:
        """The frame the relay holds as chips (`frame_chips`), Use refit enabled while a refit is pending."""
        frame, relay = relay_frame(ctx), ctx.world.mocap_relay
        html = frame_chips(frame, relay.state.anchors_missing if relay is not None else ())
        set_changed(self._frame_chips, content=chip_line(html))
        set_changed(self._use_refit, disabled=frame is None or frame.status != FRAME_REFIT_PENDING)
        set_changed(self._apply_floor, disabled=frame is None or not frame.newer)

    def _draw_cameras(self, ctx: PluginContext) -> None:
        """Draw the relay's cameras while "Show cameras" is ticked, anew when it announces others; one status line."""
        relay = ctx.world.mocap_relay
        cameras = relay.state.cameras if relay is not None else {}
        show, views, height = self._show_cameras.value, self._show_views.value, float(self._views_height.value)
        if show and redraw(self._drawn_cameras, cameras, height):
            self._draw_camera_views(ctx, cameras, height)
        for handle in self._camera_handles:
            set_changed(handle, visible=show)
        if views and redraw(self._views_for, cameras, height):
            self._draw_views(ctx, cameras, height)
        for handle in self._views_handles:
            set_changed(handle, visible=views)
        seen = (f"seen by 2+: {sum(a for n, a in self._views_areas.items() if n >= 2):.0f} m², 3+: "
                f"{sum(a for n, a in self._views_areas.items() if n >= 3):.0f} m² at {height:g} m")
        set_changed(self._cameras_status, content=values(
            f"{len(cameras)} cameras from the relay" if cameras else "no cameras from the relay (not running, or no "
                                                                     "Motive)",
            seen if views and cameras else "", dim=not (show or views)))

    def _draw_camera_views(self, ctx: PluginContext, cameras: dict, height: float) -> None:
        """Each camera's frustum and name, its view's edges as rays down to the plane at `height`, and its aim cross."""
        for handle in self._camera_handles:
            handle.remove()
        root, scene = f"{ctx.view.scene_root}/cameras", ctx.view.scene
        self._camera_handles, rays, crosses = [], [], []
        for index, camera in enumerate(cameras.values()):
            self._camera_handles += camera_frustum(scene, f"{root}/{index}", camera)
            for edge in view_edges(camera):
                end = plane_point(camera.position, edge, height)
                rays.append((camera.position, end if end is not None else np.add(camera.position, RAY_LENGTH * edge)))
            aim = aim_point(camera, height)
            if aim is not None:
                arm = AIM_SIZE / 2 / np.sqrt(2)
                crosses += [(aim + (-arm, -arm, VIEWS_LIFT), aim + (arm, arm, VIEWS_LIFT)),
                            (aim + (-arm, arm, VIEWS_LIFT), aim + (arm, -arm, VIEWS_LIFT))]
        drawn = (("rays", rays, RAY_COLOR, RAY_WIDTH), ("aims", crosses, FRUSTUM_COLOR, AIM_WIDTH))
        for name, lines, color, width in drawn:
            if lines:
                self._camera_handles.append(scene.add_line_segments(f"{root}/{name}", np.array(lines, dtype=float),
                                                                    color, thickness=width))
        self._drawn_cameras = (cameras, height)

    def _draw_views(self, ctx: PluginContext, cameras: dict, height: float) -> None:
        """Cut every camera's view with the plane at `height` and draw the regions by how many cameras see them."""
        for handle in self._views_handles:
            handle.remove()
        bounds = rectangle(COVERAGE_ORIGIN, COVERAGE_SIZE)
        shapes = [footprint(camera, height, bounds) for camera in cameras.values()]
        faces = coverage_faces(shapes, bounds) if cameras else []
        root, scene, z = f"{ctx.view.scene_root}/camera_views", ctx.view.scene, height + VIEWS_LIFT
        self._views_handles, self._views_areas = [], {}
        for count in sorted({count for count, _ in faces}):
            region = shapely.union_all([face for n, face in faces if n == count])
            self._views_areas[count] = region.area
            points, triangle_indices = triangles(region)
            if len(triangle_indices):
                color = coverage_color(min(count, CAMERAS_FULL) / CAMERAS_FULL if count else None)
                self._views_handles.append(scene.add_mesh_simple(
                    f"{root}/{count}", np.column_stack([points, np.full(len(points), z)]), triangle_indices,
                    color=color, opacity=VIEWS_OPACITY, side="double", cast_shadow=False, receive_shadow=False))
        if cameras:
            lines = segments(outlines(shapes, bounds))
            self._views_handles.append(scene.add_line_segments(
                f"{root}/outlines", np.concatenate([lines, np.full((*lines.shape[:2], 1), z + 0.001)], axis=2),
                OUTLINE_COLOR, thickness=OUTLINE_WIDTH))
        for index, (count, face) in enumerate(faces):
            if face.area >= COUNT_MIN_AREA:
                inside = face.representative_point()
                self._views_handles.append(scene.add_label(
                    f"{root}/counts/{index}", str(count), position=(inside.x, inside.y, z + 0.002),
                    font_size_mode="scene", font_scene_height=COUNT_HEIGHT, anchor="center-center"))
        self._views_for = (cameras, height)

    def draw(self, ctx: PluginContext) -> None:
        """Show the live probe pose values, the sample list and the last action's result.

        Args:
            ctx: This plugin's context.
        """
        probe = self._probe
        check = self._check(ctx)
        state = check_chip(check) if check else chip("not tracking", NONE, "enter a mocap id and press Track")

        has_pose = probe is not None and probe.position is not None
        rpy = error_mm = None
        if has_pose:
            rpy = Rotation.from_quat(probe.orientation).as_euler("xyz", degrees=True)
            error_mm = (probe.marker_error * 1e3,)
        self._big.content = self._big_status(ctx, check)
        self._phone_big.content = self._big_status(ctx, check, min_height="24vh")
        self._status.content = block(state + values(f"xyz {numbers(probe.position if has_pose else None, 3, 8, 4)} m",
                                                    f"rpy {numbers(rpy, 3, 8, 1)} °",
                                                    f"err {numbers(error_mm, 1, 8, 2)} mm",
                                                    dim=check is None or check.level == BAD))

        self._list.sync()
        # * Recolour only the spheres whose selection changed.
        selected = {sample.id for sample in self._selected_samples()}
        for sample_id in selected ^ self._highlighted:
            self._sample_handles[sample_id][0].color = SELECTED_COLOR if sample_id in selected else SAMPLE_COLOR
        self._highlighted = selected

        self._draw_coverage(ctx)
        self._coverage_status.content = self._coverage_html()
        self._draw_floor(ctx, check)
        self._draw_cameras(ctx)
        self._draw_frame(ctx)
        for which, buttons in self._confirms.items():
            due, anyway = self._confirm.get(which, (-np.inf, False))
            again = "Tap again: export" if which == "export" else f"Tap again: delete {len(self._deletable(which))}"
            for button, label, color in buttons:
                if ctx.now() > due:
                    button.label, button.color = label, color
                else:
                    button.label, button.color = "Press again: export anyway" if anyway else again, "orange"
        self._result.content = note_line(self._message, self._message_color)
        self._phone_result.content = phone_text(note_line(self._message, self._message_color))
        floor_result = note(escape(self._floor_message) or "—", self._floor_message_color)
        self._floor_result.content, self._phone_floor_result.content = floor_result, phone_text(floor_result)
        if self._mapping.value and self._map_task is None and ctx.now() - self._map_time >= MAP_PERIOD:
            self._map_time = ctx.now()
            self._map_task = ctx.spawn("phone map", self._draw_map(ctx), survives_soft_stop=True)
        recent = self.samples[-PHONE_POINTS:]
        self._phone_points.content = (section("points", SECTION_CTRL, f"{len(self.samples)} recorded")
                                      + block(values(*(f"{sample.name:<10} {numbers(sample.position, 3, 7, 3)}"
                                                       for sample in recent))) if recent else "")
