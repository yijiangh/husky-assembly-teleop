"""
A mocap probe: track one rigid body, show its live pose, and record points with it, e.g. obstacle corners.

Points are numbered per label ("table_1", ...) and shown in the 3D view. Delete and
Export act on the selected points, so one session can give one file per obstacle.
"Load points" adds the points of an exported file, unselected; a name already taken is renumbered.
A big "Probe" status panel (minimized on the right) is readable from across the room.

Coverage map: with "Map coverage" ticked, walk the probe around; each mocap message counts in the floor cell
under the probe, as tracked or not. Cells are drawn on the floor, grey until visited, then red (never tracked)
to green (always tracked). Export and Load keep a map across runs; loading then mapping adds to it.

! The recorded point is the rigid body's origin: set the pivot in Motive to the probe tip.
! Recording is refused while the mocap chip is red; amber records, with its marker error.
! A lost probe has no position: its misses count in the cell of its last fix, so walk slowly through gaps.

Run with:  -p plugins:="['mocap_probe']"
"""

from __future__ import annotations

import colorsys
import json
import time
from dataclasses import asdict, dataclass, field
from html import escape
from pathlib import Path

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ..ui.checklist import CheckList
from ..plugin_api.context import PluginContext
from ..world.mocap import MARKER_ERROR_WARN, mocap_check
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.style import (LEVEL_COLORS, NONE, OK, SECTION_CTRL, SECTION_SENSOR, block, check_chip, chip, note, numbers,
                        section, values)
from ..world.checks import BAD, GOOD, WARN, Check
from ..world.measured import TrackedObject

#: Name of the probe in `world.tracked_objects` and on the health panel.
PROBE_NAME = "probe"
#: Rigid-body id of the lab's probe, tracked from startup.
DEFAULT_MOCAP_ID = 1863
#: Label used when the Label field is empty.
DEFAULT_LABEL = "p"
#: Where "Export" writes its files.
EXPORT_FOLDER = Path.home() / "husky_probe"
#: Colour of unselected recorded points in the 3D view.
SAMPLE_COLOR = (230, 60, 150)
#: Colour of the selected recorded points.
SELECTED_COLOR = (250, 200, 30)
#: How long the big status panel flashes blue after a point is recorded, seconds.
FLASH_SECONDS = 0.8
#: Blue of that flash.
FLASH_COLOR = "#1c7ed6"
#: The big status panel's word for each mocap check level.
BIG_WORDS = {GOOD: "TRACKED", WARN: "NOISY", BAD: "NO FIX"}
#: Coverage grid: size (x, y) and cell side, metres; centred on the mocap origin.
COVERAGE_SIZE = (15.0, 15.0)
COVERAGE_CELL = 1.0
#: Lower corner (x, y) of the coverage grid in the world frame, metres.
COVERAGE_ORIGIN = (-COVERAGE_SIZE[0] / 2, -COVERAGE_SIZE[1] / 2)
#: Colour of coverage cells without samples.
UNCOVERED_COLOR = (140, 140, 140)
#: Height of the coverage tiles above the floor, metres; just above it so they do not flicker with the grid.
COVERAGE_Z = 0.004
#: File name prefix of exported coverage maps.
COVERAGE_PREFIX = "mocap_coverage"


@dataclass
class Sample:
    """One recorded probe point, in the world frame (mocap, Z-up), metres.

    Attributes:
        id: Unique within this run, in recording order.
        name: `label` plus its running number, e.g. "table_3"; unique within this run.
        label: What the operator typed, e.g. "table".
        position: Rigid-body origin, (x, y, z).
        orientation: Rigid-body orientation, quaternion (x, y, z, w).
        mocap_id: Rigid-body id the probe was tracked by.
        marker_error: Mean marker error of the pose, metres.
        stamp: ROS time of the recording, seconds.
    """

    id: int
    name: str
    label: str
    position: list[float]
    orientation: list[float]
    mocap_id: int
    marker_error: float
    stamp: float


def samples_from_json(data: dict) -> list[Sample]:
    """The samples of a file written by Export, with id 0; the caller numbers them.

    Raises:
        ValueError: If `data` is not a probe points file.
    """
    if not isinstance(data, dict) or not isinstance(data.get("samples"), list):
        raise ValueError("not a probe points file (no samples list)")
    try:
        return [Sample(id=0, name=str(item["name"]), label=str(item["label"]),
                       position=[float(v) for v in item["position"]],
                       orientation=[float(v) for v in item["orientation"]], mocap_id=int(item["mocap_id"]),
                       marker_error=float(item["marker_error"]), stamp=float(item["stamp"]))
                for item in data["samples"]]
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"bad sample: {error}") from error


@dataclass
class CoverageGrid:
    """Per-cell tracking counts on a floor grid, world frame (x, y), metres.

    Attributes:
        origin: (x, y) of the grid's lower corner.
        cell_size: Side of one square cell.
        shape: Number of cells along (x, y).
        samples: Mocap messages counted per cell, shape `shape`.
        tracked: Of those, how many had a valid pose.
        good: Of those, how many also had a marker error under MARKER_ERROR_WARN.
        error_sum: Sum of the tracked messages' mean marker errors, metres.
        error_sq_sum: Sum of their squares, for the standard deviation.
    """

    origin: tuple[float, float]
    cell_size: float
    shape: tuple[int, int]
    samples: np.ndarray = field(default=None)
    tracked: np.ndarray = field(default=None)
    good: np.ndarray = field(default=None)
    error_sum: np.ndarray = field(default=None)
    error_sq_sum: np.ndarray = field(default=None)

    def __post_init__(self):
        """Start empty unless counts were given."""
        for name in ("samples", "tracked", "good"):
            if getattr(self, name) is None:
                setattr(self, name, np.zeros(self.shape, dtype=int))
        for name in ("error_sum", "error_sq_sum"):
            if getattr(self, name) is None:
                setattr(self, name, np.zeros(self.shape))

    @classmethod
    def default(cls) -> CoverageGrid:
        """CoverageGrid: An empty grid over COVERAGE_ORIGIN, COVERAGE_SIZE and COVERAGE_CELL."""
        shape = tuple(round(size / COVERAGE_CELL) for size in COVERAGE_SIZE)
        return cls(COVERAGE_ORIGIN, COVERAGE_CELL, shape)

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        """The (i, j) index of the cell containing (x, y), or None outside the grid."""
        i = int(np.floor((x - self.origin[0]) / self.cell_size))
        j = int(np.floor((y - self.origin[1]) / self.cell_size))
        return (i, j) if 0 <= i < self.shape[0] and 0 <= j < self.shape[1] else None

    def center(self, i: int, j: int) -> tuple[float, float]:
        """The (x, y) centre of cell (i, j)."""
        return (self.origin[0] + (i + 0.5) * self.cell_size, self.origin[1] + (j + 0.5) * self.cell_size)

    def add(self, x: float, y: float, error: float | None) -> tuple[int, int] | None:
        """Count one mocap message at (x, y); returns its cell, or None (not counted) outside the grid.

        Args:
            x: Probe position, metres.
            y: Probe position, metres.
            error: Mean marker error of a tracked message, metres, or None if it was not tracked.
        """
        cell = self.cell(x, y)
        if cell is not None:
            self.samples[cell] += 1
            if error is not None:
                self.tracked[cell] += 1
                self.good[cell] += error <= MARKER_ERROR_WARN
                self.error_sum[cell] += error
                self.error_sq_sum[cell] += error * error
        return cell

    def rate(self, i: int, j: int) -> float | None:
        """The tracked fraction of cell (i, j), or None without samples."""
        samples = self.samples[i, j]
        return float(self.tracked[i, j] / samples) if samples else None

    def error(self, i: int, j: int) -> tuple[float, float] | None:
        """The (mean, standard deviation) marker error of cell (i, j)'s tracked messages, metres, or None."""
        tracked = self.tracked[i, j]
        if not tracked:
            return None
        mean = self.error_sum[i, j] / tracked
        # ? max: rounding can make the variance slightly negative.
        return float(mean), float(np.sqrt(max(self.error_sq_sum[i, j] / tracked - mean * mean, 0.0)))

    def total_error(self) -> tuple[float, float] | None:
        """The (mean, standard deviation) marker error over all tracked messages, metres, or None."""
        tracked = self.tracked.sum()
        if not tracked:
            return None
        mean = self.error_sum.sum() / tracked
        return float(mean), float(np.sqrt(max(self.error_sq_sum.sum() / tracked - mean * mean, 0.0)))

    def to_json(self) -> dict:
        """dict: The grid and its visited cells, as written by Export."""
        cells = [{"index": [int(i), int(j)], "center": list(self.center(i, j)),
                  "samples": int(self.samples[i, j]), "tracked": int(self.tracked[i, j]),
                  "good": int(self.good[i, j]), "rate": self.rate(i, j),
                  "error_mean": (self.error(i, j) or (None, None))[0],
                  "error_std": (self.error(i, j) or (None, None))[1]}
                 for i, j in zip(*np.nonzero(self.samples))]
        return {"kind": "mocap_coverage", "origin": list(self.origin), "cell_size": self.cell_size,
                "shape": list(self.shape), "cells": cells}

    @classmethod
    def from_json(cls, data: dict) -> CoverageGrid:
        """The grid written by `to_json`.

        Raises:
            ValueError: If `data` is not a coverage map.
        """
        if data.get("kind") != "mocap_coverage":
            raise ValueError("not a mocap coverage file (no kind: mocap_coverage)")
        grid = cls(tuple(data["origin"]), float(data["cell_size"]), tuple(data["shape"]))
        for cell in data["cells"]:
            index = tuple(cell["index"])
            grid.samples[index], grid.tracked[index], grid.good[index] = cell["samples"], cell["tracked"], cell["good"]
            # * Sums back from mean and std, so mapping after a load keeps adding to them.
            if cell["tracked"] and cell.get("error_mean") is not None:
                mean, std = cell["error_mean"], cell["error_std"]
                grid.error_sum[index] = mean * cell["tracked"]
                grid.error_sq_sum[index] = (std * std + mean * mean) * cell["tracked"]
        return grid


def coverage_color(rate: float | None) -> tuple[int, int, int]:
    """Red (0) through yellow to green (1) for a tracked fraction; UNCOVERED_COLOR for None."""
    if rate is None:
        return UNCOVERED_COLOR
    return tuple(round(c * 255) for c in colorsys.hsv_to_rgb(rate / 3, 0.8, 0.85))


def _error_mm(error: tuple[float, float] | None) -> str:
    """A (mean, std) marker error in metres as "0.82 ± 0.10 mm", or "–"."""
    return f"{error[0] * 1e3:.2f} ± {error[1] * 1e3:.2f} mm" if error else "–"


@register
class MocapProbePlugin(HuskyPlugin):
    """Tracks a mocap probe, shows its pose, and records, lists and exports points."""

    name = "mocap_probe"

    def __init__(self):
        """Start with no probe tracked and no samples."""
        self._probe: TrackedObject | None = None
        self.samples: list[Sample] = []
        self._next_id = 1
        # Last number given per label; never lowered, so names stay unique after a delete.
        self._label_counts: dict[str, int] = {}
        # Scene markers per sample id: the sphere and the label.
        self._markers: dict[int, tuple[viser.IcosphereHandle, viser.LabelHandle]] = {}
        # Sample ids whose sphere is drawn in SELECTED_COLOR.
        self._highlighted: set[int] = set()
        # Result of the last action (track, record, delete, export).
        self._message = ""
        # ROS time until which the big status panel shows the blue flash.
        self._flash_until = 0.0
        # Coverage map, its floor tiles per cell, and cells counted since the last draw.
        self.coverage = CoverageGrid.default()
        self._coverage_root: viser.FrameHandle | None = None
        self._tiles: dict[tuple[int, int], viser.BoxHandle] = {}
        self._dirty: set[tuple[int, int]] = set()
        # `last_update_time` of the last probe message counted, so each message counts once.
        self._counted_update: float | None = None
        # Cell under the probe at its last counted message, or None outside the grid.
        self._probe_cell: tuple[int, int] | None = None

    def setup(self, ctx: PluginContext) -> None:
        """Build the widgets and start tracking DEFAULT_MOCAP_ID.

        Args:
            ctx: This plugin's context.
        """
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
                                           hint=f"Saved as {EXPORT_FOLDER}/<name>_<date>_<time>.json")
            actions = gui.add_button_group("Selected", ["Delete", "Export"])
            load_points = gui.add_upload_button("Load points", mime_type="application/json", icon=viser.Icon.UPLOAD,
                                                hint="Add the points of an exported file, unselected.")
            self._result: viser.GuiHtmlHandle = gui.add_html("")
            self._list = CheckList(ctx, gui, "Samples", empty_text="no samples yet")

            gui.add_html(section("coverage map", SECTION_SENSOR))
            self._mapping = gui.add_checkbox("Map coverage", initial_value=False,
                                             hint=f"Count every probe message in its {COVERAGE_CELL:g} m floor cell, "
                                                  f"as tracked or not. Walk the probe around the room.")
            self._show_map = gui.add_checkbox("Show map", initial_value=True)
            coverage_actions = gui.add_button_group("Coverage", ["Export", "Clear"])
            load = gui.add_upload_button("Load coverage", mime_type="application/json", icon=viser.Icon.UPLOAD,
                                         hint="Replace the map with an exported one; mapping then adds to it.")
            self._coverage_status: viser.GuiHtmlHandle = gui.add_html("")

        # * Big status in its own panel, so it can be floated and enlarged. Raw gui api: ui() would use our folder.
        panel = ctx.view.panel()
        with panel.add_tab("Probe", icon=viser.Icon.CROSSHAIR):
            self._big: viser.GuiHtmlHandle = ctx.view.gui.add_html("")
        panel.dock_right()
        panel.minimize()

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        track.on_click(ctx.defer("track probe", lambda: self._track(ctx)))
        record.on_click(ctx.defer("record point", lambda: self._record(ctx)))
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
        self._message = f"tracking mocap id {mocap_id}"
        ctx.log_info(self._message)

    def _record(self, ctx: PluginContext) -> None:
        """Record the probe position as a new, selected sample, unless the mocap check is red.

        Args:
            ctx: This plugin's context.
        """
        check = self._check(ctx)
        if check is None or check.level == BAD:
            # ! Never record a stale or invalid pose: it would silently give a wrong corner.
            self._message = f"not recorded: {check.detail if check else 'probe not tracked'}"
            ctx.log_warn(self._message)
            return
        label = self._label.value.strip() or DEFAULT_LABEL
        self._label_counts[label] = self._label_counts.get(label, 0) + 1
        sample = Sample(id=0, name=f"{label}_{self._label_counts[label]}", label=label,
                        position=self._probe.position.tolist(), orientation=self._probe.orientation.tolist(),
                        mocap_id=self._probe.mocap_id, marker_error=self._probe.marker_error, stamp=ctx.now())
        self._add_sample(ctx, sample, selected=True)
        self._message = f"recorded {sample.name} at {numbers(sample.position, 3, 8, 4)} m"
        self._flash_until = ctx.now() + FLASH_SECONDS
        ctx.log_info(self._message)

    def _load_points(self, ctx: PluginContext, file: viser.UploadedFile) -> None:
        """Add the samples of an exported file, unselected; names already taken get the label's next number.

        Args:
            ctx: This plugin's context.
            file: The uploaded JSON file.
        """
        try:
            loaded = samples_from_json(json.loads(file.content))
        except ValueError as error:  # * json.JSONDecodeError is a ValueError
            self._message = f"not loaded {file.name}: {error}"
            ctx.log_warn(self._message)
            return
        taken = {sample.name for sample in self.samples}
        renamed = 0
        for sample in loaded:
            # * Keep "table_3" numbering going: the next recorded table becomes table_4.
            prefix, _, number = sample.name.rpartition("_")
            if prefix == sample.label and number.isdigit():
                self._label_counts[sample.label] = max(self._label_counts.get(sample.label, 0), int(number))
            if sample.name in taken:
                self._label_counts[sample.label] = self._label_counts.get(sample.label, 0) + 1
                sample.name = f"{sample.label}_{self._label_counts[sample.label]}"
                renamed += 1
            taken.add(sample.name)
            self._add_sample(ctx, sample, selected=False)
        self._message = f"loaded {len(loaded)} points from {file.name}" + (f", {renamed} renamed" if renamed else "")
        ctx.log_info(self._message)

    def _delete(self, ctx: PluginContext) -> None:
        """Delete the selected samples.

        Args:
            ctx: This plugin's context.
        """
        doomed = self._selected_samples()
        if not doomed:
            self._message = "nothing selected to delete"
            return
        for sample in doomed:
            self.samples.remove(sample)
            self._list.remove(sample.id)
            self._highlighted.discard(sample.id)
            for handle in self._markers.pop(sample.id):
                handle.remove()
        self._message = f"deleted {', '.join(sample.name for sample in doomed)}"
        ctx.log_info(self._message)

    def _export(self, ctx: PluginContext) -> None:
        """Write the selected samples to a timestamped JSON file in EXPORT_FOLDER.

        Args:
            ctx: This plugin's context.
        """
        chosen = self._selected_samples()
        if not chosen:
            self._message = "nothing selected to export"
            return
        name = self._file_name.value.strip() or "probe_points"
        EXPORT_FOLDER.mkdir(parents=True, exist_ok=True)
        path = EXPORT_FOLDER / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        data = {
            "frame": "world (mocap, Z-up), metres; quaternions are (x, y, z, w)",
            "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "samples": [asdict(sample) for sample in chosen],
        }
        path.write_text(json.dumps(data, indent=2))
        self._message = f"exported {len(chosen)} samples to {path}"
        ctx.log_info(self._message)

    def _set_mapping(self, ctx: PluginContext, on: bool) -> None:
        """Start or stop counting probe messages into the coverage map; shows the map on start."""
        if on:
            self._build_tiles(ctx)
            self._show_map.value = True
        self._counted_update = None
        self._message = "coverage mapping " + ("started" if on else "stopped")
        ctx.log_info(self._message)

    def _export_coverage(self, ctx: PluginContext) -> None:
        """Write the coverage map to a timestamped JSON file in EXPORT_FOLDER."""
        if not self.coverage.samples.any():
            self._message = "coverage map is empty; tick Map coverage and walk the probe around"
            return
        EXPORT_FOLDER.mkdir(parents=True, exist_ok=True)
        path = EXPORT_FOLDER / f"{COVERAGE_PREFIX}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        data = {"frame": "world (mocap, Z-up), metres; cells cover [origin, origin + shape * cell_size)",
                "exported_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "mocap_id": self._probe.mocap_id if self._probe else None,
                "rate": "tracked / samples; good: tracked with marker error under "
                        f"{MARKER_ERROR_WARN * 1e3:g} mm",
                "error": "error_mean, error_std: mean marker error of the tracked messages, metres",
                **self.coverage.to_json()}
        path.write_text(json.dumps(data, indent=2))
        self._message = f"exported coverage of {len(data['cells'])} cells to {path}"
        ctx.log_info(self._message)

    def _clear_coverage(self, ctx: PluginContext) -> None:
        """Empty the coverage map, keeping its grid."""
        self._dirty.update(zip(*np.nonzero(self.coverage.samples)))
        self.coverage = CoverageGrid(self.coverage.origin, self.coverage.cell_size, self.coverage.shape)
        self._message = "coverage map cleared"
        ctx.log_info(self._message)

    def _load_coverage(self, ctx: PluginContext, file: viser.UploadedFile) -> None:
        """Replace the coverage map with an exported one, its grid included."""
        try:
            grid = CoverageGrid.from_json(json.loads(file.content))
        except (ValueError, KeyError, TypeError, IndexError) as error:
            self._message = f"not loaded {file.name}: {error}"
            ctx.log_warn(self._message)
            return
        self._remove_tiles()
        self.coverage = grid
        self._build_tiles(ctx)
        self._show_map.value = True
        self._message = f"loaded coverage of {int(np.count_nonzero(grid.samples))} cells from {file.name}"
        ctx.log_info(self._message)

    # --- --- --- --- --- UPDATE --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Count the latest probe message in the coverage map, while mapping.

        Args:
            ctx: This plugin's context.
        """
        probe = self._probe
        if not self._mapping.value or probe is None or probe.position is None:
            return
        if probe.last_update_time is None or probe.last_update_time == self._counted_update:
            return
        self._counted_update = probe.last_update_time
        tracked = bool(probe.tracked and probe.tracking_valid)
        # ? A tracked message without a marker error counts as not tracked; the relay always sends one.
        error = probe.marker_error if tracked else None
        self._probe_cell = self.coverage.add(probe.position[0], probe.position[1], error)
        if self._probe_cell is not None:
            self._dirty.add(self._probe_cell)

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _selected_samples(self) -> list[Sample]:
        """The samples ticked in the list, in recording order."""
        return [sample for sample in self.samples if self._list.is_selected(sample.id)]

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
        self._markers[sample.id] = (sphere, label)

    def _build_tiles(self, ctx: PluginContext) -> None:
        """Add one floor tile per coverage cell, coloured by its rate; once per grid."""
        if self._coverage_root is not None:
            return
        path = f"{ctx.view.scene_root}/coverage"
        self._coverage_root = ctx.view.scene.add_frame(path, show_axes=False)
        # * A gap between tiles shows the grid.
        side = self.coverage.cell_size * 0.92
        for i in range(self.coverage.shape[0]):
            for j in range(self.coverage.shape[1]):
                self._tiles[i, j] = ctx.view.scene.add_box(
                    f"{path}/{i}_{j}", color=coverage_color(self.coverage.rate(i, j)), dimensions=(side, side, 0.002),
                    position=(*self.coverage.center(i, j), COVERAGE_Z), cast_shadow=False, receive_shadow=False)
        self._dirty.clear()

    def _remove_tiles(self) -> None:
        """Remove the floor tiles, e.g. before loading a map with another grid."""
        for tile in self._tiles.values():
            tile.remove()
        self._tiles.clear()
        if self._coverage_root is not None:
            self._coverage_root.remove()
            self._coverage_root = None

    def _coverage_html(self) -> str:
        """The coverage section's summary: visited cells, overall rate and the cell under the probe."""
        grid = self.coverage
        total = int(grid.samples.sum())
        visited = int(np.count_nonzero(grid.samples))
        rate = f"{grid.tracked.sum() / total:.0%}" if total else "–"
        if self._probe_cell is None:
            here = "probe outside the grid" if self._mapping.value and total else ""
        else:
            cell_rate = grid.rate(*self._probe_cell)
            here = (f"here {cell_rate:.0%} of {grid.samples[self._probe_cell]}, "
                    f"err {_error_mm(grid.error(*self._probe_cell))}" if cell_rate is not None else "")
        state = chip("mapping", OK) if self._mapping.value else chip("off", NONE)
        return block(state + values(f"cells {visited}/{grid.shape[0] * grid.shape[1]}, tracked {rate}",
                                    f"err {_error_mm(grid.total_error())}", here))

    def _big_status(self, ctx: PluginContext, check: Check | None) -> str:
        """The big status panel's HTML: one colour and one word, readable at a distance, and the probe's x y z.

        ! Any text change re-renders it: the only live numbers are x y z, rounded to the millimetre.

        Args:
            ctx: This plugin's context.
            check: The probe's mocap check, or None when nothing is tracked.
        """
        # * No check detail here (the side panel's chip has it): only the recorded point's name.
        detail = ""
        if ctx.now() < self._flash_until:
            color, word, detail = FLASH_COLOR, "RECORDED", self.samples[-1].name if self.samples else ""
        elif check is None:
            color, word = NONE, "NOT TRACKING"
        else:
            color, word = LEVEL_COLORS[check.level], BIG_WORDS[check.level]
        if detail:
            detail = f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">{escape(detail)}</div>'
        probe = self._probe
        position = ""
        if probe is not None and probe.position is not None:
            # * Fixed width and sign, so the digits stand still while the probe moves; dim when the fix is stale.
            dim = "opacity:.5;" if check is None or check.level == BAD else ""
            rows = "".join(f"<div>{axis} {value:+7.3f}</div>" for axis, value in zip("xyz", probe.position))
            position = (f'<div style="font:700 clamp(20px,5vw,64px)/1.2 monospace;white-space:pre;'
                        f'margin-top:8px;{dim}">{rows}</div>')
        # ? vh units, so the box grows with a floated, enlarged panel.
        return (f'<div style="background:{color};color:#fff;border-radius:8px;min-height:40vh;'
                f'display:flex;flex-direction:column;align-items:center;justify-content:center;'
                f'text-align:center;padding:12px;margin:0 8px 8px">'
                f'<div style="font-size:clamp(32px,9vw,120px);font-weight:800;line-height:1.1">{word}</div>'
                f'{detail}'
                f'{position}'
                f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">{len(self.samples)} points'
                f'{" · mapping coverage" if self._mapping.value else ""}</div>'
                f'</div>')

    # --- --- --- --- --- DRAW --- --- --- --- ---

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
        self._status.content = block(state + values(f"xyz {numbers(probe.position if has_pose else None, 3, 8, 4)} m",
                                                    f"rpy {numbers(rpy, 3, 8, 1)} °",
                                                    f"err {numbers(error_mm, 1, 8, 2)} mm",
                                                    dim=check is None or check.level == BAD))

        self._list.sync()
        # * Recolour only the spheres whose selection changed.
        selected = {sample.id for sample in self._selected_samples()}
        for sample_id in selected ^ self._highlighted:
            self._markers[sample_id][0].color = SELECTED_COLOR if sample_id in selected else SAMPLE_COLOR
        self._highlighted = selected

        if self._coverage_root is not None:
            self._coverage_root.visible = self._show_map.value
            for cell in self._dirty:
                self._tiles[cell].color = coverage_color(self.coverage.rate(*cell))
        self._dirty.clear()
        self._coverage_status.content = self._coverage_html()
        # ? Plain wrapping text, not `values`: an export path is long.
        self._result.content = note(escape(self._message)) if self._message else ""
