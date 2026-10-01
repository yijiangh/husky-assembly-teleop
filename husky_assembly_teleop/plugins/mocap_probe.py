"""
A mocap probe: track one rigid body, show its live pose, and record points with it, e.g. obstacle corners.

Points are numbered per label ("table_1", ...) and shown in the 3D view. Delete and
Export act on the selected points, so one session can give one file per obstacle.
A big "Probe" status panel (minimized on the right) is readable from across the room.

! The recorded point is the rigid body's origin: set the pivot in Motive to the probe tip.
! Recording is refused while the mocap chip is red; amber records, with its marker error.

Run with:  -p plugins:="['mocap_probe']"
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from html import escape
from pathlib import Path

import viser
from scipy.spatial.transform import Rotation

from ..ui.checklist import CheckList
from ..plugin_api.context import PluginContext
from ..world.mocap import mocap_check
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.style import (LEVEL_COLORS, NONE, SECTION_CTRL, SECTION_SENSOR, block, check_chip, chip, note, numbers,
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
            self._result: viser.GuiHtmlHandle = gui.add_html("")
            self._list = CheckList(ctx, gui, "Samples", empty_text="no samples yet")

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
        sample = Sample(id=self._next_id, name=f"{label}_{self._label_counts[label]}", label=label,
                        position=self._probe.position.tolist(), orientation=self._probe.orientation.tolist(),
                        mocap_id=self._probe.mocap_id, marker_error=self._probe.marker_error, stamp=ctx.now())
        self._next_id += 1
        self.samples.append(sample)
        self._add_marker(ctx, sample)
        self._list.add(sample.id, sample.name, hint=f"{numbers(sample.position, 3, 8, 4)} m", selected=True)
        self._message = f"recorded {sample.name} at {numbers(sample.position, 3, 8, 4)} m"
        self._flash_until = ctx.now() + FLASH_SECONDS
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

    # --- --- --- --- --- HELPERS --- --- --- --- ---

    def _selected_samples(self) -> list[Sample]:
        """The samples ticked in the list, in recording order."""
        return [sample for sample in self.samples if self._list.is_selected(sample.id)]

    def _check(self, ctx: PluginContext) -> Check | None:
        """The probe's mocap check (same as the health panel's), or None when nothing is tracked."""
        if self._probe is None:
            return None
        return mocap_check(f"mocap {self._probe.mocap_id}", self._probe.mocap_id, self._probe, ctx.now())

    def _add_marker(self, ctx: PluginContext, sample: Sample) -> None:
        """Show a recorded sample in the 3D view: a sphere and its name."""
        path = f"{ctx.view.scene_root}/samples/{sample.id}"
        sphere = ctx.view.scene.add_icosphere(f"{path}/point", radius=0.008, color=SAMPLE_COLOR,
                                              position=tuple(sample.position))
        label = ctx.view.scene.add_label(f"{path}/label", sample.name, position=tuple(sample.position))
        self._markers[sample.id] = (sphere, label)

    def _big_status(self, ctx: PluginContext, check: Check | None) -> str:
        """The big status panel's HTML: one colour and one word, readable at a distance.

        ! No live numbers here: any text change re-renders it.

        Args:
            ctx: This plugin's context.
            check: The probe's mocap check, or None when nothing is tracked.
        """
        if ctx.now() < self._flash_until:
            color, word, detail = FLASH_COLOR, "RECORDED", self.samples[-1].name if self.samples else ""
        elif check is None:
            color, word, detail = NONE, "NOT TRACKING", "enter a mocap id and press Track"
        else:
            color, word, detail = LEVEL_COLORS[check.level], BIG_WORDS[check.level], check.detail
        # ? vh units, so the box grows with a floated, enlarged panel.
        return (f'<div style="background:{color};color:#fff;border-radius:8px;min-height:40vh;'
                f'display:flex;flex-direction:column;align-items:center;justify-content:center;'
                f'text-align:center;padding:12px;margin:0 8px 8px">'
                f'<div style="font-size:clamp(32px,9vw,120px);font-weight:800;line-height:1.1">{word}</div>'
                f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">{escape(detail)}</div>'
                f'<div style="font-size:clamp(14px,2.5vw,32px);opacity:.9">{len(self.samples)} points</div>'
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
        # ? Plain wrapping text, not `values`: an export path is long.
        self._result.content = note(escape(self._message)) if self._message else ""
