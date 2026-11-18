#!/usr/bin/env python3
"""
Check a process's bars by hand: each bar drawn straight, with the distances between its joints' outer edges to measure.

Opening a bar selects its most important pair of joints: those closing the smallest bracing loop (a triangle first).
Each joint is its true side view, turned about the bar so the bar it connects to is above it: its place along the bar,
its screw and which way its uneven margins face stay true, while the side of the bar it is on does not show.

Measured distances are appended to a CSV log in the process folder (`runs/bar_check.csv`), never rewritten, each row
with its date and time: the newest row of a pair counts, and the bar list shows how each bar stands.
Green OK: its triangle pairs are measured and every reading is within tolerance. Yellow OK: the
triangles are, but another reading is off by up to OTHER_READING_LIMIT. Without triangles its ground pairs, or else all
its pairs, take their place. Bars not measured yet stand out in blue with TRI when they are part of a triangle, and
are greyed out when they are not. Every bar that is part of a triangle is bold.

The "Which joint?" tab compares all readings of a bar: it names the joint to nudge, or the reading to take next.

Run with:  python3 scripts/bar_check.py <process folder or process.json> [--log file.csv] [--tolerance mm]
           [--resolution 1|0.5]

Keys: type a number then Enter to go to that bar, left/right bar, up/down pair, f flip the bar, v 3D view, Esc show
every distance. m enters a measured distance: Enter logs it and goes to the next pair, up/down skip to another pair,
n selects the reading the solver asks for.
Click two joints to measure any pair, here or in the 3D view, where clicking another bar goes to it.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import queue
import re
import shutil
import time
import tkinter as tk
from collections import defaultdict, deque
from dataclasses import dataclass, replace
from datetime import datetime
from itertools import combinations
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union

from scaffolding_assembly_core.geometry import CylinderShape, shape_mesh
from scaffolding_assembly_core.process import Process, process_hashes, read

from bar_check_3d import Bar3DView, Dimension
from bar_check_solver import (CONTRADICTS, EITHER, FINE, NO_READINGS, SHARED, TOO_SMALL, UNDECIDED, WRONG, Reading,
                              Verdict, solve)

#: Pairs closing a loop of at most this many bars are worth measuring (2: both on the ground, 3: triangle, 4: quad).
LARGEST_LOOP = 4
#: Loop 0 is a pair picked by hand.
LOOP_NAMES = {0: "PICK", 2: "GND", 3: "TRI", 4: "QUAD"}
GROUND = "ground"
#: How often the window takes up what was clicked in the 3D view, milliseconds.
PICK_POLL_MS = 40
#: The drawing's height scale is at least this, pixels per mm: joints stay readable on the longest bars.
MIN_HEIGHT_SCALE = 0.9

INK, MUTED, FAINT, PAPER = "#1f2933", "#7b8794", "#d5dbe1", "#ffffff"
BAR_FILL, JOINT_FILL = "#9aa5b1", "#52606d"
OUTER_COLOR, PICK_COLOR = "#1d4ed8", "#0f766e"
#: Each kind of joint's fill, in the drawing and in the key.
KIND_FILLS = {"Male": "#bfe3d0", "Female": "#f3c9dc", "MoCap": "#fde68a", "Sub Female": "#d8c8f0", "Ground": "#d6d3d1"}
RESTING_FILL = "#f3f5f7"

#: What the log says was measured: the distance between the two joints' outer edges.
OUTER = "outer"
#: What the log says when a joint was moved: its earlier readings stop counting.
MOVED = "moved"
#: A second press within this long confirms "I moved this joint", seconds.
CONFIRM_SECONDS = 5.0
#: A second press sooner than this is a double click: ignored, the first still waits, seconds.
DOUBLE_CLICK_SECONDS = 0.25
#: A measurement within this of the design is good, mm, until the window says otherwise.
DEFAULT_TOLERANCE = 1.0
#: A reading outside a bar's required pairs may be this far from the design before the bar counts as bad, mm.
OTHER_READING_LIMIT = 5.0
#: A measurement further than this from the design takes a second Enter: probably a typo or the wrong edges, mm.
SUSPICIOUS_ERROR = 20.0
#: The log in a process folder. A file: every folder in `runs/` is taken for an assembly run.
LOG_FILE = Path("runs") / "bar_check.csv"
#: Where logs were kept before they moved into the process folder, one per process.
OLD_LOG_FOLDER = Path.home() / "bar_check_logs"
LOG_FIELDS = ("time", "process", "process_hash", "bar", "joint_a", "joint_b", "pair_type", "kind", "design_mm",
              "measured_mm", "error_mm", "tolerance_mm", "within_tolerance")
#: The states of a bar or pair in the lists: background, text and background when selected.
GOOD, ACCEPTED, PARTIAL, UNFINISHED, BAD = "good", "accepted", "partial", "unfinished", "bad"
#: The states of a bar with nothing measured yet: part of a triangle, or not.
TRIANGLE_WAITING, NO_TRIANGLE = "triangle waiting", "no triangle"
STATE_COLORS = {GOOD: ("#d1fae5", "#065f46", "#047857"), ACCEPTED: ("#fde047", "#713f12", "#a16207"),
                PARTIAL: ("#e5e7eb", "#374151", "#4b5563"), UNFINISHED: ("#fef3c7", "#92400e", "#b45309"),
                BAD: ("#fee2e2", "#991b1b", "#b91c1c"), TRIANGLE_WAITING: ("#dbeafe", "#1e3a8a", "#1d4ed8"),
                NO_TRIANGLE: ("#ffffff", "#9aa5b1", "#7b8794")}


@dataclass(frozen=True)
class Joint:
    """A connector half on a bar.

    Attributes:
        id: Its body id, e.g. "joints/J21-26_male".
        part: Its part, e.g. "T20/Male".
        low: Its face nearer the drawn bar's left end, mm from that end.
        high: Its other face, likewise.
        partner: The bar or ground it mates with ("B26", "ground"), or "" when it mates with nothing in this process.
        shape: Its side view's outlines, points (along, up) in mm: along as `low`, up from the bar's axis.
        screw: Where its screw sits along the bar, mm as `low`, or None when it has no screw or none is known.
    """

    id: str
    part: str
    low: float
    high: float
    partner: str
    shape: tuple[tuple[tuple[float, float], ...], ...] = ()
    screw: float | None = None

    @property
    def top(self) -> float:
        """float: How far its side view reaches above the bar's axis, mm."""
        return max((point[1] for outline in self.shape for point in outline), default=20.0)

    @property
    def bottom(self) -> float:
        """float: How far its side view reaches below the bar's axis, mm (negative)."""
        return min((point[1] for outline in self.shape for point in outline), default=-20.0)

    @property
    def margins(self) -> str:
        """str: The distances from its screw to its left and right edges, mm, or its width without a screw."""
        if self.screw is None:
            return f"{self.high - self.low:g} wide"
        return f"{round(self.screw - self.low, 1):g} | {round(self.high - self.screw, 1):g}"

    @property
    def label(self) -> str:
        """str: What the drawing calls it: its partner, its ground foot's number, or its name when it has no partner."""
        if self.partner == GROUND:
            return f"{GROUND}-{self.id.removesuffix('_ground').rpartition('-')[2]}"
        return self.partner or self.id.removeprefix("joints/")

    @property
    def kind(self) -> str:
        """str: Its kind in short, e.g. "Male", "MoCap", "SubLeft Female"."""
        family, _, kind = self.part.partition("/")
        sub = family.removeprefix("T20")
        return f"{sub} {kind}" if sub else kind

    @property
    def family(self) -> str:
        """str: Its kind with left and right hands together, e.g. "Sub Female"."""
        return re.sub(r"Sub(Left|Right)", "Sub", self.kind)


@dataclass(frozen=True)
class Pair:
    """Two joints of a bar worth measuring against each other.

    Attributes:
        a: One joint's id.
        b: The other's.
        loop: How many bars the smallest loop through both joints has.
    """

    a: str
    b: str
    loop: int


@dataclass(frozen=True)
class Bar:
    """A bar with its joints along it and the pairs to measure, the smallest loops first.

    Attributes:
        name: Its short name, e.g. "B26".
        length: Its length, mm.
        joints: Its joints by position along the bar.
        pairs: The pairs closing a loop of at most LARGEST_LOOP bars.
    """

    name: str
    length: float
    joints: tuple[Joint, ...]
    pairs: tuple[Pair, ...]

    def joint(self, joint_id: str) -> Joint:
        """The joint with this id."""
        return next(joint for joint in self.joints if joint.id == joint_id)

    def flipped(self) -> Bar:
        """The same bar seen from its other end."""
        end = self.length
        joints = [replace(joint, low=end - joint.high, high=end - joint.low,
                          shape=tuple(tuple((end - along, up) for along, up in outline) for outline in joint.shape),
                          screw=None if joint.screw is None else end - joint.screw) for joint in self.joints]
        return replace(self, joints=tuple(sorted(joints, key=_middle)))


def _middle(joint: Joint) -> float:
    """Sort key: the joint's middle along the bar."""
    return joint.low + joint.high


def outer(first: Joint, second: Joint) -> float:
    """The distance between the two joints' outer edges, mm."""
    return max(first.high, second.high) - min(first.low, second.low)


def design_distance(bar: Bar, joints: tuple[str, str]) -> float:
    """The design's distance between the outer edges of two joints of `bar`, mm."""
    return outer(bar.joint(joints[0]), bar.joint(joints[1]))


def in_triangle(bar: Bar) -> bool:
    """Whether the bar is part of a triangle."""
    return any(pair.loop == 3 for pair in bar.pairs)


def waiting_state(bar: Bar) -> str:
    """How the list shows a bar with nothing measured yet: triangles are what matters most."""
    return TRIANGLE_WAITING if in_triangle(bar) else NO_TRIANGLE


def required_pairs(bar: Bar) -> list[Pair]:
    """The pairs a bar needs to count as checked: its triangles, or without any its ground pairs, or else all pairs."""
    for loop in (3, 2):
        pairs = [pair for pair in bar.pairs if pair.loop == loop]
        if pairs:
            return pairs
    return list(bar.pairs)


@dataclass(frozen=True)
class Measurement:
    """A distance measured on a bar.

    Attributes:
        time: When it was logged.
        bar: The bar's short name.
        joints: The two joints' ids, sorted.
        measured: The distance measured between their outer edges, mm.
    """

    time: str
    bar: str
    joints: tuple[str, str]
    measured: float

    def error(self, bar: Bar) -> float:
        """Measured minus the design of `bar`, mm."""
        return self.measured - design_distance(bar, self.joints)

    def within(self, bar: Bar, tolerance: float) -> bool:
        """Whether it is within `tolerance` mm of the design."""
        return abs(round(self.error(bar), 6)) <= tolerance


class MeasureLog:
    """The append-only CSV of measurements; the newest row of each pair of joints counts."""

    def __init__(self, path: Path, process_name: str, process_hash: str):
        """Read what `path` already holds; rows that cannot be read are counted and left in the file."""
        self.path, self.process_name, self.process_hash = path, process_name, process_hash
        self.latest: dict[tuple[str, tuple[str, str]], Measurement] = {}
        self.unreadable = 0
        if path.is_file():
            with path.open(newline="", encoding="utf-8", errors="replace") as file:
                lines = [line.replace("\0", "") for line in file]
            # * Line by line: a stray quote in a damaged row must not swallow the rows after it.
            header = next(csv.reader(lines[:1]), [])
            for line in filter(str.strip, lines[1:]):
                try:
                    self._take(header, next(csv.reader([line])))
                except (csv.Error, KeyError, TypeError, ValueError, AttributeError):
                    self.unreadable += 1

    def _take(self, header: list[str], cells: list[str]) -> None:
        """Count one row of the file.

        Raises:
            csv.Error, KeyError, TypeError, ValueError, AttributeError: The row cannot be read.
        """
        if len(cells) != len(header):
            raise ValueError("a row cut short or run together")
        row = dict(zip(header, cells))
        if row["kind"] == MOVED:
            self._forget(row["bar"], row["joint_a"])
            return
        if row["kind"] != OUTER:
            raise ValueError(row["kind"])
        joints = tuple(sorted((row["joint_a"], row["joint_b"])))
        measured = float(row["measured_mm"])
        if not math.isfinite(measured):
            raise ValueError(row["measured_mm"])
        self.latest[(row["bar"], joints)] = Measurement(row["time"], row["bar"], joints, measured)

    def _forget(self, bar: str, joint: str) -> None:
        """Stop counting every reading of a joint that has moved."""
        self.latest = {key: item for key, item in self.latest.items() if key[0] != bar or joint not in key[1]}

    def _last_byte(self) -> bytes:
        """The file's last byte."""
        with self.path.open("rb") as file:
            file.seek(-1, os.SEEK_END)
            return file.read(1)

    def _append(self, row: tuple) -> None:
        """Append one row to the file, on disk before this returns."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not self.path.is_file() or self.path.stat().st_size == 0
        cut = not is_new and self._last_byte() != b"\n"
        with self.path.open("a", newline="", encoding="utf-8") as file:
            # * A line cut by a crash is ended first, so this row starts on a line of its own.
            file.write("\r\n" if cut else "")
            writer = csv.writer(file)
            if is_new:
                writer.writerow(LOG_FIELDS)
            writer.writerow(row)
            file.flush()
            os.fsync(file.fileno())

    def mark_moved(self, bar: Bar, joint: str, error: float | None, tolerance: float) -> None:
        """Log that a joint was moved: its readings so far stay in the file and stop counting.

        Raises:
            OSError: The file could not be written; the readings then still count.
        """
        self._append((datetime.now().astimezone().isoformat(timespec="seconds"), self.process_name, self.process_hash,
                      bar.name, joint, "", "", MOVED, "", "", "" if error is None else f"{error:+.3f}",
                      f"{tolerance:g}", ""))
        self._forget(bar.name, joint)

    def add(self, bar: Bar, joints: tuple[str, str], loop: int, measured: float, tolerance: float) -> Measurement:
        """Append a measurement to the file, on disk before this returns.

        Raises:
            OSError: The file could not be written; the measurement then does not count.
        """
        pair = tuple(sorted(joints))
        measurement = Measurement(datetime.now().astimezone().isoformat(timespec="seconds"), bar.name, pair, measured)
        design = design_distance(bar, pair)
        row = (measurement.time, self.process_name, self.process_hash, bar.name, pair[0], pair[1], LOOP_NAMES[loop],
               OUTER, f"{design:.3f}", f"{measured:.3f}", f"{measured - design:+.3f}", f"{tolerance:g}",
               "yes" if measurement.within(bar, tolerance) else "no")
        self._append(row)
        self.latest[(bar.name, pair)] = measurement
        return measurement

    def of_pair(self, bar: Bar, first: str, second: str) -> Measurement | None:
        """The newest measurement of two joints of `bar`, or None."""
        return self.latest.get((bar.name, tuple(sorted((first, second)))))

    def on_bar(self, bar: Bar) -> list[Measurement]:
        """The newest measurement of every pair of joints still on `bar`."""
        ids = {joint.id for joint in bar.joints}
        return [item for (name, joints), item in self.latest.items() if name == bar.name and set(joints) <= ids]

    @staticmethod
    def reading_state(bar: Bar, measurement: Measurement, tolerance: float) -> str:
        """GOOD within tolerance; BAD out of it on a required pair, on a bar with none, or past OTHER_READING_LIMIT."""
        if measurement.within(bar, tolerance):
            return GOOD
        required = {frozenset((pair.a, pair.b)) for pair in required_pairs(bar)}
        is_required = not required or frozenset(measurement.joints) in required
        if is_required or not measurement.within(bar, OTHER_READING_LIMIT):
            return BAD
        return ACCEPTED

    def state(self, bar: Bar, tolerance: float) -> tuple[int, int, str | None]:
        """How many of a bar's required pairs are measured, how many it has, how it stands (None: nothing measured)."""
        required = required_pairs(bar)
        done = sum(self.of_pair(bar, pair.a, pair.b) is not None for pair in required)
        measured = self.on_bar(bar)
        if any(self.reading_state(bar, item, tolerance) == BAD for item in measured):
            return done, len(required), BAD
        if measured and done == len(required):
            accepted = any(self.reading_state(bar, item, tolerance) == ACCEPTED for item in measured)
            return done, len(required), ACCEPTED if accepted else GOOD
        return done, len(required), PARTIAL if measured else None


def process_folder(path: str) -> Path:
    """The process folder named by a folder or its process.json, in full: its name names the process."""
    folder = Path(os.path.abspath(path))
    return folder.parent if folder.is_file() else folder


def process_log(folder: Path, old_folder: Path = OLD_LOG_FOLDER) -> Path:
    """The log of the process in `folder`; the first time, a copy of its old log if it has one, which stays in place."""
    path, old = folder / LOG_FILE, old_folder / f"{folder.name}.csv"
    if old.is_file() and not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(old, path)
    return path


def bar_number(name: str) -> tuple[int, str]:
    """Sort key: "B26" by 26, then by name."""
    digits = re.sub(r"\D", "", name)
    return (int(digits) if digits else 1 << 30, name)


def short(body_id: str) -> str:
    """A bar's short name, or GROUND for any ground body."""
    return GROUND if body_id.startswith("ground/") else body_id.removeprefix("bars/")


def mate_graph(process: Process) -> tuple[dict[str, str], dict[str, set[str]]]:
    """Which bar or ground each half mates with, and which bars and ground mate with each other, by short name."""
    partner, neighbours = {}, defaultdict(set)
    owner = lambda body: short(process.bodies[body].mount) if body.startswith("joints/") else short(body)  # noqa: E731
    for first, second in process.mates:
        a, b = owner(first), owner(second)
        partner[first], partner[second] = b, a
        neighbours[a].add(b)
        neighbours[b].add(a)
    return partner, neighbours


def loop_size(start: str, goal: str, without: str, neighbours: dict[str, set[str]]) -> int | None:
    """The bars in the smallest loop through `without` that leaves it to `start` and returns from `goal`, or None."""
    if start == goal:
        return 2
    seen, queue = {start, without}, deque([(start, 2)])
    while queue:
        node, size = queue.popleft()
        for neighbour in neighbours[node]:
            if neighbour == goal:
                return size + 1
            if neighbour not in seen:
                seen.add(neighbour)
                queue.append((neighbour, size + 1))
    return None


def load_screws(folder: Path) -> dict[str, np.ndarray]:
    """Each part's screw position in its own frame, metres, from the process's joint definitions; empty without them."""
    path = folder / "source" / "joint_pairs.json"
    if not path.is_file():
        return {}
    halves = json.loads(path.read_text())["halves"]
    return {half["block_name"]: np.array(half["M_screw_from_block"], dtype=float)[:3, 3] * 1e-3 for half in halves}


def bar_axis(process: Process, bar: str) -> tuple[np.ndarray, np.ndarray, float]:
    """A bar's start, its unit axis and its length in metres, from its cylinder."""
    body = process.bodies[bar]
    cylinder = next(shape for shape in body.geometry.collision if isinstance(shape, CylinderShape))
    frame = body.assembly_pose.matrix() @ cylinder.origin.matrix()
    axis = frame[:3, 2] / np.linalg.norm(frame[:3, 2])
    return frame[:3, 3] - axis * cylinder.height / 2.0, axis, float(cylinder.height)


def nearest_axis(direction: np.ndarray) -> np.ndarray:
    """The frame axis, with its sign, closest to `direction`."""
    axis = np.zeros(3)
    index = int(np.argmax(np.abs(direction)))
    axis[index] = np.sign(direction[index])
    return axis


def partner_sides(process: Process) -> dict[str, np.ndarray]:
    """Each part's side facing the bar it connects to, as an axis of its own frame."""
    seen = defaultdict(list)
    for first, second in process.mates:
        if not (first.startswith("joints/") and second.startswith("joints/")):
            continue
        start_a, axis_a, _ = bar_axis(process, process.bodies[first].mount)
        start_b, axis_b, _ = bar_axis(process, process.bodies[second].mount)
        across = np.cross(axis_a, axis_b)
        across = across / np.linalg.norm(across) * np.sign((start_b - start_a) @ across)
        for half, towards in ((first, across), (second, -across)):
            seen[process.bodies[half].part].append(process.bodies[half].assembly_pose.matrix()[:3, :3].T @ towards)
    return {part: nearest_axis(np.mean(directions, axis=0)) for part, directions in seen.items()}


def side_view(meshes: list, along: np.ndarray, up: np.ndarray) -> tuple:
    """The outlines of `meshes` seen from the side, points (along, up) in mm."""
    triangles = []
    for mesh in meshes:
        points = np.column_stack([mesh.vertices @ along, mesh.vertices @ up]) * 1e3
        triangles += [Polygon(points[face]) for face in mesh.faces]
    merged = unary_union([triangle for triangle in triangles if triangle.area > 1e-6])
    merged = merged.buffer(0.05, join_style="mitre").buffer(-0.05, join_style="mitre").simplify(0.1)
    return tuple(tuple(tuple(map(float, point)) for point in polygon.exterior.coords[:-1])
                 for polygon in getattr(merged, "geoms", [merged]) if polygon.area > 1.0)


def load_bars(process: Process, screws: dict[str, np.ndarray] | None = None) -> list[Bar]:
    """Every bar of `process` by number: its joints' edges and side views, and the pairs worth measuring."""
    partner, neighbours = mate_graph(process)
    sides = partner_sides(process)
    views: dict[str, tuple] = {}
    bars = []
    for bar_id in sorted((body for body in process.bodies if body.startswith("bars/")), key=bar_number):
        start, axis, length = bar_axis(process, bar_id)
        joints = []
        for half in process.halves_of(bar_id):
            joint = process.bodies[half]
            pose = joint.assembly_pose.matrix()
            meshes = [shape_mesh(shape) for shape in joint.geometry.collision]
            vertices = np.vstack([mesh.vertices for mesh in meshes])
            along = (vertices @ pose[:3, :3].T + pose[:3, 3] - start) @ axis * 1e3
            origin = float((pose[:3, 3] - start) @ axis * 1e3)
            own_axis = nearest_axis(pose[:3, :3].T @ axis)
            forward = np.abs(own_axis)
            direction = float(own_axis @ forward)
            if joint.part not in views:
                across = vertices - np.outer(vertices @ forward, forward)
                up = sides.get(joint.part, nearest_axis(across.mean(axis=0)))
                views[joint.part] = side_view(meshes, forward, up)
            shape = tuple(tuple((origin + direction * point[0], point[1]) for point in outline)
                          for outline in views[joint.part])
            screw = (screws or {}).get((joint.part or "").replace("/", "_"))
            at = None if screw is None else float((pose[:3, :3] @ screw + pose[:3, 3] - start) @ axis * 1e3)
            joints.append(Joint(half, joint.part or "", float(along.min()), float(along.max()), partner.get(half, ""),
                                shape, at))
        joints.sort(key=_middle)
        name = short(bar_id)
        pairs = []
        for first, second in combinations((joint for joint in joints if joint.partner), 2):
            size = loop_size(first.partner, second.partner, name, neighbours)
            if size is not None and size <= LARGEST_LOOP:
                pairs.append(Pair(first.id, second.id, size))
        pairs.sort(key=lambda pair: (pair.loop if pair.loop > 2 else 3.5, pair.a, pair.b))
        bars.append(Bar(name, length * 1e3, tuple(joints), tuple(pairs)))
    return bars


class BarRows:
    """A list like `tk.Listbox` whose rows can be bold, which a Listbox cannot do."""

    def __init__(self, parent: tk.Widget, width: int):
        """Make the empty list."""
        self.text = tk.Text(parent, width=width, font=("Courier", 12), wrap=tk.NONE, cursor="arrow", takefocus=0,
                            state=tk.DISABLED, bg=PAPER, highlightthickness=1, highlightbackground=FAINT)
        self.rows: list[dict[str, str]] = []
        self.chosen: int | None = None
        self.text.bind("<Button-1>", self._clicked)
        for event in ("<B1-Motion>", "<Double-Button-1>", "<Triple-Button-1>"):
            self.text.bind(event, lambda _event: "break")

    def pack(self, **options) -> None:
        """Place the list."""
        self.text.pack(**options)

    def bind(self, event: str, action) -> None:
        """Call `action` on `event`; a click on a row sends <<ListboxSelect>>."""
        self.text.bind(event, action)

    def delete(self, _first, _last) -> None:
        """Remove every row."""
        self.text.configure(state=tk.NORMAL)
        self.text.delete("1.0", tk.END)
        self.text.configure(state=tk.DISABLED)
        self.rows, self.chosen = [], None

    def insert(self, _where, words: str) -> None:
        """Add a row at the end."""
        row = len(self.rows)
        self.rows.append({"background": PAPER, "foreground": INK, "selectbackground": JOINT_FILL,
                          "selectforeground": PAPER})
        self.text.configure(state=tk.NORMAL)
        self.text.insert(f"{row + 1}.0", f"{words}\n", f"row{row}")
        self.text.configure(state=tk.DISABLED)
        self._show(row)

    def itemconfigure(self, row: int, **colors: str) -> None:
        """Set the row's colours, as a Listbox names them."""
        self.rows[row].update(colors)
        self._show(row)

    def set_bold(self, row: int, bold: bool) -> None:
        """Make the row bold or plain."""
        self.text.tag_configure(f"row{row}", font=("Courier", 12, "bold" if bold else "normal"))

    def selection_clear(self, _first, _last) -> None:
        """Choose no row."""
        chosen, self.chosen = self.chosen, None
        if chosen is not None:
            self._show(chosen)

    def selection_set(self, row: int) -> None:
        """Choose the row, if there is one."""
        self.selection_clear(0, tk.END)
        if 0 <= row < len(self.rows):
            self.chosen = row
            self._show(row)

    def curselection(self) -> tuple[int, ...]:
        """The chosen row, as a Listbox gives it."""
        return () if self.chosen is None else (self.chosen,)

    def see(self, row: int) -> None:
        """Scroll the row into view."""
        self.text.see(f"{row + 1}.0")

    def _show(self, row: int) -> None:
        """Colour the row by its colours and whether it is chosen."""
        colors = self.rows[row]
        prefix = "select" if row == self.chosen else ""
        self.text.tag_configure(f"row{row}", background=colors[f"{prefix}background"],
                                foreground=colors[f"{prefix}foreground"])

    def _clicked(self, event) -> str:
        """Choose the clicked row."""
        row = int(self.text.index(f"@{event.x},{event.y}").split(".")[0]) - 1
        if 0 <= row < len(self.rows):
            self.selection_set(row)
            self.text.event_generate("<<ListboxSelect>>")
        return "break"


class BarCheck:
    """The window: the bars on the left; the drawing and the bar's pairs on the right."""

    def __init__(self, root: tk.Tk, process: Process, bars: list[Bar], title: str, log: MeasureLog,
                 tolerance: float = DEFAULT_TOLERANCE, resolution: float = 1.0):
        """Build the window and show the first bar."""
        self.root, self.process, self.all_bars = root, process, bars
        self.log, self.tolerance = log, tolerance
        self.reading_step = tk.DoubleVar(value=resolution)
        self.verdict = Verdict(NO_READINGS)
        #: The joints waiting for a second press of "I moved this joint", and when the first press was.
        self.move_waiting: tuple[tuple[str, ...], float] | None = None
        self.rows: list[Pair] = []
        self.index = 0
        #: The measurement waiting for a second Enter because it is far from the design.
        self.waiting: tuple | None = None
        self.view_3d: Bar3DView | None = None
        self.picks_3d: queue.SimpleQueue[tuple[str, str | None]] = queue.SimpleQueue()
        self.is_flipped = False
        self.only_triangles = tk.BooleanVar(value=True)
        self.bars = self._listed_bars()
        self.bar: Bar = self.bars[0]
        self.selected: tuple[str, str] | None = None
        self.picked: str | None = None
        self.joint_boxes: list[tuple[float, float, float, float, str]] = []
        root.title(f"Bar check: {title}")
        root.geometry("1400x820")
        root.configure(bg=PAPER)
        self._build()
        self.show(0)

    def _build(self) -> None:
        """Lay out the widgets and bind the keys."""
        # ! The number boxes refuse anything else, also shortcut letters held down and pasted text.
        whole = (self.root.register(lambda text: re.fullmatch(r"\d*", text) is not None), "%P")
        decimal = (self.root.register(lambda text: re.fullmatch(r"\d*[.,]?\d*", text) is not None), "%P")
        left = ttk.Frame(self.root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.Y)
        jump = ttk.Frame(left)
        jump.pack(fill=tk.X)
        ttk.Label(jump, text="Bar number (just type it)").pack(side=tk.LEFT)
        self.number = ttk.Entry(jump, width=6, validate="key", validatecommand=whole)
        self.number.pack(side=tk.LEFT, padx=6)
        self.number.bind("<Return>", self._jump)
        self.number.bind("<KP_Enter>", self._jump)
        self.number.bind("<Escape>", self._leave_number)
        self.number.bind("m", self._number_to_measured)
        steps = ttk.Frame(left)
        steps.pack(fill=tk.X, pady=6)
        ttk.Button(steps, text="< Prev", command=lambda: self.step(-1)).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(steps, text="Next >", command=lambda: self.step(1)).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Checkbutton(left, text="Only bars with triangles", variable=self.only_triangles,
                        command=self._filter).pack(anchor=tk.W, pady=(0, 6))
        limit = ttk.Frame(left)
        limit.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(limit, text="Good within +/-").pack(side=tk.LEFT)
        self.tolerance_text = tk.StringVar(value=f"{self.tolerance:g}")
        self.tolerance_box = ttk.Spinbox(limit, from_=0.1, to=20.0, increment=0.1, width=5,
                                         textvariable=self.tolerance_text, command=self._tolerance_changed,
                                         validate="key", validatecommand=decimal)
        self.tolerance_box.pack(side=tk.LEFT, padx=4)
        self.tolerance_box.bind("<Return>", self._tolerance_entered)
        self.tolerance_box.bind("<KP_Enter>", self._tolerance_entered)
        self.tolerance_box.bind("<FocusOut>", lambda _event: self._tolerance_changed())
        ttk.Label(limit, text="mm").pack(side=tk.LEFT)
        self.bar_list = BarRows(left, 22)
        self.bar_list.pack(fill=tk.Y, expand=True)
        self.bar_list.bind("<<ListboxSelect>>", self._bar_clicked)
        skipped = f"\n{self.log.unreadable} unreadable rows skipped" if self.log.unreadable else ""
        ttk.Label(left, text=f"Log: {self.log.path}{skipped}", foreground=MUTED, wraplength=220,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))

        right = ttk.Frame(self.root, padding=(0, 8, 8, 8))
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.title = ttk.Label(right, font=("Helvetica", 18, "bold"))
        self.title.pack(anchor=tk.W)
        self.hint = ttk.Label(right, foreground=MUTED)
        self.hint.pack(anchor=tk.W, pady=(0, 4))
        self.canvas = tk.Canvas(right, bg=PAPER, highlightthickness=1, highlightbackground=FAINT)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.canvas.bind("<Button-1>", self._canvas_clicked)
        buttons = ttk.Frame(right)
        buttons.pack(fill=tk.X, pady=6)
        ttk.Button(buttons, text="Show all distances (Esc)", command=self.show_all).pack(side=tk.LEFT)
        ttk.Button(buttons, text="Flip bar (f)", command=self.flip).pack(side=tk.LEFT, padx=6)
        ttk.Button(buttons, text="Open 3D view (v)", command=self.open_3d).pack(side=tk.LEFT)
        entry = ttk.Frame(right)
        entry.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(entry, text="Measured (m)").pack(side=tk.LEFT)
        self.measured = ttk.Entry(entry, width=9, validate="key", validatecommand=decimal)
        self.measured.pack(side=tk.LEFT, padx=4)
        self.measured.bind("<Return>", self.log_measurement)
        self.measured.bind("<KP_Enter>", self.log_measurement)
        self.measured.bind("<Escape>", self._leave_measured)
        self.measured.bind("<Up>", lambda _event: self.step_pair(-1) or "break")
        self.measured.bind("<Down>", lambda _event: self.step_pair(1) or "break")
        ttk.Label(entry, text="mm").pack(side=tk.LEFT)
        log_button = ttk.Button(entry, text="Log (Enter)", command=self.log_measurement, takefocus=0)
        log_button.pack(side=tk.LEFT, padx=(10, 0))
        ttk.Label(entry, text="Read to").pack(side=tk.LEFT, padx=(12, 2))
        for step, text in ((1.0, "1 mm"), (0.5, "0.5 mm")):
            ttk.Radiobutton(entry, text=text, value=step, variable=self.reading_step, command=self.draw,
                            takefocus=0).pack(side=tk.LEFT)
        self.design_label = ttk.Label(entry, width=18)
        self.design_label.pack(side=tk.LEFT, padx=10)
        self.entry_status = ttk.Label(entry)
        self.entry_status.pack(side=tk.LEFT)
        bottom = ttk.Frame(right)
        bottom.pack(fill=tk.X)
        pairs = ttk.Frame(bottom)
        pairs.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Label(pairs, text=f"{'':<5}{'joints':^19} {'outer':>7}  {'meas.':>7} {'err':>5}",
                  font=("Courier", 12), foreground=MUTED).pack(anchor=tk.W)
        self.pair_list = tk.Listbox(pairs, width=55, height=7, font=("Courier", 12), exportselection=False,
                                    takefocus=0)
        self.pair_list.pack(side=tk.LEFT, fill=tk.Y)
        self.pair_list.bind("<<ListboxSelect>>", self._pair_clicked)
        tabs = ttk.Notebook(bottom)
        tabs.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0))
        solver = ttk.Frame(tabs, padding=8)
        tabs.add(solver, text="Which joint?")
        self.verdict_title = ttk.Label(solver, font=("Helvetica", 14, "bold"), wraplength=560, justify=tk.LEFT)
        self.verdict_title.pack(anchor=tk.W)
        self.verdict_detail = ttk.Label(solver, wraplength=560, justify=tk.LEFT)
        self.verdict_detail.pack(anchor=tk.W, pady=(2, 6))
        actions = ttk.Frame(solver)
        actions.pack(side=tk.BOTTOM, fill=tk.X)
        self.next_button = ttk.Button(actions, command=self.measure_next, takefocus=0)
        self.next_button.pack(side=tk.LEFT)
        self.moved_button = ttk.Button(actions, command=self.press_moved, takefocus=0)
        self.moved_button.pack(side=tk.LEFT, padx=6)
        self.key = tk.Canvas(tabs, bg=PAPER, height=160, highlightthickness=1, highlightbackground=FAINT)
        tabs.add(self.key, text="Key")
        self.key.bind("<Configure>", lambda _event: self._draw_key())

        for key, action in (("<Left>", lambda: self.step(-1)), ("<Right>", lambda: self.step(1)),
                            ("<Up>", lambda: self.step_pair(-1)), ("<Down>", lambda: self.step_pair(1)),
                            ("<Escape>", self.show_all), ("f", self.flip), ("v", self.open_3d),
                            ("m", self.measured.focus_set), ("n", self.measure_next)):
            self.root.bind(key, self._key(action))
        self.root.bind("<Key>", self._start_number)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._fill_bar_list()

    def _key(self, action):
        """A key binding that leaves the keys alone while something is being typed."""
        def handler(_event):
            if not self._typing():
                action()
                return "break"
            return None
        return handler

    def _typing(self) -> bool:
        """Whether the keyboard is in one of the window's text fields."""
        return isinstance(self.root.focus_get(), ttk.Entry)

    def _start_number(self, event) -> str | None:
        """A digit typed outside the text fields starts entering a bar number."""
        if self._typing() or not event.char.isdigit():
            return None
        self.number.focus_set()
        self.number.delete(0, tk.END)
        self.number.insert(tk.END, event.char)
        return "break"

    def _number_to_measured(self, _event) -> str:
        """Give up entering a bar number and enter a measured distance instead."""
        self.number.delete(0, tk.END)
        self.measured.focus_set()
        return "break"

    def _leave_number(self, _event) -> str:
        """Give up entering a bar number."""
        self.number.delete(0, tk.END)
        self.root.focus_set()
        return "break"

    # --- --- --- --- --- MEASUREMENTS --- --- --- --- ---

    def say(self, text: str, color: str = MUTED) -> None:
        """Show what just happened beside the measured distance."""
        self.entry_status.configure(text=text, foreground=color)

    def _leave_measured(self, _event) -> str:
        """Give up entering a measured distance."""
        self.measured.delete(0, tk.END)
        self.waiting = None
        self.say("")
        self.root.focus_set()
        return "break"

    def _update_entry(self) -> None:
        """Show the design's distance for this selection."""
        if self.selected is None:
            self.design_label.configure(text="Select two joints", foreground=MUTED)
        else:
            self.design_label.configure(text=f"design {design_distance(self.bar, self.selected):.1f} mm",
                                        foreground=OUTER_COLOR)

    def log_measurement(self, _event=None) -> str:
        """Log the typed distance for the selected pair, then go to the bar's next pair still to measure."""
        text = self.measured.get().strip().replace(",", ".")
        if self.selected is None:
            self.say("Select two joints first.", STATE_COLORS[BAD][1])
            return "break"
        if not text:
            self.root.focus_set()
            return "break"
        try:
            value = float(text)
        except ValueError:
            value = math.nan
        if not math.isfinite(value) or value <= 0.0:
            self.say(f'"{text}" is not a distance in mm.', STATE_COLORS[BAD][1])
            return "break"
        error = value - design_distance(self.bar, self.selected)
        attempt = (self.bar.name, frozenset(self.selected), value)
        if abs(error) > SUSPICIOUS_ERROR and self.waiting != attempt:
            self.waiting = attempt
            self.say(f"{_signed(error)} mm from the design: check it, Enter again to log.", STATE_COLORS[UNFINISHED][1])
            return "break"
        row = self._row_of(self.selected)
        try:
            measurement = self.log.add(self.bar, self.selected, row.loop if row is not None else 0, value,
                                       self.tolerance)
        except OSError as failure:
            self.say(f"Not logged: {failure}", STATE_COLORS[BAD][1])
            return "break"
        self.waiting = None
        self.measured.delete(0, tk.END)
        self._fill_bar_list()
        self._fill_pair_list()
        state = self.log.reading_state(self.bar, measurement, self.tolerance)
        words = {GOOD: "good", ACCEPTED: f"accepted, within {OTHER_READING_LIMIT:g} mm",
                 BAD: "out of tolerance"}[state]
        result = f"Logged {value:.1f} mm: {_signed(error)}, {words}."
        following = self._following()
        if following is not None:
            self.select(following)
            self.measured.focus_set()
        else:
            self.draw()
            self.root.focus_set()
            if self.verdict.status not in (WRONG, EITHER):
                result += f" {self.bar.name} has no pair left to measure."
        self.say(result, STATE_COLORS[state][1])
        return "break"

    # --- --- --- --- --- WHICH JOINT --- --- --- --- ---

    def _solve(self) -> Verdict:
        """What this bar's readings say, on the bar as stored: the drawing's flip must not change the answer."""
        stored = self.bars[self.index]
        measured = self.log.on_bar(stored)
        read = {joint for item in measured for joint in item.joints}
        places = {joint.id: (joint.low, joint.high) for joint in stored.joints if joint.partner or joint.id in read}
        readings = [Reading(*item.joints, item.error(stored)) for item in measured]
        return solve(places, readings, self.reading_step.get(), self.tolerance)

    def _name(self, joint: str) -> str:
        """What the window calls a joint."""
        return self.bar.joint(joint).label

    def _nudge_text(self, joint: str, error: float) -> str:
        """How to move a joint back, named after the joint it moves towards; to the nearest 0.5 mm, as readings are."""
        stored = self.bars[self.index].joints
        index = [item.id for item in stored].index(joint)
        towards = stored[index - 1] if error > 0 and index > 0 else stored[index + 1] if error < 0 and \
            index + 1 < len(stored) else None
        amount = f"{round(abs(error) * 2.0) / 2.0:g} mm"
        if towards is not None:
            return f"Move {self._name(joint)} {amount} towards {towards.label}."
        away = stored[index + 1] if error > 0 else stored[index - 1]
        return f"Move {self._name(joint)} {amount} away from {away.label}."

    def _after_move_text(self) -> str:
        """What to do once the named joints are moved: measure again, and look at any other reading out of tolerance."""
        stored = self.bars[self.index]
        named = set(self.verdict.candidates[0].joints)
        others = [item for item in self.log.on_bar(stored)
                  if not named & set(item.joints) and not item.within(stored, self.tolerance)]
        worst = max(others, key=lambda item: abs(item.error(stored)), default=None)
        if worst is None:
            return "Measure again after moving it."
        pair = f"{self._name(worst.joints[0])} + {self._name(worst.joints[1])}"
        return f"Measure again after moving it, then look at {pair} ({_signed(worst.error(stored))} mm)."

    def _pair_words(self, pair: tuple[str, str]) -> str:
        """Two joints in words."""
        return f"{self._name(pair[0])} + {self._name(pair[1])}"

    def _worst_reading(self) -> tuple[str, float] | None:
        """The reading furthest from the design on this bar: its two joints in words, and its error."""
        stored = self.bars[self.index]
        worst = max(self.log.on_bar(stored), key=lambda item: abs(item.error(stored)), default=None)
        if worst is None:
            return None
        return f"{self._name(worst.joints[0])} + {self._name(worst.joints[1])}", worst.error(stored)

    def _verdict_words(self, verdict: Verdict) -> tuple[str, str, str]:
        """The verdict's headline, what to do about it and the headline's colour."""
        bad, amber, good = STATE_COLORS[BAD][1], STATE_COLORS[UNFINISHED][1], STATE_COLORS[GOOD][1]
        names = " or ".join(" and ".join(self._name(joint) for joint in candidate.joints)
                            for candidate in verdict.candidates)
        if verdict.status == NO_READINGS:
            return "No readings yet.", "Log a reading of this bar to begin.", MUTED
        if verdict.status == FINE:
            missing = ", ".join(self._name(joint) for joint in verdict.unchecked)
            detail = f"Not read yet: {missing}." if missing else "Every joint is read."
            return "No joint is shown to be wrong.", detail, good
        if verdict.status == WRONG:
            moves = " ".join(self._nudge_text(*item) for item in verdict.candidates[0].errors)
            if verdict.confirmed:
                return moves, f"Confirmed by a closed triangle of readings. {self._after_move_text()}", bad
            detail = "Take the reading below to confirm it before moving anything." if verdict.next_pair else \
                "No further reading can confirm it: measure its readings again before moving anything."
            return f"Probably: {moves}", detail, amber
        if verdict.status == EITHER:
            worst = self._worst_reading()
            way = "closer together" if worst[1] > 0 else "further apart"
            return (f"{worst[0]} is {abs(worst[1]):.1f} mm off.",
                    f"This bar has only these two joints: move either one so they sit {way}.", bad)
        if verdict.status == SHARED:
            worst = self._worst_reading()
            way = "short" if worst[1] < 0 else "long"
            moves = " Or: ".join(self._nudge_text(*fix.errors[0]) for fix in verdict.fixes)
            detail = "Several joints disagree, so no single move fixes this. Take the reading below to compare them."
            if moves:
                detail = f"One move brings every reading within tolerance. {moves} Click the joint you move first."
            return f"{worst[0]} is {abs(worst[1]):.1f} mm {way}, but no single joint stands out.", detail, amber
        if verdict.status == UNDECIDED:
            detail = "Take the reading below to tell which." if verdict.can_settle else \
                "No further reading can tell them apart with this tape."
            return f"{names} is wrong.", detail, amber
        if verdict.status == TOO_SMALL:
            worst = self._worst_reading()
            title = f"{worst[0]} is about {abs(worst[1]):.1f} mm off, but no joint is clearly beyond the jig's 1.5 mm."
            if verdict.fixes:
                moves = " Or: ".join(self._nudge_text(*fix.errors[0]) for fix in verdict.fixes)
                return title, f"To bring every reading within tolerance: {moves}", amber
            if self._asked_pair() is None:
                return title, "No reading on this bar can narrow it down further.", amber
            if self.reading_step.get() == 1.0:
                return title, "Read the pair below to 0.5 mm and set Read to 0.5 mm.", amber
            return title, "Take the reading below. An error this small may still stay undecided.", amber
        if verdict.loop:
            around = " > ".join(self._name(joint) for joint in verdict.loop)
            title = f"A reading is wrong: {around} misses closing by {abs(verdict.closure):.1f} mm."
            if verdict.suspect is not None and len(verdict.suspects) == 1:
                return title, f"Measure {self._pair_words(verdict.suspect)} again.", bad
            options = ", ".join(self._pair_words(pair) for pair in verdict.suspects)
            if verdict.next_pair is not None:
                return title, f"It is one of: {options}. The reading below tells which.", bad
            return title, f"It is one of: {options}. Measure them again, starting with the one below.", bad
        return "The readings do not agree.", "Measure the latest pairs again.", bad

    def _update_verdict(self) -> None:
        """Show what this bar's readings say and offer the next step."""
        self.verdict = self._solve()
        title, detail, color = self._verdict_words(self.verdict)
        if self._bar_accepted():
            state = self.log.state(self.bars[self.index], self.tolerance)[2]
            name = f"{self.bar.name} is OK." if state == GOOD else f"{self.bar.name} is OK on its triangles."
            title, detail, color = name, f"Optional: {title}", STATE_COLORS[state][1]
        self.verdict_title.configure(text=title, foreground=color)
        self.verdict_detail.configure(text=detail)
        self._paint_suspects()
        pair = self._asked_pair()
        words = f"{self._name(pair[0])} + {self._name(pair[1])}" if pair is not None else ""
        self.next_button.configure(text=f"Measure next: {words} (n)" if pair else "Nothing to measure next",
                                   state=tk.NORMAL if pair else tk.DISABLED)
        targets = self._move_targets()
        waiting = self.move_waiting is not None and self.move_waiting[0] == targets
        names = ", ".join(self._name(joint) for joint in targets)
        self.moved_button.configure(text=f"Press again: I moved {names}" if waiting else
                                    f"I moved {names}" if targets else "I moved this joint",
                                    state=tk.NORMAL if targets else tk.DISABLED)

    def _bar_accepted(self) -> bool:
        """Whether the bar counts as OK while the solver still sees something small: no nudge is needed then."""
        settled = self.verdict.status in (FINE, NO_READINGS, CONTRADICTS)
        return not settled and self.log.state(self.bars[self.index], self.tolerance)[2] in (GOOD, ACCEPTED)

    def _paint_suspects(self) -> None:
        """Mark in the list of pairs the readings that could be the wrong one in a loop that does not close."""
        suspects = {frozenset(pair) for pair in self.verdict.suspects}
        for row, pair in enumerate(self.rows):
            measurement = self.log.of_pair(self.bar, pair.a, pair.b)
            if frozenset((pair.a, pair.b)) in suspects:
                self._paint_row(self.pair_list, row, UNFINISHED)
            elif measurement is not None:
                self._paint_row(self.pair_list, row, self.log.reading_state(self.bar, measurement, self.tolerance))

    def _asked_pair(self) -> tuple[str, str] | None:
        """The two joints the solver wants read next."""
        if self.verdict.status != CONTRADICTS:
            return self.verdict.next_pair
        if self.verdict.next_pair is not None or len(self.verdict.suspects) < 2:
            return self.verdict.next_pair or self.verdict.suspect
        # The reading longest ago first: one measured again and unchanged then gives way to the next suspect.
        stored = self.bars[self.index]
        return min(self.verdict.suspects, key=lambda pair: self.log.of_pair(stored, *pair).time)

    def _move_targets(self) -> tuple[str, ...]:
        """The joints "I moved this joint" applies to: the one picked by hand, or the ones the solver names."""
        if self.picked is not None:
            return (self.picked,)
        if self.verdict.status in (WRONG, EITHER):
            return self.verdict.candidates[0].joints
        return ()

    def measure_next(self) -> None:
        """Select the pair the solver wants read and get ready to type its distance."""
        pair = self._asked_pair()
        if pair is not None:
            self.select(self._row_of(pair) or Pair(*pair, 0))
            self.measured.focus_set()

    def press_moved(self) -> None:
        """Log the joints as moved, on a second press: their readings stop counting."""
        targets = self._move_targets()
        now = time.monotonic()
        if not targets:
            return
        waiting = self.move_waiting is not None and self.move_waiting[0] == targets
        if waiting and now - self.move_waiting[1] < DOUBLE_CLICK_SECONDS:
            return
        if not waiting or now - self.move_waiting[1] > CONFIRM_SECONDS:
            self.move_waiting = (targets, now)
            self._update_verdict()
            return
        self.move_waiting = None
        errors = dict(self.verdict.candidates[0].errors) if self.verdict.candidates else {}
        stored = self.bars[self.index]
        try:
            for joint in targets:
                self.log.mark_moved(stored, joint, errors.get(joint), self.tolerance)
        except OSError as failure:
            self.say(f"Not logged: {failure}", STATE_COLORS[BAD][1])
            return
        self.picked = None
        self._fill_bar_list()
        self._fill_pair_list()
        self.select(self._following())
        self.say(f"Logged the move of {', '.join(self._name(joint) for joint in targets)}: measure it again.",
                 STATE_COLORS[UNFINISHED][1])

    def _following(self) -> Pair | None:
        """The pair to read next: what the solver asks for, then the bar's pairs still to measure."""
        self.verdict = self._solve()
        pair = self._asked_pair()
        if pair is not None and self.verdict.status != FINE:
            return self._row_of(pair) or Pair(*pair, 0)
        if self.verdict.status in (WRONG, EITHER):
            return None
        return self._next_to_measure()

    def _next_to_measure(self) -> Pair | None:
        """The next required pair of this bar without a measurement, after the selected one."""
        waiting = [pair for pair in required_pairs(self.bar) if self.log.of_pair(self.bar, pair.a, pair.b) is None]
        if not waiting or self.selected is None:
            return waiting[0] if waiting else None
        order = [pair for pair in self.rows if pair in waiting]
        current = self._row_of(self.selected)
        after = self.rows.index(current) if current is not None else -1
        return next((pair for pair in order if self.rows.index(pair) > after), order[0])

    def _tolerance_entered(self, _event) -> str:
        """Apply a typed tolerance and leave the field."""
        self._tolerance_changed()
        self.root.focus_set()
        return "break"

    def _tolerance_changed(self) -> None:
        """Recolour the lists for the new tolerance; anything but a positive number restores the old one."""
        try:
            value = float(self.tolerance_text.get().replace(",", "."))
        except ValueError:
            value = -1.0
        if not math.isfinite(value) or value <= 0.0:
            self.tolerance_text.set(f"{self.tolerance:g}")
        elif value != self.tolerance:
            self.tolerance = value
            self._fill_bar_list()
            self._fill_pair_list()
            self.draw()

    def open_3d(self) -> None:
        """Open the 3D view, which then follows the selection."""
        if self.view_3d is None:
            self.view_3d = Bar3DView(self.process, on_pick=lambda bar, joint: self.picks_3d.put((bar, joint)))
            self._take_3d_picks()
        self.view_3d.open()
        self._update_3d()

    def _update_3d(self) -> None:
        """Mark this bar and its chosen joints in the 3D view, when it is open."""
        if self.view_3d is not None:
            joints = self.selected or ((self.picked,) if self.picked is not None else ())
            self.view_3d.show(self.bar.name, joints, self._dimensions_3d())

    def _dimensions_3d(self) -> list[Dimension]:
        """The selected pair's distances, along the bar as stored: the 3D view does not flip."""
        if self.selected is None:
            return []
        stored = self.bars[self.index]
        first, second = stored.joint(self.selected[0]), stored.joint(self.selected[1])
        return [Dimension(min(first.low, second.low), max(first.high, second.high),
                          f"{outer(first, second):.1f} mm outer edges", OUTER_COLOR)]

    def _take_3d_picks(self) -> None:
        """Act on what was clicked in the 3D view: its clicks arrive on another thread, Tk works on this one."""
        while not self.picks_3d.empty():
            bar, joint = self.picks_3d.get()
            same_bar = bar == self.bar.name
            if not same_bar:
                self.go_to(bar)
            if joint is not None:
                self.pick(joint)
            elif same_bar:
                self.show_all()
        self.root.after(PICK_POLL_MS, self._take_3d_picks)

    def close(self) -> None:
        """Stop the 3D view and close the window."""
        if self.view_3d is not None:
            self.view_3d.close()
        self.root.destroy()

    # --- --- --- --- --- NAVIGATION --- --- --- --- ---

    def _fill_bar_list(self) -> None:
        """List the bars, coloured by their state, with how many required pairs each has measured."""
        self.bar_list.delete(0, tk.END)
        words = {GOOD: "OK", ACCEPTED: "OK", BAD: "BAD", PARTIAL: "...", TRIANGLE_WAITING: "TRI"}
        for row, bar in enumerate(self.bars):
            done, total, state = self.log.state(bar, self.tolerance)
            state = state or waiting_state(bar)
            count = f"{done}/{total}" if total else "-"
            self.bar_list.insert(tk.END, f"{bar.name:<7}{count:<8}{words.get(state, '')}")
            self._paint_row(self.bar_list, row, state)
            self.bar_list.set_bold(row, in_triangle(bar))
        self.bar_list.selection_set(self.index)
        self.bar_list.see(self.index)

    @staticmethod
    def _paint_row(listbox: tk.Listbox | BarRows, row: int, state: str | None) -> None:
        """Colour one row of a list by its state."""
        if state is not None:
            background, text, selected = STATE_COLORS[state]
            listbox.itemconfigure(row, background=background, foreground=text, selectbackground=selected,
                                  selectforeground=PAPER)

    def _row_of(self, joints: tuple[str, str]) -> Pair | None:
        """The row of the pair list holding these two joints, or None."""
        return next((pair for pair in self.rows if {pair.a, pair.b} == set(joints)), None)

    def _fill_pair_list(self) -> None:
        """List this bar's pairs and any other pair measured on it, each with its newest measurement."""
        listed = {frozenset((pair.a, pair.b)) for pair in self.bar.pairs}
        picked = [Pair(*item.joints, 0) for item in self.log.on_bar(self.bar) if frozenset(item.joints) not in listed]
        self.rows = list(self.bar.pairs) + sorted(picked, key=lambda pair: (pair.a, pair.b))
        self.pair_list.delete(0, tk.END)
        for row, pair in enumerate(self.rows):
            first, second = self.bar.joint(pair.a), self.bar.joint(pair.b)
            measurement = self.log.of_pair(self.bar, pair.a, pair.b)
            self.pair_list.insert(tk.END, f"{LOOP_NAMES[pair.loop]:<5}{first.label:>8} + {second.label:<8} "
                                          f"{_measure_text(first, second)}  {_result_text(self.bar, measurement)}")
            if measurement is not None:
                self._paint_row(self.pair_list, row, self.log.reading_state(self.bar, measurement, self.tolerance))
        self._mark_pair_row()

    def _listed_bars(self) -> list[Bar]:
        """The bars to list: those in a triangle when asked for, every bar when no bar is in one."""
        triangles = [bar for bar in self.all_bars if in_triangle(bar)]
        return triangles if self.only_triangles.get() and triangles else self.all_bars

    def _filter(self) -> None:
        """Apply "Only bars with triangles", staying on the current bar when it is still listed."""
        name = self.bar.name
        self.bars = self._listed_bars()
        self._fill_bar_list()
        names = [bar.name for bar in self.bars]
        self.show(names.index(name) if name in names else 0)

    def show(self, index: int) -> None:
        """Show the bar at `index` of the list, the first pair still to measure selected."""
        self.index = index % len(self.bars)
        self.bar = self.bars[self.index]
        self.is_flipped = False
        self.bar_list.selection_clear(0, tk.END)
        self.bar_list.selection_set(self.index)
        self.bar_list.see(self.index)
        self.selected = None
        self._fill_pair_list()
        self.move_waiting = None
        self.select(self._following() or (self.bar.pairs[0] if self.bar.pairs else None))

    def step(self, change: int) -> None:
        """Go to the next or previous bar."""
        self.show(self.index + change)

    def step_pair(self, change: int) -> None:
        """Go to the next or previous pair of this bar."""
        if not self.rows:
            return
        rows = self.pair_list.curselection()
        row = rows[0] + change if rows else (0 if change > 0 else -1)
        self.select(self.rows[row % len(self.rows)])

    def select(self, pair: Pair | None) -> None:
        """Highlight `pair`, or show every distance when None."""
        self.picked = None
        self.selected = (pair.a, pair.b) if pair is not None else None
        self._selection_changed()

    def _selection_changed(self) -> None:
        """Forget a half-entered measurement, then show the new selection."""
        self.waiting = None
        self.measured.delete(0, tk.END)
        self.say("")
        self._mark_pair_row()
        self.draw()

    def _mark_pair_row(self) -> None:
        """Mark the selected pair in the list of pairs."""
        self.pair_list.selection_clear(0, tk.END)
        pair = self._row_of(self.selected) if self.selected is not None else None
        if pair is not None:
            row = self.rows.index(pair)
            self.pair_list.selection_set(row)
            self.pair_list.see(row)

    def show_all(self) -> None:
        """Forget the selection: every distance between neighbouring joints shows."""
        self.select(None)

    def flip(self) -> None:
        """Turn the drawing end for end, to match the bar on the bench."""
        self.bar = self.bar.flipped()
        self.is_flipped = not self.is_flipped
        self.draw()

    def _jump(self, _event) -> None:
        """Go to the typed bar number, listing every bar when the filter hides it."""
        digits = re.sub(r"\D", "", self.number.get())
        name = f"B{digits}"
        if name not in {bar.name for bar in self.all_bars}:
            self.hint.configure(text=f"There is no bar {name} in this process.", foreground=STATE_COLORS[BAD][1])
            self.number.selection_range(0, tk.END)
            return
        self.go_to(name)
        self.number.delete(0, tk.END)
        self.root.focus_set()

    def go_to(self, name: str) -> None:
        """Show the bar with this name, listing every bar when the filter hides it."""
        if name not in {bar.name for bar in self.bars}:
            self.only_triangles.set(False)
            self._filter()
        self.show([bar.name for bar in self.bars].index(name))

    def _bar_clicked(self, _event) -> None:
        rows = self.bar_list.curselection()
        if rows and rows[0] != self.index:
            self.show(rows[0])
        self.root.focus_set()

    def _pair_clicked(self, _event) -> None:
        rows = self.pair_list.curselection()
        if rows:
            self.select(self.rows[rows[0]])
        self.root.focus_set()

    def _canvas_clicked(self, event) -> None:
        """Pick the clicked joint."""
        self.root.focus_set()
        hit = next((box[4] for box in self.joint_boxes
                    if box[0] - 4 <= event.x <= box[2] + 4 and box[1] - 4 <= event.y <= box[3] + 4), None)
        if hit is not None:
            self.pick(hit)

    def pick(self, hit: str) -> None:
        """Pick a joint of this bar; the second pick measures the two, picking the same one again drops it."""
        if self.picked is None or self.picked == hit:
            self.picked = None if self.picked == hit else hit
            self.selected = None
        else:
            self.selected = (self.picked, hit)
            self.picked = None
        self._selection_changed()

    # --- --- --- --- --- DRAWING --- --- --- --- ---

    def draw(self) -> None:
        """Draw the bar, its joints, and the selected pair's distances or every neighbouring distance."""
        canvas, bar = self.canvas, self.bar
        canvas.delete("all")
        self.joint_boxes = []
        width, height = max(canvas.winfo_width(), 400), max(canvas.winfo_height(), 300)
        margin = 70
        scale = (width - 2 * margin) / bar.length
        height_scale = max(scale, MIN_HEIGHT_SCALE)
        x = lambda mm: margin + mm * scale  # noqa: E731
        axis_y = height * 0.46
        self.height_scale = height_scale
        self.dimensions_y = axis_y + 26 - min((joint.bottom for joint in bar.joints), default=-20.0) * height_scale

        flipped = "   (flipped)" if self.is_flipped else ""
        self.title.configure(text=f"{bar.name}   {bar.length:.0f} mm   {len(bar.joints)} joints{flipped}")
        self._write_hint()
        tube = 10.0 * height_scale
        canvas.create_rectangle(x(0), axis_y - tube, x(bar.length), axis_y + tube, fill="#eef1f4", outline=BAR_FILL)

        chosen = set(self.selected or ())
        lanes = _lanes(bar.joints)
        for index, joint in enumerate(bar.joints):
            lift = lanes[joint.id] * 46
            top, bottom = axis_y - joint.top * height_scale - lift, axis_y - joint.bottom * height_scale - lift
            active = not chosen or joint.id in chosen or joint.id == self.picked
            strong = joint.id in chosen or joint.id == self.picked
            color = PICK_COLOR if joint.id == self.picked else INK if strong else JOINT_FILL if active else FAINT
            fill = KIND_FILLS.get(joint.family, RESTING_FILL) if active else RESTING_FILL
            for outline in joint.shape:
                points = [value for along, up in outline for value in (x(along), axis_y - up * height_scale - lift)]
                canvas.create_polygon(points, fill=fill, outline=color, width=2 if strong else 1,
                                      dash=() if joint.partner else (4, 3))
            for side in (-tube, tube):
                canvas.create_line(x(joint.low), axis_y + side - lift, x(joint.high), axis_y + side - lift,
                                   fill=color, dash=(2, 3))
            self.joint_boxes.append((x(joint.low), top, x(joint.high), bottom, joint.id))
            anchor = x(joint.screw if joint.screw is not None else (joint.low + joint.high) / 2)
            if joint.screw is not None:
                canvas.create_line(anchor, top, anchor, bottom, fill=color, dash=(6, 2, 2, 2))
                canvas.create_oval(anchor - 4, axis_y - 4 - lift, anchor + 4, axis_y + 4 - lift, fill=color,
                                   outline=color)
            label_y = top - 14 - (index % 2) * 34
            canvas.create_line(anchor, top, anchor, label_y + 10, fill=FAINT)
            canvas.create_text(anchor, label_y, text=joint.label, fill=INK if active else FAINT,
                               font=("Helvetica", 12, "bold"))
            canvas.create_text(anchor, label_y - 16, text=joint.kind, fill=MUTED if active else FAINT,
                               font=("Helvetica", 9))
            if active:
                canvas.create_text((x(joint.low) + x(joint.high)) / 2, bottom + 10, text=joint.margins, fill=INK,
                                   font=("Helvetica", 9))

        if self.selected is not None:
            self._draw_pair(bar.joint(self.selected[0]), bar.joint(self.selected[1]), x, axis_y, lanes)
        else:
            self._draw_chain(x)
        self._update_verdict()
        self._draw_verdict(x, axis_y, lanes)
        self._draw_key()
        self._update_entry()
        self._update_3d()

    def _draw_verdict(self, x, axis_y: float, lanes: dict[str, int]) -> None:
        """Mark the joint to nudge with an arrow and its amount, and joints still in doubt with a question mark."""
        canvas, verdict = self.canvas, self.verdict
        if self._bar_accepted():
            return
        if verdict.status == WRONG:
            for joint_id, error in verdict.candidates[0].errors:
                joint = self.bar.joint(joint_id)
                middle = x((joint.low + joint.high) / 2)
                y = axis_y - joint.bottom * self.height_scale - lanes[joint.id] * 46 + 24
                # The joint goes back against its error along the stored bar; a flipped drawing shows that mirrored.
                way = (-1.0 if error > 0 else 1.0) * (-1.0 if self.is_flipped else 1.0)
                color = STATE_COLORS[BAD][2]
                canvas.create_line(middle - way * 26, y, middle + way * 26, y, fill=color, width=5, arrow=tk.LAST)
                canvas.create_text(middle, y + 16, text=f"{round(abs(error) * 2.0) / 2.0:g} mm", fill=color,
                                   font=("Helvetica", 13, "bold"))
        elif verdict.status in (UNDECIDED, TOO_SMALL, SHARED):
            for joint_id in {joint for candidate in verdict.candidates for joint in candidate.joints}:
                joint = self.bar.joint(joint_id)
                y = axis_y - joint.bottom * self.height_scale - lanes[joint.id] * 46 + 24
                canvas.create_text(x((joint.low + joint.high) / 2), y, text="?", fill=STATE_COLORS[UNFINISHED][2],
                                   font=("Helvetica", 20, "bold"))

    def _draw_key(self) -> None:
        """Explain the drawing: every kind of joint, the kinds on this bar in colour, then the symbols."""
        key = self.key
        key.delete("all")
        here = {joint.family for joint in self.bar.joints}
        samples = {}
        for bar in self.all_bars:
            for joint in bar.joints:
                samples.setdefault(joint.family, joint)
        left = 8
        for family in sorted(samples, key=lambda name: list(KIND_FILLS).index(name) if name in KIND_FILLS else 99):
            joint = samples[family]
            on_bar = family in here
            ink = INK if on_bar else FAINT
            scale = min(1.0, 92.0 / (joint.top - joint.bottom))
            middle = left + 32
            base = 118 + joint.bottom * scale
            centre = (joint.low + joint.high) / 2
            for outline in joint.shape:
                points = [value for along, up in outline
                          for value in (middle + (along - centre) * scale, base - up * scale)]
                key.create_polygon(points, fill=KIND_FILLS.get(family, RESTING_FILL) if on_bar else RESTING_FILL,
                                   outline=ink)
            if joint.screw is not None:
                at = middle + (joint.screw - centre) * scale
                key.create_line(at, base - joint.top * scale, at, base - joint.bottom * scale, fill=ink,
                                dash=(6, 2, 2, 2))
                key.create_oval(at - 3, base - 3, at + 3, base + 3, fill=ink, outline=ink)
            key.create_text(middle, 12, text=family, fill=ink, font=("Helvetica", 10, "bold"))
            key.create_text(middle, 136, text=_key_margins(joint), fill=ink, font=("Helvetica", 9))
            left += 68
        rows = [("screw", "Screw position"), ("margins", "Screw to left | right edge, mm"),
                ("top", "Connecting bar on this side"), ("tube", "Bar inside the joint"),
                ("free", "No partner in this process"), ("outer", "Outer edges")]
        for index, (symbol, text) in enumerate(rows):
            self._key_symbol(symbol, left + 4, 16 + index * 21)
            key.create_text(left + 52, 16 + index * 21, text=text, anchor=tk.W, fill=INK, font=("Helvetica", 10))

    def _key_symbol(self, symbol: str, left: float, y: float) -> None:
        """Draw one symbol of the key, 40 pixels wide."""
        key, middle = self.key, left + 20
        if symbol == "screw":
            key.create_line(middle, y - 9, middle, y + 9, fill=INK, dash=(6, 2, 2, 2))
            key.create_oval(middle - 4, y - 4, middle + 4, y + 4, fill=INK, outline=INK)
        elif symbol == "margins":
            key.create_text(middle, y, text="20 | 40", fill=INK, font=("Helvetica", 9))
        elif symbol == "top":
            key.create_line(middle, y + 9, middle, y - 9, fill=INK, width=2, arrow=tk.LAST)
        elif symbol == "tube":
            for side in (-4, 4):
                key.create_line(left, y + side, left + 40, y + side, fill=INK, dash=(2, 3))
        elif symbol == "free":
            key.create_rectangle(left + 6, y - 7, left + 34, y + 7, outline=JOINT_FILL, dash=(4, 3))
        else:
            key.create_line(left, y, left + 40, y, fill=OUTER_COLOR, width=4)

    def _write_hint(self) -> None:
        """Say what the drawing shows and what a click does next."""
        if self.picked is not None:
            text = f"{self.bar.joint(self.picked).label} picked: click the second joint."
        elif self.selected is not None:
            text = "Measure between the coloured edges. Click two joints to measure another pair."
        else:
            text = "Outer distance of each two neighbouring joints, in mm. Click two joints to measure them."
        self.hint.configure(text=text, foreground=MUTED)

    def _draw_chain(self, x) -> None:
        """Dimension each two neighbouring joints' outer edges, on alternating rows since neighbours share a joint."""
        joints = self.bar.joints
        for index, (first, second) in enumerate(zip(joints, joints[1:])):
            self._dimension(x(min(first.low, second.low)), x(max(first.high, second.high)),
                            self.dimensions_y + 42 + (index % 2) * 34, f"{outer(first, second):.1f}", INK,
                            self.dimensions_y, 11)

    def _draw_pair(self, first: Joint, second: Joint, x, axis_y: float, lanes: dict[str, int]) -> None:
        """Dimension the pair's outer edges, with those edges coloured."""
        low, high = min(first, second, key=lambda joint: joint.low), max(first, second, key=lambda joint: joint.high)
        self._edge(x(low.low), axis_y, low, lanes[low.id], OUTER_COLOR)
        self._edge(x(high.high), axis_y, high, lanes[high.id], OUTER_COLOR)
        self._dimension(x(low.low), x(high.high), self.dimensions_y + 58, f"{outer(first, second):.1f} mm outer edges",
                        OUTER_COLOR, self.dimensions_y, 20)

    def _edge(self, at: float, axis_y: float, joint: Joint, lane: int, color: str) -> None:
        """Colour a joint's measured edge over its whole height."""
        lift = lane * 46
        self.canvas.create_line(at, axis_y - joint.top * self.height_scale - lift, at,
                                axis_y - joint.bottom * self.height_scale - lift, fill=color, width=4)

    def _dimension(self, start: float, end: float, y: float, text: str, color: str, from_y: float, size: int) -> None:
        """A dimension line from `start` to `end` at height `y`, with its extension lines and its number above it."""
        canvas = self.canvas
        for at in (start, end):
            canvas.create_line(at, from_y, at, y + 8, fill=color, dash=(3, 3))
        canvas.create_line(start, y, end, y, fill=color, width=2, arrow=tk.BOTH)
        label = canvas.create_text((start + end) / 2, y - 5, text=text, fill=color, anchor=tk.S,
                                   font=("Helvetica", size, "bold"))
        canvas.tag_lower(canvas.create_rectangle(canvas.bbox(label), fill=PAPER, outline=PAPER), label)


def _key_margins(joint: Joint) -> str:
    """A kind of joint's margins for the key, the shorter one first since a joint can face either way."""
    if joint.screw is None:
        return joint.margins
    short_side, long_side = sorted((joint.screw - joint.low, joint.high - joint.screw))
    return f"{round(short_side, 1):g} | {round(long_side, 1):g}"


def _measure_text(first: Joint, second: Joint) -> str:
    """A pair's design distance between outer edges for the list, mm."""
    return f"{outer(first, second):7.1f}"


def _signed(value: float) -> str:
    """A difference in mm with its sign and one decimal; one that rounds to nothing is never "-0.0"."""
    return f"{round(value, 1) + 0.0:+.1f}"


def _result_text(bar: Bar, measurement: Measurement | None) -> str:
    """A pair's newest measurement and its error for the list, mm."""
    return "" if measurement is None else f"{measurement.measured:7.1f} {_signed(measurement.error(bar)):>5}"


def _lanes(joints: tuple[Joint, ...]) -> dict[str, int]:
    """Each joint's drawing lane: joints overlapping along the bar stack above each other."""
    ends: list[float] = []
    lanes = {}
    for joint in sorted(joints, key=lambda item: item.low):
        lane = next((index for index, end in enumerate(ends) if joint.low >= end), len(ends))
        if lane == len(ends):
            ends.append(joint.high)
        ends[lane] = joint.high
        lanes[joint.id] = lane
    return lanes


def main() -> None:
    """Open the process given on the command line, or ask for one."""
    parser = argparse.ArgumentParser(description="Check the distances between the joints on a process's bars.")
    parser.add_argument("process", nargs="?", help="process folder or its process.json")
    parser.add_argument("--log", type=Path, help=f"measurement log, default <process>/{LOG_FILE}")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE, help="good within this many mm")
    parser.add_argument("--resolution", type=float, choices=(1.0, 0.5), default=1.0, help="how finely the tape is read")
    args = parser.parse_args()
    root = tk.Tk()
    path = args.process or filedialog.askopenfilename(title="Open process.json",
                                                      filetypes=[("Process", "process.json")])
    if not path:
        return
    folder = process_folder(path)
    process = read(folder)
    bars = load_bars(process, load_screws(folder))
    if not bars:
        messagebox.showerror("Bar check", f"{folder} has no bars.")
        return
    log_path = args.log or process_log(folder)
    BarCheck(root, process, bars, folder.name, MeasureLog(log_path, folder.name, process_hashes(process)[0][:12]),
             args.tolerance, args.resolution)
    root.mainloop()


if __name__ == "__main__":
    main()
