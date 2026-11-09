"""
The observer's local recordings in Rerun: one time range of them as one Rerun recording.

Opened by `scripts/observer_logs.py rerun`. Every tick signal becomes one plot entity with a line per channel
(`/<serial>[/<arm>]/<signal>`; a pose as `.../position` and `.../yaw`, the follower and streams split by meaning), each
arm's tool0-vs-target error a plot of its own, every switching state a state timeline (`/states/...`), every event and
log line a text line (`/events/...`, `/log/<source>`), and the poses frames in a 3D view of the lab frame
(`/world/...`): each base, its arms' mounts from the URDF, tool0, the Cartesian target while the compliance controller
runs, the follower's reference and lookahead, every tracked object (a bar with its process shape). Around them the
lab's obstacles from its file in git, each sent base path while it is followed, a see-through ghost of an arm at its
trajectory's end while it runs, and a red line per pair a clearance trip found too close. Two timelines: "time", wall
time (ROS time plus the session's offset, so it reads as the local clock and lines up with `events`), and "since
start", seconds from the range's start. A value held over several ticks is logged at its first and last tick only:
the lines look the same.

The names it reads are in `names.py`; a session older than `names.FORMAT` is read through `UPGRADES`, a newer one as
well as it goes, with a note.

! Imports rerun only inside `log_range`: the monitor never needs it. Plain files, no ROS; reads whole chunks, so run
  it in a process of its own.
"""

from __future__ import annotations

import json
import re
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator

import numpy as np
from trimesh import Trimesh

from scaffolding_assembly_core.geometry import BoxShape, CylinderShape, Pose, compose, invert
from scaffolding_assembly_core.process.read import read

from ...colors import COLLISION, FITTED, MOUNTED, PLAN, rgb01
from ...config import short_serial
from ...plugin_api.trace import load_arrays
from ...shared.recordings.runs import load_description
from ...world.lab import BORDER_COLOR, DEFAULT_LAB, LAB_COLOR, lab_file, mesh_directory_of, read_lab, slug
from .export import CHUNKS, EVENTS, SAVED_MARGIN, copied_sessions, loaded_folder, sessions
from .names import (BACK, BAR_OBJECT, BAR_PREFIX, BASE_MOCAP, BASE_POSE, BATTERY, CLEARANCE_TRIP,
                    ESTOP, FOLLOWER, FOLLOWER_GROUPS, FOLLOWER_STATE, FOLLOWING, FORCE_SENSOR_ZEROED, FORMAT, HOLD_SENT,
                    JOINTS, LATE_SHARE, LATE_TICK, LEASE, LEASE_RELEASED, LEASE_TAKEN, LOG, LOG_LINES_LOST, LOST,
                    MOVING, NOISE, OBJECT, OBJECT_MOCAP, OBJECT_POSE, OBJECT_TRACKED, OBJECT_UNTRACKED, ODOMETRY,
                    PATH_SENT, PATH_STOP_SENT, POSE, PROBLEM, RELAY_DRIFT, RELAY_STATE, RUN, RUN_AS_BUILT,
                    RUN_CANCEL, RUN_CHECK_GOAL, RUN_CONFIRM, RUN_CONTINUE, RUN_EXEC, RUN_FORGET, RUN_GUIDE,
                    RUN_MARK_DONE, RUN_NEW, RUN_PLAN, RUN_PROMPT, RUN_REOPEN, RUN_SERVO, RUN_SESSION, RUN_SOFT_STOP,
                    SESSION_END, SESSION_START, SOFT_STOP, STATE, STATES, STILL, STOP_REASON, STREAM, STREAM_EVENT,
                    STREAMS, SWITCH_ERROR, TARGET, TCP, TOOL, TOOL_COMMAND_SENT, TRACKING, TRAJECTORY_SENT,
                    TWIST_SENT, WRENCH, WRENCH_PEAK, WRENCH_SENT, relay_event, signal, urdf_joint)

#: The timelines: wall time, and seconds since the range's start.
TIMELINE, SINCE_START = "time", "since start"
#: Axis lengths in the 3D view, metres: a base, an arm's mount, tool0 and target, a tracked object.
BASE_AXES, MOUNT_AXES, TOOL_AXES, OBJECT_AXES = 0.4, 0.15, 0.1, 0.15
#: Where each robot's model goes, plus its short serial: rerun escapes slashes in a URDF's entity prefix, so it is
#: one top-level path, not under /world (its frames still hang off /world/<serial>).
MODEL_ROOT = "model_"
#: The A200's body as a box, drawn with or without its model, around base_footprint, metres (half sizes, centre height).
BODY_HALF, BODY_CENTRE = (0.495, 0.335, 0.19), 0.19
#: A state's text in a state timeline is cut to this many characters: soft stop reasons are long (the text log has
#: them whole).
STATE_TEXT = 60
#: Line colours of a plot entity's channels, in order (Tableau 10).
COLORS = ((31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189), (140, 86, 75),
          (227, 119, 194), (127, 127, 127), (188, 189, 34), (23, 190, 207))
#: Keys an event's text line leaves out: the times, what the entity says, and what is too long for a line.
QUIET = ("event", "time", "ros_time", "robot", "status", "flags", "hit", "was", "file")
#: A text line is cut to this many characters, a field's value in it to this many (the file has them whole).
LINE, FIELD = 160, 30
#: Repeats of a log line (same source, same text but its numbers) closer than this are counted, not shown, seconds.
REPEAT_GAP = 5.0
#: The late ticks state goes back to "ok" this long after the last late-tick report, seconds: three times the gap
#: between two reports (`late_tick_warn_period`, 5 s).
LATE_QUIET = 15.0
LATE_TICKS = "states/lab/late_ticks"
#: A number standing on its own in a log line (not part of a name like B20 or a200-0806).
NUMBER = re.compile(r"(?<![\w/-])\d+(?:\.\d+)?(?![\w-])")
#: Where each robot's arm ghost goes (its model again, arms only, see-through), plus its short serial.
GHOST_ROOT = "ghost_"
#: A ghost arm is hidden by scaling its shoulder to this: rerun has no visibility over time for logged data.
HIDDEN_SCALE = 1e-6
#: How long a clearance trip's line stays, seconds; a sent path's line height above the floor and width, metres.
TRIP_SHOWN, PLAN_HEIGHT, PLAN_RADIUS = 2.0, 0.01, 0.01
#: A recording's format (`names.FORMAT`) -> how its names become the next format's: (a tick signal's name -> its new
#: name, an event -> the event renamed). A bump of FORMAT adds the format before it here; format 1 is the only one yet.
UPGRADES: dict[int, tuple[Callable[[str], str], Callable[[dict], dict]]] = {}


@dataclass
class Session:
    """One session the range touches: where it is, its clock offset, its robots and arms."""

    folder: Path
    #: Wall time minus ROS time, seconds.
    offset: float
    #: Serial -> its arm names.
    robots: dict[str, list[str]]
    #: The chunks that may hold ticks of the range.
    chunks: list[Path] = field(default_factory=list)
    #: Its end, wall time: its last write (a copy's: the range copied).
    ended: float = np.inf
    #: The format of its names (`names.FORMAT`); a recording without one is format 1.
    format: int = 1


@dataclass
class Model:
    """One robot's URDF model in Rerun: its tree (named frames "<serial>/<link>") and its arms' mounts."""

    #: `rerun.urdf.UrdfTree`, logged under `MODEL_ROOT`<serial>.
    tree: Any
    #: Arm -> its base_link in base_footprint.
    mounts: dict[str, Any]
    #: Entity path of the model's collision shapes (hidden in the 3D view).
    collisions: str


@dataclass
class Shown:
    """Something drawn in 3D from `start` to `end` (None: until the next on the same entity replaces it)."""

    entity: str
    start: float
    end: float | None
    #: What to draw: for a path or a trip, line strips (and labels); for a ghost, joint name -> angle.
    what: Any
    labels: list[str] = field(default_factory=list)


@dataclass
class Ghost:
    """One robot's arm ghost: its model again (arms only), and each arm's moving joints, shoulder first."""

    tree: Any
    arms: dict[str, list[Any]]


def log_range(root: Path, start: float, end: float, *, save: Path | None = None, data: Path | None = None,
              spawn: bool = True, copy: bool = False) -> dict:
    """Show (or save) what the observer recorded from wall time `start` to `end` in Rerun.

    Args:
        root: The local recordings, or with `copy` a copy of them (`copied_sessions`).
        start: The range's start, wall time, seconds since the epoch.
        end: Its end.
        save: Write an .rrd file here instead of opening the viewer.
        data: The teleop data folder, for the URDFs (each robot's model, its arms' mounts); without it the bases
            are only boxes and the arms are not placed in 3D.
        spawn: Open the viewer (ignored with `save`); False only logs into the current recording (tests).
        copy: `root` is a copy (`export.copy_range`), not the local recordings.

    Returns:
        dict: The sessions shown, the tick rows and events logged, and what could not be shown ("notes").
    """
    import rerun as rr

    found = _sessions(root, start, end, copy)
    if not found:
        return {"sessions": [], "rows": 0, "events": 0, "notes": [f"no recordings between those times in {root}"]}
    notes = _newer(found)  # * printed by the helper script, shown on the panel's line
    models = _models(rr, found, data, notes)
    sent = [shown for session in found for shown in _sent(session, start, end)]
    ghosts = _ghosts(found, data, sent, notes)
    objects = sorted({name for session in found for chunk in session.chunks for name in _object_names(chunk, session)})
    blueprint = _blueprint(found, objects, models, ghosts)
    # * A new id each time: the same range opened twice in one viewer would merge into one recording, all doubled.
    rr.init("husky_assembly_observer", recording_id=f"observer_{int(start)}_{int(end)}_{uuid.uuid4().hex[:8]}")
    if save is not None:
        rr.save(save, default_blueprint=blueprint)
    elif spawn:
        rr.spawn(memory_limit="50%")
        # * Active, not only default: the viewer keeps the layout it last showed for this app and would show that.
        rr.send_blueprint(blueprint, make_active=True, make_default=True)
    _log_static(rr, found, models)
    _log_scene(rr, found, data, objects, start, end, notes)
    _log_ghosts(rr, ghosts, start)
    _log_sent(rr, sent, ghosts, start, end)
    paths: dict[str, list[np.ndarray]] = {}
    rows = sum(_log_ticks(rr, session, start, end, models, paths) for session in found)
    _log_paths(rr, paths)
    events = sum(_log_events(rr, session, start, end, notes) for session in found)
    recording = rr.get_global_data_recording()
    if recording is not None:
        recording.flush()  # * all of it in the file or the viewer before this returns, not at the process's exit
    return {"sessions": [session.folder.name for session in found], "rows": rows, "events": events, "notes": notes}


# --- --- --- --- --- WHAT TO READ --- --- --- --- ---

def _sessions(root: Path, start: float, end: float, copy: bool = False) -> list[Session]:
    """The sessions whose recording overlaps the range, oldest first, with the chunks that may hold its ticks."""
    found = []
    for kept in copied_sessions(root) if copy else sessions(root):
        if kept["end"] < start or kept["start"] > end:
            continue
        description = load_description(kept["folder"])
        try:
            offset = float(description["wall_time"]) - float(description["ros_time"])
        except (KeyError, TypeError, ValueError):
            continue
        robots = {serial: list((about or {}).get("arms", {})) for serial, about in
                  (description.get("robots") or {}).items()}
        chunks = [chunk for chunk in sorted(kept["folder"].glob(CHUNKS))
                  if chunk.stat().st_mtime >= start - SAVED_MARGIN]
        try:
            recorded = int(description.get("format", 1))
        except (TypeError, ValueError):
            recorded = FORMAT + 1  # * not a number: unknown, read as a newer format
        found.append(Session(kept["folder"], offset, robots, chunks, kept["end"], recorded))
    return found


def _newer(found: list[Session]) -> list[str]:
    """A warning per session of a format newer than this viewer knows: it is shown as well as it reads."""
    return [f"{session.folder.name}: format {session.format} is newer than this viewer's ({FORMAT}): events renamed "
            f"since show as plain lines, signals renamed since may be missing; update the teleop checkout"
            for session in found if session.format > FORMAT]


def _upgraded_key(key: str, session: Session) -> str:
    """A chunk array's key in this viewer's names (`UPGRADES`); its ".labels" or ".units" suffix kept."""
    name, dot, suffix = key.partition(".")
    for old in range(session.format, FORMAT):
        name = UPGRADES[old][0](name)
    return name + dot + suffix


def _upgraded_event(event: dict, session: Session) -> dict:
    """An event in this viewer's names (`UPGRADES`)."""
    for old in range(session.format, FORMAT):
        event = UPGRADES[old][1](event)
    return event


def _models(rr, found: list[Session], data: Path | None, notes: list[str]) -> dict[str, Model]:
    """Serial -> its URDF model and arms' mounts; robots without one are noted (box only, arms left out)."""
    serials = sorted({serial: tuple(arms) for session in found for serial, arms in session.robots.items()}.items())
    if data is None:
        if serials:
            notes.append("no data folder: no robot models, arms not placed in 3D")
        return {}
    import rerun.urdf
    from ...config import robot_config_from_serial
    from ...robot_interface.frames import MOCAP_FRAME, fixed_transform
    models = {}
    for serial, arms in serials:
        short = short_serial(serial)
        try:
            urdf = robot_config_from_serial(serial, data).model.urdf
            mounts = {arm: fixed_transform(urdf, MOCAP_FRAME, f"{arm}_base_link") for arm in arms}
            tree = rerun.urdf.UrdfTree.from_file_path(urdf, entity_path_prefix=f"{MODEL_ROOT}{short}",
                                                      frame_prefix=f"{short}/",
                                                      static_transform_entity_path=f"/{MODEL_ROOT}{short}/tf_static")
            visual = tree.get_visual_geometry_paths(tree.root_link()) or \
                next(paths for link in (j.child_link for j in tree.joints())
                     if (paths := tree.get_visual_geometry_paths(link)))
            models[serial] = Model(tree, mounts, visual[0].split("/visual_geometries")[0] + "/collision_geometries")
        except Exception as error:  # * any config or URDF problem: that robot is only a box, its arms out of 3D
            notes.append(f"{short}: no model in 3D ({error})")
    return models


def _object_names(chunk: Path, session: Session) -> list[str]:
    """The tracked objects a chunk has columns for (only its key list is read)."""
    try:
        with np.load(chunk) as arrays:
            keys = [_upgraded_key(key, session) for key in arrays.files]
    except Exception:
        return []
    prefix, suffix = signal(OBJECT, ""), signal("", OBJECT_POSE)
    return [key.removeprefix(prefix).removesuffix(suffix) for key in keys
            if key.startswith(prefix) and key.endswith(suffix)]


def _entity(key: str, session: Session) -> tuple[str, str | None, str | None]:
    """A tick signal's entity path, and its robot's serial and arm (None for a lab or object signal)."""
    for serial, arms in session.robots.items():
        prefix = f"{short_serial(serial)}_"
        if not key.startswith(prefix):
            continue
        rest = key.removeprefix(prefix)
        for arm in arms:
            if rest.startswith(f"{arm}_"):
                return f"{short_serial(serial)}/{arm}/{rest.removeprefix(f'{arm}_')}", serial, arm
        return f"{short_serial(serial)}/{rest}", serial, None
    if key.startswith(signal(OBJECT, "")):
        name, _, what = key.removeprefix(signal(OBJECT, "")).rpartition("_")
        return f"objects/{name}/{what}", None, None
    return f"lab/{key}", None, None


# --- --- --- --- --- STATIC --- --- --- --- ---

def _log_static(rr, found: list[Session], models: dict[str, Model]) -> None:
    """The lab frame; each base's labelled body box, its robot's model hung off its frame, each arm's mount."""
    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/origin", rr.TransformAxes3D(0.5), static=True)
    for serial in sorted({serial for session in found for serial in session.robots}):
        short = short_serial(serial)
        rr.log(f"world/{short}/body", rr.Boxes3D(half_sizes=[BODY_HALF], centers=[(0.0, 0.0, BODY_CENTRE)],
                                                 labels=[short], fill_mode="majorwireframe"), static=True)
        model = models.get(serial)
        if model is None:
            continue
        model.tree.log_urdf_to_recording()
        # * The URDF's root (world_link, base_footprint at identity) is where the base pose puts the base.
        rr.log(f"{MODEL_ROOT}{short}/attach", rr.Transform3D(child_frame=f"{short}/{model.tree.root_link().name}",
                                                             parent_frame=f"tf#/world/{short}"), static=True)
        for arm, mount in model.mounts.items():
            path = f"world/{short}/{arm}"
            rr.log(path, rr.Transform3D(translation=mount.position, quaternion=mount.orientation), static=True)
            rr.log(path, rr.TransformAxes3D(MOUNT_AXES), static=True)


# --- --- --- --- --- TICKS --- --- --- --- ---

def _log_ticks(rr, session: Session, start: float, end: float, models: dict[str, Model],
               paths: dict[str, list[np.ndarray]]) -> int:
    """Every tick of the range in one session: plot lines, poses, arm joints, the follower's points.

    Its base paths are added to `paths` (short serial -> its pieces), logged once for every session (`_log_paths`).
    """
    named: set[str] = set()
    rows = 0
    for chunk in session.chunks:
        try:
            arrays = {_upgraded_key(key, session): value for key, value in load_arrays(chunk).items()}
            wall = arrays["t"] + session.offset
        except Exception:
            continue  # * damaged by a crash mid-write: `copy` counts those, the viewer only skips them
        inside = (wall >= start) & (wall <= end)
        if not inside.any():
            continue
        stamps = wall[inside]
        rows += int(inside.sum())
        tools: dict[str, dict[str, np.ndarray]] = {}
        plots = []
        for key in arrays:
            if not key.endswith(".labels") or key.removesuffix(".labels") not in arrays:
                continue
            key = key.removesuffix(".labels")
            values = np.asarray(arrays[key][inside], dtype=float).reshape(int(inside.sum()), -1)
            labels = [str(label) for label in arrays[f"{key}.labels"]]
            units = [str(unit) for unit in arrays.get(f"{key}.units", [""] * len(labels))]
            entity, serial, arm = _entity(key, session)
            plots += _plots(entity, arm, labels, units, values)
            if labels[:7] == list(POSE):
                _log_pose(rr, entity, serial, arm, values, stamps, start, paths)
                if arm is not None:
                    tools.setdefault(entity.rpartition("/")[0], {})[entity.rpartition("/")[2]] = values
            elif entity.endswith(f"/{FOLLOWER}") and serial is not None:
                _log_follower(rr, short_serial(serial), labels, values, stamps, start)
            elif entity.endswith(f"/{JOINTS}") and arm is not None and serial in models:
                _log_joints(rr, models[serial].tree, short_serial(serial), arm, labels, values, stamps, start)
        for arm, poses in tools.items():
            if TCP in poses and TARGET in poses:
                plots.append((f"{arm}/target_error", ["distance [mm]", "angle [deg]"],
                              _target_error(poses[TCP], poses[TARGET])))
        for path, names, values in plots:
            if path not in named:
                rr.log(path, rr.SeriesLines(names=names, colors=[COLORS[i % len(COLORS)] for i in range(len(names))]),
                       static=True)
                named.add(path)
            held = _held(values)
            rr.send_columns(path, indexes=_times(rr, stamps[~held], start),
                            columns=rr.Scalars.columns(scalars=values[~held]))
    return rows


def _log_paths(rr, paths: dict[str, list[np.ndarray]]) -> None:
    """Each base's path over every session of the range, static: logged once, a second log would replace it."""
    for short, pieces in paths.items():
        strips = [strip for piece in pieces for strip in _strips(piece)]
        if strips:
            rr.log(f"world/paths/{short}", rr.LineStrips3D(strips, radii=0.005), static=True)


def _times(rr, stamps: np.ndarray, start: float) -> list:
    """The time columns of these wall times: on both timelines."""
    return [rr.TimeColumn(TIMELINE, timestamp=stamps), rr.TimeColumn(SINCE_START, duration=stamps - start)]


def _set_time(rr, wall: float, start: float) -> None:
    """Log what follows at this wall time, on both timelines."""
    rr.set_time(TIMELINE, timestamp=wall)
    rr.set_time(SINCE_START, duration=wall - start)


def _plots(entity: str, arm: str | None, labels: list[str], units: list[str],
           values: np.ndarray) -> list[tuple[str, list[str], np.ndarray]]:
    """A signal's plot entities: (path, its lines' names, its values, one column per line).

    A pose is its position, and its yaw for a base or an object (an arm's tool0 and target get their error instead);
    the follower and the streams are split by what their channels mean; mocap residuals are in mm.
    """
    names = [f"{label} [{unit}]" if unit else label for label, unit in zip(labels, units)]
    what = entity.rpartition("/")[2]
    if labels[:7] == list(POSE):
        plots = [(f"{entity}/position", names[:3], values[:, :3])]
        return plots if arm is not None else plots + [(f"{entity}/yaw", ["yaw [deg]"], _yaw(values[:, 3:7])[:, None])]
    if labels == list(NOISE):
        millimetres = np.array([unit == "m" for unit in units])
        names = [f"{label} [mm]" if metres else name for label, name, metres in zip(labels, names, millimetres)]
        return [(entity, names, np.where(millimetres, values * 1000.0, values))]
    if what == FOLLOWER:
        groups = {group: [labels.index(label) for label in channels if label in labels]
                  for group, channels in FOLLOWER_GROUPS.items()}
        groups["other"] = [column for column, label in enumerate(labels)
                           if not any(label in channels for channels in FOLLOWER_GROUPS.values())]
        return [(f"{entity}/{group}", [names[column] for column in columns], values[:, columns])
                for group, columns in groups.items() if columns]
    if what == STREAMS:  # * "<stream>_<channel>": one entity per channel, a line per stream
        plots = []
        for channel in STREAM:
            columns = [column for column, label in enumerate(labels) if label.endswith(f"_{channel}")]
            if columns:
                plots.append((f"{entity}/{channel}", [names[column].replace(f"_{channel}", "") for column in columns],
                              values[:, columns]))
        return plots
    return [(entity, names, values)]


def _yaw(xyzw: np.ndarray) -> np.ndarray:
    """The yaw of each quaternion (x, y, z, w), degrees; NaN where unknown."""
    x, y, z, w = xyzw.T
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _target_error(tcp: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Tool0's distance (mm) and rotation angle (deg) to the target per tick, both in base_link; NaN where unknown."""
    distance = np.linalg.norm(target[:, :3] - tcp[:, :3], axis=1) * 1000.0
    with np.errstate(invalid="ignore", divide="ignore"):
        a = tcp[:, 3:7] / np.linalg.norm(tcp[:, 3:7], axis=1, keepdims=True)
        b = target[:, 3:7] / np.linalg.norm(target[:, 3:7], axis=1, keepdims=True)
    b = np.where(np.sum(a * b, axis=1, keepdims=True) < 0.0, -b, b)  # * q and -q are the same rotation
    # * Stable near zero, unlike the arccos of the dot product.
    angle = np.degrees(4.0 * np.arctan2(np.linalg.norm(a - b, axis=1), np.linalg.norm(a + b, axis=1)))
    return np.column_stack([distance, angle])


def _held(values: np.ndarray) -> np.ndarray:
    """The rows equal to the row before and the row after (NaN equal to NaN): leaving them out draws the same lines."""
    same = ~_changed(values)
    held = np.zeros(len(values), dtype=bool)
    held[1:-1] = same[1:-1] & same[2:]
    return held


def _changed(values: np.ndarray) -> np.ndarray:
    """The rows that differ from the row before (NaN equal to NaN), and the first: all a latest-at value needs."""
    same = ((values[1:] == values[:-1]) | (np.isnan(values[1:]) & np.isnan(values[:-1]))).all(axis=1)
    return np.concatenate([np.ones(min(len(values), 1), dtype=bool), ~same])


def _log_pose(rr, entity: str, serial: str | None, arm: str | None, values: np.ndarray, stamps: np.ndarray,
              start: float, paths: dict[str, list[np.ndarray]]) -> None:
    """A pose signal as a moving frame in 3D; its axes shrink to nothing at the ticks it is unknown."""
    *_, what = entity.split("/")
    if serial is None and entity.startswith("objects/"):
        path, length = f"world/objects/{entity.split('/')[1]}", OBJECT_AXES
    elif serial is not None and arm is None and what == BASE_POSE:
        path, length = f"world/{short_serial(serial)}", BASE_AXES
        paths.setdefault(short_serial(serial), []).append(values[:, :3])
    elif serial is not None and arm is not None and what in (TCP, TARGET):
        path, length = f"world/{short_serial(serial)}/{arm}/{what}", TOOL_AXES
    else:
        return
    known = np.isfinite(values[:, :7]).all(axis=1) & (np.linalg.norm(values[:, 3:7], axis=1) > 0.5)
    lengths = np.where(known, length, 0.0)
    shown = _changed(lengths[:, None])
    rr.send_columns(path, indexes=_times(rr, stamps[shown], start),
                    columns=rr.TransformAxes3D.columns(axis_length=lengths[shown]))
    if path.startswith("world/objects/"):
        _log_bar_shown(rr, path, known, stamps, start)
    if known.any():
        moved = np.flatnonzero(known)[_changed(values[known, :7])]
        rr.send_columns(path, indexes=_times(rr, stamps[moved], start),
                        columns=rr.Transform3D.columns(translation=values[moved, :3], quaternion=values[moved, 3:7]))


def _log_joints(rr, tree, short: str, arm: str, labels: list[str], values: np.ndarray, stamps: np.ndarray,
                start: float) -> None:
    """An arm's joints as its model's joint transforms; a joint keeps its last known angle while unknown."""
    for column, label in enumerate(labels):
        joint = tree.get_joint_by_name(urdf_joint(arm, label))
        known = np.flatnonzero(np.isfinite(values[:, column]))
        if joint is None or not len(known):
            continue
        known = known[_changed(values[known, column][:, None])]
        rr.send_columns(f"{MODEL_ROOT}{short}/joints/{joint.name}", indexes=_times(rr, stamps[known], start),
                        columns=joint.compute_transform_columns(values[known, column].tolist()))


def _log_follower(rr, short: str, labels: list[str], values: np.ndarray, stamps: np.ndarray, start: float) -> None:
    """The follower's reference and lookahead points, in the lab frame; none at the ticks it is unknown."""
    for name in ("reference", "lookahead"):
        x, y = FOLLOWER_GROUPS[name][:2]
        if x not in labels or y not in labels:
            continue
        xy = values[:, [labels.index(x), labels.index(y)]]
        xy = xy[(moved := _changed(xy))]
        known = np.isfinite(xy).all(axis=1)
        points = np.column_stack([xy[known], np.zeros(int(known.sum()))])
        rr.send_columns(f"world/follower/{short}/{name}", indexes=_times(rr, stamps[moved], start),
                        columns=rr.Points3D.columns(positions=points).partition(known.astype(int)))


def _strips(xyz: np.ndarray) -> Iterator[np.ndarray]:
    """A path as line strips, broken where it is unknown."""
    known = np.isfinite(xyz).all(axis=1)
    edges = np.flatnonzero(np.diff(np.concatenate([[0], known.astype(int), [0]])))
    for low, high in zip(edges[::2], edges[1::2]):
        if high - low >= 2:
            yield xyz[low:high]


# --- --- --- --- --- EVENTS --- --- --- --- ---

def _log_events(rr, session: Session, start: float, end: float, notes: list[str]) -> int:
    """The range's events and its runs' as text lines and state timelines, those before it as where things stood.

    A session with events but no process loaded (its log line) is noted: its runs' steps cannot be shown.
    """
    events = list(_read_events(session))
    if events and not any(loaded_folder(event) for event in events):
        notes.append(f"{session.folder.name}: no process loaded in its log, so no run steps")
    merged = sorted(events + _run_events(events), key=lambda event: float(event["ros_time"]))
    states: dict[str, str] = {}
    shown: dict[str, str] = {}
    started = False
    count = 0
    for wall, changes, line in _entries(merged, session.offset, start, end):
        if wall > end:
            continue
        if wall < start and not started:
            states.update(changes)
            continue
        if not started:  # * where things stood at the range's start, then every change
            _set_time(rr, start, start)
            for path, text in states.items():
                rr.log(path, rr.StateChange(state=text))
                shown[path] = text
            started = True
        _set_time(rr, max(wall, start), start)
        for path, text in changes.items():
            if shown.get(path) != text:
                rr.log(path, rr.StateChange(state=text))
                shown[path] = text
        if line is not None:
            path, level, text = line
            rr.log(path, rr.TextLog(text, level=level))
            count += 1
    if not started and states:
        _set_time(rr, start, start)
        for path, text in states.items():
            rr.log(path, rr.StateChange(state=text))
    return count


def _read_events(session: Session) -> Iterator[dict]:
    """A session's events in this viewer's names, line by line; a line that does not read (a crash) is left out."""
    path = session.folder / EVENTS
    if not path.is_file():
        return
    with open(path, encoding="utf-8") as lines:
        for line in lines:
            try:
                event = json.loads(line)
                float(event["ros_time"])
            except (ValueError, KeyError, TypeError):
                continue
            yield _upgraded_event(event, session)


def _run_events(events: list[dict]) -> list[dict]:
    """The events of the assembly runs of every process the session loaded, while it ran, named "run <event>"."""
    from ...shared.runs.runs import EVENTS_FILE, RUNS, read_events
    if not events:
        return []
    times = [float(event["ros_time"]) for event in events]
    low, high = min(times), max(times)
    processes = sorted({str(folder) for event in events if (folder := loaded_folder(event)) is not None})
    picked = []
    for process in processes:
        for log in sorted(Path(process).glob(f"{RUNS}/*/{EVENTS_FILE}")):
            for event in read_events(log.parent):
                try:
                    inside = low <= float(event["ros_time"]) <= high
                except (KeyError, TypeError, ValueError):
                    continue  # * written outside the monitor (a new run): no ROS time
                if inside:
                    picked.append({**event, "event": f"{RUN}{event.get('event')}", "run": log.parent.name})
    return picked


def _entries(events: Iterable[dict], offset: float, start: float = -np.inf,
             end: float = np.inf) -> Iterator[tuple[float, dict[str, str], tuple | None]]:
    """Each event as (wall time, its state changes, its text line or None), log floods collapsed.

    - Late-tick warnings become the state `states/lab/late_ticks`, "late" from the first and "ok" `LATE_QUIET` after
      the last, with a line at each switch.
    - Any other log line repeating (same source, same text but its numbers) less than `REPEAT_GAP` apart is shown
      once; the repeats are counted in one line at the last of them. Only lines from wall time `start` to `end` count:
      the first repeat in the range shows, and a flood still going at `end` is counted up to its last repeat before.
    """
    late: list[float] = []  # * the late ticks' times since the state went "late"
    repeats: dict[tuple[str, str], dict] = {}
    for event in events:
        wall = float(event["ros_time"]) + offset
        if late and wall - late[-1] > LATE_QUIET:
            yield late[-1] + LATE_QUIET, *_ticks_back(late)
            late = []
        for key in [key for key, seen in repeats.items() if wall - seen["last"] > REPEAT_GAP or wall > end]:
            if (seen := repeats.pop(key))["more"]:
                yield seen["last"], {}, _repeated(seen)
        changes, line = _state_changes(event), _text_line(event)
        if event.get("event") == LOG and LATE_TICK.match(str(event.get("text"))):
            late.append(wall)
            if len(late) == 1:
                yield wall, {LATE_TICKS: "late"}, ("log/ticks", "WARN", line[2])
            continue
        if event.get("event") == LOG and start <= wall <= end:
            key = (str(event.get("source")), NUMBER.sub("#", str(event.get("text"))))
            if (seen := repeats.get(key)) is not None:
                # * The observer writes a line repeating again with its count so far.
                same = seen["event"].get("text") == event.get("text")
                had, has = int(seen["event"].get("count") or 1), int(event.get("count") or 1)
                seen.update(more=seen["more"] + (has - had if same and has > had else has), last=wall, event=event)
                continue
            repeats[key] = {"more": 0, "last": wall, "event": event}
        yield wall, changes, line
    if late:
        yield late[-1] + LATE_QUIET, *_ticks_back(late)
    for seen in repeats.values():
        if seen["more"]:
            yield seen["last"], {}, _repeated(seen)


def _ticks_back(late: list[float]) -> tuple[dict[str, str], tuple[str, str, str]]:
    """The late ticks state back to "ok", and its line."""
    return {LATE_TICKS: "ok"}, ("log/ticks", "INFO", f"[ticks] on time again, after {len(late)} late-tick "
                                                     f"reports in {late[-1] - late[0]:.0f} s")


def _repeated(seen: dict) -> tuple[str, str, str]:
    """The line counting a log line's repeats, with the last one's text."""
    path, level, text = _text_line(seen["event"])
    return path, level, f"{re.sub(r' [(]×[0-9]+[)]$', '', text)} (×{seen['more']} more)"


def _state_changes(event: dict) -> dict[str, str]:
    """State timeline path -> its new text, for what an event says switched (every state, for a snapshot)."""
    name = event.get("event", "")
    who = short_serial(event["robot"]) if event.get("robot") else "lab"
    if name.startswith(RUN):
        return _run_changes(name.removeprefix(RUN), event)
    if name == STATE:
        changes = {}
        for key, value in {**(event.get("status") or {}), **(event.get("flags") or {})}.items():
            changes[_state_path(who, key)] = _state_text(value)
        for part, moving in (event.get("moving") or {}).items():
            changes[_state_path(who, f"{part} motion")] = MOVING if moving else STILL
        return changes
    if name == RELAY_STATE:
        return {_state_path("lab", relay_event(key)): _state_text(value) for key, value in event.items()
                if key not in QUIET and not isinstance(value, (dict, list))}
    if name in (LEASE_TAKEN, LEASE_RELEASED):
        return {_state_path(who, LEASE): _state_text(event.get("holder") if name == LEASE_TAKEN else None)}
    for part in (MOVING, STILL):
        if name.endswith(f" {part}"):
            subject = event.get("arm") or name.removesuffix(f" {part}")
            return {_state_path(who, f"{subject} motion"): part}
    if "value" in event and not isinstance(event["value"], (dict, list)):
        return {_state_path(who, name): _state_text(event["value"])}
    return {}


def _run_changes(name: str, event: dict) -> dict[str, str]:
    """A run event's state changes: the run, its action and step (movement), the guide's part and mode."""
    if name in (RUN_FORGET, RUN_REOPEN, RUN_NEW, RUN_CONTINUE):
        return {}  # * many at once, for actions other than the current one
    changes = {_state_path("run", "run"): str(event.get("run"))}
    if event.get("action"):
        changes[_state_path("run", "action")] = _state_text(f"{event['action']} done" if name == RUN_MARK_DONE
                                                            else event["action"])
    if event.get("movement"):
        changes[_state_path("run", "step")] = _state_text(event["movement"])
    data = event.get("data") or {}
    if name == RUN_GUIDE:
        changes[_state_path("run", "guide mode")] = _state_text(data.get("mode"))
        if data.get("event") in ("start", "done"):
            changes[_state_path("run", "guide")] = _state_text(data.get("part") if data["event"] == "start" else
                                                               "idle")
    return changes


def _state_path(who: str, name: str) -> str:
    """A state timeline's entity path."""
    return f"states/{who}/{name.replace(' ', '_').replace('/', '_')}"


def _state_text(value: Any) -> str:
    """A state's value as the text of its timeline."""
    if value is None:
        return "none"
    text = str(value).lower() if isinstance(value, bool) else str(value)
    return _cut(text, STATE_TEXT)


def _text_line(event: dict) -> tuple[str, str, str] | None:
    """An event's text log entity, level and short text; None for what only the state timelines show."""
    name = event.get("event", "")
    if name == LOG:
        text, *more = str(event.get("text")).strip().splitlines() or [""]
        text = _cut(f"[{event.get('source')}] {text}" + (" …" if more else ""), LINE)
        if event.get("count", 1) > 1:
            text += f" (×{event['count']})"
        return f"log/{event.get('source') or 'unknown'}", str(event.get("level") or "INFO"), text
    if name.startswith(RUN):
        text = _run_text(name.removeprefix(RUN), event)
        return None if text is None else ("events/run", "WARN" if _warns(event) else "INFO", _cut(text, LINE))
    if name in STATES:
        return None  # * every state at a moment: the timelines show it
    who = short_serial(event["robot"]) if event.get("robot") else None
    text = _event_text(name, event)
    return f"events/{who or 'lab'}", "WARN" if _warns(event) else "INFO", _cut(f"{who} {text}" if who else text, LINE)


def _event_text(name: str, event: dict) -> str:
    """An observer event as a short sentence, without its robot."""
    by = f" by {_by(event['by'])}" if event.get("by") else ""
    arm = event.get("arm") or ""
    value = event.get("value")
    if name == LEASE_TAKEN:
        return f"lease → {_by(event.get('holder'))}"
    if name == LEASE_RELEASED:
        return f"lease released by {_by(event.get('holder'))}"
    if name.endswith(f" {MOVING}"):
        return f"{arm or name.removesuffix(f' {MOVING}')} {MOVING}{by}" + \
            ("" if event.get("commanded", True) else " (not commanded)")
    if name.endswith(f" {STILL}"):
        subject = arm or name.removesuffix(f" {STILL}")
        moved = [f"{event['distance']:.2f} m", f"{np.degrees(event['turn']):.0f}°"] if "distance" in event else \
            [f"largest joint {event['largest_joint_change']:.2f} rad"] if "largest_joint_change" in event else []
        return f"{subject} {STILL} after {event.get('seconds', 0.0):.1f} s" + "".join(f", {part}" for part in moved)
    if name in (TRAJECTORY_SENT, HOLD_SENT):
        what = "hold" if name == HOLD_SENT else f"trajectory, {event.get('waypoints')} points"
        return f"{arm} {what}, {event.get('duration', 0.0):.1f} s{by}"
    if name == TOOL_COMMAND_SENT:
        return f"{arm} tool {event.get('command')} {event.get('position', '')}{by}".replace(" " * 2, " ")
    if name == WRENCH_SENT:
        return f"{arm} wrench {np.linalg.norm(event.get('force') or [0]):.1f} N, " \
               f"{np.linalg.norm(event.get('torque') or [0]):.2f} N m{by}"
    if name == FORCE_SENSOR_ZEROED:
        return f"{arm} force sensor zeroed{by}"
    if name in (PATH_SENT, PATH_STOP_SENT):
        return f"base path stop{by}" if name == PATH_STOP_SENT else \
            f"base path, {event.get('waypoints')} points{by}"
    if name == CLEARANCE_TRIP:
        return f"clearance trip: {event.get('problem')}"
    if name == OBJECT_TRACKED:
        return f"{event.get('object')} tracked (mocap id {event.get('mocap_id')})"
    if name in (SESSION_START, SESSION_END):
        return name
    if name == OBJECT_UNTRACKED:
        return f"{event.get('object')} no longer tracked"
    if name == LOG_LINES_LOST:
        return f"up to {event.get('at_most')} log lines lost: pushed out of the console before the observer read them"
    if name.endswith((STREAM_EVENT, TRACKING)) and value in (LOST, BACK):
        return f"{name} back after {event['lost_for']:.2f} s" if "lost_for" in event else f"{name} {value}"
    if "value" in event and not isinstance(value, (dict, list)):
        if name.endswith((SOFT_STOP, STOP_REASON, SWITCH_ERROR, PROBLEM)):
            return f"{name}: {value}" if value else f"{name} cleared"
        return f"{name} → {_state_text(value) or 'none'}"
    return _fields(name, event)


def _run_text(name: str, event: dict) -> str | None:
    """An assembly run event as a short sentence; None for what only the state timelines show (or the log has)."""
    data = event.get("data") or {}
    step = event.get("movement") or event.get("action") or ""
    if name == RUN_EXEC:
        status = str(data.get("status") or "")
        return f"{step} {data.get('result')}" + (f" ({status})" if status.lower() != data.get("result") else "")
    if name == RUN_SERVO:
        return f"{step} servo {data.get('status')}, {data.get('passes')} passes"
    if name == RUN_PLAN:
        if data.get("message"):
            return f"{step} planned: {data['message']}"
        return f"{step} planned: {data.get('waypoints')} points in {data.get('seconds', 0.0):.1f} s"
    if name == RUN_MARK_DONE:
        return f"{event.get('action')} done" + (" (assumed)" if event.get("assumed") else "")
    if name == RUN_CANCEL:
        return f"{step} cancelled: {data.get('reason')}"
    if name == RUN_SOFT_STOP:
        return f"{step} soft stop"
    if name == RUN_PROMPT:
        return f"{step} asks: {str(data.get('text')).partition(' — ')[2] or data.get('text')}"
    if name == RUN_CONFIRM:
        return f"{step} confirmed"
    if name == RUN_CHECK_GOAL:
        return f"{step} goal ok" if data.get("ok") else f"{step} goal refused: {data.get('reason')}"
    if name == RUN_GUIDE:
        if data.get("event") in ("start", "done", "mode"):
            return None
        reopened = f" (reopens {len(data['reopened'])})" if data.get("reopened") else ""
        return f"guide {data.get('event')}: {data.get('part')}{reopened}"
    if name in (RUN_FORGET, RUN_REOPEN, RUN_AS_BUILT, RUN_SESSION):
        return None  # * forget and reopen come by the dozen with a guide redo; the as-built has its log line
    if name in (RUN_NEW, RUN_CONTINUE):
        return f"run {event.get('run')}: {name.replace('_', ' ')}"
    return _fields(name, event)


def _fields(name: str, event: dict) -> str:
    """An event without a sentence of its own: its name and its fields, long values cut."""
    def short(value: Any) -> str:
        return _cut(value if isinstance(value, str) else json.dumps(value), FIELD)
    return f"{name}: " + ", ".join(f"{key}={short(value)}" for key, value in event.items() if key not in QUIET)


def _by(who: Any) -> str:
    """Who moves or holds a lease, short: a movement_exec step by its movement."""
    text = str(who)
    return _cut(re.sub(r"^movement_exec: exec (\S+):.*$", r"movement_exec: \1", text), FIELD * 2)


def _cut(text: str, length: int) -> str:
    """`text`, cut to `length` characters."""
    return text if len(text) <= length else text[:length - 1] + "…"


def _warns(event: dict) -> bool:
    """Whether an event is something going wrong: a stop, a trip, a stream lost, a step failed or cancelled."""
    name, value = event.get("event", ""), event.get("value")
    data = event.get("data") or {}
    if name.startswith(RUN):
        name = name.removeprefix(RUN)
        return name in (RUN_CANCEL, RUN_SOFT_STOP) or \
            name == RUN_EXEC and data.get("result") in ("failed", "refused") or \
            name == RUN_SERVO and data.get("status") in ("failed", "refused") or \
            name == RUN_CHECK_GOAL and not data.get("ok") or name == RUN_PROMPT and bool(data.get("warn")) or \
            name == RUN_PLAN and bool(data.get("warnings"))
    return (name in (ESTOP, SOFT_STOP) and bool(value)) or name in (CLEARANCE_TRIP, LOG_LINES_LOST) or \
        name.endswith((SWITCH_ERROR, PROBLEM)) and bool(value) or \
        name.endswith((STREAM_EVENT, TRACKING)) and value == LOST


# --- --- --- --- --- LAB AND BARS --- --- --- --- ---

def _log_scene(rr, found: list[Session], data: Path | None, objects: list[str], start: float, end: float,
               notes: list[str]) -> None:
    """The lab's obstacles from its file in git under /world/lab, and each tracked bar's shape on its object frame.

    A bar's shape is hidden from `start` until its pose is known (`_log_bar_shown`): without one it would stand at
    the lab's origin.
    """
    if data is not None:
        path = lab_file(data, DEFAULT_LAB)
        try:
            lab = read_lab(path)
        except (OSError, ValueError) as error:
            notes.append(f"no lab obstacles ({error})")
        else:
            for shape in lab.shapes:
                _log_body(rr, f"world/lab/{slug(shape.name)}", shape.pose,
                          shape.geometry(mesh_directory_of(path)).visual, LAB_COLOR)
            for name, geometry, pose in lab.border.walls() if lab.border is not None else ():
                _log_body(rr, f"world/lab/{slug(name)}", pose, geometry.visual, BORDER_COLOR)
    bars = [name for name in objects if name.startswith(BAR_OBJECT)]
    process = _process(found, end, notes) if bars else None
    missing = []
    for name in bars:
        bar_id = BAR_PREFIX + name.removeprefix(BAR_OBJECT)
        bar = process.bodies.get(bar_id) if process is not None else None
        if bar is None:
            missing.append(name)
            continue
        _log_body(rr, f"world/objects/{name}/bar", Pose(), bar.geometry.visual or bar.geometry.collision,
                  rgb01(FITTED, 1.0))
        _set_time(rr, start, start)
        rr.log(f"world/objects/{name}/bar", rr.Transform3D.from_fields(scale=[HIDDEN_SCALE] * 3))
        # * Mounted halves stay where they sit on their bar, so the assembly poses give the offset.
        for half_id in process.halves_of(bar_id):
            half = process.bodies[half_id]
            _log_body(rr, f"world/objects/{name}/bar/{slug(half_id)}",
                      compose(invert(bar.assembly_pose), half.assembly_pose), half.geometry.visual,
                      rgb01(MOUNTED, 1.0))
    if missing and process is not None:
        notes.append(f"not in the process, axes only: {', '.join(missing)}")


def _process(found: list[Session], end: float, notes: list[str]):
    """The process the monitor last loaded by `end` (its "loaded <folder> in" log line), or None (noted)."""
    folder = None
    for session in found:
        for event in _read_events(session):
            if (loaded := loaded_folder(event)) is not None and float(event["ros_time"]) + session.offset <= end:
                folder = loaded
    if folder is None:
        notes.append("no process loaded: tracked bars are axes only")
        return None
    try:
        return read(folder)
    except Exception as error:  # * moved, another machine's path, an old export: the bars are only axes
        notes.append(f"process {folder} not read, tracked bars are axes only ({error})")
        return None


def _log_body(rr, path: str, pose, shapes, color: tuple[float, ...]) -> None:
    """A body's shapes, static, at `pose` in its parent: boxes and cylinders as such, meshes as triangles."""
    rr.log(path, rr.Transform3D(translation=pose.position, quaternion=pose.orientation), static=True)
    for index, shape in enumerate(shapes):
        if isinstance(shape, BoxShape):
            item = rr.Boxes3D(half_sizes=[np.asarray(shape.size) / 2], centers=[shape.origin.position],
                              quaternions=[shape.origin.orientation], colors=[color], fill_mode="solid")
        elif isinstance(shape, CylinderShape):
            item = rr.Cylinders3D(lengths=[shape.height], radii=[shape.radius], centers=[shape.origin.position],
                                  quaternions=[shape.origin.orientation], colors=[color], fill_mode="solid")
        else:
            mesh = Trimesh(shape.vertices, shape.faces, process=False)
            item = rr.Mesh3D(vertex_positions=shape.vertices, triangle_indices=shape.faces,
                             vertex_normals=mesh.vertex_normals, albedo_factor=color)
        rr.log(f"{path}/{index}", item, static=True)


def _log_bar_shown(rr, path: str, known: np.ndarray, stamps: np.ndarray, start: float) -> None:
    """A tracked bar's shape only at the ticks its pose is known: shrunk to nothing at the others."""
    scale = np.where(known, 1.0, HIDDEN_SCALE)
    shown = _changed(scale[:, None])
    rr.send_columns(f"{path}/bar", indexes=_times(rr, stamps[shown], start),
                    columns=rr.Transform3D.columns(scale=np.repeat(scale[shown, None], 3, axis=1)))


# --- --- --- --- --- SENT COMMANDS --- --- --- --- ---

def _sent(session: Session, start: float, end: float) -> list[Shown]:
    """What the session's sent base paths, arm trajectories and clearance trips draw in 3D, within the range.

    A path shows until its follower stops following, a trajectory's ghost until its last point's time or a hold sent
    to its arm, a trip for `TRIP_SHOWN`; any of them until the next on the same entity replaces it, and at most until
    the session's end.
    """
    drawn: list[Shown] = []
    for event in _read_events(session):
        wall = float(event["ros_time"]) + session.offset
        name, serial = event.get("event"), event.get("robot")
        if wall > end or not serial:
            continue
        short = short_serial(serial)
        if (name == FOLLOWER_STATE and event.get("value") != FOLLOWING) or name == HOLD_SENT:
            entity = f"world/plans/{short}" if name == FOLLOWER_STATE else f"{GHOST_ROOT}{short}/{event.get('arm', '')}"
            for shown in reversed(drawn):
                if shown.entity == entity:
                    if shown.start <= wall and (shown.end is None or shown.end > wall):
                        shown.end = wall
                    break
        elif name in (PATH_SENT, TRAJECTORY_SENT) and event.get("file"):
            try:
                sent = json.loads((session.folder / event["file"]).read_text())
            except (OSError, ValueError):
                continue  # * the event's text line still says it was sent
            if name == PATH_SENT:
                path = sent.get("path") or {}
                xyz = np.column_stack([path.get("x", []), path.get("y", []), np.full(len(path.get("x", [])),
                                                                                     PLAN_HEIGHT)])
                drawn.append(Shown(f"world/plans/{short}", wall, None, [xyz], [event.get("by") or "sent path"]))
            elif sent.get("points"):
                last = sent["points"][-1]
                since = last.get("time_from_start") or {}
                arm = event.get("arm", "")
                joints = {f"{arm}_{joint}": float(value) for joint, value in
                          zip(sent.get("joint_names", []), last.get("positions", []))}
                drawn.append(Shown(f"{GHOST_ROOT}{short}/{arm}", wall,
                                   wall + since.get("sec", 0) + since.get("nanosec", 0) * 1e-9, joints))
        elif name == CLEARANCE_TRIP and event.get("hit"):
            strips, labels = [], []
            for pair in event["hit"].get("pairs", []):
                contact = pair.get("contact")
                if contact:
                    strips.append([contact["point_a"], contact["point_b"]])
                    labels.append(" / ".join(str(pair.get(side, "")).rsplit("/", 1)[-1] for side in ("a", "b")))
            if strips:
                drawn.append(Shown(f"world/trips/{short}", wall, wall + TRIP_SHOWN, strips, labels))
    for shown in drawn:  # * a path still followed when the session ended has no end: it would stay through the next
        if shown.end is None or shown.end > session.ended:
            shown.end = max(shown.start, session.ended)
    return _in_range(drawn, start, end)


def _in_range(drawn: list[Shown], start: float, end: float) -> list[Shown]:
    """The drawn things that show in the range, oldest first; one replaced by the next on its entity gets no end."""
    kept = []
    for entity in sorted({shown.entity for shown in drawn}):
        items = sorted((shown for shown in drawn if shown.entity == entity), key=lambda shown: shown.start)
        for shown, after in zip(items, items[1:] + [None]):
            until = shown.end if shown.end is not None else float("inf")
            if after is not None and after.start <= until:
                shown.end, until = None, after.start
            if shown.start <= end and until >= start:
                kept.append(shown)
    return sorted(kept, key=lambda shown: shown.start)


def _ghosts(found: list[Session], data: Path | None, sent: list[Shown], notes: list[str]) -> dict[str, Ghost]:
    """Short serial -> the arm ghost of each robot a trajectory was sent to in the range; noted where none loads."""
    serials = {short_serial(serial): serial for session in found for serial in session.robots}
    arms: dict[str, set[str]] = {}
    for shown in sent:
        root, _, arm = shown.entity.partition("/")
        if root.startswith(GHOST_ROOT) and root.removeprefix(GHOST_ROOT) in serials:
            arms.setdefault(serials[root.removeprefix(GHOST_ROOT)], set()).add(arm)
    if data is None or not arms:
        return {}
    import rerun.urdf
    from ...config import robot_config_from_serial
    ghosts = {}
    for serial, names in sorted(arms.items()):
        short = short_serial(serial)
        try:
            urdf = robot_config_from_serial(serial, data).model.urdf
            tree = rerun.urdf.UrdfTree.from_file_path(urdf, entity_path_prefix=f"{GHOST_ROOT}{short}",
                                                      frame_prefix=f"{GHOST_ROOT}{short}/",
                                                      static_transform_entity_path=f"/{GHOST_ROOT}{short}/tf_static")
        except Exception as error:  # * any config or URDF problem: that robot's trajectories have no ghost
            notes.append(f"{short}: no arm ghost ({error})")
            continue
        joints = {arm: _arm_joints(tree, arm) for arm in sorted(names)}
        for arm in [arm for arm, chain in joints.items() if not chain]:
            notes.append(f"{short}: no arm ghost for {arm} (no {arm}_shoulder_pan_joint in its URDF)")
            del joints[arm]
        if joints:
            ghosts[short] = Ghost(tree, joints)
    return ghosts


def _arm_joints(tree, arm: str) -> list[Any]:
    """The joints from an arm's shoulder down to its tool, shoulder first; empty if the URDF has no such shoulder."""
    shoulder = tree.get_joint_by_name(f"{arm}_shoulder_pan_joint")
    if shoulder is None:
        return []
    chain, links = [shoulder], [shoulder.child_link]
    for link in links:
        for joint in tree.joints():
            if joint.parent_link == link:
                chain.append(joint)
                links.append(joint.child_link)
    return chain


def _log_ghosts(rr, ghosts: dict[str, Ghost], start: float) -> None:
    """Each ghost's arm meshes (only those), see-through, hung off its base, hidden until a trajectory shows it."""
    for short, ghost in ghosts.items():
        tree, root = ghost.tree, f"/{GHOST_ROOT}{short}"
        links = {joint.child_link for chain in ghost.arms.values() for joint in chain}
        every = [tree.root_link().name] + [joint.child_link for joint in tree.joints()]
        shown = [path for link in links for path in tree.get_visual_geometry_paths(link)]
        hidden = [path for link in every if link not in links for path in tree.get_visual_geometry_paths(link)]
        model = (shown or hidden)[0].split("/visual_geometries")[0]
        rr.send_chunks(tree.stream().filter(content=[f"+ {root}/**", f"- {model}/collision_geometries/**",
                                                     *(f"- {path}/**" for path in hidden)]))
        for path in shown:
            rr.log(path, rr.Asset3D.from_fields(albedo_factor=rgb01(PLAN, 0.4)), static=True)
        rr.log(f"{root}/attach", rr.Transform3D(child_frame=f"{GHOST_ROOT}{short}/{tree.root_link().name}",
                                                parent_frame=f"tf#/world/{short}"), static=True)
        _set_time(rr, start, start)
        for chain in ghost.arms.values():
            _log_ghost_arm(rr, root.lstrip("/"), chain, {}, shown=False)


def _log_ghost_arm(rr, root: str, chain: list, joints: dict[str, float] | None, shown: bool) -> None:
    """Show or hide a ghost arm at the current time, its joints set first (those not given at zero; None: kept)."""
    for joint in chain if joints is not None else ():
        rr.log(f"{root}/joints/{joint.name}", joint.compute_transform(joints.get(joint.name, 0.0), clamp=False))
    rr.log(f"{root}/joints/{chain[0].name}", rr.Transform3D.from_fields(scale=[1.0 if shown else HIDDEN_SCALE] * 3))


def _log_sent(rr, sent: list[Shown], ghosts: dict[str, Ghost], start: float, end: float) -> None:
    """Draw each sent path, arm ghost and clearance trip at its time, and take it away at its end."""
    for shown in sent:
        root, _, arm = shown.entity.partition("/")
        ghost = ghosts.get(root.removeprefix(GHOST_ROOT)) if root.startswith(GHOST_ROOT) else None
        chain = ghost.arms.get(arm) if ghost is not None else None
        if root.startswith(GHOST_ROOT) and chain is None:
            continue  # * that robot has no model, noted
        _set_time(rr, max(shown.start, start), start)
        if chain is not None:
            _log_ghost_arm(rr, root, chain, shown.what, shown=True)
        else:
            trip = root == "world" and shown.entity.startswith("world/trips/")
            rr.log(shown.entity, rr.LineStrips3D(shown.what, radii=0.005 if trip else PLAN_RADIUS,
                                                 colors=rgb01(COLLISION if trip else PLAN, 1.0), labels=shown.labels))
        if shown.end is not None and shown.end <= end:
            _set_time(rr, shown.end, start)
            if chain is not None:
                _log_ghost_arm(rr, root, chain, None, shown=False)
            else:
                rr.log(shown.entity, rr.Clear(recursive=False))


# --- --- --- --- --- LAYOUT --- --- --- --- ---

def _blueprint(found: list[Session], objects: list[str], models: dict[str, Model], ghosts: dict[str, Ghost]):
    """The default layout: the 3D lab and the state timelines left, a plot tab per robot and the text log right."""
    import rerun.blueprint as rrb

    robots: dict[str, list[str]] = {}
    for session in found:
        for serial, arms in session.robots.items():
            robots.setdefault(short_serial(serial), [])
            robots[short_serial(serial)] += [arm for arm in arms if arm not in robots[short_serial(serial)]]
    tabs = []
    for short, arms in sorted(robots.items()):
        views = [
            rrb.TimeSeriesView(name=f"{short} follower errors", origin=f"/{short}/{FOLLOWER}/errors"),
            rrb.TimeSeriesView(name=f"{short} twist sent vs odometry", origin=f"/{short}",
                               contents=[f"$origin/{TWIST_SENT}", f"$origin/{ODOMETRY}"]),
            rrb.TimeSeriesView(name=f"{short} mocap", origin=f"/{short}/{BASE_MOCAP}"),
            rrb.TimeSeriesView(name=f"{short} stream rates", origin=f"/{short}/{STREAMS}/{STREAM[0]}"),
            rrb.TimeSeriesView(name=f"{short} battery", origin=f"/{short}/{BATTERY}"),
        ]
        for arm in arms:
            views += [rrb.TimeSeriesView(name=f"{short} {arm} joints", origin=f"/{short}/{arm}/{JOINTS}"),
                      rrb.TimeSeriesView(name=f"{short} {arm} wrench", origin=f"/{short}/{arm}",
                                         contents=[f"$origin/{WRENCH}", f"$origin/{WRENCH_PEAK}"]),
                      rrb.TimeSeriesView(name=f"{short} {arm} tool0 vs target", origin=f"/{short}/{arm}/target_error"),
                      rrb.TimeSeriesView(name=f"{short} {arm} tool", origin=f"/{short}/{arm}/{TOOL}")]
        tabs.append(rrb.Grid(*views, name=short))
    lab = [rrb.TimeSeriesView(name="relay drift", origin=f"/lab/{RELAY_DRIFT}"),
           rrb.TimeSeriesView(name="late ticks", origin=f"/lab/{LATE_SHARE}")]
    if objects:
        # * Rerun takes a `*` only as a trailing `/**`: every object listed.
        lab.append(rrb.TimeSeriesView(name="objects mocap", origin="/objects",
                                      contents=[f"+ $origin/{name}/{OBJECT_MOCAP}" for name in objects]))
    tabs.append(rrb.Grid(*lab, name="lab"))
    # * The models sit at the top level (`MODEL_ROOT`), their collision shapes hidden.
    lab_frame = ["+ /world/**"]
    for serial, model in models.items():
        lab_frame += [f"+ /{MODEL_ROOT}{short_serial(serial)}/**", f"- /{model.collisions.lstrip('/')}/**"]
    lab_frame += [f"+ /{GHOST_ROOT}{short}/**" for short in ghosts]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(rrb.Spatial3DView(name="lab frame", origin="/", contents=lab_frame),
                         rrb.StateTimelineView(name="states", origin="/states"), row_shares=[3, 2]),
            rrb.Vertical(rrb.Tabs(*tabs),
                         rrb.TextLogView(name="events and log", origin="/", contents=["/events/**", "/log/**"]),
                         row_shares=[3, 2]),
            column_shares=[2, 3]),
        rrb.TimePanel(timeline=TIMELINE), collapse_panels=False)
