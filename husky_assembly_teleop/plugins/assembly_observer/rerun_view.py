"""
The observer's local recordings in Rerun: one time range of them as one Rerun recording.

Opened by `scripts/observer_logs.py rerun`. Every tick signal becomes one plot entity with a line per channel
(`/<serial>[/<arm>]/<signal>`; a pose as `.../position` and `.../yaw`, the follower and streams split by meaning), each
arm's tool0-vs-target error a plot of its own, every switching state a state timeline (`/states/...`), every event and
log line a text line (`/events/...`, `/log/<source>`), and the poses frames in a 3D view of the lab frame
(`/world/...`): each base, its arms' mounts from the URDF, tool0, the Cartesian target while the compliance controller
runs, the follower's reference and lookahead, every tracked object. Two timelines: "time", wall time (ROS time plus
the session's offset, so it reads as the local clock and lines up with `events`), and "since start", seconds from the
range's start. A value held over several ticks is logged at its first and last tick only: the lines look the same.

! Imports rerun only inside `log_range`: the monitor never needs it. Plain files, no ROS; reads whole chunks, so run
  it in a process of its own.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator

import numpy as np

from ...config import short_serial
from ...plugin_api.trace import load_arrays
from ...shared.recordings.runs import load_description
from .columns import NOISE, POSE, STREAM
from .export import CHUNKS, EVENTS, SAVED_MARGIN, STATES, copied_sessions, sessions

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
#: The follower's channels, plotted as these entities under its signal (the rest under "other").
FOLLOWER = {"errors": ("cross_track", "along_track", "yaw_error"), "progress": ("progress",), "command": ("v", "w"),
            "pose": ("x", "y", "yaw"), "reference": ("reference_x", "reference_y", "reference_yaw"),
            "lookahead": ("lookahead_x", "lookahead_y")}
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
LATE_TICK = re.compile(r"tick \d+ started [\d.]+ ms late")
#: A number standing on its own in a log line (not part of a name like B20 or a200-0806).
NUMBER = re.compile(r"(?<![\w/-])\d+(?:\.\d+)?(?![\w-])")
#: The process plugin's log line naming the process folder it loaded (its runs are under it).
LOADED = re.compile(r"loaded (/.+?) in \d")
#: An assembly run's event as an event here: "run <its event>".
RUN = "run "


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


@dataclass
class Model:
    """One robot's URDF model in Rerun: its tree (named frames "<serial>/<link>") and its arms' mounts."""

    #: `rerun.urdf.UrdfTree`, logged under `MODEL_ROOT`<serial>.
    tree: Any
    #: Arm -> its base_link in base_footprint.
    mounts: dict[str, Any]
    #: Entity path of the model's collision shapes (hidden in the 3D view).
    collisions: str


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
    notes: list[str] = []
    models = _models(rr, found, data, notes)
    objects = sorted({name for session in found for chunk in session.chunks for name in _object_names(chunk)})
    blueprint = _blueprint(found, objects, models)
    rr.init("husky_assembly_observer", recording_id=f"observer_{int(start)}_{int(end)}")
    if save is not None:
        rr.save(save, default_blueprint=blueprint)
    elif spawn:
        rr.spawn(memory_limit="50%")
        # * Active, not only default: the viewer keeps the layout it last showed for this app and would show that.
        rr.send_blueprint(blueprint, make_active=True, make_default=True)
    _log_static(rr, found, models)
    rows = sum(_log_ticks(rr, session, start, end, models) for session in found)
    events = sum(_log_events(rr, session, start, end) for session in found)
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
        found.append(Session(kept["folder"], offset, robots, chunks))
    return found


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


def _object_names(chunk: Path) -> list[str]:
    """The tracked objects a chunk has columns for (only its key list is read)."""
    try:
        with np.load(chunk) as arrays:
            keys = arrays.files
    except Exception:
        return []
    return [key.removeprefix("object_").removesuffix("_pose") for key in keys
            if key.startswith("object_") and key.endswith("_pose")]


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
    if key.startswith("object_"):
        name, _, what = key.removeprefix("object_").rpartition("_")
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

def _log_ticks(rr, session: Session, start: float, end: float, models: dict[str, Model]) -> int:
    """Every tick of the range in one session: plot lines, poses, arm joints, the follower's points, base paths."""
    named: set[str] = set()
    paths: dict[str, list[np.ndarray]] = {}
    rows = 0
    for chunk in session.chunks:
        try:
            arrays = load_arrays(chunk)
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
            elif entity.endswith("/follower") and serial is not None:
                _log_follower(rr, short_serial(serial), labels, values, stamps, start)
            elif entity.endswith("/joints") and arm is not None and serial in models:
                _log_joints(rr, models[serial].tree, short_serial(serial), arm, labels, values, stamps, start)
        for arm, poses in tools.items():
            if "tcp" in poses and "target" in poses:
                plots.append((f"{arm}/target_error", ["distance [mm]", "angle [deg]"],
                              _target_error(poses["tcp"], poses["target"])))
        for path, names, values in plots:
            if path not in named:
                rr.log(path, rr.SeriesLines(names=names, colors=[COLORS[i % len(COLORS)] for i in range(len(names))]),
                       static=True)
                named.add(path)
            held = _held(values)
            rr.send_columns(path, indexes=_times(rr, stamps[~held], start),
                            columns=rr.Scalars.columns(scalars=values[~held]))
    for short, pieces in paths.items():
        strips = [strip for piece in pieces for strip in _strips(piece)]
        if strips:
            rr.log(f"world/paths/{short}", rr.LineStrips3D(strips, radii=0.005), static=True)
    return rows


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
    if what == "follower":
        groups = {group: [labels.index(label) for label in channels if label in labels]
                  for group, channels in FOLLOWER.items()}
        groups["other"] = [column for column, label in enumerate(labels)
                           if not any(label in channels for channels in FOLLOWER.values())]
        return [(f"{entity}/{group}", [names[column] for column in columns], values[:, columns])
                for group, columns in groups.items() if columns]
    if what == "streams":  # * "<stream>_<channel>": one entity per channel, a line per stream
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
    elif serial is not None and arm is None and what == "base_pose":
        path, length = f"world/{short_serial(serial)}", BASE_AXES
        paths.setdefault(short_serial(serial), []).append(values[:, :3])
    elif serial is not None and arm is not None and what in ("tcp", "target"):
        path, length = f"world/{short_serial(serial)}/{arm}/{what}", TOOL_AXES
    else:
        return
    known = np.isfinite(values[:, :7]).all(axis=1) & (np.linalg.norm(values[:, 3:7], axis=1) > 0.5)
    lengths = np.where(known, length, 0.0)
    shown = _changed(lengths[:, None])
    rr.send_columns(path, indexes=_times(rr, stamps[shown], start),
                    columns=rr.TransformAxes3D.columns(axis_length=lengths[shown]))
    if known.any():
        moved = np.flatnonzero(known)[_changed(values[known, :7])]
        rr.send_columns(path, indexes=_times(rr, stamps[moved], start),
                        columns=rr.Transform3D.columns(translation=values[moved, :3], quaternion=values[moved, 3:7]))


def _log_joints(rr, tree, short: str, arm: str, labels: list[str], values: np.ndarray, stamps: np.ndarray,
                start: float) -> None:
    """An arm's joints as its model's joint transforms; a joint keeps its last known angle while unknown."""
    for column, label in enumerate(labels):
        joint = tree.get_joint_by_name(f"{arm}_{label}_joint")
        known = np.flatnonzero(np.isfinite(values[:, column]))
        if joint is None or not len(known):
            continue
        known = known[_changed(values[known, column][:, None])]
        rr.send_columns(f"{MODEL_ROOT}{short}/joints/{joint.name}", indexes=_times(rr, stamps[known], start),
                        columns=joint.compute_transform_columns(values[known, column].tolist()))


def _log_follower(rr, short: str, labels: list[str], values: np.ndarray, stamps: np.ndarray, start: float) -> None:
    """The follower's reference and lookahead points, in the lab frame; none at the ticks it is unknown."""
    for name, (x, y) in (("reference", ("reference_x", "reference_y")), ("lookahead", ("lookahead_x", "lookahead_y"))):
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

def _log_events(rr, session: Session, start: float, end: float) -> int:
    """The range's events and its runs' as text lines and state timelines, those before it as where things stood."""
    events = list(_read_events(session.folder / EVENTS))
    merged = sorted(events + _run_events(events), key=lambda event: float(event["ros_time"]))
    states: dict[str, str] = {}
    shown: dict[str, str] = {}
    started = False
    count = 0
    for wall, changes, line in _entries(merged, session.offset):
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


def _read_events(path: Path) -> Iterator[dict]:
    """A session's events, line by line; a line that does not read (cut by a crash) is left out."""
    if not path.is_file():
        return
    with open(path, encoding="utf-8") as lines:
        for line in lines:
            try:
                event = json.loads(line)
                float(event["ros_time"])
            except (ValueError, KeyError, TypeError):
                continue
            yield event


def _run_events(events: list[dict]) -> list[dict]:
    """The events of the assembly runs of every process the session loaded, while it ran, named "run <event>"."""
    from ...shared.runs.runs import EVENTS_FILE, RUNS, read_events
    if not events:
        return []
    times = [float(event["ros_time"]) for event in events]
    low, high = min(times), max(times)
    processes = sorted({found.group(1) for event in events if event.get("event") == "log"
                        and event.get("source") == "process" and (found := LOADED.match(str(event.get("text"))))})
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


def _entries(events: Iterable[dict], offset: float) -> Iterator[tuple[float, dict[str, str], tuple | None]]:
    """Each event as (wall time, its state changes, its text line or None), log floods collapsed.

    - Late-tick warnings become the state `states/lab/late_ticks`, "late" from the first and "ok" `LATE_QUIET` after
      the last, with a line at each switch.
    - Any other log line repeating (same source, same text but its numbers) less than `REPEAT_GAP` apart is shown
      once; the repeats are counted in one line at the last of them.
    """
    late: list[float] = []  # * the late ticks' times since the state went "late"
    repeats: dict[tuple[str, str], dict] = {}
    for event in events:
        wall = float(event["ros_time"]) + offset
        if late and wall - late[-1] > LATE_QUIET:
            yield late[-1] + LATE_QUIET, *_ticks_back(late)
            late = []
        for key in [key for key, seen in repeats.items() if wall - seen["last"] > REPEAT_GAP]:
            if (seen := repeats.pop(key))["more"]:
                yield seen["last"], {}, _repeated(seen)
        changes, line = _state_changes(event), _text_line(event)
        if event.get("event") == "log" and LATE_TICK.match(str(event.get("text"))):
            late.append(wall)
            if len(late) == 1:
                yield wall, {LATE_TICKS: "late"}, ("log/ticks", "WARN", line[2])
            continue
        if event.get("event") == "log":
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
    if name == "state":
        changes = {}
        for key, value in {**(event.get("status") or {}), **(event.get("flags") or {})}.items():
            changes[_state_path(who, key)] = _state_text(value)
        for part, moving in (event.get("moving") or {}).items():
            changes[_state_path(who, f"{part} motion")] = "moving" if moving else "still"
        return changes
    if name == "relay state":
        return {_state_path("lab", f"relay {key}"): _state_text(value) for key, value in event.items()
                if key not in QUIET and not isinstance(value, (dict, list))}
    if name in ("lease taken", "lease released"):
        return {_state_path(who, "lease"): _state_text(event.get("holder") if name == "lease taken" else None)}
    for part in ("moving", "still"):
        if name.endswith(f" {part}"):
            subject = event.get("arm") or name.removesuffix(f" {part}")
            return {_state_path(who, f"{subject} motion"): part}
    if "value" in event and not isinstance(event["value"], (dict, list)):
        return {_state_path(who, name): _state_text(event["value"])}
    return {}


def _run_changes(name: str, event: dict) -> dict[str, str]:
    """A run event's state changes: the run, its action and step (movement), the guide's part and mode."""
    if name in ("forget", "reopen", "new_run", "continue"):
        return {}  # * many at once, for actions other than the current one
    changes = {_state_path("run", "run"): str(event.get("run"))}
    if event.get("action"):
        changes[_state_path("run", "action")] = _state_text(f"{event['action']} done" if name == "mark_done"
                                                            else event["action"])
    if event.get("movement"):
        changes[_state_path("run", "step")] = _state_text(event["movement"])
    data = event.get("data") or {}
    if name == "guide":
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
    if name == "log":
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
    if name == "lease taken":
        return f"lease → {_by(event.get('holder'))}"
    if name == "lease released":
        return f"lease released by {_by(event.get('holder'))}"
    if name.endswith(" moving"):
        return f"{arm or name.removesuffix(' moving')} moving{by}" + \
            ("" if event.get("commanded", True) else " (not commanded)")
    if name.endswith(" still"):
        subject = arm or name.removesuffix(" still")
        moved = [f"{event['distance']:.2f} m", f"{np.degrees(event['turn']):.0f}°"] if "distance" in event else \
            [f"largest joint {event['largest_joint_change']:.2f} rad"] if "largest_joint_change" in event else []
        return f"{subject} still after {event.get('seconds', 0.0):.1f} s" + "".join(f", {part}" for part in moved)
    if name in ("trajectory sent", "hold sent"):
        what = "hold" if name == "hold sent" else f"trajectory, {event.get('waypoints')} points"
        return f"{arm} {what}, {event.get('duration', 0.0):.1f} s{by}"
    if name == "tool command sent":
        return f"{arm} tool {event.get('command')} {event.get('position', '')}{by}".replace(" " * 2, " ")
    if name == "wrench sent":
        return f"{arm} wrench {np.linalg.norm(event.get('force') or [0]):.1f} N, " \
               f"{np.linalg.norm(event.get('torque') or [0]):.2f} N m{by}"
    if name == "force sensor zeroed":
        return f"{arm} force sensor zeroed{by}"
    if name in ("path sent", "path stop sent"):
        return f"base path stop{by}" if name == "path stop sent" else \
            f"base path, {event.get('waypoints')} points{by}"
    if name == "clearance trip":
        return f"clearance trip: {event.get('problem')}"
    if name == "object tracked from now on":
        return f"{event.get('object')} tracked (mocap id {event.get('mocap_id')})"
    if name in ("session start", "session end"):
        return name
    if name == "object no longer tracked":
        return f"{event.get('object')} no longer tracked"
    if name.endswith(("stream", "tracking")) and value in ("lost", "back"):
        return f"{name} back after {event['lost_for']:.2f} s" if "lost_for" in event else f"{name} {value}"
    if "value" in event and not isinstance(value, (dict, list)):
        if name.endswith(("soft stop", "stop reason", "switch error", "problem")):
            return f"{name}: {value}" if value else f"{name} cleared"
        return f"{name} → {_state_text(value) or 'none'}"
    return _fields(name, event)


def _run_text(name: str, event: dict) -> str | None:
    """An assembly run event as a short sentence; None for what only the state timelines show (or the log has)."""
    data = event.get("data") or {}
    step = event.get("movement") or event.get("action") or ""
    if name == "exec":
        status = str(data.get("status") or "")
        return f"{step} {data.get('result')}" + (f" ({status})" if status.lower() != data.get("result") else "")
    if name == "servo":
        return f"{step} servo {data.get('status')}, {data.get('passes')} passes"
    if name == "plan":
        if data.get("message"):
            return f"{step} planned: {data['message']}"
        return f"{step} planned: {data.get('waypoints')} points in {data.get('seconds', 0.0):.1f} s"
    if name == "mark_done":
        return f"{event.get('action')} done" + (" (assumed)" if event.get("assumed") else "")
    if name == "cancel":
        return f"{step} cancelled: {data.get('reason')}"
    if name == "soft_stop":
        return f"{step} soft stop"
    if name == "prompt":
        return f"{step} asks: {str(data.get('text')).partition(' — ')[2] or data.get('text')}"
    if name == "confirm":
        return f"{step} confirmed"
    if name == "check_goal":
        return f"{step} goal ok" if data.get("ok") else f"{step} goal refused: {data.get('reason')}"
    if name == "guide":
        if data.get("event") in ("start", "done", "mode"):
            return None
        reopened = f" (reopens {len(data['reopened'])})" if data.get("reopened") else ""
        return f"guide {data.get('event')}: {data.get('part')}{reopened}"
    if name in ("forget", "reopen", "as_built", "session"):
        return None  # * forget and reopen come by the dozen with a guide redo; the as-built has its log line
    if name in ("new_run", "continue"):
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
        return name in ("cancel", "soft_stop") or name == "exec" and data.get("result") in ("failed", "refused") or \
            name == "servo" and data.get("status") in ("failed", "refused") or \
            name == "check_goal" and not data.get("ok") or name == "prompt" and bool(data.get("warn")) or \
            name == "plan" and bool(data.get("warnings"))
    return (name in ("e-stop", "soft stop") and bool(value)) or name == "clearance trip" or \
        name.endswith(("switch error", "problem")) and bool(value) or \
        name.endswith(("stream", "tracking")) and value == "lost"


# --- --- --- --- --- LAYOUT --- --- --- --- ---

def _blueprint(found: list[Session], objects: list[str], models: dict[str, Model]):
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
            rrb.TimeSeriesView(name=f"{short} follower errors", origin=f"/{short}/follower/errors"),
            rrb.TimeSeriesView(name=f"{short} twist sent vs odometry", origin=f"/{short}",
                               contents=["$origin/twist_sent", "$origin/odometry"]),
            rrb.TimeSeriesView(name=f"{short} mocap", origin=f"/{short}/base_mocap"),
            rrb.TimeSeriesView(name=f"{short} stream rates", origin=f"/{short}/streams/rate"),
            rrb.TimeSeriesView(name=f"{short} battery", origin=f"/{short}/battery"),
        ]
        for arm in arms:
            views += [rrb.TimeSeriesView(name=f"{short} {arm} joints", origin=f"/{short}/{arm}/joints"),
                      rrb.TimeSeriesView(name=f"{short} {arm} wrench", origin=f"/{short}/{arm}",
                                         contents=["$origin/wrench", "$origin/wrench_peak"]),
                      rrb.TimeSeriesView(name=f"{short} {arm} tool0 vs target", origin=f"/{short}/{arm}/target_error"),
                      rrb.TimeSeriesView(name=f"{short} {arm} tool", origin=f"/{short}/{arm}/tool")]
        tabs.append(rrb.Grid(*views, name=short))
    lab = [rrb.TimeSeriesView(name="relay drift", origin="/lab/relay_drift"),
           rrb.TimeSeriesView(name="late ticks", origin="/lab/monitor_late_share")]
    if objects:
        lab.append(rrb.TimeSeriesView(name="objects mocap", origin="/objects", contents=["$origin/*/mocap"]))
    tabs.append(rrb.Grid(*lab, name="lab"))
    # * The models sit at the top level (`MODEL_ROOT`), their collision shapes hidden.
    lab_frame = ["+ /world/**"]
    for serial, model in models.items():
        lab_frame += [f"+ /{MODEL_ROOT}{short_serial(serial)}/**", f"- /{model.collisions.lstrip('/')}/**"]
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(rrb.Spatial3DView(name="lab frame", origin="/", contents=lab_frame),
                         rrb.StateTimelineView(name="states", origin="/states"), row_shares=[3, 2]),
            rrb.Vertical(rrb.Tabs(*tabs),
                         rrb.TextLogView(name="events and log", origin="/", contents=["/events/**", "/log/**"]),
                         row_shares=[3, 2]),
            column_shares=[2, 3]),
        rrb.TimePanel(timeline=TIMELINE), collapse_panels=False)
