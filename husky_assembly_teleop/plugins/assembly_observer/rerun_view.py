"""
The observer's local recordings in Rerun: one time range of them as one Rerun recording.

Opened by `scripts/observer_logs.py rerun`. Every tick column becomes a plot line (`/<serial>/<signal>/<label>`),
every switching state a state timeline (`/states/...`), every event and log line a text line (`/events/...`,
`/log/<source>`), and the poses frames in a 3D view of the lab frame (`/world/...`): each base, its arms' mounts from
the URDF, tool0, the Cartesian target while the compliance controller runs, the follower's reference and lookahead,
every tracked object. One timeline, "time": wall time (ROS time plus the session's offset), so it reads as the local
clock and lines up with `events`.

! Imports rerun only inside `log_range`: the monitor never needs it. Plain files, no ROS; reads whole chunks, so run
  it in a process of its own.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from ...config import short_serial
from ...plugin_api.trace import load_arrays
from ...shared.recordings.runs import load_description
from .export import CHUNKS, EVENTS, SAVED_MARGIN, STATES, copied_sessions, sessions

#: The one timeline.
TIMELINE = "time"
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
#: Keys an event's text line leaves out: the times, what the entity says, and what is too long for a line.
QUIET = ("event", "time", "ros_time", "robot", "status", "flags", "hit")


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
        times = [rr.TimeColumn(TIMELINE, timestamp=stamps)]
        rows += int(inside.sum())
        for key in arrays:
            if not key.endswith(".labels") or key.removesuffix(".labels") not in arrays:
                continue
            key = key.removesuffix(".labels")
            values = np.asarray(arrays[key][inside], dtype=float).reshape(int(inside.sum()), -1)
            labels = [str(label) for label in arrays[f"{key}.labels"]]
            units = [str(unit) for unit in arrays.get(f"{key}.units", [""] * len(labels))]
            entity, serial, arm = _entity(key, session)
            for column, (label, unit) in enumerate(zip(labels, units)):
                path = f"{entity}/{label}"
                if path not in named:
                    rr.log(path, rr.SeriesLines(names=f"{label} [{unit}]" if unit else label), static=True)
                    named.add(path)
                rr.send_columns(path, indexes=times, columns=rr.Scalars.columns(scalars=values[:, column]))
            if labels[:7] == ["x", "y", "z", "qx", "qy", "qz", "qw"]:
                _log_pose(rr, entity, serial, arm, values, stamps, paths)
            elif entity.endswith("/follower") and serial is not None:
                _log_follower(rr, short_serial(serial), labels, values, times)
            elif entity.endswith("/joints") and arm is not None and serial in models:
                _log_joints(rr, models[serial].tree, short_serial(serial), arm, labels, values, stamps)
    for short, pieces in paths.items():
        strips = [strip for piece in pieces for strip in _strips(piece)]
        if strips:
            rr.log(f"world/paths/{short}", rr.LineStrips3D(strips, radii=0.005), static=True)
    return rows


def _log_pose(rr, entity: str, serial: str | None, arm: str | None, values: np.ndarray, stamps: np.ndarray,
              paths: dict[str, list[np.ndarray]]) -> None:
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
    rr.send_columns(path, indexes=[rr.TimeColumn(TIMELINE, timestamp=stamps)],
                    columns=rr.TransformAxes3D.columns(axis_length=np.where(known, length, 0.0)))
    if known.any():
        rr.send_columns(path, indexes=[rr.TimeColumn(TIMELINE, timestamp=stamps[known])],
                        columns=rr.Transform3D.columns(translation=values[known, :3], quaternion=values[known, 3:7]))


def _log_joints(rr, tree, short: str, arm: str, labels: list[str], values: np.ndarray, stamps: np.ndarray) -> None:
    """An arm's joints as its model's joint transforms; a joint keeps its last known angle while unknown."""
    for column, label in enumerate(labels):
        joint = tree.get_joint_by_name(f"{arm}_{label}_joint")
        known = np.isfinite(values[:, column])
        if joint is None or not known.any():
            continue
        rr.send_columns(f"{MODEL_ROOT}{short}/joints/{joint.name}",
                        indexes=[rr.TimeColumn(TIMELINE, timestamp=stamps[known])],
                        columns=joint.compute_transform_columns(values[known, column].tolist()))


def _log_follower(rr, short: str, labels: list[str], values: np.ndarray, times: list) -> None:
    """The follower's reference and lookahead points, in the lab frame; none at the ticks it is unknown."""
    for name, (x, y) in (("reference", ("reference_x", "reference_y")), ("lookahead", ("lookahead_x", "lookahead_y"))):
        if x not in labels or y not in labels:
            continue
        xy = values[:, [labels.index(x), labels.index(y)]]
        known = np.isfinite(xy).all(axis=1)
        points = np.column_stack([xy[known], np.zeros(int(known.sum()))])
        rr.send_columns(f"world/follower/{short}/{name}", indexes=times,
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
    """The range's events as text lines and its states as state timelines, those before it as where things stood."""
    states: dict[str, str] = {}
    shown: dict[str, str] = {}
    started = False
    count = 0
    for event in _read_events(session.folder / EVENTS):
        wall = float(event["ros_time"]) + session.offset
        if wall > end:
            continue
        changes = _state_changes(event)
        if wall < start:
            states.update(changes)
            continue
        if not started:  # * where things stood at the range's start, then every change
            rr.set_time(TIMELINE, timestamp=start)
            for path, text in states.items():
                rr.log(path, rr.StateChange(state=text))
                shown[path] = text
            started = True
        rr.set_time(TIMELINE, timestamp=wall)
        for path, text in changes.items():
            if shown.get(path) != text:
                rr.log(path, rr.StateChange(state=text))
                shown[path] = text
        if event.get("event") in STATES:
            continue  # * every state at a moment: the timelines show it
        path, level, text = _text_line(event)
        rr.log(path, rr.TextLog(text, level=level))
        count += 1
    if not started and states:
        rr.set_time(TIMELINE, timestamp=start)
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


def _state_changes(event: dict) -> dict[str, str]:
    """State timeline path -> its new text, for what an event says switched (every state, for a snapshot)."""
    name = event.get("event", "")
    who = short_serial(event["robot"]) if event.get("robot") else "lab"
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
            return {_state_path(who, f"{name.removesuffix(f' {part}')} motion"): part}
    if "value" in event and not isinstance(event["value"], (dict, list)):
        return {_state_path(who, name): _state_text(event["value"])}
    return {}


def _state_path(who: str, name: str) -> str:
    """A state timeline's entity path."""
    return f"states/{who}/{name.replace(' ', '_').replace('/', '_')}"


def _state_text(value: Any) -> str:
    """A state's value as the text of its timeline."""
    if value is None:
        return "none"
    text = str(value).lower() if isinstance(value, bool) else str(value)
    return text if len(text) <= STATE_TEXT else text[:STATE_TEXT - 1] + "…"


def _text_line(event: dict) -> tuple[str, str, str]:
    """An event's text log entity, level and text."""
    name = event.get("event", "")
    if name == "log":
        text = f"[{event.get('source')}] {event.get('text')}"
        if event.get("count", 1) > 1:
            text += f" (x{event['count']})"
        return f"log/{event.get('source') or 'unknown'}", str(event.get("level") or "INFO"), text
    who = short_serial(event["robot"]) if event.get("robot") else "lab"
    rest = " ".join(f"{key}={json.dumps(value)}" for key, value in event.items() if key not in QUIET)
    return f"events/{who}", "WARN" if _warns(event) else "INFO", f"{who} {name}  {rest}".rstrip()


def _warns(event: dict) -> bool:
    """Whether an event is something going wrong: a stop, a trip, a switch error, a stream or tracking lost."""
    name, value = event.get("event", ""), event.get("value")
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
            rrb.TimeSeriesView(name=f"{short} follower errors", origin=f"/{short}/follower",
                               contents=["$origin/cross_track", "$origin/along_track", "$origin/yaw_error"]),
            rrb.TimeSeriesView(name=f"{short} twist sent vs odometry", origin=f"/{short}",
                               contents=["$origin/twist_sent/**", "$origin/odometry/**"]),
            rrb.TimeSeriesView(name=f"{short} mocap", origin=f"/{short}/base_mocap"),
            rrb.TimeSeriesView(name=f"{short} stream rates", origin=f"/{short}/streams",
                               contents=["$origin/*_rate"]),
            rrb.TimeSeriesView(name=f"{short} battery", origin=f"/{short}/battery"),
        ]
        for arm in arms:
            views += [rrb.TimeSeriesView(name=f"{short} {arm} joints", origin=f"/{short}/{arm}/joints"),
                      rrb.TimeSeriesView(name=f"{short} {arm} wrench", origin=f"/{short}/{arm}",
                                         contents=["$origin/wrench/**", "$origin/wrench_peak/**"]),
                      rrb.TimeSeriesView(name=f"{short} {arm} tool", origin=f"/{short}/{arm}/tool")]
        tabs.append(rrb.Grid(*views, name=short))
    lab = [rrb.TimeSeriesView(name="relay drift", origin="/lab/relay_drift"),
           rrb.TimeSeriesView(name="late ticks", origin="/lab/monitor_late_share")]
    if objects:
        lab.append(rrb.TimeSeriesView(name="objects mocap", origin="/objects", contents=["$origin/*/mocap/**"]))
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
        collapse_panels=False)
