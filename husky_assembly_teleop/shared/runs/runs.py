"""
The `runs/` record of an assembly: which actions are done, where each robot was last seen, the run's settings.

A run is a folder `runs/<YYYYMMDD_HHMMSS>/` in the process folder (`_2`, `_3` on a clash). The core's `write` leaves
`runs/` alone, so a run survives a re-export into the same folder; the hashes then differ and it no longer matches.

  run.json        schema, kind "run", name, created_at, process (relative to the Drive root if inside it),
                  process_hash, action_hashes {action id: sha}, robots {live id: {serial, urdf_sha256,
                  relay_calibration}}, commits {monitor, core, tamp}, settings, continued_from, changed_since
  progress.json   schema, kind "progress", updated_at, current {action, movement} or null,
                  actions {action id: {status, seq, marked_at, assumed, ends {live id: robot}}},
                  robots {live id: robot}: where each robot was left by the last action marked done with it
  events.jsonl    one {time, ros_time, event, action, movement, ...} per line
  movements/<action>/<movement>.<n>.json and servo/<action>/<movement>.<n>.json: {schema, kind, ...payload}

A robot is {base: [x, y, z, qx, qy, qz, qw], joints: {name: rad}, source: "live" | "assumed", at, after_action}.
Hashes are the core's `content_hash`, the values a solution's `solved_against` holds; `urdf_sha256` is of the robot's
URDF as kept in the data folder (the same on every PC). Times are local ISO times.

- Pure functions return a new `Run`; `new_run`, `save_*` and `record_*` write. Files are replaced atomically, so a
  reader never sees half a file; a stray `.tmp` is ignored.
- The run used last is the one whose progress changed last (`updated_at`, to the microsecond), not the newest folder.
- * A file with another `schema` is refused (`RunError`).
"""

from __future__ import annotations

import json
import subprocess
from dataclasses import asdict, dataclass, field, fields, replace
from datetime import datetime
from hashlib import sha256
from importlib.util import find_spec
from os import fsync
from os import replace as replace_file
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Mapping

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import Process, content_hash, writer_info

from ..assembly_planning.steps import first_step

if TYPE_CHECKING:
    from ...config import RobotConfig

#: The format of every file a run holds.
SCHEMA = "husky_run/1"
#: A process folder's manifest (the core's process format, §2).
PROCESS_FILE = "process.json"
#: The folder of runs in a process folder, and the files of a run.
RUNS, RUN_FILE, PROGRESS_FILE, EVENTS_FILE = "runs", "run.json", "progress.json", "events.jsonl"
#: Action statuses, and where a robot record came from.
PENDING, DONE = "pending", "done"
LIVE, ASSUMED = "live", "assumed"
#: A robot counts as live while its joints and its base were measured this recently, seconds.
LIVE_JOINTS_AGE = 0.5
LIVE_BASE_AGE = 1.0
#: Folder of recorded files -> the `kind` written into them.
KINDS = {"movements": "movement", "servo": "servo"}

_PACKAGE = Path(__file__).resolve().parents[2]


class RunError(ValueError):
    """A run file that can't be read (another schema, a missing key), or a change the run refuses."""


#: The settings' ranges: the insert split, millimetres; the spiral's duration to full radius, seconds, and its whole
#: turns (M:250-251).
SPLIT_RANGE = (0.0, 30.0)
SPIRAL_SECONDS_RANGE = (1.0, 60.0)
SPIRAL_TURNS_RANGE = (1, 10)
#: The free searches' ranges (`route_search.FreeSearch`): seconds and iterations per try, the joint step of the
#: checks (rad), tries.
PLAN_TIME_RANGE = (5.0, 600.0)
PLAN_ITERATIONS_RANGE = (1, 1000)
PLAN_RESOLUTION_RANGE = (0.01, 0.5)
PLAN_TRIES_RANGE = (1, 10)


@dataclass(frozen=True)
class RunSettings:
    """How the run executes and plans; kept in run.json, where an older run without a setting reads its default.

    The transfer workaround is not one: it is the process plugin's.

    Attributes:
        insert_split_mm: The last part of an insert run compliant, millimetres (0 to 30); a retreat's compliant part
            is as long, from its start.
        spiral: The insert's compliant part searches in a spiral.
        servo_by_bar: Servo a grounded bar by its mocap fit instead of tool0.
        rigid_only: The whole insert runs under joint control, its screw tightening, never compliant; the split and
            the spiral are ignored.
        spiral_seconds: How long the spiral takes to reach its full radius, seconds (SPIRAL_SECONDS_RANGE).
        spiral_turns: How many turns it makes on the way (SPIRAL_TURNS_RANGE).
        plan_max_time: Seconds per try of tamp's free search (PLAN_TIME_RANGE); `FreeSearch`'s defaults, the five.
        plan_max_iterations: Iterations per try of it (PLAN_ITERATIONS_RANGE).
        plan_joint_resolution: The joint step of its collision checks, radians (PLAN_RESOLUTION_RANGE).
        plan_tries: Tries of a free or one-arm search before giving up (PLAN_TRIES_RANGE).
        plan_swept_check: Check a found path between its waypoints before taking it.
    """

    insert_split_mm: float = 5.0
    spiral: bool = False
    servo_by_bar: bool = False
    rigid_only: bool = False
    spiral_seconds: float = 20.0
    spiral_turns: int = 4
    plan_max_time: float = 120.0
    plan_max_iterations: int = 50
    plan_joint_resolution: float = 0.1
    plan_tries: int = 3
    plan_swept_check: bool = True


@dataclass(frozen=True)
class RobotRecord:
    """Where one robot was at one moment.

    Attributes:
        base: World pose of its base.
        joints: Its movable joints by name, radians.
        source: LIVE (measured) or ASSUMED (the export's end, confirmed by the operator).
        at: When, local ISO time.
        after_action: The action whose end it is, if any.
    """

    base: Pose
    joints: Mapping[str, float]
    source: str
    at: str
    after_action: str | None = None


@dataclass(frozen=True)
class ActionRecord:
    """One action's status.

    Attributes:
        status: PENDING or DONE.
        seq: Order of marking, from 1; 0 while pending.
        marked_at: When it was marked done.
        assumed: Its robot's end is the export's, not measured.
        ends: Live id -> where each robot live (or assumed) at the mark was.
    """

    status: str = PENDING
    seq: int = 0
    marked_at: str | None = None
    assumed: bool = False
    ends: Mapping[str, RobotRecord] = field(default_factory=dict)


@dataclass(frozen=True)
class Progress:
    """How far a run got: progress.json.

    Attributes:
        actions: Action id -> its record, one per scheduled action.
        current: (action id, movement index) to carry on from, or None.
        robots: Live id -> where it was last seen (the beliefs).
        updated_at: When this changed last.
    """

    actions: Mapping[str, ActionRecord]
    current: tuple[str, int] | None
    robots: Mapping[str, RobotRecord]
    updated_at: str


@dataclass(frozen=True)
class RunInfo:
    """What a run ran: run.json. The fields are the file's keys (module docstring)."""

    name: str
    created_at: str
    process: str
    process_hash: str
    action_hashes: Mapping[str, str]
    robots: Mapping[str, Mapping[str, str]]
    commits: Mapping[str, str]
    settings: RunSettings
    continued_from: str | None = None
    changed_since: tuple[str, ...] = ()


@dataclass(frozen=True)
class Run:
    """One run: its folder, run.json and progress.json."""

    folder: Path
    info: RunInfo
    progress: Progress


# --- --- --- --- --- FINDING AND READING --- --- --- --- ---

def file_hashes(process: Process) -> tuple[str, dict[str, str]]:
    """The content hashes of a process's files: process.json, and each scheduled action's file.

    Raises:
        TypeError: If the process was never written (`folder` is None).
    """
    folder = Path(process.folder)
    return content_hash(folder / PROCESS_FILE), {action_id: content_hash(folder / "actions" / f"{action_id}.json")
                                                 for action_id in process.schedule}


def find_runs(process_folder: Path) -> tuple[list[Run], list[tuple[str, str]]]:
    """Every run in a process folder, the one used last first, and (folder name, why) per folder that can't be read.

    Used last: its progress changed last (`updated_at`, else `created_at`); on a tie, the newer folder.
    """
    runs, problems = [], []
    folder = Path(process_folder) / RUNS
    for path in sorted(folder.iterdir()) if folder.is_dir() else ():
        if path.is_dir():
            try:
                runs.append(read_run(path))
            except (RunError, OSError, ValueError, KeyError, TypeError) as error:
                problems.append((path.name, str(error)))
    return sorted(runs, key=_last_used, reverse=True), problems


def made_after(name: str, run: Run) -> bool:
    """Whether a run folder, by its name, was made after a run's folder (its time, then its clash number)."""
    return _newness(name) > _newness(run.folder.name)


def read_run(folder: Path) -> Run:
    """Read a run; without progress.json every action is pending.

    Raises:
        RunError: If a file has another schema or kind.
        OSError, KeyError, TypeError, ValueError: If run.json is missing or malformed.
    """
    folder = Path(folder)
    data = _read(folder / RUN_FILE, "run")
    # * Only the settings there are now: an older run.json may hold more (e.g. "transfer_as_free").
    known = {each.name for each in fields(RunSettings)}
    settings = RunSettings(**{name: value for name, value in data["settings"].items() if name in known})
    info = RunInfo(data["name"], data["created_at"], data["process"], data["process_hash"],
                   dict(data["action_hashes"]), dict(data["robots"]), dict(data["commits"]),
                   settings, data.get("continued_from"), tuple(data.get("changed_since", ())))
    path = folder / PROGRESS_FILE
    if not path.is_file():
        pending = {action_id: ActionRecord() for action_id in info.action_hashes}
        return Run(folder, info, Progress(pending, None, {}, ""))
    data = _read(path, "progress")
    actions = {action_id: ActionRecord(entry["status"], entry["seq"], entry["marked_at"], entry["assumed"],
                                       {live: _record_of(end) for live, end in entry["ends"].items()})
               for action_id, entry in data["actions"].items()}
    current = data["current"]
    progress = Progress(actions, (current["action"], current["movement"]) if current else None,
                        {live: _record_of(robot) for live, robot in data["robots"].items()}, data["updated_at"])
    return Run(folder, info, progress)


def continue_run(runs: list[Run], hashes: tuple[str, Mapping[str, str]]) -> Run | None:
    """The newest run made on exactly these files (`file_hashes`), or None."""
    process_hash, action_hashes = hashes
    return next((run for run in runs if run.info.process_hash == process_hash
                 and dict(run.info.action_hashes) == dict(action_hashes)), None)


# --- --- --- --- --- CHANGING A RUN (pure) --- --- --- --- ---

def mark_done(run: Run, action_id: str, ends: Mapping[str, RobotRecord], assumed: bool, *,
              now: datetime | None = None) -> Run:
    """The run with an action done: next in order, its ends kept, and each robot in them believed there.

    Args:
        run: The run.
        action_id: A pending action.
        ends: Live id -> where it is at the end: every live robot, and the acting one if assumed.
        assumed: The acting robot's end is the export's, not measured.
        now: The time to record; now if None.

    Raises:
        RunError: If the action is not in the run, or already done.
    """
    progress, at = run.progress, stamp(now)
    record = progress.actions.get(action_id)
    if record is None or record.status == DONE:
        raise RunError(f"{action_id} is not a pending action of run {run.info.name}")
    ends = {live: replace(end, after_action=action_id) for live, end in ends.items()}
    seq = max((other.seq for other in progress.actions.values()), default=0) + 1
    actions = {**progress.actions, action_id: ActionRecord(DONE, seq, at, assumed, ends)}
    return replace(run, progress=replace(progress, actions=actions, robots={**progress.robots, **ends},
                                         updated_at=stamp(now, precise=True)))


def with_current(run: Run, current: tuple[str, int] | None, *, now: datetime | None = None) -> Run:
    """The run carrying on from another action and movement.

    Args:
        run: The run.
        current: (action id, movement index), or None.
        now: The time to record; now if None.

    Returns:
        Run: The run with `progress.current` replaced.
    """
    return replace(run, progress=replace(run.progress, current=current, updated_at=stamp(now, precise=True)))


def reopen(run: Run, action_id: str, *, now: datetime | None = None) -> Run:
    """The run with a done action pending again; a robot believed at its end goes back to its latest earlier end.

    A robot with no earlier end (in an action marked before) loses its belief.

    Args:
        run: The run.
        action_id: A done action.
        now: The time to record; now if None.

    Returns:
        Run: The run with the action pending.

    Raises:
        RunError: If the action is not done.
    """
    progress = run.progress
    if progress.actions.get(action_id, ActionRecord()).status != DONE:
        raise RunError(f"{action_id} is not done: nothing to reopen")
    actions = {**progress.actions, action_id: ActionRecord()}
    robots = dict(progress.robots)
    for live, belief in progress.robots.items():
        if belief.after_action != action_id:
            continue
        earlier = [record for record in actions.values() if record.status == DONE and live in record.ends]
        if earlier:
            robots[live] = max(earlier, key=lambda record: record.seq).ends[live]
        else:
            del robots[live]
    return replace(run, progress=replace(progress, actions=actions, robots=robots, updated_at=stamp(now, precise=True)))


# --- --- --- --- --- WRITING --- --- --- --- ---

def new_run(process: Process, robots: tuple[RobotConfig, ...], settings: RunSettings, *,
            continued_from: Run | None = None, now: datetime | None = None, root: Path | None = None,
            transfer_as_free: bool) -> Run:
    """Start a run in the process folder: its folder, run.json, progress.json and a first event.

    Continuing an earlier run carries its statuses, ends, beliefs and current action by action id; actions no longer
    scheduled are dropped (named in the first event), and `changed_since` lists what differs from it. A current
    movement the action no longer has becomes the action's first step (`current_moved` in the first event).

    Args:
        process: The process, as read from its folder.
        robots: The configured robots, recorded with their URDF's hash.
        settings: The run's settings.
        continued_from: A run on an earlier export of this process, or None for a fresh start.
        now: The run's time, which names it; now if None.
        root: The Drive root, to record the process folder relative to it.
        transfer_as_free: The transfer workaround (the process plugin's), for a carried action's first step.

    Returns:
        Run: The new run, as written.
    """
    # ! Everything that can fail on what was read comes before the folder is made: no empty run folder is left.
    now = now or datetime.now().astimezone()
    at = stamp(now)
    process_hash, action_hashes = file_hashes(process)
    actions = {action_id: ActionRecord() for action_id in process.schedule}
    current, beliefs, changed, event = None, {}, (), {}
    if continued_from is not None:
        old, old_info = continued_from.progress, continued_from.info
        actions.update((action_id, record) for action_id, record in old.actions.items() if action_id in actions)
        current = old.current if old.current is not None and old.current[0] in actions else None
        movement = current[1] if current is not None else None
        # ? A hand-edited progress.json may hold "1" or true: not a movement index.
        if current is not None and (isinstance(movement, bool) or not isinstance(movement, int)
                                    or not 0 <= movement < len(process.actions[current[0]].movements)):
            action = process.actions[current[0]]
            current = (action.id, first_step(process, action, transfer_as_free))
            event["current_moved"] = [list(old.current), list(current)]
        beliefs = dict(old.robots)
        changed = (*((PROCESS_FILE,) if process_hash != old_info.process_hash else ()),
                   *(action_id for action_id in process.schedule
                     if action_hashes[action_id] != old_info.action_hashes.get(action_id)))
        event.update({"from": old_info.name, "changed": list(changed),
                      "dropped": [action_id for action_id in old.actions if action_id not in actions]})
    # ? The source URDF, not the model's copy: that one holds this PC's absolute mesh paths.
    entries = {robot_id(config.serial): {"serial": config.serial,
                                         "urdf_sha256": _file_sha(config.source_urdf or config.model.urdf),
                                         "relay_calibration": ""} for config in robots}
    process_path, made_by = _relative(Path(process.folder), root), commits()
    folder = _new_folder(Path(process.folder) / RUNS, now.strftime("%Y%m%d_%H%M%S"))
    info = RunInfo(folder.name, at, process_path, process_hash, action_hashes, entries, made_by, settings,
                   continued_from.info.name if continued_from is not None else None, changed)
    run = Run(folder, info, Progress(actions, current, beliefs, stamp(now, precise=True)))
    save_info(run)
    save_progress(run)
    record_event(folder, "continue" if continued_from is not None else "new_run", None, **event)
    return run


def save_info(run: Run) -> None:
    """Write run.json, atomically.

    Args:
        run: The run; its `info` is written into its folder.
    """
    info = run.info
    _write(run.folder / RUN_FILE, {
        "schema": SCHEMA, "kind": "run", "name": info.name, "created_at": info.created_at, "process": info.process,
        "process_hash": info.process_hash, "action_hashes": dict(info.action_hashes),
        "robots": {live: dict(entry) for live, entry in info.robots.items()}, "commits": dict(info.commits),
        "settings": asdict(info.settings), "continued_from": info.continued_from,
        "changed_since": list(info.changed_since)})


def save_progress(run: Run) -> None:
    """Write progress.json, atomically."""
    progress = run.progress
    current = progress.current
    _write(run.folder / PROGRESS_FILE, {
        "schema": SCHEMA, "kind": "progress", "updated_at": progress.updated_at,
        "current": {"action": current[0], "movement": current[1]} if current is not None else None,
        "actions": {action_id: {"status": record.status, "seq": record.seq, "marked_at": record.marked_at,
                                "assumed": record.assumed,
                                "ends": {live: _record_json(end) for live, end in record.ends.items()}}
                    for action_id, record in progress.actions.items()},
        "robots": {live: _record_json(robot) for live, robot in progress.robots.items()}})


def record_event(folder: Path, event: str, ros_time: float | None, **fields: Any) -> None:
    """Append one line to events.jsonl, flushed to disk.

    Args:
        folder: The run's folder.
        event: What happened, e.g. "mark_done".
        ros_time: The monitor's clock, or None outside it.
        fields: JSON values; `action` and `movement` are written on every line, None if not given.
    """
    line = {"time": stamp(None), "ros_time": ros_time, "event": event, "action": fields.pop("action", None),
            "movement": fields.pop("movement", None), **fields}
    path = Path(folder) / EVENTS_FILE
    # * A line cut by a crash is ended first, so this one starts on a line of its own.
    cut = path.is_file() and path.stat().st_size > 0 and _last_byte(path) != b"\n"
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(("\n" if cut else "") + json.dumps(line, allow_nan=False) + "\n")
        handle.flush()
        fsync(handle.fileno())


def read_events(folder: Path, log: Callable[[str], None] | None = None) -> list[dict]:
    """Every event of a run, in order; lines that are not JSON (cut by a crash while writing) are left out.

    Args:
        folder: The run's folder.
        log: Told once how many lines were left out, if any.

    Returns:
        list[dict]: The events.
    """
    path = Path(folder) / EVENTS_FILE
    events, skipped = [], 0
    for line in path.read_text(encoding="utf-8").splitlines() if path.is_file() else ():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            skipped += 1
    if skipped and log is not None:
        log(f"{path}: skipped {skipped} unreadable line{'s' if skipped > 1 else ''}")
    return events


def record_movement(folder: Path, action: str, movement: str, payload: Mapping[str, Any],
                    kind: str = "movements") -> Path:
    """Write one record of a movement as `<kind>/<action>/<movement>.<n>.json`, n one past the highest there.

    Args:
        folder: The run's folder.
        action: The action id.
        movement: The movement id.
        payload: JSON values.
        kind: A key of KINDS: "movements" or "servo".

    Returns:
        Path: The file written.
    """
    where = Path(folder) / kind / action
    where.mkdir(parents=True, exist_ok=True)
    taken = [int(number) for number in (path.name[len(movement) + 1:-len(".json")]
                                        for path in where.glob(f"{movement}.*.json")) if number.isdigit()]
    path = where / f"{movement}.{max(taken, default=0) + 1}.json"
    _write(path, {"schema": SCHEMA, "kind": KINDS[kind], **payload})
    return path


def commits() -> dict[str, str]:
    """The commit (12 digits, "+dirty" with uncommitted changes, or "unknown") of this package, the core and tamp."""
    core = writer_info()
    tamp = find_spec("husky_assembly_tamp")
    return {"monitor": _commit(_PACKAGE), "core": core.commit + ("+dirty" if core.dirty else ""),
            "tamp": _commit(Path(tamp.origin).parent) if tamp is not None and tamp.origin else "unknown"}


# --- --- --- --- --- HELPERS --- --- --- --- ---

def stamp(now: datetime | None = None, precise: bool = False) -> str:
    """A local ISO time to the second (`precise`: to the microsecond), as runs record times; now if None."""
    return (now or datetime.now().astimezone()).isoformat(timespec="microseconds" if precise else "seconds")


def _last_used(run: Run) -> tuple[float, tuple[str, int]]:
    """Sort key of a run: when its progress last changed (else when it was made), then its folder's newness."""
    for at in (run.progress.updated_at, run.info.created_at):
        try:
            return datetime.fromisoformat(at).timestamp(), _newness(run.folder.name)
        except (TypeError, ValueError):
            continue
    return 0.0, _newness(run.folder.name)


def _newness(name: str) -> tuple[str, int]:
    """Sort key of a run folder name: its time, then its clash number (`_2` after the plain name)."""
    stamp, _, count = name.rpartition("_") if name.count("_") >= 2 else (name, "", "1")
    return stamp, int(count) if count.isdigit() else 0


def _new_folder(runs: Path, stamp: str) -> Path:
    """Make a new folder `runs/<stamp>`, or `<stamp>_2`, `_3`, ... if taken; `runs/` is made if missing."""
    runs.mkdir(exist_ok=True)
    for count in range(1, 1000):
        folder = runs / (stamp if count == 1 else f"{stamp}_{count}")
        try:
            folder.mkdir()
            return folder
        except FileExistsError:
            continue
    raise RunError(f"{runs}: too many runs named {stamp}")


def _relative(folder: Path, root: Path | None) -> str:
    """A folder relative to the Drive root when inside it, else absolute."""
    folder = folder.resolve()
    if root is not None and folder.is_relative_to(Path(root).resolve()):
        return str(folder.relative_to(Path(root).resolve()))
    return str(folder)


def _file_sha(path: Path) -> str:
    """The SHA-256 of a file's bytes."""
    return sha256(Path(path).read_bytes()).hexdigest()


def _last_byte(path: Path) -> bytes:
    """The last byte of a file that is not empty."""
    with open(path, "rb") as handle:
        handle.seek(-1, 2)
        return handle.read(1)


def _commit(folder: Path) -> str:
    """The commit of the git checkout holding `folder`, "+dirty" if files under it changed; "unknown" without git."""
    def git(*args: str) -> subprocess.CompletedProcess:
        return subprocess.run(["git", *args], cwd=folder, capture_output=True, text=True, timeout=5,
                              stdin=subprocess.DEVNULL)
    try:
        head, status = git("rev-parse", "--short=12", "HEAD"), git("status", "--porcelain", "--", ".")
    except (OSError, subprocess.SubprocessError):
        return "unknown"
    if head.returncode != 0:
        return "unknown"
    return head.stdout.strip() + ("+dirty" if status.stdout.strip() else "")


def _read(path: Path, kind: str) -> dict:
    """A run file's JSON, after checking its schema and kind.

    Raises:
        RunError: If its schema or kind differ.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema") != SCHEMA or data.get("kind") != kind:
        raise RunError(f"{path}: schema {data.get('schema')!r} kind {data.get('kind')!r}, not {SCHEMA} {kind}")
    return data


def _write(path: Path, data: Mapping[str, Any]) -> None:
    """Replace a JSON file atomically: write `<name>.tmp`, flush it to disk, then rename it over the file."""
    temporary = path.with_name(path.name + ".tmp")
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(data, indent=2, allow_nan=False) + "\n")
        handle.flush()
        fsync(handle.fileno())
    replace_file(temporary, path)


def _record_json(record: RobotRecord) -> dict:
    """A robot record as JSON values."""
    return {"base": [*record.base.position, *record.base.orientation], "joints": dict(record.joints),
            "source": record.source, "at": record.at, "after_action": record.after_action}


def _record_of(data: Mapping[str, Any]) -> RobotRecord:
    """A robot record from its JSON values."""
    base = data["base"]
    return RobotRecord(Pose.from_arrays(base[:3], base[3:]), dict(data["joints"]), data["source"], data["at"],
                       data.get("after_action"))
