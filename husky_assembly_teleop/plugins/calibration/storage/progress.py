"""
A calibration run's progress, saved after every change, so a crash or restart loses at most the take in progress.

`<session>/run_<digits>_<arm>_<HHMMSS>.json` sits next to the take files and lists them; take files are written once
and never changed (`session_files.write_take`). The operator picks a run of the arm to resume from a list; it is rebuilt
from its file and its take files.

- ! Writes are atomic (a temporary file, then a rename): a crash mid-write leaves the previous version.
- A run copies its sweeps, so editing the sweeps file later cannot renumber its takes.

Plain files; any thread.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from ....drive import REAL, environment
from ..analysis.checks import CHECKLIST_BOXES, TOOL_BOX
from ..robot.sweeps import DEFAULT_TOOL_BODY, ArmSweeps, Sweep

#: A run's file name: `run_<digits>_<arm>_<HHMMSS>.json`.
RUN_PATTERN = "run_{digits}_{arm}_*.json"


@dataclass
class Progress:
    """Everything a run found so far; the take files hold the samples.

    Attributes:
        file: The run file, in its session folder.
        serial: The robot, e.g. "a200-0805".
        arm: The arm, e.g. "ur_arm".
        tool_body: The Motive rigid body on the flange.
        sweeps: The takes' sweeps, as when the run started.
        drive_seconds: Seconds the base was driven in Marker error.
        marker_max: (base, tool) largest marker error while driving, metres, or None.
        marker_time: When the marker errors were last measured (Unix time), or None.
        marker_made: What they were measured with (`Made.as_dict`), or None.
        ticks: When each box of Setup's checklist was ticked (Unix time), by label; unticked boxes are missing.
        tool_ticked_for: The tool model Tool on flange was ticked for ("none" for a bare arm), or None.
        collisions: Per sweep index: "" clear, else why not; missing if not checked.
        collision_base: The base's pose when the collisions were checked, [[x, y, z], [qx, qy, qz, qw]], or None.
        takes: Per sweep index: its take file, relative to the session folder.
        validations: The validation files, one per base spot, relative to the session folder.
        made: What each take and validation file was measured with, by file (`Made.as_dict`: tool body, base and tool
            body definitions); a file missing here was measured with the run's tool body, definitions not known.
        output_sha: Hash of what Solve wrote (the relay's calibration file, or an arm mount check), or None.
        applied: Whether the run is finished: the relay applied its calibration, or its mount check was written.
        acceptances: Per step name, the stamp of the inputs its accepted result was calculated from (`Run.inputs`).
        overrides: Older run files' record of going on past red checks: read and kept, used for nothing.
        punch_file: The punch validation file, relative to the session folder, or None.
        punch_offset: The punch's TCP offset, tool0 to tip, metres: committed from Tip offset, or entered.
        punch_mount: How often the punch was remounted during the run; punch_target: how often the target moved.
        punch_bases: The base body's definition at each punch take (`body_dict`), or None if not known.
        punch_mounts: The punch mount count at each punch take; punch_targets: the target count.
        tip_joints: The arm's joints at each pose of the tip offset, UR driver order.
        tip_mounts: The punch mount count at each of them.
        on_stand: Whether the base stood on its stand (wheels off the floor) rather than on its wheels; a record only.
        notes: The operator's notes on the run, for people to read; nothing uses them.
        environment: "sim" for a run through a simulation's relay, else "real" (`drive.environment`); ros_domain: the
            monitor's.
    """

    file: Path
    serial: str
    arm: str
    tool_body: int
    sweeps: tuple[Sweep, ...]
    drive_seconds: float = 0.0
    marker_max: list[float] | None = None
    marker_time: float | None = None
    marker_made: dict | None = None
    ticks: dict[str, float] = field(default_factory=dict)
    tool_ticked_for: str | None = None
    collisions: dict[int, str] = field(default_factory=dict)
    collision_base: list | None = None
    takes: dict[int, str] = field(default_factory=dict)
    validations: list[str] = field(default_factory=list)
    made: dict[str, dict] = field(default_factory=dict)
    output_sha: str | None = None
    applied: bool = False
    acceptances: dict[str, str] = field(default_factory=dict)
    overrides: list[dict] = field(default_factory=list)
    punch_file: str | None = None
    punch_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    punch_mount: int = 0
    punch_target: int = 0
    punch_bases: list[dict | None] = field(default_factory=list)
    punch_mounts: list[int] = field(default_factory=list)
    punch_targets: list[int] = field(default_factory=list)
    tip_joints: list[list[float]] = field(default_factory=list)
    tip_mounts: list[int] = field(default_factory=list)
    on_stand: bool = False
    notes: str = ""
    environment: str = REAL
    ros_domain: int = 0

    @property
    def session(self) -> Path:
        """Path: The session folder the run's files are in."""
        return self.file.parent

    def save(self) -> None:
        """Write the run file atomically."""
        data = {key: value for key, value in asdict(self).items() if key != "file"}
        data["sweeps"] = [asdict(sweep) for sweep in self.sweeps]
        temporary = self.file.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(data, indent=2))
        os.replace(temporary, self.file)

    @classmethod
    def load(cls, file: Path) -> Progress:
        """Read a run file.

        Raises:
            ValueError: If it is not a run file.
        """
        try:
            data = json.loads(file.read_text())
            ticks, tool_ticked_for = _ticks(data)
            return cls(file=file, serial=data["serial"], arm=data["arm"], tool_body=int(data["tool_body"]),
                       sweeps=tuple(Sweep(int(s["joint"]), tuple(s["joints"]), float(s["start"]), float(s["end"]),
                                          int(s["steps"])) for s in data["sweeps"]),
                       # * Older run files' refine counts ("refines", "punch_refines") and furthest step reached
                       # * ("step") are not used: their measurements count as made with definitions not known.
                       drive_seconds=float(data["drive_seconds"]),
                       # * Older run files kept every driven tick's errors.
                       marker_max=data["marker_max"] if "marker_max" in data else
                       (np.max(data["drive_errors"], axis=0).tolist() if data.get("drive_errors") else None),
                       collisions={int(k): v for k, v in data["collisions"].items()},
                       collision_base=data["collision_base"],
                       takes={int(k): v for k, v in data["takes"].items()},
                       # * Older run files kept one validation file.
                       validations=list(data["validations"]) if "validations" in data else
                       [data["validation"]] if data.get("validation") else [],
                       made={name: _made(made) for name, made in data.get("made", {}).items()},
                       marker_time=data.get("marker_time"), marker_made=data.get("marker_made"), ticks=ticks,
                       tool_ticked_for=tool_ticked_for,
                       output_sha=data["output_sha"], applied=bool(data["applied"]),
                       acceptances={str(step): str(stamp) for step, stamp in data.get("acceptances", {}).items()},
                       overrides=list(data.get("overrides", [])), punch_file=data.get("punch_file"),
                       punch_offset=[float(v) for v in data.get("punch_offset", [0.0, 0.0, 0.0])],
                       punch_mount=int(data.get("punch_mount", 0)), punch_target=int(data.get("punch_target", 0)),
                       punch_bases=list(data.get("punch_bases", [])),
                       punch_mounts=[int(n) for n in data.get("punch_mounts", [])],
                       punch_targets=[int(n) for n in data.get("punch_targets", [])],
                       tip_joints=[[float(v) for v in joints] for joints in data.get("tip_joints", [])],
                       tip_mounts=[int(n) for n in data.get("tip_mounts", [])],
                       on_stand=bool(data.get("on_stand", False)), notes=str(data.get("notes", "")),
                       environment=str(data.get("environment", REAL)), ros_domain=int(data.get("ros_domain", 0)))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"{file.name}: not a calibration run file ({error!r})") from None

    @property
    def arm_sweeps(self) -> ArmSweeps:
        """ArmSweeps: The run's sweeps."""
        return ArmSweeps(self.sweeps)


def _ticks(data: dict) -> tuple[dict[str, float], str | None]:
    """Each checklist box's tick time and the tool model Tool on flange was ticked for, from a run file.

    An older one kept one time for the boxes but Tool on flange (`checklist_time`), and that box's [model, time]
    (`tool_checked`).
    """
    if "ticks" in data:
        return {label: float(at) for label, at in data["ticks"].items()}, data.get("tool_ticked_for")
    ticks = {label: float(data["checklist_time"]) for label in CHECKLIST_BOXES
             if label != TOOL_BOX and data.get("checklist_time") is not None}
    model = None
    if data.get("tool_checked"):
        model, ticks[TOOL_BOX] = data["tool_checked"][0], float(data["tool_checked"][1])
    return ticks, model


def _made(value) -> dict:
    """What a file was measured with (`Made.as_dict`); an older run file's list gives definitions not known."""
    return dict(value) if isinstance(value, dict) else {"tool_body": int(value[0]), "base": None, "tool": None}


def new_run(session: Path, serial: str, arm: str, sweeps: ArmSweeps, time_of_day: str,
            tool_body: int = DEFAULT_TOOL_BODY, simulated: bool = False) -> Progress:
    """Start a run in a session folder and save it.

    Args:
        session: The session folder, made already.
        serial: The robot.
        arm: The arm.
        sweeps: The arm's sweeps.
        time_of_day: HHMMSS, for the file name.
        tool_body: The Motive rigid body on the flange, as the operator chose it.
        simulated: Whether the run measures a simulation (`Run.simulated`).
    """
    progress = Progress(session / f"run_{serial[-4:]}_{arm}_{time_of_day}.json", serial, arm, tool_body,
                        sweeps.sweeps, **environment(simulated))
    progress.save()
    return progress


def runs(sessions: Path, serial: str, arm: str) -> list[Progress]:
    """Every readable run of a robot's arm, in any session folder, newest first."""
    files = sorted(sessions.glob(f"*/{RUN_PATTERN.format(digits=serial[-4:], arm=arm)}"),
                   key=lambda path: (path.parent.name, path.name), reverse=True)
    found = []
    for file in files:
        try:
            found.append(Progress.load(file))
        except ValueError:
            continue
    return found
