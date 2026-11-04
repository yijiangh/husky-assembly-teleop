"""
A calibration session's files on the Drive, in the old pipeline's formats so both read each other's sessions.

    <session>/                             data_experiment/calibration_data/<YYYYMMDD>
      j0/ j1/ validation/calibration_*.json  one take each: {"raw_data": [sample, ...]}; archive/ below each holds
                                             the takes a run no longer uses, which nothing reads
      base_frame_calibration.json            the arm base in the base rigid body, and the axis lines
      calibrated_transformation_<digits>_rhino.json   base_mocap_from_base_footprint: what the relay applies
      calibrated_transformation_<digits>_rhino_in_progress.json   the same from Solve, until Apply renames it
      punch_validation/punch_validation_<arm>_<YYYYMMDD_HHMM>.json   punch takes, read by 4_punch_validation.py
      analysis/<digits>_<arm>/               the plugin's own analysis (`report.py`); the old scripts never touch it

A sample holds `joint_conf` (six angles, radians), `base_mocap_pose` (the base rigid body) and `flange_mocap_pose`
(the tool body), poses as [[x, y, z], [qx, qy, qz, qw]] in the world; extra keys are kept and ignored.

Plain files; any thread.
"""

from __future__ import annotations

import json
import os
import shutil
import time
from datetime import datetime
from pathlib import Path
from typing import Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ....drive import environment
from ....robot_interface.mocap_definitions import BodyDefinition, body_dict
from ....world.calibration_sessions import DEFINITION_KEY, SESSION_OUTPUT
from ..analysis.fit import Solution, Take, arm_base_link, tool_points

#: The batches a session's takes are filed under, by swept joint; validation samples go to the last.
BATCHES = ("j0", "j1")
VALIDATION_BATCH = "validation"
PUNCH_BATCH = "punch_validation"
#: Where files no longer used go, below their folder: no solve and no relay reads it.
ARCHIVE = "archive"
#: Added to the relay's file name while Solve's result awaits Apply.
IN_PROGRESS = "_in_progress"
#: The plugin's own analysis, one folder per robot and arm below it.
ANALYSIS = "analysis"
#: A joint counts as held within a take if it moves less than this, radians.
HELD = 1e-3


def pose_list(pose: Pose) -> list[list[float]]:
    """Return a pose in the files' format, [[x, y, z], [qx, qy, qz, qw]]."""
    return [list(pose.position), list(pose.orientation)]


def pose_from(entry: Sequence[Sequence[float]]) -> Pose:
    """Read a pose in the files' format."""
    return Pose.from_arrays(entry[0], entry[1])


def swept_joint(joints: np.ndarray) -> int | None:
    """Which single joint a take turns, or None if none or several move.

    Args:
        joints: (n, 6) joint angles per sample, radians.
    """
    moving = np.flatnonzero(np.ptp(joints, axis=0) > HELD)
    return int(moving[0]) if len(moving) == 1 else None


def complete_samples(path: Path) -> list[dict]:
    """Read a take file's samples that have joints, the base body and the tool body."""
    return [entry for entry in json.loads(path.read_text())["raw_data"]
            if entry.get("flange_mocap_pose") and entry.get("base_mocap_pose") and entry.get("joint_conf")]


def read_takes(session: Path) -> tuple[list[Take], list[str]]:
    """Read a session's J0 and J1 takes from all its `j0/` and `j1/` take files, filed by the joint they turn.

    A file goes by the joint its samples turn, not its folder, so a take saved to the wrong folder still counts.
    Files with the same samples count once.

    Returns:
        tuple: The takes, J0 ones first, in file-name order; and one line per file skipped, saying why.
    """
    takes, skipped, seen = [], [], set()
    files = (path for batch in BATCHES for path in (session / batch).glob("calibration_*.json"))
    for path in sorted(files, key=lambda path: path.name):
        label = f"{path.parent.name}/{path.name}"
        samples = complete_samples(path)
        if len(samples) < 3:
            skipped.append(f"{label}: {len(samples)} complete samples, a circle needs three")
            continue
        joints = np.array([entry["joint_conf"] for entry in samples], dtype=float)
        joint = swept_joint(joints)
        if joint not in (0, 1):
            skipped.append(f"{label}: turns no single joint of J0 and J1")
            continue
        key = json.dumps(samples, sort_keys=True)
        if key in seen:
            skipped.append(f"{label}: the same samples as an earlier file")
            continue
        seen.add(key)
        points = tool_points([pose_from(entry["base_mocap_pose"]) for entry in samples],
                             [pose_from(entry["flange_mocap_pose"]) for entry in samples])
        takes.append(Take(joint, points, joints[:, joint], label))
    return sorted(takes, key=lambda take: take.joint), skipped


def read_take(path: Path, joint: int) -> tuple[Take, Pose]:
    """Read one take file as recorded by the calibration plugin.

    Returns:
        tuple: The take, in the base rigid body; and the calibration the relay applied during its first sample
        (identity for a file without it).

    Raises:
        ValueError: With fewer than three complete samples.
    """
    samples = complete_samples(path)
    if len(samples) < 3:
        raise ValueError(f"{path.name}: {len(samples)} complete samples, a circle needs three")
    points = tool_points([pose_from(entry["base_mocap_pose"]) for entry in samples],
                         [pose_from(entry["flange_mocap_pose"]) for entry in samples])
    applied = samples[0].get("applied_calibration")
    calibration = pose_from(applied["pose"]) if applied else Pose()
    return Take(joint, points, np.array([entry["joint_conf"][joint] for entry in samples]), path.name), calibration


def read_samples(path: Path) -> tuple[list[Pose], list[Pose], np.ndarray]:
    """Read one take file's complete samples.

    Returns:
        tuple: The base body's poses, the tool body's poses, and the (n, 6) joint angles.
    """
    samples = complete_samples(path)
    return ([pose_from(entry["base_mocap_pose"]) for entry in samples],
            [pose_from(entry["flange_mocap_pose"]) for entry in samples],
            np.array([entry["joint_conf"] for entry in samples], dtype=float).reshape(-1, 6))


def read_marker_errors(path: Path) -> list[dict | None]:
    """The marker errors stored with each of a take file's complete samples ({"base": m, "tool": m}); None without."""
    return [entry.get("marker_errors") for entry in complete_samples(path)]


def read_validation(session: Path) -> tuple[list[Pose], list[Pose], np.ndarray]:
    """Read every sample of the session's `validation/` take files.

    Returns:
        tuple: The base body's poses, the tool body's poses, and the (n, 6) joint angles.
    """
    samples = [entry for path in sorted((session / VALIDATION_BATCH).glob("calibration_*.json"))
               for entry in complete_samples(path)]
    return ([pose_from(entry["base_mocap_pose"]) for entry in samples],
            [pose_from(entry["flange_mocap_pose"]) for entry in samples],
            np.array([entry["joint_conf"] for entry in samples], dtype=float).reshape(-1, 6))


def write_take(folder: Path, name: str, samples: Sequence[dict]) -> Path:
    """Write one take's samples as `calibration_<name>.json`; ! never overwrites a file.

    Raises:
        FileExistsError: If the file is there already.
    """
    path = folder / f"calibration_{name}.json"
    with path.open("x") as file:
        json.dump({"raw_data": list(samples)}, file, indent=4)
    return path


def _point(pose: Pose) -> dict:
    return {"position": [float(v) for v in pose.position], "quaternion": [float(v) for v in pose.orientation]}


def punch_take(arm_name: str, arm_index: int, tool0: str, joints: np.ndarray, base: Pose, tip: Pose,
               offset: np.ndarray, calibration: Pose = Pose(), calibration_sha: str = "",
               base_error: float | None = None, lab_from_raw: Pose | None = None) -> dict:
    """One punch take in the old monitor's format, plus the base body's pose and marker error, calibration, lab frame.

    The old keys (`base_pose`, `world_from_punch_tip`) are under the calibration applied then; `base_mocap_pose`, the
    joints and `tool0_from_punch_tip` let the take be judged again under any calibration and TCP offset.

    Args:
        arm_name: "single", "left" or "right", as the old monitor named arms.
        arm_index: 0 for a single or left arm, 1 for right.
        tool0: The arm's tool0 link.
        joints: Six joint angles, radians.
        base: base_footprint in the world, as the relay published it.
        tip: The tip in the world.
        offset: tool0 to tip, metres: the TCP offset used.
        calibration: base_footprint in the base rigid body, as the relay applied it; identity for none.
        calibration_sha: Its hash, from the relay; "" for none.
        base_error: The base body's marker error the relay reported at the take, metres; None if unknown. Older
            files have no "marker_errors".
        lab_from_raw: The lab frame the poses are in, as the relay applied it (`Run.lab_frame`); None if not known,
            as in older files.
    """
    return {"timestamp": datetime.now().isoformat(), "arm_index": arm_index, "arm_name": arm_name,
            "tool0_link_name": tool0, "joint_conf": [float(v) for v in joints], "base_pose": _point(base),
            "world_from_punch_tip": _point(tip), "tool0_from_punch_tip": _point(Pose(tuple(offset))),
            "base_mocap_pose": pose_list(compose(base, invert(calibration))),
            "applied_calibration": {"sha256": calibration_sha, "pose": pose_list(calibration)},
            "marker_errors": {"base": None if base_error is None else float(base_error)},
            "lab_from_raw": pose_list(lab_from_raw) if lab_from_raw is not None else None}


def write_punch(path: Path, takes: Sequence[dict]) -> None:
    """Write a punch validation file, replacing it atomically: its takes and, from the first, the arm and offset."""
    first = takes[0]
    data = {key: first[key] for key in ("arm_index", "arm_name", "tool0_link_name", "tool0_from_punch_tip")}
    data["takes"] = list(takes)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=4))
    os.replace(temporary, path)


def read_punch(path: Path) -> list[dict]:
    """A punch validation file's takes."""
    return list(json.loads(path.read_text())["takes"])


def restore(path: Path) -> Path:
    """Move a file back out of `archive/`, as it is used again; return where it went."""
    if path.parent.name != ARCHIVE:
        return path
    return Path(shutil.move(path, path.parent.parent / path.name))


def archive(path: Path) -> Path:
    """Move a file no longer used into `archive/` next to it, where nothing reads it; return where it went."""
    if path.parent.name == ARCHIVE:
        return path
    (path.parent / ARCHIVE).mkdir(exist_ok=True)
    return Path(shutil.move(path, path.parent / ARCHIVE / path.name))


def analysis_folder(session: Path, digits: str, arm: str) -> Path:
    """The plugin's own analysis of one robot arm in a session: `analysis/<digits>_<arm>/`, created if missing."""
    folder = session / ANALYSIS / f"{digits}_{arm}"
    folder.mkdir(parents=True, exist_ok=True)
    return folder


def calibration_file(session: Path, digits: str, approved: bool = True) -> Path:
    """Return the session's calibration for robot `digits` (e.g. "0806"): the relay's, or Solve's result in progress.

    No relay reads a result in progress, until `approve_calibration` gives it the relay's name.
    """
    name = SESSION_OUTPUT.format(digits=digits)
    return session / (name if approved else name.replace(".json", f"{IN_PROGRESS}.json"))


def approve_calibration(session: Path, digits: str) -> Path:
    """Give the session's calibration in progress the name the relay reads; an approved one there moves to archive/.

    Returns:
        Path: The approved calibration file.

    Raises:
        FileNotFoundError: If no calibration is in progress.
    """
    approved, pending = calibration_file(session, digits), calibration_file(session, digits, approved=False)
    if not pending.is_file():
        raise FileNotFoundError(f"no calibration in progress: {pending}")
    if approved.is_file():
        (session / ARCHIVE).mkdir(exist_ok=True)
        shutil.move(approved, session / ARCHIVE / f"{approved.stem}_{time.strftime('%H%M%S')}.json")
    os.replace(pending, approved)
    return approved


def mount_check_file(session: Path, digits: str, arm: str) -> Path:
    """Return the session's mount check of one arm; the relay never reads it."""
    return session / f"arm_mount_check_{digits}_{arm}.json"


def write_mount_check(session: Path, digits: str, arm: str, measured: Pose, correction: Pose,
                      calibration: Pose) -> Path:
    """Write an arm's measured mount, for checking the URDF's (`fit.mount_correction`), never the relay's file.

    Returns:
        Path: The file written.
    """
    output = mount_check_file(session, digits, arm)
    output.write_text(json.dumps({f"base_footprint_from_{arm_base_link(arm)}": pose_list(measured),
                                  "urdf_to_measured": pose_list(correction),
                                  "base_calibration_applied": pose_list(calibration)}, indent=4))
    return output


def write_solution(session: Path, digits: str, arm: str, mocap_id: int, solution: Solution,
                   approved: bool = True, body_definition: BodyDefinition | None = None,
                   simulated: bool = False) -> Path:
    """Write the session's outputs, as the old scripts 1 and 2 did, for a solution in the base rigid body's frame.

    The calibration file also names the rigid body (`mocap_id`) and its definition the takes were recorded with
    (`body_definition`, if known), which the relay checks, and whether it was `simulated` (`drive.environment`). Not
    `approved`, it is written in progress (`calibration_file`).

    Returns:
        Path: The calibration file the relay reads.
    """
    measured = f"base_mocap_from_{arm_base_link(arm)}"
    frame = {
        measured: solution.arm_base.matrix().tolist(),
        "j0_line": {"point": solution.j0.point.tolist(), "direction": solution.j0.direction.tolist()},
        "j1_line": {"point": solution.j1.point.tolist(), "direction": solution.j1.direction.tolist()},
        "base_offset": solution.shoulder_height,
    }
    (session / "base_frame_calibration.json").write_text(json.dumps(frame, indent=4))
    output = calibration_file(session, digits, approved)
    content = {"mocap_id": mocap_id, measured: pose_list(solution.arm_base),
               "base_mocap_from_base_footprint": pose_list(solution.footprint), **environment(simulated)}
    if body_definition is not None:
        content[DEFINITION_KEY] = body_dict(body_definition)
    output.write_text(json.dumps(content, indent=4))
    return output
