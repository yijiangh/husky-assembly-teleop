"""
Which base calibration the mocap relay applies to each robot's rigid body: crl-husky's default, or a newer one.

- Default: the one in the robot's `mocap_config.json` in crl-husky (`config_resolver`), always there and rough.
- Newer: the newest calibration session on the Drive, `data_experiment/calibration_data/<YYYYMMDD>/
  calibrated_transformation_<serial>_rhino.json`, used only if it passes `sanity_problem` against the default.
- Rigid bodies of no robot (probe, bars) get none: Motive's pivot defines them.

Pure functions of files; no ROS, any thread.
"""

from __future__ import annotations

import hashlib
import json
import re
import textwrap
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from crl_husky import config_resolver

#: Calibration sessions, below the Drive root: one folder per day, YYYYMMDD.
SESSIONS_FOLDER = Path("data_experiment") / "calibration_data"
SESSION_NAME = re.compile(r"\d{8}")
#: The pipeline's output in a session, per robot's four serial digits.
SESSION_OUTPUT = "calibrated_transformation_{digits}_rhino.json"
#: A Drive calibration further than this from the default is refused: metres, radians.
#: ? A recalibration moves the base by millimetres; centimetres mean a wrong robot or a failed run.
SANITY_POSITION = 0.05
SANITY_ANGLE = np.radians(5.0)
BASE_KEY = "base_mocap_from_base_footprint"
#: Column titles over `Calibration.describe` rows.
DESCRIBE_HEADER = f"{'body':<6}{'robot':<11}{'calibration':<24}{'sha256':<14}measured on"


@dataclass(frozen=True)
class Calibration:
    """One rigid body's base calibration: the robot's base_footprint in the rigid body's frame.

    Attributes:
        mocap_id: The rigid body.
        robot_serial: Its robot, e.g. "a200-0806".
        position: `base_mocap_from_base_footprint` position, metres, rhino convention (Z-up).
        orientation: Its quaternion (x, y, z, w).
        source: Where it came from: crl-husky's config file, or the session file below the Drive root.
        sha256: Hash of the calibration's content, to tell versions apart.
        measured: What the calibration measured besides the derived base, e.g.
            ("base_mocap_from_left_ur_arm_base_link_inertia",): which arm it was calibrated on.
        session: The Drive session's YYYYMMDD, or "" for the default.
    """

    mocap_id: int
    robot_serial: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    source: str
    sha256: str
    measured: tuple[str, ...]
    session: str = ""

    def describe(self) -> str:
        """One row of a table under DESCRIBE_HEADER: id, robot, origin, short hash and what was measured."""
        origin = f"Drive session {self.session}" if self.session else "crl-husky default"
        measured = ", ".join(key.removeprefix("base_mocap_from_") for key in self.measured) or "?"
        return f"{self.mocap_id:<6}{self.robot_serial:<11}{origin:<24}{self.sha256[:12]:<14}{measured}"

    def as_dict(self) -> dict:
        """Everything, for the relay's published list."""
        return {"mocap_id": self.mocap_id, "robot_serial": self.robot_serial, "source": self.source,
                "session": self.session, "sha256": self.sha256, "measured": list(self.measured),
                BASE_KEY: [list(self.position), list(self.orientation)]}


def _calibration(mocap_id: int, serial: str, content: dict, source: str, session: str = "") -> Calibration:
    """Build a Calibration from a calibration file's content.

    Raises:
        ValueError: If the base transform is missing or malformed.
    """
    pose = content.get(BASE_KEY)
    try:
        position = tuple(float(v) for v in pose[0])
        orientation = tuple(float(v) for v in pose[1])
    except (TypeError, ValueError, IndexError):
        raise ValueError(f"{source}: {BASE_KEY} must be [[x, y, z], [qx, qy, qz, qw]]") from None
    if len(position) != 3 or len(orientation) != 4 or not np.all(np.isfinite([*position, *orientation])):
        raise ValueError(f"{source}: {BASE_KEY} must be 3 + 4 finite numbers")
    if abs(np.linalg.norm(orientation) - 1.0) > 1e-3:
        raise ValueError(f"{source}: {BASE_KEY} quaternion is not unit length")
    measured = tuple(sorted(key for key in content if key.startswith("base_mocap_from_") and key != BASE_KEY))
    # * Of the transforms only, so the same numbers have the same hash wherever they are kept.
    transforms = {key: content[key] for key in (*measured, BASE_KEY)}
    digest = hashlib.sha256(json.dumps(transforms, sort_keys=True).encode()).hexdigest()
    return Calibration(mocap_id, serial, position, orientation, source, digest, measured, session)


def robot_mocap_ids() -> dict[int, str]:
    """Every robot's rigid body id and its serial, from crl-husky."""
    return {mocap_id: serial for serial in config_resolver.get_robot_serials_with_mocap_config()
            for mocap_id in config_resolver.get_mocap_ids_for_robot_serial(serial)}


def default_calibration(mocap_id: int) -> Calibration | None:
    """crl-husky's default for a robot's rigid body, or None if the id is no robot's or has none.

    Raises:
        ValueError: If the default is malformed.
    """
    entry = config_resolver.get_default_mocap_calibration(mocap_id)
    if entry is None:
        return None
    content = {key: value for key, value in entry.items() if key not in ("mocap_id", "robot_serial", "file")}
    return _calibration(mocap_id, entry["robot_serial"], content, entry["file"])


def newest_session_file(root: Path, robot_serial: str) -> Path | None:
    """The robot's calibration file in the newest session folder that has one, or None.

    ! Only directly inside each dated session folder: `archive/` and `backup/` below it are older attempts.
    """
    sessions = root / SESSIONS_FOLDER
    if not sessions.is_dir():
        return None
    name = SESSION_OUTPUT.format(digits=robot_serial[-4:])
    for session in sorted((p for p in sessions.iterdir() if p.is_dir() and SESSION_NAME.fullmatch(p.name)),
                          key=lambda p: p.name, reverse=True):
        if (session / name).is_file():
            return session / name
    return None


def read_session_file(path: Path, root: Path, mocap_id: int, robot_serial: str) -> Calibration:
    """Read one session's calibration for a robot's rigid body.

    TEMPORARY compatibility: the pipeline writes no `mocap_id` yet (TODO in the calibration pipeline), so the id
    comes from crl-husky by the robot's serial; a file that does name one must name this one.

    Raises:
        ValueError: If the file does not parse, names another rigid body, or its transform is malformed.
    """
    source = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
    try:
        content = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"{source}: cannot read it ({error}); half-synced?") from None
    if not isinstance(content, dict):
        raise ValueError(f"{source}: not a calibration object")
    if "mocap_id" in content and int(content["mocap_id"]) != mocap_id:
        raise ValueError(f"{source}: for rigid body {content['mocap_id']}, not {mocap_id}")
    return _calibration(mocap_id, robot_serial, content, source, session=path.parent.name)


def offset(candidate: Calibration, default: Calibration) -> tuple[float, float]:
    """How far one calibration's base is from another's: metres and radians."""
    distance = float(np.linalg.norm(np.subtract(candidate.position, default.position)))
    angle = float((Rotation.from_quat(default.orientation).inv() * Rotation.from_quat(candidate.orientation))
                  .magnitude())
    return distance, angle


def sanity_problem(candidate: Calibration, default: Calibration) -> str | None:
    """Why a newer calibration is too far from the default to be trusted, or None."""
    distance, angle = offset(candidate, default)
    if distance > SANITY_POSITION or angle > SANITY_ANGLE:
        return (f"{candidate.source} is {distance * 1e3:.0f} mm / {np.degrees(angle):.1f}° from the default "
                f"(over {SANITY_POSITION * 1e3:.0f} mm / {np.degrees(SANITY_ANGLE):.0f}°): "
                f"wrong robot or a failed run?")
    return None


def resolve(mocap_id: int, robot_serial: str, root: Path | None) -> tuple[Calibration | None, list[str]]:
    """The calibration to apply to a robot's rigid body, and warnings about what was passed over.

    The newest Drive session if there is one and it passes the sanity check, else the default.

    Args:
        mocap_id: The rigid body.
        robot_serial: Its robot.
        root: The Drive root, or None to use only the default.

    Returns:
        tuple[Calibration | None, list[str]]: The calibration (None: the robot has none at all), and warnings.
    """
    warnings = []
    try:
        default = default_calibration(mocap_id)
    except ValueError as error:
        default = None
        warnings.append(f"crl-husky's default is unusable: {error}")
    path = newest_session_file(root, robot_serial) if root is not None else None
    if path is None:
        return default, warnings
    try:
        candidate = read_session_file(path, root, mocap_id, robot_serial)
    except ValueError as error:
        warnings.append(f"{error}; using the default")
        return default, warnings
    if default is None:
        warnings.append(f"no default to check {candidate.source} against; using it unchecked")
        return candidate, warnings
    problem = sanity_problem(candidate, default)
    if problem is not None:
        warnings.append(f"{problem}; using the default")
        return default, warnings
    return candidate, warnings


def explain(mocap_id: int, robot_serial: str, calibration: Calibration | None, warnings: list[str],
            root: Path | None, width: int = 96) -> list[str]:
    """Table lines for one robot, under DESCRIBE_HEADER: its row, where it came from, and why a newer one was refused.

    Args:
        mocap_id: The rigid body.
        robot_serial: Its robot.
        calibration: What `resolve` chose, or None.
        warnings: What `resolve` warned about.
        root: The Drive root looked in, or None.
        width: Characters per line.
    """
    def detail(label: str, text: str) -> str:
        # Drive paths relative to the sessions folder and crl-husky's from config/, which the table's title names.
        text = text.replace(f"{SESSIONS_FOLDER.as_posix()}/", "")
        text = text[text.find("config/robots/"):] if text.startswith("/") and "config/robots/" in text else text
        return textwrap.fill(text, width, initial_indent=f"{'':6}{label:<12}", subsequent_indent=" " * 18,
                             break_long_words=False, break_on_hyphens=False)

    lines = [calibration.describe() if calibration else f"{mocap_id:<6}{robot_serial:<11}NONE"]
    if calibration is not None:
        lines.append(detail("from", calibration.source))
        default = default_calibration_or_none(mocap_id) if calibration.session else None
        if default is not None:
            distance, angle = offset(calibration, default)
            lines.append(detail("vs default", "the same transform" if calibration.sha256 == default.sha256 else
                                f"{distance * 1e3:.1f} mm / {np.degrees(angle):.2f}°"))
    if root is not None and calibration is not None and not calibration.session and not warnings:
        lines.append(detail("Drive", "no session for this robot"))
    lines += [detail("! refused", warning) for warning in warnings]
    return lines


def default_calibration_or_none(mocap_id: int) -> Calibration | None:
    """crl-husky's default, or None when it is missing or malformed (`resolve` already warned)."""
    try:
        return default_calibration(mocap_id)
    except ValueError:
        return None
