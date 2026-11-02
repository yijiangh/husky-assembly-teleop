"""The lab's cameras and anchors, Motive's frame moved, and floor calibrations as the probe exports them (raw, Z-up)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from repo_paths import DATA
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.mocap_definitions import MocapCamera, cameras_json, read_cameras
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER
from husky_assembly_teleop.world.mocap_frame import FLOOR_SUFFIX

#: The lab's cameras, Z-up, as a floor calibration stores them.
LAB_CAMERAS = read_cameras((DATA / "sim" / "motive_cameras.json").read_text())
#: A recalibration: Motive's frame moved by this much (metres, degrees about z).
SHIFT, TURN = np.array([0.02, -0.01, 0.003]), 0.5
#: Six anchors spread over the lab (Z-up), and a loose marker that is none of them.
LAB_ANCHORS = {"A1": (-3.0, -4.0, 0.01), "A2": (3.5, -4.2, 0.01), "A3": (3.4, 5.0, 0.02), "A4": (-3.2, 5.1, 0.01),
               "A5": (0.4, 0.3, 1.9), "A6": (-1.0, 2.0, 0.6)}
LOOSE = (1.5, 1.1, 1.0)
#: The floor calibration `export` writes by default.
STAMP = "20261024_090000"


def seen_after(point, motions) -> np.ndarray:
    """Where Motive puts a fixed point after its frame moved by each (shift, degrees about z) in turn."""
    for shift, turn in motions:
        point = Rotation.from_euler("z", turn, degrees=True).inv().apply(np.subtract(point, shift))
    return np.asarray(point)


def cameras_after(motions) -> dict:
    """Every lab camera raw (Z-up) after Motive's frame moved by each (shift, degrees about z) in turn."""
    cameras = dict(LAB_CAMERAS)
    for shift, turn in motions:
        inverse = Rotation.from_euler("z", turn, degrees=True).inv()
        cameras = {name: MocapCamera(name, tuple(inverse.apply(np.subtract(camera.position, shift))),
                                     tuple((inverse * Rotation.from_quat(camera.orientation)).as_quat()))
                   for name, camera in cameras.items()}
    return cameras


def anchors_after(motions, names=None) -> list:
    """The lab's anchors (or those `names`) as Motive sees them after its frame moved, raw."""
    return [tuple(seen_after(position, motions)) for name, position in LAB_ANCHORS.items()
            if names is None or name in names]


def export(drive: Path, stamp: str = STAMP, cameras: dict | None = None, floor_from_raw=None,
           anchors: dict | None = None) -> Path:
    """A floor calibration as the probe exports it below `drive`; cameras and anchors raw.

    `floor_from_raw` is (position, quaternion); the cameras are the lab's unless given.
    """
    position, orientation = floor_from_raw or ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    folder = drive / SESSIONS_FOLDER / stamp[:8]
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{stamp}{FLOOR_SUFFIX}"
    path.write_text(json.dumps(
        {"kind": "mocap_floor_calibration", "floor_from_raw": {"position": list(position),
                                                               "orientation": list(orientation)},
         "cameras": json.loads(cameras_json(list((LAB_CAMERAS if cameras is None else cameras).values()),
                                            1.0))["cameras"],
         "anchors": {name: list(at) for name, at in (anchors or {}).items()}}))
    return path
