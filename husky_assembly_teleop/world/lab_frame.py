"""
The lab frame: the ideal positions of the marks on the floor and of Motive's anchor markers.

These ideal positions only fix the lab frame across floor calibrations: every floor calibration fits the frame's
position and turn on the floor plane to the probed marks and the anchors pointed at (`fit_floor_points`), so it finds
the same lab frame again. The relay never reads them: it follows Motive's frame by the anchors and cameras as measured
with the floor calibration (`world/mocap_frame.py`). Pure data and maths; no UI.

! The file (`data/lab/<lab>_frame.json`) defines the lab's frame: written by hand, from a floor calibration's export
  (the marks' floor_xy, the anchors' floor positions), and committed:
  {"format": "husky_lab_frame", "marks": {"origin": [0.0, 0.0], "mark1": [x, y], ...},
   "anchors": {"A1": [x, y, z], ...}}
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .lab import lab_file

#: The marks the probe offers, in order; "origin" is the lab frame's origin.
MARKS = ("origin", "mark1", "mark2", "mark3")
#: A mark or anchor this far from where the file puts it, metres, is suspect: hit badly, or it moved.
MARK_WARN = 2e-3
#: The file's format name.
FORMAT = "husky_lab_frame"


@dataclass(frozen=True)
class LabFrame:
    """The lab frame's ideal points, in it, metres.

    Attributes:
        marks: Each mark's (x, y) on the floor.
        anchors: Each anchor's (x, y, z), z above the floor.
    """

    marks: dict[str, np.ndarray] = field(default_factory=dict)
    anchors: dict[str, np.ndarray] = field(default_factory=dict)


def lab_frame_file(data_directory: Path, lab: str) -> Path:
    """The lab frame file next to the lab file `lab` (a name under `data/lab/`, or a .json path)."""
    path = lab_file(data_directory, lab)
    return path.with_name(f"{path.stem}_frame.json")


def read_lab_frame(path: Path) -> LabFrame:
    """The marks and anchors of a lab frame file; none if the file is missing.

    Raises:
        ValueError: If the file is not a lab frame file, or a mark and an anchor share a name.
    """
    if not path.exists():
        return LabFrame()
    data = json.loads(path.read_text())
    if data.get("format") != FORMAT:
        raise ValueError(f"{path.name} is not a lab frame file (format {data.get('format')!r})")
    try:
        marks = {str(name): np.array(xy, dtype=float).reshape(2) for name, xy in data.get("marks", {}).items()}
        anchors = {str(name): np.array(xyz, dtype=float).reshape(3) for name, xyz in data.get("anchors", {}).items()}
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError(f"{path.name}: a mark or anchor does not read ({error!r})") from None
    if set(marks) & set(anchors):
        raise ValueError(f"{path.name}: marks and anchors share names {sorted(set(marks) & set(anchors))}")
    return LabFrame(marks, anchors)


@dataclass(frozen=True)
class FloorPointsFit:
    """The lab frame fitted to the marks and anchors probed on the floor plane.

    Attributes:
        floor_from_raw: 4x4 transform from raw (Motive's, Z-up) to floor coordinates.
        residuals: Per mark or anchor, how far its probed position lands from its stored one on the floor, metres.
    """

    floor_from_raw: np.ndarray
    residuals: dict[str, float]

    @property
    def worst(self) -> tuple[str, float]:
        """The mark or anchor farthest from its stored position, and how far, metres."""
        return max(self.residuals.items(), key=lambda item: item[1])


def plane_axes(normal: np.ndarray) -> np.ndarray:
    """Rows x, y, z of a frame on the plane with `normal`: z the normal, x the mocap x laid onto the plane."""
    z = np.asarray(normal, dtype=float)
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z
    x /= np.linalg.norm(x)
    return np.array([x, np.cross(z, x), z])


def fit_floor_points(measured: dict[str, np.ndarray], known: dict[str, np.ndarray], normal: np.ndarray,
                     on_plane: np.ndarray) -> FloorPointsFit | None:
    """Fit the lab frame's position and turn on the plane so the probed marks land on their stored (x, y).

    A least-squares fit of all of them alike, none held exactly; z stays the plane's normal.

    Args:
        measured: Probed position of each mark or anchor, raw, metres; only those in `known` are used.
        known: Stored floor (x, y) of each mark or anchor.
        normal: The floor plane's unit normal, raw.
        on_plane: Any point on the plane, raw.

    Returns:
        The fit, or None with fewer than two of the stored points probed.
    """
    names = [name for name in measured if name in known]
    if len(names) < 2:
        return None
    axes, on_plane = plane_axes(normal), np.asarray(on_plane, dtype=float)
    # * Each point in plane coordinates (dropped onto the plane), then the 2D turn and shift that best match them.
    probed = np.array([axes[:2] @ (np.asarray(measured[name], dtype=float) - on_plane) for name in names])
    stored = np.array([known[name] for name in names])
    probed_mean, stored_mean = probed.mean(axis=0), stored.mean(axis=0)
    p, s = probed - probed_mean, stored - stored_mean
    angle = np.arctan2(np.sum(p[:, 0] * s[:, 1] - p[:, 1] * s[:, 0]), np.sum(p * s))
    turn = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    shift = stored_mean - turn @ probed_mean
    rotation = np.vstack([turn @ axes[:2], axes[2]])  # rows: the floor axes in raw coordinates
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.array([*shift, 0.0]) - rotation @ on_plane
    landed = probed @ turn.T + shift
    return FloorPointsFit(transform, {name: float(np.linalg.norm(landed[k] - stored[k]))
                                      for k, name in enumerate(names)})


def floor_point(floor_from_raw: np.ndarray, point: np.ndarray) -> np.ndarray:
    """A raw point in floor coordinates, (x, y, z), metres."""
    return floor_from_raw[:3, :3] @ np.asarray(point, dtype=float) + floor_from_raw[:3, 3]


def floor_xy(floor_from_raw: np.ndarray, point: np.ndarray) -> np.ndarray:
    """A raw point's floor (x, y), metres."""
    return floor_point(floor_from_raw, point)[:2]
