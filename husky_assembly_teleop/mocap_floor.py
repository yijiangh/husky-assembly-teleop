"""The measured lab floor in the MoCap world (numpy + stdlib only).

Motive's ground plane is not the real floor. Probing the floor with a tracked
tip (2026-10-22, 10 points) found it tilted by 0.49 deg against the MoCap XY
plane, 2.5 mm below the MoCap origin and about 41 mm below it at y = -4.3 m. The
probe session writes a ``<stamp>_mocap_floor_calibration.json``::

    {"kind": "mocap_floor_calibration",
     "floor_from_mocap": {"matrix": [[...4x4...]], "position": [...], "orientation": [...]},
     "plane": {"points": 10, "normal": [...], "centroid": [...], "rms": 0.002, "tilt_deg": 0.49, ...},
     "origin": {"name": "origin_1", "position": [...], "height": -0.0032},
     "samples": [{"name": "floor_1", "label": "floor", "position": [x, y, z], ...}, ...]}

``floor_from_mocap`` maps a MoCap (world, Z-up, metres) point into the floor
frame: z along the floor's normal, the origin the MoCap origin dropped onto the
floor, x the MoCap x laid onto the floor. So the floor-frame z of a point is its
height above the floor.

The world of the monitor stays the MoCap world: nothing measured by MoCap is
moved (robot bases, progress.json, markers). Things that ARE the floor (the
collision slab, the floor grids) and readouts of "height above the floor" use
this plane, and the Rhino design is PLACED on it: the design was drawn on a flat
ground at the exported WalkableGround height (-15.55 mm), so its ground bars'
footers would float 6-23 mm above the real floor. ``MocapFloor.world_from_design``
is the rigid move that lays that flat ground onto the measured floor
(bar_action_io applies it to the Rhino poses of every action the monitor reads).

What is here:

- `load_mocap_floor(path)` -> `MocapFloor`, or None when there is no usable file
- `MocapFloor.z_at(x, y)` -> the floor's MoCap height under a point
- `MocapFloor.height_above(points)` -> a point's height above the floor
- `MocapFloor.angle_to_normal_deg(direction)` -> how far a direction leans from the floor's up
- `MocapFloor.grid_segments(...)` -> straight lines on the floor, to draw a grid
- `MocapFloor.world_from_design(ground_z)` -> 4x4, the Rhino design placed on the floor
- `walkable_ground_faces(path)` -> a WalkableGround.json's faces (outline + heights, metres)
- `design_ground_height(path)` -> the height of a design's flat ground
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from json import load as json_load_std
from typing import List, Optional, Tuple

import numpy as np

from husky_assembly_teleop import MOCAP_FLOOR_CALIBRATION_FILE

FLOOR_CALIBRATION_KIND = 'mocap_floor_calibration'
# WalkableGround.json coordinates above this magnitude are millimetres (Rhino's
# unit) and get scaled to metres. Same heuristic as mocap_experiment._walkable_ground_polygons.
WALKABLE_GROUND_MM_THRESHOLD = 50.0
# A WalkableGround face whose corners differ in height by less than this is a flat
# ground (a wall-like face, as 260920 has one, is not).
FLAT_GROUND_TOL_M = 0.001


@dataclass(frozen=True)
class MocapFloor:
    """The floor plane measured in the MoCap world.

    Attributes:
        floor_from_mocap (np.ndarray): 4x4, MoCap (world) point -> floor frame.
        samples (np.ndarray): (N, 3) MoCap positions of the probed floor points, metres.
        rms_m (float): How far the probed points sit from the fitted plane (rms), metres.
        tilt_deg (float): Angle between the floor's normal and the MoCap Z axis.
        path (str): The calibration file it was read from.
    """
    floor_from_mocap: np.ndarray
    samples: np.ndarray
    rms_m: float
    tilt_deg: float
    path: str

    @property
    def world_from_floor(self) -> np.ndarray:
        """4x4, floor frame -> MoCap (world): places floor-frame drawings in the world."""
        return np.linalg.inv(self.floor_from_mocap)

    @property
    def normal(self) -> np.ndarray:
        """The floor's up direction in the MoCap world (unit vector)."""
        return self.floor_from_mocap[2, :3]

    def height_above(self, points) -> np.ndarray:
        """Height above the floor, metres (negative below it).

        Measured along the floor's normal. At a 0.5 deg tilt this differs from a
        plumb-line height by less than 0.01 %, so it is what a tape measure reads.

        Args:
            points (array-like): One (3,) point or (N, 3) points in the MoCap world.

        Returns:
            np.ndarray: A float for one point, (N,) for several.
        """
        pts = np.asarray(points, dtype=float)
        return pts @ self.floor_from_mocap[2, :3] + self.floor_from_mocap[2, 3]

    def z_at(self, x: float, y: float) -> float:
        """The floor's MoCap z right under (or above) the MoCap point (x, y).

        Args:
            x (float): MoCap x, metres.
            y (float): MoCap y, metres.

        Returns:
            float: z of the floor at (x, y), metres.
        """
        a, b, c, d = self.floor_from_mocap[2]
        # * On the floor: a x + b y + c z + d = 0, solved for z (c is ~1).
        return float(-(a * x + b * y + d) / c)

    def angle_to_normal_deg(self, direction) -> float:
        """Angle between a direction and the floor's up, degrees.

        Args:
            direction (array-like): (3,) direction in the MoCap world, e.g. a
                robot base's z axis.

        Returns:
            float: 0 when it stands straight up from the floor.
        """
        d = np.asarray(direction, dtype=float)
        cos = float(np.dot(d, self.normal) / np.linalg.norm(d))
        return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))

    def grid_segments(self, spacing: float = 1.0,
                      margin: float = 1.0) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Straight lines lying on the floor, a square grid over the probed area.

        The grid covers the probed points plus ``margin`` on every side, on whole
        multiples of ``spacing`` in the floor frame (so a line passes through the
        floor origin, right under the MoCap origin).

        Args:
            spacing (float): Distance between lines, metres.
            margin (float): How far the grid reaches past the outermost probed point, metres.

        Returns:
            list: ``(start, end)`` pairs of (3,) MoCap-world points.
        """
        # A file without probed points still gets a grid around the origin.
        samples = self.samples if len(self.samples) else np.zeros((1, 3))
        samples_floor = (self.floor_from_mocap[:3, :3] @ samples.T).T + self.floor_from_mocap[:3, 3]
        lo = np.floor((samples_floor[:, :2].min(axis=0) - margin) / spacing) * spacing
        hi = np.ceil((samples_floor[:, :2].max(axis=0) + margin) / spacing) * spacing
        world_from_floor = self.world_from_floor

        def to_world(x: float, y: float) -> np.ndarray:
            """A floor-frame point on the floor (z = 0) in the MoCap world."""
            return world_from_floor[:3, :3] @ np.array([x, y, 0.0]) + world_from_floor[:3, 3]

        segments = []
        for x in np.arange(lo[0], hi[0] + spacing / 2, spacing):
            segments.append((to_world(x, lo[1]), to_world(x, hi[1])))
        for y in np.arange(lo[1], hi[1] + spacing / 2, spacing):
            segments.append((to_world(lo[0], y), to_world(hi[0], y)))
        return segments

    def world_from_design(self, ground_z: float) -> np.ndarray:
        """The rigid move that places the Rhino design on the measured floor.

        The design stands on a flat ground at height ``ground_z`` (its
        WalkableGround). This move tilts the design with the floor (the floor
        frame's rotation: the design's up becomes the floor's up, its x stays
        over the MoCap x) and lifts it so that the design ground point under the
        MoCap origin, ``(0, 0, ground_z)``, lands on the floor right under the
        MoCap origin, ``(0, 0, z_at(0, 0))``. Every point of the design's ground
        then lies on the floor, and every other point keeps its height above it.
        The MoCap origin's x, y do not move (a point at the design ground there
        only goes up or down).

        Args:
            ground_z (float): The design's ground height, metres (-0.01555 for
                the 2026-10 exports).

        Returns:
            np.ndarray: 4x4, design point -> MoCap (world) point.
        """
        rotation = self.world_from_floor[:3, :3]
        design_pivot = np.array([0.0, 0.0, float(ground_z)])
        floor_pivot = np.array([0.0, 0.0, self.z_at(0.0, 0.0)])
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = floor_pivot - rotation @ design_pivot
        return matrix


def walkable_ground_faces(path: str) -> List[Tuple[List[Tuple[float, float]], List[float]]]:
    """Every face of a WalkableGround.json, in metres.

    Args:
        path (str): The problem's ``WalkableGround.json``.

    Returns:
        list: ``(outline, heights)`` per face: ``outline`` the ``(x, y)``
        corners in order, ``heights`` their z. Empty when the file is missing
        or unreadable (a warning is printed for the latter).
    """
    if not os.path.exists(path):
        return []
    try:
        with open(path, 'r') as f:
            raw = json_load_std(f)
    except Exception as exc:
        print(f"[ground] WARN: could not read {path}: {exc}")
        return []
    data = raw.get('data', raw)
    faces = []
    for ground in (data.get('grounds') or {}).values():
        md = ground.get('data', ground)
        vertex = md.get('vertex') or {}
        if not vertex:
            continue
        # Rhino exports these in millimetres; scale only when they look like it.
        coords = np.array([[v.get('x', 0.0), v.get('y', 0.0)] for v in vertex.values()], dtype=float)
        scale = 0.001 if np.abs(coords).max() > WALKABLE_GROUND_MM_THRESHOLD else 1.0
        for face_vertices in (md.get('face') or {}).values():
            corners = [vertex[str(i)] for i in face_vertices]
            faces.append(([(c.get('x', 0.0) * scale, c.get('y', 0.0) * scale) for c in corners],
                          [c.get('z', 0.0) * scale for c in corners]))
    return faces


def design_ground_height(path: str) -> Optional[float]:
    """The height of a design's flat ground, from its WalkableGround.json.

    Args:
        path (str): The problem's ``WalkableGround.json``.

    Returns:
        float | None: Metres (the median over the flat faces, -0.01555 for the
        2026-10 exports); None without a flat face.
    """
    flat = [float(np.mean(heights)) for _outline, heights in walkable_ground_faces(path)
            if heights and np.ptp(heights) < FLAT_GROUND_TOL_M]
    return float(np.median(flat)) if flat else None


def load_mocap_floor(path: Optional[str] = None) -> Optional[MocapFloor]:
    """Read a floor calibration file.

    Args:
        path (str | None): The ``*_mocap_floor_calibration.json``; None reads the
            pinned ``MOCAP_FLOOR_CALIBRATION_FILE``.

    Returns:
        MocapFloor | None: None (with a printed warning) when the file is missing
        or is not a floor calibration; callers then keep the exported floor height.
    """
    if path is None:
        path = MOCAP_FLOOR_CALIBRATION_FILE
    if not os.path.exists(path):
        print(f"[floor] WARN: no floor calibration at {path}; using the exported floor height.")
        return None
    try:
        with open(path, 'r') as f:
            raw = json_load_std(f)
        if raw.get('kind') != FLOOR_CALIBRATION_KIND:
            raise ValueError(f"kind is {raw.get('kind')!r}, not {FLOOR_CALIBRATION_KIND!r}")
        matrix = np.array(raw['floor_from_mocap']['matrix'], dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(f"floor_from_mocap is {matrix.shape}, not 4x4")
        # ! A rotation that is not orthonormal would make height_above lie.
        if not np.allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-6):
            raise ValueError("floor_from_mocap's rotation is not orthonormal")
        # * The points the plane was fitted to: the floor probes, plus (since 2026-10-27)
        # * the probes of the fixed floor points (origin, mark1..), labelled by their name.
        on_floor = {'floor'} | set((raw.get('fixed_points') or {}).keys())
        samples = np.array([s['position'] for s in raw.get('samples', [])
                            if s.get('label') in on_floor], dtype=float).reshape(-1, 3)
        plane = raw.get('plane', {})
    except Exception as exc:
        print(f"[floor] WARN: could not read {path}: {exc}; using the exported floor height.")
        return None
    return MocapFloor(floor_from_mocap=matrix, samples=samples,
                      rms_m=float(plane.get('rms', float('nan'))),
                      tilt_deg=float(plane.get('tilt_deg', float('nan'))), path=path)
