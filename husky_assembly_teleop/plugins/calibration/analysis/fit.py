"""
The base calibration's math, kept as in the old offline pipeline (`data/calibration_data/0_…` to `3_…`).

- Each sweep of one joint moves the tool body on a circle around that joint's axis: fit a circle per sweep, in the
  frame of a reference body on the base (`fit_circle`).
- The circles of one joint give its axis line (`fit_axis`); J0's and J1's lines give the arm base frame
  (`arm_base_frame`); the URDF chain from the arm base to base_footprint gives the calibration (`solve`).
- Validation: under a good calibration the tool body stays put in tool0 whatever the joints (`tool_offsets`).

The reference body is the base rigid body (the old pipeline), or base_footprint under the calibration applied when
measuring: the result is then in that frame, and is composed with the applied calibration.

Plain numpy; any thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES, load_urdf

from ....robot_interface.frames import MOCAP_FRAME, fixed_transform


@dataclass(frozen=True)
class Take:
    """One sweep of one joint: where the tool body was, and the joint's angle, at each waypoint.

    Attributes:
        joint: The swept joint, 0 (shoulder pan) or 1 (shoulder lift).
        points: (n, 3) tool body positions in the reference body's frame, metres.
        angles: (n,) the swept joint's angle at each point, radians.
        name: Where it came from, for reports, e.g. a file name.
    """

    joint: int
    points: np.ndarray
    angles: np.ndarray
    name: str = ""


@dataclass(frozen=True)
class Circle:
    """A circle fitted to one take.

    Attributes:
        center: (3,) metres.
        normal: (3,) unit normal of the circle's plane, sign as the fit gives it.
        radius: Metres.
        errors: (n,) each point's distance to the circle, metres.
        turn: How consistently the tool turned one way about `normal` as the angle grew: +1 every step
            counter-clockwise, -1 every step clockwise; near 0 means the joint did not sweep.
    """

    center: np.ndarray
    normal: np.ndarray
    radius: float
    errors: np.ndarray
    turn: float


@dataclass(frozen=True)
class Axis:
    """One joint's axis line, from the circles of its takes.

    Attributes:
        point: (3,) a point on the line, metres: the mean circle centre.
        direction: (3,) unit direction, the joint's positive axis (right-hand rule on its angle).
        normal_angles: (k,) each circle's normal angle to `direction`, degrees, sign ignored.
        center_distances: (k,) each circle centre's distance to the line, metres.
    """

    point: np.ndarray
    direction: np.ndarray
    normal_angles: np.ndarray
    center_distances: np.ndarray


@dataclass(frozen=True)
class Solution:
    """A base calibration, solved from J0 and J1 takes.

    Attributes:
        j0_circles: One per J0 take, in take order.
        j1_circles: One per J1 take.
        j0: J0's axis line.
        j1: J1's axis line.
        arm_base: The arm base link (`arm_base_link`) in the reference frame.
        footprint: base_footprint in the reference frame: the calibration when the reference is the base body.
        skew: Shortest distance between the J0 and J1 axis lines, metres (0 for a perfect UR arm).
        axes_angle: Angle between the J0 and J1 axes, degrees (90 for a perfect UR arm).
        shoulder_height: The URDF's height of J1 above the arm base link, metres.
    """

    j0_circles: tuple[Circle, ...]
    j1_circles: tuple[Circle, ...]
    j0: Axis
    j1: Axis
    arm_base: Pose
    footprint: Pose
    skew: float
    axes_angle: float
    shoulder_height: float


def tool_points(references: Sequence[Pose], tools: Sequence[Pose]) -> np.ndarray:
    """Each tool body position in its reference body's frame.

    Args:
        references: The reference body's pose per sample, in the world.
        tools: The tool body's pose per sample, in the world.

    Returns:
        np.ndarray: (n, 3) metres.
    """
    return np.array([compose(invert(reference), tool).position for reference, tool in zip(references, tools)])


def _unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def fit_circle(take: Take) -> Circle:
    """Fit a circle to a take's points: best-fit plane, then a least-squares circle in that plane.

    The same steps as the old pipeline's `circle_fitting_3d.Circle3D` (duplicate points dropped, SVD plane,
    algebraic circle fit), so results match it.

    Raises:
        ValueError: If there are fewer than three distinct points.
    """
    points = np.unique(np.asarray(take.points, dtype=float), axis=0)
    if len(points) < 3:
        raise ValueError(f"take {take.name!r}: a circle needs three distinct points, got {len(points)}")
    centroid = points.mean(axis=0)
    normal = _unit(np.linalg.svd((points - centroid).T)[0][:, 2])
    projected = points - np.outer((points - centroid) @ normal, normal)
    origin = projected.mean(axis=0)
    centered = projected - origin
    u = _unit(centered[1] - centered[0])
    to_plane = np.column_stack((u, np.cross(u, normal), normal))
    flat = (centered @ to_plane)[:, :2]
    solution = np.linalg.lstsq(np.hstack((2 * flat, np.ones((len(flat), 1)))), (flat ** 2).sum(axis=1), rcond=None)[0]
    center = to_plane @ np.array([solution[0], solution[1], 0.0]) + origin
    radius = float(np.sqrt(solution[2] + solution[0] ** 2 + solution[1] ** 2))
    # * Errors and turning sense over all points as measured, duplicates included.
    raw = np.asarray(take.points, dtype=float)
    in_plane = raw - np.outer((raw - center) @ normal, normal) - center
    on_circle = center + radius * in_plane / np.linalg.norm(in_plane, axis=1, keepdims=True)
    steps = np.sign(np.cross(raw[:-1] - center, raw[1:] - center) @ normal) * np.sign(np.diff(take.angles))
    return Circle(center, normal, radius, np.linalg.norm(on_circle - raw, axis=1),
                  float(steps.mean()) if len(steps) else 0.0)


def fit_axis(circles: Sequence[Circle]) -> Axis:
    """Fit one joint's axis line to the circles of its takes: mean normal through the mean centre.

    The direction's sign comes from how the tool turned as the angle grew (the old `joint_axis_from_motion`).
    """
    centers = np.array([circle.center for circle in circles])
    # * Each normal turned to the joint's positive axis first, so the mean cannot cancel out.
    normals = np.array([circle.normal if circle.turn > 0 else -circle.normal for circle in circles])
    direction = _unit(normals.mean(axis=0))
    point = centers.mean(axis=0)
    offsets = centers - point
    distances = np.linalg.norm(offsets - np.outer(offsets @ direction, direction), axis=1)
    angles = np.degrees(np.arccos(np.clip(np.abs(normals @ direction), -1.0, 1.0)))
    return Axis(point, direction, angles, distances)


def arm_base_frame(j0: Axis, j1: Axis, shoulder_height: float) -> np.ndarray:
    """The arm base link's frame from the J0 and J1 axis lines (the old `compute_base_frame`).

    Z is +J0 and Y is -J1, X completes the frame. The origin is where J1 crosses J0's station (plane through J0
    normal to J1, intersected with the plane through J1 containing both axes' directions), `shoulder_height` below.

    Returns:
        np.ndarray: 4x4, the arm base link in the reference frame.
    """
    z = j0.direction
    y = -j1.direction
    x = _unit(np.cross(y, z))
    z = _unit(np.cross(x, y))
    plane0 = (j1.direction, j0.point)
    plane1 = (_unit(np.cross(j0.direction, j1.direction)), j1.point)
    line_direction = _unit(np.cross(plane0[0], plane1[0]))
    # * Any point on both planes; the crossing does not depend on which.
    line_point = np.linalg.lstsq(np.array([plane0[0], plane1[0]]),
                                 np.array([plane0[0] @ plane0[1], plane1[0] @ plane1[1]]), rcond=None)[0]
    crossing = line_point + ((j1.point - line_point) @ line_direction) * line_direction
    matrix = np.eye(4)
    matrix[:3, :3] = np.column_stack((x, y, z))
    matrix[:3, 3] = crossing - shoulder_height * z
    return matrix


def arm_base_link(arm: str) -> str:
    """The link the calibration measures: the parent of the arm's shoulder pan joint, e.g. "ur_arm_base_link_inertia".

    Args:
        arm: The arm's name, e.g. "left_ur_arm".
    """
    return f"{arm}_base_link_inertia"


def shoulder_height(urdf: Path, arm: str) -> float:
    """Height of the shoulder pan joint above the arm base link in the URDF, metres.

    Raises:
        KeyError: If the URDF has no such joint.
    """
    name = f"{arm}_{UR_JOINT_NAMES[0]}"
    joint = next((joint for joint in load_urdf(urdf).robot.joints if joint.name == name), None)
    if joint is None:
        raise KeyError(f"{urdf.name} has no joint {name}")
    return float(joint.origin[2, 3])


def solve(j0_takes: Sequence[Take], j1_takes: Sequence[Take], urdf: Path, arm: str) -> Solution:
    """Solve a base calibration from J0 and J1 takes, in the takes' reference frame.

    Args:
        j0_takes: Sweeps of J0, at least two (different heights).
        j1_takes: Sweeps of J1 with J0 at 0, at least two.
        urdf: The robot's URDF: shoulder height and the chain from the arm base to base_footprint.
        arm: The measured arm, e.g. "ur_arm".

    Raises:
        ValueError: If a joint has fewer than two takes, or a take cannot be fitted.
    """
    if len(j0_takes) < 2 or len(j1_takes) < 2:
        raise ValueError(f"need two takes per joint, got {len(j0_takes)} J0 and {len(j1_takes)} J1")
    j0_circles = tuple(fit_circle(take) for take in j0_takes)
    j1_circles = tuple(fit_circle(take) for take in j1_takes)
    j0, j1 = fit_axis(j0_circles), fit_axis(j1_circles)
    height = shoulder_height(urdf, arm)
    arm_base = Pose.from_matrix(arm_base_frame(j0, j1, height))
    footprint = compose(arm_base, invert(fixed_transform(urdf, MOCAP_FRAME, arm_base_link(arm))))
    common = np.cross(j0.direction, j1.direction)
    skew = float(abs((j1.point - j0.point) @ common) / np.linalg.norm(common))
    axes_angle = float(np.degrees(np.arccos(np.clip(j0.direction @ j1.direction, -1.0, 1.0))))
    return Solution(j0_circles, j1_circles, j0, j1, arm_base, footprint, skew, axes_angle, height)


def mount_correction(calibration: Pose, solution: Solution, urdf: Path, arm: str) -> tuple[Pose, Pose]:
    """An arm's measured mount against the URDF's, for an arm that does not calibrate the base (Cindy's right).

    Args:
        calibration: The base calibration applied while measuring: base_footprint in the base rigid body.
        solution: The arm's solution, in the base rigid body's frame.
        urdf: The robot's URDF.
        arm: The measured arm.

    Returns:
        tuple: The measured arm base link in base_footprint; and the correction, the measured arm base in the URDF's
        (identity if the URDF's mount is right).
    """
    measured = compose(invert(calibration), solution.arm_base)
    return measured, compose(invert(fixed_transform(urdf, MOCAP_FRAME, arm_base_link(arm))), measured)


def tool_offsets(references: Sequence[Pose], tools: Sequence[Pose], joints: Sequence[Sequence[float]],
                 footprint: Pose, urdf: Path, arm: str) -> list[Pose]:
    """The tool body's pose in tool0 per sample, under a calibration: constant if the calibration is right.

    Args:
        references: The reference body's pose per sample, in the world.
        tools: The tool body's pose per sample, in the world.
        joints: The arm's six joint angles per sample, radians, UR driver order.
        footprint: The calibration: base_footprint in the reference frame.
        urdf: The robot's URDF.
        arm: The arm, e.g. "ur_arm".
    """
    offsets = []
    for reference, tool, angles in zip(references, tools, joints):
        values: Mapping[str, float] = {f"{arm}_{name}": float(a) for name, a in zip(UR_JOINT_NAMES, angles)}
        tool0 = link_pose(urdf, compose(reference, footprint), values, f"{arm}_tool0")
        offsets.append(compose(invert(tool0), tool))
    return offsets


def offset_spread(offsets: Sequence[Pose]) -> tuple[np.ndarray, np.ndarray]:
    """How far each tool offset is from their mean: the validation's error per sample.

    Returns:
        tuple: (n,) position distance from the mean position, metres; (n, 3) each of the offset's X, Y and Z axes'
        angle to the mean of that axis, degrees (the old verification's measure).
    """
    positions = np.array([offset.position for offset in offsets])
    distances = np.linalg.norm(positions - positions.mean(axis=0), axis=1)
    rotations = Rotation.from_quat([offset.orientation for offset in offsets]).as_matrix()
    angles = np.empty((len(offsets), 3))
    for axis in range(3):
        columns = rotations[:, :, axis]
        mean = _unit(columns.mean(axis=0))
        angles[:, axis] = np.degrees(np.arccos(np.clip(columns @ mean, -1.0, 1.0)))
    return distances, angles
