"""
One calibration sample: the base and tool bodies and the arm's joints, averaged once the arm stands still.

- `Sampler` collects one reading per tick; `still` says whether the tool stands still on the base, `sample` averages.
- The relay publishes the base calibrated; `Sample.record` rebuilds the base rigid body's own pose with the
  calibration applied at that moment, so take files keep the old pipeline's format (`session_files.py`).

Plain numpy; any thread.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ..storage.session_files import pose_list

#: Ticks of readings averaged into one sample (1 s at 20 Hz).
SAMPLE_TICKS = 20
#: Ticks the tool must stand still on the base before sampling, and how still: spread of its position, metres.
STILL_TICKS = 10
STILL_SPREAD = 2e-4


@dataclass(frozen=True)
class Reading:
    """One tick's measurement.

    Attributes:
        footprint: The base's pose from the relay: base_footprint under the applied calibration, in the world.
        tool: The tool body's pose in the world.
        joints: The arm's six joint angles, radians.
        base_error: The base body's marker error, metres.
        tool_error: The tool body's marker error, metres.
    """

    footprint: Pose
    tool: Pose
    joints: np.ndarray
    base_error: float
    tool_error: float


@dataclass(frozen=True)
class Sample:
    """Readings averaged: one waypoint of a take, or one validation pose.

    Attributes:
        footprint: Mean published base pose (base_footprint under `calibration`).
        tool: Mean tool body pose.
        joints: Mean joint angles, radians.
        base_error: Mean base marker error, metres.
        tool_error: Mean tool marker error, metres.
        spread: Largest distance of a reading's tool position on the base from their mean, metres.
        calibration: base_footprint in the base rigid body, as the relay applied it.
        calibration_sha: Its hash, from the relay.
        time: ROS time of the last reading, seconds.
    """

    footprint: Pose
    tool: Pose
    joints: np.ndarray
    base_error: float
    tool_error: float
    spread: float
    calibration: Pose
    calibration_sha: str
    time: float

    @property
    def base(self) -> Pose:
        """Pose: The base rigid body's own pose, the calibration taken back off."""
        return compose(self.footprint, invert(self.calibration))

    def record(self) -> dict:
        """Return the sample as the old pipeline's take entry, with this pipeline's extras."""
        return {"joint_conf": self.joints.tolist(), "base_mocap_pose": pose_list(self.base),
                "flange_mocap_pose": pose_list(self.tool), "base_footprint_pose": pose_list(self.footprint),
                "applied_calibration": {"sha256": self.calibration_sha, "pose": pose_list(self.calibration)},
                "marker_errors": {"base": self.base_error, "tool": self.tool_error}, "spread": self.spread,
                "time": self.time}


def mean_pose(poses: list[Pose]) -> Pose:
    """The mean of poses: mean position, and the rotation mean of the orientations."""
    return Pose.from_arrays(np.mean([pose.position for pose in poses], axis=0),
                            Rotation.from_quat([pose.orientation for pose in poses]).mean().as_quat())


@dataclass
class Sampler:
    """Readings of one waypoint, newest last."""

    readings: list[Reading] = field(default_factory=list)

    def add(self, reading: Reading) -> None:
        """Keep a reading."""
        self.readings.append(reading)

    def clear(self) -> None:
        """Forget every reading, e.g. after a mocap dropout."""
        self.readings.clear()

    def _tool_on_base(self, readings: list[Reading]) -> np.ndarray:
        return np.array([compose(invert(r.footprint), r.tool).position for r in readings])

    def spread(self, count: int) -> float | None:
        """Largest distance of the last `count` tool positions on the base from their mean, metres; None if fewer."""
        if len(self.readings) < count:
            return None
        points = self._tool_on_base(self.readings[-count:])
        return float(np.linalg.norm(points - points.mean(axis=0), axis=1).max())

    def still(self) -> bool:
        """Whether the tool stood still on the base for the last STILL_TICKS readings."""
        spread = self.spread(STILL_TICKS)
        return spread is not None and spread <= STILL_SPREAD

    def sample(self, calibration: Pose, calibration_sha: str, time: float) -> Sample:
        """Average the last SAMPLE_TICKS readings (fewer if there are fewer).

        Args:
            calibration: The calibration the relay applied to the base during the readings.
            calibration_sha: Its hash.
            time: ROS time now, seconds.

        Raises:
            ValueError: Without readings.
        """
        readings = self.readings[-SAMPLE_TICKS:]
        if not readings:
            raise ValueError("no readings to average")
        return Sample(mean_pose([r.footprint for r in readings]), mean_pose([r.tool for r in readings]),
                      np.mean([r.joints for r in readings], axis=0), float(np.mean([r.base_error for r in readings])),
                      float(np.mean([r.tool_error for r in readings])), self.spread(len(readings)) or 0.0,
                      calibration, calibration_sha, time)
