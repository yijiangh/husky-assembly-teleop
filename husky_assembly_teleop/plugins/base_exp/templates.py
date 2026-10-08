"""
Test paths for the base: poses in the robot's start frame, placed in the world and timed. Plain math.

Poses are (n x 3) arrays of x, y in metres and unwrapped yaw in radians, as `BaseInterface.send_path` takes.
A negative length drives backwards; a negative angle turns clockwise.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

#: Spacing of the generated poses; the follower fits a curve through them.
STEP = 0.05                    # m
TURN_STEP = math.radians(5.0)  # rad


@dataclass(frozen=True)
class Parameter:
    """One number a template takes.

    Attributes:
        name: Key in the values passed to a template.
        label: What the operator sees, with the unit.
        default: Initial value.
        hint: Hover text.
        step: Increment of the number field.
    """

    name: str
    label: str
    default: float
    hint: str
    step: float = 0.05


PARAMETERS = (
    Parameter("length", "Length m", 1.0, "Straight distance; negative drives backwards. Sine: distance along x"),
    Parameter("length_2", "Length 2 m", 1.0, "Drive, turn, drive: the second straight"),
    Parameter("radius", "Radius m", 0.6, "Circle radius"),
    Parameter("angle", "Angle °", 90.0, "Turn or arc angle; negative turns clockwise", 5.0),
    Parameter("amplitude", "Amplitude m", 0.2, "Sine: the wave swings left by up to twice this"),
    Parameter("wavelength", "Wavelength m", 1.0, "Sine: distance per full wave"),
)


def straight(length: float) -> np.ndarray:
    """Drive `length` metres along the start heading; backwards if negative."""
    x = np.linspace(0.0, length, _count(abs(length), STEP))
    return np.column_stack([x, np.zeros_like(x), np.zeros_like(x)])


def turn(angle: float) -> np.ndarray:
    """Turn on the spot by `angle` radians; clockwise if negative."""
    yaw = np.linspace(0.0, angle, _count(abs(angle), TURN_STEP))
    return np.column_stack([np.zeros_like(yaw), np.zeros_like(yaw), yaw])


def arc(radius: float, angle: float) -> np.ndarray:
    """Drive forwards on a circle of `radius` through `angle` radians; to the right if negative."""
    side = 1.0 if angle >= 0.0 else -1.0
    phase = np.linspace(0.0, abs(angle), _count(abs(angle) * radius, STEP))
    return np.column_stack([radius * np.sin(phase), side * radius * (1.0 - np.cos(phase)), side * phase])


def sine(length: float, amplitude: float, wavelength: float) -> np.ndarray:
    """Drive forwards along a wave over `length` metres, swinging left by up to twice `amplitude`.

    The wave is amplitude * (1 - cos), so it leaves along the start heading.
    """
    x = np.linspace(0.0, length, _count(abs(length), STEP))
    k = 2.0 * math.pi / wavelength
    return np.column_stack([x, amplitude * (1.0 - np.cos(k * x)), np.arctan(amplitude * k * np.sin(k * x))])


def drive_turn_drive(length: float, angle: float, length_2: float) -> np.ndarray:
    """Drive `length`, turn on the spot by `angle`, then drive `length_2` along the new heading."""
    first = straight(length)
    turned = place(turn(angle), first[-1])
    second = place(straight(length_2), turned[-1])
    return np.vstack([first, turned[1:], second[1:]])


@dataclass(frozen=True)
class Template:
    """A named test path.

    Attributes:
        parameters: Names of the PARAMETERS it uses.
        make: Builds the poses from the values by name; angles in radians.
    """

    parameters: tuple[str, ...]
    make: Callable[[dict[str, float]], np.ndarray]


TEMPLATES = {
    "Straight": Template(("length",), lambda v: straight(v["length"])),
    "Turn on the spot": Template(("angle",), lambda v: turn(v["angle"])),
    "Arc": Template(("radius", "angle"), lambda v: arc(v["radius"], v["angle"])),
    "Sine": Template(("length", "amplitude", "wavelength"),
                     lambda v: sine(v["length"], v["amplitude"], v["wavelength"])),
    "Drive, turn, drive": Template(("length", "angle", "length_2"),
                                   lambda v: drive_turn_drive(v["length"], v["angle"], v["length_2"])),
}


def place(poses: np.ndarray, start) -> np.ndarray:
    """Move poses from the start frame into the world.

    Args:
        poses: Poses in the start frame (n x 3).
        start: The start pose (x, y, yaw) in the world.

    Returns:
        np.ndarray: World poses; yaw stays unwrapped, offset by the start yaw.
    """
    x0, y0, yaw0 = (float(v) for v in start)
    c, s = math.cos(yaw0), math.sin(yaw0)
    return np.column_stack([x0 + c * poses[:, 0] - s * poses[:, 1],
                            y0 + s * poses[:, 0] + c * poses[:, 1],
                            poses[:, 2] + yaw0])


def timing(poses: np.ndarray, linear_speed: float, angular_speed: float) -> np.ndarray:
    """Time of each pose when every step takes as long as its slower part, driving or turning.

    Args:
        poses: Poses (n x 3).
        linear_speed: m/s.
        angular_speed: rad/s.

    Returns:
        np.ndarray: Seconds from the first pose.
    """
    steps = np.diff(poses, axis=0)
    seconds = np.maximum(np.hypot(steps[:, 0], steps[:, 1]) / linear_speed, np.abs(steps[:, 2]) / angular_speed)
    return np.concatenate([[0.0], np.cumsum(seconds)])


def _count(span: float, step: float) -> int:
    """Poses to cover `span` in steps of at most `step`, both ends included."""
    return max(2, int(math.ceil(span / step)) + 1)
