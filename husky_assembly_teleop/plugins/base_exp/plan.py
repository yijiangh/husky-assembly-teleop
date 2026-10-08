"""A test path ready to send: template poses placed at the robot, timed, and cut into the follower's pieces."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from crl_husky.follower_path import FollowerPath

from .templates import TEMPLATES, place, timing


@dataclass(eq=False)
class Plan:
    """A path in the world, ready to send.

    Attributes:
        serial: The robot it was made for.
        poses: World poses (n x 3), unwrapped yaw.
        t: Times per pose for a timed run, or None.
        path: The pieces the follower will make of it, for drawing.
        expected: Seconds the run should take.
        label: Template, numbers and mode, for people.
        slug: Short template name, for file names.
        settings: What it was made from, as `plan_from` takes it; saved with the run.
    """

    serial: str
    poses: np.ndarray
    t: np.ndarray | None
    path: FollowerPath
    expected: float
    label: str
    slug: str
    settings: dict

    @property
    def speed_caps(self) -> tuple[float, float]:
        """Linear (m/s) and angular (rad/s) caps sent with the path; none for a timed one, so it can catch up."""
        if self.t is not None:
            return 0.0, 0.0
        return self.settings["linear_speed"], self.settings["turn_rate"]


def plan_from(serial: str, settings: dict, anchor) -> tuple[Plan | None, str]:
    """Build a plan starting at `anchor`, or say why not.

    Args:
        serial: The robot.
        settings: name (a TEMPLATES key), parameters (angles in degrees), mode ("Geometric" or "Timed"),
            linear_speed (m/s) and turn_rate (rad/s); as the panel and `auto.cells` give them.
        anchor: The robot's floor pose (x, y, yaw).

    Returns:
        tuple[Plan | None, str]: The plan and "", or None and the reason.
    """
    template, given = settings["name"], settings["parameters"]
    numbers = {k: math.radians(v) if k == "angle" else v for k, v in given.items()}
    timed = settings["mode"] == "Timed"
    try:
        poses = place(TEMPLATES[template].make(numbers), anchor)
        t = timing(poses, settings["linear_speed"], settings["turn_rate"])
        path = FollowerPath.from_poses(*poses.T, t if timed else None)
    except (ValueError, ZeroDivisionError) as error:
        return None, f"bad template values: {error}"
    label = f"{template} " + " ".join(f"{k}={v:g}" for k, v in given.items()) + f" {settings['mode']}"
    slug = "".join(c if c.isalnum() else "_" for c in template.lower())
    return Plan(serial, poses, t if timed else None, path, float(t[-1]), label, slug, settings), ""
