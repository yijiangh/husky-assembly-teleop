"""
A UR arm's six joint sliders, in degrees at a 0.5° step: added, written and read the same way in every panel.

! Wire their callbacks with `ctx.defer_value(..., operator_only=True)`: writing a slider calls back too.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
import viser

from ..robot_interface.arm import UR_JOINT_LIMITS

#: Slider label per UR joint, in UR_JOINT_NAMES order.
JOINT_SLIDER_NAMES = ("pan", "lift", "elbow", "w1", "w2", "w3")
#: Slider step, degrees.
JOINT_SLIDER_STEP = 0.5


def add_joint_sliders(gui: viser.GuiApi) -> list[viser.GuiSliderHandle]:
    """Add the six joint sliders, at zero, each over its joint's range (`UR_JOINT_LIMITS`)."""
    return [gui.add_slider(f"{name} °", min=-np.degrees(limit), max=np.degrees(limit), step=JOINT_SLIDER_STEP,
                           initial_value=0.0)
            for name, limit in zip(JOINT_SLIDER_NAMES, UR_JOINT_LIMITS)]


def set_joint_sliders(sliders: Sequence[viser.GuiSliderHandle], angles: Sequence[float]) -> None:
    """Set the sliders to joint angles in radians, rounded to the sliders' step."""
    for slider, value in zip(sliders, np.degrees(angles)):
        slider.value = round(float(value) / JOINT_SLIDER_STEP) * JOINT_SLIDER_STEP


def joint_slider_angles(sliders: Sequence[viser.GuiSliderHandle]) -> np.ndarray:
    """The sliders' joint angles, radians."""
    return np.radians([slider.value for slider in sliders])
