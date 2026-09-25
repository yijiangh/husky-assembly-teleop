"""
The design: the elements and the order they go in.

! Deliberately thin, and staying that way until the port says otherwise.
  An earlier draft of this file computed the per-step layout -- which elements
  are present, staged or absent at index k -- as a pure projection of the design.
  That was wrong: the layout is not computed, it is authored. Every movement in a
  BarAction file carries its own compas_fab `RobotCellState` with frames,
  attachments, `is_hidden` and `touch_links` already in it.

  So this holds only what a design says independent of any movement, and the
  authored states are loaded straight from the file. If it turns out something
  really does need deriving, derive it then.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Position in metres and orientation as a quaternion, in world frame. Matches
# pybullet_planning's convention so it can be handed straight to `pp`.
Pose = tuple[np.ndarray, np.ndarray]


@dataclass(frozen=True)
class AssemblyElement:
    """One part the robots will place.

    Attributes:
        name: Unique identifier, matching the rigid-body name in the compas_fab
            robot cell so the design and the planner can be matched up.
        mesh_file: Geometry.
        final_pose: Where it belongs in the finished structure.
    """

    name: str
    mesh_file: Path
    final_pose: Pose


@dataclass(frozen=True)
class AssemblyStep:
    """One step of the sequence: the placing of one element.

    Attributes:
        step_id: Stable identifier, as authored.
        element_name: The element this step places.
    """

    step_id: str
    element_name: str

    # ! No allowed-contact list here.
    #   Contact exceptions are authored per movement, as touch_links and
    #   touch_bodies on the movement's RigidBodyState, and compas_fab consumes
    #   them directly. Copying them into a type of our own would mean translating
    #   in both directions for no gain -- and debugging already happens in
    #   compas_fab's vocabulary.


@dataclass(frozen=True)
class Assembly:
    """A whole design: the elements, and the order they are assembled in.

    Attributes:
        elements: Every element, in no particular order.
        steps: The sequence, in execution order.
    """

    elements: tuple[AssemblyElement, ...] = ()
    steps: tuple[AssemblyStep, ...] = ()

    def element(self, name: str) -> AssemblyElement:
        """Look up one element by name.

        Args:
            name: Element name.

        Returns:
            AssemblyElement: The element.

        Raises:
            KeyError: If no element has that name.
        """
        for element in self.elements:
            if element.name == name:
                return element
        raise KeyError(f"no element named {name!r} in this assembly")
