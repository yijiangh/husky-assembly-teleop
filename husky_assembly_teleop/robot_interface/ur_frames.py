"""The UR frame convention the arms' Cartesian commands rely on (see doc/ur_frames.md); the URDF check is the core's."""

from __future__ import annotations

from scipy.spatial.transform import Rotation

from bar_assembly_core.urdf import STOCK_YAW, stock_frame_problem

__all__ = ["BASE_LINK_FROM_UR_BASE", "STOCK_YAW", "stock_frame_problem"]

#: The compliance controller's `base_link` seen from the UR Base frame (the reported TCP's frame).
#: ! Fixed by the stock description; never read it from our URDFs.
BASE_LINK_FROM_UR_BASE = Rotation.from_euler("z", STOCK_YAW)
