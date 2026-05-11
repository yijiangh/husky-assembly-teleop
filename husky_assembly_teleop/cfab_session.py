"""Long-lived compas_fab PyBullet session for a single design-study problem.

Owns a `PyBulletClient` + `PyBulletPlanner` and the deserialized
`RobotCell`. Single source of truth for scene materialization on the
planning side — replaces ad-hoc URDF / tool / robot-cell loading that
previously lived in `common.py` and `design_interface/`.

Usage:

    s = CfabSession("2026-05-08_dual-arm_transfer_test")
    s.planner.set_robot_cell_state(some_state)
    s.planner.check_collision(some_state, {"full_report": True})
    ...
    s.close()
"""

from __future__ import annotations

import os

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner

# Importing rs_data_structure registers the legacy "core.bar_action" dtype
# alias so existing BarAction JSONs (and any compas object referencing the
# old dtype) deserialize correctly.
import rs_data_structure  # noqa: F401

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY


class CfabSession:
    """Per-problem cfab planner session.

    Materializes the entire RobotCell (robot URDF, tool URDFs, rigid body
    meshes) into the client's PyBullet world in one go via
    `planner.set_robot_cell`. Per-movement state is pushed in via
    `planner.set_robot_cell_state(state)`.
    """

    def __init__(self, problem_name: str, *,
                 connection_type: str = "direct",
                 enable_debug_gui: bool = False):
        self.problem_name = problem_name
        # ``enable_debug_gui`` toggles ``pybullet.COV_ENABLE_GUI``. Off by
        # default (matches compas_fab); set to True to get the sidebar +
        # debug-parameter sliders in the cfab GUI window.
        self.client = PyBulletClient(
            connection_type=connection_type, verbose=False,
            enable_debug_gui=enable_debug_gui,
        )
        self.client.__enter__()  # open the PyBullet connection
        try:
            self.planner = PyBulletPlanner(self.client)
            robot_cell_path = os.path.join(
                DESIGN_DATA_DIRECTORY, problem_name, "RobotCell.json"
            )
            robot_cell = json_load(robot_cell_path)
            self.planner.set_robot_cell(robot_cell)
            self.robot_cell = robot_cell
        except Exception:
            # If anything fails after the client is open, make sure we don't
            # leak the PyBullet connection.
            self.client.__exit__(None, None, None)
            self.client = None
            raise

    def close(self):
        if self.client is not None:
            self.client.__exit__(None, None, None)
            self.client = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False  # don't suppress exceptions
