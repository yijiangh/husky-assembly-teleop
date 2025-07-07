from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import RobotCellLibrary
import pybullet_planning as pp

# Export robot_cell and robot_cell_state to JSON files in the current folder using compas' json_dump
from compas import json_dump
# Load robot_cell and robot_cell_state from JSON files
from compas import json_load

import os
HERE = os.path.dirname(__file__)

# Starting the PyBulletClient with the "direct" mode means that the GUI is not shown
with PyBulletClient("gui") as client:

    # Load robot cell from library with a gripper and a beam
    robot_cell, robot_cell_state = RobotCellLibrary.ur5_gripper_one_beam()

    # The planner is used for passing the robot cell into the PyBullet client
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)

    # Save robot_cell
    robot_cell_path = os.path.join(HERE, "robot_cell.json")
    robot_cell_state_path = os.path.join(HERE, "robot_cell_state.json")
    json_dump(robot_cell, robot_cell_path)
    json_dump(robot_cell_state, robot_cell_state_path)

    # pp.wait_if_gui("Created robot_cell.json and robot_cell_state.json")


with PyBulletClient("gui") as client2:
    robot_cell_path = os.path.join(HERE, "robot_cell.json")
    robot_cell_state_path = os.path.join(HERE, "robot_cell_state.json")
    loaded_robot_cell = json_load(robot_cell_path)
    loaded_robot_cell_state = json_load(robot_cell_state_path)

    planner2 = PyBulletPlanner(client2)
    planner2.set_robot_cell(loaded_robot_cell)

    # Optionally, do something with the loaded robot cell, e.g., print confirmation
    print("Loaded robot cell and robot cell state from JSON files.")

    pp.wait_if_gui("Loaded robot cell and robot cell state from JSON files.")