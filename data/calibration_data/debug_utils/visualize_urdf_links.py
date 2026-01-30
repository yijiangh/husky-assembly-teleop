#!/usr/bin/env python3
"""
Visualize all link poses of the dual-arm URDF in PyBullet.

Loads the robot URDF, draws coordinate frames at every link,
and labels each link with its name.
"""

import os
import sys
import numpy as np

# Add parent directory so we can import config_loader
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from config_loader import load_config, get_robot_urdf

# On Windows, PyBullet's connect(GUI) can invalidate stdout/stderr handles.
# Save and restore them around the import and connect calls.
_saved_stdout = sys.stdout
_saved_stderr = sys.stderr

import pybullet as p
import pybullet_planning as pp

sys.stdout = _saved_stdout
sys.stderr = _saved_stderr


def visualize_all_link_poses(robot, draw_length=0.1):
    """Draw pose axes and name labels for every link in the robot."""
    num_links = pp.get_num_links(robot)
    print(f"Robot has {num_links} links")

    for link_id in range(num_links):
        link_name = pp.get_link_name(robot, link_id)
        if not link_name:
            continue

        link_pose = pp.get_link_pose(robot, link_id)
        pp.draw_pose(link_pose, length=draw_length)

        # Small random offset so labels don't overlap
        pos = link_pose[0]
        offset = [np.random.uniform(-0.02, 0.02) for _ in range(2)] + [np.random.uniform(0.05, 0.07)]
        text_pos = [pos[i] + offset[i] for i in range(3)]
        pp.add_text(link_name, position=text_pos)


def set_robot_transparent(robot, rgba=(1, 1, 1, 0.3)):
    """Set all links to the given RGBA colour (default: white, 30% opacity)."""
    for link_id in range(pp.get_num_links(robot)):
        pp.set_color(robot, list(rgba), link=link_id)


def main():
    config = load_config()
    robot_name = config['robot_name']

    robot_urdf = get_robot_urdf(robot_name)
    print(f"Loading URDF: {robot_urdf}")

    # Initialise PyBullet
    # Save handles before connect — PyBullet GUI can invalidate them on Windows
    _so, _se = sys.stdout, sys.stderr
    pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
    sys.stdout, sys.stderr = _so, _se

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)

    # Load robot
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

    # Make transparent
    set_robot_transparent(robot, rgba=(1, 1, 1, 0.3))

    # Draw world origin
    pp.draw_pose(pp.Pose(), length=0.3)
    pp.add_text("world_origin", position=[0.05, 0.05, 0.05])

    # Draw all link poses
    visualize_all_link_poses(robot, draw_length=0.1)

    pp.wait_if_gui("All link poses visualised. Press Enter to exit.")


if __name__ == "__main__":
    main()
