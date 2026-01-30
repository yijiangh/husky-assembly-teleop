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
from config_loader import load_config, get_robot_urdf, get_arm_base_link_name

# On Windows, PyBullet's connect(GUI) can invalidate stdout/stderr handles.
# Save and restore them around the import and connect calls.
_saved_stdout = sys.stdout
_saved_stderr = sys.stderr

import pybullet as p
import pybullet_planning as pp

sys.stdout = _saved_stdout
sys.stderr = _saved_stderr


def visualize_all_link_poses(robot, robot_name, draw_length=0.1):
    """Draw pose axes and name labels for selected links.
    For arms, only draw the arm base link."""
    num_links = pp.get_num_links(robot)
    print(f"Robot has {num_links} links")

    # Get the arm base link names for this robot
    left_arm_base = get_arm_base_link_name(robot_name, arm='left')
    right_arm_base = get_arm_base_link_name(robot_name, arm='right')
    arm_base_links = {left_arm_base, right_arm_base}
    print(f"Arm base links to display: {arm_base_links}")

    for link_id in range(num_links):
        link_name = pp.get_link_name(robot, link_id)
        if not link_name:
            continue

        # Filter arm links: only draw arm base links
        link_name_lower = link_name.lower()
        if 'arm' in link_name_lower or 'ur5' in link_name_lower or 'manipulator' in link_name_lower:
            # Skip all arm-related links except the arm base links
            if link_name not in arm_base_links:
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


def create_joint_sliders(robot):
    """Create debug sliders for each movable joint in the robot."""
    sliders = {}
    num_joints = p.getNumJoints(robot, physicsClientId=pp.CLIENT)

    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot, joint_idx, physicsClientId=pp.CLIENT)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]

        # Only create sliders for revolute (type 0) and prismatic (type 1) joints
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            # Use reasonable defaults if limits are not defined
            if lower_limit >= upper_limit:
                if joint_type == p.JOINT_REVOLUTE:
                    lower_limit = -np.pi
                    upper_limit = np.pi
                else:
                    lower_limit = -1.0
                    upper_limit = 1.0

            # Create slider
            slider_id = p.addUserDebugParameter(
                joint_name,
                lower_limit,
                upper_limit,
                0.0,  # Start at 0
                physicsClientId=pp.CLIENT
            )
            sliders[joint_idx] = slider_id
            print(f"Created slider for joint {joint_idx}: {joint_name} (type={joint_type}, range=[{lower_limit:.2f}, {upper_limit:.2f}])")

    return sliders


def draw_joint_axes(robot, axis_length=0.15):
    """Draw the axis of rotation/translation for each revolute and prismatic joint."""
    num_joints = p.getNumJoints(robot, physicsClientId=pp.CLIENT)
    axis_lines = []

    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot, joint_idx, physicsClientId=pp.CLIENT)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        joint_axis = joint_info[13]  # Joint axis in joint frame

        # Only draw for revolute and prismatic joints
        if joint_type not in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            continue

        # Get the link state for the child link (joint_idx corresponds to child link index)
        # The joint frame is at the child link's origin
        link_state = p.getLinkState(robot, joint_idx, computeForwardKinematics=True, physicsClientId=pp.CLIENT)

        # link_state[4] and [5] give us the world position and orientation of the link frame
        # which corresponds to the joint frame
        joint_pos_world = link_state[4]
        joint_orn_world = link_state[5]

        # Transform joint axis to world frame
        rotation_matrix = np.array(p.getMatrixFromQuaternion(joint_orn_world)).reshape(3, 3)
        joint_axis_world = rotation_matrix @ np.array(joint_axis)
        joint_axis_world = joint_axis_world / (np.linalg.norm(joint_axis_world) + 1e-9)  # Normalize

        # Calculate end point of axis vector
        axis_end = [
            joint_pos_world[0] + joint_axis_world[0] * axis_length,
            joint_pos_world[1] + joint_axis_world[1] * axis_length,
            joint_pos_world[2] + joint_axis_world[2] * axis_length
        ]

        # Color: Red for revolute, Blue for prismatic
        if joint_type == p.JOINT_REVOLUTE:
            color = [1, 0, 0]  # Red for revolute
        else:
            color = [0, 0, 1]  # Blue for prismatic

        # Draw the axis as a line
        line_id = p.addUserDebugLine(
            joint_pos_world,
            axis_end,
            lineColorRGB=color,
            lineWidth=3,
            physicsClientId=pp.CLIENT
        )
        axis_lines.append(line_id)

        print(f"Drew axis for joint {joint_idx}: {joint_name} (type={'REVOLUTE' if joint_type == p.JOINT_REVOLUTE else 'PRISMATIC'})")

    return axis_lines


def update_joint_axes(robot, axis_lines, axis_length=0.15):
    """Update the drawn joint axes to reflect current robot configuration.
    Returns new axis_lines list."""
    # Remove all old axis lines
    for line_id in axis_lines:
        p.removeUserDebugItem(line_id, physicsClientId=pp.CLIENT)

    # Draw new axis lines with updated positions
    new_axis_lines = []
    num_joints = p.getNumJoints(robot, physicsClientId=pp.CLIENT)

    for joint_idx in range(num_joints):
        joint_info = p.getJointInfo(robot, joint_idx, physicsClientId=pp.CLIENT)
        joint_type = joint_info[2]
        joint_axis = joint_info[13]

        if joint_type not in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            continue

        # Get the current link state for the child link
        link_state = p.getLinkState(robot, joint_idx, computeForwardKinematics=True, physicsClientId=pp.CLIENT)
        joint_pos_world = link_state[4]
        joint_orn_world = link_state[5]

        rotation_matrix = np.array(p.getMatrixFromQuaternion(joint_orn_world)).reshape(3, 3)
        joint_axis_world = rotation_matrix @ np.array(joint_axis)
        joint_axis_world = joint_axis_world / (np.linalg.norm(joint_axis_world) + 1e-9)

        axis_end = [
            joint_pos_world[0] + joint_axis_world[0] * axis_length,
            joint_pos_world[1] + joint_axis_world[1] * axis_length,
            joint_pos_world[2] + joint_axis_world[2] * axis_length
        ]

        if joint_type == p.JOINT_REVOLUTE:
            color = [1, 0, 0]
        else:
            color = [0, 0, 1]

        # Draw new line
        line_id = p.addUserDebugLine(
            joint_pos_world,
            axis_end,
            lineColorRGB=color,
            lineWidth=3,
            physicsClientId=pp.CLIENT
        )
        new_axis_lines.append(line_id)

    return new_axis_lines


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

    # Enable debug visualizer with GUI controls
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
    p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1, physicsClientId=pp.CLIENT)

    # Load robot
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

    # Make transparent
    set_robot_transparent(robot, rgba=(1, 1, 1, 0.3))

    # Draw world origin
    pp.draw_pose(pp.Pose(), length=0.3)
    pp.add_text("world_origin", position=[0.05, 0.05, 0.05])

    # Draw all link poses
    visualize_all_link_poses(robot, robot_name, draw_length=0.1)

    # Create sliders for each joint
    print("\n--- Creating joint sliders ---")
    sliders = create_joint_sliders(robot)

    # Draw joint axes
    print("\n--- Drawing joint axes ---")
    axis_lines = draw_joint_axes(robot, axis_length=0.15)

    print("\n--- Interactive mode ---")
    print("Use the sliders in the PyBullet GUI to control each joint.")
    print("Red lines = revolute joint axes")
    print("Blue lines = prismatic joint axes")
    print("Press Ctrl+C to exit.")

    # Interactive loop
    try:
        while True:
            # Update joint positions based on slider values
            for joint_idx, slider_id in sliders.items():
                slider_value = p.readUserDebugParameter(slider_id, physicsClientId=pp.CLIENT)
                p.resetJointState(robot, joint_idx, slider_value, physicsClientId=pp.CLIENT)

            # Update joint axis visualizations (remove old lines and draw new ones)
            axis_lines = update_joint_axes(robot, axis_lines, axis_length=0.15)

            # Small delay to avoid hogging CPU
            p.stepSimulation(physicsClientId=pp.CLIENT)

    except KeyboardInterrupt:
        print("\nExiting...")


if __name__ == "__main__":
    main()
