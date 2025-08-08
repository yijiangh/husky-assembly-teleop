from compas.geometry import Frame
import random
import time

from compas_fab.backends import PyBulletClient
from compas_fab.backends import PyBulletPlanner
from compas_fab.robots import FrameTarget
from compas_fab.robots import RobotCellLibrary
from compas_fab.robots import TargetMode
from compas_fab.backends import CollisionCheckError
from compas_fab.backends.exceptions import InverseKinematicsError
from compas_rhino.conversions import plane_to_compas_frame
import pybullet_planning as pp
import pybullet as p

from compas.geometry import Frame, Transformation

def pose_from_frame(frame, scale=1.0):
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)

def frame_from_pose(pose, scale=1.0):
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])

# Maximum number of sampling attempts
MAX_SAMPLING_ATTEMPTS = 100

new_robot_cell_state = None

if trigger:
    print(planner.client._get_pose_joint_names_and_puids())

    new_robot_cell_state = robot_cell_state.copy()
    
    # Step 0: Compute transformation between the two tool frames
    # Convert Grasshopper planes to COMPAS frames
    A_from_point_tool_tf = Transformation.from_frame(plane_to_compas_frame(A_from_point_tool))
    A_from_board_tool_tf = Transformation.from_frame(plane_to_compas_frame(A_from_board_tool))
    
    # Compute the transformation from point_tool to board_tool
    # point_tool_from_board_tool = A_from_point_tool.inverse() * A_from_board_tool
    point_tool_from_board_tool = A_from_point_tool_tf.inverse() * A_from_board_tool_tf
    
    print("Transformation from point_tool to board_tool:")
    print(point_tool_from_board_tool)
    
    # Main sampling loop - combines left arm sampling and right arm IK
    sampling_attempts = 0
    success = False
    
    while sampling_attempts < MAX_SAMPLING_ATTEMPTS and not success:
        sampling_attempts += 1
        print(f"Sampling attempt {sampling_attempts}")
        
        # Step 1: Sample left arm configuration randomly
        left_arm_config = planner.client.robot_cell.random_configuration(LEFT_ARM_GROUP)
        
        # Update the robot cell state with the random left arm configuration
        left_arm_joint_names = planner.client.robot_cell.get_configurable_joint_names(LEFT_ARM_GROUP)
        for joint_name, joint_value in zip(left_arm_config.joint_names, left_arm_config.joint_values):
            new_robot_cell_state.robot_configuration[joint_name] = joint_value
        
        # Check if the left arm configuration is in collision
        try:
            planner.check_collision(new_robot_cell_state)
        except CollisionCheckError as e:
            print(f"Left arm configuration {sampling_attempts} in collision: {e}")
            continue
        
        print(f"Found collision-free left arm configuration after {sampling_attempts} attempts")
        
        # Step 2: Get the attached tool TCF in World coordinate frame using TargetMode.TOOL
        world_from_point_tool = planner.forward_kinematics(new_robot_cell_state, TargetMode.TOOL, group=LEFT_ARM_GROUP)
        print("Point tool TCF frame in World coordinate frame:")
        print(world_from_point_tool)
        pp.draw_pose(pose_from_frame(world_from_point_tool))

        # Transform to get the board tool frame in world coordinates
        # world_from_board_tool = world_from_point_tool * point_tool_from_board_tool
        world_from_board_tool = Transformation.from_frame(world_from_point_tool) * point_tool_from_board_tool
        
        print("Board tool TCF frame in World coordinate frame:")
        print(world_from_board_tool)
        pp.draw_pose(pose_from_frame(Frame.from_transformation(world_from_board_tool)))

        # Step 3: Solve IK for the right arm using the transformed TCF frame as target
        target = FrameTarget(Frame.from_transformation(world_from_board_tool), TargetMode.TOOL)
        
        options = {"max_results": 20, "check_collision": check_collision}
        
        try:
            right_arm_config = planner.inverse_kinematics(
                target, 
                new_robot_cell_state, 
                group=RIGHT_ARM_GROUP, 
                options=options
            )
        
            if right_arm_config is not None:
                # Update the right arm joints in the robot cell state
                right_arm_joint_names = planner.client.robot_cell.get_configurable_joint_names(RIGHT_ARM_GROUP)
                for joint_name, joint_value in zip(right_arm_config.joint_names, right_arm_config.joint_values):
                    if joint_name in right_arm_joint_names:
                        new_robot_cell_state.robot_configuration[joint_name] = joint_value
                
                # Set the final configuration to the planner
                planner.client._set_robot_configuration(new_robot_cell_state.robot_configuration)
                print("Successfully found IK solution for both arms")
                print("Final configuration:", new_robot_cell_state.robot_configuration)
                success = True
        except InverseKinematicsError as e:
            print(f"No IK solution found for right arm in attempt {sampling_attempts}, {e} \nresampling left arm...")
            continue
    
    if not success:
        print(f"Failed to find valid configuration pair after {MAX_SAMPLING_ATTEMPTS} attempts")



