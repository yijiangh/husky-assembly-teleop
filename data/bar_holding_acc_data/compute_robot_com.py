import json, os
import pybullet_planning as pp
import numpy as np
import pybullet as p

EXPORT = 1
viewer = 1
new_data = []

HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

UR5E_LINK_NAMES = ["ur_arm_base_link_inertia",
                   "ur_arm_shoulder_link",
                   "ur_arm_upper_arm_link",
                   "ur_arm_forearm_link",
                   "ur_arm_wrist_1_link",
                   "ur_arm_wrist_2_link",
                   "ur_arm_wrist_3_link"]

HUSKY_WHEEL_LINK_NAMES = ["front_left_wheel_link",
                         "front_right_wheel_link",
                         "rear_left_wheel_link",
                         "rear_right_wheel_link"]

def compute_ur5e_com(robot_id):
    """
    Compute the center of mass for the UR5e robot arm.
    
    Args:
        robot_id: PyBullet body ID for the robot
        
    Returns:
        center_of_mass: [x, y, z] coordinates of the center of mass in world frame
    """
    total_mass = 0.0
    weighted_pos = np.zeros(3)
    
    # Get link indices for the UR5e arm links
    link_indices = []
    for link_name in UR5E_LINK_NAMES:
        for i in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, i)
            if joint_info[12].decode('utf-8') == link_name:
                link_indices.append(i)
                break
    
    # Calculate CoM
    for link_idx in link_indices:
        # Get dynamics info for the link
        dynamics_info = p.getDynamicsInfo(robot_id, link_idx)
        link_mass = dynamics_info[0]
        
        if link_mass > 0:
            # Get link state (position)
            link_state = p.getLinkState(robot_id, link_idx)
            link_com_pos = np.array(link_state[0])  # CoM position in world frame
            
            # Add weighted contribution to overall CoM
            weighted_pos += link_mass * link_com_pos
            total_mass += link_mass
    
    # Compute final CoM
    if total_mass > 0:
        center_of_mass = weighted_pos / total_mass
    else:
        center_of_mass = np.zeros(3)
        print("Warning: Total mass is zero, returning origin as CoM")
    
    return center_of_mass

def compute_robot_com(robot_id):
    """
    Compute the center of mass for the entire Husky robot including base and arm.
    
    Args:
        robot_id: PyBullet body ID for the robot
        
    Returns:
        center_of_mass: [x, y, z] coordinates of the center of mass in world frame
    """
    total_mass = 0.0
    weighted_pos = np.zeros(3)
    
    # Base link mass and CoM
    # PyBullet considers the base as link index -1
    base_pos, base_orientation = p.getBasePositionAndOrientation(robot_id)
    base_mass = p.getDynamicsInfo(robot_id, -1)[0]
    
    if base_mass > 0:
        weighted_pos += base_mass * np.array(base_pos)
        total_mass += base_mass
    
    # Process all other links
    for link_idx in range(p.getNumJoints(robot_id)):
        # Get dynamics info for the link
        dynamics_info = p.getDynamicsInfo(robot_id, link_idx)
        link_mass = dynamics_info[0]
        
        if link_mass > 0:
            # Get link state (position)
            link_state = p.getLinkState(robot_id, link_idx)
            link_com_pos = np.array(link_state[0])  # CoM position in world frame
            
            # Add weighted contribution to overall CoM
            weighted_pos += link_mass * link_com_pos
            total_mass += link_mass
    
    # Compute final CoM
    if total_mass > 0:
        center_of_mass = weighted_pos / total_mass
    else:
        center_of_mass = np.zeros(3)
        print("Warning: Total mass is zero, returning origin as CoM")
    
    return center_of_mass

def get_wheel_contact_points(robot_id):
    """
    Get the contact points of the Husky's four wheels with the ground.
    
    Args:
        robot_id: PyBullet body ID for the robot
        
    Returns:
        contact_points: List of [x, y, z] coordinates of wheel contact points
    """
    # Get wheel link indices
    wheel_indices = []
    for wheel_name in HUSKY_WHEEL_LINK_NAMES:
        for i in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, i)
            if joint_info[12].decode('utf-8') == wheel_name:
                wheel_indices.append(i)
                break
    
    # Check if we found all four wheels
    if len(wheel_indices) != 4:
        print(f"Warning: Found {len(wheel_indices)} wheels instead of 4")
    
    contact_points = []
    
    # Get all contact points between the robot and the ground (plane)
    # The ground is typically body ID 0
    ground_plane_id = 0
    
    for wheel_idx in wheel_indices:
        # Get contact points for this wheel and the ground
        contact_points_wheel = p.getContactPoints(robot_id, ground_plane_id, wheel_idx)
        
        if contact_points_wheel:
            # There might be multiple contact points per wheel
            # For simplicity, we'll use the average position
            avg_point = np.zeros(3)
            count = 0
            
            for contact in contact_points_wheel:
                # contact[6] is the position on body B (ground) in world coordinates
                avg_point += np.array(contact[6])
                count += 1
            
            if count > 0:
                avg_point /= count
                contact_points.append(avg_point)
        else:
            # If no contact is found, use the wheel position as an approximation
            # and project it onto the ground (z=0)
            wheel_state = p.getLinkState(robot_id, wheel_idx)
            wheel_pos = np.array(wheel_state[0])
            # Project onto ground (assuming ground is at z=0)
            wheel_pos[2] = 0
            contact_points.append(wheel_pos)
    
    return contact_points

def compute_support_polygon_center(contact_points):
    """
    Compute the center of the support polygon formed by the wheel contact points.
    
    Args:
        contact_points: List of [x, y, z] coordinates of wheel contact points
        
    Returns:
        center: [x, y, z] coordinates of the support polygon center
    """
    if not contact_points:
        return np.zeros(3)
    
    # Simple arithmetic mean of the contact points
    center = np.mean(contact_points, axis=0)
    return center

def compute_projected_distance(point_a, point_b):
    """
    Compute the projected distance between two points on the ground plane (XY plane).
    
    Args:
        point_a: [x, y, z] coordinates of the first point
        point_b: [x, y, z] coordinates of the second point
        
    Returns:
        distance: Euclidean distance between projections of points on XY plane
    """
    # Project points onto XY plane by taking only the x and y coordinates
    projected_a = np.array([point_a[0], point_a[1]])
    projected_b = np.array([point_b[0], point_b[1]])
    
    # Compute Euclidean distance
    distance = np.linalg.norm(projected_a - projected_b)
    return distance

def draw_support_polygon(contact_points, height=0.01, color=(0, 1, 0, 0.7)):
    """
    Draw the support polygon formed by the wheel contact points.
    
    Args:
        contact_points: List of wheel contact points
        height: Height of the polygon above the ground
        color: RGBA color for the polygon
    
    Returns:
        visual_id: PyBullet visual shape ID
    """
    if len(contact_points) < 3:
        print("Warning: At least 3 contact points needed to form a polygon")
        return None
    
    # Sort points to form a convex hull (simplified approach)
    # For a quadrilateral robot like Husky, we can sort based on angle from centroid
    centroid = np.mean(contact_points, axis=0)
    
    def angle_from_centroid(point):
        return np.arctan2(point[1] - centroid[1], point[0] - centroid[0])
    
    sorted_points = sorted(contact_points, key=angle_from_centroid)
    
    # Slightly elevate the points for visibility
    elevated_points = [np.array([p[0], p[1], p[2] + height]) for p in sorted_points]
    
    # Create visual shapes for the polygon sides
    visual_ids = []
    
    # Draw lines connecting consecutive points
    for i in range(len(elevated_points)):
        start_point = elevated_points[i]
        end_point = elevated_points[(i + 1) % len(elevated_points)]
        
        line_id = p.addUserDebugLine(
            start_point,
            end_point,
            lineColorRGB=color[:3],
            lineWidth=3.0
        )
        visual_ids.append(line_id)
    
    # Create a single color for the support polygon (semi-transparent)
    # PyBullet doesn't have a direct way to create a filled polygon
    # We approximate using a mesh visual shape
    vertices = []
    indices = []
    
    # Add the centroid
    vertices.append(np.array([centroid[0], centroid[1], centroid[2] + height]))
    
    # Add the perimeter points
    for point in elevated_points:
        vertices.append(point)
    
    # Create triangles from centroid to each edge
    for i in range(len(elevated_points)):
        indices.extend([0, i+1, (i+1) % len(elevated_points) + 1])
    
    # Create the visual shape
    mesh_visual = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        vertices=vertices,
        indices=indices,
        rgbaColor=color,
        physicsClientId=0
    )
    
    # Create a multi-body for the mesh
    polygon_id = p.createMultiBody(
        baseVisualShapeIndex=mesh_visual,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 1]
    )
    
    return polygon_id

data_batch = ''
HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, data_batch)

file_path = os.path.join(data_folder, 'analysis_bar_holding_acc_.json')

# Load the JSON file
with open(file_path, 'r') as file:
    data = json.load(file)

pp.connect(use_gui=viewer, shadows=True, color=[0.9, 0.9, 1.0])
robot_urdf = os.path.join(r'D:\0_Project\03-2025_husky_assembly\Code\husky-asembly-teleop\data',r'husky_urdf\mt_husky_moveit_config\urdf\husky_ur5_e_no_base_joint.urdf')

# Create a ground plane for contact detection
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

with pp.HideOutput():
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

polygon_id = None  # Track support polygon visualization

for entry in data:
    footprint_pose = entry.get("footprint_pose", [])
    conf = entry.get("joint_conf", [])

    pp.set_pose(robot, footprint_pose)

    arm_joints = pp.joints_from_names(robot, HUSKY_UR5e_JOINT_NAMES)
    pp.set_joint_positions(robot, arm_joints, conf)

    # Compute center of mass for just the UR5e arm
    arm_com = compute_ur5e_com(robot)
    print(f"UR5e Arm Center of Mass: {arm_com}")
    
    # Compute center of mass for the entire robot
    robot_com = compute_robot_com(robot)
    print(f"Entire Robot Center of Mass: {robot_com}")
    
    # Get wheel contact points
    contact_points = get_wheel_contact_points(robot)
    print(f"Wheel contact points: {contact_points}")
    
    # Sort contact points to form a consistent polygon (for export)
    center = compute_support_polygon_center(contact_points)
    
    def angle_from_center(point):
        return np.arctan2(point[1] - center[1], point[0] - center[0])
    
    sorted_contact_points = sorted(contact_points, key=angle_from_center)
    
    # Compute the center of the support polygon
    support_polygon_center = compute_support_polygon_center(contact_points)
    print(f"Support Polygon Center: {support_polygon_center}")
    
    # Compute projected distance between robot COM and support polygon center
    distance_com_to_polygon = compute_projected_distance(robot_com, support_polygon_center)
    print(f"Projected Distance from Robot COM to Support Polygon Center: {distance_com_to_polygon} m")
    
    # Export data if requested
    if EXPORT:
        # Create a copy of the current entry to preserve original data
        new_entry = entry.copy()
        
        # Add new data
        new_entry["ur5e_com"] = arm_com.tolist()
        new_entry["robot_com"] = robot_com.tolist()
        new_entry["support_polygon_vertices"] = [point.tolist() for point in sorted_contact_points]
        new_entry["support_polygon_center"] = support_polygon_center.tolist()
        new_entry["distance_com_to_polygon_center"] = float(distance_com_to_polygon)
        
        new_data.append(new_entry)
    
    # Remove previous polygon if it exists
    if polygon_id is not None:
        p.removeBody(polygon_id)
    
    # Draw new support polygon
    polygon_id = draw_support_polygon(contact_points)
    
    # Visualize support polygon center with a green sphere
    polygon_center_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 0.8, 0, 0.7])
    p.createMultiBody(baseVisualShapeIndex=polygon_center_visual, basePosition=support_polygon_center)
    
    # Visualize arm CoM with a small red sphere
    arm_com_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.7])
    p.createMultiBody(baseVisualShapeIndex=arm_com_visual, basePosition=arm_com)
    
    # Visualize entire robot CoM with a small blue sphere
    robot_com_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 0.7])
    p.createMultiBody(baseVisualShapeIndex=robot_com_visual, basePosition=robot_com)
    
    # Draw a line from robot COM to support polygon center to visualize distance
    p.addUserDebugLine(
        [robot_com[0], robot_com[1], 0.02],  # Slightly above ground
        [support_polygon_center[0], support_polygon_center[1], 0.02],
        lineColorRGB=[1, 0.5, 0],  # Orange
        lineWidth=4.0
    )
    
    # Visualize wheel contact points
    for point in contact_points:
        wheel_contact_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 1, 0, 0.7])
        p.createMultiBody(baseVisualShapeIndex=wheel_contact_visual, basePosition=point)

    pp.draw_pose(footprint_pose)
    # pp.wait_if_gui()

# Save the updated data if export is enabled
if EXPORT and new_data:
    print(f"Exporting updated data to {file_path}")
    with open(file_path, 'w') as file:
        json.dump(new_data, file, indent=2)
    print("Export completed successfully")

pp.set_pose(robot, pp.unit_pose())

pp.wait_if_gui()