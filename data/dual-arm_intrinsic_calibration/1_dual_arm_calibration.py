"""
Dual-Arm Intrinsic Calibration Script

This script computes the relative transformation between two robot arms using
TCP point data collected when both arms point to the same physical points.

Author: Based on original work by Jingwen Wang
"""

import json
import numpy as np
import os
import logging
from itertools import combinations
from scipy.optimize import minimize
from compas.geometry import (
    Point, Frame, Transformation, Translation, Rotation, Vector
)
from compas.geometry import matrix_from_frame_to_frame as frame_to_frame_matrix
import copy

# Set precision for COMPAS
import compas
compas.PRECISION = '12f'

# Define HERE constant for file paths
HERE = os.path.dirname(os.path.abspath(__file__))


def load_calibration_data(json_file_path):
    """
    Load calibration data from JSON file.
    
    Args:
        json_file_path (str): Path to the JSON file containing calibration data
        
    Returns:
        tuple: (left_arm_points, right_arm_points) where each is a list of Point objects
    """
    with open(json_file_path, 'r') as f:
        data = json.load(f)
    
    left_arm_points = []
    right_arm_points = []
    
    for entry in data:
        # Skip entries where left arm configuration is null
        if entry['left_arm']['conf'] is None:
            continue
            
        left_tcp = entry['left_arm']['tcp_point_in_base_frame']
        right_tcp = entry['right_arm']['tcp_point_in_base_frame']
        
        left_arm_points.append(Point(*left_tcp))
        right_arm_points.append(Point(*right_tcp))
    
    return left_arm_points, right_arm_points


def are_points_colinear(p1, p2, p3, tolerance=1e-6):
    """
    Check if three points are colinear.
    
    Args:
        p1, p2, p3: Point objects
        tolerance (float): Tolerance for colinearity check
        
    Returns:
        bool: True if points are colinear, False otherwise
    """
    # Create vectors from p1 to p2 and p1 to p3
    v1 = Vector(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    v2 = Vector(p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2])
    
    # Check if cross product is close to zero (indicating colinearity)
    cross_product = v1.cross(v2)
    return np.linalg.norm([cross_product[0], cross_product[1], cross_product[2]]) < tolerance


def get_valid_point_combinations(left_points, right_points, min_points=3):
    """
    Get all valid combinations of 3 points that are not colinear.
    
    Args:
        left_points (list): List of left arm TCP points
        right_points (list): List of right arm TCP points
        min_points (int): Minimum number of points required (default: 3)
        
    Returns:
        list: List of tuples containing (left_indices, right_indices) for valid combinations
    """
    valid_combinations = []
    n_points = len(left_points)
    
    # Generate all combinations of 3 points
    for indices in combinations(range(n_points), min_points):
        left_triplet = [left_points[i] for i in indices]
        right_triplet = [right_points[i] for i in indices]
        
        # Check if left arm points are not colinear
        if not are_points_colinear(left_triplet[0], left_triplet[1], left_triplet[2]):
            valid_combinations.append((indices, indices))  # Same indices for both arms
    
    return valid_combinations


def compute_transformation_from_triplet(left_triplet, right_triplet):
    """
    Compute transformation from right arm base to left arm base using three points.
    
    Args:
        left_triplet (list): Three left arm points
        right_triplet (list): Three corresponding right arm points
        
    Returns:
        list: Transformation matrix from right arm base to left arm base
    """
    # Create frames from the triplets
    left_frame = Frame.from_points(left_triplet[0], left_triplet[1], left_triplet[2])
    right_frame = Frame.from_points(right_triplet[0], right_triplet[1], right_triplet[2])
    
    # Create transformations from base frames to point frames
    left_base_from_pt = Transformation.from_frame(left_frame)
    right_base_from_pt = Transformation.from_frame(right_frame)
    
    right_base_from_left_base = right_base_from_pt * left_base_from_pt.inverse()
    
    return np.array(right_base_from_left_base).tolist()


def average_transformations(transformation_matrices):
    """
    Average multiple transformation matrices by averaging their matrix elements.
    
    Args:
        transformation_matrices (list): List of transformation matrices
        
    Returns:
        Transformation: Averaged transformation
    """
    if not transformation_matrices:
        raise ValueError("No transformations provided for averaging")
    
    # Initialize average matrix using the same structure as the first matrix
    M_ave = copy.deepcopy(transformation_matrices[0])
    for i in range(4):
        for j in range(4):
            M_ave[i][j] = 0
    
    # Sum all transformation matrices
    for M in transformation_matrices:
        for i in range(4):
            for j in range(4):
                M_ave[i][j] += 1/len(transformation_matrices) * M[i][j]
    
    # Create transformation from averaged matrix
    return Transformation.from_matrix(M_ave)


def transformation_to_parameters(transformation):
    """
    Convert transformation to optimization parameters [x, y, z, roll, pitch, yaw].
    
    Args:
        transformation (Transformation): COMPAS transformation object
        
    Returns:
        list: [x, y, z, roll, pitch, yaw] parameters
    """
    scale, shear, rotation, translation, projection = transformation.decomposed()
    
    # Extract translation
    xyz = [translation.translation_vector[0], 
           translation.translation_vector[1], 
           translation.translation_vector[2]]
    
    # Extract rotation as Euler angles
    rpy = rotation.euler_angles(static=True, axes='xyz')
    
    return xyz + list(rpy)


def parameters_to_transformation(params):
    """
    Convert optimization parameters to transformation.
    
    Args:
        params (list): [x, y, z, roll, pitch, yaw] parameters
        
    Returns:
        Transformation: COMPAS transformation object
    """
    xyz = params[0:3]
    rpy = params[3:6]
    
    translation = Translation.from_vector(Vector(*xyz))
    rotation = Rotation.from_euler_angles(rpy, static=True, axes='xyz')
    
    return translation * rotation


def compute_error(transformation, left_points, right_points):
    """
    Compute the total error between transformed right arm points and left arm points.
    
    Args:
        transformation (Transformation): Transformation to apply, right_base_from_left_base
        left_points (list): Left arm points
        right_points (list): Right arm points
        
    Returns:
        float: Total squared error
    """
    total_error = 0.0
    right_base_from_left_base = transformation
    for left_pt, right_pt in zip(left_points, right_points):
        # Transform right arm point
        # right_From_pt, right_from_left
        # transformed_right_pt = right_pt.transformed(transformation)
        # Convert the left_pt (a Point) to a Transformation (as a translation)
        left_pt_transform = Translation.from_vector(Vector(*left_pt))
        right_base_from_left_point = (right_base_from_left_base * left_pt_transform)
        
        # Compute squared distance
        error = ((right_base_from_left_point.translation_vector[0] - right_pt[0])**2 + 
                (right_base_from_left_point.translation_vector[1] - right_pt[1])**2 + 
                (right_base_from_left_point.translation_vector[2] - right_pt[2])**2)
        
        total_error += error
    
    return total_error


def error_function(params, left_points, right_points):
    """
    Error function for optimization.
    
    Args:
        params (list): [x, y, z, roll, pitch, yaw] parameters
        left_points (list): Left arm points
        right_points (list): Right arm points
        
    Returns:
        float: Total error
    """
    transformation = parameters_to_transformation(params)
    return compute_error(transformation, left_points, right_points)


def main():
    """
    Main function to perform dual-arm intrinsic calibration.
    """
    # Configure logging
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO) 
    logger.addHandler(console_handler)

    # Create file handler with URDF type in name
    LOG_PATH = os.path.join(HERE, f"dual_arm_intrinsic_calibration_log.txt")
    file_handler = logging.FileHandler(LOG_PATH, mode='w')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(file_handler)

    logger.info("=== Dual-Arm Intrinsic Calibration ===")
    
    # Load calibration data
    # Ensure the JSON file path is correct relative to this script's location
    json_file_path = os.path.join(HERE, "20250822_dual-arm-intrinsic_data.json")
    left_arm_points, right_arm_points = load_calibration_data(json_file_path)
    
    logger.info(f"Loaded {len(left_arm_points)} valid data points")
    
    # Step 1: Find valid point combinations
    valid_combinations = get_valid_point_combinations(left_arm_points, right_arm_points)
    logger.info(f"Found {len(valid_combinations)} valid point combinations")
    
    # Step 2: Compute transformations for all valid combinations
    transformations = []
    for left_indices, right_indices in valid_combinations:
        left_triplet = [left_arm_points[i] for i in left_indices]
        right_triplet = [right_arm_points[i] for i in right_indices]
        
        try:
            right_base_from_left_base = compute_transformation_from_triplet(left_triplet, right_triplet)
            transformations.append(right_base_from_left_base)
        except Exception as e:
            logger.warning(f"Could not compute transformation for combination {left_indices}: {e}")
            continue
    
    if not transformations:
        raise ValueError("No valid transformations could be computed")
    
    # Step 3: Average all transformations
    average_transformation = average_transformations(transformations)
    initial_params = transformation_to_parameters(average_transformation)
    
    logger.info("\n=== Initial Average Transformation ===")
    logger.info(f"Translation: [{initial_params[0]:.3f}, {initial_params[1]:.3f}, {initial_params[2]:.3f}] mm")
    logger.info(f"Rotation (RPY): [{initial_params[3]:.3f}, {initial_params[4]:.3f}, {initial_params[5]:.3f}] rad")
    
    initial_error = error_function(initial_params, left_arm_points, right_arm_points)
    logger.info(f"Initial error: {initial_error:.3f} mm")
    
    # Step 4: Optimize transformation
    logger.info("\n=== Optimizing Transformation ===")
    
    result = minimize(
        error_function,
        initial_params,
        args=(left_arm_points, right_arm_points),
        method='Nelder-Mead',
        tol=1e-7,
        options={'maxiter': 10000}
    )
    
    if result.success:
        optimized_params = result.x
        optimized_transformation = parameters_to_transformation(optimized_params)
        final_error = error_function(optimized_params, left_arm_points, right_arm_points)
        
        logger.info("\n=== Optimized Transformation ===")
        logger.info(f"Translation: [{optimized_params[0]:.3f}, {optimized_params[1]:.3f}, {optimized_params[2]:.3f}] mm")
        logger.info(f"Rotation (RPY): [{optimized_params[3]:.3f}, {optimized_params[4]:.3f}, {optimized_params[5]:.3f}] rad")
        logger.info(f"Final error: {final_error:.3f} mm")
        logger.info(f"Optimization message: {result.message}")
        logger.info(f"Number of iterations: {result.nit}")
        
        # Save results
        results = {
            'translation': optimized_params[0:3].tolist(),
            'rotation_rpy': optimized_params[3:6].tolist(),
            'final_error': float(final_error),
            'initial_error': float(initial_error),
            'num_valid_combinations': len(transformations),
            'num_data_points': len(left_arm_points)
        }
        
        output_file = os.path.join(HERE, 'calibration_results.json')
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        logger.info(f"\nResults saved to: {output_file}")
        
    else:
        logger.error(f"Optimization failed: {result.message}")


if __name__ == "__main__":
    main()
