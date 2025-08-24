"""
Test script using original hardcoded data to verify our approach
"""

import json
import numpy as np
from itertools import combinations
from scipy.optimize import minimize
from compas.geometry import (
    Point, Frame, Transformation, Translation, Rotation, Vector
)
import copy

# Set precision for COMPAS
import compas
compas.PRECISION = '12f'

# Original hardcoded data from Cali_Transformation.py
x0 = [ 1669.46,   862.52,  912.38, 1608.12, 1660.35, 1645.74,  808.80,  838.56]
y0 = [ -401.07,  -398.49, -319.31, -275.45,  460.34,  413.09,  466.66,  433.28]
z0 = [  240.99,   241.12,  413.56,  413.31,  240.24,  412.44,  240.39,  412.72]
x1 = [ 687.90, 1495.00, 1445.47,    750.06,  701.66,   715.81, 1553.08, 1522.84]
y1 = [ 369.48,  370.24,  290.84,    243.90, -492.44,  -444.86, -494.72, -461.06]
z1 = [ 241.03,  243.13,  415.16,    413.33,  241.83,   413.91,  243.79,  415.95]

def are_points_colinear(p1, p2, p3, tolerance=1e-6):
    """Check if three points are colinear."""
    v1 = Vector(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    v2 = Vector(p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2])
    cross_product = v1.cross(v2)
    return np.linalg.norm([cross_product[0], cross_product[1], cross_product[2]]) < tolerance

def get_valid_point_combinations(left_points, right_points, min_points=3):
    """Get all valid combinations of 3 points that are not colinear."""
    valid_combinations = []
    n_points = len(left_points)
    
    for indices in combinations(range(n_points), min_points):
        left_triplet = [left_points[i] for i in indices]
        right_triplet = [right_points[i] for i in indices]
        
        if not are_points_colinear(left_triplet[0], left_triplet[1], left_triplet[2]):
            valid_combinations.append((indices, indices))
    
    return valid_combinations

def compute_transformation_from_triplet(left_triplet, right_triplet):
    """Compute transformation from right arm base to left arm base using three points."""
    left_frame = Frame.from_points(left_triplet[0], left_triplet[1], left_triplet[2])
    right_frame = Frame.from_points(right_triplet[0], right_triplet[1], right_triplet[2])
    
    left_base_from_point = Transformation.from_frame_to_frame(Frame.worldXY(), left_frame)
    right_base_from_point = Transformation.from_frame_to_frame(Frame.worldXY(), right_frame)
    
    left_base_from_right_base = left_base_from_point * right_base_from_point.inverse()
    
    return np.array(left_base_from_right_base).tolist()

def average_transformations(transformation_matrices):
    """Average multiple transformation matrices."""
    if not transformation_matrices:
        raise ValueError("No transformations provided for averaging")
    
    M_ave = copy.deepcopy(transformation_matrices[0])
    for i in range(4):
        for j in range(4):
            M_ave[i][j] = 0
    
    for M in transformation_matrices:
        for i in range(4):
            for j in range(4):
                M_ave[i][j] += 1/len(transformation_matrices) * M[i][j]
    
    return Transformation.from_matrix(M_ave)

def transformation_to_parameters(transformation):
    """Convert transformation to optimization parameters [x, y, z, roll, pitch, yaw]."""
    scale, shear, rotation, translation, projection = transformation.decomposed()
    
    xyz = [translation.translation_vector[0], 
           translation.translation_vector[1], 
           translation.translation_vector[2]]
    
    rpy = rotation.euler_angles(static=True, axes='xyz')
    
    return xyz + list(rpy)

def parameters_to_transformation(params):
    """Convert optimization parameters to transformation."""
    xyz = params[0:3]
    rpy = params[3:6]
    
    translation = Translation.from_vector(Vector(*xyz))
    rotation = Rotation.from_euler_angles(rpy, static=True, axes='xyz')
    
    return translation * rotation

def compute_error(transformation, left_points, right_points):
    """Compute the total error between transformed right arm points and left arm points."""
    total_error = 0.0
    
    for left_pt, right_pt in zip(left_points, right_points):
        transformed_right_pt = right_pt.transformed(transformation)
        
        error = ((transformed_right_pt[0] - left_pt[0])**2 + 
                (transformed_right_pt[1] - left_pt[1])**2 + 
                (transformed_right_pt[2] - left_pt[2])**2)
        
        total_error += error
    
    return total_error

def error_function(params, left_points, right_points):
    """Error function for optimization."""
    transformation = parameters_to_transformation(params)
    return compute_error(transformation, left_points, right_points)

def main():
    """Test with original hardcoded data."""
    # Create points from original data
    left_arm_points = []
    right_arm_points = []
    
    for x, y, z in zip(x0, y0, z0):
        left_arm_points.append(Point(x, y, z))
    
    for x, y, z in zip(x1, y1, z1):
        right_arm_points.append(Point(x, y, z))
    
    print(f"Loaded {len(left_arm_points)} data points from original script")
    
    # Find valid point combinations
    valid_combinations = get_valid_point_combinations(left_arm_points, right_arm_points)
    print(f"Found {len(valid_combinations)} valid point combinations")
    
    # Compute transformations for all valid combinations
    transformations = []
    for left_indices, right_indices in valid_combinations:
        left_triplet = [left_arm_points[i] for i in left_indices]
        right_triplet = [right_arm_points[i] for i in right_indices]
        
        try:
            transformation = compute_transformation_from_triplet(left_triplet, right_triplet)
            transformations.append(transformation)
        except Exception as e:
            print(f"Warning: Could not compute transformation for combination {left_indices}: {e}")
            continue
    
    if not transformations:
        raise ValueError("No valid transformations could be computed")
    
    # Average all transformations
    average_transformation = average_transformations(transformations)
    initial_params = transformation_to_parameters(average_transformation)
    
    print("\n=== Initial Average Transformation ===")
    print(f"Translation: [{initial_params[0]:.3f}, {initial_params[1]:.3f}, {initial_params[2]:.3f}]")
    print(f"Rotation (RPY): [{initial_params[3]:.3f}, {initial_params[4]:.3f}, {initial_params[5]:.3f}]")
    
    initial_error = error_function(initial_params, left_arm_points, right_arm_points)
    print(f"Initial error: {initial_error:.3f}")
    
    # Optimize transformation
    print("\n=== Optimizing Transformation ===")
    
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
        final_error = error_function(optimized_params, left_arm_points, right_arm_points)
        
        print("\n=== Optimized Transformation ===")
        print(f"Translation: [{optimized_params[0]:.3f}, {optimized_params[1]:.3f}, {optimized_params[2]:.3f}]")
        print(f"Rotation (RPY): [{optimized_params[3]:.3f}, {optimized_params[4]:.3f}, {optimized_params[5]:.3f}]")
        print(f"Final error: {final_error:.3f}")
        print(f"Optimization message: {result.message}")
        print(f"Number of iterations: {result.nit}")
        
    else:
        print(f"Optimization failed: {result.message}")

if __name__ == "__main__":
    main()
