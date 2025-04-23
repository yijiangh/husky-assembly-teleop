import pybullet_planning as pp
import numpy as np

rb_pos = [-0.44, 0.05, -0.12]
rb_quat = [0.02, -0.28, 0.96, 0.00]
pose = (rb_pos, rb_quat)

pts = [
 [-0.47, -0.03, -0.03],
 [-0.27, -0.04, 0.02],
 [-0.47, 0.02, 0.04],
 [-0.27, 0.03, -0.02],
 [0.47, -0.04, 0.01],
 [0.47, 0.05, -0.01],
 [0.27, 0.01, 0.04],
 [0.27, -0.00, -0.04]
]

tfed_pts = [pp.tform_point(pose, pt) for pt in pts]

def compare_point_sets(source_pts, target_pts, tag1, tag2):
    # Find the closest point in marker_positions for each point in tfed_pts
    for tfed_pt in source_pts:
        distances = [np.linalg.norm(np.array(tfed_pt) - np.array(marker_pt)) for marker_pt in target_pts]
        closest_index = np.argmin(distances)
        closest_point = target_pts[closest_index]
        closest_distance = distances[closest_index]
        print(f"Point {tfed_pt} -> Closest Point: {closest_point}, Distance: {closest_distance}")

    import matplotlib.pyplot as plt

    # Prepare data for plotting
    x_differences = []
    y_differences = []
    z_differences = []

    for tfed_pt in source_pts:
        distances = [np.linalg.norm(np.array(tfed_pt) - np.array(marker_pt)) for marker_pt in target_pts]
        closest_index = np.argmin(distances)
        closest_point = target_pts[closest_index]
        x_differences.append(tfed_pt[0] - closest_point[0])
        y_differences.append(tfed_pt[1] - closest_point[1])
        z_differences.append(tfed_pt[2] - closest_point[2])

    # Plot the differences
    plt.figure(figsize=(10, 6))

    plt.plot(x_differences, label='X-axis Difference', marker='o')
    plt.plot(y_differences, label='Y-axis Difference', marker='o')
    plt.plot(z_differences, label='Z-axis Difference', marker='o')

    plt.title(f'Differences in X, Y, Z axes for Closest Points ({tag1} and {tag2}')
    plt.xlabel('Point Index')
    plt.ylabel('Difference (m)')
    plt.legend()
    plt.grid()
    # save fig to HERE
    plt.savefig(f'{__file__.rsplit("/", 1)[0]}/differences_{tag1}_{tag2}.png')


# ! labeled markers
# Extract positions from the text
labeled_marker_positions = [
    [0.04, 0.10, -0.15],
    [-0.16, 0.07, -0.09],
    [0.04, 0.02, -0.12],
    [-0.17, 0.04, -0.17],
    [-0.91, 0.08, -0.07],
    [-0.91, 0.02, -0.13],
    [-0.71, 0.02, -0.08],
    [-0.71, 0.08, -0.14]
]

compare_point_sets(tfed_pts, labeled_marker_positions, 'rb tfed pts', 'labeled_marker_positions')

print('------------')

# ! unlabeled markers
# Extract marker positions into a list
unlabeled_marker_positions = [
    [-0.91, 0.08, -0.07],
    [-0.71, 0.08, -0.14],
    [-0.17, 0.04, -0.17],
    [-0.16, 0.07, -0.09],
    [0.04, 0.10, -0.15],
    [0.04, 0.02, -0.12],
    [-0.71, 0.02, -0.08],
    [-0.91, 0.02, -0.13],
    [-1.43, 0.48, -4.56]
]

compare_point_sets(tfed_pts, unlabeled_marker_positions, 'rb tfed pts', 'unlabeled_marker_positions')


compare_point_sets(labeled_marker_positions, unlabeled_marker_positions, 'labeled markers', 'unlabeled_marker_positions')

