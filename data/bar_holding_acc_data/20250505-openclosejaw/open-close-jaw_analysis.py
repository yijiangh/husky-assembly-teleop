import os
import json
import numpy as np
import matplotlib.pyplot as plt
from skspatial.objects import Line

# Define the path to the JSON file
HERE = os.path.dirname(os.path.abspath(__file__))
json_file_path = os.path.join(HERE, "bar_holding_acc_20250505_1446_.json")

# Marker name pairs
MARKER_NAME_PAIRS = [
    ['5', '6'],
    ['7', '8'],
    ['2', '4'],
    ['1', '3']
]

# Load the JSON data
with open(json_file_path, "r") as file:
    data = json.load(file)

# Extract raw data
raw_data = data["raw_data"]

# Function to compute the angle between two lines
def compute_angle_between_lines(line1, line2):
    direction1 = line1.direction / np.linalg.norm(line1.direction)
    direction2 = line2.direction / np.linalg.norm(line2.direction)
    angle_rad = np.arccos(np.clip(np.dot(direction1, direction2), -1.0, 1.0))
    return np.degrees(angle_rad)

# Compute fitted lines for each take
fitted_lines = []
marker_position_differences = {pair: [] for pair in MARKER_NAME_PAIRS}

for take in raw_data:
    marker_centers = []
    for marker1, marker2 in MARKER_NAME_PAIRS:
        pos1 = np.array(take["bar_rig"][marker1]["pos"])
        pos2 = np.array(take["bar_rig"][marker2]["pos"])
        center = (pos1 + pos2) / 2  # Average position of the marker pair
        marker_centers.append(center)

        # Compute position difference between the two markers
        distance = np.linalg.norm(pos1 - pos2)
        marker_position_differences[(marker1, marker2)].append(distance)

    # Fit a line using the averaged marker positions
    line_fit = Line.best_fit(marker_centers)
    fitted_lines.append(line_fit)

# Compute angle differences between specified takes
angle_diff_1_2 = compute_angle_between_lines(fitted_lines[0], fitted_lines[1])
angle_diff_3_4 = compute_angle_between_lines(fitted_lines[2], fitted_lines[3])

# Plot the angle differences
plt.figure(figsize=(8, 6))
x_labels = ["Take 1-2", "Take 3-4"]
y_values = [angle_diff_1_2, angle_diff_3_4]

plt.bar(x_labels, y_values, color="skyblue", alpha=0.8)
plt.ylabel("Angle Difference (degrees)")
plt.title("Angle Differences Between Fitted Lines")
plt.tight_layout()

# Save the angle difference plot
angle_diff_plot_path = os.path.join(HERE, "line_angle_differences.png")
plt.savefig(angle_diff_plot_path)
plt.show()

print(f"Angle differences plotted and saved to {angle_diff_plot_path}")
print(f"Angle difference (Take 1-2): {angle_diff_1_2:.2f} degrees")
print(f"Angle difference (Take 3-4): {angle_diff_3_4:.2f} degrees")

# Plot the marker-to-marker position differences
plt.figure(figsize=(12, 8))
for pair, distances in marker_position_differences.items():
    plt.plot(distances, label=f"Marker {pair[0]} - Marker {pair[1]}")

plt.xlabel("Data Take Index")
plt.ylabel("Position Difference (m)")
plt.title("Marker-to-Marker Position Differences")
plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
plt.tight_layout()

# Save the position difference plot
position_diff_plot_path = os.path.join(HERE, "marker_position_differences.png")
plt.savefig(position_diff_plot_path)
plt.show()

print(f"Marker-to-marker position differences plotted and saved to {position_diff_plot_path}")