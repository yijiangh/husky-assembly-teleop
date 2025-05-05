import os
import json
import numpy as np
import matplotlib.pyplot as plt

# Define the path to the JSON file
HERE = os.path.dirname(os.path.abspath(__file__))
json_file_path = os.path.join(HERE, "bar_holding_acc_20250505_1446_.json")

# Load the JSON data
with open(json_file_path, "r") as file:
    data = json.load(file)

# Extract raw data
raw_data = data["raw_data"]

# We have 4 data takes in total, and we want to compare:
# 1) First take vs Second take
# 2) Third take vs Fourth take
comparisons = [(0, 1), (2, 3)]  # Indices for the comparisons

# Initialize a dictionary to store position differences
pos_differences = {}

# For each marker (1-8), compute position differences between specified takes
for marker_id in range(1, 9):
    marker = str(marker_id)
    
    # Compare first take with second take
    first_pos = np.array(raw_data[0]["bar_rig"][marker]["pos"])
    second_pos = np.array(raw_data[1]["bar_rig"][marker]["pos"])
    diff_1_2 = np.linalg.norm(first_pos - second_pos)
    
    # Compare third take with fourth take
    third_pos = np.array(raw_data[2]["bar_rig"][marker]["pos"])
    fourth_pos = np.array(raw_data[3]["bar_rig"][marker]["pos"])
    diff_3_4 = np.linalg.norm(third_pos - fourth_pos)
    
    # Store both comparisons
    pos_differences[f"Marker {marker} (Take 1-2)"] = diff_1_2
    pos_differences[f"Marker {marker} (Take 3-4)"] = diff_3_4

# Calculate mean and standard deviation of position differences
# Separate Take 1-2 and Take 3-4 for better analysis
take_1_2_diffs = [diff for key, diff in pos_differences.items() if "Take 1-2" in key]
take_3_4_diffs = [diff for key, diff in pos_differences.items() if "Take 3-4" in key]
all_diffs = list(pos_differences.values())

# Calculate statistics
mean_all = np.mean(all_diffs)
std_all = np.std(all_diffs)
mean_1_2 = np.mean(take_1_2_diffs)
std_1_2 = np.std(take_1_2_diffs)
mean_3_4 = np.mean(take_3_4_diffs)
std_3_4 = np.std(take_3_4_diffs)

# Print statistics
print(f"All position differences - Mean: {mean_all:.6f} m, Std: {std_all:.6f} m")
print(f"Take 1-2 differences - Mean: {mean_1_2:.6f} m, Std: {std_1_2:.6f} m")
print(f"Take 3-4 differences - Mean: {mean_3_4:.6f} m, Std: {std_3_4:.6f} m")

# Plot the position differences as a scatter plot
plt.figure(figsize=(12, 8))
markers = sorted(pos_differences.keys())
x_values = []
y_values = []
labels = []

for i, marker in enumerate(markers):
    # Alternate between take 1-2 and take 3-4 comparisons
    take_type = "Take 1-2" if "Take 1-2" in marker else "Take 3-4"
    marker_num = marker.split()[1]
    
    x_values.append(i)
    y_values.append(pos_differences[marker])
    labels.append(f"{marker_num} ({take_type})")

plt.scatter(x_values, y_values, s=100, c=range(len(x_values)), cmap='viridis')

# Add labels to each point
for i, label in enumerate(labels):
    plt.annotate(label, (x_values[i], y_values[i]), 
                 xytext=(5, 5), textcoords='offset points')

plt.xlabel("Data Take Index")
plt.ylabel("Position Difference (m)")
plt.title("Position Differences Between Markers")
plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left")
plt.tight_layout()

# Save the plot
output_plot_path = os.path.join(HERE, "marker_position_differences.png")
plt.savefig(output_plot_path)
plt.show()

print(f"Plot saved to {output_plot_path}")