import json
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestRegressor
from sklearn.feature_selection import mutual_info_regression
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import train_test_split

HERE = os.path.dirname(os.path.abspath(__file__))

# Set up logging to file
log_file = os.path.join(HERE, 'bar_acc_stat_analysis_log.txt')
sys.stdout = open(log_file, 'w')

# Load the JSON data
with open(os.path.join(HERE, 'analysis_bar_holding_acc_.json'), 'r') as f:
    data = json.load(f)

# Create a list to hold the extracted data
extracted_data = []

# Extract relevant data from each entry
for entry in data:
    # Extract footprint pose (just use x,y coordinates for simplicity)
    footprint_x = entry['footprint_pose'][0][0]
    footprint_y = entry['footprint_pose'][0][1]
    
    # Extract bar position (height)
    bar_height = entry['fitted_line']['point'][2]
    
    # Extract closest axis
    closest_axis = entry['closest_axis']
    
    # Extract the distance from CoM to support polygon center
    distance_com_to_polygon = entry.get('distance_com_to_polygon_center')
    
    # Extract angle deviation (our output variable), in degrees
    angle_deviation = entry['angle_to_closest_axis']
    
    # Add data to our list
    extracted_data.append({
        'footprint_x': footprint_x,
        'footprint_y': footprint_y,
        'bar_height': bar_height,
        'closest_axis': closest_axis,
        'distance_com_to_polygon': distance_com_to_polygon,
        'angle_deviation': angle_deviation
    })

# Convert to DataFrame for easier analysis
df = pd.DataFrame(extracted_data)

# Save original closest_axis before one-hot encoding (for coloring plots)
df['axis_label'] = df['closest_axis'].astype(str)

# Categorize bar height into low, mid, high
bar_heights = df['bar_height'].sort_values().unique()
height_thresholds = [np.percentile(bar_heights, 33), np.percentile(bar_heights, 66)]

def categorize_height(height):
    if height <= height_thresholds[0]:
        return 'low'
    elif height <= height_thresholds[1]:
        return 'mid'
    else:
        return 'high'

df['height_category'] = df['bar_height'].apply(categorize_height)

# Add one-hot encoding for closest axis and height category
df = pd.get_dummies(df, columns=['closest_axis', 'height_category'], prefix=['axis', 'height'])

# Display basic statistics
print("Basic Statistics:")
print(df.describe())

# Correlation analysis
print("\nCorrelation with angle_deviation:")
correlations = df.corr()['angle_deviation'].sort_values(ascending=False)
print(correlations)

# Visualize data

# Figure 1: Scatter plot of distance_com_to_polygon vs angle_deviation
plt.figure(figsize=(10, 6))
sns.scatterplot(x='distance_com_to_polygon', y='angle_deviation', 
                hue='axis_label', data=df)
plt.title('Distance CoM to Polygon vs Angle Deviation')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Angle Deviation (degrees)')
plt.savefig(os.path.join(HERE, 'com_distance_vs_angle.png'))

# Figure 2: Boxplot of angle deviation by closest axis
plt.figure(figsize=(10, 6))
axis_data = pd.melt(df, id_vars=['angle_deviation'], 
                     value_vars=['axis_0', 'axis_1', 'axis_2'], 
                     var_name='axis', value_name='is_axis')
axis_data = axis_data[axis_data['is_axis'] == 1]
sns.boxplot(x='axis', y='angle_deviation', data=axis_data)
plt.title('Angle Deviation by Closest Axis')
plt.xlabel('Closest Axis')
plt.ylabel('Angle Deviation (degrees)')
plt.savefig(os.path.join(HERE, 'angle_by_axis.png'))

# Figure 3: Boxplot of angle deviation by height category
plt.figure(figsize=(10, 6))
height_data = pd.melt(df, id_vars=['angle_deviation'], 
                       value_vars=['height_low', 'height_mid', 'height_high'], 
                       var_name='height', value_name='is_height')
height_data = height_data[height_data['is_height'] == 1]
sns.boxplot(x='height', y='angle_deviation', data=height_data)
plt.title('Angle Deviation by Bar Height')
plt.xlabel('Bar Height Category')
plt.ylabel('Angle Deviation (degrees)')
plt.savefig(os.path.join(HERE, 'angle_by_height.png'))

# Figure 4: Footprint position effect on angle deviation (heatmap)
plt.figure(figsize=(10, 8))
pivot_table = df.pivot_table(
    values='angle_deviation', 
    index=pd.cut(df['footprint_y'], 10), 
    columns=pd.cut(df['footprint_x'], 10),
    aggfunc='mean'
)
sns.heatmap(pivot_table, cmap='viridis')
plt.title('Mean Angle Deviation by Robot Position')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.savefig(os.path.join(HERE, 'position_heatmap.png'))

# Feature importance analysis using Random Forest
X = df.drop(['angle_deviation', 'axis_label'], axis=1)  # Also drop axis_label
y = df['angle_deviation']

# Scale features
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)
X_scaled = pd.DataFrame(X_scaled, columns=X.columns)

# Split data for testing
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.25, random_state=42)

# Train a Random Forest regression model
rf = RandomForestRegressor(n_estimators=100, random_state=42)
rf.fit(X_train, y_train)

# Evaluate model
y_pred = rf.predict(X_test)
print("\nRandom Forest Model Performance:")
print(f"R² score: {r2_score(y_test, y_pred):.4f}")
print(f"Mean squared error: {mean_squared_error(y_test, y_pred):.4f}")

# Calculate feature importance from Random Forest
feature_importance = pd.DataFrame({
    'Feature': X.columns,
    'Importance': rf.feature_importances_
}).sort_values('Importance', ascending=False)

print("\nFeature Importance from Random Forest:")
print(feature_importance)

# Calculate mutual information for non-linear relationships
mi_scores = mutual_info_regression(X_scaled, y)
mi_df = pd.DataFrame({
    'Feature': X.columns,
    'MI Score': mi_scores
}).sort_values('MI Score', ascending=False)

print("\nMutual Information Scores (for non-linear relationships):")
print(mi_df)

# Visualize feature importance
plt.figure(figsize=(12, 6))
sns.barplot(x='Importance', y='Feature', data=feature_importance)
plt.title('Feature Importance for Predicting Angle Deviation')
plt.tight_layout()
plt.savefig(os.path.join(HERE, 'feature_importance.png'))

# Visualize mutual information
plt.figure(figsize=(12, 6))
sns.barplot(x='MI Score', y='Feature', data=mi_df)
plt.title('Mutual Information Scores for Predicting Angle Deviation')
plt.tight_layout()
plt.savefig(os.path.join(HERE, 'mutual_info.png'))

# ANOVA analysis for categorical variables

# Analyze effect of axis choice on angle deviation
print("\nANOVA Analysis - Effect of axis choice on angle deviation:")
axis_0_angles = df[df['axis_0'] == 1]['angle_deviation']
axis_1_angles = df[df['axis_1'] == 1]['angle_deviation']
axis_2_angles = df[df['axis_2'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(axis_0_angles, axis_1_angles, axis_2_angles)
print(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    print("There is a statistically significant difference in angle deviation between axes.")
else:
    print("No statistically significant difference in angle deviation between axes.")

# Analyze effect of height category on angle deviation
print("\nANOVA Analysis - Effect of height category on angle deviation:")
low_angles = df[df['height_low'] == 1]['angle_deviation']
mid_angles = df[df['height_mid'] == 1]['angle_deviation']
high_angles = df[df['height_high'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(low_angles, mid_angles, high_angles)
print(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    print("There is a statistically significant difference in angle deviation between height categories.")
else:
    print("No statistically significant difference in angle deviation between height categories.")

# Summary visualization - control variables vs angle deviation
plt.figure(figsize=(10, 6))
plt.scatter(df['distance_com_to_polygon'], df['angle_deviation'], 
            c=df['bar_height'], cmap='viridis', alpha=0.7)
plt.colorbar(label='Bar Height')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Angle Deviation (degrees)')
plt.title('Distance CoM vs Angle Deviation (colored by bar height)')
plt.savefig(os.path.join(HERE, 'summary_visualization.png'))

print("\nAnalysis complete. Visualizations saved to disk.")

# Close the file to ensure all output is written
sys.stdout.close()

# Restore stdout for any remaining prints
sys.stdout = sys.__stdout__
print(f"Analysis complete. Results logged to {log_file}")