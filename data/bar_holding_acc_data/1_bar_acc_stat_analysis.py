import json
import os
import sys, logging
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import pybullet_planning as pp
from scipy import stats
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestRegressor
from sklearn.feature_selection import mutual_info_regression
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.model_selection import train_test_split

DATA_BATCH = '20250507'

# Set up logging to file
HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, DATA_BATCH)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'bar_holding_acc_analysis_log_{DATA_BATCH}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

# Load the JSON data
# Search for compiled_bar_holding_acc JSON files in the data folder
json_files = [f for f in os.listdir(data_folder) if f.startswith('compiled_bar_holding_acc') and f.endswith('.json')]

if not json_files:
    logger.error(f"Error: No compiled_bar_holding_acc JSON files found in {data_folder}")
    sys.exit(1)

# Use the first matching file found
json_file_path = os.path.join(data_folder, json_files[0])
logger.info(f"Loading data from: {json_file_path}")

with open(json_file_path, 'r') as f:
    data = json.load(f)

# Create a list to hold the extracted data
extracted_data = []

# Extract relevant data from each entry
for entry in data:
    # Extract footprint pose (just use x,y coordinates for simplicity)
    # footprint_x = entry['footprint_pose'][0][0]
    # footprint_y = entry['footprint_pose'][0][1]
    roll, pitch, yaw = pp.euler_from_quat(entry['footprint_pose'][1])
    logger.info('footprint roll: {:.3f}, pitch: {:.3f}, yaw: {:.3f}'.format(roll, pitch, yaw))
    footprint_x, footprint_y, footprint_yaw = pp.base_values_from_pose(entry['footprint_pose'], 
                                                                       tolerance=0.013)
    
    # Extract bar position (height)
    bar_height = entry['fitted_line']['point'][2]
    
    # Extract closest axis
    closest_axis = entry['closest_axis']
    
    # Extract the distance from CoM to support polygon center
    distance_com_to_polygon = entry.get('distance_com_to_polygon_center')
    
    # Extract angle deviation (our output variable), in degrees
    angle_deviation = np.deg2rad(entry['angle_to_closest_axis'])
 
    # Add data to our list
    extracted_data.append({
        'footprint_x': footprint_x,
        'footprint_y': footprint_y,
        'footprint_yaw': footprint_yaw,
        'bar_height': bar_height,
        'closest_axis': closest_axis,
        'distance_com_to_polygon': distance_com_to_polygon,
        'angle_deviation': angle_deviation
    })

# Convert to DataFrame for easier analysis
df = pd.DataFrame(extracted_data)

# Calculate average angle deviation and standard deviation
average_angle_deviation = df['angle_deviation'].mean()
std_angle_deviation = df['angle_deviation'].std()

# Print the statistics
logger.info(f"Average angle deviation: {average_angle_deviation:.4f} rad ({np.degrees(average_angle_deviation):.4f} degrees)")
logger.info(f"Standard deviation: {std_angle_deviation:.4f} rad ({np.degrees(std_angle_deviation):.4f} degrees)")

# Also compute median and interquartile range
median_angle_deviation = df['angle_deviation'].median()
q1 = df['angle_deviation'].quantile(0.25)
q3 = df['angle_deviation'].quantile(0.75)
iqr = q3 - q1

logger.info(f"Median angle deviation: {median_angle_deviation:.4f} rad ({np.degrees(median_angle_deviation):.4f} degrees)")
logger.info(f"Interquartile range: {iqr:.4f} rad ({np.degrees(iqr):.4f} degrees)")

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

# Categorize footprint yaw into 8 partitions (0-360 degrees)
YAW_CATEGORIES =['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'] 
def categorize_yaw(yaw_rad):
    # Normalize to [0, 2π)
    yaw_normalized = yaw_rad % (2 * np.pi)
    # Each partition is 45 degrees (π/4 radians)
    partition_size = np.pi / 4
    partition_number = int(yaw_normalized / partition_size)
    return YAW_CATEGORIES[partition_number]

df['yaw_category'] = df['footprint_yaw'].apply(categorize_yaw)

# Add one-hot encoding for closest axis and height category
df = pd.get_dummies(df, columns=['closest_axis', 'height_category', 'yaw_category'], 
                    prefix=['axis', 'height', 'yaw'])

# # Remove bar_height and axis_label columns as they are redundant after feature engineering
# df = df.drop(['bar_height', 'axis_label'], axis=1)
# logger.info("Dropped redundant columns: bar_height and axis_label")

# Export DataFrame to CSV
csv_output_path = os.path.join(data_folder, f'bar_holding_acc_data_{DATA_BATCH}.csv')
df.to_csv(csv_output_path, index=False)
logger.info(f"DataFrame exported to CSV: {csv_output_path}")

# Display basic statistics
logger.info("Basic Statistics:")
logger.info(df.describe())

# Correlation analysis
logger.info("\nCorrelation with angle_deviation:")
# Filter out bar_height and axis_label from correlation analysis
correlation_columns = [col for col in df.columns if col != 'bar_height' and col != 'axis_label']
correlations = df[correlation_columns].corr()['angle_deviation'].sort_values(ascending=False)
logger.info(correlations)

# Visualize data

# Figure 1: Scatter plot of distance_com_to_polygon vs angle_deviation
plt.figure(figsize=(10, 6))
sns.scatterplot(x='distance_com_to_polygon', y='angle_deviation', 
                hue='axis_label', data=df)
plt.title('Distance CoM to Polygon vs Angle Deviation')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Angle Deviation (rad)')
plt.savefig(os.path.join(data_folder, '1_com_distance_vs_angle.png'))

# Figure 2: Boxplot of angle deviation by closest axis
plt.figure(figsize=(10, 6))
axis_data = pd.melt(df, id_vars=['angle_deviation'], 
                     value_vars=['axis_0', 'axis_1', 'axis_2'], 
                     var_name='axis', value_name='is_axis')
axis_data = axis_data[axis_data['is_axis'] == 1]
sns.boxplot(x='axis', y='angle_deviation', data=axis_data)
plt.title('Angle Deviation by Closest Axis')
plt.xlabel('Closest Axis')
plt.ylabel('Angle Deviation (rad)')
plt.savefig(os.path.join(data_folder, '2_bar_axis_vs_angle.png'))

# Figure 3: Boxplot of angle deviation by height category
plt.figure(figsize=(10, 6))
height_data = pd.melt(df, id_vars=['angle_deviation'], 
                       value_vars=['height_low', 'height_mid', 'height_high'], 
                       var_name='height', value_name='is_height')
height_data = height_data[height_data['is_height'] == 1]
sns.boxplot(x='height', y='angle_deviation', data=height_data)
plt.title('Angle Deviation by Bar Height')
plt.xlabel('Bar Height Category')
plt.ylabel('Angle Deviation (rad)')
plt.savefig(os.path.join(data_folder, '3_bar_height_vs_angle.png'))

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

# Figure 5: Boxplot of angle deviation by yaw category
plt.figure(figsize=(10, 6))

# Get yaw direction columns that actually exist in the dataframe
yaw_cols = ['yaw_' + n for n in YAW_CATEGORIES if 'yaw_' + n in df.columns]

if yaw_cols:  # Only proceed if we found valid yaw columns
    yaw_data = pd.melt(df, id_vars=['angle_deviation'], 
                       value_vars=yaw_cols,
                       var_name='yaw', value_name='is_yaw_direction')
    yaw_data = yaw_data[yaw_data['is_yaw_direction'] == 1]
    
    # Extract the yaw category from column name for better labeling
    yaw_data['yaw_category'] = yaw_data['yaw'].str.replace('yaw_', '')
    
    sns.boxplot(x='yaw_category', y='angle_deviation', data=yaw_data)
    plt.title('Angle Deviation by Footprint Yaw Direction')
    plt.xlabel('Footprint Yaw Direction')
    plt.ylabel('Angle Deviation (rad)')
    plt.savefig(os.path.join(data_folder, '5_yaw_vs_angle.png'))
else:
    logger.warning("No yaw direction columns found in the dataframe. Skipping yaw vs angle plot.")

# Feature importance analysis using Random Forest
# Drop one-hot encoded categorical columns to avoid multicollinearity
categorical_cols = [col for col in df.columns if col.startswith(('axis_', 'yaw_', 'height_'))]
logger.info(f"Dropping one-hot encoded columns: {categorical_cols}")

X = df.drop(['angle_deviation', 'axis_label'] + categorical_cols, axis=1)
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
logger.info("\nRandom Forest Model Performance:")
logger.info(f"R² score: {r2_score(y_test, y_pred):.4f}")
logger.info(f"Mean squared error: {mean_squared_error(y_test, y_pred):.4f}")

# Calculate feature importance from Random Forest
feature_importance = pd.DataFrame({
    'Feature': X.columns,
    'Importance': rf.feature_importances_
}).sort_values('Importance', ascending=False)

logger.info("\nFeature Importance from Random Forest:")
logger.info(feature_importance)

# Calculate mutual information for non-linear relationships
mi_scores = mutual_info_regression(X_scaled, y)
mi_df = pd.DataFrame({
    'Feature': X.columns,
    'MI Score': mi_scores
}).sort_values('MI Score', ascending=False)

logger.info("\nMutual Information Scores (for non-linear relationships):")
logger.info(mi_df)

# Visualize feature importance
plt.figure(figsize=(12, 6))
sns.barplot(x='Importance', y='Feature', data=feature_importance)
plt.title('Feature Importance for Predicting Angle Deviation')
plt.tight_layout()
plt.savefig(os.path.join(data_folder, '6_feature_importance.png'))

# Visualize mutual information
plt.figure(figsize=(12, 6))
sns.barplot(x='MI Score', y='Feature', data=mi_df)
plt.title('Mutual Information Scores for Predicting Angle Deviation')
plt.tight_layout()
plt.savefig(os.path.join(data_folder, '7_mutual_info.png'))

# ANOVA analysis for categorical variables

# Analyze effect of axis choice on angle deviation
logger.info("\nANOVA Analysis - Effect of axis choice on angle deviation:")
axis_0_angles = df[df['axis_0'] == 1]['angle_deviation']
axis_1_angles = df[df['axis_1'] == 1]['angle_deviation']
axis_2_angles = df[df['axis_2'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(axis_0_angles, axis_1_angles, axis_2_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.info("There is a statistically significant difference in angle deviation between axes.")
else:
    logger.info("No statistically significant difference in angle deviation between axes.")

# Analyze effect of height category on angle deviation
logger.info("\nANOVA Analysis - Effect of height category on angle deviation:")
low_angles = df[df['height_low'] == 1]['angle_deviation']
mid_angles = df[df['height_mid'] == 1]['angle_deviation']
high_angles = df[df['height_high'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(low_angles, mid_angles, high_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.info("There is a statistically significant difference in angle deviation between height categories.")
else:
    logger.info("No statistically significant difference in angle deviation between height categories.")

# Summary visualization - control variables vs angle deviation
plt.figure(figsize=(10, 6))
plt.scatter(df['distance_com_to_polygon'], df['angle_deviation'], 
            c=df['bar_height'], cmap='viridis', alpha=0.7)
plt.colorbar(label='Bar Height')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Angle Deviation (rad)')
plt.title('Distance CoM vs Angle Deviation (colored by bar height)')
plt.savefig(os.path.join(data_folder, '8_summary_visualization.png'))

# Analyze effect of CoM distance on angle deviation
# Discretize distance_com_to_polygon into quartiles for ANOVA
logger.info("\nANOVA Analysis - Effect of CoM distance on angle deviation:")
df['distance_category'] = pd.qcut(df['distance_com_to_polygon'], 4, labels=['Q1', 'Q2', 'Q3', 'Q4'])

# Group angles by distance quartile
q1_angles = df[df['distance_category'] == 'Q1']['angle_deviation']
q2_angles = df[df['distance_category'] == 'Q2']['angle_deviation']
q3_angles = df[df['distance_category'] == 'Q3']['angle_deviation']
q4_angles = df[df['distance_category'] == 'Q4']['angle_deviation']

# Run ANOVA test
f_stat, p_val = stats.f_oneway(q1_angles, q2_angles, q3_angles, q4_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.info("There is a statistically significant difference in angle deviation between CoM distance quartiles.")
else:
    logger.info("No statistically significant difference in angle deviation between CoM distance quartiles.")

# Visualize the relationship with a box plot
plt.figure(figsize=(10, 6))
sns.boxplot(x='distance_category', y='angle_deviation', data=df)
plt.title('Angle Deviation by Distance to Support Polygon (Quartiles)')
plt.xlabel('Distance from CoM to Support Polygon Center (Quartiles)')
plt.ylabel('Angle Deviation (rad)')
plt.savefig(os.path.join(data_folder, '9_com_distance_categories_vs_angle.png'))

logger.info("\nAnalysis complete. Visualizations saved to disk.")