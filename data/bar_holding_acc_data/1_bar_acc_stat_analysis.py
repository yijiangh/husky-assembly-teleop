import json
import os
import colorlog
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

DATA_BATCH = '20250519_fixed_pos_vary_yaw'
# DATA_BATCH = '20250519_vary_pos_vary_yaw'

# Set up logging to file
HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, DATA_BATCH)

# Configure logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

color_formatter = colorlog.ColoredFormatter(
    '%(log_color)s%(asctime)s - %(levelname)s - %(message)s',
    log_colors={
        'DEBUG': 'cyan',
        'INFO': 'green',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'red,bg_white',
    }
)
console_handler.setFormatter(color_formatter)
logger.addHandler(console_handler)

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
                                                                       tolerance=0.02)

    # 'point_centers': [list(center) for center in center_points], 
    # 'fitted_line': {'point' : list(line_fit.point), 'direction' : list(line_fit.direction)}, 
    
    # Extract bar position (height)
    bar_height = entry['fitted_line']['point'][2]
    
    # Extract closest axis
    closest_axis = entry['closest_axis']
    
    # Extract the distance from CoM to support polygon center
    distance_com_to_polygon = entry.get('distance_com_to_polygon_center')
    
    # Extract angle deviation (our output variable), in degrees, converted to rad
    angle_deviation = np.deg2rad(entry['angle_to_closest_axis'])

    pos_deviation = entry['bar_pos_error']
 
    # Add data to our list
    extracted_data.append({
        'footprint_x': footprint_x,
        'footprint_y': footprint_y,
        'footprint_yaw': footprint_yaw,
        'bar_height': bar_height,
        'closest_axis': closest_axis,
        'distance_com_to_polygon': distance_com_to_polygon,
        'angle_deviation': angle_deviation,
        'pos_deviation': pos_deviation,
    })

# Convert to DataFrame for easier analysis
df = pd.DataFrame(extracted_data)

# Identify outliers in pos_deviation
q1_pos = df['pos_deviation'].quantile(0.25)
q3_pos = df['pos_deviation'].quantile(0.75)
iqr_pos = q3_pos - q1_pos
lower_bound = q1_pos - 1.5 * iqr_pos
upper_bound = q3_pos + 1.5 * iqr_pos

# Log outlier information
outlier_count = df[df['pos_deviation'] > upper_bound].shape[0]
logger.warning(f"Detected {outlier_count} outliers in position deviation data (above {upper_bound:.4f})")
logger.warning(f"Position deviation range: {df['pos_deviation'].min():.4f} to {df['pos_deviation'].max():.4f}")

# Print some details about outliers
if outlier_count > 0:
    outliers = df[df['pos_deviation'] > upper_bound]
    logger.info(f"Outlier details (first 5):")
    for i, (_, row) in enumerate(outliers.iterrows()):
        if i >= 5: break
        logger.warning(f"  Outlier {i+1}: pos_dev={row['pos_deviation']:.4f}, angle_dev={row['angle_deviation']:.4f}, "
                   f"footprint=({row['footprint_x']:.2f}, {row['footprint_y']:.2f}, {row['footprint_yaw']:.2f}), "
                   f"bar_height={row['bar_height']:.2f}")

# Create a copy of the original dataframe for reference
df_original = df.copy()

# Remove outliers
df = df[df['pos_deviation'] <= upper_bound]
removed_count = df_original.shape[0] - df.shape[0]
logger.info(f"Removed {removed_count} outlier rows, {df.shape[0]} rows remaining")

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

# Add this code after the angle deviation statistics section

# Calculate position deviation statistics
average_pos_deviation = df['pos_deviation'].mean() * 1000  # Convert to mm
std_pos_deviation = df['pos_deviation'].std() * 1000  # Convert to mm
median_pos_deviation = df['pos_deviation'].median() * 1000  # Convert to mm
q1_pos = df['pos_deviation'].quantile(0.25) * 1000  # Convert to mm
q3_pos = df['pos_deviation'].quantile(0.75) * 1000  # Convert to mm
iqr_pos = q3_pos - q1_pos  # Already in mm

# Log position deviation statistics
logger.info("\nPosition Deviation Statistics:")
logger.info(f"Average position deviation: {average_pos_deviation:.2f} mm")
logger.info(f"Standard deviation: {std_pos_deviation:.2f} mm")
logger.info(f"Median position deviation: {median_pos_deviation:.2f} mm")
logger.info(f"Interquartile range: {iqr_pos:.2f} mm")

# --- Combined Visualizations (angle and position deviation) ---

# Figure 1: Distance CoM to Polygon vs Angle/Position Deviation
plt.figure(figsize=(12, 10))

# Create two subplots
plt.subplot(2, 1, 1)
sns.scatterplot(x='distance_com_to_polygon', y='angle_deviation', 
                hue='axis_label', data=df)
plt.title('Distance CoM to Polygon vs Angle Deviation')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Angle Deviation (rad)')

plt.subplot(2, 1, 2)
# Convert pos_deviation to mm for plotting
sns.scatterplot(x='distance_com_to_polygon', y=df['pos_deviation'] * 1000, 
                hue='axis_label', data=df)
plt.title('Distance CoM to Polygon vs Position Deviation')
plt.xlabel('Distance from CoM to Support Polygon Center')
plt.ylabel('Position Deviation (mm)')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '1_com_distance_vs_deviations.png'))

# Figure 2: Axis Effect on Angle/Position Deviation
plt.figure(figsize=(12, 10))

# For angle deviation
plt.subplot(2, 1, 1)
axis_data_angle = pd.melt(df, id_vars=['angle_deviation'], 
                     value_vars=['axis_0', 'axis_1', 'axis_2'], 
                     var_name='axis', value_name='is_axis')
axis_data_angle = axis_data_angle[axis_data_angle['is_axis'] == 1]
sns.boxplot(x='axis', y='angle_deviation', data=axis_data_angle)
plt.title('Angle Deviation by Closest Axis')
plt.xlabel('Closest Axis')
plt.ylabel('Angle Deviation (rad)')

# For position deviation
plt.subplot(2, 1, 2)
axis_data_pos = pd.melt(df, id_vars=['pos_deviation'], 
                     value_vars=['axis_0', 'axis_1', 'axis_2'], 
                     var_name='axis', value_name='is_axis')
axis_data_pos = axis_data_pos[axis_data_pos['is_axis'] == 1]
# Convert pos_deviation to mm for plotting
axis_data_pos['pos_deviation_mm'] = axis_data_pos['pos_deviation'] * 1000
sns.boxplot(x='axis', y='pos_deviation_mm', data=axis_data_pos)
plt.title('Position Deviation by Closest Axis')
plt.xlabel('Closest Axis')
plt.ylabel('Position Deviation (mm)')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '2_bar_axis_vs_deviations.png'))

# Figure 3: Height Effect on Angle/Position Deviation
plt.figure(figsize=(12, 10))

# For angle deviation
plt.subplot(2, 1, 1)
height_data_angle = pd.melt(df, id_vars=['angle_deviation'], 
                       value_vars=['height_low', 'height_mid', 'height_high'], 
                       var_name='height', value_name='is_height')
height_data_angle = height_data_angle[height_data_angle['is_height'] == 1]
sns.boxplot(x='height', y='angle_deviation', data=height_data_angle)
plt.title('Angle Deviation by Bar Height')
plt.xlabel('Bar Height Category')
plt.ylabel('Angle Deviation (rad)')

# For position deviation
plt.subplot(2, 1, 2)
height_data_pos = pd.melt(df, id_vars=['pos_deviation'], 
                       value_vars=['height_low', 'height_mid', 'height_high'], 
                       var_name='height', value_name='is_height')
height_data_pos = height_data_pos[height_data_pos['is_height'] == 1]
# Convert pos_deviation to mm for plotting
height_data_pos['pos_deviation_mm'] = height_data_pos['pos_deviation'] * 1000
sns.boxplot(x='height', y='pos_deviation_mm', data=height_data_pos)
plt.title('Position Deviation by Bar Height')
plt.xlabel('Bar Height Category')
plt.ylabel('Position Deviation (mm)')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '3_bar_height_vs_deviations.png'))

# Figure 4: Footprint Position Effect on Angle/Position Deviation
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 14))

# Calculate bin edges with 0.5 meter spacing
x_min, x_max = df['footprint_x'].min(), df['footprint_x'].max()
y_min, y_max = df['footprint_y'].min(), df['footprint_y'].max()
x_bins = np.arange(np.floor(x_min), np.ceil(x_max) + 0.5, 0.5)
y_bins = np.arange(np.floor(y_min), np.ceil(y_max) + 0.5, 0.5)

# For angle deviation
pivot_table_angle = df.pivot_table(
    values='angle_deviation', 
    index=pd.cut(df['footprint_y'], bins=y_bins),
    columns=pd.cut(df['footprint_x'], bins=x_bins),
    aggfunc='mean'
)
sns.heatmap(pivot_table_angle, cmap='viridis', annot=False, ax=ax1)
ax1.set_title('Mean Angle Deviation by Robot Position (0.5m grid)')
ax1.set_xlabel('X Position (m)')
ax1.set_ylabel('Y Position (m)')

# For position deviation - convert to mm
pivot_table_pos = df.pivot_table(
    values='pos_deviation', 
    index=pd.cut(df['footprint_y'], bins=y_bins),
    columns=pd.cut(df['footprint_x'], bins=x_bins),
    aggfunc='mean'
) * 1000  # Convert to mm
sns.heatmap(pivot_table_pos, cmap='viridis', annot=False, ax=ax2)
ax2.set_title('Mean Position Deviation by Robot Position (0.5m grid)')
ax2.set_xlabel('X Position (m)')
ax2.set_ylabel('Y Position (m)')
cbar = ax2.collections[0].colorbar
cbar.set_label('Position Deviation (mm)')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '4_footprint_position_deviations.png'))

# Figure 5: Yaw Effect on Angle/Position Deviation
yaw_cols = ['yaw_' + n for n in YAW_CATEGORIES if 'yaw_' + n in df.columns]
if yaw_cols:
    plt.figure(figsize=(12, 10))
    
    # For angle deviation
    plt.subplot(2, 1, 1)
    yaw_data_angle = pd.melt(df, id_vars=['angle_deviation'], 
                           value_vars=yaw_cols,
                           var_name='yaw', value_name='is_yaw_direction')
    yaw_data_angle = yaw_data_angle[yaw_data_angle['is_yaw_direction'] == 1]
    yaw_data_angle['yaw_category'] = yaw_data_angle['yaw'].str.replace('yaw_', '')
    
    sns.boxplot(x='yaw_category', y='angle_deviation', data=yaw_data_angle)
    plt.title('Angle Deviation by Footprint Yaw Direction')
    plt.xlabel('Footprint Yaw Direction')
    plt.ylabel('Angle Deviation (rad)')
    
    # For position deviation
    plt.subplot(2, 1, 2)
    yaw_data_pos = pd.melt(df, id_vars=['pos_deviation'], 
                           value_vars=yaw_cols,
                           var_name='yaw', value_name='is_yaw_direction')
    yaw_data_pos = yaw_data_pos[yaw_data_pos['is_yaw_direction'] == 1]
    yaw_data_pos['yaw_category'] = yaw_data_pos['yaw'].str.replace('yaw_', '')
    # Convert pos_deviation to mm for plotting
    yaw_data_pos['pos_deviation_mm'] = yaw_data_pos['pos_deviation'] * 1000
    
    sns.boxplot(x='yaw_category', y='pos_deviation_mm', data=yaw_data_pos)
    plt.title('Position Deviation by Footprint Yaw Direction')
    plt.xlabel('Footprint Yaw Direction')
    plt.ylabel('Position Deviation (mm)')
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, '5_yaw_vs_deviations.png'))

# Feature importance analysis using Random Forest
# Drop one-hot encoded categorical columns to avoid multicollinearity
categorical_cols = [col for col in df.columns if col.startswith(('axis_', 'yaw_', 'height_'))]
logger.info(f"Dropping one-hot encoded columns: {categorical_cols}")

# Define the features and targets for both analyses
X = df.drop(['angle_deviation', 'pos_deviation', 'axis_label'] + categorical_cols, axis=1)
y_angle = df['angle_deviation']
y_pos = df['pos_deviation']

# Scale features
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)
X_scaled = pd.DataFrame(X_scaled, columns=X.columns)

# Split data for testing - angle deviation
X_train, X_test, y_train_angle, y_test_angle = train_test_split(
    X_scaled, y_angle, test_size=0.25, random_state=42)

# Split data for testing - position deviation
_, _, y_train_pos, y_test_pos = train_test_split(
    X_scaled, y_pos, test_size=0.25, random_state=42)

# Train Random Forest for angle deviation
rf_angle = RandomForestRegressor(n_estimators=100, random_state=42)
rf_angle.fit(X_train, y_train_angle)

# Train Random Forest for position deviation
rf_pos = RandomForestRegressor(n_estimators=100, random_state=42)
rf_pos.fit(X_train, y_train_pos)

# Evaluate angle deviation model
y_pred_angle = rf_angle.predict(X_test)
logger.info("\nRandom Forest Model Performance for Angle Deviation:")
logger.info(f"R² score: {r2_score(y_test_angle, y_pred_angle):.4f}")
logger.info(f"Mean squared error: {mean_squared_error(y_test_angle, y_pred_angle):.4f}")

# Evaluate position deviation model
y_pred_pos = rf_pos.predict(X_test)
logger.info("\nRandom Forest Model Performance for Position Deviation:")
logger.info(f"R² score: {r2_score(y_test_pos, y_pred_pos):.4f}")
logger.info(f"Mean squared error: {mean_squared_error(y_test_pos, y_pred_pos):.4f}")

# Calculate feature importance for angle deviation
feature_importance = pd.DataFrame({
    'Feature': X.columns,
    'Importance': rf_angle.feature_importances_
}).sort_values('Importance', ascending=False)

# Calculate feature importance for position deviation
feature_importance_pos = pd.DataFrame({
    'Feature': X.columns,
    'Importance': rf_pos.feature_importances_
}).sort_values('Importance', ascending=False)

logger.info("\nFeature Importance from Random Forest for Angle Deviation:")
logger.info(feature_importance)

logger.info("\nFeature Importance from Random Forest for Position Deviation:")
logger.info(feature_importance_pos)

# Feature importance for both angle and position deviation
plt.figure(figsize=(14, 10))

# For angle deviation
plt.subplot(2, 1, 1)
sns.barplot(x='Importance', y='Feature', data=feature_importance)
plt.title('Feature Importance for Predicting Angle Deviation')

# For position deviation
plt.subplot(2, 1, 2)
sns.barplot(x='Importance', y='Feature', data=feature_importance_pos)
plt.title('Feature Importance for Predicting Position Deviation')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '6_feature_importance_combined.png'))

# Calculate mutual information for angle deviation (non-linear relationships)
mi_scores_angle = mutual_info_regression(X_scaled, y_angle)
mi_df = pd.DataFrame({
    'Feature': X.columns,
    'MI Score': mi_scores_angle
}).sort_values('MI Score', ascending=False)

logger.info("\nMutual Information Scores for Angle Deviation (non-linear relationships):")
logger.info(mi_df)

# Calculate mutual information for position deviation
mi_scores_pos = mutual_info_regression(X_scaled, y_pos)
mi_df_pos = pd.DataFrame({
    'Feature': X.columns,
    'MI Score': mi_scores_pos
}).sort_values('MI Score', ascending=False)

logger.info("\nMutual Information Scores for Position Deviation (non-linear relationships):")
logger.info(mi_df_pos)

plt.figure(figsize=(14, 10))

# For angle deviation
plt.subplot(2, 1, 1)
sns.barplot(x='MI Score', y='Feature', data=mi_df)
plt.title('Mutual Information Scores for Predicting Angle Deviation')

# For position deviation
plt.subplot(2, 1, 2)
sns.barplot(x='MI Score', y='Feature', data=mi_df_pos)
plt.title('Mutual Information Scores for Predicting Position Deviation')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '7_mutual_info_combined.png'))

# Add ANOVA analysis for position deviation alongside angle deviation
# ANOVA analysis for both angle and position deviation by axis choice
logger.info("\nANOVA Analysis - Effect of axis choice on angle deviation:")
axis_0_angles = df[df['axis_0'] == 1]['angle_deviation']
axis_1_angles = df[df['axis_1'] == 1]['angle_deviation']
axis_2_angles = df[df['axis_2'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(axis_0_angles, axis_1_angles, axis_2_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.error("There is a statistically significant difference in angle deviation between axes.")
else:
    logger.info("No statistically significant difference in angle deviation between axes.")

logger.info("\nANOVA Analysis - Effect of axis choice on position deviation:")
axis_0_pos = df[df['axis_0'] == 1]['pos_deviation']
axis_1_pos = df[df['axis_1'] == 1]['pos_deviation']
axis_2_pos = df[df['axis_2'] == 1]['pos_deviation']

f_stat_pos, p_val_pos = stats.f_oneway(axis_0_pos, axis_1_pos, axis_2_pos)
logger.info(f"F-statistic: {f_stat_pos:.4f}, p-value: {p_val_pos:.4f}")
if p_val_pos < 0.05:
    logger.error("There is a statistically significant difference in position deviation between axes.")
else:
    logger.info("No statistically significant difference in position deviation between axes.")

# ANOVA analysis for both angle and position deviation by height category
logger.info("\nANOVA Analysis - Effect of height category on angle deviation:")
low_angles = df[df['height_low'] == 1]['angle_deviation']
mid_angles = df[df['height_mid'] == 1]['angle_deviation']
high_angles = df[df['height_high'] == 1]['angle_deviation']

f_stat, p_val = stats.f_oneway(low_angles, mid_angles, high_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.error("There is a statistically significant difference in angle deviation between height categories.")
else:
    logger.info("No statistically significant difference in angle deviation between height categories.")

logger.info("\nANOVA Analysis - Effect of height category on position deviation:")
low_pos = df[df['height_low'] == 1]['pos_deviation']
mid_pos = df[df['height_mid'] == 1]['pos_deviation']
high_pos = df[df['height_high'] == 1]['pos_deviation']

f_stat_pos, p_val_pos = stats.f_oneway(low_pos, mid_pos, high_pos)
logger.info(f"F-statistic: {f_stat_pos:.4f}, p-value: {p_val_pos:.4f}")
if p_val_pos < 0.05:
    logger.error("There is a statistically significant difference in position deviation between height categories.")
else:
    logger.info("No statistically significant difference in position deviation between height categories.")

# Analyze effect of CoM distance on both deviations
logger.info("\nANOVA Analysis - Effect of CoM distance on angle deviation:")
df['distance_category'] = pd.qcut(df['distance_com_to_polygon'], 4, labels=['Q1', 'Q2', 'Q3', 'Q4'])

# Group by distance quartile for angle deviation
q1_angles = df[df['distance_category'] == 'Q1']['angle_deviation']
q2_angles = df[df['distance_category'] == 'Q2']['angle_deviation']
q3_angles = df[df['distance_category'] == 'Q3']['angle_deviation']
q4_angles = df[df['distance_category'] == 'Q4']['angle_deviation']

f_stat, p_val = stats.f_oneway(q1_angles, q2_angles, q3_angles, q4_angles)
logger.info(f"F-statistic: {f_stat:.4f}, p-value: {p_val:.4f}")
if p_val < 0.05:
    logger.error("There is a statistically significant difference in angle deviation between CoM distance quartiles.")
else:
    logger.info("No statistically significant difference in angle deviation between CoM distance quartiles.")

logger.info("\nANOVA Analysis - Effect of CoM distance on position deviation:")
# Group by distance quartile for position deviation
q1_pos = df[df['distance_category'] == 'Q1']['pos_deviation']
q2_pos = df[df['distance_category'] == 'Q2']['pos_deviation']
q3_pos = df[df['distance_category'] == 'Q3']['pos_deviation']
q4_pos = df[df['distance_category'] == 'Q4']['pos_deviation']

f_stat_pos, p_val_pos = stats.f_oneway(q1_pos, q2_pos, q3_pos, q4_pos)
logger.info(f"F-statistic: {f_stat_pos:.4f}, p-value: {p_val_pos:.4f}")
if p_val_pos < 0.05:
    logger.error("There is a statistically significant difference in position deviation between CoM distance quartiles.")
else:
    logger.info("No statistically significant difference in position deviation between CoM distance quartiles.")

# Figure 8: Summary visualization with both metrics
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

# Angle deviation vs CoM distance
scatter1 = ax1.scatter(df['distance_com_to_polygon'], df['angle_deviation'], 
                     c=df['bar_height'], cmap='viridis', alpha=0.7)
ax1.set_xlabel('Distance from CoM to Support Polygon Center')
ax1.set_ylabel('Angle Deviation (rad)')
ax1.set_title('Distance CoM vs Angle Deviation')
fig.colorbar(scatter1, ax=ax1, label='Bar Height')

# Position deviation vs CoM distance - convert to mm
scatter2 = ax2.scatter(df['distance_com_to_polygon'], df['pos_deviation'] * 1000, 
                     c=df['bar_height'], cmap='viridis', alpha=0.7)
ax2.set_xlabel('Distance from CoM to Support Polygon Center')
ax2.set_ylabel('Position Deviation (mm)')
ax2.set_title('Distance CoM vs Position Deviation')
fig.colorbar(scatter2, ax=ax2, label='Bar Height')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '8_summary_visualization_combined.png'))

# Figure 9: CoM distance categories effect on both deviations
plt.figure(figsize=(12, 10))

# For angle deviation
plt.subplot(2, 1, 1)
sns.boxplot(x='distance_category', y='angle_deviation', data=df)
plt.title('Angle Deviation by Distance to Support Polygon (Quartiles)')
plt.xlabel('Distance from CoM to Support Polygon Center (Quartiles)')
plt.ylabel('Angle Deviation (rad)')

# For position deviation - convert to mm
plt.subplot(2, 1, 2)
# Create a new column for mm display
df['pos_deviation_mm'] = df['pos_deviation'] * 1000
sns.boxplot(x='distance_category', y='pos_deviation_mm', data=df)
plt.title('Position Deviation by Distance to Support Polygon (Quartiles)')
plt.xlabel('Distance from CoM to Support Polygon Center (Quartiles)')
plt.ylabel('Position Deviation (mm)')

plt.tight_layout()
plt.savefig(os.path.join(data_folder, '9_com_distance_categories_vs_deviations.png'))

# Figure 10: Correlation between angle and position deviation
plt.figure(figsize=(10, 6))
corr_angle_pos = df['angle_deviation'].corr(df['pos_deviation'])
logger.info(f"\nCorrelation between angle deviation and position deviation: {corr_angle_pos:.4f}")

# Use mm for position deviation
sns.scatterplot(x='angle_deviation', y='pos_deviation_mm', hue='axis_label', data=df)
plt.title(f'Angle Deviation vs Position Deviation (Correlation: {corr_angle_pos:.4f})')
plt.xlabel('Angle Deviation (rad)')
plt.ylabel('Position Deviation (mm)')
plt.savefig(os.path.join(data_folder, '10_angle_vs_pos_deviation.png'))

# Figure 11: Bar position error vector in tool0 frame
logger.info("\nAnalyzing bar position error vector in tool0 frame")

# Extract the position error vector data if it exists in the raw data
position_error_vectors = []
for entry in data:
    if 'bar_pos_error_vector_tool0' in entry:
        position_error_vectors.append({
            'x': entry['bar_pos_error_vector_tool0'][0],
            'y': entry['bar_pos_error_vector_tool0'][1],
            'z': entry['bar_pos_error_vector_tool0'][2],
            'index': len(position_error_vectors)
        })
    
if position_error_vectors:
    # Convert to DataFrame for easier analysis
    error_vector_df = pd.DataFrame(position_error_vectors)
    
    # Calculate statistics before outlier removal - convert to mm
    logger.info(f"Position error vectors before outlier removal: {len(error_vector_df)}")
    logger.info(f"X range before: {error_vector_df['x'].min():.5f} to {error_vector_df['x'].max():.5f}")
    logger.info(f"Y range before: {error_vector_df['y'].min():.5f} to {error_vector_df['y'].max():.5f}")
    logger.info(f"Z range before: {error_vector_df['z'].min():.5f} to {error_vector_df['z'].max():.5f}")
    
    # Identify outliers using IQR method for each dimension
    outlier_indices = set()
    
    for dim in ['x', 'y', 'z']:
        q1 = error_vector_df[dim].quantile(0.25)
        q3 = error_vector_df[dim].quantile(0.75)
        iqr = q3 - q1
        lower_bound = q1 - 1.5 * iqr
        upper_bound = q3 + 1.5 * iqr
        
        # Find indices of outliers in this dimension
        dim_outliers = error_vector_df[(error_vector_df[dim] < lower_bound) | (error_vector_df[dim] > upper_bound)].index
        if len(dim_outliers) > 0:
            logger.warning(f"Detected {len(dim_outliers)} outliers in {dim} dimension")
            outlier_indices.update(dim_outliers)
    
    # Log total outliers to be removed
    if outlier_indices:
        logger.warning(f"Removing {len(outlier_indices)} total rows with outliers in any dimension")
        
    # Keep only rows that don't have outliers in any dimension
    error_vector_df = error_vector_df.drop(list(outlier_indices))
    # Re-index after removing outliers
    error_vector_df['index'] = range(len(error_vector_df))
    
    # Log statistics after outlier removal
    logger.info(f"Position error vectors after outlier removal: {len(error_vector_df)}")
    logger.info(f"X range after: {error_vector_df['x'].min() * 1000:.2f} to {error_vector_df['x'].max() * 1000:.2f} mm")
    logger.info(f"Y range after: {error_vector_df['y'].min() * 1000:.2f} to {error_vector_df['y'].max() * 1000:.2f} mm")
    logger.info(f"Z range after: {error_vector_df['z'].min() * 1000:.2f} to {error_vector_df['z'].max() * 1000:.2f} mm")

    # Calculate average and std for each component - convert to mm
    avg_x = error_vector_df['x'].mean() * 1000
    std_x = error_vector_df['x'].std() * 1000
    avg_y = error_vector_df['y'].mean() * 1000
    std_y = error_vector_df['y'].std() * 1000
    avg_z = error_vector_df['z'].mean() * 1000
    std_z = error_vector_df['z'].std() * 1000

    # Log the statistics
    logger.info("Position error vector statistics in tool0 frame:")
    logger.info(f"X component - Average: {avg_x:.2f} mm, Std Dev: {std_x:.2f} mm")
    logger.info(f"Y component - Average: {avg_y:.2f} mm, Std Dev: {std_y:.2f} mm")
    logger.info(f"Z component - Average: {avg_z:.2f} mm, Std Dev: {std_z:.2f} mm")

    # Calculate the overall magnitude of the error vector - convert to mm
    magnitude = np.sqrt(error_vector_df['x']**2 + error_vector_df['y']**2 + error_vector_df['z']**2) * 1000
    avg_magnitude = magnitude.mean()
    std_magnitude = magnitude.std()
    logger.info(f"Overall magnitude - Average: {avg_magnitude:.2f} mm, Std Dev: {std_magnitude:.2f} mm")

    # Create a line plot with mm scale
    plt.figure(figsize=(12, 8))
    
    plt.plot(error_vector_df['index'], error_vector_df['x'] * 1000, label='X', marker='o', linestyle='-', alpha=0.7)
    plt.plot(error_vector_df['index'], error_vector_df['y'] * 1000, label='Y', marker='s', linestyle='-', alpha=0.7)
    plt.plot(error_vector_df['index'], error_vector_df['z'] * 1000, label='Z', marker='^', linestyle='-', alpha=0.7)
    
    plt.title('Bar Position Error in Tool0 Frame')
    plt.xlabel('Sample Index')
    plt.ylabel('Error (mm)')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, '11_bar_position_error_tool0_frame.png'))
else:
    logger.warning("No bar_pos_error_vector_tool0 data found in the input file")

logger.info("\nCombined angle and position deviation analysis complete.")