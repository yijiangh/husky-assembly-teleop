# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json, os
import logging, datetime
import matplotlib.pyplot as plt
import numpy as np
import pybullet_planning as pp

MARKER_NAME_PAIRS = [
    ['5', '6'],
    ['7', '8'],
    ['2', '4'],
    ['1', '3']
]

DATA_BATCH = '20250509'
EXPORT = 1


HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, DATA_BATCH)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'dual_arm_acc_processing_log_{DATA_BATCH}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

json_files = [f for f in os.listdir(data_folder) if f.startswith('dual_arm_acc_') and f.endswith('.json')]

new_data = []
# accumulated data, for drawing
centers = []
fitted_lines = []
base_positions = []

fig = plt.figure()

for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    # Load the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    # TODO fix hndling of multiple files
    
    offset_data = []
    # compute offset data
    for entry in data['raw_data']:
        left_EE_pose = entry.get("left_EE_pose", [])
        right_EE_pose = entry.get("right_EE_pose", [])
        
        left_EE_position = np.array(left_EE_pose[0])
        right_EE_position = np.array(right_EE_pose[0])
        
        offset = left_EE_position - right_EE_position
        offset_data.append(np.linalg.norm(offset))
    
    mean = np.mean(offset_data)
    offset_data -= mean
    
    ax = fig.add_subplot()
    ax.plot(offset_data)

if EXPORT:
    plt.savefig(os.path.join(data_folder, f'dual_arm_acc_{DATA_BATCH}.png'))
    logger.info('Exported plot to %s', os.path.join(data_folder, f'dual_arm_acc_{DATA_BATCH}.png'))

plt.show()