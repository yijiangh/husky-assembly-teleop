# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json, os
import logging, datetime
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import numpy as np
import pybullet_planning as pp

MARKER_NAME_PAIRS = [
    ['5', '6'],
    ['7', '8'],
    ['2', '4'],
    ['1', '3']
]

#DATA_BATCH = '20250509'
#DATA_BATCH = '20250516'
DATA_BATCH = '20250605'
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

for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    # Load the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    # TODO fix hndling of multiple files
    fig = plt.figure()
    
    pos_offset_data = []
    rot_offset_data = []
    # compute offset data
    for entry in data['raw_data']:
        left_EE_pose = entry.get("left_EE_pose", [])
        right_EE_pose = entry.get("right_EE_pose", [])
        
        left_EE_position = np.array(left_EE_pose[0])
        right_EE_position = np.array(right_EE_pose[0])
        
        pos_offset = left_EE_position - right_EE_position
        pos_offset_data.append(np.linalg.norm(pos_offset))

        left_EE_rot = R.from_quat(left_EE_pose[1])
        right_EE_rot = R.from_quat(right_EE_pose[1])
 
        rot_offset = right_EE_rot.inv() * left_EE_rot
        rot_offset_data.append(rot_offset.as_euler("xyz", degrees=True))
    
    mean_pos_offset = np.mean(pos_offset_data)
    logger.info('Mean positional offset %.4f', mean_pos_offset)
    pos_offset_data -= mean_pos_offset

    mean_rot_offset = np.mean(rot_offset_data, axis=0)
    logger.info('Mean rotational offset %.4f %.4f %.4f', *mean_rot_offset)
    rot_offset_data -= mean_rot_offset
    
    axp = fig.add_subplot(2,1, 1)

    axp.plot(pos_offset_data)
    axp.set_title(f'Positional Offset {i+1}')
    axp.set_xlabel('Sample Index')
    axp.set_ylabel('Offset (m)')
    axp.grid(True)

    axr = fig.add_subplot(2,1, 2)
    axr.plot(rot_offset_data)
    axr.set_title(f'Rotational Offset {i+1}')
    axr.set_xlabel('Sample Index')
    axr.set_ylabel('Offset (deg)')
    axr.legend(['X', 'Y', 'Z'])
    axr.grid(True)

    #plt.show()
    
    if EXPORT:
        plt.savefig(os.path.join(data_folder, f'dual_arm_acc_{DATA_BATCH}_{i}.png'))

if EXPORT:
    logger.info('Exported plot to %s', os.path.join(data_folder, f'dual_arm_acc_{DATA_BATCH}.png'))

plt.show()