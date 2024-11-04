import os
from ament_index_python import get_package_share_directory

DATA_DIRECTORY = os.path.join(get_package_share_directory('pybullet_mocap'), 'data')
RECORD_DIRECTORY = os.path.join(get_package_share_directory('pybullet_mocap'), 'recorded_data')