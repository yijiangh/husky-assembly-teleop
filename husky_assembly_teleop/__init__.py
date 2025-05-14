import os
from ament_index_python import get_package_share_directory

DATA_DIRECTORY = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'data')
RECORD_DIRECTORY = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'recorded_data')