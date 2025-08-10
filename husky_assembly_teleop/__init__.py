import os
from ament_index_python import get_package_share_directory

DATA_DIRECTORY = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'data')
DESIGN_DATA_DIRECTORY = os.path.join(DATA_DIRECTORY, 'husky_assembly_design_study')
RECORD_DIRECTORY = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'recorded_data')