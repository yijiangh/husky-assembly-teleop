import os

try:
    from ament_index_python import get_package_share_directory
except Exception:
    get_package_share_directory = None

def _get_data_directory():
    """
    Determine the data directory path.
    If running from source (development), use the source data directory.
    Otherwise, use the installed package data directory.
    """
    # Non-ROS fallback (e.g. Rhino CPython / standalone)
    if get_package_share_directory is None:
        local_data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'data'))
        if os.path.exists(local_data_dir):
            return local_data_dir
        return os.path.join(os.getcwd(), 'data')

    # Get the installed package data directory
    installed_data_dir = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'data')
    
    # Extract workspace path from installed directory
    # installed_data_dir = /home/yijiangh/ros2_ws/install/husky_assembly_teleop/share/husky_assembly_teleop/data
    # We want to extract: /home/yijiangh/ros2_ws/
     
    # Split the path and find the 'install' directory
    path_parts = installed_data_dir.split(os.sep)
    try:
        install_index = path_parts.index('install')
        # Everything before 'install' is the workspace path
        ws_path = os.sep.join(path_parts[:install_index])
        
        # Construct source data directory
        source_data_dir = os.path.join(ws_path, 'src', 'husky-assembly-teleop', 'data')
        
        if os.path.exists(source_data_dir):
            print(f"Using source data directory: {source_data_dir}")
            return source_data_dir
        else:
            print(f"Source data directory not found: {source_data_dir}")
            print(f"Using installed data directory: {installed_data_dir}")
            return installed_data_dir
            
    except ValueError:
        # 'install' not found in path, fallback to installed directory
        print(f"Could not extract workspace path from: {installed_data_dir}")
        print(f"Using installed data directory: {installed_data_dir}")
        return installed_data_dir

DATA_DIRECTORY = _get_data_directory()
DESIGN_DATA_DIRECTORY = '/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_design_study'
RECORD_DIRECTORY = os.path.join(DATA_DIRECTORY, '..', 'recorded_data')

CALIBRATION_DATE = '20260303'
CALIBRATION_BATCHES = ['j0', 'j1', 'validation', 'punch_validation']
VALIDATION_PROBLEM_NAME = '2026-05-08_dual-arm_transfer_test'