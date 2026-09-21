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
    # Ask ROS where the package was installed. This fails in two cases, and both
    # fall back to the data folder sitting next to this source file:
    #   1. no ROS at all (Rhino CPython / a plain "python some_script.py" run)
    #   2. ROS is there but the package has not been built into an install space
    try:
        installed_data_dir = os.path.join(get_package_share_directory('husky_assembly_teleop'), 'data')
    except Exception:
        local_data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'data'))
        if os.path.exists(local_data_dir):
            return local_data_dir
        return os.path.join(os.getcwd(), 'data')

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
DESIGN_DATA_DIRECTORY = '/home/su/Insync/2025-03 Husky Assembly/data_design_study'
EXPERIMENT_DATA_DIRECTORY = '/home/su/Insync/2025-03 Husky Assembly/data_experiment'
RECORD_DIRECTORY = os.path.join(DATA_DIRECTORY, '..', 'recorded_data')

# ============================================================================
# * CALIBRATION DATASET -- the ONE file that chooses the dated folders
# ============================================================================
# Both dates below live here so you always see them together and cannot update
# one while forgetting the other. They are normally the SAME folder: you record
# data into it, then analyse it. Set them apart only when you deliberately want
# the live app writing into a new folder while you re-analyse an older capture.
#
# ! Each folder must already exist under data/calibration_data/ and must hold a
# ! config.yaml -- copy one from an older date folder when starting a session.

# Used by the LIVE APP (the running monitor):
#   * husky_world reads <date>/calibrated_transformation_<robot>.json
#   * husky_monitor reads the punch-tool TCP offsets from <date>/config.yaml
#   * the Record/Save buttons WRITE new j0 / j1 / validation / punch_validation
#     data into <date>/
CALIBRATION_DATE = '20260916'

# Used by the OFFLINE SCRIPTS in data/calibration_data/ (0_ ... 4_ and the
# export/visualise helpers). config_loader.py re-exports this as its
# DEFAULT_DATE_FOLDER, which is the folder every load_config() call reads.
CALIBRATION_ANALYSIS_DATE = '20260916'

CALIBRATION_BATCHES = ['j0', 'j1', 'validation', 'punch_validation']

# DESIGN_PROBLEM_NAME = '260811_RobArch_demo'
DESIGN_PROBLEM_NAME = '260716_phase1_test'
# DESIGN_PROBLEM_NAME = '2026-05-19_reoriented2'

# Rhino .3dm whose "Environment Obstacles" layer is drawn as the layout-diagram
# environment (0_/1_ --viewer). Change the filename here to switch environments,
# or override per-run with --env-3dm <path>.
DEFAULT_ENV_3DM = os.path.join(
    os.path.dirname(DESIGN_DATA_DIRECTORY), 'assembly - demo',
    '260715_phase1_test_v2.3dm')