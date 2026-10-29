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
# * The gdrive folders sit at a different path on every PC (Insync mount name,
# * user name). The defaults below are the robot-control PC's; set the
# * environment variables DESIGN_DATA_DIRECTORY / EXPERIMENT_DATA_DIRECTORY to
# * point at your own copy without editing this file.
DESIGN_DATA_DIRECTORY = os.environ.get(
    'DESIGN_DATA_DIRECTORY',
    '/home/su/Insync/2025-03 Husky Assembly/data_design_study')
EXPERIMENT_DATA_DIRECTORY = os.environ.get(
    'EXPERIMENT_DATA_DIRECTORY',
    '/home/su/Insync/2025-03 Husky Assembly/data_experiment')
# * Calibration DATASETS (the dated <date>/ folders) live on the gdrive, not in
# * the repo. The analysis scripts, the config template (_data_template/) and
# * debug_utils/ stay in the repo under data/calibration_data/.
CALIBRATION_DATA_DIRECTORY = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'calibration_data')
RECORD_DIRECTORY = os.path.join(DATA_DIRECTORY, '..', 'recorded_data')

# ============================================================================
# * CALIBRATION DATASET -- the ONE file that chooses the dated folders
# ============================================================================
# Both dates below live here so you always see them together and cannot update
# one while forgetting the other. They are normally the SAME folder: you record
# data into it, then analyse it. Set them apart only when you deliberately want
# the live app writing into a new folder while you re-analyse an older capture.
#
# ! Each folder must already exist under CALIBRATION_DATA_DIRECTORY (gdrive) and
# ! must hold a config.yaml -- copy one from an older date folder (or the repo's
# ! data/calibration_data/_data_template/) when starting a session.

# Used by the LIVE APP (the running monitor):
#   * husky_world reads <date>/calibrated_transformation_<robot>.json for every
#     robot not listed in BASE_CALIBRATION_DATE_BY_ROBOT below
#   * husky_monitor reads the punch-tool TCP offsets from <date>/config.yaml
#   * the Record/Save buttons WRITE new j0 / j1 / validation / punch_validation
#     data into <date>/
CALIBRATION_DATE = '20261016' # the newest session: Cindy (0806) left arm, 2026-10-16

# * Base (mocap) calibration folder per robot, by registry name. The huskies are
# * calibrated on different days, so each one reads its own newest
# * calibrated_transformation_<serial>_rhino.json. A robot not listed here reads
# * CALIBRATION_DATE (none today: all three are listed). Only the base calibration
# * file is read from these folders; recording and config.yaml stay on CALIBRATION_DATE.
BASE_CALIBRATION_DATE_BY_ROBOT = {
    'Alice': '20260623',  # 0804, the newest Alice calibration
    'Belle': '20261013',  # 0805, the newest Belle calibration (recorded 2026-10-14)
    # 0806 left arm, re-calibrated 2026-10-16 (validation 1.6 mm RMS, punch 0.7 mm RMS).
    # * Fallback: '20260916' (a copy of the June 20260622 result; every session from
    # * September to 2026-10-15 ran under it). The two agree within 0.8 mm / 0.02 deg,
    # * so switching back is safe.
    'Cindy': '20261016',
}

# * The measured lab floor in the MoCap world (mocap_floor.py): a plane fitted to
# * probe points on the floor, saved as <stamp>_mocap_floor_calibration.json in
# * MOCAP_FLOOR_DIRECTORY. The floor is a property of the lab, not of a design, so
# * every problem's collision floor, the PyBullet floor grid and the viewer pages'
# * grid use it. A missing file falls back to the exported WalkableGround height.
# * 2026-10-27 (since then): 0.26 deg tilt, about -1 mm at the MoCap origin; 22 points,
# * 1.6 mm rms; its x, y and origin are fitted to the fixed floor points of
# * main_floor_points.json. The file also carries the cameras, the fixed points and a
# * relay-frame correction marked 'untrusted': the loader (mocap_floor.load_mocap_floor)
# * reads only kind, floor_from_mocap, the floor samples and the plane's rms / tilt.
# * Before: 20261022_161428 (0.49 deg, -2.5 mm at the origin, 10 points, 2.0 mm rms),
# * the floor the 2026-10-23 test and its as-built pages were built on.
MOCAP_FLOOR_DIRECTORY = os.path.join(CALIBRATION_DATA_DIRECTORY, 'mocap_floor')
MOCAP_FLOOR_CALIBRATION_FILE = os.path.join(
    MOCAP_FLOOR_DIRECTORY, '20261027_160043_mocap_floor_calibration.json')
# * Place the Rhino design on that floor (mocap_floor.MocapFloor.world_from_design): the
# * live monitor tilts and lifts every Rhino pose it reads (bars, joints, targets, the
# * room) so the design's flat WalkableGround lies on the measured floor, and the ground
# * bars' footers touch it. Poses measured by MoCap (robot bases, progress.json, markers)
# * never move. Off: export DESIGN_ON_MEASURED_FLOOR=0 (the design as exported).
DESIGN_ON_MEASURED_FLOOR = os.environ.get('DESIGN_ON_MEASURED_FLOOR', '1') != '0'
# * How far the design's ground (WalkableGround) is raised before it is laid on the floor,
# * mm: the whole design then sits this much LOWER against the real floor. 2026-10-23,
# * 261023: the feeler gauge read ~23 mm under B20's footers (the model said 15.5 mm), so
# * 23 brings them onto the real floor; by the model they sit 7.5 mm into it (the floor
# * there is lower than its fitted plane, or the bar sits lower in the tools). 0 = as
# * exported. Per run: export DESIGN_GROUND_RAISE_MM=<mm>. ! Re-check all three ground
# * bars (G20, G34, G46) after a change: the rigid insert has no force limit.
DESIGN_GROUND_RAISE_MM = float(os.environ.get('DESIGN_GROUND_RAISE_MM', '23.0'))

# Used by the OFFLINE SCRIPTS in data/calibration_data/ (0_ ... 4_ and the
# export/visualise helpers). config_loader.py re-exports this as its
# DEFAULT_DATE_FOLDER, which is the folder every load_config() call reads.
CALIBRATION_ANALYSIS_DATE = '20261016'

CALIBRATION_BATCHES = ['j0', 'j1', 'validation', 'punch_validation']

# DESIGN_PROBLEM_NAME = '260811_RobArch_demo'
# * Override per run with the environment variable DESIGN_PROBLEM_NAME, e.g.
# *   export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
DESIGN_PROBLEM_NAME = os.environ.get('DESIGN_PROBLEM_NAME', '261006_3bar_holding_test')
# * Where MoCapMarkers.json (the bars' MoCap ball table) is read from when the design
# * problem above has none of its own (a problem's own file always wins).
# ! For now 261016_2ModulesDemo_YH: 261006_3bar_holding_test has no MoCapMarkers.json, and
# ! its B20 / B21 sit at the same poses as in 261016 (checked 2026-10-20). Remove this
# ! borrowing once the running problem's Rhino export writes its own file.
# *   export MOCAP_MARKERS_PROBLEM_NAME=<problem> to borrow from another problem.
MOCAP_MARKERS_PROBLEM_NAME = os.environ.get('MOCAP_MARKERS_PROBLEM_NAME', '261016_2ModulesDemo_YH')
# DESIGN_PROBLEM_NAME = '2026-05-19_reoriented2'

# Rhino .3dm whose "Environment Obstacles" layer is drawn as the layout-diagram
# environment (0_/1_ --viewer). Change the filename here to switch environments,
# or override per-run with --env-3dm <path>.
DEFAULT_ENV_3DM = os.path.join(
    os.path.dirname(DESIGN_DATA_DIRECTORY), 'assembly - demo',
    '260715_phase1_test_v2.3dm')