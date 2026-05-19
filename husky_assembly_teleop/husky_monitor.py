"""
- Setting up the pybullet simulation
- Setting up the mocap client
- Updating the simulation state
- Handling user input
"""
import sys, re
print(f"Running with Python: {sys.executable}")

from collections import defaultdict
import os
import time, copy
import threading
import numpy as np

from typing import List, Tuple
from scipy.spatial.transform import Rotation as R

import rclpy
import rclpy.executors
from rclpy.node import Node

import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_DATA_DIRECTORY, CALIBRATION_BATCHES, DESIGN_PROBLEM_NAME, CALIBRATION_DATE
import husky_assembly_teleop.husky_world as world
import husky_assembly_teleop.mocap_experiment as mocap_experiment
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_from_markerset, bar_deviation_from_goal,
)
from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
from husky_assembly_teleop.common import (
    Button, Slider, SliderGroup, Husky, TrackedObject, HuskyObject, AssemblyObject, HUSKY_UR5e_JOINT_NAMES, lerp, load_gripper
)
from husky_assembly_teleop.optitrack.NatNetClient import NatNetClient
from husky_assembly_teleop.utils import (
    pose_from_frame, frame_from_pose, pose_from_transformation, transformation_from_pose,
    mocap_pos_y_up_to_z_up, mocap_quat_y_up_to_z_up,
    vec12_from_conf, conf_from_12vec, joint_trajectory_from_path, path_12_from_joint_trajectory,
    HUSKY_DUAL_ARM_HOME_CONF_12, HUSKY_DUAL_UR5e_JOINT_NAMES,
)

# BarAction (gdrive design-study) loading
from husky_assembly_teleop.bar_action_io import (
    parse_bar_action, list_bar_actions, find_movement, movement_type,
)
from husky_assembly_teleop.cfab_session import CfabSession
from compas_fab.backends import CollisionCheckError

DEFAULT_GREY = [0.2, 0.2, 0.2, 0.7]
GOAL_BLUE = [0, 0.2, 0.5, 0.7]
TRAJECTORY_GREEN = [0, 0.5, 0.2, 0.7]
TRANSPARENT = [0, 0.0, 0.0, 0.0]

EXISTING_ELEMENT_COLOR = pp.RED
CURRENT_ELEMENT_COLOR = pp.BLUE
DEFAULT_BAR_POS = pp.Point(0.8, 0, 1.3)

CLIENT_IP = '192.168.0.21' # Set to your own IP
MOCAP_IP = '192.168.0.117' # set to the mocap PC's IP, get this from Motive Settings>Streaming pane->Local interface

class HuskyMonitor(Node):
    USE_MOCAP = 0
    FAKE_HARDWARE = 0

    # When USE_MOCAP=1, by default the husky base in PyBullet tracks mocap.
    # Set USE_CELL_STATE_BASE_POSE=1 to override that and pin the base to
    # whatever was loaded from the goal RobotCellState's robot_base_frame
    # (or set via sliders). Useful for testing planning with mocap on for
    # end-effector tracking but the husky physically far from the assembly
    # scaffolding (e.g., at the lab desk during dual-arm accuracy tests).
    USE_CELL_STATE_BASE_POSE = 0
    USE_DPG_UI = 0   # 0 = legacy PyBullet debug GUI; 1 = Dear PyGui control panel
    UI_FONT_SIZE = 20  # DPG control-panel font size in px

    CALIBRATION = 0

    BAR_HOLDING_ACCURACY_TEST = 1
    DUAL_ARM_ACCURACY_TEST = 0

    # Mocap (y-up) -> z-up axis convention. See utils.mocap_pos_y_up_to_z_up.
    # 'rhino'   : rhino_x = mocap_x, rhino_y = -mocap_z, rhino_z = mocap_y (preferred).
    # 'rotated' : legacy convention previously hardcoded in receive_*_frame.
    MOCAP_AXIS_CONVENTION = "rhino"

    BOARD_VALIDATION = 0
    PUNCH_CALIB_VALIDATION = 0

    DUAL_ARM_KISSING = 0 # set 1 to enable kissing experiment + compliance controller buttons

    # When 1, HuskyRobotInterface creates the compliant-controller ROS interfaces
    # (target_wrench publishers, start_force_mode / zero_ftsensor / switch_controller
    # service clients). Off by default so we don't block startup waiting on
    # services that aren't running on most rigs.
    CONNECT_COMPLIANT_CONTROLLER = 1

    def __init__(self):
        super().__init__('husky_monitor')
        self.tick_timer = self.create_timer(0.05, self.update)

        # simple async tasks to be executed every tick
        self.tasks = []

        # Marks this instance as the live ROS-driven monitor (vs. a headless
        # test harness that bypasses __init__). Headless flows skip
        # _hide_cfab_robot since there's no overlapping pp-side husky.
        self._is_live_monitor = True

        self.huskies = []
        self.tracked_objects = []
        self.name_from_mocap_id = {}
        self._mocap_cache_lock = threading.Lock()
        self._mocap_rigidbody_cache = {}
        self._mocap_rigidbody_id_from_name = {}
        self._mocap_labeled_marker_cache = defaultdict(dict)
        self.mocap_experiment_recording = None
        self.mocap_experiment_last_output_path = None

        # Legacy pp-side scene state (used by free trajectory / calibration
        # code paths). The BarAction flow does NOT populate this; collision
        # checking for planning goes through monitor.cfab.planner.
        self.static_obstacles = {}
        self.active_bar_body = None       # legacy pp body; None on BarAction path
        self.active_bar_aabb_dims = None  # cached from rs RigidBody mesh on BarAction path
        self.active_bar_name = None
        self.active_extra_bodies = []     # legacy
        self.bar_from_extra = []          # legacy

        # BarAction / cfab planning state.
        self.cfab = None                       # CfabSession (lazy per problem)
        self.current_action = None             # rs_data_structure BarAssemblyAction
        self.current_movement = None           # selected Movement
        self.current_movement_index = None     # int
        self.movement_type = None              # "constrained" | "linear" | "free"
        self.movement_start_state = None       # compas_fab RobotCellState
        self.target_ee_frames = None           # {"left": Frame, "right": Frame} | None
        self.grasp_link_from_bar = None        # compas.geometry.Frame
        self.constrained_planner_stage = 3
        self.staging_free_trajectory = [None, None]   # left, right (per-arm tuples)
        self.constrained_trajectory = [None, None]
        self.constrained_display_mode = 0  # 0=FREE_STAGE, 1=CONSTRAINED
        self.constrained_start_conf = None  # 12-DOF target for manual staging
        self.constrained_goal_conf = None   # 12-DOF constrained-plan endpoint
        # cfab→pp bridge state for the BarAction planning path.
        self._bar_action_husky = None          # SimpleNamespace husky stub (cfab robot)
        self._bar_action_ghost_bodies = set()  # tiny invisible EE proxy pybullet bodies
        self._bar_action_cfab_id = None        # cfab client_id the ghosts belong to
        self.bar_action_staging_seed_conf = None  # feasible 12-DOF staging START seed
        self._bar_action_scrub = None          # scrub-slider state dict (Task C)
        self.assembly_objects = []
        self.current_seq_index = 0

        self.calibration_data = []
        self.marker_set_data = []
        self.dual_arm_EE_mocap_data = []
        self._bar_holding_fit_line_uids = []
        self.goal_base_pose_frozen = False
        self._current_action_path = None

        # Per-movement BarAction loader (replaces single-movement load_bar_action).
        self._loaded_action = None              # BarAssemblyAction | None
        self._loaded_movements = []             # list[Movement]; index 0 = synthetic M0
        self._selected_action_file_idx = 0
        self._selected_movement_idx = 0
        self._ee_target_pose_uids = []          # pp.add_line uids for drawn EE targets
        # Per-movement attached-body ghosts. The bodies are the ones cfab
        # already spawned via set_robot_cell_state; we just re-color them
        # TRAJECTORY_GREEN and re-pose them via goal_model FK each tick so
        # they ride along the trajectory preview.
        self._traj_ghost_bodies = []            # list[{'body','link','attach'}]
        self._traj_ghost_orig_colors = {}       # body puid -> RGBA (for restore)

        # UI
        self.buttons = []
        self.assembly_position_sliders = []
        self.joint_state_sliders = []
        self.assembly_goal_position_slider_group = None
        self.bar_goal_pose_slider_group = None
        self.bar_grasp_long_distance_slider = None
        self.dump_sep_sliders = []
        self.calib_joint_range_slider = None
        self.calib_target_axis_slider = None
        self.data_collection_mode_slider = None
        self.data_collection_mode = True  # True = data collection mode, False = validation mode
        self.calib_batch_slider = None
        self.selected_calib_batch_index = 0

        self.selected_robot_id = 0
        
        # Board validation mode variables
        self.board_validation_state_slider = None
        self.trajectory_selection_slider = None
        self.available_bar_actions = []
        self.selected_state_index = 1
        self.available_joint_trajectories = []  # Store available JointTrajectory files
        self.selected_trajectory_index = 0
        

        # goal and trajectory interface
        self.selected_arm_index = 0
        
        # Punch tool calibration validation
        default_punch_tool_offset = np.array([0.0, 0.0, 0.15], dtype=float)
        self.punch_tool_offsets = {
            0: default_punch_tool_offset.copy(),
            1: default_punch_tool_offset.copy(),
        }
        self.punch_tool_offset = self.punch_tool_offsets[self.selected_arm_index].copy()
        self.tool0_from_punch_tip = pp.Pose(point=self.punch_tool_offset)
        self.punch_validation_results = []

        self.goal_base_pose = (np.zeros(3), np.array([0, 0, 0, 1]))
        self.goal_gripper = 0.0
        self.gripper_slider = None
        self.goal_arm_pose = [np.zeros(6), np.zeros(6)]
        self.show_goal_state = True  

        self.goal_model = None
        self.goal_gripper_model = None

        self.base_from_goal_bar_pos = None
        self.world_from_goal_bar_euler = None
        self.goal_element = None 

        self.calib_tool_from_robot_arm_id = defaultdict(lambda: defaultdict(lambda: None))
        self.calib_joint_range = np.pi*2
        self.calib_target_axis = 0

        self.goal_bar_grasp = None
        self.grasp_distance = 0.0 # fixed for now
        self.goal_element_axis = 0

        self.trajectory_time_max = 20 if self.CALIBRATION else 120
        self.trajectory_time = self.trajectory_time_max

        # list of conf, velocity, total time, attachment other than the ee
        self.planned_arm_trajectory = [(None, None, None, None), (None, None, None, None)]
        self.free_arm_trajectory = None
        self.linear_arm_trajectory = None

        self.plan_traj_seg = None
        self.planned_base_trajectory = (None, None)

        # call setup code
        self.start_pybullet()
        if self.USE_MOCAP:
            self.start_mocap()

        # Load punch tool config before world.init so cone dimensions match the offset
        if self.PUNCH_CALIB_VALIDATION:
            self._load_punch_tool_config()

        # Initialize the UI backend BEFORE world.init / build_ui creates any widgets.
        from .ui_backend import make_backend
        from . import common as _common
        _common._global_backend = make_backend(
            use_dpg=bool(self.USE_DPG_UI),
            window_title="Husky Monitor",
            font_size=int(self.UI_FONT_SIZE),
        )

        world.init(self)

        # Load goal model after robots are created to ensure it matches the actual robot
        self.load_goal_model()

        # ! an inflated bar for goal
        goal_bar_body = pp.create_cylinder((0.025)/2, 1.0, mass=pp.STATIC_MASS)
        far_away_pose = pp.Pose(pp.Point(0,0,100))
        self.goal_element = AssemblyObject(self, 'b_goal', goal_bar_body, far_away_pose,
                                           pp.unit_pose())
        pp.set_color(self.goal_element.body, GOAL_BLUE)

        # Initialize board validation if enabled
        if self.BOARD_VALIDATION:
            self.available_bar_actions = self._load_available_bar_actions()
            self.available_joint_trajectories = self._load_available_joint_trajectories()
        
        self.build_ui()
        self.update_partial_assembly()
        self.update_goal_model_and_color()
        
    def add_tracked_object(self, obstacle: TrackedObject):
        """Registers an object to be tracked by mocap"""
        self.tracked_objects.append(obstacle)
        self.name_from_mocap_id[obstacle.mocap_id] = obstacle.name

    def add_assembly_objects(self, aobject: AssemblyObject):
        self.assembly_objects.append(aobject)

    def add_static_obstacles(self, pb_body, name):
        self.static_obstacles[name] = pb_body
        
    def add_husky(self, husky: Husky):
        """Registers a husky to connect to ROS and be tracked by mocap"""
        self.huskies.append(husky)
        self.name_from_mocap_id[husky.mocap_id] = husky.name

    def assign_calibration_tool_to_robot(self, robot_id, arm_id, tool_name):
        """Assigns a calibration tool to a robot's arm"""
        if robot_id < 0 or robot_id >= len(self.huskies):
            raise ValueError(f"Invalid robot_id: {robot_id}")
        self.calib_tool_from_robot_arm_id[robot_id][arm_id] = tool_name

    @property
    def active_calib_tool_name(self):
        """Returns the active calibration tool for the selected robot and arm"""
        return self.calib_tool_from_robot_arm_id[self.selected_robot_id][self.selected_arm_index]
        
    def set_base_trajectry(self, base_trajectory: Tuple[List[Tuple[np.ndarray, np.ndarray]], float]):
            """ set base trajectory for visualization"""
            self.planned_base_trajectory = base_trajectory
            
            # draw
            points = [
                pos for pos, _ in self.planned_base_trajectory[0]
            ]
            with pp.LockRenderer():
                with pp.HideOutput():
                    if self.plan_traj_seg is not None:
                       pp.remove_all_debug()
                    self.plan_traj_seg = pp.add_segments(points)
    
    def set_arm_trajectory(self, arm_trajectory, index=0):
        """ set arm trajectory for visualization"""
        # Tuple[List[np.ndarray], List[np.ndarray] | None, float], AssemblyObject
        # list of confs, list of velocities, total time, grasped element
        self.planned_arm_trajectory[index] = arm_trajectory

    def _reset_planned_arm_trajectory(self):
        # reset the planned arm trajectory to None
        self.planned_arm_trajectory = [(None, None, None, None), (None, None, None, None)]
        self.free_arm_trajectory = None
        self.linear_arm_trajectory = None

    def append_calibration_data(self, data):
        self.calibration_data.append(data)

    def _get_selected_trajectory_filename_suffix(self) -> str:
        """
        Return a filesystem-friendly suffix derived from the currently selected joint trajectory filename.
        Example: "ext_calib_0806_J1_traj0_JointTrajectory.json" -> "ext_calib_0806_J1_traj0_JointTrajectory"
        """
        # Prefer a cached attribute if present (set when loading / selecting trajectories)
        selected = getattr(self, "selected_trajectory_file", None)
        if not selected and getattr(self, "available_joint_trajectories", None):
            try:
                selected = self.available_joint_trajectories[self.selected_trajectory_index]
            except Exception:
                selected = None

        if not selected:
            return ""

        # Remove extension and sanitize to avoid problematic characters in filenames
        base = os.path.splitext(os.path.basename(str(selected)))[0]
        sanitized = re.sub(r"[^A-Za-z0-9_.-]+", "_", base).strip("_")
        return sanitized

    def update_calib_batch_index(self, value):
        self.selected_calib_batch_index = int(np.clip(int(value), 0, len(CALIBRATION_BATCHES) - 1))

    @property
    def selected_calib_batch(self):
        return CALIBRATION_BATCHES[self.selected_calib_batch_index]

    def record_calibration_data(self):
        if self.data_collection_mode:
            # In data collection mode, use the selected trajectory filename as suffix
            filename_suffix = self._get_selected_trajectory_filename_suffix()
        else:
            # In validation mode, use "validation" as suffix
            filename_suffix = "validation"
        world.save_calibration(self, filename_suffix=filename_suffix,
                               date_folder=CALIBRATION_DATE,
                               data_batch=self.selected_calib_batch)
        self.calibration_data = []

    def record_markerset_data(self):
        world.save_markerset_data(self)
        self.marker_set_data = []
        
    def reset_ui(self, target_conf=None):
        # reset all sliders to default value by recreating them...
        # pybullet seems to lack a setUserDebugParameter() method :(
        p.removeAllUserParameters()
        self.buttons.clear()
        self.assembly_position_sliders.clear()
        self.joint_state_sliders.clear()
        self.dump_sep_sliders.clear()
        self.build_ui(target_conf)
        
    def toggle_show_goal_state(self):
        self.show_goal_state = not self.show_goal_state
        self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)

    def set_to_show_goal_state(self):
        self.show_goal_state = False
        self.toggle_show_goal_state()

    def set_to_show_traj_state(self):
        self.show_goal_state = True
        self.toggle_show_goal_state()

    def update_selected_arm_id(self, arm_index):
        new_index = np.clip(int(arm_index), 0, self.get_active_arm_count() - 1)
        if new_index != self.selected_arm_index:
            self.selected_arm_index = new_index
            self._set_active_punch_tool_offset(new_index)
            self.reset_ui(target_conf=self.goal_arm_pose) #[self.selected_arm_index])

    def update_trajectory_time(self, time):
        self.trajectory_time = time

    def update_calib_joint_range(self, value):
        self.calib_joint_range = value

    def update_calib_target_axis(self, value):
        self.calib_target_axis = int(np.floor(value))

    def update_data_collection_mode(self, value):
        """Update data collection mode: 0 = validation mode, 1 = data collection mode"""
        self.data_collection_mode = bool(round(value))

    @staticmethod
    def _arm_name_from_index(arm_index):
        return 'left' if int(arm_index) == 0 else 'right'

    def get_punch_tool_offset(self, arm_index=None):
        arm_index = self.selected_arm_index if arm_index is None else int(arm_index)
        return np.array(self.punch_tool_offsets[arm_index], dtype=float)

    def get_tool0_from_punch_tip(self, arm_index=None):
        return pp.Pose(point=self.get_punch_tool_offset(arm_index))

    def _set_active_punch_tool_offset(self, arm_index=None):
        self.punch_tool_offset = self.get_punch_tool_offset(arm_index)
        self.tool0_from_punch_tip = pp.Pose(point=self.punch_tool_offset)

    def get_active_arm_count(self):
        if self.huskies:
            return 2 if self.huskies[self.selected_robot_id].dual_arm else 1
        return 2

    # --- Punch tool calibration validation ---
    def _load_punch_tool_config(self):
        """Load punch tool offset from config.yaml."""
        import yaml
        try:
            punch_config_path = os.path.join(
                DATA_DIRECTORY, 'calibration_data', CALIBRATION_DATE, 'config.yaml'
            )
            with open(punch_config_path, 'r') as f:
                config = yaml.safe_load(f) or {}

            punch_config = config.get('punch_tool') or {}
            updated_offsets = {
                arm_index: np.array(offset, dtype=float)
                for arm_index, offset in self.punch_tool_offsets.items()
            }

            legacy_offset = punch_config.get('offset_xyz')
            if legacy_offset is not None:
                legacy_offset = np.array(legacy_offset, dtype=float)
                updated_offsets = {
                    0: legacy_offset.copy(),
                    1: legacy_offset.copy(),
                }

            for arm_index, arm_name in enumerate(('left', 'right')):
                arm_config = punch_config.get(arm_name) or {}
                if 'offset_xyz' in arm_config:
                    updated_offsets[arm_index] = np.array(arm_config['offset_xyz'], dtype=float)

            self.punch_tool_offsets = updated_offsets
            self._set_active_punch_tool_offset(self.selected_arm_index)
            self.get_logger().info(
                'Loaded punch tool offsets: '
                f"left={self.punch_tool_offsets[0].tolist()}, "
                f"right={self.punch_tool_offsets[1].tolist()}"
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to load punch tool config: {e}')

    def record_punch_reference_pose(self):
        """Record the current punch tip pose in world frame via FK."""
        world.record_punch_reference(self, date_folder=CALIBRATION_DATE)

    def save_punch_validation_data(self):
        """Save all accumulated punch validation results to JSON."""
        world.save_punch_validation_data(self, date_folder=CALIBRATION_DATE)

    def record_raw_mocap_take(self):
        if not self.USE_MOCAP:
            self.get_logger().warn('MoCap experiment recording requires USE_MOCAP.')
            return
        if not hasattr(self, 'mocap_client') or not self.mocap_client.connected():
            self.get_logger().warn('MoCap client is not connected.')
            return
        if self.mocap_experiment_recording is not None:
            self.get_logger().warn('A MoCap experiment take is already recording.')
            return

        try:
            config_path, config = mocap_experiment.load_experiment_config()
        except Exception as exc:
            self.get_logger().error(f'Failed to load MoCap experiment config: {exc}')
            return

        selected_husky = self.huskies[self.selected_robot_id]
        output_paths = mocap_experiment.prepare_take_output(config)
        self.mocap_experiment_recording = {
            'config_path': config_path,
            'config': config,
            'output_paths': output_paths,
            'target_rigid_body': selected_husky.name,
            'selected_robot_id': int(self.selected_robot_id),
            'wall_start_time': time.monotonic(),
            'frames': [],
            'rigid_body_ids': {},
            'auto_reference_images': [],
            'mocap_camera_inventory': self.get_mocap_camera_inventory(refresh=True),
            'webcam_timelapse': None,
        }

        webcam_asset = mocap_experiment.capture_workspace_webcam_image(config, output_paths)
        if webcam_asset is not None:
            self.mocap_experiment_recording['auto_reference_images'].append(webcam_asset)
            if webcam_asset.get('status') == 'captured':
                self.get_logger().info(
                    f"Captured workspace image to "
                    f"{os.path.join(output_paths['session_dir'], webcam_asset['session_relative_path'])}"
                )
            else:
                self.get_logger().warn(
                    f"Workspace webcam capture failed: {webcam_asset.get('reason', 'unknown_error')}"
                )

        webcam_timelapse = mocap_experiment.start_workspace_webcam_timelapse(config, output_paths)
        self.mocap_experiment_recording['webcam_timelapse'] = webcam_timelapse
        if webcam_timelapse is not None and webcam_timelapse.get('status') == 'capture_failed':
            self.get_logger().warn(
                f"Workspace webcam timelapse failed to start: {webcam_timelapse.get('reason', 'unknown_error')}"
            )

        self.get_logger().info(
            f"Started raw MoCap take for '{selected_husky.name}' "
            f"({config['experiment']['duration_sec']:.1f}s) using {config_path}"
        )

    def test_webcam_capture(self):
        try:
            config_path, config = mocap_experiment.load_experiment_config()
        except Exception as exc:
            self.get_logger().error(f'Failed to load MoCap experiment config: {exc}')
            return

        test_config = copy.deepcopy(config)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        base_take_id = str(test_config.get('take', {}).get('take_id', '') or 'webcam_test')
        test_config['take']['take_id'] = f'{base_take_id}_webcam_test_{timestamp}'
        output_paths = mocap_experiment.prepare_take_output(test_config)
        webcam_asset = mocap_experiment.capture_workspace_webcam_image(test_config, output_paths)

        if webcam_asset is None:
            self.get_logger().warn('Webcam test capture is disabled in the current config.')
            return

        if webcam_asset.get('status') == 'captured':
            asset_path = os.path.join(output_paths['session_dir'], webcam_asset['session_relative_path'])
            self.get_logger().info(f'Webcam test capture saved to {asset_path}')
        else:
            self.get_logger().warn(
                f"Webcam test capture failed: {webcam_asset.get('reason', 'unknown_error')}"
            )

    def _record_raw_mocap_snapshot(self, timestamp, raw_snapshot, rigid_body_ids):
        recording = self.mocap_experiment_recording
        if recording is None:
            return

        elapsed_sec = time.monotonic() - recording['wall_start_time']
        frame_payload = {
            'timestamp': float(timestamp),
            'elapsed_sec': float(elapsed_sec),
            'rigid_bodies': {
                name: {
                    'position_m': [float(value) for value in pose[0]],
                    'quaternion_xyzw': [float(value) for value in pose[1]],
                }
                for name, pose in sorted(raw_snapshot.items())
            },
        }
        recording['frames'].append(frame_payload)
        recording['rigid_body_ids'].update({name: int(rb_id) for name, rb_id in rigid_body_ids.items()})
        recording['webcam_timelapse'] = mocap_experiment.step_workspace_webcam_timelapse(
            recording.get('webcam_timelapse'),
            elapsed_sec,
            recording['output_paths'],
        )

        if elapsed_sec >= recording['config']['experiment']['duration_sec']:
            self._finalize_raw_mocap_take(stop_reason='duration_elapsed')

    def _finalize_raw_mocap_take(self, stop_reason):
        recording = self.mocap_experiment_recording
        if recording is None:
            return

        webcam_timelapse_result = mocap_experiment.finalize_workspace_webcam_timelapse(
            recording.get('webcam_timelapse'),
            recording['output_paths'],
        )

        payload = mocap_experiment.build_take_payload(
            config=recording['config'],
            config_path=recording['config_path'],
            output_paths=recording['output_paths'],
            target_rigid_body=recording['target_rigid_body'],
            selected_robot_id=recording['selected_robot_id'],
            frames=recording['frames'],
            rigid_body_ids=recording['rigid_body_ids'],
            stop_reason=stop_reason,
            auto_reference_images=recording.get('auto_reference_images', []),
            mocap_camera_inventory=recording.get('mocap_camera_inventory'),
            webcam_timelapse=webcam_timelapse_result,
        )
        take_path = mocap_experiment.save_take_payload(
            payload=payload,
            take_path=recording['output_paths']['take_path'],
            manifest_path=recording['output_paths']['manifest_path'],
        )

        self.mocap_experiment_last_output_path = take_path
        self.mocap_experiment_recording = None
        if webcam_timelapse_result is not None and webcam_timelapse_result.get('status') == 'created':
            self.get_logger().info(
                f"Saved webcam timelapse to "
                f"{os.path.join(recording['output_paths']['session_dir'], webcam_timelapse_result['session_relative_path'])}"
            )
        self.get_logger().info(
            f"Saved raw MoCap take with {payload['frame_count']} frames to {take_path}"
        )

    def update_goal_align_axis(self, value):
        self.goal_element_axis = value

    def update_partial_assembly(self):
        for i, obj in enumerate(self.assembly_objects):
            if i <= self.current_seq_index:
                obj.show()
                pp.set_color(obj.body, EXISTING_ELEMENT_COLOR)
            else:
                obj.hide()
        pp.set_color(self.assembly_objects[self.current_seq_index].body, CURRENT_ELEMENT_COLOR)

        # if the partial assembly changes, the previously planned arm trajectory is invalidated
        self._reset_planned_arm_trajectory()

    def update_assembly_goal_position(self, centroid):
        for i, obj in enumerate(self.assembly_objects):
            obj.update_goal_pose((np.array(centroid) + obj.archived_goal_position, obj.goal_pose[1]))
        self.update_partial_assembly()

    def update_base_conf(self, base_conf):
        base_pose = pp.pose_from_base_values(base_conf)
        self.huskies[self.selected_robot_id].interface.position = base_pose[0]
        self.huskies[self.selected_robot_id].interface.rotation = base_pose[1]
        # # since we are teloperating the base, update the base goal pose
        # self.goal_pose = base_pose
        
        # if the base changes, the previously planned arm trajectory is invalidated
        self._reset_planned_arm_trajectory()

    def update_traj_goal_configuration(self):
        # goal_arm_pose is always length 2 (per __init__); slice for single-arm goal_model.
        arm_pose = self.goal_arm_pose if self.goal_model.dual_arm else self.goal_arm_pose[:1]
        self.goal_model.set_pose(self.goal_base_pose, arm_pose)

    def execute_linear_trajectory(self):
        # only execute part of the traj returned by transfer planning
        if self.linear_arm_trajectory is None:
            print('Linear arm trajectory is not planned!')
        else:
            self.execute_arm_trajectory(self.linear_arm_trajectory)

    def execute_free_trajectory(self):
        if self.free_arm_trajectory is None:
            print('Free arm trajectory is not planned!')
        else:
            self.execute_arm_trajectory(self.free_arm_trajectory)
    
    def execute_arm_trajectory(self, trajectory=None):
        # TODO merge dual arm execution into this one
        # Make a trajectory class that contains robot index info
        # Since we are already using compas_fab, consider extending their JointTrajectory class
        # https://compas.dev/compas_fab/latest/api/generated/compas_fab.robots.JointTrajectory.html
        if trajectory is None:
            trajectory = self.planned_arm_trajectory[self.selected_arm_index]

        if not self.FAKE_HARDWARE:
            world.execute_arm_trajectory(self, trajectory, index=self.selected_arm_index)
        else:
            # fake execution in sim
            if trajectory is None:
                self.get_logger().warn('Arm trajectory must be planed before executing!')
            else: 
                ho = self.huskies[self.selected_robot_id].object
                hi = self.huskies[self.selected_robot_id].interface
                if trajectory[3] is not None:
                    obj = trajectory[3]
                    gripper_tcp_from_object = obj.grasp

                for conf in trajectory[0]:
                    hi.arm_joint_pose[self.selected_arm_index] = conf
                    ho.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)

                    if trajectory[3] is not None:
                        # update attached object based on FK
                        world_from_tcp = ho.get_link_pose_from_name("ur_arm_tool0")
                        object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                        obj.set_pose(object_pose)
                    
                    hi.is_arm_executing = True
                    pp.wait_for_duration(0.01)

                hi.is_arm_executing = False

    def execute_arm_trajectory_with_servoing(self, trajectory=None):
        if trajectory is None:
            trajectory = self.planned_arm_trajectory[self.selected_arm_index]

        if self.FAKE_HARDWARE:
            self.logger.warn('Fake hardware does not support servoing!')
        else:
            # TODO make compatiable with dual arm
            world.execute_task_goal_arm_trajectory_with_servoing(self, trajectory, 
                                                                 log_data=0)

    def set_goal_joint_0_to_zero(self):
        self.goal_arm_pose[self.selected_arm_index][0] = 0.0
        self.reset_ui(self.goal_arm_pose)

    def sample_calib_traj(self):
        attachments = [ee[1] for ee in self.huskies[self.selected_robot_id].object.ee_list]
        obstacles = list(self.static_obstacles.values())
        packed_trajs = world.sample_calib_motion(self, int(self.selected_arm_index), int(self.calib_target_axis), self.calib_joint_range, 
                                                 attachments=attachments, obstacles=obstacles)

        if packed_trajs is not None:
            full_traj, transit_traj, calib_traj = packed_trajs
            self.set_arm_trajectory(full_traj, index=self.selected_arm_index)
            self.free_arm_trajectory = transit_traj
            self.linear_arm_trajectory = calib_traj
            self.set_to_show_traj_state()

    def execute_calib_traj(self):
        # if self.linear_arm_trajectory is None or self.free_arm_trajectory is None:
        #     self.get_logger().warn('Transit and calib trajectories must be planned before executing!')
        # else:
            # conf = self.planned_arm_trajectory[self.selected_arm_index][0].pop(0)
            # world.execute_arm_conf(self, conf, index=self.selected_arm_index)

        world.execute_arm_trajectory_and_record_each_conf(self, self.planned_arm_trajectory[self.selected_arm_index], index=self.selected_arm_index)
        self.record_calibration_data()

    def get_world_from_bar_goal_pose(self):
        world_from_base_link = self.goal_model.get_link_pose_from_name("base_footprint")
        world_pos = pp.multiply(world_from_base_link, pp.Pose(point=self.base_from_goal_bar_pos))[0]
        world_quat = pp.Pose(euler=pp.Euler(*self.world_from_goal_bar_euler))[1]
        return world_pos, world_quat

    def get_bar_action_goal_bar_pose(self):
        """world_from_bar from M2 cell state: target_ee_frames[side] ∘ attachment_frame.

        Returns ``(pos, quat)`` or ``None`` if no BarAction is loaded.
        """
        if self.movement_start_state is None or self.target_ee_frames is None:
            return None
        if not self.active_bar_name:
            return None
        rb_states = getattr(self.movement_start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name)
        if bar_rb is None or bar_rb.attachment_frame is None:
            return None
        attached_link = getattr(bar_rb, 'attached_to_link', '') or ''
        side = 'left' if 'left' in attached_link else 'right'
        target = self.target_ee_frames.get(side)
        if target is None:
            return None
        world_from_tool = pose_from_frame(target)
        tool_from_bar = pose_from_frame(bar_rb.attachment_frame)
        return pp.multiply(world_from_tool, tool_from_bar)

    def update_constrained_planner_stage(self, val):
        self.constrained_planner_stage = int(round(float(val)))

    def update_constrained_display_mode(self, val):
        self.constrained_display_mode = int(round(float(val)))
        self._refresh_constrained_displayed_trajectory()

    def _refresh_constrained_displayed_trajectory(self):
        src = self.constrained_trajectory if self.constrained_display_mode == 1 \
              else self.staging_free_trajectory
        if src[0] is not None and src[1] is not None:
            self.set_arm_trajectory(src[0], index=0)
            self.set_arm_trajectory(src[1], index=1)
            self.set_to_show_traj_state()

    def _goal_matches_constrained_start(self):
        """True when the current goal is the staged start of the constrained path."""
        start_conf = getattr(self, "constrained_start_conf", None)
        if start_conf is None:
            return False
        goal_conf = np.concatenate([
            np.asarray(self.goal_arm_pose[0], dtype=float),
            np.asarray(self.goal_arm_pose[1], dtype=float),
        ])
        return np.allclose(goal_conf, np.asarray(start_conf, dtype=float), atol=1e-4)

    def _capture_manual_staging_plan(self, arm_index=None):
        """Cache manual free plans in display slot 0 when they target constrained start."""
        if not self._goal_matches_constrained_start():
            return

        if arm_index is None:
            if self.planned_arm_trajectory[0][0] is None or self.planned_arm_trajectory[1][0] is None:
                return
            self.staging_free_trajectory = [
                copy.deepcopy(self.planned_arm_trajectory[0]),
                copy.deepcopy(self.planned_arm_trajectory[1]),
            ]
            self.constrained_display_mode = 0
            print("Cached manual both-arm staging plan as Display Traj = 0.")
            return

        arm_index = int(arm_index)
        if self.planned_arm_trajectory[arm_index][0] is None:
            return
        self.staging_free_trajectory[arm_index] = copy.deepcopy(
            self.planned_arm_trajectory[arm_index]
        )
        self.constrained_display_mode = 0
        print(f"Cached manual arm {arm_index} staging plan as Display Traj = 0.")

    def _set_goal_to_constrained_start(self):
        """Restore manual staging target to the constrained trajectory start."""
        start_conf = getattr(self, "constrained_start_conf", None)
        if start_conf is None:
            return
        start_conf = np.asarray(start_conf, dtype=float)
        self.goal_arm_pose[0] = start_conf[:6].copy()
        self.goal_arm_pose[1] = start_conf[6:].copy()
        self.update_traj_goal_configuration()

    def sample_random_goal_conf(self, max_attempts=200):
        """Sample a collision-free random arm conf for the active husky and
        stage it as ``goal_arm_pose``. Auto-adapts to single/dual arm via
        ``HuskyObject.get_arm_joint_names`` + ``husky.dual_arm``."""
        husky = self.huskies[self.selected_robot_id]
        ho = husky.object
        robot = ho.robot
        if husky.dual_arm:
            arm_specs = [('left_', 0), ('right_', 1)]
            joint_names = list(ho.get_arm_joint_names(0)) + list(ho.get_arm_joint_names(1))
            attachments = [ho.ee_list[0][1], ho.ee_list[1][1]]
        else:
            arm_specs = [('', 0)]
            joint_names = list(ho.get_arm_joint_names(0))
            attachments = [ho.ee_list[0][1]]

        # ACM: wrist links vs mounted tool body. Mirrors plan_transit_motion's
        # extra_disabled_collisions logic (utils.py:233-272). Without these,
        # the tool body collides with its own mount link / nearby wrist links.
        ee_types = getattr(ho, "ee_types", None) or []
        extra_disabled_collisions = []
        for arm_prefix, idx in arm_specs:
            attach = attachments[idx]
            ee_type = ee_types[idx] if idx < len(ee_types) else None
            wrist_links = ['ur_arm_wrist_3_link']  # mount link
            if isinstance(ee_type, str):
                if ee_type.startswith('assembly_tool_v3'):
                    wrist_links += ['ur_arm_wrist_2_link', 'ur_arm_wrist_1_link']
                elif ee_type == 'robotiq_gripper':
                    wrist_links += ['ur_arm_wrist_1_link']
            for wl in wrist_links:
                extra_disabled_collisions.append(
                    ((robot, pp.link_from_name(robot, arm_prefix + wl)),
                     (attach.child, pp.BASE_LINK))
                )

        joints = pp.joints_from_names(robot, joint_names)
        obstacles = list(self.static_obstacles.values())
        sample_fn = pp.get_sample_fn(robot, joints)
        collision_fn = pp.get_collision_fn(
            robot, joints,
            obstacles=obstacles,
            attachments=attachments,
            self_collisions=1,
            extra_disabled_collisions=extra_disabled_collisions,
            max_distance=0,
        )
        with pp.WorldSaver():
            for attempt in range(max_attempts):
                q = sample_fn()
                if not collision_fn(q):
                    if husky.dual_arm:
                        self.goal_arm_pose[0] = np.array(q[:6])
                        self.goal_arm_pose[1] = np.array(q[6:])
                    else:
                        self.goal_arm_pose[0] = np.array(q)
                    self.update_traj_goal_configuration()
                    self.get_logger().info(
                        f"Sampled collision-free goal conf in {attempt+1} attempts."
                    )
                    return
        self.get_logger().warn(
            f"No collision-free goal conf in {max_attempts} attempts."
        )

    def plan_single_arm_to_goal_action(self):
        """Plan selected arm, then cache it as manual staging if applicable."""
        self._set_goal_to_constrained_start()
        world.plan_arm_to_goal(self)
        self._capture_manual_staging_plan(self.selected_arm_index)

    def plan_both_arms_to_goal_action(self, use_composite=True, debug=False):
        """Plan both arms, then cache it as manual staging if applicable."""
        self._set_goal_to_constrained_start()
        world.plan_both_arms_to_goal(self, use_composite=use_composite, debug=debug)
        self._capture_manual_staging_plan()

    def plan_free_to_movement_start_with_cfab_cc(self):
        """Free dual-arm plan from CURRENT arm conf -> start_conf of the
        currently selected movement, using cfab's PyBulletCheckCollision.

        Analogous to plan_both_arms_to_goal_action (composite) but the goal
        is taken from mv.start_state.robot_configuration and collision
        checking is forced through cfab CC regardless of the
        use_cfab_collision_for_free toggle.
        """
        if self.current_movement is None:
            self.get_logger().warn("Load a movement first.")
            return
        mv = self.current_movement
        if mv.start_state is None or mv.start_state.robot_configuration is None:
            self.get_logger().warn(
                f"Movement {mv.movement_id!r} has no start_state.robot_configuration."
            )
            return
        husky = self.huskies[self.selected_robot_id]
        robot = husky.object.robot
        left_joints = pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_joints = pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        current_left = np.asarray(pp.get_joint_positions(robot, left_joints), dtype=float)
        current_right = np.asarray(pp.get_joint_positions(robot, right_joints), dtype=float)
        start_conf = np.concatenate([current_left, current_right])
        goal_conf = vec12_from_conf(mv.start_state.robot_configuration)

        scene = self._build_pp_scene_for_free()
        if scene is None:
            return
        cfab_cf = self._build_cfab_free_collision_fn(mv.start_state, force=True)
        if cfab_cf is None:
            self.get_logger().warn("cfab CC unavailable (planner not initialized?); aborting.")
            return

        from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm
        path, info = plan_free_dual_arm(
            scene, start_conf, goal_conf,
            max_time=120.0, max_iterations=1000,
            cfab_collision_fn=cfab_cf,
        )
        if path is None:
            self.get_logger().warn(
                f"plan_free→mv-start failed: {info.get('failure_reason', 'unknown')}"
            )
            return

        nL = len(left_joints)
        left_path = np.array([q[:nL] for q in path])
        right_path = np.array([q[nL:] for q in path])
        t = self.trajectory_time
        self.set_arm_trajectory((left_path, None, t, None), index=0)
        self.set_arm_trajectory((right_path, None, t, None), index=1)
        self.set_to_show_traj_state()
        print(f"[plan free→mv-start, cfab CC] OK: {mv.movement_id!r} "
              f"({len(path)} waypoints)")

    # --- --- --- --- --- BARACTION LOADING --- --- --- --- ---

    def load_bar_action(self, action_path=None, movement=0, *, update_goal_state=True):
        """Load one movement of a BarAssemblyAction via the cfab planner.

        Replaces the legacy ``load_board_validation_state`` flow. Scene
        materialization (rigid bodies, attached tool bodies, ACM) goes
        through ``self.cfab.planner.set_robot_cell_state(...)`` — no
        per-body pp spawning, no manual ACM translation.

        Parameters
        ----------
        action_path : str | None
            Absolute path or bare filename (resolved under
            ``DESIGN_DATA_DIRECTORY/<problem>/BarActions/``). If None, uses
            the slider-selected entry of ``available_bar_actions``.
        movement : int | str
            Integer index OR movement_id substring (e.g. ``"M1"``).
        update_goal_state : bool
            If True, refresh the UI's goal display after loading.

        Returns
        -------
        bool
            True on success, False otherwise.
        """
        # 1) Resolve action path.
        if action_path is None:
            if not self.available_bar_actions:
                print("No BarAction files available!")
                return False
            if self.selected_state_index >= len(self.available_bar_actions):
                print(f"Invalid BarAction index: {self.selected_state_index}")
                return False
            action_path = self.available_bar_actions[self.selected_state_index]
        if not os.path.isabs(action_path):
            action_path = os.path.join(
                DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME,
                'BarActions', action_path,
            )
        self._current_action_path = action_path

        for uid in getattr(self, '_bar_holding_fit_line_uids', []) or []:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._bar_holding_fit_line_uids = []

        print(f"Loading BarAction: {action_path}")

        # 2) Parse + resolve movement.
        try:
            action = parse_bar_action(action_path)
            idx, mv = find_movement(action, movement)
        except Exception as e:
            print(f"Error parsing BarAction: {e}")
            return False

        # 3) Ensure a cfab session for this problem.
        if self.cfab is None or self.cfab.problem_name != DESIGN_PROBLEM_NAME:

            if self.cfab is not None:
                self.cfab.close()
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                        connection_type="gui",
                                        enable_debug_gui=True,
                                        existing_client_id=existing_client_id)
                if existing_client_id is not None:
                    pp.CLIENTS.setdefault(existing_client_id, True)
            except Exception as e:
                print(f"Error initializing CfabSession for {DESIGN_PROBLEM_NAME}: {e}")
                self.cfab = None
                return False

        # Cfab's set_robot_cell loads its own husky URDF (+ tool URDFs) into
        # the shared GUI client, overlapping the real robot from world.init.
        # Hide them so the live scene reads cleanly. Collision/FK on the cfab
        # side still use these bodies. Idempotent on subsequent calls.
        # Skipped in headless tests where no pp-side husky overlaps.
        if getattr(self, '_is_live_monitor', False):
            self._hide_cfab_robot()

        if mv.start_state is None:
            print(f"Movement {mv.movement_id!r} has no start_state; skipping.")
            return False

        # 4) Reset monitor BarAction tracking fields.
        self.current_action = action
        self.current_movement = mv
        self.current_movement_index = idx
        self.movement_type = movement_type(mv)
        self.movement_start_state = mv.start_state
        self.target_ee_frames = mv.target_ee_frames or None
        self.active_bar_name = f"bar_{action.active_bar_id}"

        # Read grasp (= attachment_frame of the active bar in the gripper
        # link's frame). Same info already lives in start_state; we cache
        # for downstream planner consumers.
        rb_states = getattr(mv.start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name)
        if bar_rb is not None and bar_rb.attachment_frame is not None:
            self.grasp_link_from_bar = bar_rb.attachment_frame
        else:
            self.grasp_link_from_bar = None

        # 5) Push state into the cfab planner. This materializes all rigid
        # body poses, attaches tool bodies to their parent links, and sets
        # up the ACM internally.
        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"Error setting cfab robot cell state: {e}")
            return False

        # Bridge the loaded cfab scene into the pp-side state that
        # plan_and_stage_constrained consumes.
        try:
            self._bridge_cfab_to_pp_for_bar_action()
        except Exception as e:
            print(f"Error bridging cfab scene to pp for BarAction: {e}")
            return False

        # 6) Sanity-check the start state for collisions (non-fatal).
        try:
            self.cfab.planner.check_collision(
                mv.start_state,
                {"_skip_set_robot_cell_state": True,
                 "full_report": False, "verbose": False},
            )
            print(f"Start state of {mv.movement_id} is collision-free.")
        except CollisionCheckError as e:
            n_pairs = len(getattr(e, 'collision_pairs', None) or [])
            first = (e.message.splitlines()[0] if e.message else "(no message)")
            print(f"WARN: start state of {mv.movement_id} has "
                  f"{n_pairs} collision pair(s); continuing. First: {first}")

        # 7) Extract goal_arm_pose / goal_base_pose from start_state's
        # robot_configuration (for visualization + downstream IK seed).
        if hasattr(mv.start_state, 'robot_configuration') and \
                mv.start_state.robot_configuration is not None:
            robot_config = mv.start_state.robot_configuration
            if hasattr(robot_config, 'joint_values') and hasattr(robot_config, 'joint_names'):
                from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
                left_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
                right_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
                try:
                    self.goal_arm_pose[0] = np.array(
                        [robot_config[n] for n in left_arm_names])
                    self.goal_arm_pose[1] = np.array(
                        [robot_config[n] for n in right_arm_names])
                    if update_goal_state:
                        self.reset_ui(self.goal_arm_pose)
                except (KeyError, AttributeError) as e:
                    print(f"WARN: could not extract arm joint values: {e}")
        if hasattr(mv.start_state, 'robot_base_frame') and \
                mv.start_state.robot_base_frame is not None:
            self.goal_base_pose = pose_from_frame(mv.start_state.robot_base_frame)
            if self.BAR_HOLDING_ACCURACY_TEST:
                self.goal_base_pose_frozen = True

        # 7b) For BAR_HOLDING_ACCURACY_TEST, override goal_arm_pose with the IK
        # solution on target_ee_frames so the goal ghost reflects the target
        # EE pose (not the movement's start config, which can be identical
        # across adjacent movements: M2.start == M1.end etc).
        if self.BAR_HOLDING_ACCURACY_TEST and self.target_ee_frames is not None:
            from husky_assembly_teleop.husky_world import _solve_bar_action_goal_ik
            conf12 = _solve_bar_action_goal_ik(
                self, mv.start_state, skip_env_collisions=True, verbose=False,
            )
            if conf12 is not None:
                self.goal_arm_pose[0] = np.asarray(conf12[:6])
                self.goal_arm_pose[1] = np.asarray(conf12[6:])
                if update_goal_state:
                    self.reset_ui(self.goal_arm_pose)
                print(
                    f"BAR_HOLDING_ACCURACY_TEST: goal_arm_pose overridden from "
                    f"IK on target_ee_frames (movement {mv.movement_id})."
                )
            else:
                print(
                    f"WARN: IK on target_ee_frames failed for {mv.movement_id}; "
                    f"goal ghost falls back to start_state config."
                )

        if update_goal_state:
            self.set_to_show_goal_state()

        print(
            f"Loaded BarAction {action.action_id} "
            f"movement[{idx}]={mv.movement_id} ({self.movement_type}) "
            f"active_bar={action.active_bar_id} "
            f"rigid_bodies={len(self.cfab.client.rigid_bodies_puids)}"
        )
        return True

    def _hide_cfab_robot(self):
        """Tint the cfab-side robot URDF (red, alpha=0.5) so its pose updates
        from `set_robot_cell_state` are visible during cfab CC debugging.

        Tools stay transparent to avoid duplicating the real robot's tool
        meshes; only the cfab husky body links are tinted.
        """
        if self.cfab is None or self.cfab.client is None:
            return
        client = self.cfab.client
        if client.robot_puid is not None:
            pp.set_color(client.robot_puid, [1.0, 0.0, 0.0, 0.5])
        for tool_puid in (client.tools_puids or {}).values():
            pp.set_color(tool_puid, TRANSPARENT)

    def _bridge_cfab_to_pp_for_bar_action(self):
        """Wire the loaded cfab scene into the pp-side state that
        plan_and_stage_constrained consumes. Headless-equivalent of the
        bridge block in scripts/headless_live_monitor_test.py.

        Does NOT permanently change pp.CLIENT (the monitor's update() loop
        needs the monitor's own pp client). plan_and_stage_constrained does
        a temporary swap when it runs.
        """
        import pybullet as _pb
        import pybullet_planning as _pp
        from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE
        from types import SimpleNamespace

        client = self.cfab.client
        robot_puid = client.robot_puid
        cid = client.client_id

        # 1) Ghost EE proxy bodies (tiny invisible spheres) — recreate per
        #    cfab session. pp routes EE attachments through get_collision_fn,
        #    so the child must be a distinct body (robot-vs-robot collapses).
        if getattr(self, "_bar_action_cfab_id", None) != cid:
            col = _pb.createCollisionShape(_pb.GEOM_SPHERE, radius=0.001, physicsClientId=cid)
            ghost_L = _pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                          basePosition=[0.0, 0.0, -100.0], physicsClientId=cid)
            col2 = _pb.createCollisionShape(_pb.GEOM_SPHERE, radius=0.001, physicsClientId=cid)
            ghost_R = _pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=col2,
                                          basePosition=[0.0, 0.0, -100.0], physicsClientId=cid)
            self._bar_action_ghost_bodies = {ghost_L, ghost_R}
            self._bar_action_cfab_id = cid
            # Need pp.CLIENT == cid for link_from_name / Attachment below.
            _saved = _pp.CLIENT
            _pp.CLIENT = cid
            _pp.CLIENTS.setdefault(cid, True)
            try:
                left_tool_link = _pp.link_from_name(robot_puid, 'left_ur_arm_tool0')
                right_tool_link = _pp.link_from_name(robot_puid, 'right_ur_arm_tool0')
                identity_grasp = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
                self._bar_action_husky = SimpleNamespace(object=SimpleNamespace(
                    robot=robot_puid,
                    ee_list=[
                        (ghost_L, _pp.Attachment(robot_puid, left_tool_link, identity_grasp, ghost_L)),
                        (ghost_R, _pp.Attachment(robot_puid, right_tool_link, identity_grasp, ghost_R)),
                    ],
                ))
            finally:
                _pp.CLIENT = _saved

        # 2) Active bar + static obstacles (exclude the ghosts).
        ghosts = getattr(self, "_bar_action_ghost_bodies", set())
        puids = client.rigid_bodies_puids
        self.active_bar_body = (puids.get(self.active_bar_name) or [None])[0]
        self.static_obstacles = {
            n: ids[0] for n, ids in puids.items()
            if ids and n != self.active_bar_name and ids[0] not in ghosts
        }
        self.active_extra_bodies = []
        self.bar_from_extra = []
        self.active_bar_aabb_dims = self.get_active_bar_aabb_dims()

        # 3) Feasible staging seed near UR5e HOME (HOME self-collides ~2mm).
        home_dual = np.concatenate([UR5e_HOME_STATE, UR5e_HOME_STATE])
        self.bar_action_staging_seed_conf = self._sample_feasible_staging_seed(home_dual)

    def _sample_feasible_staging_seed(self, base_conf, *, max_attempts=200,
                                      perturb=0.6, seed=0):
        """Sample a collision-free 12-DOF config near `base_conf` for the
        staging plan's START. UR5e HOME sits ~2mm inside the dual-arm husky's
        self-collision margin, which makes plan_free_dual_arm reject it.

        Uses pp.get_collision_fn with self_collisions=1, max_distance=0 to
        match what plan_transit_motion checks internally.
        """
        import pybullet_planning as _pp
        from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

        all_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        cid = self.cfab.client.client_id
        robot = self.cfab.client.robot_puid

        _saved = _pp.CLIENT
        _pp.CLIENT = cid
        _pp.CLIENTS.setdefault(cid, True)
        try:
            all_joints = _pp.joints_from_names(robot, all_names)
            collision_fn = _pp.get_collision_fn(
                robot, all_joints,
                obstacles=[],
                attachments=[],
                self_collisions=1,
                max_distance=0,
            )
            base_conf = np.asarray(base_conf, dtype=float)
            rng = np.random.default_rng(seed)
            with _pp.WorldSaver():
                for attempt in range(max_attempts):
                    q = base_conf if attempt == 0 else (
                        base_conf + rng.uniform(-perturb, perturb, size=12))
                    if not collision_fn(tuple(q.tolist())):
                        if attempt > 0:
                            print(f"[seed] feasible staging seed sampled at attempt "
                                  f"{attempt+1}/{max_attempts} "
                                  f"(|Δ|={float(np.linalg.norm(q - base_conf)):.3f} rad).")
                        return q
            print(f"[seed] WARN: no collision-free staging seed in {max_attempts} "
                  f"attempts; falling back to base conf.")
            return base_conf
        finally:
            _pp.CLIENT = _saved

    def plan_and_stage_constrained_bar_action(self):
        """Run constrained planning; on success build cfab scrub sliders.

        Defaults below were tuned against the hard B226 floor-level case (see
        tasks/2026-05-15_dual_arm_rrt_b226_birrt.md). Single-tree RRT with one
        fixed home cannot find a plan there; BiRRT + multi-start (re-derive
        the home bar pose on failure) reliably does.

        WARNING: ``ignore_env_obstacles=True`` is a temporary stopgap that
        skips ALL environment/static obstacles in the constrained planner
        (only robot self-collision + attached-bar-vs-robot remain). Set to
        ``False`` before planning paths for real-hardware execution.
        """
        world.plan_and_stage_constrained(
            self,
            max_time=60.0,
            max_attempts=2,
            bidirectional=True,
            start_retries=6,
            ignore_env_obstacles=True,  # TODO: turn back on (False) before real-hardware runs
        )
        traj_c = self.constrained_trajectory
        if not (traj_c and traj_c[0] is not None and traj_c[1] is not None):
            self.get_logger().warn("Plan & Stage: no constrained trajectory produced.")
            return
        ctx = getattr(self, "_bar_action_plan_ctx", None) or {}
        n_pts = len(traj_c[0][0])
        print(f"[Plan & Stage] constrained trajectory: {n_pts} waypoints "
              f"(position_res={ctx.get('position_res')} m, "
              f"rotation_res={ctx.get('rotation_res')} rad)")
        if self.cfab is None or self.current_movement is None:
            return  # not a BarAction run; nothing cfab-side to scrub
        self._build_bar_action_scrub_sliders()

    def replan_free_from_live_base(self):
        """Replan free dual-arm from current mocap base; hide bar during exec."""
        world.plan_free_dual_arm_from_live_base(self)
        self._hide_goal_bar()

    def replan_constrained_from_live_base(self):
        """Replan constrained dual-arm from current mocap base."""
        world.plan_constrained_from_live_base(self)
        self._show_goal_bar()

    def _hide_goal_bar(self):
        if getattr(self, 'goal_gripper_model', None) is not None:
            pp.set_color(self.goal_gripper_model, TRANSPARENT)

    def _show_goal_bar(self):
        if getattr(self, 'goal_gripper_model', None) is not None:
            pp.set_color(self.goal_gripper_model, GOAL_BLUE)

    # --- --- --- --- --- PER-MOVEMENT BARACTION FLOW --- --- --- --- ---

    def _match_movement_role(self, mv):
        """Return 'M0' | 'M1' | 'M2' | 'M3' | 'M4' | None based on movement_id."""
        mid = getattr(mv, 'movement_id', '') or ''
        if mid == '__M0_synthetic_staging':
            return 'M0'
        for m in ('M1', 'M2', 'M3', 'M4'):
            if f'_{m}_' in mid:
                return m
        return None

    def _print_cfab_collision_check_setup(self, state, header='cfab CC setup'):
        """Pretty-print the Allowed-Collision-Matrix (ACM) that cfab's
        `check_collision` would apply at the given RobotCellState.

        The cfab checker runs 5 categories (see
        compas_fab/backends/pybullet/.../pybullet_check_collision.py):

          CC.1  robot link ↔ robot link
                SKIP if {a,b} in client.unordered_disabled_collisions (SRDF).
          CC.2  robot link ↔ tool
                SKIP if link_name in tool_state.touch_links, or tool hidden.
          CC.3  robot link ↔ rigid body
                SKIP if link_name in rb_state.touch_links, or rb hidden.
          CC.4  attached rigid body ↔ other rigid body
                SKIP if neither body is attached, hidden, or in the other's
                touch_bodies.
          CC.5  tool ↔ rigid body
                SKIP if rb attached to that tool, tool hidden, rb hidden,
                or tool in rb_state.touch_bodies.

        This dump tells you, at a glance, why a given pair WOULD be
        checked or skipped — useful when you see an obvious tool↔link
        overlap getting flagged: the tool's touch_links is missing that
        link.
        """
        if self.cfab is None or getattr(self.cfab, 'client', None) is None:
            print(f"[{header}] cfab session not initialized; skipping.")
            return
        client = self.cfab.client
        rc = client.robot_cell
        robot_name = getattr(getattr(rc, 'robot_model', None), 'name', None) or '?'
        n_links = len(client.robot_link_puids or {})
        tools_puids = client.tools_puids or {}
        bodies_puids = client.rigid_bodies_puids or {}
        tool_states = (state.tool_states or {}) if state is not None else {}
        rb_states = (state.rigid_body_states or {}) if state is not None else {}

        print(f"\n=== {header} ===")
        print(f"robot: '{robot_name}'  ({n_links} links)")
        print(f"tools loaded: {len(tools_puids)} | rigid bodies loaded: {len(bodies_puids)}")

        # CC.1
        disabled = getattr(client, 'unordered_disabled_collisions', None) or set()
        total_pairs = n_links * (n_links - 1) // 2 if n_links else 0
        print(f"\n[CC.1]  robot link ↔ robot link")
        print(f"  pairs:        {total_pairs}")
        print(f"  SRDF-skipped: {len(disabled)}")
        sample = list(disabled)[:6]
        for s in sample:
            a, b = sorted(s)
            print(f"    SKIP  {a}  <->  {b}")
        if len(disabled) > 6:
            print(f"    … +{len(disabled) - 6} more SRDF-disabled pair(s)")

        # CC.2
        print(f"\n[CC.2]  robot link ↔ tool")
        if not tools_puids:
            print(f"  (no tools loaded)")
        for tool_name in sorted(tools_puids):
            ts = tool_states.get(tool_name)
            if ts is None:
                print(f"  tool '{tool_name}': NO tool_state — every (link, tool) pair is checked")
                continue
            hidden = bool(getattr(ts, 'is_hidden', False))
            touch = sorted(getattr(ts, 'touch_links', None) or [])
            flag = " [HIDDEN — all CC.2 SKIP]" if hidden else ""
            print(f"  tool '{tool_name}'{flag}")
            print(f"    touch_links ({len(touch)}): {touch if touch else '∅'}")
            if not hidden:
                missing = sorted(set(client.robot_link_puids or {}) - set(touch))
                # Show only the closest robot-arm links to flag missing ACM
                # for tool-mounted geometry; full list is long.
                arm_link_keywords = (
                    'tool0', 'flange', 'wrist_3', 'wrist_2', 'wrist_1',
                    'forearm', 'upper_arm', 'shoulder', 'elbow',
                )
                missing_arm = [l for l in missing
                               if any(k in l for k in arm_link_keywords)]
                if missing_arm:
                    print(f"    arm-links NOT in touch_links (CC.2 will CHECK these against '{tool_name}'):")
                    for l in missing_arm:
                        print(f"      CHECK  {l}  <->  {tool_name}")

        # CC.3 / CC.4 / CC.5: per rigid body.
        print(f"\n[CC.3 / CC.4 / CC.5]  rigid bodies (state-attached / touch info)")
        if not rb_states:
            print(f"  (no rigid_body_states in state)")
        arm_link_keywords = (
            'tool0', 'flange', 'wrist_3', 'wrist_2', 'wrist_1',
            'forearm', 'upper_arm', 'shoulder', 'elbow',
        )
        all_links = list(client.robot_link_puids or {})
        for body_name in sorted(rb_states):
            rb = rb_states[body_name]
            hidden = bool(getattr(rb, 'is_hidden', False))
            att_link = getattr(rb, 'attached_to_link', None)
            att_tool = getattr(rb, 'attached_to_tool', None)
            touch_links = sorted(getattr(rb, 'touch_links', None) or [])
            touch_bodies = sorted(getattr(rb, 'touch_bodies', None) or [])
            flags = []
            if hidden:
                flags.append('HIDDEN')
            if att_link:
                flags.append(f"attached_to_link={att_link!r}")
            if att_tool:
                flags.append(f"attached_to_tool={att_tool!r}")
            tag = ('  [' + ', '.join(flags) + ']') if flags else ''
            print(f"  body '{body_name}'{tag}")
            print(f"    CC.3 touch_links  ({len(touch_links)}): "
                  f"{touch_links if touch_links else '∅'}")
            print(f"    CC.4/5 touch_bodies ({len(touch_bodies)}): "
                  f"{touch_bodies if touch_bodies else '∅'}")
            # For attached rigid bodies, surface the arm-side links that
            # are NOT in touch_links — those are the ones CC.3 will FLAG
            # the moment the body's mesh overlaps them by a hair. This is
            # almost always how a missing ACM entry shows up (e.g.
            # tool-mesh overlaps forearm/elbow on a folded-wrist pose).
            if att_link and not hidden:
                # Pick the "side" of the robot the body is mounted on
                # (left_/right_) so we only surface the relevant arm.
                side = None
                if att_link.startswith('left_'):
                    side = 'left_'
                elif att_link.startswith('right_'):
                    side = 'right_'
                missing_arm = [
                    l for l in all_links
                    if (side is None or l.startswith(side))
                    and any(k in l for k in arm_link_keywords)
                    and l not in touch_links
                ]
                if missing_arm:
                    print(f"    arm-links NOT in touch_links "
                          f"(CC.3 will CHECK these against '{body_name}'):")
                    for l in missing_arm:
                        print(f"      CHECK  {l}  <->  {body_name}")
        print(f"=== end {header} ===\n")

    def _make_synthetic_m0(self, m1_start_state):
        """Build a RoboticFreeMovement representing live->M1.start staging.

        start_state = deep copy of M1.start_state with robot_base_frame +
        robot_configuration overwritten to reflect the LIVE husky pose at
        the moment this is called.
        """
        from rs_data_structure.bar_action import RoboticFreeMovement
        state = m1_start_state.copy()
        # Diagnostic: one-shot dump of cfab's ACM at M1.start_state. Fires
        # only on the first M0 synthesis per cfab session so a per-movement
        # reload doesn't spam.
        if not getattr(self, '_cfab_acm_printed_for_cid', None) == getattr(
                getattr(self.cfab, 'client', None), 'client_id', None):
            try:
                self._print_cfab_collision_check_setup(
                    m1_start_state,
                    header="cfab CC setup @ M1.start_state",
                )
            except Exception as e:
                print(f"[cfab CC setup] ERROR: {e}")
            self._cfab_acm_printed_for_cid = getattr(
                getattr(self.cfab, 'client', None), 'client_id', None)
        hi = self.huskies[self.selected_robot_id].interface
        state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
        left = hi.arm_joint_pose[0]
        right = hi.arm_joint_pose[1] if len(hi.arm_joint_pose) > 1 else hi.arm_joint_pose[0]
        for n, v in zip(HUSKY_DUAL_UR5e_JOINT_NAMES[0], left):
            state.robot_configuration[n] = float(v)
        for n, v in zip(HUSKY_DUAL_UR5e_JOINT_NAMES[1], right):
            state.robot_configuration[n] = float(v)
        return RoboticFreeMovement(
            movement_id='__M0_synthetic_staging',
            tag='synthetic',
            start_state=state,
            target_ee_frames={},
        )

    def load_bar_action_file(self):
        """Parse the selected BarAction JSON; prepend synthetic M0; log roster."""
        files = self.available_bar_actions
        if not files:
            if hasattr(self, '_load_available_bar_actions'):
                self.available_bar_actions = self._load_available_bar_actions()
                files = self.available_bar_actions
        if not files:
            self.get_logger().warn("No BarAction files available.")
            return
        idx = max(0, min(self._selected_action_file_idx, len(files) - 1))
        fname = files[idx]
        action_path = fname if os.path.isabs(fname) else os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions', fname,
        )
        self._current_action_path = action_path
        self._loaded_action = parse_bar_action(action_path)
        m1_state = self._loaded_action.movements[0].start_state if self._loaded_action.movements else None
        if m1_state is None:
            self.get_logger().warn("BarAction has no movements with start_state; cannot prepend M0.")
            self._loaded_movements = list(self._loaded_action.movements)
        else:
            self._loaded_movements = [self._make_synthetic_m0(m1_state)] + list(self._loaded_action.movements)

        self.get_logger().info(f"Loading BarAction from file {action_path}")

        # Init cfab session + load the robot cell now so 'Load Movement' is
        # just a state push afterwards.
        if self.cfab is None:
            try:
                existing_client_id = pp.CLIENT if pp.is_connected() else None
                self.cfab = CfabSession(DESIGN_PROBLEM_NAME,
                                        connection_type="gui",
                                        enable_debug_gui=True,
                                        existing_client_id=existing_client_id)
                if existing_client_id is not None:
                    pp.CLIENTS.setdefault(existing_client_id, True)
            except Exception as e:
                print(f"Error initializing CfabSession: {e}")
                return
            if getattr(self, '_is_live_monitor', False):
                self._hide_cfab_robot()

        print(f"[BarAction] loaded {os.path.basename(action_path)} "
              f"with {len(self._loaded_movements)} movements (incl. synthetic M0):")
        for i, mv in enumerate(self._loaded_movements):
            print(f"  [{i}] {mv.movement_id!r} role={self._match_movement_role(mv)}")
        # Refresh UI so the Movement slider's range now matches the loaded
        # movement count (was 0..8 before; now 0..len(movements)-1).
        self.reset_ui(self.goal_arm_pose)

        # Auto-load any <mv>_trajectory.json that already exists under
        # Trajectories/, run consistency checks (start_conf agreement,
        # forward-chain handoff, M0 live-conf), drop any inconsistent
        # trajectories, and print the roster.
        self._auto_load_all_trajectories()

    def load_selected_movement(self):
        """Load the selected movement's start state into cfab + goal ghost."""
        if not self._loaded_movements:
            self.get_logger().warn("No BarAction loaded; click 'Load BarAction' first.")
            return
        idx = max(0, min(self._selected_movement_idx, len(self._loaded_movements) - 1))
        mv = self._loaded_movements[idx]

        # If M0, re-snapshot live conf/base into its start_state.
        if self._match_movement_role(mv) == 'M0' and len(self._loaded_movements) > 1:
            mv = self._make_synthetic_m0(self._loaded_movements[1].start_state)
            self._loaded_movements[0] = mv

        if mv.start_state is None:
            self.get_logger().warn(f"Movement {mv.movement_id!r} has no start_state.")
            return

        if self.cfab is None:
            self.get_logger().warn("cfab not initialized; click 'Load BarAction' first.")
            return

        self.current_action = self._loaded_action
        self.current_movement = mv
        self.current_movement_index = idx
        self.movement_type = movement_type(mv) if mv.movement_id != '__M0_synthetic_staging' else 'free'
        self.movement_start_state = mv.start_state
        self.target_ee_frames = mv.target_ee_frames or None
        bar_id = getattr(self._loaded_action, 'active_bar_id', None) if self._loaded_action else None
        self.active_bar_name = f"bar_{bar_id}" if bar_id else None

        # Restore previously-ghosted bodies' original colors before pushing
        # the new state (which may re-spawn or change which bodies are attached).
        for body, c in list(self._traj_ghost_orig_colors.items()):
            try:
                pp.set_color(body, c)
            except Exception:
                pass
        self._traj_ghost_bodies = []
        self._traj_ghost_orig_colors = {}

        try:
            self.cfab.planner.set_robot_cell_state(mv.start_state)
        except Exception as e:
            print(f"Error setting cfab robot cell state: {e}")
            return
        try:
            self._bridge_cfab_to_pp_for_bar_action()
        except Exception as e:
            print(f"Error bridging cfab scene to pp: {e}")
            return

        rb_states = getattr(mv.start_state, 'rigid_body_states', {}) or {}
        bar_rb = rb_states.get(self.active_bar_name) if self.active_bar_name else None
        self.grasp_link_from_bar = bar_rb.attachment_frame if (bar_rb and bar_rb.attachment_frame) else None

        # Collect attached-body ghosts (bar + any joint pieces). Color the
        # cfab-spawned body green; cache original RGBA so we can restore it
        # on next load.
        for name, rbs in rb_states.items():
            if getattr(rbs, 'attached_to_link', None) is None:
                continue
            if getattr(rbs, 'attachment_frame', None) is None:
                continue
            ids = (self.cfab.client.rigid_bodies_puids or {}).get(name) or []
            if not ids:
                continue
            body = ids[0]
            try:
                vis = p.getVisualShapeData(body)
                self._traj_ghost_orig_colors[body] = list(vis[0][7]) if vis else [0.7, 0.7, 0.7, 1.0]
            except Exception:
                self._traj_ghost_orig_colors[body] = [0.7, 0.7, 0.7, 1.0]
            try:
                pp.set_color(body, TRAJECTORY_GREEN)
            except Exception:
                pass
            self._traj_ghost_bodies.append({
                'body': body,
                'link': rbs.attached_to_link,
                'attach': pose_from_frame(rbs.attachment_frame),
            })
        if self._traj_ghost_bodies:
            print(f"[Movement] attached-body ghosts: "
                  f"{[g['link'] for g in self._traj_ghost_bodies]}")

        if mv.start_state.robot_configuration is not None:
            rc = mv.start_state.robot_configuration
            try:
                self.goal_arm_pose[0] = np.array(
                    [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[0]])
                self.goal_arm_pose[1] = np.array(
                    [rc[n] for n in HUSKY_DUAL_UR5e_JOINT_NAMES[1]])
            except (KeyError, AttributeError) as e:
                print(f"WARN: could not extract arm joint values: {e}")
        if mv.start_state.robot_base_frame is not None:
            self.goal_base_pose = pose_from_frame(mv.start_state.robot_base_frame)
            if self.BAR_HOLDING_ACCURACY_TEST:
                self.goal_base_pose_frozen = True

            # In FAKE_HARDWARE mode, teleport the real-robot base exactly to
            # the movement's start_state base. With FAKE_HARDWARE=0, leave
            # the live mocap reading to drive the real-robot base via
            # receive_mocap_frame.
            if self.FAKE_HARDWARE:
                hi = self.huskies[self.selected_robot_id].interface
                hi.position = np.asarray(self.goal_base_pose[0], dtype=float)
                hi.rotation = np.asarray(self.goal_base_pose[1], dtype=float)

        for uid in self._ee_target_pose_uids:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._ee_target_pose_uids = []
        if mv.target_ee_frames:
            for side, frame in mv.target_ee_frames.items():
                if frame is None:
                    continue
                pose = pose_from_frame(frame)
                uids = pp.draw_pose(pose, length=0.15)
                if uids:
                    self._ee_target_pose_uids.extend(uids if isinstance(uids, (list, tuple)) else [uids])

        self.reset_ui(self.goal_arm_pose)
        self.set_to_show_goal_state()

        print(f"[Movement] loaded [{idx}] {mv.movement_id!r} type={self.movement_type} "
              f"role={self._match_movement_role(mv)} "
              f"has_targets={bool(mv.target_ee_frames)} traj={mv.trajectory is not None}")

        # Auto-load saved trajectory (if present) so viz/exec is wired up
        # without a second click on 'Load Movement Trajectory'.
        if os.path.exists(self._trajectory_file_for(mv)):
            self.load_selected_movement_trajectory()

    def plan_selected_movement(self):
        """Dispatch the right planner for the loaded movement; store trajectory."""
        if self.current_movement is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        mv = self.current_movement
        role = self._match_movement_role(mv)
        if role is None:
            self.get_logger().warn(f"Unknown movement role for {mv.movement_id!r}; skipping.")
            return
        if mv.trajectory is not None:
            self.get_logger().warn(
                f"Overwriting existing trajectory for {mv.movement_id!r}"
            )

        dispatch = {
            'M0': self._plan_M0_dispatch,
            'M1': self._plan_M1_dispatch,
            'M2': self._plan_M2_dispatch,
            'M3': self._plan_M3_dispatch,
            'M4': self._plan_M4_dispatch,
        }[role]
        jt = dispatch(mv)
        if jt is None:
            self.get_logger().warn(f"Plan for {mv.movement_id!r} ({role}) FAILED.")
            if role == 'M1':
                self._clear_m1_start_conf_without_trajectory()
            return

        self._accept_trajectory(mv, jt, source='Plan', role=role, save_to_disk=True)

    def _accept_trajectory(self, mv, jt, *, source='Plan', role=None, save_to_disk=False):
        """Common post-step after a trajectory is either planned or loaded.

        Assigns mv.trajectory, propagates first/last conf to start states,
        wires the visualizer, optionally saves to disk, runs CDFM validation,
        and prints the movement roster.
        """
        mv.trajectory = jt
        path = path_12_from_joint_trajectory(jt)
        if path:
            chain_role = role if role is not None else self._match_movement_role(mv)
            start_vec = np.asarray(path[0], dtype=float)
            if chain_role in ('M2', 'M3') and mv.start_state is not None:
                existing = mv.start_state.robot_configuration
                if existing is None:
                    self.get_logger().warn(
                        f"{source} {mv.movement_id!r} has no propagated start_conf; "
                        "rejecting trajectory."
                    )
                    mv.trajectory = None
                    return
                diff = float(np.abs(start_vec - vec12_from_conf(existing)).max())
                if diff > 1e-3:
                    self.get_logger().warn(
                        f"{source} start of {mv.movement_id!r} differs from "
                        f"propagated start_conf by max {diff:.4f} rad/m; "
                        "rejecting trajectory."
                    )
                    mv.trajectory = None
                    return
            else:
                # M1 owns its generated start_conf; M0/M4 keep the legacy
                # behavior of mirroring trajectory start into start_state.
                mv.start_state.robot_configuration = conf_from_12vec(start_vec)

            # Step (3) forward-chain propagation — role-based:
            #   M1/M2/M3: strict chain owners; ALWAYS overwrite next.start
            #     with traj[-1] (warn first if there's an existing value).
            #   M0/M4:    NOT part of the chain. M0 stages live -> M1.start
            #     (M1 owns its own start_conf via its plan), M4 is the
            #     sequence terminator. Neither writes the next list-index
            #     movement's start_state.robot_configuration.
            if chain_role in ('M0', 'M4'):
                pass
            elif self.current_movement_index + 1 < len(self._loaded_movements):
                next_mv = self._loaded_movements[self.current_movement_index + 1]
                if next_mv.start_state is not None:
                    existing = next_mv.start_state.robot_configuration
                    new_end = conf_from_12vec(path[-1])
                    existing_vec = None
                    if existing is not None:
                        existing_vec = vec12_from_conf(existing)
                    elif self._trajectory_has_waypoints(next_mv):
                        # If next.start_state has not been populated yet, its
                        # loaded trajectory still owns the effective start.
                        existing_vec = path_12_from_joint_trajectory(next_mv.trajectory)[0]
                    if existing_vec is None:
                        next_mv.start_state.robot_configuration = new_end
                        print(
                            f"[{source}] propagated {mv.movement_id!r}.traj[-1] "
                            f"-> {next_mv.movement_id!r}."
                            f"start_state.robot_configuration (was None)."
                        )
                    else:
                        diff = np.abs(path[-1] - existing_vec).max()
                        if diff > 1e-3:
                            self.get_logger().warn(
                                f"{source} end of {mv.movement_id!r} differs from "
                                f"existing {next_mv.movement_id!r}.start by "
                                f"max {diff:.4f} rad/m; overwriting "
                                f"(M1/M2/M3 chain rule)."
                            )
                            if chain_role == 'M1':
                                self._drop_m2_m3_after_m1_chain_break(
                                    f"{source} M1 endpoint changed by max {diff:.4f} rad/m"
                                )
                            elif chain_role == 'M2' and self._match_movement_role(next_mv) == 'M3':
                                self._drop_movement_trajectory(
                                    next_mv,
                                    f"{source} M2 endpoint changed by max {diff:.4f} rad/m"
                                )
                        next_mv.start_state.robot_configuration = new_end

            # Backward continuity check: previous movement's last traj point
            # should match this movement's first traj point.
            if self.current_movement_index > 0:
                prev_mv = self._loaded_movements[self.current_movement_index - 1]
                prev_jt = getattr(prev_mv, 'trajectory', None)
                if prev_jt is not None:
                    prev_path = path_12_from_joint_trajectory(prev_jt)
                    if prev_path:
                        diff = float(np.abs(
                            np.asarray(prev_path[-1]) - np.asarray(path[0])
                        ).max())
                        if diff > 1e-3:
                            self.get_logger().warn(
                                f"{source} start of {mv.movement_id!r} differs "
                                f"from {prev_mv.movement_id!r}.trajectory[-1] "
                                f"by max {diff:.4f} rad/m."
                            )
                        else:
                            print(
                                f"[{source}] start agrees with "
                                f"{prev_mv.movement_id!r}.trajectory[-1] "
                                f"(max diff {diff:.6f})."
                            )

        self.planned_arm_trajectory = [
            (np.asarray([q[:6] for q in path]), None, self.trajectory_time, None),
            (np.asarray([q[6:] for q in path]), None, self.trajectory_time, None),
        ]
        self.set_to_show_traj_state()
        tag = f"{source}{' ' + role if role else ''}"
        print(f"[{tag}] {mv.movement_id!r}: {len(path)} waypoints stored.")
        self._validate_cdfm_planned_path(mv, path)

        if save_to_disk:
            try:
                from compas.data import json_dump
                out_path = self._trajectory_file_for(mv)
                os.makedirs(os.path.dirname(out_path), exist_ok=True)
                json_dump(jt, out_path)
                print(f"[{tag}] saved trajectory to {out_path}")
            except Exception as e:
                self.get_logger().warn(f"failed to save trajectory: {e}")

        self._print_movement_roster(tag=tag)

    def load_selected_movement_trajectory(self):
        """Load the planned trajectory JSON for the currently selected movement.

        Reads from ``<DESIGN_DATA_DIRECTORY>/<problem>/Trajectories/<movement_id>_trajectory.json``
        (the path plan_selected_movement writes on save). Runs the same
        post-acceptance steps as plan_selected_movement minus the save.
        """
        if self.current_movement is None:
            self.get_logger().warn("No movement loaded; click 'Load Movement' first.")
            return
        mv = self.current_movement
        traj_path = self._trajectory_file_for(mv)
        if not os.path.exists(traj_path):
            self.get_logger().warn(
                f"No trajectory file for {mv.movement_id!r} at {traj_path}"
            )
            return
        try:
            from compas.data import json_load
            jt = json_load(traj_path)
        except Exception as e:
            self.get_logger().warn(f"Failed to load {traj_path}: {e}")
            return
        if mv.trajectory is not None:
            self.get_logger().warn(
                f"Overwriting existing in-memory trajectory for {mv.movement_id!r}"
            )
        print(f"[LoadTraj] loaded {traj_path}")
        self._accept_trajectory(
            mv, jt,
            source='LoadTraj',
            role=self._match_movement_role(mv),
            save_to_disk=False,
        )

    def _trajectory_file_for(self, mv):
        """Disk path for a movement's saved trajectory JSON.

        Synthetic / role-only movement ids (e.g. `__M0_synthetic_staging`)
        get the active BarAction's action_id prepended so the same M0
        from different BarAction runs doesn't share one file and silently
        clobber each other (and so auto-load reload picks up the right
        M0 for the current BarAction).
        """
        action_id = getattr(self._loaded_action, 'action_id', None) if self._loaded_action else None
        return self._trajectory_path_for_movement_id(mv.movement_id, action_id)

    def _trajectory_path_for_movement_id(self, movement_id, action_id=None):
        """Build the saved trajectory path for one movement id."""
        # Keep M0 and every real movement scoped to the BarAction action_id.
        name = movement_id
        if action_id and not name.startswith(f'{action_id}_'):
            name = f'{action_id}_{name}'
        return os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'Trajectories',
            f'{name}_trajectory.json',
        )

    def _selected_bar_action_path(self):
        """Return the BarAction path selected by the BarAction file slider."""
        files = self.available_bar_actions
        if not files:
            self.available_bar_actions = self._load_available_bar_actions()
            files = self.available_bar_actions
        if not files:
            return None

        # The Bar Holding flow uses _selected_action_file_idx.
        idx = max(0, min(int(self._selected_action_file_idx), len(files) - 1))
        fname = files[idx]
        if os.path.isabs(fname):
            return fname
        return os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions', fname,
        )

    def delete_saved_movement_trajectories_for_current_bar_action(self):
        """Delete all saved per-movement trajectory JSONs for selected BarAction."""
        action_path = self._selected_bar_action_path()
        if action_path is None:
            self.get_logger().warn("No BarAction files available to delete trajectories for.")
            return

        loaded_path = getattr(self, '_current_action_path', None)
        is_loaded_action = (
            loaded_path is not None
            and os.path.abspath(loaded_path) == os.path.abspath(action_path)
            and self._loaded_action is not None
        )

        try:
            action = self._loaded_action if is_loaded_action else parse_bar_action(action_path)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse BarAction for trajectory delete: {e}")
            return

        action_id = getattr(action, 'action_id', None)
        movements = self._loaded_movements if is_loaded_action else list(action.movements)
        movement_ids = [getattr(mv, 'movement_id', None) for mv in movements]
        if not is_loaded_action and movement_ids:
            # The synthetic M0 save file is created by load_bar_action_file().
            movement_ids.insert(0, '__M0_synthetic_staging')

        deleted = []
        missing = 0
        errors = []
        seen_paths = set()
        for movement_id in movement_ids:
            if not movement_id:
                continue
            traj_path = self._trajectory_path_for_movement_id(movement_id, action_id)
            if traj_path in seen_paths:
                continue
            seen_paths.add(traj_path)
            if not os.path.exists(traj_path):
                missing += 1
                continue
            try:
                os.remove(traj_path)
                deleted.append(traj_path)
            except OSError as e:
                errors.append((traj_path, e))

        if is_loaded_action:
            # Keep UI state honest after disk files are removed.
            for mv in self._loaded_movements:
                mv.trajectory = None
            self._clear_m1_start_conf_without_trajectory()
            self._reset_planned_arm_trajectory()
            self.set_to_show_goal_state()

        print(f"[delete-traj] BarAction {action_id!r}: deleted {len(deleted)} "
              f"saved trajectory file(s), missing {missing}.")
        for traj_path in deleted:
            print(f"  deleted: {traj_path}")
        for traj_path, err in errors:
            self.get_logger().warn(f"Failed to delete {traj_path}: {err}")
        if is_loaded_action:
            self._print_movement_roster(tag='delete-traj')

    def _print_movement_roster(self, tag='roster'):
        """Print which loaded movements have a start_conf and a trajectory."""
        print(f"[{tag}] movement roster:")
        for i, m in enumerate(self._loaded_movements):
            has_conf = (m.start_state is not None
                        and getattr(m.start_state, 'robot_configuration', None) is not None)
            has_traj = getattr(m, 'trajectory', None) is not None
            print(f"  [{i}] {m.movement_id!r}")
            print(f"     - start state: has robot_conf = {self._color_bool(has_conf)}")
            print(f"     - has trajectory = {self._color_bool(has_traj)}")

    def _trajectory_has_waypoints(self, mv):
        """Return True only when a movement has a non-empty 12-DOF trajectory."""
        jt = getattr(mv, 'trajectory', None)
        if jt is None:
            return False
        try:
            return bool(path_12_from_joint_trajectory(jt))
        except Exception:
            # If parsing fails, treat any raw points as a trajectory so stale
            # files still get invalidated instead of being silently kept.
            return bool(getattr(jt, 'points', None))

    def _drop_movement_trajectory(self, mv, reason, *, delete_file=True):
        """Clear a movement trajectory in memory and remove its saved JSON."""
        had_traj = getattr(mv, 'trajectory', None) is not None
        mv.trajectory = None
        if self._match_movement_role(mv) == 'M1':
            # M1 start_conf is generated by M1 planning; without M1 traj it is
            # stale by definition and must not survive as an authored start.
            if mv.start_state is not None:
                mv.start_state.robot_configuration = None

        deleted = False
        if delete_file:
            traj_path = self._trajectory_file_for(mv)
            if os.path.exists(traj_path):
                try:
                    os.remove(traj_path)
                    deleted = True
                except OSError as e:
                    self.get_logger().warn(f"Failed to delete stale trajectory {traj_path}: {e}")
        if had_traj or deleted:
            print(f"[drop-traj] {mv.movement_id!r}: {reason}")
            if deleted:
                print(f"  deleted: {self._trajectory_file_for(mv)}")

    def _drop_m2_m3_after_m1_chain_break(self, reason):
        """Drop stale downstream linear trajectories after M1 endpoint changes."""
        dropped = 0
        for m in self._loaded_movements:
            if self._match_movement_role(m) in ('M2', 'M3') and self._trajectory_has_waypoints(m):
                self._drop_movement_trajectory(m, reason)
                dropped += 1
        return dropped

    def _clear_m1_start_conf_without_trajectory(self):
        """Keep invariant: M1 has start_conf only when it has a trajectory."""
        for m in self._loaded_movements:
            if self._match_movement_role(m) != 'M1':
                continue
            if m.start_state is None or self._trajectory_has_waypoints(m):
                continue
            if getattr(m.start_state, 'robot_configuration', None) is not None:
                m.start_state.robot_configuration = None
                print(f"[M1] cleared start_state.robot_configuration because M1 has no trajectory.")

    def _auto_load_all_trajectories(self, tol_rad: float = 1e-3):
        """After 'Load BarAction': try to load <mv>_trajectory.json for every
        loaded movement, then run consistency checks. On any mismatch, warn
        and drop that movement's trajectory.

        Per-movement checks (in load order, so check (b) can consult a
        predecessor's just-loaded or just-dropped trajectory):
          (a) traj[0] vs mv.start_state.robot_configuration
          (b) mv.start_state.robot_configuration vs prev movement's traj[-1]
          (c) For M0 only: live robot arm conf vs M0.start_state's
              robot_configuration. Saved M0 filename now carries the active
              BarAction prefix (see _trajectory_file_for) so a stale file
              from a different run can't sneak in across BarActions.
        """
        from compas.data import json_load

        traj_dir = os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'Trajectories',
        )
        if not os.path.isdir(traj_dir):
            print(f"[auto-load-traj] no Trajectories/ at {traj_dir}; nothing to load.")
            self._clear_m1_start_conf_without_trajectory()
            return

        # Pass 1: try to load each movement's trajectory from disk.
        n_loaded = 0
        for mv in self._loaded_movements:
            traj_path = self._trajectory_file_for(mv)
            if not os.path.exists(traj_path):
                continue
            try:
                jt = json_load(traj_path)
            except Exception as e:
                self.get_logger().warn(
                    f"[auto-load-traj] {mv.movement_id!r}: failed to load: {e}"
                )
                continue
            mv.trajectory = jt
            n_loaded += 1
        if n_loaded == 0:
            print(f"[auto-load-traj] no <mv>_trajectory.json files found under {traj_dir}.")
            self._clear_m1_start_conf_without_trajectory()
            self._print_movement_roster(tag='auto-load-traj')
            return

        # Pass 2: per-movement consistency. IN ORDER so each iteration's
        # check (2) can consult the predecessor's actually-kept trajectory,
        # and the post-check forward propagation can fill the next
        # movement's start_state.robot_configuration when it's None
        # (mirrors what _accept_trajectory does at plan time, lost across
        # BarAction reload since the BarAction file holds the authored
        # start_state, not the planned chain).
        n_dropped = 0
        for i, mv in enumerate(self._loaded_movements):
            traj = getattr(mv, 'trajectory', None)
            if traj is None:
                continue
            role = self._match_movement_role(mv)
            path12 = path_12_from_joint_trajectory(traj)
            if not path12:
                print(f"[auto-load-traj] {mv.movement_id!r}: trajectory has no waypoints; dropping.")
                mv.trajectory = None
                n_dropped += 1
                continue

            if role == 'M2':
                prev = self._loaded_movements[i - 1] if i > 0 else None
                if prev is None or self._match_movement_role(prev) != 'M1' or prev.trajectory is None:
                    msg = (f"[auto-load-traj] {mv.movement_id!r}: missing kept M1 trajectory; "
                           "dropping M2/M3 saved trajectories.")
                    print(msg)
                    n_dropped += self._drop_m2_m3_after_m1_chain_break(msg)
                    continue
            elif role == 'M3':
                prev = self._loaded_movements[i - 1] if i > 0 else None
                if prev is None or self._match_movement_role(prev) != 'M2' or prev.trajectory is None:
                    msg = (f"[auto-load-traj] {mv.movement_id!r}: missing kept M2 trajectory; "
                           "dropping loaded trajectory.")
                    print(msg)
                    self._drop_movement_trajectory(mv, msg)
                    n_dropped += 1
                    continue

            sc = mv.start_state.robot_configuration if mv.start_state is not None else None
            sc_vec = vec12_from_conf(sc) if sc is not None else None
            traj_first = np.asarray(path12[0], dtype=float)
            traj_last = np.asarray(path12[-1], dtype=float)

            dropped = False

            # (1) start_state.robot_configuration vs traj[0] — role-based:
            #   M1 owns its generated start_conf, so traj[0] is allowed to
            #     repopulate M1.start_state across reload.
            #   M2/M3 must obey the start_conf propagated from M1/M2. If
            #     their saved traj[0] does not match that hard start, the
            #     saved trajectory is stale and must be replanned.
            #   M0/M4 are NOT chain owners; their authored start_conf
            #     IS authoritative (M0.start_state is a live snapshot,
            #     M4 is the chain terminator). Run the strict compare
            #     and drop on mismatch.
            if role == 'M1' and mv.start_state is not None:
                if sc_vec is not None:
                    diff_owner = float(np.abs(traj_first - sc_vec).max())
                    if diff_owner > tol_rad:
                        print(
                            f"[auto-load-traj] {mv.movement_id!r}: "
                            f"overwriting start_state.robot_configuration with "
                            f"traj[0] (was authored, max-joint Δ "
                            f"{diff_owner:.4f} rad) per M1 generated-start rule."
                        )
                mv.start_state.robot_configuration = conf_from_12vec(traj_first)
                sc_vec = traj_first.copy()
            elif sc_vec is not None:
                diff = float(np.abs(traj_first - sc_vec).max())
                if diff > tol_rad:
                    msg = (f"[auto-load-traj] {mv.movement_id!r}: "
                           f"start_state.robot_configuration disagrees with "
                           f"traj[0] by max {diff:.4f} rad; dropping loaded trajectory.")
                    self.get_logger().warn(msg)
                    print(msg)
                    if role == 'M2':
                        # M2 start comes from M1.traj[-1]; if stale, M3's
                        # start inherited from old M2 is stale too.
                        n_dropped += self._drop_m2_m3_after_m1_chain_break(msg)
                    else:
                        self._drop_movement_trajectory(mv, msg, delete_file=(role in ('M2', 'M3')))
                        n_dropped += 1
                    dropped = True
            elif role in ('M2', 'M3'):
                msg = (f"[auto-load-traj] {mv.movement_id!r}: missing propagated "
                       "start_state.robot_configuration; dropping loaded trajectory.")
                self.get_logger().warn(msg)
                print(msg)
                if role == 'M2':
                    n_dropped += self._drop_m2_m3_after_m1_chain_break(msg)
                else:
                    self._drop_movement_trajectory(mv, msg)
                    n_dropped += 1
                dropped = True

            # (2) mv.start_state.robot_configuration vs prev movement's traj[-1]
            if not dropped and i > 0 and sc_vec is not None:
                prev = self._loaded_movements[i - 1]
                prev_traj = getattr(prev, 'trajectory', None)
                if prev_traj is not None:
                    prev_path = path_12_from_joint_trajectory(prev_traj)
                    if prev_path:
                        diff = float(np.abs(sc_vec - np.asarray(prev_path[-1])).max())
                        if diff > tol_rad:
                            msg = (f"[auto-load-traj] {mv.movement_id!r}: "
                                   f"start_state.robot_configuration disagrees "
                                   f"with prev {prev.movement_id!r}.traj[-1] by max "
                                   f"{diff:.4f} rad; dropping loaded trajectory.")
                            self.get_logger().warn(msg)
                            print(msg)
                            if role == 'M2' and self._match_movement_role(prev) == 'M1':
                                # M3's start depends on M2's end, so an M1->M2
                                # chain break invalidates both linear files.
                                n_dropped += self._drop_m2_m3_after_m1_chain_break(msg)
                            else:
                                self._drop_movement_trajectory(mv, msg, delete_file=(role in ('M2', 'M3')))
                                n_dropped += 1
                            dropped = True

            # (3) M0 only: live robot arm conf vs M0.start_state.robot_configuration
            if not dropped and role == 'M0':
                live12 = self._read_live_arm_conf_12()
                if live12 is None:
                    print(f"[auto-load-traj] M0 live-conf check skipped "
                          f"(no live robot interface available).")
                elif sc_vec is None:
                    print(f"[auto-load-traj] M0 live-conf check skipped "
                          f"(M0.start_state has no robot_configuration).")
                else:
                    diff = float(np.abs(live12 - sc_vec).max())
                    if diff > tol_rad:
                        msg = (f"[auto-load-traj] M0: live robot conf disagrees "
                               f"with M0.start_state.robot_configuration by max "
                               f"{diff:.4f} rad; dropping M0 trajectory (the "
                               f"live robot has moved since this M0 was planned).")
                        self.get_logger().warn(msg)
                        print(msg)
                        mv.trajectory = None
                        n_dropped += 1
                        dropped = True

            if dropped:
                continue

            # Forward-propagate path[-1] to next mv's start_state.robot_configuration.
            # Role-based, matching plan-time _accept_trajectory step 3:
            #   M1/M2/M3: chain owners — overwrite next.start unconditionally.
            #   M0/M4:    NOT chain owners — never write to next.start.
            if role not in ('M0', 'M4') and i + 1 < len(self._loaded_movements):
                next_mv = self._loaded_movements[i + 1]
                if next_mv.start_state is not None:
                    next_mv.start_state.robot_configuration = conf_from_12vec(traj_last)
                    print(f"[auto-load-traj] propagated {mv.movement_id!r}.traj[-1] "
                          f"-> {next_mv.movement_id!r}.start_state.robot_configuration "
                          f"(M1/M2/M3 chain rule).")

        print(f"[auto-load-traj] kept {n_loaded - n_dropped}/{n_loaded} loaded "
              f"trajectories after consistency checks "
              f"(dropped {n_dropped}).")
        self._clear_m1_start_conf_without_trajectory()
        self._print_movement_roster(tag='auto-load-traj')

    def _read_live_arm_conf_12(self):
        """Return the live robot's 12-DOF arm conf as np.ndarray, or None
        if no husky interface is wired (headless without stub, etc.)."""
        if not self.huskies:
            return None
        try:
            hi = self.huskies[self.selected_robot_id].interface
            left = np.asarray(hi.arm_joint_pose[0], dtype=float)
            right = (np.asarray(hi.arm_joint_pose[1], dtype=float)
                     if len(hi.arm_joint_pose) > 1
                     else np.asarray(hi.arm_joint_pose[0], dtype=float))
            if left.shape != (6,) or right.shape != (6,):
                return None
            return np.concatenate([left, right])
        except (AttributeError, IndexError, TypeError):
            return None

    def _color_bool(self, value):
        """Return a terminal-colored bool string for planning status prints."""
        if bool(value):
            return "\033[32mTrue\033[0m"
        return "\033[31mFalse\033[0m"

    def _validate_cdfm_planned_path(self, mv, path12):
        """Run sparse path_validation checks for any planned CDFM path."""
        movement_id = getattr(mv, 'movement_id', '') or ''
        if 'CDFM' not in movement_id:
            return
        if not path12:
            self.get_logger().warn("[CDFM validation] skipped: empty planned path.")
            return

        ctx = getattr(self, "_bar_action_plan_ctx", None) or {}
        pose_path = ctx.get("path_poses")
        if pose_path is None or len(pose_path) != len(path12):
            pose_len = None if pose_path is None else len(pose_path)
            self.get_logger().warn(
                f"[CDFM validation] skipped for {movement_id!r}: pose path length {pose_len} "
                f"does not match joint path length {len(path12)}."
            )
            return

        husky = getattr(self, "_bar_action_husky", None)
        if self.cfab is None or husky is None:
            self.get_logger().warn(f"[CDFM validation] skipped for {movement_id!r}: cfab pp robot is unavailable.")
            return

        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import STAGE3_GRASP_MASK_LINKS
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.path_validation import validate_stage_trajectory
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run import (
            HUSKY_DUAL_SRDF_PATH,
            HUSKY_DUAL_URDF_PATH,
        )

        saved_client = pp.CLIENT
        pp.CLIENT = self.cfab.client.client_id
        pp.CLIENTS.setdefault(pp.CLIENT, True)
        try:
            robot = husky.object.robot
            joint_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
            arm_joints = pp.joints_from_names(robot, joint_names)
            scene = {
                "robot": robot,
                "arm_joints": arm_joints,
                "tool_link_left": pp.link_from_name(robot, "left_ur_arm_tool0"),
                "tool_link_right": pp.link_from_name(robot, "right_ur_arm_tool0"),
                # Keep the scene shaped like run.py even though sparse mode
                # only consumes robot/joints/tool links.
                "bar_body": self.active_bar_body,
                "grasp_bar_from_left": ctx.get("grasp_bar_from_left"),
                "collision_obstacles": list(ctx.get("obstacles_for_constrained") or []),
                "bar_label": self.active_bar_name,
            }
            validation = validate_stage_trajectory(
                stage=int(ctx.get("stage", self.constrained_planner_stage)),
                scene=scene,
                path=pose_path,
                joint_path=[np.asarray(q, dtype=float) for q in path12],
                original_joint_path=None,
                joint_path_source="monitor_planned_path",
                joint_path_reason=None,
                urdf_path=HUSKY_DUAL_URDF_PATH,
                srdf_path=HUSKY_DUAL_SRDF_PATH,
                grasp_mask_links=STAGE3_GRASP_MASK_LINKS,
                target_label=self.active_bar_name,
                position_res=ctx.get("position_res"),
                rotation_res=ctx.get("rotation_res"),
                dense_joint_validation_step_rad=0.0,
                skip_dense_collision_checks=True,
                # Monitor validation is visual-only: show the plot, do not
                # write a PNG report into the TAMP validation reports folder.
                save_plot=False,
                show_plot=True,
            )
        except Exception as exc:
            self.get_logger().warn(f"[CDFM validation] failed for {movement_id!r}: {exc}")
            return
        finally:
            pp.CLIENT = saved_client

        wrap_count = int(validation.get("raw_wrap_segment_count") or 0)
        rel_ok = validation.get("relative_transform_ok")
        joint_ok = validation.get("joint_continuity_ok")
        max_dq = validation.get("joint_continuity_max_delta_rad")
        max_trans = validation.get("relative_transform_max_translation_m")
        max_axis = validation.get("relative_transform_max_axis_angle_deg") or {}
        max_axis_deg = max((v for v in max_axis.values() if v is not None), default=None)
        max_dq_text = None if max_dq is None else f"{max_dq:.4f} rad"
        max_trans_text = None if max_trans is None else f"{max_trans * 1000.0:.3f} mm"
        max_axis_text = None if max_axis_deg is None else f"{max_axis_deg:.3f} deg"
        print(
            f"[CDFM validation] {movement_id!r} sparse checks: "
            f"joint_continuity={joint_ok}, raw_wraps={wrap_count}, "
            f"ee_constraint={rel_ok}, max_dq={max_dq_text}, "
            f"ee_trans={max_trans_text}, ee_rot_axis={max_axis_text}"
        )
        if wrap_count or joint_ok is False or rel_ok is False:
            self.get_logger().warn(f"[CDFM validation] sparse validation FAILED for {movement_id!r}.")

    def _plan_M0_dispatch(self, mv):
        """Free dual-arm from live conf -> M1.start conf."""
        from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm
        if len(self._loaded_movements) < 2:
            return None
        m1 = self._loaded_movements[1]
        if m1.start_state is None or m1.start_state.robot_configuration is None:
            self.get_logger().warn("M1.start_state has no robot_configuration; cannot plan M0.")
            return None
        goal_conf = vec12_from_conf(m1.start_state.robot_configuration)
        start_conf = vec12_from_conf(mv.start_state.robot_configuration)
        scene = self._build_pp_scene_for_free()
        if scene is None:
            return None
        cfab_cf = self._build_cfab_free_collision_fn(mv.start_state)
        path, info = plan_free_dual_arm(scene, start_conf, goal_conf, 
                                        max_time=120.0,
                                        max_iterations=50,
                                        cfab_collision_fn=cfab_cf)
        if path is None:
            print(f"[M0] plan_free_dual_arm failed: {info.get('failure_reason')}")
            return None
        return joint_trajectory_from_path(path)

    def _plan_M1_dispatch(self, mv):
        """Constrained dual-arm planning via plan_and_stage_constrained."""
        self.constrained_trajectory = [None, None]
        world.plan_and_stage_constrained(self, ignore_env_obstacles=False)
        traj = self.constrained_trajectory
        if not (traj and traj[0] is not None and traj[1] is not None):
            return None
        left_path = traj[0][0]
        right_path = traj[1][0]
        T = min(len(left_path), len(right_path))
        path12 = [np.concatenate([left_path[i], right_path[i]]) for i in range(T)]
        return joint_trajectory_from_path(path12)

    def _plan_M2_dispatch(self, mv):
        """Constrained linear (bar-held)."""
        from husky_assembly_tamp.motion_planner.api import (
            plan_constrained_dual_arm_linear, _fk_link_frame,
        )
        from compas.geometry import Transformation, Frame
        if mv.start_state.robot_configuration is None or not mv.target_ee_frames:
            self.get_logger().warn("M2: missing start conf or target_ee_frames.")
            return None
        start_conf = vec12_from_conf(mv.start_state.robot_configuration)

        bar_rb = mv.start_state.rigid_body_states.get(self.active_bar_name) if self.active_bar_name else None
        if bar_rb is None or bar_rb.attachment_frame is None:
            self.get_logger().warn("M2: bar not attached in start_state.")
            return None
        attached_to_link = bar_rb.attached_to_link
        attach_T = Transformation.from_frame(bar_rb.attachment_frame)

        planner = self.cfab.planner
        robot_cell = self.cfab.robot_cell
        start_state = mv.start_state.copy()
        from husky_assembly_teleop.husky_world import _augment_tool_touch_links_for_v3
        _augment_tool_touch_links_for_v3(start_state, self.huskies[self.selected_robot_id])
        start_left = _fk_link_frame(planner, start_state, "left_ur_arm_tool0")
        start_right = _fk_link_frame(planner, start_state, "right_ur_arm_tool0")

        if 'left' in attached_to_link:
            start_world_from_attached_link = start_left
        else:
            start_world_from_attached_link = start_right
        start_world_from_bar = Transformation.from_frame(start_world_from_attached_link) * attach_T

        start_world_from_left_T = Transformation.from_frame(start_left)
        start_world_from_right_T = Transformation.from_frame(start_right)
        bar_from_left_tool0 = start_world_from_bar.inverted() * start_world_from_left_T
        bar_from_right_tool0 = start_world_from_bar.inverted() * start_world_from_right_T

        side = 'left' if 'left' in attached_to_link else 'right'
        target_arm_frame = mv.target_ee_frames.get(side)
        if target_arm_frame is None:
            self.get_logger().warn(f"M2: target_ee_frames missing key {side!r}.")
            return None
        target_arm_T = Transformation.from_frame(target_arm_frame)
        bar_from_arm = bar_from_left_tool0 if side == 'left' else bar_from_right_tool0
        goal_world_from_bar_T = target_arm_T * bar_from_arm.inverted()
        goal_world_from_bar = Frame.from_transformation(goal_world_from_bar_T)

        jt = plan_constrained_dual_arm_linear(
            planner, robot_cell, start_state, start_conf,
            goal_world_from_bar, bar_from_left_tool0, bar_from_right_tool0,
            skip_env_collisions=False,
        )
        if jt is not None:
            self._check_inter_ee_invariance(jt, start_state)
        return jt

    def _check_inter_ee_invariance(self, jt, template_state):
        """For an M2 (bar-held) trajectory, verify the left_from_right
        relative pose is constant over the path. Logs max/mean translation
        + rotation drift relative to the first waypoint.
        """
        from husky_assembly_tamp.motion_planner.api import _fk_link_frame
        from compas.geometry import Frame, Transformation

        planner = self.cfab.planner
        path = path_12_from_joint_trajectory(jt)
        if len(path) < 2:
            return
        left_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        names_12 = left_names + right_names

        state = template_state.copy()
        relatives = []
        for q12 in path:
            for n, v in zip(names_12, q12):
                state.robot_configuration[n] = float(v)
            lf = _fk_link_frame(planner, state, "left_ur_arm_tool0")
            rf = _fk_link_frame(planner, state, "right_ur_arm_tool0")
            T_l = Transformation.from_frame(lf)
            T_r = Transformation.from_frame(rf)
            relatives.append(T_l.inverted() * T_r)

        ref = relatives[0]
        ref_inv = ref.inverted()
        pos_devs = []
        ang_devs = []
        for rel in relatives:
            delta = ref_inv * rel
            tv = list(Frame.from_transformation(delta).point)
            pos_devs.append(float(np.linalg.norm(tv)))
            qw = abs(float(Frame.from_transformation(delta).quaternion.w))
            qw = min(max(qw, 0.0), 1.0)
            ang_devs.append(2.0 * float(np.arccos(qw)))
        pos_max = max(pos_devs); pos_mean = float(np.mean(pos_devs))
        ang_max = max(ang_devs); ang_mean = float(np.mean(ang_devs))
        print(
            f"[M2 inter-EE invariance] over {len(path)} waypoints: "
            f"pos drift max={pos_max*1000:.2f} mm (mean={pos_mean*1000:.2f}); "
            f"rot drift max={np.degrees(ang_max):.3f} deg (mean={np.degrees(ang_mean):.3f})"
        )

    def _plan_M3_dispatch(self, mv):
        """Linear retreat with independent EE interpolation."""
        from husky_assembly_tamp.motion_planner.api import plan_dual_arm_linear_independent
        if mv.start_state.robot_configuration is None or not mv.target_ee_frames:
            self.get_logger().warn("M3: missing start conf or target_ee_frames.")
            return None
        start_conf = vec12_from_conf(mv.start_state.robot_configuration)
        left_frame = mv.target_ee_frames.get('left')
        right_frame = mv.target_ee_frames.get('right')
        if left_frame is None or right_frame is None:
            self.get_logger().warn("M3: target_ee_frames must have both 'left' and 'right'.")
            return None
        start_state = mv.start_state.copy()
        from husky_assembly_teleop.husky_world import _augment_tool_touch_links_for_v3
        _augment_tool_touch_links_for_v3(start_state, self.huskies[self.selected_robot_id])
        return plan_dual_arm_linear_independent(
            self.cfab.planner, self.cfab.robot_cell, start_state,
            start_conf, left_frame, right_frame,
            skip_env_collisions=False,
        )

    def _plan_M4_dispatch(self, mv):
        """Free dual-arm from M3 end -> fixed home conf."""
        from husky_assembly_tamp.motion_planner.api import plan_free_dual_arm
        if mv.start_state.robot_configuration is None:
            self.get_logger().warn("M4: missing start_state.robot_configuration.")
            return None
        start_conf = vec12_from_conf(mv.start_state.robot_configuration)
        goal_conf = HUSKY_DUAL_ARM_HOME_CONF_12.copy()
        scene = self._build_pp_scene_for_free()
        if scene is None:
            return None
        cfab_cf = self._build_cfab_free_collision_fn(mv.start_state)
        path, info = plan_free_dual_arm(scene, start_conf, goal_conf, max_time=30.0,
                                        cfab_collision_fn=cfab_cf)
        if path is None:
            print(f"[M4] plan_free_dual_arm failed: {info.get('failure_reason')}")
            return None
        return joint_trajectory_from_path(path)

    def _build_cfab_free_collision_fn(self, mv_start_state, *, force=False):
        """Return a cfab-backed (conf12) -> bool collision predicate for the
        free dual-arm planner, or None if cfab CC is disabled.

        Free planner currently runs with composite_obstacles=[] in some paths
        and with env obstacles in others. cfab CC honors the state's tools +
        rigid_body attachments + SRDF disables + per-state touch_links — more
        correct than pp's get_collision_fn which leaves tool bodies stationary.
        Default OFF (legacy pp behavior); toggle via
        monitor.use_cfab_collision_for_free = True. Pass force=True to bypass
        the toggle gate (cfab/planner availability is still required).
        """
        if not force and not getattr(self, "use_cfab_collision_for_free", False):
            return None
        if self.cfab is None or getattr(self.cfab, "planner", None) is None:
            return None
        if mv_start_state is None:
            return None
        from copy import deepcopy
        from husky_assembly_teleop.cfab_collision_adapter import make_cfab_collision_fn
        from husky_assembly_teleop.husky_world import (
            _augment_tool_touch_links_for_v3,
            _augment_assembly_arm_tool_body_touch_links,
        )
        template = deepcopy(mv_start_state)
        _augment_tool_touch_links_for_v3(template, self.huskies[self.selected_robot_id])
        _augment_assembly_arm_tool_body_touch_links(template)
        print("[cfab-cc] free planner: using cfab PyBulletCheckCollision")
        return make_cfab_collision_fn(self.cfab, template)

    def _build_pp_scene_for_free(self):
        """Build SceneContext dict for plan_free_dual_arm using cfab pp-side robot."""
        husky = getattr(self, '_bar_action_husky', None)
        if husky is None or self.cfab is None:
            husky = self.huskies[self.selected_robot_id] if self.huskies else None
        if husky is None:
            self.get_logger().warn("No husky available for free-plan scene.")
            return None
        robot = husky.object.robot
        left_joints = pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[0])
        right_joints = pp.joints_from_names(robot, HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        arm_joints_all = list(left_joints) + list(right_joints)
        tool_link_L = pp.link_from_name(robot, 'left_ur_arm_tool0')
        tool_link_R = pp.link_from_name(robot, 'right_ur_arm_tool0')
        ee_attachments = [ee[1] for ee in husky.object.ee_list][:2]
        if len(ee_attachments) != 2:
            ee_attachments = (ee_attachments * 2)[:2]
        # ACM for the free planner: drop every robot-mounted body (tools,
        # held bar, attached joint parts) from scene["obstacles"]. Removing
        # a body from `obstacles` skips:
        #   1. robot link <-> that mounted body  -- the real fix; wrist mesh
        #      and tool mesh overlap by 1-5 cm by design.
        #   2. EE-ghost <-> that mounted body    -- harmless; ghosts sit at
        #      z=-100, far from anything.
        #   3. (indirect) mounted body <-> mounted body  -- left/right tool
        #      collision is no longer checked; the wrist_L <-> wrist_R robot
        #      self-collision check is the proxy that catches tool clashes.
        # Robot self-collision and robot <-> non-mounted env bodies remain
        # fully checked. The constrained planner sidesteps this issue via
        # its expected-neighbor-contact probe (5 mm getClosestPoints(bar,
        # body) at goal) which absorbs the overlap; free planner has no
        # such probe.
        mounted_names: set[str] = set()
        ss = getattr(self, 'movement_start_state', None)
        if ss is not None:
            for name, rbs in (getattr(ss, 'rigid_body_states', None) or {}).items():
                if getattr(rbs, 'attached_to_link', None) is not None:
                    mounted_names.add(name)
        obstacles = [
            body for name, body in (getattr(self, 'static_obstacles', None) or {}).items()
            if name not in mounted_names
        ]
        scene = {
            "robot": robot,
            "arm_joints": arm_joints_all,
            "joint_names": list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1]),
            "tool_link_left": tool_link_L,
            "tool_link_right": tool_link_R,
            "obstacles": obstacles,
            "attachments": ee_attachments,
            "disabled_collisions": None,
            "ee_types": list(getattr(husky.object, "ee_types", []) or []),
        }
        return scene

    def ik_live_base_for_selected_movement(self):
        """Debug IK at LIVE base for the current movement's START EE frames.

        Start EE frames are derived (in order of preference):
          1. FK from mv.start_state.robot_configuration + robot_base_frame
             (stored base, NOT live).
          2. Previous movement's target_ee_frames.

        Does NOT write to mv.trajectory. After success, the user can click
        'Plan Both Arms to Goal (composite)' to drive the real robot.
        """
        if self.current_movement is None:
            self.get_logger().warn("Load a movement first.")
            return
        mv = self.current_movement

        # 1) Derive start EE frames.
        start_ee_frames = None
        if mv.start_state is not None and mv.start_state.robot_configuration is not None:
            try:
                from husky_assembly_tamp.motion_planner.api import _fk_link_frame
                self.cfab.planner.set_robot_cell_state(mv.start_state)
                left_frame = _fk_link_frame(self.cfab.planner, mv.start_state, "left_ur_arm_tool0")
                right_frame = _fk_link_frame(self.cfab.planner, mv.start_state, "right_ur_arm_tool0")
                start_ee_frames = {"left": left_frame, "right": right_frame}
                print("[IK Live Base] start EE frames from FK at start_state.")
            except Exception as e:
                self.get_logger().warn(f"FK from start_state failed: {e}")
        if start_ee_frames is None and self.current_movement_index > 0:
            prev = self._loaded_movements[self.current_movement_index - 1]
            if prev.target_ee_frames:
                start_ee_frames = prev.target_ee_frames
                print(f"[IK Live Base] start EE frames from prev mv {prev.movement_id!r} target_ee_frames.")
        if not start_ee_frames or 'left' not in start_ee_frames or 'right' not in start_ee_frames:
            self.get_logger().warn(
                "Cannot derive start EE frames (no FK seed in start_state, "
                "no prev-movement target_ee_frames)."
            )
            return

        # 2) IK at live base using the derived start EE frames.
        live_state = mv.start_state.copy()
        hi = self.huskies[self.selected_robot_id].interface
        live_state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
        self.cfab.planner.set_robot_cell_state(live_state)
        # Override target_ee_frames so _solve_bar_action_goal_ik uses the
        # start-state derived frames (it reads monitor.target_ee_frames).
        saved_targets = self.target_ee_frames
        self.target_ee_frames = start_ee_frames
        try:
            from husky_assembly_teleop.husky_world import _solve_bar_action_goal_ik
            conf12 = _solve_bar_action_goal_ik(
                self, live_state, skip_env_collisions=True, verbose=False,
            )
        finally:
            self.target_ee_frames = saved_targets

        if conf12 is None:
            self.get_logger().warn("IK at live base FAILED.")
            return
        self.goal_arm_pose[0] = np.asarray(conf12[:6])
        self.goal_arm_pose[1] = np.asarray(conf12[6:])
        # Ghost must render live_base + IK conf together; otherwise the
        # ghost's tool0 drifts (live_base != start_state base, so rendering
        # stored-base + IK-conf gives a different tool0).
        self.goal_base_pose = (hi.position, hi.rotation)

        # Self-test: FK at the GOAL state (live_base + IK_conf, set by
        # _solve_bar_action_goal_ik on monitor.movement_goal_state). Do NOT
        # use the local live_state here — _solve_bar_action_goal_ik writes
        # the new conf onto a copy, so live_state.robot_configuration is
        # still the OLD seed conf, which would FK to (live_base * FK(old))
        # — i.e. the target offset by exactly the base offset, masking a
        # successful IK as an apparent failure.
        gs = getattr(self, 'movement_goal_state', None)
        try:
            from husky_assembly_tamp.motion_planner.api import _fk_link_frame
            fk_left = _fk_link_frame(self.cfab.planner, gs, "left_ur_arm_tool0")
            fk_right = _fk_link_frame(self.cfab.planner, gs, "right_ur_arm_tool0")
            def _residual(fk_frame, tg_frame):
                d_pos = float(np.linalg.norm(
                    np.asarray(fk_frame.point) - np.asarray(tg_frame.point)
                ))
                q_fk = np.asarray(fk_frame.quaternion.xyzw, dtype=float)
                q_tg = np.asarray(tg_frame.quaternion.xyzw, dtype=float)
                d_ang = 2.0 * float(np.arccos(
                    np.clip(abs(float(np.dot(q_fk, q_tg))), 0.0, 1.0)
                ))
                return d_pos, d_ang
            d_pos_L, d_ang_L = _residual(fk_left, start_ee_frames['left'])
            d_pos_R, d_ang_R = _residual(fk_right, start_ee_frames['right'])
            print(
                f"[IK Live Base] FK self-test residual: "
                f"L pos={d_pos_L*1000:.2f} mm ang={np.degrees(d_ang_L):.3f} deg | "
                f"R pos={d_pos_R*1000:.2f} mm ang={np.degrees(d_ang_R):.3f} deg"
            )
        except Exception as e:
            self.get_logger().warn(f"FK self-test failed: {e}")

        self.reset_ui(self.goal_arm_pose)
        self.set_to_show_goal_state()
        print("[IK Live Base] OK - goal_arm_pose updated (start-EE targets); "
              "click composite plan to drive.")

    def record_bar_holding_marker_take(self):
        """Record one labeled-marker take + run inline fit + log deviation."""
        world.request_marketset_button(self, 'bar_rig')

    def save_bar_holding_marker_data(self):
        """Save accumulated marker takes to the gdrive experiment dir; clear viz."""
        world.save_markerset_data(self, use_experiment_dir=True)
        self.marker_set_data = []
        for uid in self._bar_holding_fit_line_uids:
            try:
                pp.remove_debug(uid)
            except Exception:
                pass
        self._bar_holding_fit_line_uids = []

    def _build_bar_action_scrub_sliders(self):
        """Build (on the cfab GUI window) up to two debug-parameter sliders to
        scrub the staging + constrained trajectories, and precompute the
        per-waypoint RobotCellStates. Stashes everything in
        self._bar_action_scrub (serviced each tick by update())."""
        import pybullet as _pb
        from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

        left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        client_id = self.cfab.client.client_id

        def _build_waypoint_states(traj):
            if traj is None or traj[0] is None or traj[1] is None:
                return []
            left_path = traj[0][0]
            right_path = traj[1][0]
            n = len(left_path)
            if n < 1 or n != len(right_path):
                return []
            states = []
            for i in range(n):
                wp = self.movement_start_state.copy()
                for j, name in enumerate(left_names):
                    wp.robot_configuration[name] = float(left_path[i][j])
                for j, name in enumerate(right_names):
                    wp.robot_configuration[name] = float(right_path[i][j])
                states.append(wp)
            return states

        staging_states = _build_waypoint_states(getattr(self, "staging_free_trajectory", None))
        constrained_states = _build_waypoint_states(getattr(self, "constrained_trajectory", None))

        ns = len(staging_states)
        nc = len(constrained_states)
        if ns == 0 and nc == 0:
            self._bar_action_scrub = None
            return

        staging_slider = None
        constrained_slider = None
        if ns > 0:
            staging_slider = _pb.addUserDebugParameter(
                f"Staging t (0..{ns-1})", 0.0, float(max(ns - 1, 0)), 0.0,
                physicsClientId=client_id,
            )
        if nc > 0:
            constrained_slider = _pb.addUserDebugParameter(
                f"Constrained t (0..{nc-1})", 0.0, float(max(nc - 1, 0)), 0.0,
                physicsClientId=client_id,
            )
        self._bar_action_scrub = {
            "client_id": client_id,
            "staging_slider": staging_slider,
            "constrained_slider": constrained_slider,
            "staging_states": staging_states,
            "constrained_states": constrained_states,
            "last_staging": -1,
            "last_constrained": -1,
        }
        print(f"[scrub] '{self.current_movement.movement_id}' plan loaded: "
              f"staging={ns} wp, constrained={nc} wp. Drag the sliders on the "
              f"cfab PyBullet panel to scrub.")

    def _service_bar_action_scrub_sliders(self):
        """Poll the BarAction scrub sliders (once per tick) and re-pose the
        cfab scene when an index changed. No-op when no scrub state."""
        s = self._bar_action_scrub
        if s is None or self.cfab is None:
            return
        if self.cfab.client.client_id != s["client_id"]:
            self._bar_action_scrub = None
            return
        import pybullet as _pb
        cid = s["client_id"]
        if s["staging_slider"] is not None:
            t = _pb.readUserDebugParameter(s["staging_slider"], physicsClientId=cid)
            n = len(s["staging_states"])
            idx = max(0, min(n - 1, int(round(t))))
            if idx != s["last_staging"]:
                self.cfab.planner.set_robot_cell_state(s["staging_states"][idx])
                s["last_staging"] = idx
        if s["constrained_slider"] is not None:
            t = _pb.readUserDebugParameter(s["constrained_slider"], physicsClientId=cid)
            n = len(s["constrained_states"])
            idx = max(0, min(n - 1, int(round(t))))
            if idx != s["last_constrained"]:
                self.cfab.planner.set_robot_cell_state(s["constrained_states"][idx])
                s["last_constrained"] = idx

    def get_active_bar_aabb_dims(self):
        """AABB extents (m) of the active bar mesh from the RobotCell model.

        Used by the constrained planner to seed RRT feature points. Cached
        on first call.
        """
        if self.active_bar_aabb_dims is not None:
            return self.active_bar_aabb_dims
        if self.cfab is None or self.active_bar_name is None:
            return None
        rb_model = self.cfab.robot_cell.rigid_body_models.get(self.active_bar_name)
        if rb_model is None:
            return None
        # Walk visual meshes (in meters) and compute the per-axis extents.
        try:
            meshes = getattr(rb_model, 'visual_meshes_in_meters', None) or []
            if not meshes:
                meshes = getattr(rb_model, 'collision_meshes_in_meters', None) or []
            if not meshes:
                return None
            xs, ys, zs = [], [], []
            for m in meshes:
                for v in m.vertices():
                    pt = m.vertex_coordinates(v)
                    xs.append(pt[0]); ys.append(pt[1]); zs.append(pt[2])
            if not xs:
                return None
            self.active_bar_aabb_dims = (
                max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs),
            )
            return self.active_bar_aabb_dims
        except Exception as e:
            print(f"WARN: failed to compute active bar AABB: {e}")
            return None


    def load_joint_trajectory(self):
        """
        Load a JointTrajectory file and convert it to planned_arm_trajectory format.
        """
        if not self.available_joint_trajectories:
            print("No joint trajectory files available!")
            return
            
        if self.selected_trajectory_index >= len(self.available_joint_trajectories):
            print(f"Invalid trajectory index: {self.selected_trajectory_index}")
            return
            
        selected_trajectory_file = self.available_joint_trajectories[self.selected_trajectory_index]
        # Cache for downstream logging / filenames (e.g., calibration record suffix)
        self.selected_trajectory_file = selected_trajectory_file
        trajectory_filepath = os.path.join(
            DESIGN_DATA_DIRECTORY,
            DESIGN_PROBLEM_NAME,
            'Trajectories',
            selected_trajectory_file
        )
        
        print(f"Loading joint trajectory: {selected_trajectory_file}")
        
        try:
            # Load the joint trajectory using standard json
            import json
            with open(trajectory_filepath, 'r') as f:
                joint_trajectory_data = json.load(f)
            
            # Extract trajectory data
            if 'data' in joint_trajectory_data and 'points' in joint_trajectory_data['data']:
                points = joint_trajectory_data['data']['points']
                
                # Get joint names from the trajectory
                if points and 'joint_names' in points[0]:
                    joint_names = points[0]['joint_names']
                    
                    # Find indices for left and right arm joints
                    from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES
                    left_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
                    right_arm_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
                    
                    # Find indices for each arm's joints
                    left_arm_indices = [joint_names.index(name) for name in left_arm_names if name in joint_names]
                    right_arm_indices = [joint_names.index(name) for name in right_arm_names if name in joint_names]
                    
                    if len(left_arm_indices) != 6 or len(right_arm_indices) != 6:
                        print(f"Warning: Expected 6 joints per arm, got {len(left_arm_indices)} left, {len(right_arm_indices)} right")
                    
                    # Extract joint values for each arm
                    left_arm_trajectory = []
                    right_arm_trajectory = []
                    
                    for point in points:
                        if 'joint_values' in point:
                            left_joint_values = [point['joint_values'][i] for i in left_arm_indices]
                            right_joint_values = [point['joint_values'][i] for i in right_arm_indices]
                            left_arm_trajectory.append(np.array(left_joint_values))
                            right_arm_trajectory.append(np.array(right_joint_values))
                    
                    # Convert to planned_arm_trajectory format: (configurations, velocities, time, grasped_element)
                    # For now, we assume no grasped element (None) and no velocity information
                    left_trajectory_tuple = (left_arm_trajectory, None, self.trajectory_time, None)
                    right_trajectory_tuple = (right_arm_trajectory, None, self.trajectory_time, None)
                    
                    # Set the trajectories
                    self.set_arm_trajectory(left_trajectory_tuple, index=0)
                    self.set_arm_trajectory(right_trajectory_tuple, index=1)
                    
                    # Show trajectory state
                    self.set_to_show_traj_state()
                    
                    print(f"[Load Joint Traj] dual-arm trajectory: "
                          f"{len(left_arm_trajectory)} waypoints "
                          f"(left={len(left_arm_trajectory)}, "
                          f"right={len(right_arm_trajectory)}) "
                          f"from {selected_trajectory_file}")
                else:
                    print("Joint trajectory does not have expected joint_names structure")
            else:
                print("Joint trajectory does not have expected data structure")
                
        except Exception as e:
            print(f"Error loading joint trajectory: {e}")

    def update_board_validation_state_index(self, state_index):
        """
        Update the selected robot cell state index.
        """
        new_index = int(state_index)
        if 0 <= new_index < len(self.available_bar_actions):
            self.selected_state_index = new_index
            print(f"Selected state: {self.available_bar_actions[self.selected_state_index]}")

    def update_trajectory_index(self, trajectory_index):
        """
        Update the selected joint trajectory index.
        """
        new_index = int(trajectory_index)
        if 0 <= new_index < len(self.available_joint_trajectories):
            self.selected_trajectory_index = new_index
            self.selected_trajectory_file = self.available_joint_trajectories[self.selected_trajectory_index]
            print(f"Selected trajectory: {self.available_joint_trajectories[self.selected_trajectory_index]}")

    def _load_available_bar_actions(self):
        """Return sorted *.json BarAction filenames under <problem>/BarActions/.

        Attribute is kept under the legacy name for back-compat with
        UI/widgets and existing callers; contents are now BarAction files.
        """
        action_dir = os.path.join(
            DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'BarActions',
        )
        files = list_bar_actions(action_dir)
        if not files:
            print(f"No BarAction *.json files under: {action_dir}")
            return []
        print(f"Found {len(files)} BarAction files:")
        for i, fname in enumerate(files):
            print(f"  {i}: {fname}")
        return files

    def _load_available_joint_trajectories(self):
        """
        Load available JointTrajectory files from the hardcoded directory.
        """
        trajectory_dir = os.path.join(
            DESIGN_DATA_DIRECTORY,
            DESIGN_PROBLEM_NAME,
            'Trajectories'
        )

        if not os.path.exists(trajectory_dir):
            print(f"Trajectories directory does not exist: {trajectory_dir}")
            return []

        trajectory_files = [f for f in os.listdir(trajectory_dir) if f.endswith('.json')]
        trajectory_files.sort()

        print(f"Found {len(trajectory_files)} joint trajectory files:")
        for i, filename in enumerate(trajectory_files):
            print(f"  {i}: {filename}")

        return trajectory_files
    
    # --- --- --- --- --- SETUP PYBULLET --- --- --- --- ---
    def start_pybullet(self):
        # start pybullet simulator
        pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
        # turn on the GUI panels
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
        
        # draw world frame
        pp.draw_pose(pp.unit_pose(), 0.1)
        
    def load_goal_model(self):
        """
        Load goal robot model that mirrors the actual robot loaded in world.init.
        This ensures the goal model has the same configuration as the real robot.
        """
        # Get the first husky robot to determine the configuration
        if not self.huskies:
            self.get_logger().warn('No husky robots loaded yet. Cannot create goal model.')
            return
        
        # Get the configuration from the first robot
        first_husky = self.huskies[0]
        dual_arm = first_husky.dual_arm
        calibration = self.CALIBRATION
        
        # Determine end effector types from the actual robot
        ee_types = first_husky.object.ee_types

        # Load only the goal model that matches the actual robot configuration
        with pp.LockRenderer():
            with pp.HideOutput():
                if dual_arm:
                    # Load dual arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=True, 
                        ee_types=ee_types,  # Use all types for dual arm
                        force_regenerate=False,
                        punch_tool_offset=[self.get_punch_tool_offset(0), self.get_punch_tool_offset(1)]
                    )
                    self.goal_model_single = None  # Not needed for dual arm
                    self.goal_model_dual = self.goal_model
                else:
                    # Load single arm goal model
                    self.goal_model = HuskyObject(
                        calibration=calibration, 
                        dual_arm=False, 
                        ee_types=ee_types[:1] if ee_types else None,  # Take first type for single arm
                        force_regenerate=False,
                        punch_tool_offset=self.get_punch_tool_offset(0)
                    )
                    self.goal_model_single = self.goal_model
                    self.goal_model_dual = None  # Not needed for single arm
                
                self.goal_model.set_color(TRANSPARENT)

                # Load goal gripper model
                self.goal_gripper_model = load_gripper(calibration)
                pp.set_color(self.goal_gripper_model, GOAL_BLUE)

    def update_goal_model_and_color(self):
        # Since we now load only the goal model that matches the actual robot,
        # we don't need to switch between single and dual arm models
        # Just update the color based on the current state
        self.goal_model.set_color(GOAL_BLUE if self.show_goal_state else TRAJECTORY_GREEN)

    # --- mocap base XYZ offset side-window (standalone DPG) ---
    def _init_mocap_offset_window(self):
        """Spawn standalone DPG window with x/y/z text inputs + Apply/Reset.
        Independent of _common._global_backend so PyBullet primary UI is unaffected.
        """
        from . import common as _common
        from .ui_backend import DearPyGuiBackend, bind_default_font
        self._offset_dpg = None
        self._mocap_offset_pending = [0.0, 0.0, 0.0]

        # Avoid a 2nd DPG create_context() when primary backend is already DPG.
        if isinstance(_common._global_backend, DearPyGuiBackend):
            print("[mocap offset] primary backend is DPG; skipping private offset window.")
            return
        try:
            import dearpygui.dearpygui as dpg
        except ImportError:
            print("[mocap offset] dearpygui not installed; offset textboxes disabled. "
                  "`pip install dearpygui` to enable.")
            return

        self._offset_dpg = dpg
        dpg.create_context()
        dpg.create_viewport(title="Husky Base Mocap Offset", width=340, height=220)
        bind_default_font(dpg, int(self.UI_FONT_SIZE))
        dpg.setup_dearpygui()
        with dpg.window(tag="offset_window", label="Base XYZ Offset (world, m)",
                        width=340, height=220, no_close=True):
            dpg.add_input_float(tag="offset_x", label="x [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(0, a))
            dpg.add_input_float(tag="offset_y", label="y [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(1, a))
            dpg.add_input_float(tag="offset_z", label="z [m]", default_value=0.0,
                                step=0.0, format="%.4f",
                                callback=lambda s, a, u: self._set_pending_offset(2, a))
            dpg.add_separator()
            dpg.add_button(label="Apply", callback=lambda *a: self._apply_base_offset())
            dpg.add_button(label="Reset to Zero", callback=lambda *a: self._reset_base_offset())
        dpg.set_primary_window("offset_window", True)
        dpg.show_viewport()

    def _set_pending_offset(self, i, v):
        try:
            self._mocap_offset_pending[i] = float(v)
        except (TypeError, ValueError):
            pass

    def _apply_base_offset(self):
        h = self.huskies[self.selected_robot_id]
        h.mocap_base_offset_xyz = np.array(self._mocap_offset_pending, dtype=float)
        print(f"[mocap offset] applied: {h.mocap_base_offset_xyz.tolist()}")

    def _reset_base_offset(self):
        h = self.huskies[self.selected_robot_id]
        h.mocap_base_offset_xyz = np.zeros(3)
        self._mocap_offset_pending = [0.0, 0.0, 0.0]
        if self._offset_dpg is not None:
            dpg = self._offset_dpg
            for tag in ("offset_x", "offset_y", "offset_z"):
                if dpg.does_item_exist(tag):
                    dpg.set_value(tag, 0.0)
        print("[mocap offset] reset to zero")

    def _pump_mocap_offset_window(self):
        dpg = getattr(self, '_offset_dpg', None)
        if dpg is None:
            return
        if dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def _shutdown_mocap_offset_window(self):
        dpg = getattr(self, '_offset_dpg', None)
        if dpg is not None:
            dpg.destroy_context()
            self._offset_dpg = None

    def build_ui(self, target_conf=None):
        arm_slider_label = "arm id (0 only)" if self.get_active_arm_count() == 1 else "arm id (0:L,1:R)"
        arm_slider_max = 1 if self.get_active_arm_count() == 1 else 2
        self.arm_slider = Slider(arm_slider_label, self.update_selected_arm_id, 0, arm_slider_max, self.selected_arm_index)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, self.trajectory_time_max, self.trajectory_time)

        self.time_slider = p.addUserDebugParameter("Traj viz time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
                      
        self.buttons.append(Button('Plan S.Arm to conf target', self.plan_single_arm_to_goal_action))
        self.buttons.append(Button('Exec S.Arm Traj', self.execute_arm_trajectory))

        # Add buttons for planning both arms to goal (sequential and composite)
        # self.buttons.append(Button('Plan Both Arms to Goal (sequential)', lambda: world.plan_both_arms_to_goal(self, use_composite=False)))
        self.buttons.append(Button('Plan Both Arms to Goal (composite)', self.plan_both_arms_to_goal_action))
        self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))

        # BarAction loading — flag-independent, available for both single-arm and dual-arm.
        # Reset on rebuild; slider is (re)created below only when >= 2 entries
        # (a 1-entry slider has rangeMin == rangeMax which segfaults pybullet's GUI thread).
        self.board_validation_state_slider = None
        if not self.available_bar_actions:
            self.available_bar_actions = self._load_available_bar_actions()
        n_actions = len(self.available_bar_actions)
        if n_actions == 0:
            print("No robot cell state files found")
        else:
            self.dump_sep_sliders.append(Slider("----------BarAction Loading", lambda : None))
            if n_actions > 1:
                self.board_validation_state_slider = Slider(
                    "Bar Action",
                    self.update_board_validation_state_index,
                    0, n_actions - 1, self.selected_state_index
                )
            else:
                self.selected_state_index = 0  # only one; nothing to pick
            self.buttons.append(Button('Load BarAction', self.load_bar_action))

        # # Constrained dual-arm planner controls — only when the active robot is dual-arm.
        # # Stored as named attributes so update() polls them — items
        # # appended to self.dump_sep_sliders are not polled.
        # if self.huskies[self.selected_robot_id].dual_arm:
        #     self.constrained_stage_slider = Slider(
        #         "Constrained Stage",
        #         self.update_constrained_planner_stage,
        #         1, 3, 3,
        #     )
        #     self.buttons.append(Button(
        #         'Plan & Stage Constrained',
        #         self.plan_and_stage_constrained_bar_action,
        #     ))
        #     self.buttons.append(Button(
        #         'Export Dual-Traj',
        #         self.export_constrained_dual_arm_trajectory,
        #     ))
        #     self.buttons.append(Button(
        #         'Load Dual-Traj',
        #         self.parse_constrained_dual_arm_trajectory,
        #     ))
        #     self.constrained_display_slider = Slider(
        #         "Display Traj (0=Free,1=Constrained)",
        #         self.update_constrained_display_mode,
        #         0, 1, 0,
        #     )
        # else:
        #     # Clear stale handles from a prior dual-arm build (reset_ui removes
        #     # the underlying pybullet params but leaves Python attrs behind).
        #     for _attr in ('constrained_stage_slider', 'constrained_display_slider'):
        #         if hasattr(self, _attr):
        #             delattr(self, _attr)

        # # Button to export planned trajectory to JSON
        # self.buttons.append(Button(
        #     'Export Trajectory (JSON)',
        #     lambda: self.export_planned_trajectory_to_json()
        # ))

        # Gripper controls — only when the active robot connected its gripper.
        self.gripper_slider = None
        if self.huskies[self.selected_robot_id].connect_gripper:
            self.dump_sep_sliders.append(Slider("----------Gripper", lambda: None))
            self.gripper_slider = Slider(
                "gripper pos (0=open, 0.85=closed)",
                lambda v: setattr(self, 'goal_gripper', float(v)),
                0.0, 0.85, self.goal_gripper,
            )
            self.buttons.append(Button('Open Gripper Full', lambda: world.open_gripper_full(self)))
            self.buttons.append(Button('Close Gripper for Bar', lambda: world.close_gripper_for_bar(self)))
            self.buttons.append(Button('Set Gripper (slider)', lambda: world.set_gripper(self)))

        # Scaffolding V3 controls — only when active robot has assembly_tool_v3_*.
        active_husky = self.huskies[self.selected_robot_id]
        has_scaffold_left = any('assembly_tool_v3_left' in (t or '') for t in active_husky.ee_types)
        has_scaffold_right = any('assembly_tool_v3_right' in (t or '') for t in active_husky.ee_types)
        if has_scaffold_left or has_scaffold_right:
            self.dump_sep_sliders.append(Slider("----------Scaffolding V3", lambda: None))

            def send_scaffolding_cmd_both_motors(direction, arm_index):
                interface = self.huskies[self.selected_robot_id].interface
                interface.send_scaffolding_cmd(direction, 1, arm_index)
                interface.send_scaffolding_cmd(direction, 2, arm_index)

            def send_scaffolding_cmd_motor(direction, motor, arm_index):
                self.huskies[self.selected_robot_id].interface.send_scaffolding_cmd(direction, motor, arm_index)

            if has_scaffold_left:
                self.buttons.append(Button('- L Stop All', lambda: send_scaffolding_cmd_both_motors(0, 0)))
                self.buttons.append(Button('- L Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, 1, 0)))
                self.buttons.append(Button('- L Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, 1, 0)))
                self.buttons.append(Button('- L Tighten Joint', lambda: send_scaffolding_cmd_motor(1, 2, 0)))
                self.buttons.append(Button('- L Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, 2, 0)))

            if has_scaffold_right and active_husky.dual_arm:
                self.buttons.append(Button('- R Stop All', lambda: send_scaffolding_cmd_both_motors(0, 1)))
                self.buttons.append(Button('- R Tighten Gripper', lambda: send_scaffolding_cmd_motor(1, 1, 1)))
                self.buttons.append(Button('- R Loosen Gripper', lambda: send_scaffolding_cmd_motor(-1, 1, 1)))
                self.buttons.append(Button('- R Tighten Joint', lambda: send_scaffolding_cmd_motor(1, 2, 1)))
                self.buttons.append(Button('- R Loosen Joint', lambda: send_scaffolding_cmd_motor(-1, 2, 1)))

        if self.DUAL_ARM_KISSING:
            self.dump_sep_sliders.append(Slider("----------KISSING EXPERIMENT", lambda: None))
            self.buttons.append(Button('Conduct Kissing Experiment',
                lambda: self.tasks.append(world.kissing_experiment(self))))
            self.buttons.append(Button('Move Forward 1cm',
                lambda: world.move_left_linear_z(self, 0.01, 0.001)))
            self.buttons.append(Button('Move Back 1cm',
                lambda: world.move_left_linear_z(self, -0.01, 0.001)))

        if self.CONNECT_COMPLIANT_CONTROLLER:
            self.dump_sep_sliders.append(Slider("----------CONTROLLERS", lambda: None))
            def _switch_to_compliance_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.switch_controller(
                        'scaled_joint_trajectory_controller',
                        'cartesian_compliance_controller', i)
            def _switch_to_joint_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.switch_controller(
                        'cartesian_compliance_controller',
                        'scaled_joint_trajectory_controller', i)
            def _zero_force_sensor_both():
                h = self.huskies[self.selected_robot_id]
                for i in range(2 if h.dual_arm else 1):
                    h.interface.zero_ft_sensor(i)
            self.buttons.append(Button('Switch to Compliance (BOTH)', _switch_to_compliance_both))
            self.buttons.append(Button('Switch to Joint (BOTH)', _switch_to_joint_both))   # = "ensure joint controller"
            self.buttons.append(Button('Zero Force Sensor (BOTH)', _zero_force_sensor_both))
            self.buttons.append(Button('Draw TCP Pose', lambda: world.draw_tcp_pose(self)))

        if self.BOARD_VALIDATION:
            # Reset on rebuild; selection sliders are (re)created below only
            # when there are >= 2 entries. A 1-entry slider has
            # rangeMin == rangeMax which segfaults pybullet's GUI thread.
            self.trajectory_selection_slider = None

            # Joint trajectory: slider only when there's a choice; button
            # only when at least one file exists. Lazy-load once.
            if not self.available_joint_trajectories:
                self.available_joint_trajectories = self._load_available_joint_trajectories()
            n_traj = len(self.available_joint_trajectories)
            if n_traj > 0:
                self.dump_sep_sliders.append(Slider("----------Joint Trajectory Loading", lambda : None))
                if n_traj > 1:
                    self.trajectory_selection_slider = Slider(
                        "Joint Trajectory",
                        self.update_trajectory_index,
                        0, n_traj - 1, self.selected_trajectory_index
                    )
                else:
                    self.selected_trajectory_index = 0
                self.buttons.append(Button('Load Joint Trajectory', self.load_joint_trajectory))

        # if self.USE_MOCAP:
        #     self.dump_sep_sliders.append(Slider("----------MoCap Experiment", lambda : None))
        #     self.buttons.append(Button('Test Webcam Capture', self.test_webcam_capture))
        #     self.buttons.append(Button('Record Raw MoCap Take', self.record_raw_mocap_take))

        # if not self.CALIBRATION:
        #     # in calibration mode, we do not have task space targets so this is disabled
        #     pass
        #     # self.buttons.append(Button('Exec S.Arm Traj with servoing', self.execute_arm_trajectory_with_servoing))

        # if not self.CALIBRATION:
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))
        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        # Scaffolding tool control removed - outdated, will be remade later.

        if self.BAR_HOLDING_ACCURACY_TEST:
            self.dump_sep_sliders.append(Slider("----------Bar Holding Acc Test", lambda: None))
            if not self.available_bar_actions and hasattr(self, '_load_available_bar_actions'):
                self.available_bar_actions = self._load_available_bar_actions()
            n_files = len(self.available_bar_actions)
            if n_files >= 1:
                self.bar_action_file_slider = Slider(
                    "BarAction file (idx)",
                    lambda v: setattr(self, '_selected_action_file_idx', int(round(float(v)))),
                    0, max(0, n_files - 1),
                    int(self._selected_action_file_idx),
                    integer=True,
                )
            self.buttons.append(Button('Load BarAction', self.load_bar_action_file))
            n_movs = max(1, len(self._loaded_movements))
            self.bar_movement_slider = Slider(
                "Movement (idx; 0=M0_synth)",
                lambda v: setattr(self, '_selected_movement_idx', int(round(float(v)))),
                0, max(0, n_movs - 1),
                int(self._selected_movement_idx),
                integer=True,
            )
            self.buttons.append(Button('Load Movement', self.load_selected_movement))
            self.buttons.append(Button('Plan Movement', self.plan_selected_movement))
            # self.buttons.append(Button('Load Movement Trajectory', self.load_selected_movement_trajectory))
            self.buttons.append(Button('Delete All Saved Trajs', self.delete_saved_movement_trajectories_for_current_bar_action))
            self.buttons.append(Button('IK Live Base (debug)', self.ik_live_base_for_selected_movement))
            self.buttons.append(Button('Plan Free → Mv Start (cfab CC)', self.plan_free_to_movement_start_with_cfab_cc))
            self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))
            self.buttons.append(Button(
                'Exec Compliant (M2/M3 only)',
                lambda: self.tasks.append(world.execute_planned_trajectory_compliant(self))))

            self.buttons.append(Button(
                'Move Arms to Movement Start',
                lambda: world.move_arms_to_movement_start(self)))

            self.buttons.append(Button('Record markerset take', self.record_bar_holding_marker_take))
            self.buttons.append(Button('Save markerset data', self.save_bar_holding_marker_data))

        if self.DUAL_ARM_ACCURACY_TEST:
            self.dump_sep_sliders.append(Slider("----------Dual Arm Acc Test", lambda : None))
            self.buttons.append(Button('Compute Trajectory', lambda: world.next_dual_arm_bar_trajectory(self)))
            self.buttons.append(Button('Exec Arms', lambda: world.execute_arm_trajectory_both(self)))
            self.buttons.append(Button('Exec Arms and Record', lambda: self.tasks.append(world.execute_and_log_mocap(self))))
            self.buttons.append(Button('Record EE mocap pose', lambda: world.record_dual_arm_E_mocap(self)))
            self.buttons.append(Button('Save EE mocap data', lambda: world.save_dual_arm_E_mocap(self)))
            
        if self.CALIBRATION:
            self.dump_sep_sliders.append(Slider("----------Calibration", lambda : None))
            # self.calib_joint_range_slider = Slider("calib joint range", self.update_calib_joint_range, 0.0, np.pi*2, np.pi*2)
            # self.calib_target_axis_slider = Slider("calib target joint id", self.update_calib_target_axis, 0, 1, 0)
            # Mode slider: 0 = validation mode, 1 = data collection mode
            self.data_collection_mode_slider = Slider(
                "Mode (0:validation, 1:data_collection)",
                self.update_data_collection_mode,
                0.0, 1.0,
                1.0 if self.data_collection_mode else 0.0
            )
            self.calib_batch_slider = Slider(
                "Batch (0:j0,1:j1,2:valid,3:punch)",
                self.update_calib_batch_index,
                0, len(CALIBRATION_BATCHES) - 1,
                self.selected_calib_batch_index
            )
            # self.buttons.append(Button('Sample calib path', self.sample_calib_traj))
            # self.buttons.append(Button('Execute transit to calib traj', self.execute_free_trajectory))
            self.buttons.append(Button('Execute calib traj', self.execute_calib_traj))
            self.buttons.append(Button('Export calib data to json', self.record_calibration_data))

            # self.buttons.append(Button('Set joint 0 to zero', self.set_goal_joint_0_to_zero))
            # self.buttons.append(Button('Calib joint 1', lambda: world.calibrate_joint(self, 1, self.active_calib_tool_name)))

        if self.PUNCH_CALIB_VALIDATION:
            self.dump_sep_sliders.append(Slider("----------Punch Calib Validation", lambda : None))
            self.buttons.append(Button('Record Punch Take', self.record_punch_reference_pose))
            self.buttons.append(Button('Save Punch Validation Data', self.save_punch_validation_data))

        self.dump_sep_sliders.append(Slider("----------DEBUG utils", lambda : None))
        # self.buttons.append(Button('Record current calib conf', lambda: world.calibrate_button(self, self.active_calib_tool_name)))
        self.buttons.append(Button('Sample Random Goal Conf', self.sample_random_goal_conf))
        self.buttons.append(Button('Remove all drawing', lambda : pp.remove_all_debug()))
        # Button to load RobotCellState from file and update arm goal configuration
        # self.buttons.append(Button(
        #     'Load RobotCellState (robotx_box_A15-S13)',
        #     lambda: world.load_robotcellstate_and_update_goal(
        #         self,
        #         os.path.join(
        #             DATA_DIRECTORY,
        #             'robotx_box',
        #             'robotx_box_A15-S13_RobotCellState.json'
        #         )
        #     )
        # ))

        if self.USE_MOCAP:
            self._init_mocap_offset_window()

    # --- --- --- --- --- MOCAP --- --- --- --- ---
    _ANSI_GREEN = '\033[92m'
    _ANSI_RED = '\033[91m'
    _ANSI_RESET = '\033[0m'

    def start_mocap(self):
        self.get_logger().info('Starting mocap!')
        self.mocap_client = NatNetClient()
        self.mocap_client.set_client_address(CLIENT_IP)
        self.mocap_client.set_server_address(MOCAP_IP)
        self.mocap_client.set_use_multicast(False)
        self.mocap_client.print_level = 1

        self.mocap_client.rigid_body_listener = self.receive_rigid_body_frame
        self.mocap_client.new_frame_listener = self.receive_mocap_frame
        if self.BAR_HOLDING_ACCURACY_TEST:
            self.mocap_client.labeled_marker_listener = self.receive_labeled_marker

        if self.mocap_client.run():
            start_connect = time.time()
            while not self.mocap_client.connected():
                time.sleep(0.25)
                if time.time() - start_connect > 5:
                    break
            connected = self.mocap_client.connected()
            color = self._ANSI_GREEN if connected else self._ANSI_RED
            self.get_logger().info(f"{color}mocap client connected: {connected}{self._ANSI_RESET}")
            if connected:
                self.mocap_client.request_model_definitions()
        else:
            self.get_logger().info(f"{self._ANSI_RED}Failed to run mocap client!{self._ANSI_RESET}")

    def get_mocap_camera_inventory(self, refresh=False, timeout_sec=0.5):
        if not hasattr(self, 'mocap_client') or not self.mocap_client.connected():
            return None

        if refresh:
            self.mocap_client.request_model_definitions()

        deadline = time.time() + timeout_sec
        data_descs = self.mocap_client.get_latest_data_descriptions()
        while data_descs is None and time.time() < deadline:
            time.sleep(0.05)
            data_descs = self.mocap_client.get_latest_data_descriptions()

        if data_descs is None:
            return None

        camera_list = []
        for camera in getattr(data_descs, 'camera_list', []):
            camera_list.append(
                {
                    'name': camera.name.decode('utf-8') if isinstance(camera.name, bytes) else str(camera.name),
                    'position': [float(value) for value in camera.position],
                    'orientation': [float(value) for value in camera.orientation],
                }
            )

        return {
            'snapshot_time': time.time(),
            'camera_count': len(camera_list),
            'cameras': camera_list,
        }

    def send_request_to_mocap(self):
        # self.mocap_client.send_request(self.mocap_client.command_socket, self.mocap_client.NAT_REQUEST_MODELDEF,    "",  (self.mocap_client.server_ip_address, self.mocap_client.command_port) )
        # time.sleep(1)
        world.request_marketset_button(self, 'bar_rig')
    
    # mocap updates are happening in a separate thread
    def receive_rigid_body_frame(self, id, pos, rot):
        pos = np.array(mocap_pos_y_up_to_z_up(pos, self.MOCAP_AXIS_CONVENTION))
        rot = np.array(mocap_quat_y_up_to_z_up(rot, self.MOCAP_AXIS_CONVENTION))

        name = self.name_from_mocap_id.get(id, f'rigid_body_{id}')
        with self._mocap_cache_lock:
            self._mocap_rigidbody_cache[name] = (pos, rot)
            self._mocap_rigidbody_id_from_name[name] = int(id)
    
    def receive_mocap_frame(self, data):
        ts = data['timestamp']
        with self._mocap_cache_lock:
            raw_snapshot = {
                name: (np.array(pose[0], dtype=float), np.array(pose[1], dtype=float))
                for name, pose in self._mocap_rigidbody_cache.items()
            }
            rigid_body_ids = dict(self._mocap_rigidbody_id_from_name)

        if self.mocap_experiment_recording is not None:
            self._record_raw_mocap_snapshot(ts, raw_snapshot, rigid_body_ids)

        for h in self.huskies:
            if h.name not in raw_snapshot:
                continue
            world_from_mocap = raw_snapshot[h.name]
            # apply calibrated base transformation here
            # we keep the raw mocap data in _mocap_rigidbody_cache
            calibrated_pose = pp.multiply(world_from_mocap, h.base_mocap_from_base_footprint)
            # World-frame XYZ offset; rebind from UI thread is atomic in CPython.
            pos_with_offset = np.array(calibrated_pose[0]) + h.mocap_base_offset_xyz
            h.interface.mocap_callback(pos_with_offset, np.array(calibrated_pose[1]), ts)

        for o in self.tracked_objects:
            if o.name not in raw_snapshot:
                continue
            (pos, rot) = raw_snapshot[o.name]
            o.mocap_callback(pos, rot, ts)
        # self._mocap_rigidbody_cache.clear()

    def receive_labeled_marker(self, labeled_marker_from_model_id):
        # print('Received labeled marker data:', labeled_marker_from_model_id)
        # name = self.name_from_mocap_id[id]
        # if name not in self._mocap_rigidbody_cache:
        #     self.get_logger().warn(f'Mocap {name} not found in rb cache!')
        #     return
        # rb_pose = self._mocap_rigidbody_cache[name]

        for model_id, marker_datas in labeled_marker_from_model_id.items():
            if model_id not in self.name_from_mocap_id:
                continue

            name = self.name_from_mocap_id[model_id]
            if name not in self._mocap_labeled_marker_cache:
                self._mocap_labeled_marker_cache[name] = {}

            for marker_id, marker_data in marker_datas.items():
                pos = mocap_pos_y_up_to_z_up(marker_data['pos'], self.MOCAP_AXIS_CONVENTION)
                self._mocap_labeled_marker_cache[name][marker_id] = {
                    'pos': pos,
                    'size': marker_data['size'],
                    'error': marker_data['error'],
                }
            # print(f'Received marker set data for {name}:', self._mocap_labeled_marker_cache[name])
     
    # --- --- --- --- --- UPDATE --- --- --- --- --- 
    def update(self):
        from . import common as _common
        if _common._global_backend is not None:
            if not _common._global_backend.step():
                # User closed the UI window - request a clean shutdown.
                rclpy.shutdown()
                return

        self._pump_mocap_offset_window()

        # Keyboard shortcuts removed - outdated, will be remade later.

        for b in self.buttons:
            b.update()

        # Scaffolding-tool live status overlay removed - outdated, will be remade later.

        # update tracked objects
        for i, o in enumerate(self.tracked_objects):
            o.set_pose((o.pos, o.rot))
        
        # update robot state
        for i, h in enumerate(self.huskies):
            hi = h.interface
            if self.USE_MOCAP and not self.USE_CELL_STATE_BASE_POSE:
                # mocap drives the husky base pose
                h.object.set_pose((hi.position, hi.rotation), hi.arm_joint_pose)
                # set the goal pose of base since we are teleoperating the base
                if not self.goal_base_pose_frozen:
                    self.goal_base_pose = (hi.position, hi.rotation)
            else:
                # base is whatever the cell state set (or sliders set);
                # mocap only drives EE tracking in this branch
                h.object.set_pose(self.goal_base_pose, hi.arm_joint_pose)

        # pp.draw_pose(self.goal_model.get_link_pose_from_name("ur_arm_base_link"))

        self.arm_slider.update()
        self.trajectory_time_slider.update()
        if self.gripper_slider is not None:
            self.gripper_slider.update()

        # BarAction trajectory scrub sliders on the cfab GUI window (no-op
        # until 'Plan & Stage Constrained' has run on a BarAction).
        self._service_bar_action_scrub_sliders()

        # if self.CALIBRATION:
        #     self.calib_joint_range_slider.update()
        #     self.calib_target_axis_slider.update()
        
        if self.CALIBRATION and self.data_collection_mode_slider:
            self.data_collection_mode_slider.update()
        if self.CALIBRATION and self.calib_batch_slider:
            self.calib_batch_slider.update()

        if self.board_validation_state_slider:
            self.board_validation_state_slider.update()

        if self.BOARD_VALIDATION and hasattr(self, 'trajectory_selection_slider') and self.trajectory_selection_slider:
            self.trajectory_selection_slider.update()

        if hasattr(self, 'constrained_stage_slider'):
            self.constrained_stage_slider.update()
        if hasattr(self, 'constrained_display_slider'):
            self.constrained_display_slider.update()

        if self.BAR_HOLDING_ACCURACY_TEST:
            if hasattr(self, 'bar_action_file_slider') and self.bar_action_file_slider:
                self.bar_action_file_slider.update()
            if hasattr(self, 'bar_movement_slider') and self.bar_movement_slider:
                self.bar_movement_slider.update()

        if not self.USE_MOCAP:
            pass
            # self.teleop_base_slider_group.update()
        
        # update goal robot base state
        # state_slider_values = [p.readUserDebugParameter(ps) for ps in self.state_sliders]
        # self.goal_pose = (
        #     np.array((state_slider_values[0], state_slider_values[1], 0)),
        #     R.from_euler("z", state_slider_values[2], degrees=False).as_quat()
        # )
        # if not self.FAKE_HARDWARE:
        #     self.goal_gripper = p.readUserDebugParameter(self.gripper_slider)

        # update assembly goal position
        # self.assembly_goal_position_slider_group.update()
            
        preview_time = p.readUserDebugParameter(self.time_slider)
        goal_base_pose = self.goal_base_pose
        # Preview must not mutate self.goal_arm_pose; planners consume that
        # field as the actual target configuration.
        goal_arm_pose = [
            np.array(self.goal_arm_pose[0], dtype=float).copy(),
            np.array(self.goal_arm_pose[1], dtype=float).copy(),
        ]
        if not self.show_goal_state:
            # Trajectory preview rides on the LIVE robot's base pose (not the
            # frozen goal_base_pose / cell-state base) so the planned arm
            # motion is shown as it would actually look at the real-robot
            # location. The arm conf below is read from the planned
            # trajectory; pairing it with the live base matches what gets
            # executed.
            if self.huskies:
                _hi = self.huskies[self.selected_robot_id].interface
                goal_base_pose = (_hi.position, _hi.rotation)
            # if self.planned_base_trajectory[0] is not None:
            #     N = len(self.planned_base_trajectory[0])
            #     print('N:', N)
            #     base_traj_idx = int(preview_time * (N - 1))
            #     # TODO sometime the trajectory preview gets cut off halfway
            #     goal_base_pose = self.planned_base_trajectory[0][base_traj_idx]

            for i in range(0,2):
                if self.planned_arm_trajectory[i][0] is not None:
                    N = len(self.planned_arm_trajectory[i][0])
                    arm_traj_idx_float = preview_time * (N - 1)
                    arm_traj_idx = int(arm_traj_idx_float)
                    
                    # jg: i reenabled interpolation to see the whole motion including on sparse trajectories
                    # jg: the prerecorded trajectory had weird joint values in the >pi ranges which would lead to double rotations and self intersections
                    
                    if arm_traj_idx < len(self.planned_arm_trajectory[i][0]) and len(self.planned_arm_trajectory[i][0]) > 0:
                        goal_arm_pose[i] = self.planned_arm_trajectory[i][0][arm_traj_idx]

                    # we don't do interpolation here bc I want to see the exact trajectory points
                    # dt = arm_traj_idx_float - arm_traj_idx
                    # arm_traj_idx_plus = min(int(preview_time * (N - 1) + 1), N-1)
                    # goal_arm_pose[i] = lerp(self.planned_arm_trajectory[i][0][arm_traj_idx], self.planned_arm_trajectory[i][0][arm_traj_idx_plus], dt)

                if self.planned_arm_trajectory[i][3] is not None:
                    # update attached object based on FK
                    obj = self.planned_arm_trajectory[i][3]
                    gripper_tcp_from_object = obj.grasp
                    world_from_tcp = self.goal_model.get_link_pose_from_name("ur_arm_tool0")
                    object_pose = pp.multiply(world_from_tcp, gripper_tcp_from_object)
                    obj.set_pose(object_pose)
 
        # always update goal robot based on current slider values
        # goal_arm_pose is always length 2 (per __init__); slice for single-arm goal_model.
        arm_pose = goal_arm_pose if self.goal_model.dual_arm else goal_arm_pose[:1]
        self.goal_model.set_pose(goal_base_pose, arm_pose)

        # Drag attached-body ghosts along with the goal_model: pose follows
        # the parent link's FK at the current goal_arm_pose / preview-time
        # interpolation, composed with the stored attachment_frame.
        for g in self._traj_ghost_bodies:
            try:
                world_from_link = self.goal_model.get_link_pose_from_name(g['link'])
                pp.set_pose(g['body'], pp.multiply(world_from_link, g['attach']))
            except Exception:
                pass
                        
        # run tasks
        for t in self.tasks:
            try:
               next(t)
            except StopIteration:
                self.tasks.remove(t)
                
        world.update(self)

    def _trajectories_dir(self):
        d = os.path.join(DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME, 'Trajectories')
        os.makedirs(d, exist_ok=True)
        return d

    def export_constrained_dual_arm_trajectory(self, filename=None):
        """Export self.constrained_trajectory (left+right) as a single 12-DOF
        compas_fab JointTrajectory JSON, written to <problem>/Trajectories/."""
        from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
        from compas_fab.robots.time_ import Duration
        from compas_robots import Configuration
        from compas_robots.model import Joint
        from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

        traj = self.constrained_trajectory
        if not (traj and traj[0] is not None and traj[1] is not None):
            print("No constrained dual-arm trajectory to export. Run 'Plan & Stage Constrained' first.")
            return None
        left_path, _, left_time, _ = traj[0]
        right_path, _, right_time, _ = traj[1]
        n = len(left_path)
        if n == 0 or n != len(right_path):
            print(f"Constrained trajectory length mismatch: left={n}, right={len(right_path)}.")
            return None

        joint_names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
        joint_types = [Joint.REVOLUTE] * len(joint_names)
        total_time = float(left_time if left_time is not None else (right_time or 0.0))
        points = []
        for i in range(n):
            joint_values = [float(v) for v in left_path[i]] + [float(v) for v in right_path[i]]
            t = (total_time * i / (n - 1)) if n > 1 else 0.0
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            points.append(JointTrajectoryPoint(
                joint_values=joint_values,
                joint_types=joint_types,
                joint_names=joint_names,
                time_from_start=Duration(secs, nsecs),
            ))
        start_configuration = Configuration(
            joint_values=list(points[0].joint_values),
            joint_types=joint_types,
            joint_names=joint_names,
        ) if points else None
        jt = JointTrajectory(
            trajectory_points=points,
            joint_names=joint_names,
            start_configuration=start_configuration,
            fraction=1.0,
        )

        if filename is None:
            mv = self.current_movement
            act = self.current_action
            if mv is not None and act is not None:
                stem = f"{act.action_id}_{mv.movement_id}_constrained_dual_arm_JointTrajectory"
            else:
                stem = f"constrained_dual_arm_JointTrajectory_{int(time.time())}"
            filename = stem + '.json'
        out_path = os.path.join(self._trajectories_dir(), filename)
        jt.to_json(out_path, pretty=True)
        print(f"Exported constrained dual-arm trajectory ({n} waypoints) to {out_path}")
        # Refresh available list so the parse-side slider can pick it up.
        self.available_joint_trajectories = self._load_available_joint_trajectories()
        return out_path

    def parse_constrained_dual_arm_trajectory(self, filename=None):
        """Load a 12-DOF compas_fab JointTrajectory JSON from <problem>/Trajectories/
        and populate self.constrained_trajectory + per-arm display trajectories."""
        from compas_fab.robots import JointTrajectory
        from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES

        if filename is None:
            if not self.available_joint_trajectories:
                self.available_joint_trajectories = self._load_available_joint_trajectories()
            if not self.available_joint_trajectories:
                print("No JointTrajectory files in Trajectories/ to parse.")
                return False
            idx = self.selected_trajectory_index
            if not (0 <= idx < len(self.available_joint_trajectories)):
                print(f"Invalid trajectory index: {idx}")
                return False
            filename = self.available_joint_trajectories[idx]
        path = filename if os.path.isabs(filename) else os.path.join(self._trajectories_dir(), filename)
        if not os.path.isfile(path):
            print(f"Trajectory file not found: {path}")
            return False

        try:
            jt = JointTrajectory.from_json(path)
        except Exception as e:
            print(f"Failed to load JointTrajectory from {path}: {e}")
            return False

        left_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        right_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        # Resolve per-point joint name list (fall back to trajectory-level names).
        traj_names = list(jt.joint_names) if jt.joint_names else []
        try:
            left_idx = [traj_names.index(n) for n in left_names]
            right_idx = [traj_names.index(n) for n in right_names]
        except ValueError as e:
            print(f"Trajectory missing required dual-arm joints: {e}")
            return False

        left_path, right_path, times = [], [], []
        for pt in jt.points:
            names = pt.joint_names if pt.joint_names else traj_names
            if names == traj_names:
                li, ri = left_idx, right_idx
            else:
                try:
                    li = [list(names).index(n) for n in left_names]
                    ri = [list(names).index(n) for n in right_names]
                except ValueError as e:
                    print(f"Trajectory point missing required joints: {e}")
                    return False
            jv = pt.joint_values
            left_path.append(np.array([jv[i] for i in li], dtype=float))
            right_path.append(np.array([jv[i] for i in ri], dtype=float))
            times.append(pt.time_from_start.seconds)

        total_time = float(times[-1]) if times and times[-1] > 0 else float(self.trajectory_time)
        left_arr = np.array(left_path)
        right_arr = np.array(right_path)
        self.constrained_trajectory = [
            (left_arr, None, total_time, None),
            (right_arr, None, total_time, None),
        ]
        self.constrained_start_conf = np.concatenate([left_arr[0], right_arr[0]])
        self.constrained_goal_conf = np.concatenate([left_arr[-1], right_arr[-1]])
        self.set_arm_trajectory(self.constrained_trajectory[0], index=0)
        self.set_arm_trajectory(self.constrained_trajectory[1], index=1)
        self.constrained_display_mode = 1
        try:
            self._refresh_constrained_displayed_trajectory()
        except Exception:
            pass
        try:
            self.set_to_show_traj_state()
        except Exception:
            pass
        if self.cfab is not None and self.movement_start_state is not None:
            self._build_bar_action_scrub_sliders()
        print(f"[Parse Constrained Traj] dual-arm trajectory: "
              f"{len(left_path)} waypoints from {path}")
        return True

    def export_planned_trajectory_to_json(self, filename='planned_trajectory.json', arm_index=None):
        """
        Export the planned arm trajectory to a JSON file as a list of joint configurations.
        Save to the DATA_DIRECTORY/robotx_box subfolder.
        """
        import json
        if arm_index is None:
            arm_index = self.selected_arm_index
        traj = self.planned_arm_trajectory[arm_index][0]
        if traj is None or len(traj) == 0:
            print('No planned trajectory to export!')
            return
        # Convert numpy arrays to lists
        traj_list = [list(map(float, conf)) for conf in traj]
        # Save to DATA_DIRECTORY/robotx_box
        out_dir = os.path.join(DATA_DIRECTORY, 'robotx_box')
        os.makedirs(out_dir, exist_ok=True)
        # Add arm index to the filename before the extension
        base, ext = os.path.splitext(filename)
        filename_with_arm = f"{base}_arm{arm_index}{ext}"
        out_path = os.path.join(out_dir, filename_with_arm)
        with open(out_path, 'w') as f:
            json.dump(traj_list, f, indent=2)
        print(f'Trajectory exported to {out_path}')

    def destroy_node(self):
        from . import common as _common
        if _common._global_backend is not None:
            try:
                _common._global_backend.shutdown()
            except Exception as e:
                self.get_logger().warn(f"UI backend shutdown error: {e}")
            _common._global_backend = None
        try:
            self._shutdown_mocap_offset_window()
        except Exception as e:
            self.get_logger().warn(f"mocap offset window shutdown error: {e}")
        super().destroy_node()

# --- --- --- --- --- MAIN --- --- --- --- ---
def main(args=None):
    rclpy.init(args=args)

    husky_monitor = HuskyMonitor()

    rclpy.spin(husky_monitor)

    husky_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':     
    main()
