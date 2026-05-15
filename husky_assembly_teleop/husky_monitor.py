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
    UI_FONT_SIZE = 16  # DPG control-panel font size in px

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

        self.selected_robot_slider = None
        self.selected_robot_id = 0
        
        # Board validation mode variables
        self.board_validation_state_slider = None
        self.trajectory_selection_slider = None
        self.available_robot_cell_states = []
        self.selected_state_index = 0
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

        self.trajectory_time = 20 if self.CALIBRATION else 240

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
            self.available_robot_cell_states = self._load_available_bar_actions()
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

    def update_selected_robot_id(self, robot_id):
        new_id = np.clip(int(robot_id), 0, len(self.huskies)-1)
        if new_id != self.selected_robot_id:
            self.selected_robot_id = new_id
            self.selected_arm_index = min(self.selected_arm_index, self.get_active_arm_count() - 1)
            self._set_active_punch_tool_offset(self.selected_arm_index)
            if not self.USE_CELL_STATE_BASE_POSE:
                # update goal pose based on sensed base pose since we are teleoperating the base
                hi = self.huskies[self.selected_robot_id].interface
                self.goal_base_pose = (hi.position, hi.rotation)
            self.update_goal_model_and_color()
            self.reset_ui()
            
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
            the slider-selected entry of ``available_robot_cell_states``.
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
            if not self.available_robot_cell_states:
                print("No BarAction files available!")
                return False
            if self.selected_state_index >= len(self.available_robot_cell_states):
                print(f"Invalid BarAction index: {self.selected_state_index}")
                return False
            action_path = self.available_robot_cell_states[self.selected_state_index]
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
        """Hide the cfab-side robot URDF (and its tools) in the shared GUI.

        Set alpha=0 on every link so the duplicate husky/tool meshes loaded
        by ``planner.set_robot_cell`` stop overlapping the real robot. Pose
        and collision queries are unaffected — only visuals change.
        """
        if self.cfab is None or self.cfab.client is None:
            return
        client = self.cfab.client
        if client.robot_puid is not None:
            pp.set_color(client.robot_puid, TRANSPARENT)
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
        if 0 <= new_index < len(self.available_robot_cell_states):
            self.selected_state_index = new_index
            print(f"Selected state: {self.available_robot_cell_states[self.selected_state_index]}")

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
        
    def build_ui(self, target_conf=None):
        self.selected_robot_slider = Slider("robot id", self.update_selected_robot_id, 0, len(self.huskies)+1, self.selected_robot_id)
        arm_slider_label = "arm id (0 only)" if self.get_active_arm_count() == 1 else "arm id (0:L,1:R)"
        arm_slider_max = 1 if self.get_active_arm_count() == 1 else 2
        self.arm_slider = Slider(arm_slider_label, self.update_selected_arm_id, 0, arm_slider_max, self.selected_arm_index)

        self.trajectory_time_slider = Slider("traj time", self.update_trajectory_time, 1.0, self.trajectory_time, self.trajectory_time)

        self.time_slider = p.addUserDebugParameter("Traj viz time", 0.0, 1.0, 1.0)
        
        self.buttons.append(Button('Toggle Goal/Trajectory', self.toggle_show_goal_state))
        self.buttons.append(Button('Reset Goal State', self.reset_ui))
        
        if not self.USE_MOCAP:
            # teleop base when no mocap
            # self.dump_sep_sliders.append(Slider("----------Base Control", lambda : None))
            # pose2d = pp.pose2d_from_pose((self.huskies[self.selected_robot_id].interface.position, self.huskies[self.selected_robot_id].interface.rotation), tolerance=0.1)
            # self.teleop_base_slider_group = SliderGroup(["teleop base {}".format(t) for t in ["x","y","yaw"]], self.update_base_conf, [-5.0, -5.0, -np.pi], [5.0,5.0,np.pi], pose2d)
            # self.state_sliders.append(p.addUserDebugParameter("x", -5.0, 5.0, pose2d[0]))
            # self.state_sliders.append(p.addUserDebugParameter("y", -5.0, 5.0, pose2d[1]))
            # self.state_sliders.append(p.addUserDebugParameter("yaw", -np.pi, np.pi, pose2d[2]))
            pass
        else:
            pass
            # self.buttons.append(Button('Plan base', lambda: world.plan_to_goal(self)))
            # self.buttons.append(Button('Exec Base', lambda: world.move_to_goal(self)))
               
        self.buttons.append(Button('Plan S.Arm to conf target', self.plan_single_arm_to_goal_action))
        self.buttons.append(Button('Exec S.Arm Traj', self.execute_arm_trajectory))
        self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))

        # Add buttons for planning both arms to goal (sequential and composite)
        # self.buttons.append(Button('Plan Both Arms to Goal (sequential)', lambda: world.plan_both_arms_to_goal(self, use_composite=False)))
        self.buttons.append(Button('Plan Both Arms to Goal (composite)', self.plan_both_arms_to_goal_action))

        # Button to export planned trajectory to JSON
        self.buttons.append(Button(
            'Export Trajectory (JSON)',
            lambda: self.export_planned_trajectory_to_json()
        ))

        if self.DUAL_ARM_KISSING:
            self.dump_sep_sliders.append(Slider("----------KISSING EXPERIMENT", lambda: None))
            self.buttons.append(Button('Conduct Kissing Experiment',
                lambda: self.tasks.append(world.kissing_experiment(self))))
            self.buttons.append(Button('Move Forward 1cm',
                lambda: world.move_left_linear_z(self, 0.01, 0.001)))
            self.buttons.append(Button('Move Back 1cm',
                lambda: world.move_left_linear_z(self, -0.01, 0.001)))

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
            self.dump_sep_sliders.append(Slider("----------BarAction Loading", lambda : None))
            # Reset on rebuild; selection sliders are (re)created below only
            # when there are >= 2 entries. A 1-entry slider has
            # rangeMin == rangeMax which segfaults pybullet's GUI thread.
            self.board_validation_state_slider = None
            self.trajectory_selection_slider = None

            # Load available BarAction files if not already loaded
            if not self.available_robot_cell_states:
                self.available_robot_cell_states = self._load_available_bar_actions()

            n_actions = len(self.available_robot_cell_states)
            if n_actions == 0:
                print("No robot cell state files found for board validation")
            else:
                if n_actions > 1:
                    self.board_validation_state_slider = Slider(
                        "Bar Action",
                        self.update_board_validation_state_index,
                        0, n_actions - 1, self.selected_state_index
                    )
                else:
                    self.selected_state_index = 0  # only one; nothing to pick

                # Add button to load the selected BarAction (default movement = M1)
                self.buttons.append(Button('Load BarAction (M1)', self.load_bar_action))

                # Constrained dual-arm planner controls.
                # Stored as named attributes so update() polls them — items
                # appended to self.dump_sep_sliders are not polled.
                self.constrained_stage_slider = Slider(
                    "Constrained Stage",
                    self.update_constrained_planner_stage,
                    1, 3, 3,
                )
                self.buttons.append(Button(
                    'Plan & Stage Constrained',
                    self.plan_and_stage_constrained_bar_action,
                ))
                self.buttons.append(Button(
                    'Export Dual-Traj',
                    self.export_constrained_dual_arm_trajectory,
                ))
                self.buttons.append(Button(
                    'Load Dual-Traj',
                    self.parse_constrained_dual_arm_trajectory,
                ))
                self.constrained_display_slider = Slider(
                    "Display Traj (0=Free,1=Constrained)",
                    self.update_constrained_display_mode,
                    0, 1, 0,
                )

                # Joint trajectory: slider only when there's a choice; button
                # only when at least one file exists.
                n_traj = len(self.available_joint_trajectories)
                if n_traj > 1:
                    self.trajectory_selection_slider = Slider(
                        "Joint Trajectory",
                        self.update_trajectory_index,
                        0, n_traj - 1, self.selected_trajectory_index
                    )
                elif n_traj == 1:
                    self.selected_trajectory_index = 0
                if n_traj >= 1:
                    self.buttons.append(Button('Load Joint Trajectory', self.load_joint_trajectory))

        if self.USE_MOCAP:
            self.dump_sep_sliders.append(Slider("----------MoCap Experiment", lambda : None))
            self.buttons.append(Button('Test Webcam Capture', self.test_webcam_capture))
            self.buttons.append(Button('Record Raw MoCap Take', self.record_raw_mocap_take))

        if not self.CALIBRATION:
            # in calibration mode, we do not have task space targets so this is disabled
            pass
            # self.buttons.append(Button('Exec S.Arm Traj with servoing', self.execute_arm_trajectory_with_servoing))

        # if not self.CALIBRATION:
        #     self.buttons.append(Button('Exec Free Motion', self.execute_free_trajectory))
        #     self.buttons.append(Button('Exec Linear Motion', self.execute_linear_trajectory))
        # self.buttons.append(Button('Plan arm wave', lambda: world.plan_arm_wave(self)))

        # Scaffolding tool control removed - outdated, will be remade later.

        if self.BAR_HOLDING_ACCURACY_TEST:
            self.dump_sep_sliders.append(Slider("----------Bar Holding Acc Test", lambda: None))
            self.bar_holding_movement_slider = Slider(
                "BarAction Movement (M index)",
                lambda v: setattr(self, '_bar_holding_movement_idx', int(round(float(v)))),
                0, 5, 2,
            )
            self.buttons.append(Button('Load BarAction (selected M)',
                lambda: self.load_bar_action(movement=getattr(self, '_bar_holding_movement_idx', 2))))
            self.buttons.append(Button('Replan Free (live base)', self.replan_free_from_live_base))
            self.buttons.append(Button('Replan Constrained (live base)', self.replan_constrained_from_live_base))
            self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))
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
        self.buttons.append(Button('Record current calib conf', lambda: world.calibrate_button(self, self.active_calib_tool_name)))
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
        
        self.dump_sep_sliders.append(Slider("----------KISSING EXPERIMENT", lambda : None))
        self.buttons.append(Button('Conduct Kissing Experiment', lambda: self.tasks.append(world.kissing_experiment(self))))
        self.buttons.append(Button('Move Forward 1cm', lambda: world.move_left_linear_z(self, 0.01, 0.001)))
        self.buttons.append(Button('Move Back 1cm', lambda: world.move_left_linear_z(self, -0.01, 0.001)))
        
        self.dump_sep_sliders.append(Slider("----------CONTROLLERS", lambda : None))
        
        def switch_to_compliance_both():
            if self.huskies[self.selected_robot_id].dual_arm:
                self.huskies[self.selected_robot_id].interface.switch_controller('scaled_joint_trajectory_controller', 'cartesian_compliance_controller', 0)
                self.huskies[self.selected_robot_id].interface.switch_controller('scaled_joint_trajectory_controller', 'cartesian_compliance_controller', 1)
            else:
                self.huskies[self.selected_robot_id].interface.switch_controller('scaled_joint_trajectory_controller', 'cartesian_compliance_controller', 0)
        def switch_to_joint_both():
            if self.huskies[self.selected_robot_id].dual_arm:
                self.huskies[self.selected_robot_id].interface.switch_controller('cartesian_compliance_controller', 'scaled_joint_trajectory_controller', 0)
                self.huskies[self.selected_robot_id].interface.switch_controller('cartesian_compliance_controller', 'scaled_joint_trajectory_controller', 1)
            else:
                self.huskies[self.selected_robot_id].interface.switch_controller('cartesian_compliance_controller', 'scaled_joint_trajectory_controller', 0)
        def zero_force_sensor_both():
            if self.huskies[self.selected_robot_id].dual_arm:
                self.huskies[self.selected_robot_id].interface.zero_ft_sensor(0)
                self.huskies[self.selected_robot_id].interface.zero_ft_sensor(1)
            else:
                self.huskies[self.selected_robot_id].interface.zero_ft_sensor(0)
        self.buttons.append(Button('Switch to Compliance (BOTH)', switch_to_compliance_both))
        self.buttons.append(Button('Switch to Joint (BOTH)', switch_to_joint_both))
        self.buttons.append(Button('Zero Force Sensor (BOTH)', zero_force_sensor_both))
        self.buttons.append(Button('Draw TCP Pose', lambda: world.draw_tcp_pose(self)))
        
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
            h.interface.mocap_callback(np.array(calibrated_pose[0]), np.array(calibrated_pose[1]), ts)

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

        self.selected_robot_slider.update()
        self.arm_slider.update()
        self.trajectory_time_slider.update()

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

        if self.BOARD_VALIDATION and self.board_validation_state_slider:
            self.board_validation_state_slider.update()

        if self.BOARD_VALIDATION and hasattr(self, 'trajectory_selection_slider') and self.trajectory_selection_slider:
            self.trajectory_selection_slider.update()

        if self.BOARD_VALIDATION and hasattr(self, 'constrained_stage_slider'):
            self.constrained_stage_slider.update()
        if self.BOARD_VALIDATION and hasattr(self, 'constrained_display_slider'):
            self.constrained_display_slider.update()

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
