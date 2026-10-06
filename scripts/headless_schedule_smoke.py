"""Smoke test: the ActionSchedule operator flow through the real HuskyMonitor methods.

No ROS, no hardware, no window. The monitor is built without its ``__init__``
(same trick as ``headless_live_monitor_test.py``) and talks to a stub robot
interface that records every command and answers like ROS would, one monitor
tick later. The buttons are the monitor's own methods ('Load entry' =
``load_schedule_entry``, 'Mark entry done' = ``mark_entry_done``, ...).

! It ALWAYS runs on a scratch copy of the problem folder in a temp dir
! (ActionSchedule.json, WalkableGround.json and the clean BarActions/ are copied,
! the three RobotCell*.json are symlinked and only read). progress.json and the
! .live-solved.json sidecars are written there, never into the real design
! folder -- the last check confirms the real folder is untouched.

First, the dispatch check (its own Cindy monitor): for every movement of the
fixture's entry 0 (B1_J) and entry 1 (B1_R) it records which planner method
'Plan Movement' calls, which ``husky_world`` exec function 'Exec Selected Mv
Traj' calls, the scaffolding tool command a compliant exec sends (tighten /
gripper loosen; none for the insert of B1, a grounded bar), the traj time
default, whether the start is live and the preview type, and checks them
against ``DISPATCH_EXPECTED``. Planners and exec
functions are replaced by recorders (nothing is planned or moved); only the
compliant exec runs for real on the stub, up to its tool command. It does not
check planning outcomes.

Then the carry check (its own Cindy monitor, entry 0, fake configurations,
nothing planned): adopting a transfer start writes it into the steps where no
arm moves before the transfer (the manual mount, the tool grasp) and into the
travel to load's goal; accepting a transfer plan writes its last waypoint into
the insert's start and into the tighten step before it.

Then the transfer start check (its own Cindy monitor, entry 0, the real
manual-start IK, the travel to load's planner replaced by a recorder): 'Confirm
transfer start + plan travel to load' with the bar 3 m away plans nothing and
leaves the transfer loaded; with the default sliders right after 'Load entry'
it loads the travel to load and plans it, its goal being the adopted start.

Then the operator steps check (its own Cindy monitor, entries 0 and 1, the
compliant exec replaced by a recorder): entry 0 has 5 steps, entry 1 has 2; a
'Step' slider at 4 loads the insert and the '-> step' readout and 'Now:' line
count steps; without a slider widget an index still means a movement (the
tighten loads); exec of the insert names the tighten that runs with it, exec
of the retreat names the untighten and the ungrasp; entry 1 opens on its
retreat.

Then the transfer free move check (its own Cindy monitor, entry 0, the
workaround box ``TRANSFER_AS_FREE_MOVE`` ticked, the free planner replaced by a
recorder that returns [start, goal]): the goal puts both flanges on the
transfer's target frames; the planner gets a state with the bar and its joints
hidden and not attached; after acceptance the manual mount, the tool grasp,
the tighten and the insert start at the goal; unticked, the bar-held planner
is called as before. One real free plan is reported as INFO (not a check).

Then the grounded insert check (its own Cindy monitor, entries 0 and 2, a
straight joint path stamped for each insert, nothing planned): the REAL
compliant exec runs on the stub. B1's insert (a grounded bar: ground joints in
the tools) through 'Exec Selected Mv Traj' to its end: one joint trajectory for
both arms with the whole path, only the motor stops, no compliance controller,
no wait for the joint motors to stall, the 'grounded bar' lines, and its
tighten step logged as not sent (Rhino note D9). B3's insert (a normal bar) up
to its request for the compliance controller: TIGHTEN on both joint motors
first, as before.

Then the flow, on the fixture's first hold (Alice holds B3):

  Cindy's run (domain 86)
    - the built-structure switch first (``built_bars_check``), on entries 0 and 1:
      switch OFF (default) -> B1_J_M5_LM_insert's start and end states have no
      pair with the floor ``obstacle_ground`` in the full collision report (the
      ground joints may stand on it), B1_R never hides B1 or its joints; the panel
      toggle ON -> the entry reloads, the built joints are ignored and drawn
      faint where they stand, bodies not built yet stay blanked; OFF again ->
      the reload brings clean states back (also for a joint that rode in a
      held-bar preview in between)
    - entry 0 (B1_J): load; while its manual step waits for 'Confirm Exec',
      Mark entry done and Load entry are refused (one task at a time); Mark
      entry done -> progress.json, Cindy's belief 'live'
    - entry 3 (B3_H, Alice's): Load entry drops the previous planned path;
      display only, Plan / Exec step refused, Mark done waits at the Confirm
      Exec gate -> Alice's belief 'assumed' = B3__H's end; the drawn Alice moves
      to it, a husky mocap sees keeps its base
    - entry 4 (B3_R): ObstacleRobotAlice posed from that belief (not parked at
      (50, 50, 0)), equal to what B4__J exports; Reopen of this pending entry is
      refused; the retreat and the free move home plan with Alice
      posed from her belief; Load Movement of a step drops the planned path;
      Reset All reloads the entry from its clean export
    - entry 16 (B3_HR, Alice's): Mark done -> Alice released and drawn parked;
      Reopen (no index) reopens the LOADED entry, not the slider's
    - reopen entry 3 -> pending again, Alice's belief dropped
  Alice's run (domain 84)
    - entry 3 is hers: plan + exec H_M0 (free), gripper OPEN step, plan + exec
      H_M2 (linear), gripper CLOSE step with the compliant handoff (controllers
      joint -> compliance -> joint), Mark entry done -> belief 'live' at B3__H's
      end, trajectories saved to B3__H.live-solved.json
    - the live goal (``alice_live_goal_check``, F3): base not tracked -> the free
      approach keeps its exported goal; base tracked and moved 3 cm -> its goal
      is re-solved by IK at the live base (flange on the approach pose), the
      gripper step and the linear approach start from it, the linear approach
      plans; base 2 m away -> nothing planned, one 'not planning' warning
  Cindy's restart (domain 86, a new monitor object)
    - start-up: her simulated arms take her own progress.json belief (after
      entry 0), one log line
    - Load entry sets the traj time to the entry's first default (B1_R: its
      retreat's, as the release opens with tool steps); her next jointing
      entry's movement 0 starts from the believed configuration
    - 'Move Arms to Movement Start' with FAKE_HARDWARE moves the simulated arms
      to the start and sends no command
    - after Alice's hold is marked done, the header's second line reads
      'others: Alice <- assumed (entry 3) | Belle <- ...'

? Why do H_M0 / H_M2 plan here when ``smoke_single_arm_plan.py --collisions
? exported`` says the exported hold scene collides? Alice's run turns the
? monitor's built-bar switch ON (IGNORE_BUILT_ASSEMBLY_COLLISIONS, the panel's
? "Ignore built-bar collisions" toggle), which hides the built assembly from
? collision checks ("[built bars] collisions with N built bodies ignored (drawn
? faint)"), future bar B5 included. Everything else runs with the switch OFF,
? its default; Cindy's release plans do not need it.

Usage:

    cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
    export DESIGN_DATA_DIRECTORY="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study"
    export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
    python src/husky-assembly-teleop/scripts/headless_schedule_smoke.py [--keep]

    --keep leaves the temp dir (its path is printed) to inspect progress.json
    and the sidecar. (If an IK call complains that ``ssik`` is not installed,
    also ``export HUSKY_IK_BACKEND=gradient``.)

Exit code 0 when every check passes, 1 otherwise. Loads the ~340 MB
RobotCell files one at a time (Cindy's seven times, Alice's, then Cindy's again; ~1 GB RAM each);
takes under a minute (most of it Cindy's R_M3 free plan to home).
"""

import argparse
import hashlib
import io
import os
import shutil
import sys
import tempfile
import time
import traceback
from contextlib import redirect_stdout
from types import SimpleNamespace
from typing import Optional

import numpy as np
import pybullet as p
import pybullet_planning as pp

# The legacy harness and the single-arm smoke in this same folder (importable
# because Python puts the script's folder on sys.path).
from headless_live_monitor_test import StubLogger, _bypass_init_monitor
from smoke_single_arm_plan import collision_lines
from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop import cfab_session, husky_monitor, husky_world
from husky_assembly_teleop.bar_action_io import (
    STATIONARY_KINDS, MovementKind, OperatorStep, held_ground_joints, is_built_assembly_body,
    movement_kind, step_kind, tool_runs_with_next_motion,
)
from husky_assembly_teleop.cfab_session import CfabSession
from husky_assembly_teleop.husky_monitor import BUILT_IGNORED_RGBA
from husky_assembly_teleop.husky_robot import GRIPPER_MOTOR, JOINT_MOTOR, UR5e_HOME_STATE
from husky_assembly_teleop.husky_world import (
    GRIPPER_CLOSE_FOR_BAR_POS, GRIPPER_OPEN_POS, HOLD_FOR_STALL_TIMEOUT_S, _live_tool0_in_arm_base,
)
from husky_assembly_teleop.progress_io import (
    ARCHIVE_DIRNAME, PARKED_BASE_FRAME, STATUS_PENDING, belief_after, load_progress,
    obstacle_sources,
)
from husky_assembly_teleop.robot_registry import other_robots, robot_by_name
from husky_assembly_teleop.schedule_io import load_schedule, problem_root
from husky_assembly_teleop.utils import (
    joint_trajectory_from_path, path_from_joint_trajectory, pose_from_frame, vec12_from_conf,
)
from husky_assembly_tamp.motion_planner.api import _fk_link_frame
from rs_data_structure.bar_action import CONTROLLER_JOINT_TRACKING

# * The fixture's first hold: Alice holds B3 (entry B3_H); B3_R is Cindy's
# * release of B3 and B4_J the jointing whose export poses Alice correctly.
SUPPORT_ROBOT = 'Alice'
OTHER_SUPPORT_ROBOT = 'Belle'
ASSEMBLY_ROBOT = 'Cindy'
HELD_BAR = 'B3'
NEXT_BAR = 'B4'

JOINT_CTRL = 'scaled_joint_trajectory_controller'
COMPLIANCE_CTRL = 'cartesian_compliance_controller'
# Where the stub's fingers stop on the bar when closing (they stall there).
BAR_CONTACT_POS = 0.78
# Wall-clock time per simulated monitor tick; the handoff's waits are in seconds.
TICK_S = 0.005
SAME_TOL = 1e-6
# A copied configuration is equal to rounding (the restarted robot's arms from
# its belief; the carry rule's copies).
SEED_TOL = 1e-9
# The linear plan ends within ~1e-3 rad of the exported target configuration.
CONF_TOL = 2e-3
# * Alice's live goal check: how far her tracked base is moved from the authored
# * one, and how close the re-solved goal must put the flange on the approach pose
# * (the tolerances of the monitor's goal IK, husky_world.solve_goal_ik_generic).
LIVE_BASE_SHIFT_M = 0.03
OUT_OF_REACH_SHIFT_M = 2.0
FLANGE_POS_TOL = 1e-3
FLANGE_ANG_TOL = 0.01
# The re-solved goal at the shifted base must differ from the exported one by more than this.
GOAL_MOVED_TOL = 1e-3
# * Grounded insert check: the inserts run for real on the stub, along a straight
# * joint path of this many waypoints, with a short traj time (the rigid chunk
# * waits it out by the clock). A grounded insert must be over well before the
# * 30 s wait for the joint motors to stall.
INSERT_PATH_WAYPOINTS = 11
INSERT_TRAJ_TIME_S = 0.5
GROUNDED_INSERT_MAX_S = 5.0


# * ---------------------------------------------------------------------------
# * Stubs: logger and robot interface
# * ---------------------------------------------------------------------------

class RecordingLogger(StubLogger):
    """The harness's print logger that also keeps every message, to check refusals."""

    def __init__(self):
        self.msgs = []  # (level, message)

    def warn(self, msg: str) -> None:
        """Keep and print a warning."""
        self.msgs.append(('warn', msg))
        super().warn(msg)

    def info(self, msg: str) -> None:
        """Keep and print an info line."""
        self.msgs.append(('info', msg))
        super().info(msg)

    def error(self, msg: str) -> None:
        """Keep and print an error."""
        self.msgs.append(('error', msg))
        super().error(msg)

    def since(self, n: int, level: str, text: str) -> list:
        """Messages after the first ``n`` of one level that contain ``text``.

        Args:
            n (int): How many messages to skip (a ``len(msgs)`` taken earlier).
            level (str): ``'info'``, ``'warn'`` or ``'error'``.
            text (str): Substring to look for.

        Returns:
            list[str]: The matching messages.
        """
        return [m for lvl, m in self.msgs[n:] if lvl == level and text in m]


class StubInterface:
    """Stands in for ``HuskyRobotInterface``: records commands, answers on ``tick()``.

    Commands take effect the way ROS delivers them: a controller switch is
    acknowledged on the next tick, a gripper goal streams feedback positions
    tick by tick and then its result. An arm trajectory (one arm or both arms
    in one command) is "executed" at once: the arm stands at the last waypoint
    afterwards.
    """

    def __init__(self, spec, base_frame):
        """Build the stub for one robot, standing at one base frame with the arms at UR5e home.

        Args:
            spec (RobotSpec): The connected robot.
            base_frame (Frame): Where mocap "sees" the base.
        """
        n = spec.n_arms
        pos, rot = pose_from_frame(base_frame)
        self.n_arms = n
        self.position = np.asarray(pos, dtype=float)
        self.rotation = np.asarray(rot, dtype=float)
        self.arm_joint_pose = [np.asarray(UR5e_HOME_STATE, dtype=float).copy() for _ in range(n)]
        self.is_arm_executing = [False] * n
        self.active_controller = [JOINT_CTRL] * n
        self.controller_switch_error = [None] * n
        self.gripper_feedback = [None] * n
        self.gripper_result = [None] * n
        self.gripper_goal_handle = [None] * n
        # The gripper action server "answers" (see husky_world._gripper_server_ready).
        self.act_grippers = [SimpleNamespace(server_is_ready=lambda: True) for _ in range(n)]
        # What the compliant exec reads every tick: the force/torque sensors (no
        # force) and the scaffolding tools' status (no message yet, so no motor
        # ever reports STALLED).
        self.arm_ft_sensor = [[0.0] * 6 for _ in range(n)]
        self.scaffolding_status = [None] * n
        self.finger_pos = 0.0
        self.calls = []                          # every command, in order
        self.controller_history = [JOINT_CTRL]   # arm 0's active controller after each ack
        self.tcp_fn = None                       # arm-reported TCP; set by the Alice run
        self._pending = []                       # answers the next ticks deliver

    @property
    def arm_tcp_pose(self) -> list:
        """What the arm reports as its TCP in the arm base (a healthy arm: same as FK)."""
        return [self.tcp_fn(i) for i in range(self.n_arms)]

    # * --- commands (called by the monitor) ---
    def send_arm_cmd(self, path, vel=None, traj_time: float = 10.0, index: int = 0) -> None:
        """Joint trajectory: the arm ends up at the last waypoint.

        Args:
            path: Waypoints (one 6-vector each).
            vel: Unused (velocities).
            traj_time (float): Requested duration, recorded.
            index (int): Arm index.
        """
        self.calls.append(('arm', len(path), traj_time, index))
        self.arm_joint_pose[index] = np.asarray(path[-1], dtype=float)

    def send_dual_arm_cmd(self, multi_arm_trajectory) -> None:
        """One joint trajectory for both arms at once: each arm ends up at its last waypoint.

        Args:
            multi_arm_trajectory: One ``(path, vel, traj_time, attached)`` tuple
                per arm, the layout of the monitor's ``planned_arm_trajectory``.
        """
        self.calls.append(('dual_arm', tuple(len(traj[0]) for traj in multi_arm_trajectory),
                           multi_arm_trajectory[0][2]))
        for i, traj in enumerate(multi_arm_trajectory):
            self.arm_joint_pose[i] = np.asarray(traj[0][-1], dtype=float)

    def send_gripper_cmd(self, pos: float, effort: float, index: int = 0) -> bool:
        """Gripper goal: feedback positions over the next ticks, then the result.

        Args:
            pos (float): Target finger position.
            effort (float): Max effort, echoed in the result.
            index (int): Arm index.

        Returns:
            bool: Always True (the goal was "sent").
        """
        self.calls.append(('gripper', pos, index))
        self.gripper_feedback[index] = None
        self.gripper_result[index] = None
        # Closing stops (stalls) on the bar; opening reaches its goal.
        end = min(pos, BAR_CONTACT_POS)
        for p in np.linspace(self.finger_pos, end, 8)[1:]:
            self._pending.append(('feedback', index, float(p)))
        self._pending.append(('result', index, {
            'position': float(end), 'effort': effort, 'stalled': end < pos,
            'reached_goal': end >= pos, 'status': 4}))
        return True

    def switch_controller(self, from_ctrl: str, to_ctrl: str, arm_index: int = 0) -> bool:
        """Controller switch request; acknowledged on the next tick.

        Args:
            from_ctrl (str): Controller to stop.
            to_ctrl (str): Controller to start.
            arm_index (int): Arm index.

        Returns:
            bool: Always True (the request was "sent").
        """
        self.calls.append(('switch', from_ctrl, to_ctrl, arm_index))
        self.controller_switch_error[arm_index] = None
        self._pending.insert(0, ('controller', arm_index, to_ctrl))
        return True

    def send_arm_cmd_cartesian(self, pose, index: int = 0) -> None:
        """Compliance target (tool0 in the arm base): recorded with the active controller.

        Args:
            pose: pybullet (point, quat).
            index (int): Arm index.
        """
        self.calls.append(('cartesian', index, self.active_controller[index],
                           tuple(np.round(pose[0], 9))))

    def send_arm_cmd_cartesian_force(self, force, index: int = 0) -> None:
        """Compliance wrench target: recorded with the active controller.

        Args:
            force: 3-vector.
            index (int): Arm index.
        """
        self.calls.append(('force', tuple(force), index, self.active_controller[index]))

    def zero_ft_sensor(self, index: int = 0) -> bool:
        """F/T sensor zero request: recorded.

        Args:
            index (int): Arm index.

        Returns:
            bool: Always True (the request was "sent").
        """
        self.calls.append(('zero_ft', index))
        return True

    def send_scaffolding_cmd(self, direction: int, motor: int, index: int = 0) -> None:
        """Scaffolding tool motor command: recorded.

        Args:
            direction (int): -1 loosen, 0 stop, 1 tighten.
            motor (int): ``GRIPPER_MOTOR`` or ``JOINT_MOTOR``.
            index (int): Arm index.
        """
        self.calls.append(('scaffolding', direction, motor, index))

    # * --- what ROS delivers between two monitor ticks ---
    def tick(self) -> None:
        """Deliver the next pending answer (controller ack, gripper feedback or result)."""
        if not self._pending:
            return
        kind, i, value = self._pending.pop(0)
        if kind == 'controller':
            self.active_controller[i] = value
            if i == 0:
                self.controller_history.append(value)
        elif kind == 'feedback':
            self.finger_pos = value
            self.gripper_feedback[i] = {'position': value, 'effort': 0.0, 'stalled': False,
                                        'reached_goal': False, 't': time.time()}
        else:
            self.gripper_result[i] = dict(value, t=time.time())


# * ---------------------------------------------------------------------------
# * Harness helpers
# * ---------------------------------------------------------------------------

class Results:
    """Collects PASS / FAIL lines and prints the summary at the end."""

    def __init__(self):
        self.lines = []

    def check(self, name: str, ok: bool, detail: str = '') -> bool:
        """Record and print one check.

        Args:
            name (str): What was checked.
            ok (bool): Whether it held.
            detail (str): Numbers to print next to it.

        Returns:
            bool: ``ok``.
        """
        self.lines.append((bool(ok), name, detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ''))
        return bool(ok)

    def n_failed(self) -> int:
        """Number of failed checks."""
        return sum(not ok for ok, _, _ in self.lines)

    def print_summary(self) -> None:
        """Print every check again, grouped at the end of the log."""
        print("\n" + "=" * 78 + "\nSUMMARY\n" + "=" * 78)
        for ok, name, detail in self.lines:
            print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ''))
        n = len(self.lines)
        print(f"\n  {n - self.n_failed()}/{n} checks passed"
              + (" -> PASS" if self.n_failed() == 0 else " -> FAIL"))


def folder_snapshot(folder: str) -> dict:
    """Size and modification time of every file under a folder.

    Args:
        folder (str): The folder.

    Returns:
        dict: ``{relative path: (size, mtime_ns)}``.
    """
    out = {}
    for dirpath, _dirs, files in os.walk(folder):
        for name in files:
            path = os.path.join(dirpath, name)
            st = os.lstat(path)
            out[os.path.relpath(path, folder)] = (st.st_size, st.st_mtime_ns)
    return out


def sha1(path: str) -> str:
    """SHA-1 of a file's bytes (to prove a clean export was not rewritten)."""
    with open(path, 'rb') as f:
        return hashlib.sha1(f.read()).hexdigest()


def make_scratch_problem(real_root: str, scratch_design: str, problem: str) -> str:
    """Copy what the schedule flow writes next to into a scratch problem folder.

    Clean exports only (no ``.live-solved.json``, no ``progress.json``), so
    the smoke starts from the exported state whatever the real folder holds.
    The big RobotCell files are symlinked: they are only read.

    Args:
        real_root (str): The real problem folder.
        scratch_design (str): The scratch design data directory.
        problem (str): Problem folder name.

    Returns:
        str: The scratch problem folder.
    """
    root = os.path.join(scratch_design, problem)
    os.makedirs(root)
    for name in ('ActionSchedule.json', 'WalkableGround.json'):
        shutil.copy(os.path.join(real_root, name), os.path.join(root, name))
    shutil.copytree(os.path.join(real_root, 'BarActions'), os.path.join(root, 'BarActions'),
                    ignore=shutil.ignore_patterns('*.live-solved.json'))
    for name in ('RobotCell.json', 'RobotCell_Alice.json', 'RobotCell_Belle.json'):
        os.symlink(os.path.join(real_root, name), os.path.join(root, name))
    return root


def point_package_at(scratch_design: str) -> None:
    """Make the monitor and the cfab session read / write the scratch folder.

    Both modules copied ``DESIGN_DATA_DIRECTORY`` at import time, so the copies
    are replaced (the same way the legacy harness swaps ``DESIGN_PROBLEM_NAME``).

    Args:
        scratch_design (str): The scratch design data directory.
    """
    husky_monitor.DESIGN_DATA_DIRECTORY = scratch_design   # schedule, progress.json, sidecars
    cfab_session.DESIGN_DATA_DIRECTORY = scratch_design    # RobotCell (symlink), WalkableGround


def make_monitor(spec, base_frame, problem: str):
    """A headless HuskyMonitor connected to one robot, with a stub interface.

    Args:
        spec (RobotSpec): The connected robot.
        base_frame (Frame): Where the stub robot's base stands.
        problem (str): Problem folder name.

    Returns:
        tuple: ``(monitor, stub interface, logger)``.
    """
    monitor = _bypass_init_monitor()
    logger = RecordingLogger()
    monitor.get_logger = lambda: logger
    monitor.connected_robot = spec
    monitor.tasks = []
    monitor.cfab = CfabSession(problem, cell_filename=spec.cell_file, connection_type='direct')
    # pybullet_planning keeps its own "current client"; point it at the planner's world.
    pp.CLIENT = monitor.cfab.client.client_id
    pp.CLIENTS[monitor.cfab.client.client_id] = None
    iface = StubInterface(spec, base_frame)
    monitor.huskies = [SimpleNamespace(interface=iface, object=None,
                                       dual_arm=spec.dual_arm, name=spec.namespace)]
    monitor.selected_robot_id = 0
    return monitor, iface, logger


def add_viz_huskies(monitor, connected) -> None:
    """Give the monitor the other two huskies it draws (viz only: a pose, no robot).

    Like ``husky_world.create_registry_huskies``: ``monitor.husky_by_name`` maps
    every registry robot to its husky, the connected one being ``huskies[0]``.
    The others start at the world origin with their arms at UR5e home.

    Args:
        monitor (HuskyMonitor): The headless monitor.
        connected (RobotSpec): The connected robot.
    """
    monitor.husky_by_name = {connected.name: monitor.huskies[0]}
    for spec in other_robots(connected.name):
        monitor.husky_by_name[spec.name] = SimpleNamespace(
            name=spec.namespace,
            interface=SimpleNamespace(
                position=np.zeros(3), rotation=np.array([0.0, 0.0, 0.0, 1.0]),
                arm_joint_pose=[np.asarray(UR5e_HOME_STATE, dtype=float).copy()
                                for _ in range(spec.n_arms)]))


def drawn_base(monitor, name: str) -> np.ndarray:
    """Where a viz-only husky is drawn (its interface position)."""
    return np.asarray(monitor.husky_by_name[name].interface.position, dtype=float)


def run_tasks(monitor, iface: StubInterface, *, confirm: bool = True, max_ticks: int = 3000) -> int:
    """Pump ``monitor.tasks`` like the monitor's update() does, one tick at a time.

    Args:
        monitor (HuskyMonitor): The headless monitor.
        iface (StubInterface): Delivers ROS answers between ticks.
        confirm (bool): Click 'Confirm Exec' on every tick (answers every confirm gate).
        max_ticks (int): Give up (raise) after this many ticks.

    Returns:
        int: Ticks it took.
    """
    ticks = 0
    while monitor.tasks:
        if confirm:
            monitor._servo_exec_confirmed = True
        for task in list(monitor.tasks):
            # Same bookkeeping as the monitor's update(): the running task is not
            # "another" task for the one-task-at-a-time guard.
            monitor._running_task = task
            try:
                next(task)
            except StopIteration:
                monitor.tasks.remove(task)
            finally:
                monitor._running_task = None
        iface.tick()
        ticks += 1
        if ticks > max_ticks:
            raise RuntimeError(f"monitor tasks still running after {max_ticks} ticks")
        time.sleep(TICK_S)
    return ticks


def frame_dist(a, b) -> float:
    """Largest difference between two frames' point and axes."""
    return max(float(np.abs(np.subtract(list(getattr(a, k)), list(getattr(b, k)))).max())
               for k in ('point', 'xaxis', 'yaxis'))


def pt(frame) -> list:
    """A frame's point rounded to 4 decimals, for printing."""
    return np.round(list(frame.point), 4).tolist()


def frame_residual(a, b) -> tuple:
    """Distance (m) and rotation angle (rad) between two frames.

    Args:
        a (Frame): One frame.
        b (Frame): The other frame.

    Returns:
        tuple: ``(distance between the points, angle of the rotation from a to b)``.
    """
    d_pos = float(np.linalg.norm(np.subtract(list(a.point), list(b.point))))
    dot = abs(float(np.dot(a.quaternion.xyzw, b.quaternion.xyzw)))
    return d_pos, 2.0 * float(np.arccos(min(1.0, dot)))


def call_keeping_prints(fn, *args) -> tuple:
    """Call a function, keep what it prints (and still print it), for checks on print-only lines.

    Args:
        fn: The function.
        *args: Its arguments.

    Returns:
        tuple: ``(its return value, everything it printed)``.
    """
    buf = io.StringIO()
    with redirect_stdout(buf):
        out = fn(*args)
    sys.stdout.write(buf.getvalue())
    return out, buf.getvalue()


def select_movement(monitor, idx: int):
    """The 'Movement' slider + 'Load Movement' button.

    Args:
        monitor (HuskyMonitor): The headless monitor.
        idx (int): Movement index in the loaded action.

    Returns:
        Movement: The loaded movement.
    """
    monitor._selected_movement_idx = idx
    monitor.load_selected_movement()
    return monitor.current_movement


# * ---------------------------------------------------------------------------
# * Dispatch check: which planner / exec / tool command each movement reaches
# * ---------------------------------------------------------------------------

# * What today's monitor does with every movement of the fixture's entry 0 (J)
# * and entry 1 (R), keyed by (entry kind, movement index in the file). The
# * movement id is written without its bar prefix ('B1_'). '-' = not run for
# * this movement; traj_time None = the movement has no default traj time.
# ! Since stage 1 (F5d) the stationary steps whose start state has the bar
# ! attached (J 1, J 2, J 4, R 0) preview as 'bar_held': the bar is drawn in the tools.
# ! B1 (entry 0) is a GROUNDED bar (held on its ground joints at the insert): its
# ! insert runs rigid and never tightens, so J 5 sends no tool command; the
# ! exported tighten step (J 4) is an export defect (Rhino note D9).
# ! grounded_insert_check covers the grounded insert and a normal one (B3).
DISPATCH_FIELDS = ('movement', 'planner', 'exec', 'tool_cmd', 'traj_time', 'starts_live', 'preview')
DISPATCH_EXPECTED = {
    ('J', 0): ('J_M0_free_to_load', 'free_to_load', 'zero_ft', '-', 30, True, 'free'),
    ('J', 1): ('J_M1_manual_mount_bar', 'none', '-', '-', None, False, 'bar_held'),
    ('J', 2): ('J_M2_tool_grasp_bar', 'none', '-', '-', None, False, 'bar_held'),
    ('J', 3): ('J_M3_CDFM_transfer_to_approach', 'transfer', 'arm_both', '-', 10, False, 'bar_held'),
    ('J', 4): ('J_M4_tool_tighten_joint', 'none', '-', '-', None, False, 'bar_held'),
    ('J', 5): ('J_M5_LM_insert', 'insert', 'compliant', 'none sent', 5, False, 'bar_held'),
    ('R', 0): ('R_M0_tool_untighten_joint', 'none', '-', '-', None, False, 'bar_held'),
    ('R', 1): ('R_M1_tool_ungrasp_bar', 'none', '-', '-', None, False, 'free'),
    ('R', 2): ('R_M2_LM_retreat', 'retreat', 'compliant', 'loosen_gripper', 5, False, 'free'),
    ('R', 3): ('R_M3_free_home', 'free_home', 'arm_both', '-', 10, False, 'free'),
}


def tool_command_sent(monitor, iface: StubInterface, compliant_exec) -> str:
    """Run the real compliant exec on the stub until it sends its tool command, then stop it.

    The compliant exec sends a "stop" (direction 0) to every tool motor, then
    its own command, before it moves an arm. The stops are left out here. A
    grounded bar's insert sends no command of its own: the 50 steps then run
    inside its rigid joint motion, which closing the exec stops.

    Args:
        monitor (HuskyMonitor): The headless monitor, the movement loaded, a
            planned path stamped and ``ready_for_compliant_exec`` called.
        iface (StubInterface): Records the scaffolding commands.
        compliant_exec: The real ``husky_world.execute_planned_trajectory_compliant``.

    Returns:
        str: ``'tighten'`` (joint motor +1, both arms), ``'loosen_gripper'``
        (gripper motor -1, both arms), the raw ``(direction, motor, arm)``
        commands when they are neither, or ``'none sent'``.
    """
    names = {
        ((1, JOINT_MOTOR, 0), (1, JOINT_MOTOR, 1)): 'tighten',
        ((-1, GRIPPER_MOTOR, 0), (-1, GRIPPER_MOTOR, 1)): 'loosen_gripper',
    }
    n = len(iface.calls)

    def sent() -> tuple:
        return tuple(sorted(c[1:] for c in iface.calls[n:] if c[0] == 'scaffolding' and c[1] != 0))

    task = compliant_exec(monitor)
    try:
        for _ in range(50):
            next(task)
            iface.tick()
            if sent():
                break
    except StopIteration:
        pass  # the exec ended on its own (its warning says why)
    finally:
        # Stops the exec where it is; its own clean-up sends the motor stops.
        task.close()
    return names.get(sent(), repr(sent()) if sent() else 'none sent')


def ready_for_compliant_exec(monitor) -> None:
    """Give a headless Cindy monitor what the real compliant exec reads besides the stub interface.

    The exec reads tool0 by FK on the ghost robot; headless, the planner's robot
    body (Cindy's URDF) is the ghost (same as the Alice run). Its rigid chunk
    (the insert's approach, the whole insert of a grounded bar) reads the drawn
    husky's robot body only to hand it to the wrench recorder, which does not
    use it, so an empty one is enough.

    Args:
        monitor (HuskyMonitor): The headless monitor (``make_monitor``).
    """
    monitor.goal_model.robot = monitor.cfab.client.robot_puid
    monitor.huskies[0].object = SimpleNamespace(robot=None)


def dispatch_check(results: Results, problem: str, root: str, schedule) -> None:
    """Record which planner, exec function and tool command each Cindy movement reaches.

    For every movement of entry 0 (J) and entry 1 (R): 'Plan Movement' with the
    planner methods replaced by recorders (nothing is planned), then 'Exec
    Selected Mv Traj' with the ``husky_world`` exec functions replaced by
    recorders (nothing runs), and for a compliant exec the REAL compliant exec
    on the stub up to its tool command. Each movement is checked against
    ``DISPATCH_EXPECTED``.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    # * Planner method on the monitor -> label. Stage 4 renames the methods;
    # * only this dict changes then.
    planner_labels = {
        '_plan_M0_dispatch': 'free_to_load',
        '_plan_M1_dispatch': 'transfer',
        '_plan_M2_dispatch': 'insert',
        '_plan_M3_dispatch': 'retreat',
        '_plan_M4_dispatch': 'free_home',
    }
    # * husky_world function the exec button calls -> (label, whether the
    # * monitor queues what it returns as a task).
    exec_labels = {
        'execute_planned_trajectory_compliant': ('compliant', True),
        'execute_trajectory_and_zero_ft': ('zero_ft', True),
        'execute_arm_trajectory_both': ('arm_both', False),
        'execute_arm_trajectory_all': ('arm_all', False),
    }
    # Set before each Load Movement; no default can give it (defaults are at least 1 s).
    no_default = -1.0

    cindy = robot_by_name(ASSEMBLY_ROBOT)
    entries = [schedule.entry(0), schedule.entry(1)]
    assert [e.kind for e in entries] == ['J', 'R'], \
        f"the dispatch check needs a J then an R entry first, got {[e.action_id for e in entries]}"
    print(f"\n{'=' * 30} DISPATCH CHECK {'=' * 30}")
    base = schedule.load_action(entries[0], prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, _log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    ready_for_compliant_exec(monitor)

    called = []  # labels of the recorders the last button reached

    def recorder(label: str, queued: bool = False):
        def record(*_args, **_kwargs):
            called.append(label)
            return iter(()) if queued else None  # a queued task that ends at once
        return record

    real_compliant = husky_world.execute_planned_trajectory_compliant
    originals = {name: getattr(husky_world, name) for name in exec_labels}
    try:
        for name, (label, queued) in exec_labels.items():
            setattr(husky_world, name, recorder(label, queued))
        monitor._load_schedule_state()
        for entry in entries:
            # * --- planner sweep
            monitor.load_schedule_entry(entry.index)
            n_mv = len(monitor._loaded_movements)
            planners = []
            for name, label in planner_labels.items():
                setattr(monitor, name, recorder(label))
            for idx in range(n_mv):
                select_movement(monitor, idx)
                called.clear()
                monitor.plan_selected_movement()
                planners.append('+'.join(called) or 'none')
            for name in planner_labels:
                delattr(monitor, name)  # back to the monitor's own methods

            # * --- exec sweep, on a fresh load: a "failed" plan (the recorders
            # * --- return None) clears state, e.g. the transfer's start configuration
            monitor.load_schedule_entry(entry.index)
            for idx in range(n_mv):
                monitor.trajectory_time = no_default
                mv = select_movement(monitor, idx)
                traj_time = None if monitor.trajectory_time == no_default else monitor.trajectory_time
                exec_label = tool_cmd = '-'
                if step_kind(mv) == 'arm':
                    # A two-waypoint path where the stub arms stand: exec's "arms at
                    # the trajectory start" check passes, as after 'Move Arms to
                    # Movement Start'.
                    for i, q in enumerate(iface.arm_joint_pose):
                        monitor.set_arm_trajectory(
                            (np.asarray([q, q]), None, monitor.trajectory_time, None), i)
                    called.clear()
                    monitor.exec_selected_movement_traj()
                    run_tasks(monitor, iface)
                    exec_label = '+'.join(called) or 'none'
                    if exec_label == 'compliant':
                        tool_cmd = tool_command_sent(monitor, iface, real_compliant)
                recorded = (mv.movement_id.split('_', 1)[1], planners[idx], exec_label, tool_cmd,
                            traj_time, idx in monitor._live_start_indices,
                            monitor._authored_motion_type(mv))
                expected = DISPATCH_EXPECTED.get((entry.kind, idx))
                shown = ' '.join(f'{k}={v}' for k, v in zip(DISPATCH_FIELDS[1:], recorded[1:]))
                diff = ('no expected row' if expected is None else ', '.join(
                    f'{k} expected {e}' for k, r, e in zip(DISPATCH_FIELDS, recorded, expected)
                    if r != e))
                results.check(f'dispatch: {mv.movement_id} {shown}', recorded == expected, diff)
            n_rows = sum(kind == entry.kind for kind, _ in DISPATCH_EXPECTED)
            results.check(f"dispatch: entry {entry.index} ({entry.action_id}) has the table's "
                          f"{n_rows} {entry.kind} movements", n_mv == n_rows, f"{n_mv} loaded")
    finally:
        for name, fn in originals.items():
            setattr(husky_world, name, fn)
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Carry check: a configuration reaches the steps where no arm moves
# * ---------------------------------------------------------------------------

def carry_check(results: Results, problem: str, root: str, schedule) -> None:
    """Check the carry rule on entry 0 (J): adopting a transfer start, then accepting a transfer plan.

    (a) Adopting a transfer start (what 'Transfer start: Confirm manual pose'
    ends with) writes it into every step where no arm moves between the travel
    to load and the transfer (F5e: the bar is drawn in the tools there) and
    into the travel to load's goal. (b) Accepting a transfer plan writes its
    last waypoint into the insert's start and into every step where no arm
    moves in between (the chain fix: before, only the very next step got it).
    Fake configurations (offsets from the insert's exported start), nothing is
    planned; only the monitor's bookkeeping is checked.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy = robot_by_name(ASSEMBLY_ROBOT)
    first = schedule.entry(0)
    print(f"\n{'=' * 30} CARRY CHECK {'=' * 30}")
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, _iface, _log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    try:
        monitor._load_schedule_state()
        monitor.load_schedule_entry(first.index)
        movements = monitor._loaded_movements
        fi = monitor._loaded_index_of(MovementKind.DUAL_FREE, free_home=False)
        ti = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
        ii = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
        insert_start = vec12_from_conf(movements[ii].start_state.robot_configuration)

        def start_of(i: int) -> np.ndarray:
            return vec12_from_conf(movements[i].start_state.robot_configuration)

        def stationary_between(lo: int, hi: int) -> list:
            return [i for i in range(lo + 1, hi) if movement_kind(movements[i]) in STATIONARY_KINDS]

        # * --- (a) adopt a transfer start (no file write: the harness skips __init__)
        s = insert_start + 0.05
        monitor._m1_adopt_writes_file = False
        monitor._m1_derived = {'start_conf': s, 'goal_conf': insert_start,
                               'corridor': None, 'source': 'manual'}
        monitor.adopt_m1_derived_start()
        steps = stationary_between(fi, ti)
        diffs = [float(np.abs(start_of(i) - s).max()) for i in steps]
        goal_diff = float(np.abs(vec12_from_conf(movements[fi].target_configuration) - s).max())
        results.check(f"carry: adopting the transfer start writes it into "
                      f"{[movements[i].movement_id for i in steps]} and "
                      f"{movements[fi].movement_id}'s goal",
                      bool(steps) and max(diffs) < SEED_TOL and goal_diff < SEED_TOL,
                      f"max diffs {np.round(diffs, 9).tolist()}, goal {goal_diff:.1e}")

        # * --- (b) accept a transfer plan that ends away from the insert's start
        select_movement(monitor, ti)
        e = insert_start + 0.02
        # Display only (plots, and validators meant for a real transfer path):
        # replaced on the instance by functions that do nothing.
        display_only = ('_validate_cdfm_planned_path', 'show_transfer_validation',
                        'show_planned_joint_values')
        for name in display_only:
            setattr(monitor, name, lambda *_args, **_kwargs: None)
        try:
            monitor._accept_trajectory(movements[ti], joint_trajectory_from_path([s, e]),
                                       source='Plan')
        finally:
            for name in display_only:
                delattr(monitor, name)  # back to the monitor's own methods
        steps = stationary_between(ti, ii)
        diffs = [float(np.abs(start_of(i) - e).max()) for i in steps + [ii]]
        results.check("carry: after accepting a transfer plan, insert start = transfer's last "
                      f"waypoint (and so do {[movements[i].movement_id for i in steps]})",
                      bool(steps) and max(diffs) < SEED_TOL,
                      f"max diffs {np.round(diffs, 9).tolist()}")
    finally:
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Transfer start: confirm it and plan the travel to load in one click (F1)
# * ---------------------------------------------------------------------------

def transfer_start_check(results: Results, problem: str, root: str, schedule) -> None:
    """The 'Confirm transfer start + plan travel to load' button on entry 0 (J).

    Runs the real manual-start IK (``confirm_m1_manual_start``); the travel to
    load's planner is replaced by a recorder that plans nothing. (a) A bar
    pose 3 m away (perpendicular shift 1): the button returns False, the
    recorder is not called, the transfer stays loaded, the travel to load has
    no goal, and the warning says no arm configuration holds the bar. (b) The
    sliders at their defaults, right after a fresh 'Load entry': the recorder
    is called once with the travel to load, which is left loaded, and its goal
    is the adopted transfer start.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy = robot_by_name(ASSEMBLY_ROBOT)
    first = schedule.entry(0)
    print(f"\n{'=' * 30} TRANSFER START CHECK {'=' * 30}")
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, _iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    # The monitor's __init__ (skipped by the harness) sets the transfer-start caches.
    monitor._m1_derived = None
    monitor._m1_adopt_writes_file = False  # no file write
    monitor._m1_home_anchor_idx = 0
    monitor._m1_manual_slide_m = monitor._m1_manual_roll_deg = 0.0
    monitor._m1_manual_perp1_m = monitor._m1_manual_perp2_m = 0.0
    # Display only (the PyBullet 'Constrained t' slider): replaced by a function that does nothing.
    monitor._build_trajectory_waypoint_sliders = lambda: None
    planned = []  # the movements the travel to load's planner was called with

    def record_free_to_load(mv):
        planned.append(mv)
        return None  # nothing planned

    monitor._plan_M0_dispatch = record_free_to_load
    try:
        monitor._load_schedule_state()

        # * --- (a) a bar pose out of reach: nothing planned, the transfer stays loaded
        monitor.load_schedule_entry(first.index)
        movements = monitor._loaded_movements
        ti = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
        fi = monitor._loaded_index_of(MovementKind.DUAL_FREE, free_home=False)
        monitor._m1_manual_perp1_m = 3.0
        n = len(log.msgs)
        ok = monitor.confirm_transfer_start_and_plan_free_to_load()
        no_conf = log.since(n, 'warn', 'no arm configuration')
        nothing = log.since(n, 'warn', 'nothing planned')
        results.check(f"transfer start: bar 3 m away -> False, nothing planned, "
                      f"{movements[ti].movement_id} stays loaded, "
                      f"{movements[fi].movement_id} has no goal",
                      ok is False and not planned and monitor.current_movement_index == ti
                      and movements[fi].target_configuration is None
                      and len(no_conf) == 1 and len(nothing) == 1,
                      f"returned {ok}, planner calls {len(planned)}, "
                      f"loaded {monitor.current_movement_index}, "
                      f"warning {no_conf[0] if no_conf else 'none'!r}")

        # * --- (b) default sliders after a fresh Load entry: the travel to load is planned
        monitor._m1_manual_perp1_m = 0.0
        monitor.load_schedule_entry(first.index)
        movements = monitor._loaded_movements
        ok = monitor.confirm_transfer_start_and_plan_free_to_load()
        free_to_load = movements[fi]
        transfer_start = movements[ti].start_state.robot_configuration
        goal_diff = np.inf
        if free_to_load.target_configuration is not None and transfer_start is not None:
            goal_diff = float(np.abs(vec12_from_conf(free_to_load.target_configuration)
                                     - vec12_from_conf(transfer_start)).max())
        results.check(f"transfer start: default sliders -> {free_to_load.movement_id} loaded "
                      f"and planned once, its goal = the adopted transfer start",
                      len(planned) == 1 and planned[0] is free_to_load
                      and monitor.current_movement is free_to_load and goal_diff < SEED_TOL,
                      f"returned {ok} (the recorder plans nothing), planner calls {len(planned)}, "
                      f"loaded {monitor.current_movement_index}, |goal - transfer start| {goal_diff:.1e}")
    finally:
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Operator steps: a tool step that runs with the next arm movement is no stop (F2)
# * ---------------------------------------------------------------------------

def operator_steps_check(results: Results, problem: str, root: str, schedule) -> None:
    """The 'Step' slider on entry 0 (J) and entry 1 (R): operator steps instead of movements.

    (a) Entry 0 has 5 steps; the last one is the insert, with the joint tighten
    running with it. (b) A fake 'Step' slider widget at 4 (the last step):
    'Load Movement' loads the insert, the '-> step' readout names the insert
    and the tighten, the 'Now:' line says step 5/5. (c) Without a widget (the
    headless scripts) the selected index is still a MOVEMENT index:
    ``select_movement`` on the tighten loads the tighten. (d) 'Exec Selected
    Mv Traj' on the insert (the compliant exec replaced by a recorder) logs
    one line naming the tighten. (e) Entry 1, loaded the way the GUI does,
    has 2 steps and opens on its retreat; the retreat's exec logs one line
    each for the untighten and the ungrasp.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy = robot_by_name(ASSEMBLY_ROBOT)
    first, release = schedule.entry(0), schedule.entry(1)
    print(f"\n{'=' * 30} OPERATOR STEPS CHECK {'=' * 30}")
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    compliant_calls = []  # the movement loaded at each compliant exec

    def record_compliant(mon):
        compliant_calls.append(mon.current_movement)
        return iter(())  # a queued task that ends at once

    def exec_notes(idx: int) -> list:
        """Load an arm movement, stamp a path where the stub arms stand, press 'Exec Selected Mv Traj'.

        Args:
            idx (int): Movement index in the loaded action.

        Returns:
            list[str]: The info lines it logged about tool steps that run with it.
        """
        select_movement(monitor, idx)
        for i, q in enumerate(iface.arm_joint_pose):
            monitor.set_arm_trajectory((np.asarray([q, q]), None, monitor.trajectory_time, None), i)
        n = len(log.msgs)
        monitor.exec_selected_movement_traj()
        run_tasks(monitor, iface)
        # B1 (entry 0) is a grounded bar: the line for its tighten says it is not
        # sent (its wording is checked in grounded_insert_check).
        return (log.since(n, 'info', 'runs with this movement')
                + log.since(n, 'info', 'not sent: grounded bar'))

    real_compliant = husky_world.execute_planned_trajectory_compliant
    husky_world.execute_planned_trajectory_compliant = record_compliant
    try:
        monitor._load_schedule_state()

        # * --- (a) entry 0: five steps, the tighten runs with the insert
        monitor.load_schedule_entry(first.index)
        movements = monitor._loaded_movements
        steps = monitor._shown_steps()
        ii = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR)
        tool_idx = [i for i, mv in enumerate(movements) if tool_runs_with_next_motion(mv)]
        insert, tighten = movements[ii], movements[tool_idx[0]]
        results.check(f"steps: entry 0 has 5 steps, the last = {insert.movement_id} "
                      f"with {tighten.movement_id}",
                      len(steps) == 5 and steps[-1] == OperatorStep(ii, (tool_idx[0],)),
                      f"{steps}")

        # * --- (b) a 'Step' slider widget at 4: the insert, readout and Now line count steps
        shown = []  # what the '-> step' readout was set to
        monitor.bar_movement_text = SimpleNamespace(set_text=shown.append)
        monitor.bar_movement_slider = SimpleNamespace(value=4)
        monitor.load_selected_movement()
        monitor._refresh_bar_action_readouts()
        now = monitor._now_line()
        want_readout = f"step 5/5: {insert.movement_id}  (+ {tighten.movement_id} runs with it)"
        want_now = (f"-- step 5/5 {insert.movement_id} "
                    f"[{MovementKind.DUAL_CONSTRAINED_LINEAR.value}] ctrl={insert.controller}")
        results.check(f"steps: 'Step' slider at 4 -> {insert.movement_id} loaded, "
                      f"readout '{want_readout}', Now line '{want_now}'",
                      monitor.current_movement is insert and monitor._selected_movement_idx == ii
                      and shown == [want_readout] and now.endswith(want_now),
                      f"loaded {monitor.current_movement_index}, readout {shown}, {now!r}")

        # * --- (c) no widget: select_movement keeps meaning a movement index
        del monitor.bar_movement_slider
        mv = select_movement(monitor, tool_idx[0])
        results.check(f"steps: without a slider widget, movement {tool_idx[0]} still loads "
                      f"{tighten.movement_id}",
                      mv is tighten and monitor.current_movement_index == tool_idx[0],
                      f"loaded {monitor.current_movement_index} "
                      f"({mv.movement_id if mv is not None else None})")

        # * --- (d) exec the insert: one line names the tighten
        notes = exec_notes(ii)
        results.check(f"steps: exec {insert.movement_id} logs one line naming {tighten.movement_id}",
                      compliant_calls == [insert] and len(notes) == 1
                      and tighten.movement_id in notes[0],
                      f"compliant exec calls {len(compliant_calls)}, lines {notes}")

        # * --- (e) entry 1 as the GUI loads it: two steps, opens on the retreat
        # The live monitor opens a loaded action on its first step; the pink
        # assembled-bar line of the accuracy test is left out headless.
        monitor._is_live_monitor = True
        monitor.BAR_ACTION_MOCAP_ACCURACY_TEST = 0
        monitor.load_schedule_entry(release.index)
        movements = monitor._loaded_movements
        steps = monitor._shown_steps()
        ri = monitor._loaded_index_of(MovementKind.DUAL_INDEPENDENT_LINEAR)
        tool_idx = [i for i, mv in enumerate(movements) if tool_runs_with_next_motion(mv)]
        retreat = movements[ri]
        ids = [movements[j].movement_id for j in tool_idx]
        results.check(f"steps: entry 1 has 2 steps and opens on {retreat.movement_id} "
                      f"(with {ids})",
                      len(steps) == 2 and steps[0] == OperatorStep(ri, tuple(tool_idx))
                      and monitor.current_movement is retreat
                      and monitor._selected_movement_idx == ri,
                      f"{steps}, loaded {monitor.current_movement_index}")
        notes = exec_notes(ri)
        results.check(f"steps: exec {retreat.movement_id} logs one line each for {ids}",
                      compliant_calls[1:] == [retreat] and len(ids) == 2 and len(notes) == 2
                      and all(i in note for i, note in zip(ids, notes)),
                      f"compliant exec calls {len(compliant_calls)}, lines {notes}")
    finally:
        husky_world.execute_planned_trajectory_compliant = real_compliant
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Transfer workaround: the transfer planned as a free move without the bar
# * ---------------------------------------------------------------------------

def transfer_free_move_check(results: Results, problem: str, root: str, schedule) -> None:
    """The transfer workaround box (``TRANSFER_AS_FREE_MOVE``) on entry 0 (J).

    With the box ticked, 'Plan Movement' on the transfer (loaded by kind) runs
    with the free planner (``plan_free_dual_arm``, replaced where husky_monitor
    looks it up) recording its call and returning the two-point path [start,
    goal], and the swept check passing. (a) The goal puts both flanges on the
    transfer's target frames (1 mm / 0.01 rad) and the workaround's log line is
    printed. (b) The state handed to the planner (and to the swept check) has
    every body held in the transfer's start state -- the bar and its joints --
    hidden and not attached; the transfer's own start state still holds them.
    (c) After acceptance the tighten step and the insert start at the goal, so
    do the manual mount and the tool grasp, the transfer itself starts at the
    live arms, and the bar-hold checks did not run. (d) Box unticked: the
    bar-held planner (``_plan_M1_dispatch``, a recorder) is called and the free
    planner is not (the dispatch check's rows cover the other movements).
    Then ONE real free plan (box ticked, nothing replaced but the bar-hold
    checks) is reported as INFO, not as a check: the move ends next to the
    structure and may not be found.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy = robot_by_name(ASSEMBLY_ROBOT)
    first = schedule.entry(0)
    print(f"\n{'=' * 30} TRANSFER FREE MOVE CHECK {'=' * 30}")
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    free_plans = []    # (state, goal 12-vec) the free planner was called with
    swept_states = []  # the states the swept check was called with
    m1_calls = []      # the movements the bar-held planner was called with
    hold_checks = []   # the bar-hold checks that ran

    def record_free_plan(_planner, state, goal_conf, **_kwargs):
        goal = vec12_from_conf(goal_conf)
        free_plans.append((state, goal))
        return [vec12_from_conf(state.robot_configuration), goal], {}

    def record_swept_check(_mv, _path12, state=None):
        swept_states.append(state)
        return {'ok': True}

    def record_bar_held_plan(mv):
        m1_calls.append(mv)
        return None  # nothing planned

    real_free_plan = husky_monitor.plan_free_dual_arm
    try:
        monitor._load_schedule_state()
        monitor.load_schedule_entry(first.index)
        movements = monitor._loaded_movements
        fi = monitor._loaded_index_of(MovementKind.DUAL_FREE, free_home=False)
        ti = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_FREE)
        ii = monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR)

        def start_of(i: int) -> np.ndarray:
            return vec12_from_conf(movements[i].start_state.robot_configuration)

        def stationary_between(lo: int, hi: int) -> list:
            return [i for i in range(lo + 1, hi) if movement_kind(movements[i]) in STATIONARY_KINDS]

        # * --- box ticked: the free planner and the swept check replaced by recorders
        monitor.TRANSFER_AS_FREE_MOVE = 1
        husky_monitor.plan_free_dual_arm = record_free_plan
        monitor._validate_free_planned_path = record_swept_check
        monitor._plan_M1_dispatch = record_bar_held_plan
        for name in ('show_transfer_validation', '_validate_cdfm_planned_path'):
            setattr(monitor, name, lambda *_a, _name=name, **_k: hold_checks.append(_name))
        monitor.load_selected_movement(index=ti)
        mv = monitor.current_movement
        live = np.concatenate([np.asarray(a, dtype=float) for a in iface.arm_joint_pose])
        _, printed = call_keeping_prints(monitor.plan_selected_movement)
        if not free_plans:
            results.check(f"transfer free move: ticked -> {mv.movement_id} reaches the free planner",
                          False, f"bar-held planner calls {len(m1_calls)}")
            return
        plan_state, goal = free_plans[0]

        # * --- (a) the goal: both flanges on the transfer's target frames
        goal_state = plan_state.copy()
        monitor._merge_arm_values(goal_state, cindy.all_arm_joint_names, goal)
        residuals = [frame_residual(_fk_link_frame(monitor.cfab.planner, goal_state,
                                                   cindy.flange_for_side(side)),
                                    mv.target_ee_frames[side])
                     for side in cindy.side_keys]
        results.check(f"transfer free move: ticked -> {mv.movement_id} plans a free move whose "
                      f"goal puts both flanges on its target frames, log line printed",
                      len(free_plans) == 1 and not m1_calls
                      and '[transfer workaround] free move (no bar)' in printed
                      and all(d < FLANGE_POS_TOL and a < FLANGE_ANG_TOL for d, a in residuals),
                      f"free planner calls {len(free_plans)}, bar-held planner calls {len(m1_calls)}, "
                      "flanges " + ', '.join(f"{d * 1000:.2f} mm / {a:.4f} rad" for d, a in residuals))

        # * --- (b) the planning state: the bar and its joints let go and hidden
        held = sorted(n for n, rb in mv.start_state.rigid_body_states.items()
                      if rb.attached_to_link or rb.attached_to_tool)
        planned_rbs = plan_state.rigid_body_states
        let_go = [n for n in held if planned_rbs[n].is_hidden and not planned_rbs[n].attached_to_link
                  and not planned_rbs[n].attached_to_tool]
        results.check(f"transfer free move: the planning state has the bar and its joints hidden and "
                      f"not attached ({mv.movement_id}'s own start state still holds them)",
                      monitor.active_bar_name in held and let_go == held
                      and len(swept_states) == 1 and swept_states[0] is plan_state,
                      f"held in the start state {held}, let go and hidden {let_go}, "
                      f"swept check calls {len(swept_states)}")

        # * --- (c) after acceptance: the steps around the transfer stand at the goal
        steps = stationary_between(fi, ti) + stationary_between(ti, ii) + [ii]
        diffs = [float(np.abs(start_of(i) - goal).max()) for i in steps]
        start_diff = float(np.abs(start_of(ti) - live).max())
        results.check(f"transfer free move: after acceptance "
                      f"{[movements[i].movement_id for i in steps]} start at the goal, "
                      f"{mv.movement_id} at the live arms, no bar-hold checks",
                      mv.trajectory is not None and len(steps) == 4 and max(diffs) < SEED_TOL
                      and start_diff < SEED_TOL and not hold_checks,
                      f"max diffs {np.round(diffs, 9).tolist()}, |start - live arms| {start_diff:.1e}, "
                      f"bar-hold checks run {hold_checks}")

        # * --- (d) box unticked: the bar-held planner, as before
        monitor.TRANSFER_AS_FREE_MOVE = 0
        monitor.load_schedule_entry(first.index)
        monitor.load_selected_movement(index=ti)
        n_free = len(free_plans)
        monitor.plan_selected_movement()
        results.check(f"transfer free move: unticked -> {monitor.current_movement.movement_id} goes "
                      f"to the bar-held planner (_plan_M1_dispatch) as before",
                      len(m1_calls) == 1 and m1_calls[0] is monitor.current_movement
                      and len(free_plans) == n_free,
                      f"bar-held planner calls {len(m1_calls)}, free planner calls "
                      f"{len(free_plans) - n_free}")

        # * --- INFO, not a check: one real free plan with the box ticked
        husky_monitor.plan_free_dual_arm = real_free_plan
        delattr(monitor, '_validate_free_planned_path')  # back to the monitor's own method
        monitor.TRANSFER_AS_FREE_MOVE = 1
        monitor.load_schedule_entry(first.index)
        monitor.load_selected_movement(index=ti)
        mv = monitor.current_movement
        n = len(log.msgs)
        t0 = time.time()
        _, printed = call_keeping_prints(monitor.plan_selected_movement)
        took = time.time() - t0
        if mv.trajectory is not None:
            outcome = f"found, {len(mv.trajectory.points)} points"
        else:
            reasons = ([line.strip() for line in printed.splitlines() if 'plan_free_dual_arm failed' in line]
                       + log.since(n, 'error', 'REJECTED') + log.since(n, 'warn', '[transfer workaround]'))
            outcome = f"not found: {reasons[0] if reasons else 'see the log above'}"
        print(f"[INFO] transfer free move, one real plan (box ticked, arms at UR5e home): "
              f"{outcome} ({took:.1f} s)")
    finally:
        husky_monitor.plan_free_dual_arm = real_free_plan
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Grounded bar: its insert runs rigid and never tightens (Rhino note D9)
# * ---------------------------------------------------------------------------

def grounded_insert_check(results: Results, problem: str, root: str, schedule) -> None:
    """A grounded bar's insert runs rigid and never tightens; a normal bar's insert is unchanged.

    The fixture's entry 0 is B1, a GROUNDED bar (its insert's start state holds
    two ground joints in the tools); entry 2 is B3, a normal bar. Each insert
    (loaded by kind) gets a straight joint path from its start to its target
    configuration (nothing planned), the stub arms at its start and a short
    traj time, and runs the REAL ``husky_world.execute_planned_trajectory_compliant``
    on the stub. B1 runs through 'Exec Selected Mv Traj', pumped to its end
    like the monitor tick: (a) one joint trajectory for both arms with the
    whole path, no tool command but the motor stops, no request for the
    compliance controller; (b) it ends with its motion (no wait for the joint
    motors to stall) and logs both 'grounded bar' lines; (c) the exec says the
    absorbed tighten step is not sent (Rhino note D9) and, when the export asks
    for another controller, that it runs joint tracking. (d) B3: the exec runs
    until it asks for the compliance controller and is stopped there; it sent
    TIGHTEN to both joint motors first, as before.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder (unused; same signature as the other runs).
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy = robot_by_name(ASSEMBLY_ROBOT)
    grounded, normal = schedule.entry(0), schedule.entry(2)
    assert [e.kind for e in (grounded, normal)] == ['J', 'J'], \
        f"the grounded insert check needs jointing entries 0 and 2, got " \
        f"{[e.action_id for e in (grounded, normal)]}"
    print(f"\n{'=' * 30} GROUNDED INSERT CHECK {'=' * 30}")
    base = schedule.load_action(grounded, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    ready_for_compliant_exec(monitor)

    def load_insert(entry):
        """'Load entry', load its insert, stamp a straight joint path, stand the stub arms at its start.

        Args:
            entry (ScheduleEntry): One of Cindy's jointing entries.

        Returns:
            tuple: ``(the insert, its (n, 12) path)``.
        """
        monitor.load_schedule_entry(entry.index)
        mv = select_movement(monitor, monitor._loaded_index_of(MovementKind.DUAL_CONSTRAINED_LINEAR))
        path = np.linspace(vec12_from_conf(mv.start_state.robot_configuration),
                           vec12_from_conf(mv.target_configuration), INSERT_PATH_WAYPOINTS)
        monitor.trajectory_time = INSERT_TRAJ_TIME_S
        for i in range(cindy.n_arms):
            arm_path = path[:, 6 * i:6 * (i + 1)]
            monitor.set_arm_trajectory((arm_path, None, INSERT_TRAJ_TIME_S, None), i)
            iface.arm_joint_pose[i] = arm_path[0].copy()
        return mv, path

    def tool_cmds(n: int) -> list:
        """Scaffolding commands after the first ``n`` stub calls, the stops left out.

        Args:
            n (int): How many stub calls to skip.

        Returns:
            list[tuple]: Sorted ``(direction, motor, arm)``.
        """
        return sorted(c[1:] for c in iface.calls[n:] if c[0] == 'scaffolding' and c[1] != 0)

    def compliance_requests(n: int) -> list:
        """Requests to switch to the compliance controller after the first ``n`` stub calls.

        Args:
            n (int): How many stub calls to skip.

        Returns:
            list[tuple]: The recorded ``switch`` calls.
        """
        return [c for c in iface.calls[n:] if c[0] == 'switch' and c[2] == COMPLIANCE_CTRL]

    try:
        monitor._load_schedule_state()

        # * --- (a)-(c) B1: the whole insert through the exec button, to its end
        mv, path = load_insert(grounded)
        ground_joints = held_ground_joints(mv.start_state)
        tighten = [m.movement_id for m in monitor._loaded_movements if tool_runs_with_next_motion(m)]
        n_calls, n_log = len(iface.calls), len(log.msgs)
        t0 = time.time()
        monitor.exec_selected_movement_traj()
        queued = len(monitor.tasks)
        run_tasks(monitor, iface)
        took = time.time() - t0
        joint_cmds = [c for c in iface.calls[n_calls:] if c[0] == 'dual_arm']
        stops = {c[1:] for c in iface.calls[n_calls:] if c[0] == 'scaffolding' and c[1] == 0}
        all_stops = {(0, motor, arm) for motor in (GRIPPER_MOTOR, JOINT_MOTOR) for arm in (0, 1)}
        results.check(
            f"grounded insert: {mv.movement_id} (holds {ground_joints}) runs as ONE joint "
            f"trajectory for both arms with the whole path, sends only the motor stops, "
            f"never asks for {COMPLIANCE_CTRL}",
            len(ground_joints) == 2 and queued == 1
            and joint_cmds == [('dual_arm', (len(path), len(path)), INSERT_TRAJ_TIME_S)]
            and not tool_cmds(n_calls) and stops == all_stops
            and not compliance_requests(n_calls),
            f"joint commands {joint_cmds}, tool commands {tool_cmds(n_calls)}, "
            f"stops {sorted(stops)}, compliance requests {len(compliance_requests(n_calls))}")

        started = log.since(n_log, 'info', 'grounded bar (holds ')
        done = log.since(n_log, 'info', 'grounded bar: rigid insert done (no tighten).')
        results.check(
            f"grounded insert: {mv.movement_id} ends with its {INSERT_TRAJ_TIME_S} s motion, "
            f"no wait for the joint motors to stall (up to {HOLD_FOR_STALL_TIMEOUT_S:.0f} s), "
            f"both 'grounded bar' lines logged",
            took < GROUNDED_INSERT_MAX_S and len(started) == 1 and len(done) == 1
            and all(j in started[0] for j in ground_joints),
            f"took {took:.2f} s, lines {started + done}")

        notes = log.since(n_log, 'info', 'not sent: grounded bar')
        tool_lines = log.since(n_log, 'info', 'runs with this movement')
        ctrl_warns = log.since(n_log, 'warn', 'the monitor runs this')
        # The export may already ask for joint tracking (Rhino note D9): then no warning.
        want_warns = int(mv.controller != CONTROLLER_JOINT_TRACKING)
        results.check(
            f"grounded insert: exec of {mv.movement_id} logs "
            f"'{', '.join(tighten)} not sent: grounded bar' "
            f"(Rhino note D9) and says it runs under {CONTROLLER_JOINT_TRACKING}",
            len(tighten) == 1 and len(notes) == 1
            and notes[0].startswith(f"{tighten[0]} not sent: grounded bar") and not tool_lines
            and len(ctrl_warns) == want_warns
            and all(w.endswith(f"under {CONTROLLER_JOINT_TRACKING}.") for w in ctrl_warns),
            f"notes {notes}, tool step lines {tool_lines}, controller warnings {ctrl_warns}")

        # * --- (d) B3, a normal bar: the exec until it asks for the compliance controller
        mv, path = load_insert(normal)
        n_calls, n_log = len(iface.calls), len(log.msgs)
        task = husky_world.execute_planned_trajectory_compliant(monitor)
        try:
            for _ in range(3000):
                next(task)
                if compliance_requests(n_calls):
                    break
                iface.tick()
                time.sleep(TICK_S)
        except StopIteration:
            pass  # the exec ended on its own (its log says why)
        finally:
            # Stopped while it waits for the switch: the compliance controller
            # never became active, so its own clean-up only stops the motors.
            task.close()
        joint_cmds = [c for c in iface.calls[n_calls:] if c[0] == 'dual_arm']
        results.check(
            f"grounded insert: {mv.movement_id} (a normal bar, holds "
            f"{held_ground_joints(mv.start_state)}) still TIGHTENs both joint motors, "
            f"then asks for {COMPLIANCE_CTRL}",
            not held_ground_joints(mv.start_state)
            and tool_cmds(n_calls) == [(1, JOINT_MOTOR, 0), (1, JOINT_MOTOR, 1)]
            and len(compliance_requests(n_calls)) == cindy.n_arms
            and not log.since(n_log, 'info', 'grounded bar'),
            f"tool commands {tool_cmds(n_calls)}, compliance requests "
            f"{len(compliance_requests(n_calls))}, rigid approach before it: "
            f"{f'{joint_cmds[0][1][0]} waypoints' if joint_cmds else 'none'}")
    finally:
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Built-structure switch: collisions with the built bars checked or ignored
# * ---------------------------------------------------------------------------

def body_color(monitor, name: str) -> list:
    """The RGBA a rigid body is drawn with in the planner's PyBullet world.

    Args:
        monitor (HuskyMonitor): The headless monitor.
        name (str): Rigid-body name in the cell.

    Returns:
        list[float]: RGBA of the body's first visual shape.
    """
    body = monitor.cfab.client.rigid_bodies_puids[name][0]
    return list(p.getVisualShapeData(body, physicsClientId=pp.CLIENT)[0][7])


def built_bars_check(results: Results, monitor, log: RecordingLogger, schedule) -> None:
    """The built-bar switch (IGNORE_BUILT_ASSEMBLY_COLLISIONS) on Cindy's entries 0 and 1.

    Switch OFF (its default): the start and end states of entry 0's insert
    (movement 5, ``B1_J_M5_LM_insert``) have no pair with the floor
    ``obstacle_ground`` in the full collision report -- the ground joints may
    stand on it -- and entry 1
    (B1_R) never hides B1 or its joints. Then the panel toggle's method turns it
    ON: the entry reloads, the built bodies are ignored and drawn faint where
    they stand (even after something moved one: compas_fab does not move hidden
    bodies, the monitor places them), bodies the export hides (not built yet)
    stay blanked. OFF again: the reload brings clean states back. Last, a joint
    standing in J_M0 (faint when ON) and riding in J_M3's preview is drawn as
    usual again once the switch is OFF. Ends with the switch OFF.

    Args:
        results (Results): Where the checks go.
        monitor (HuskyMonitor): Cindy's monitor in schedule mode, switch OFF.
        log (RecordingLogger): The monitor's logger.
        schedule (ActionSchedule): The scratch schedule.
    """
    joint_entry, release = schedule.entry(0), schedule.entry(1)
    results.check('Built bars: the switch is OFF by default in schedule mode',
                  not monitor._ignore_built_assembly())

    # * --- the ground joints may stand on the floor during the insert
    monitor.load_schedule_entry(joint_entry.index)
    mv = select_movement(monitor, 5)
    # ! The ground joints reach the floor only at the insert's END (bar assembled):
    # ! its start state is clean even without the allowance. So the end state --
    # ! the start state with the robot at the target configuration -- is checked too.
    end = mv.start_state.copy()
    end.robot_configuration = mv.target_configuration
    for label, state in (('start', mv.start_state), ('end', end)):
        lines = collision_lines(monitor.cfab.planner, state)
        print(f"  full collision report of {mv.movement_id}'s {label} state: {len(lines)} pair(s)")
        for line in lines:
            print(f"    {line}")
        ground = [line for line in lines if 'obstacle_ground' in line]
        results.check(f'Built bars OFF: {mv.movement_id} {label} state has no obstacle_ground pair '
                      f'(full collision report)',
                      mv.movement_id.endswith('_LM_insert')
                      and 'obstacle_ground' in state.rigid_body_states and not ground,
                      f"{len(lines)} other colliding pair(s), listed above" if not ground
                      else '; '.join(ground))

    # * --- the release: B1 (bar + joints) is never hidden
    monitor.load_schedule_entry(release.index)
    mv = select_movement(monitor, 0)
    held = sorted(n for n, rb in mv.start_state.rigid_body_states.items()
                  if is_built_assembly_body(n) and (rb.attached_to_link or rb.attached_to_tool))

    def hidden_somewhere(names: list) -> list:
        """The names hidden in the start state of any loaded movement."""
        return sorted({n for m in monitor._loaded_movements for n in names
                       if m.start_state.rigid_body_states[n].is_hidden})

    results.check(f"Built bars OFF: {release.action_id} never hides {monitor.active_bar_name} and "
                  f"its joints, nothing ignored",
                  monitor.active_bar_name in held and not hidden_somewhere(held)
                  and not monitor._collision_ignored_bodies,
                  f"{len(held)} bodies held at {mv.movement_id}")

    # * --- the panel toggle ON: reload, the built joints ignored and drawn faint
    clean = schedule.load_action(release, prefer_sidecar=False).movements
    # What the monitor must ignore: every static built body of the action that
    # the export does not hide, except the action's own bar.
    built = {n for m in clean for n, rb in m.start_state.rigid_body_states.items()
             if is_built_assembly_body(n) and n != monitor.active_bar_name and not rb.is_hidden
             and not (rb.attached_to_link or rb.attached_to_tool)}
    n = len(log.msgs)
    monitor.set_ignore_built_assembly_collisions(True)
    select_movement(monitor, 0)
    ignored = set(monitor._collision_ignored_bodies)
    logged = log.since(n, 'info', f"[built bars] collisions with {len(built)} built bodies "
                                  f"ignored (drawn faint)")
    results.check(f"Built bars ON (toggle): entry {release.index} reloaded, its built bodies "
                  f"ignored, one log line",
                  monitor._loaded_entry.index == release.index and built and ignored == built
                  and len(logged) == 1, f"{len(ignored)} ignored: {sorted(ignored)}")

    # Something moves a built joint away; the next Load Movement puts it back.
    joint = sorted(built)[0]
    body = monitor.cfab.client.rigid_bodies_puids[joint][0]
    pp.set_pose(body, ((50.0, 50.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    mv = select_movement(monitor, 2)
    rb_states = mv.start_state.rigid_body_states
    pos_err = float(np.abs(np.subtract(pp.get_pose(body)[0], list(rb_states[joint].frame.point))).max())
    not_built = sorted(n for n, rb in rb_states.items()
                       if is_built_assembly_body(n) and rb.is_hidden and n not in ignored)[0]
    results.check(f"Built bars ON: at {mv.movement_id} {joint} drawn faint at its state frame, "
                  f"{not_built} (not built yet) blanked",
                  rb_states[joint].is_hidden and pos_err < SAME_TOL
                  and np.allclose(body_color(monitor, joint), BUILT_IGNORED_RGBA)
                  and body_color(monitor, not_built)[3] == 0.0,
                  f"|pos - frame| {pos_err:.1e} m, rgba {np.round(body_color(monitor, joint), 2).tolist()}")

    # * --- OFF again: clean states, drawn as usual
    monitor.set_ignore_built_assembly_collisions(False)
    select_movement(monitor, 2)
    results.check(f"Built bars OFF again (toggle): entry {release.index} reloaded clean, "
                  f"nothing ignored, {joint} drawn as usual",
                  not monitor._collision_ignored_bodies and not hidden_somewhere(held)
                  and body_color(monitor, joint)[3] > BUILT_IGNORED_RGBA[3],
                  f"rgba {np.round(body_color(monitor, joint), 2).tolist()}")

    # * --- the same joint stands in J_M0 (faint when ON) and rides in J_M3's
    # * --- preview: switched OFF, the faint colour must not come back
    monitor.load_schedule_entry(joint_entry.index)
    monitor.set_ignore_built_assembly_collisions(True)
    select_movement(monitor, 0)
    faint_on = np.allclose(body_color(monitor, joint), BUILT_IGNORED_RGBA)
    select_movement(monitor, 3)
    monitor.set_ignore_built_assembly_collisions(False)
    mv = select_movement(monitor, 0)
    results.check(f"Built bars: {joint} faint at {mv.movement_id} when ON, drawn as usual there "
                  f"after a held preview and the switch OFF",
                  faint_on and body_color(monitor, joint)[3] > BUILT_IGNORED_RGBA[3],
                  f"rgba {np.round(body_color(monitor, joint), 2).tolist()}")


# * ---------------------------------------------------------------------------
# * The two runs
# * ---------------------------------------------------------------------------

def cindy_run(results: Results, problem: str, root: str, schedule) -> None:
    """Cindy's monitor: her own entry, Alice's entries (display only), the next release, reopen.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder.
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy, alice, belle = (robot_by_name(ASSEMBLY_ROBOT), robot_by_name(SUPPORT_ROBOT),
                           robot_by_name(OTHER_SUPPORT_ROBOT))
    first = schedule.entry(0)
    hold = schedule.find_entry(HELD_BAR, 'H')
    release = schedule.find_entry(HELD_BAR, 'R')
    hold_release = schedule.find_entry(HELD_BAR, 'HR')
    oracle_entry = schedule.find_entry(NEXT_BAR, 'J')
    parked = np.asarray(list(PARKED_BASE_FRAME.point), dtype=float)
    print(f"\n{'=' * 30} CINDY RUN {'=' * 30}")
    # Cindy's "mocap" puts her where her first action was authored.
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    try:
        monitor._load_schedule_state()
        results.check('Cindy: schedule mode ON, first pending entry selected',
                      monitor._schedule is not None and monitor._selected_entry_idx == 0,
                      f"{len(monitor._schedule.entries) if monitor._schedule else 0} entries")

        # * --- the built-bar switch on her first two entries (OFF again afterwards)
        built_bars_check(results, monitor, log, schedule)

        # * --- her own entry: load + Mark entry done
        monitor.load_schedule_entry(first.index)
        results.check(f'Cindy: entry {first.index} ({first.action_id}) loads for execution',
                      monitor._entry_is_executable_here()
                      and monitor._loaded_action.action_id == first.action_id)

        # * --- one task at a time: while the manual step waits for 'Confirm Exec',
        # * --- neither Mark entry done nor Load entry may queue / change anything
        mv = select_movement(monitor, 1)
        monitor.exec_selected_movement_step()
        next(monitor.tasks[0])    # one tick: the step now waits at its confirm gate
        n = len(log.msgs)
        monitor.mark_entry_done()
        monitor.load_schedule_entry(hold.index)
        refused = log.since(n, 'warn', 'still running or waiting')
        results.check(f"Cindy: while {mv.movement_id} waits, Mark entry done and Load entry "
                      f"are refused", len(monitor.tasks) == 1 and len(refused) == 2
                      and monitor._loaded_entry.index == first.index, f"{len(refused)} refusal(s)")
        monitor._servo_abort = True    # 'Cancel Exec': the manual step ends
        run_tasks(monitor, iface, confirm=False)
        results.check("Cindy: 'Cancel Exec' ends the step, nothing marked",
                      not monitor.tasks and not monitor._progress.is_done(first.index))

        monitor.mark_entry_done()
        run_tasks(monitor, iface)
        progress = load_progress(root, load_schedule(root))
        belief = progress.belief(cindy.name)
        results.check(f"Cindy: entry {first.index} done in progress.json, her belief 'live'",
                      progress.is_done(first.index) and belief is not None
                      and belief.source == 'live' and belief.after_entry == first.index)
        results.check('Cindy: Alice and Belle (no belief yet) drawn parked',
                      np.allclose(drawn_base(monitor, alice.name), parked)
                      and np.allclose(drawn_base(monitor, belle.name), parked))

        # * --- Alice's hold entry: the previous planned path is dropped, display only,
        # * --- refused, marked through the confirm gate
        path = np.array([UR5e_HOME_STATE, UR5e_HOME_STATE])
        monitor.planned_arm_trajectory = [(path, None, 5.0, None), (path, None, 5.0, None)]
        monitor.load_schedule_entry(hold.index)
        results.check(f'Cindy: Load entry {hold.index} drops the loaded movement and planned path',
                      monitor.current_movement is None
                      and all(slot[0] is None for slot in monitor.planned_arm_trajectory))
        shown = [mv.movement_id for mv in monitor._shown_movements()]
        results.check(f'Cindy: entry {hold.index} ({hold.action_id}) is display only',
                      not monitor._entry_is_executable_here() and len(shown) == 4
                      and all(m.startswith(f'{HELD_BAR}_H_M') for m in shown), ', '.join(shown))
        n = len(log.msgs)
        monitor.plan_selected_movement()
        monitor.exec_selected_movement_step()
        refused = log.since(n, 'warn', 'refused')
        results.check('Cindy: Plan Movement and Exec step refused on it',
                      len(refused) == 2 and not monitor.tasks, f"{len(refused)} refusal(s)")
        monitor.mark_entry_done()
        # One tick without clicking Confirm Exec: the task waits at the gate.
        next(monitor.tasks[0])
        results.check(f"Cindy: Mark entry done on entry {hold.index} waits for 'Confirm Exec'",
                      not monitor._progress.is_done(hold.index) and len(monitor.tasks) == 1)
        # Mocap sees Belle right now: her drawn base must stay where mocap put it.
        belle_hi = monitor.husky_by_name[belle.name].interface
        belle_hi.position = np.array([1.0, 2.0, 0.0])
        monitor._mocap_rigidbody_stamp = {belle.namespace: time.monotonic()}
        run_tasks(monitor, iface, confirm=True)
        monitor._mocap_rigidbody_stamp = {}
        progress = load_progress(root, load_schedule(root))
        belief = progress.belief(alice.name)
        oracle = belief_after(schedule.load_action(hold, prefer_sidecar=False), alice, hold.index)
        ok = (progress.is_done(hold.index) and belief is not None
              and belief.source == 'assumed'
              and frame_dist(belief.base_frame, oracle.base_frame) < SAME_TOL
              and np.allclose(belief.configuration.joint_values, oracle.configuration.joint_values,
                              atol=SAME_TOL)
              and progress.hold_state(HELD_BAR).state == 'holding')
        results.check(f"Cindy: after Confirm Exec entry {hold.index} is done, Alice's belief "
                      f"'assumed' = {hold.action_id}'s end, {HELD_BAR} held", ok,
                      f"base {pt(belief.base_frame) if belief else None}")
        alice_hi = monitor.husky_by_name[alice.name].interface
        drawn_conf = alice_hi.arm_joint_pose[0]
        results.check("Cindy: drawn Alice moved to that belief; Belle (seen by mocap) kept her base",
                      np.allclose(drawn_base(monitor, alice.name), list(belief.base_frame.point))
                      and np.allclose(drawn_conf, [belief.configuration[n]
                                                   for n in alice.arm_joint_names[0]])
                      and np.allclose(drawn_base(monitor, belle.name), [1.0, 2.0, 0.0]),
                      f"Alice at {np.round(drawn_base(monitor, alice.name), 4).tolist()}")

        # * --- the release: Alice posed from her belief
        monitor.load_schedule_entry(release.index)
        tool = alice.obstacle_tool_name
        posed = monitor._loaded_movements[0].start_state.tool_states[tool].frame
        exported = schedule.load_action(oracle_entry, prefer_sidecar=False) \
            .movements[0].start_state.tool_states[tool].frame
        results.check(f"Cindy: entry {release.index} ({release.action_id}) movement 0 poses {tool} "
                      f"from the belief, == {oracle_entry.action_id}'s export",
                      frame_dist(posed, PARKED_BASE_FRAME) > 1.0
                      and frame_dist(posed, belief.base_frame) < SAME_TOL
                      and frame_dist(posed, exported) < SAME_TOL, f"at {pt(posed)}")
        n = len(log.msgs)
        monitor.reopen_entry()
        results.check(f"Cindy: Reopen of the loaded, still pending entry {release.index} is refused",
                      len(log.since(n, 'warn', 'still pending')) == 1
                      and monitor._progress.status(release.index) == 'pending')

        # * --- plan the release with Alice posed from her belief: the retreat, then the free move home
        plan_release(results, monitor, iface, alice, belief)

        # * --- Reset All in schedule mode: the loaded ENTRY again, from its clean export
        monitor.reset_all_movements_to_clean()
        results.check(f"Cindy: Reset All reloads entry {release.index} from its clean export",
                      monitor._loaded_entry.index == release.index
                      and monitor._current_action_path == schedule.action_path(release, prefer_sidecar=False)
                      and all(mv.trajectory is None for mv in monitor._loaded_movements),
                      os.path.basename(monitor._current_action_path))

        # * --- Alice's hold release: she leaves the scene (drawn parked)
        monitor.load_schedule_entry(hold_release.index)
        monitor.mark_entry_done()
        run_tasks(monitor, iface, confirm=True)
        results.check(f"Cindy: entry {hold_release.index} ({hold_release.action_id}) done -> Alice "
                      f"released and drawn parked",
                      monitor._progress.is_done(hold_release.index)
                      and alice.name in monitor._progress.released_robots()
                      and np.allclose(drawn_base(monitor, alice.name), parked))
        # The slider now points at the first pending entry, not at the loaded one.
        n = monitor._selected_entry_idx
        monitor.reopen_entry()
        # The slider shows another entry, so the reopen first waits for 'Confirm Exec'.
        results.check(f"Cindy: Reopen (no index) of the LOADED entry {hold_release.index} while the "
                      f"slider shows {n} waits for 'Confirm Exec'",
                      n != hold_release.index and len(monitor.tasks) == 1
                      and monitor._progress.is_done(hold_release.index))
        run_tasks(monitor, iface, confirm=True)
        restored = monitor._progress.belief(alice.name)
        results.check(f"Cindy: after Confirm Exec the LOADED entry {hold_release.index} is reopened, "
                      f"Alice holds {HELD_BAR} again with her belief restored from entry {hold.index}",
                      monitor._progress.status(hold_release.index) == 'pending'
                      and monitor._progress.hold_state(HELD_BAR).state == 'holding'
                      and monitor._loaded_entry.index == hold_release.index
                      and restored is not None and restored.after_entry == hold.index,
                      f"belief after_entry={getattr(restored, 'after_entry', None)}")

        # * --- reopen the hold entry
        monitor.reopen_entry(hold.index)
        progress = load_progress(root, load_schedule(root))
        results.check(f"Cindy: reopen entry {hold.index} -> pending, Alice's belief dropped, hold pending",
                      progress.status(hold.index) == 'pending' and progress.belief(alice.name) is None
                      and progress.hold_state(HELD_BAR).state == 'pending')
    finally:
        monitor.cfab.close()


def plan_release(results: Results, monitor, iface: StubInterface, alice, belief) -> None:
    """Plan Cindy's release movements with the loaded release entry, Alice posed from her belief.

    The retreat (independent dual-arm linear) plans from its exported start;
    its end is then taken as "executed" (the stub arms jump there) and the free
    move home plans from those live arms. Both are found by kind, not by index.

    Args:
        results (Results): Where the checks go.
        monitor (HuskyMonitor): Cindy's monitor with the release entry loaded.
        iface (StubInterface): Cindy's stub robot.
        alice (RobotSpec): The support robot holding the bar.
        belief (RobotBelief): Alice's belief after her hold entry.
    """
    tool = alice.obstacle_tool_name
    retreat_idx = monitor._loaded_index_of(MovementKind.DUAL_INDEPENDENT_LINEAR)
    home_idx = monitor._loaded_index_of(MovementKind.DUAL_FREE, free_home=True)
    # Cindy's "mocap" now sees her where the release was authored.
    retreat = monitor._loaded_movements[retreat_idx]
    iface.position, iface.rotation = [np.asarray(v, dtype=float) for v in
                                      pose_from_frame(retreat.start_state.robot_base_frame)]

    mv = select_movement(monitor, retreat_idx)
    monitor.plan_selected_movement()
    jt = mv.trajectory
    posed = mv.start_state.tool_states[tool].frame
    results.check(f"Cindy: {mv.movement_id} ({movement_kind(mv).value}) planned "
                  f"with {tool} at Alice's belief",
                  jt is not None and frame_dist(posed, belief.base_frame) < SAME_TOL,
                  f"{len(jt.points) if jt else 0} points, {tool} at {pt(posed)}")
    if jt is None:
        return
    # "Executed": the arms stand at the retreat's last waypoint.
    iface.arm_joint_pose = [np.asarray(monitor.planned_arm_trajectory[i][0][-1], dtype=float)
                            for i in range(2)]

    # A step without a trajectory: Load Movement drops the retreat's planned path.
    select_movement(monitor, 0)
    results.check('Cindy: Load Movement of R_M0 (a tool step) drops the planned path',
                  all(slot[0] is None for slot in monitor.planned_arm_trajectory))

    mv = select_movement(monitor, home_idx)
    monitor.plan_selected_movement()
    jt = mv.trajectory
    posed = mv.start_state.tool_states[tool].frame
    results.check(f"Cindy: {mv.movement_id} ({movement_kind(mv).value}) planned "
                  f"from the retreat's end with {tool} at Alice's belief",
                  jt is not None and frame_dist(posed, belief.base_frame) < SAME_TOL,
                  f"{len(jt.points) if jt else 0} points")


def alice_run(results: Results, problem: str, root: str, schedule) -> None:
    """Alice's monitor: run her hold entry end to end, with the compliant gripper handoff.

    Ends with ``alice_live_goal_check`` on the same monitor.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder.
        schedule (ActionSchedule): The scratch schedule.
    """
    alice = robot_by_name(SUPPORT_ROBOT)
    hold = schedule.find_entry(HELD_BAR, 'H')
    clean = schedule.load_action(hold, prefer_sidecar=False)
    clean_path = schedule.action_path(hold, prefer_sidecar=False)
    clean_sha = sha1(clean_path)
    print(f"\n{'=' * 30} ALICE RUN {'=' * 30}")
    # Alice's "mocap" puts her where B3__H was authored; her arm starts at UR5e home.
    monitor, iface, log = make_monitor(alice, clean.movements[-1].start_state.robot_base_frame, problem)
    # ! Alice's exported hold scene collides with the built structure (frozen Cindy
    # ! vs future bar B5, Rhino note D5), so her plans need the built-bar switch ON
    # ! (the panel's "Ignore built-bar collisions"). Set before her entry is loaded.
    monitor.IGNORE_BUILT_ASSEMBLY_COLLISIONS = 1
    try:
        monitor._load_schedule_state()
        monitor.load_schedule_entry(hold.index)
        results.check(f'Alice: entry {hold.index} ({hold.action_id}) is hers, loaded on her cell',
                      monitor._entry_is_executable_here()
                      and monitor._loaded_action.action_id == hold.action_id
                      and monitor.cfab.cell_filename == alice.cell_file)

        # * --- M0: free move from the live joints, then execute
        mv = select_movement(monitor, 0)
        monitor.plan_selected_movement()
        jt0 = mv.trajectory
        monitor.exec_selected_movement_traj()
        at_target = np.abs(iface.arm_joint_pose[0]
                           - [mv.target_configuration[n] for n in alice.arm_joint_names[0]]).max()
        results.check(f'Alice: {mv.movement_id} planned and executed',
                      jt0 is not None and iface.calls[-1][0] == 'arm' and at_target < SAME_TOL,
                      f"{len(jt0.points) if jt0 else 0} points")

        # * --- M1: gripper OPEN step
        mv = select_movement(monitor, 1)
        monitor.exec_selected_movement_step()
        run_tasks(monitor, iface)
        res = iface.gripper_result[0]
        results.check(f'Alice: {mv.movement_id} opens the gripper',
                      ('gripper', GRIPPER_OPEN_POS, 0) in iface.calls and res is not None
                      and res['reached_goal'], f"fingers at {iface.finger_pos:.3f}")

        # * --- M2: linear approach from M0's end, then execute
        mv = select_movement(monitor, 2)
        monitor.plan_selected_movement()
        jt2 = mv.trajectory
        monitor.exec_selected_movement_traj()
        err = np.abs(iface.arm_joint_pose[0]
                     - [mv.target_configuration[n] for n in alice.arm_joint_names[0]]).max()
        results.check(f'Alice: {mv.movement_id} planned and executed',
                      jt2 is not None and iface.calls[-1][0] == 'arm' and err < CONF_TOL,
                      f"{len(jt2.points) if jt2 else 0} points, |end - target conf| {err:.1e} rad")

        # * --- M3: gripper CLOSE with the compliant handoff
        # The handoff needs the compliance controller wired (on hardware:
        # CONNECT_COMPLIANT_CONTROLLER = 1) and reads tool0 by FK on the ghost
        # robot; headless, the planner's robot body (Alice's URDF) is the ghost.
        monitor.CONNECT_COMPLIANT_CONTROLLER = 1
        monitor.goal_model.robot = monitor.cfab.client.robot_puid
        iface.tcp_fn = lambda i: _live_tool0_in_arm_base(monitor, i)
        mv = select_movement(monitor, 3)
        n_calls = len(iface.calls)
        monitor.exec_selected_movement_step()
        ticks = run_tasks(monitor, iface)
        calls = iface.calls[n_calls:]
        kinds = [c[0] for c in calls]
        switches = [(c[1], c[2]) for c in calls if c[0] == 'switch']
        cart = [c for c in calls if c[0] == 'cartesian']
        forces = [c for c in calls if c[0] == 'force']
        res = iface.gripper_result[0]
        print(f"  controller history (arm 0): {' -> '.join(iface.controller_history)}")
        print(f"  command order: {kinds[:6]} ... {kinds[-3:]} ({ticks} ticks)")
        results.check(f'Alice: {mv.movement_id} controllers joint -> compliance -> joint',
                      iface.controller_history == [JOINT_CTRL, COMPLIANCE_CTRL, JOINT_CTRL]
                      and switches == [(JOINT_CTRL, COMPLIANCE_CTRL), (COMPLIANCE_CTRL, JOINT_CTRL)])
        results.check(f'Alice: {mv.movement_id} F/T zeroed, close sent, THEN compliance; '
                      f'fixed target + zero wrench while compliant',
                      kinds.index('zero_ft') < kinds.index('gripper') < kinds.index('switch')
                      and ('gripper', GRIPPER_CLOSE_FOR_BAR_POS, 0) in calls
                      and cart and all(c[2] == COMPLIANCE_CTRL for c in cart)
                      and len({c[3] for c in cart}) == 1
                      and forces and all(c[1] == (0.0, 0.0, 0.0) for c in forces),
                      f"{len(cart)} compliance targets")
        results.check(f'Alice: {mv.movement_id} gripper stalled on the bar',
                      res is not None and res['stalled'], f"fingers at {iface.finger_pos:.3f}")

        # * --- Mark entry done: live belief, sidecar with the planned trajectories
        monitor.mark_entry_done()
        run_tasks(monitor, iface)
        progress = load_progress(root, load_schedule(root))
        belief = progress.belief(alice.name)
        oracle = belief_after(clean, alice, hold.index)
        conf_err = max(abs(belief.configuration[n] - oracle.configuration[n])
                       for n in oracle.configuration.joint_names) if belief else np.inf
        results.check(f"Alice: entry {hold.index} done, belief 'live' at {hold.action_id}'s end, "
                      f"{HELD_BAR} held",
                      progress.is_done(hold.index) and belief is not None and belief.source == 'live'
                      and frame_dist(belief.base_frame, oracle.base_frame) < SAME_TOL
                      and conf_err < CONF_TOL and progress.hold_state(HELD_BAR).state == 'holding',
                      f"|conf - exported end| {conf_err:.1e} rad")
        sidecar = schedule.action_path(hold)
        results.check('Alice: trajectories saved to the sidecar, clean export untouched',
                      sidecar.endswith('.live-solved.json') and os.path.isfile(sidecar)
                      and sha1(clean_path) == clean_sha, os.path.basename(sidecar))

        # * --- reload from the sidecar: H_M0 still starts from the LIVE arm, not
        # * --- the start configuration the sidecar stored
        monitor.load_schedule_entry(hold.index)
        start = monitor._loaded_movements[0].start_state.robot_configuration
        live_err = np.abs(np.subtract([start[n] for n in alice.arm_joint_names[0]],
                                      iface.arm_joint_pose[0])).max()
        results.check('Alice: Load entry opens the sidecar; H_M0 re-snapshots the live arm',
                      monitor._loaded_entry_bundle.path == sidecar
                      and 0 in monitor._live_start_indices and live_err < SAME_TOL,
                      f"|start - live| {live_err:.1e} rad")

        # * --- Reopen: the clean export, and it stays clean for the rest of this run
        monitor.reopen_entry()
        monitor.load_schedule_entry(hold.index)
        results.check('Alice: after Reopen, Load entry keeps opening the clean export this run',
                      monitor._loaded_entry_bundle.path == clean_path and os.path.isfile(sidecar)
                      and not monitor._schedule_flags[hold.index].sidecar
                      and all(mv.trajectory is None for mv in monitor._loaded_movements))

        # * --- F3: the free approach's goal re-solved by IK at a tracked base
        alice_live_goal_check(results, monitor, iface, log, hold)
    finally:
        monitor.cfab.close()


def alice_live_goal_check(results: Results, monitor, iface: StubInterface,
                          log: RecordingLogger, hold) -> None:
    """Alice's free approach plans to the goal IK solves at her LIVE base (F3).

    (a) Base not tracked (USE_MOCAP 0): the goal is the exported target, no IK
    runs. (b) Base tracked and moved +3 cm in x, arms at home: the free approach
    plans from the live base, its flange ends on the exported approach pose,
    and its goal differs from the exported one. (c) The gripper step and the
    linear approach start where the free approach ends. (d) The linear approach
    plans. (e) Base +2 m (out of reach): no plan, one 'not planning' warning.
    Movements are found by kind. The monitor's flags (set on the instance) and
    the stub's pose are restored at the end.

    Args:
        results (Results): Where the checks go.
        monitor (HuskyMonitor): Alice's monitor, her hold entry loaded from the clean export.
        iface (StubInterface): Alice's stub robot.
        log (RecordingLogger): Alice's logger.
        hold (ScheduleEntry): Her hold entry.
    """
    alice = monitor._connected_robot()
    side = alice.side_keys[0]
    names = list(alice.arm_joint_names[0])
    exported = monitor._schedule.load_action(hold, prefer_sidecar=False)
    free_idx = monitor._loaded_index_of(MovementKind.SINGLE_FREE)
    linear_idx = monitor._loaded_index_of(MovementKind.SINGLE_LINEAR)
    exported_goal = np.array([exported.movements[free_idx].target_configuration[n] for n in names])
    authored_pos, authored_rot = pose_from_frame(
        exported.movements[free_idx].start_state.robot_base_frame)

    def goal_of(mv) -> np.ndarray:
        return np.array([mv.target_configuration[n] for n in names])

    def arms_at_start(i: int) -> np.ndarray:
        conf = monitor._loaded_movements[i].start_state.robot_configuration
        return np.array([conf[n] for n in names])

    flags = ('USE_MOCAP', 'USE_CELL_STATE_BASE_POSE', 'FAKE_HARDWARE')
    own_flags = {name: vars(monitor)[name] for name in flags if name in vars(monitor)}
    saved_pose = (iface.position.copy(), iface.rotation.copy(),
                  [np.asarray(a, dtype=float).copy() for a in iface.arm_joint_pose])
    try:
        # * --- (a) base not tracked: the exported goal, no IK
        monitor.USE_MOCAP = 0
        mv = select_movement(monitor, free_idx)
        ok, printed = call_keeping_prints(monitor._resolve_single_arm_goal_live, mv)
        diff = float(np.abs(goal_of(mv) - exported_goal).max())
        results.check(f"Alice live goal: base not tracked -> {mv.movement_id} keeps the "
                      f"exported goal, no IK",
                      ok is True and diff == 0.0 and 'base not tracked' in printed
                      and '[goal IK]' not in printed,
                      f"|goal - exported| {diff:.1e} rad")

        # * --- (b) base tracked, moved +3 cm in x, arms at home: plan the free approach
        monitor.USE_MOCAP = 1
        monitor.USE_CELL_STATE_BASE_POSE = 0
        # ! Mocap drives the base here: with FAKE_HARDWARE, Load Movement would
        # ! teleport the stub base back to the movement's authored base.
        monitor.FAKE_HARDWARE = 0
        iface.position = np.asarray(authored_pos, dtype=float) + [LIVE_BASE_SHIFT_M, 0.0, 0.0]
        iface.rotation = np.asarray(authored_rot, dtype=float)
        iface.arm_joint_pose = [np.asarray(UR5e_HOME_STATE, dtype=float).copy()]
        monitor.load_schedule_entry(hold.index)
        mv = select_movement(monitor, free_idx)
        monitor.plan_selected_movement()
        jt = mv.trajectory
        end = np.asarray(path_from_joint_trajectory(jt, names)[-1], dtype=float) if jt is not None else None
        base_err = float(np.abs(np.subtract(list(mv.start_state.robot_base_frame.point),
                                            iface.position)).max())
        d_pos = d_ang = goal_moved = np.inf
        if end is not None:
            end_state = mv.start_state.copy()
            monitor._merge_arm_values(end_state, names, end)
            flange = _fk_link_frame(monitor.cfab.planner, end_state, alice.flange_for_side(side))
            d_pos, d_ang = frame_residual(flange, mv.target_ee_frames[side])
            goal_moved = float(np.abs(goal_of(mv) - exported_goal).max())
        results.check(f"Alice live goal: base tracked +{LIVE_BASE_SHIFT_M * 100:.0f} cm -> "
                      f"{mv.movement_id} plans from the live base, its flange ends on the "
                      f"approach pose, goal re-solved",
                      jt is not None and base_err < SAME_TOL and d_pos < FLANGE_POS_TOL
                      and d_ang < FLANGE_ANG_TOL and goal_moved > GOAL_MOVED_TOL,
                      f"{len(jt.points) if jt is not None else 0} points, |base - live| {base_err:.1e} m, "
                      f"flange {d_pos * 1000:.2f} mm / {d_ang:.4f} rad, "
                      f"|goal - exported| {goal_moved:.4f} rad")
        if end is None:
            return

        # * --- (c) the gripper step and the linear approach start at the free approach's end
        steps = [i for i in range(free_idx + 1, linear_idx)
                 if movement_kind(monitor._loaded_movements[i]) in STATIONARY_KINDS] + [linear_idx]
        diffs = [float(np.abs(arms_at_start(i) - end).max()) for i in steps]
        results.check(f"Alice live goal: {[monitor._loaded_movements[i].movement_id for i in steps]} "
                      f"start where {mv.movement_id} ends",
                      len(steps) > 1 and max(diffs) < SEED_TOL,
                      f"max diffs {np.round(diffs, 9).tolist()}")

        # * --- (d) the linear approach plans from there
        mv = select_movement(monitor, linear_idx)
        monitor.plan_selected_movement()
        jt = mv.trajectory
        results.check(f"Alice live goal: {mv.movement_id} plans from the re-solved goal",
                      jt is not None, f"{len(jt.points) if jt is not None else 0} points")

        # * --- (e) base 2 m away: no collision-free IK, nothing planned
        iface.position = np.asarray(authored_pos, dtype=float) + [OUT_OF_REACH_SHIFT_M, 0.0, 0.0]
        monitor.load_schedule_entry(hold.index)
        mv = select_movement(monitor, free_idx)
        n = len(log.msgs)
        monitor.plan_selected_movement()
        warned = log.since(n, 'warn', 'not planning')
        results.check(f"Alice live goal: base +{OUT_OF_REACH_SHIFT_M:.0f} m -> {mv.movement_id} "
                      f"not planned, one warning",
                      mv.trajectory is None and len(warned) == 1,
                      warned[0] if warned else 'no warning')
    finally:
        for name in flags:
            if name in own_flags:
                setattr(monitor, name, own_flags[name])
            else:
                vars(monitor).pop(name, None)   # back to the class value
        iface.position, iface.rotation, iface.arm_joint_pose = saved_pose


def first_default_traj_time(kind: str) -> float:
    """The first default traj time of an entry kind, from ``DISPATCH_EXPECTED``.

    Movements without a default (tool and manual steps) are skipped.

    Args:
        kind (str): Entry kind, ``'J'`` or ``'R'``.

    Returns:
        float: Seconds (the J entry's free move to load, the R entry's retreat).
    """
    return next(row[4] for (k, _idx), row in sorted(DISPATCH_EXPECTED.items())
                if k == kind and row[4] is not None)


def cindy_restart_run(results: Results, problem: str, root: str, schedule) -> None:
    """Cindy's monitor started again: she continues from her progress.json state.

    A NEW monitor object goes through the start-up path (``_load_schedule_state``
    seeds the connected robot from its own belief, here the one Cindy's run
    stored when it marked entry 0 done); the stub arms start elsewhere, so the
    seeding shows. Then: Load entry sets the traj time to the entry's first
    default; her next jointing entry's movement 0 starts from the believed
    configuration (it starts live, from the seeded arms); 'Move Arms to Movement
    Start' with FAKE_HARDWARE moves the simulated arms and sends no command; and
    after Alice's hold is marked done the header's second line names where
    Alice's and Belle's poses come from. Last, 'Reset schedule to the Rhino
    export': with 'Cancel Exec' nothing changes; confirmed, progress.json and the
    saved plans move to archive/, every entry is pending and entry 0 loads its
    clean export.

    Args:
        results (Results): Where the checks go.
        problem (str): Problem folder name.
        root (str): The scratch problem folder.
        schedule (ActionSchedule): The scratch schedule.
    """
    cindy, alice, belle = (robot_by_name(ASSEMBLY_ROBOT), robot_by_name(SUPPORT_ROBOT),
                           robot_by_name(OTHER_SUPPORT_ROBOT))
    first = schedule.entry(0)
    release = schedule.entry(1)
    joint_entry = next(e for e in schedule.entries_for_robot(cindy.name)
                       if e.index > release.index and e.kind == 'J')
    hold = schedule.find_entry(HELD_BAR, 'H')
    belief = load_progress(root, schedule).belief(cindy.name)
    print(f"\n{'=' * 30} CINDY RESTART {'=' * 30}")
    base = schedule.load_action(first, prefer_sidecar=False).movements[-1].start_state.robot_base_frame
    monitor, iface, log = make_monitor(cindy, base, problem)
    add_viz_huskies(monitor, cindy)
    # The stub arms start at zeros, away from the belief, so the seeding is visible.
    iface.arm_joint_pose = [np.zeros(6) for _ in range(cindy.n_arms)]
    try:
        # * --- start-up: the simulated arms take her own belief
        n = len(log.msgs)
        monitor._load_schedule_state()
        believed = np.array([belief.configuration[name] for name in cindy.all_arm_joint_names]) \
            if belief is not None else None
        err = (float(np.abs(np.concatenate(iface.arm_joint_pose) - believed).max())
               if belief is not None else np.inf)
        started = log.since(n, 'info', f"[Schedule] {cindy.name} starts from the progress.json state")
        results.check(f"Cindy restart: her simulated arms start from her progress.json belief "
                      f"(after entry {first.index}), one log line",
                      belief is not None and belief.after_entry == first.index
                      and err < SEED_TOL and len(started) == 1,
                      f"|arms - belief| {err:.1e} rad")

        # * --- Load entry: traj time = the entry's first default (not the slider's
        # * --- start-up maximum); the release keeps its exported start (info only)
        for entry in (release, joint_entry):
            monitor.trajectory_time = monitor.trajectory_time_max
            monitor.load_schedule_entry(entry.index)
            expected = first_default_traj_time(entry.kind)
            results.check(f"Cindy restart: Load entry {entry.index} ({entry.action_id}) sets traj "
                          f"time to the entry's first default",
                          monitor.trajectory_time == expected,
                          f"{monitor.trajectory_time} s, expected {expected} s")
            mv0 = monitor._loaded_movements[0]
            start = mv0.start_state.robot_configuration
            err = max(abs(start[name] - belief.configuration[name])
                      for name in cindy.all_arm_joint_names) if belief is not None else np.inf
            print(f"  {mv0.movement_id}: starts live {0 in monitor._live_start_indices}, "
                  f"|start - belief| {err:.1e} rad")
            if entry is joint_entry:
                # * F6 b: the jointing entry's movement 0 starts where she is believed to be
                results.check(f"Cindy restart: entry {entry.index} ({entry.action_id}) "
                              f"{mv0.movement_id} starts from the believed configuration",
                              0 in monitor._live_start_indices and err < SEED_TOL,
                              f"|start - belief| {err:.1e} rad")

        # * --- 'Move Arms to Movement Start' with FAKE_HARDWARE: the simulated arms
        # * --- move to the start, nothing is sent
        idx = next(i for i, m in enumerate(monitor._loaded_movements)
                   if m.movement_id.endswith('_LM_insert'))
        mv = select_movement(monitor, idx)
        target = [np.array([mv.start_state.robot_configuration[name] for name in names])
                  for names in monitor._arm_joint_name_sets()]
        iface.arm_joint_pose = [t + 0.1 for t in target]   # inside the pi/3 guard
        monitor.FAKE_HARDWARE = True
        # The fake exec poses the drawn husky (none headless) and waits traj time per waypoint.
        monitor.huskies[0].object = SimpleNamespace(set_pose=lambda *_a, **_k: None)
        monitor.trajectory_time = 0.01
        n_calls = len(iface.calls)
        try:
            monitor.move_arms_to_movement_start()
        finally:
            monitor.FAKE_HARDWARE = False
        err = max(float(np.abs(np.asarray(a) - t).max()) for a, t in zip(iface.arm_joint_pose, target))
        results.check(f"Cindy restart: Move Arms to Movement Start ({mv.movement_id}) with "
                      f"FAKE_HARDWARE moves the simulated arms, sends no command",
                      err < SEED_TOL and len(iface.calls) == n_calls
                      and not any(iface.is_arm_executing),
                      f"|arms - start| {err:.1e} rad, {len(iface.calls) - n_calls} command(s)")

        # * --- the header's second line: where Alice's and Belle's poses come from
        monitor.load_schedule_entry(hold.index)
        monitor.mark_entry_done()
        run_tasks(monitor, iface, confirm=True)
        lines = monitor._schedule_header_text().split('\n')
        sources = obstacle_sources(monitor._progress, cindy.name,
                                   exported_action=monitor._loaded_action)
        print('  header: ' + '\n          '.join(lines))
        results.check("Cindy restart: the header's second line names Alice's and Belle's sources",
                      len(lines) == 2 and lines[1].startswith('others: ')
                      and f"{alice.name} <- assumed (entry {hold.index})" in lines[1]
                      and f"{belle.name} <- {sources[belle.obstacle_tool_name]}" in lines[1],
                      lines[-1])

        # * --- 'Reset schedule to the Rhino export': cancelled first, then confirmed
        before = folder_snapshot(root)
        monitor.reset_schedule_to_export()
        next(monitor.tasks[0])    # one tick: the reset now waits at its confirm gate
        monitor._servo_abort = True    # 'Cancel Exec'
        run_tasks(monitor, iface, confirm=False)
        results.check("Cindy restart: 'Reset schedule to the Rhino export' + 'Cancel Exec' "
                      "changes nothing", folder_snapshot(root) == before
                      and monitor._progress.is_done(hold.index))

        saved_plans = sorted(rel for rel in before if rel.endswith('.live-solved.json'))
        monitor.reset_schedule_to_export()
        run_tasks(monitor, iface, confirm=True)
        after = folder_snapshot(root)
        archived = sorted(rel for rel in after if rel.startswith(ARCHIVE_DIRNAME + os.sep))
        left = sorted(rel for rel in after
                      if rel.endswith('.live-solved.json') and rel not in archived)
        progress = monitor._progress
        parked = np.asarray(list(PARKED_BASE_FRAME.point), dtype=float)
        results.check("Cindy restart: after 'Confirm Exec' the reset moves progress.json and the "
                      "saved plans to archive/, every entry is pending, entry 0 loads its clean "
                      "export, Alice and Belle are drawn parked",
                      'progress.json' not in after and not left and len(saved_plans) > 0
                      and len(archived) == len(saved_plans) + 1
                      and all(progress.status(e.index) == STATUS_PENDING for e in schedule.entries)
                      and not progress.robots and progress.current_index == 0
                      and monitor._loaded_entry is not None and monitor._loaded_entry.index == 0
                      and '.live-solved' not in os.path.basename(monitor._current_action_path)
                      and all(np.allclose(drawn_base(monitor, spec.name), parked)
                              for spec in (alice, belle)),
                      f"{len(saved_plans)} saved plan(s) + progress.json -> "
                      f"{sorted({os.sep.join(rel.split(os.sep)[:2]) for rel in archived})}")
    finally:
        monitor.cfab.close()


# * ---------------------------------------------------------------------------
# * Main
# * ---------------------------------------------------------------------------

def main(argv: Optional[list] = None) -> int:
    """Build the scratch problem, run dispatch, carry, transfer start, operator steps, transfer free move and grounded insert checks, Cindy, Alice, Cindy's restart; print the summary.

    Args:
        argv (list | None): Command line (None = ``sys.argv[1:]``).

    Returns:
        int: 0 when every check passed, else 1.
    """
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="See the module docstring for the flow. Problem: env DESIGN_PROBLEM_NAME.")
    parser.add_argument('--keep', action='store_true',
                        help="keep the temp dir (progress.json, sidecars) for inspection")
    args = parser.parse_args(argv)

    problem = DESIGN_PROBLEM_NAME
    real_root = problem_root(problem, DESIGN_DATA_DIRECTORY)
    if load_schedule(real_root) is None:
        print(f"{problem} has no ActionSchedule.json; this smoke needs the support-robot fixture.")
        return 1
    before = folder_snapshot(real_root)

    scratch_design = tempfile.mkdtemp(prefix='husky_schedule_smoke_')
    print(f"[smoke] real problem folder (read only): {real_root}")
    print(f"[smoke] scratch copy: {os.path.join(scratch_design, problem)}")
    results = Results()
    try:
        root = make_scratch_problem(real_root, scratch_design, problem)
        point_package_at(scratch_design)
        schedule = load_schedule(root)
        for run in (dispatch_check, carry_check, transfer_start_check, operator_steps_check,
                    transfer_free_move_check, grounded_insert_check, cindy_run, alice_run,
                    cindy_restart_run):
            try:
                run(results, problem, root, schedule)
            except Exception as e:  # keep going: the summary shows where it stopped
                traceback.print_exc()
                results.check(f'{run.__name__} ran to the end', False, f"{type(e).__name__}: {e}")
        results.check('real design folder untouched', folder_snapshot(real_root) == before)
    finally:
        if args.keep:
            print(f"[smoke] kept the scratch copy: {scratch_design}")
        else:
            shutil.rmtree(scratch_design, ignore_errors=True)
            print("[smoke] removed the scratch copy (use --keep to inspect it)")

    results.print_summary()
    return 1 if results.n_failed() else 0


if __name__ == '__main__':
    sys.exit(main())
