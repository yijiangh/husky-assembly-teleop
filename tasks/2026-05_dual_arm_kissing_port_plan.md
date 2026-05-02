# Plan: Port c81e373 Dual-Arm Kissing/Insertion Test into `yh/compliant_controlelr_dualarm`

> Snapshot of the original integration plan written in plan mode on 2026-05-02,
> before implementation began. Source plan file:
> `~/.claude/plans/lucky-dazzling-emerson.md`.

## Context

- Goal: get `Conduct Kissing Experiment` end-to-end working on the current branch so we can re-run the double-kissing experiment with Victor's tools. Eventual target = v3 tools, but **first cut assumes v1** (= same geometry as c81e373).
- Source = commit `c81e373` (on `origin/jg/dev`). This is Jakob's snapshot but jg/dev tip has 28 commits of unrelated drift (data collection, mocap pkg WIP, etc.). The user wants the kissing+compliance bits, not the new ROS2 mocap pkg, not the data dirs.
- HEAD already has the constrained dual-arm planner (per `tasks/cc_lessons.md`) and the load-state / load-trajectory / plan-to-goal / move-to-goal buttons. Gap = the **compliance controller infrastructure** + **kissing experiment driver** + **two new button sections**.
- Strategy chosen: **hand-port** (no cherry-pick, no merge of jg/dev). NatNetClient stays. Keep current `FAKE_HARDWARE` flag wiring. Add a new `DUAL_ARM_KISSING` mode flag.

## Files to modify

| File | Change kind | Notes |
|---|---|---|
| `husky_assembly_teleop/__init__.py` | small add | (no changes needed; flag lives on monitor class) |
| `husky_assembly_teleop/husky_robot.py` | additive port | compliance/cartesian/IO/wrench infra + 4 callbacks + 5 methods + 4 state slots |
| `husky_assembly_teleop/husky_world.py` | additive port | 8 functions + 5 module constants |
| `husky_assembly_teleop/husky_monitor.py` | additive port | 1 mode flag + 2 button sections (KISSING + CONTROLLERS) |
| `tasks/cc_lessons.md` | append | end-of-plan log per CLAUDE.md |

No deletions. No refactors of existing HEAD code. All ports are additive — existing behavior unchanged when `DUAL_ARM_KISSING == 0`.

## Existing helpers to reuse (do NOT re-create)

| Symbol | File:line | Use |
|---|---|---|
| `Husky.dual_arm` | `common.py:448` | branch in switch-controller-both lambdas |
| `HuskyRobotInterface.dual_arm` | `husky_robot.py:77` | gates dual-arm subscriber/publisher creation |
| `HuskyRobotInterface.send_dual_arm_cmd` | `husky_robot.py:266` | already present, called by `kissing_experiment` |
| `HuskyRobotInterface.send_arm_cmd` | `husky_robot.py:314` | called by `move_left_linear_z` |
| `HuskyRobotInterface.toggle_screw` / `setio_clients` | `husky_robot.py:418`, `:176` | reuse plumbing for new `set_screw(state, index)` |
| `monitor.load_board_validation_state` | `husky_monitor.py:855` | wired button — "load state" |
| `monitor.load_joint_trajectory` | `husky_monitor.py:1167` | wired button — "load trajectory" |
| `world.plan_both_arms_to_goal(monitor, use_composite=True)` | `husky_world.py:1743` | wired button — "plan to goal" |
| `world.execute_arm_trajectory_both` | `husky_world.py:1373` | wired button — "move to goal" |
| `world.set_arm_trajectory` (on monitor) | `husky_monitor.py:237` | called inside ported helpers |
| `planning.IK_SOLVER_DUAL` | `husky_planning.py` | used by `generate_insertion_motion_bar` / `generate_reset_trajectory_bar` |
| `utils.get_arm_ik_for_grasp_bar` | `utils.py` | already imported in HEAD `husky_world.py:20` |
| `DATA_DIRECTORY` | `__init__.py` | basis for new `KISSING_DATA_DIR` |

## Detailed port spec

### A. `husky_robot.py` — compliance/cartesian/IO/wrench infra

Source lines refer to `git show c81e373:husky_assembly_teleop/husky_robot.py`.

**A1. New imports** (after current import block):
```python
from control_msgs.msg._dynamic_joint_state import DynamicJointState
from control_msgs.msg._interface_value import InterfaceValue
from geometry_msgs.msg import PoseStamped, WrenchStamped
from ur_msgs.msg._io_states import IOStates
from std_srvs.srv._trigger import Trigger
from controller_manager_msgs.srv._switch_controller import SwitchController
```

**A2. New class state slots** on `HuskyRobotInterface` (next to existing `arm_joint_pose`):
```python
arm_tcp_pose = [pp.Pose()]
arm_ft_sensor = [[0]*6]
io_states = [[False]*18]
active_controller = [""]
```
And in `__init__`, when extending arrays for dual_arm, append the second slot for each (mirror existing `arm_joint_pose.append(...)` pattern).

**A3. New subscribers** in `__init__`, after existing `sub_arms` block (c81e373 lines 130–187):
- `sub_dynamic_arm` → `DynamicJointState` topic `<name>/{left_,right_,}ur5e/rate_limiter/dynamic_joint_states` → `dynamic_arm_callback(index, msg)`
- `sub_io_states` → `IOStates` topic `.../rate_limiter/io_and_status_controller/io_states` → `io_state_callback(index, msg)`
- `sub_ft_sensor` → `WrenchStamped` topic `.../rate_limiter/ft_sensor_wrench` → `ft_sensor_callback(index, msg)`

Single-arm vs dual-arm topic naming follows the existing `sub_arms` pattern (left_ur5e/right_ur5e vs ur5e).

**A4. New publishers** in `__init__`, after existing `pub_cmd_arm` block:
- `pub_cmd_arm_cartesian` → `PoseStamped` on `<name>/.../target_frame` (per arm)
- `pub_cmd_arm_cartesian_force` → `WrenchStamped` on `<name>/.../target_wrench`

**A5. New service clients** in `__init__`, after existing `setio_clients` block:
- `zero_ft_sensor_client` → `Trigger` on `.../io_and_status_controller/zero_ftsensor`
- `controller_change_service_client` → `SwitchController` on `.../controller_manager/switch_controller`

Use `wait_for_service(timeout_sec=2.5)` like other clients.

**A6. New callback methods** (verbatim from c81e373:388–428):
- `dynamic_arm_callback(self, index, msg: DynamicJointState)` — extracts `tcp_pose` interface value, applies `correction_transform = pp.Pose(pp.Point(), pp.Euler(0, 0, np.deg2rad(-180 if self.dual_arm else -90)))`, writes `self.arm_tcp_pose[index]`
- `io_state_callback(self, index, msg: IOStates)` — writes `self.io_states[index][pin] = state` for 18 pins
- `ft_sensor_callback(self, index, msg: WrenchStamped)` — writes 6-vec to `self.arm_ft_sensor[index]`

**A7. New methods** (verbatim from c81e373):
- `switch_controller(self, from_ctrl, to_ctrl, arm_index=0)` (c81e373:316–337) — async SwitchController call; on success sets `self.active_controller[arm_index] = to_ctrl`
- `zero_ft_sensor(self, index=0)` (c81e373:430–432) — async Trigger
- `set_screw(self, state, index=0)` (c81e373:657–680) — like existing `toggle_screw` but takes explicit bool. Refactor `toggle_screw` to call `self.set_screw(not self.screw_states[index], index)` (c81e373:684–687 — already does this).
- `send_arm_cmd_cartesian(self, pose_arm_local, index=0)` (c81e373:469–488) — publishes PoseStamped with start-pose safety check
- `send_arm_cmd_cartesian_force(self, force_arm_local, index=0)` (c81e373:489–498) — publishes WrenchStamped

### B. `husky_world.py` — kissing experiment driver

Source = `git show c81e373:husky_assembly_teleop/husky_world.py`.

**B1. Module constants** (top of file, near existing data dir constants):
```python
KISSING_DATA_DIR = os.path.join(DATA_DIR, "kissing_experiment_data")
Z_MOVE_TO_INSERT = 0.035
CARTESIAN_SPEEDUP = 5
TIME_PER_ROTATION = 14
PROBE_END_WAIT_TIME = 1
USE_CARTESIAN_CONTROLLER = True
```
(Replace `DATA_FOLDER = '/home/jakobgenhart/...'` with `KISSING_DATA_DIR`.)

**B2. New functions** (verbatim from c81e373, with one path swap inside `kissing_experiment`):

| Function | c81e373 lines | Notes |
|---|---|---|
| `compute_bar_pose_from_EE_poses(left, right)` | 1621–1635 | pure helper |
| `draw_tcp_pose(monitor)` | 1610–1620 | viz |
| `execute_linear_cartesian_move(robot, hi, start_time, cartesian_trajectory, index)` | 1638–1665 | publishes cartesian cmd |
| `move_left_linear_z(monitor, length, speed)` | 1824–1837 | uses `set_screw` + `send_arm_cmd` + `generate_insertion_motion_bar` |
| `generate_insertion_motion_bar(monitor, depth, speed, cartesian_speedup=1, neutral_start_pose=None)` | 1839–1886 | returns `(arm_trajectories, cartesian_trajectories)` |
| `generate_reset_trajectory_bar(monitor, speed, goal_pose)` | 1888–1925 | uses `get_arm_ik_for_grasp_bar` |
| `kissing_probe_once(monitor, neutral_bar_pose, starting_bar_pose, offset, file_location, name)` | 1668–1822 | the experiment generator; data dump path = `f"{file_location}/{name}.json"` |
| `kissing_experiment(monitor)` | 1565–1608 | top generator. Pass `KISSING_DATA_DIR` (after `os.makedirs(KISSING_DATA_DIR, exist_ok=True)`) instead of c81e373's hardcoded `DATA_FOLDER` |

All called by `monitor.tasks.append(world.kissing_experiment(self))` from monitor button.

### C. `husky_monitor.py` — mode flag + two button sections

**C1. New mode flag** next to existing flags (line ~60):
```python
DUAL_ARM_KISSING = 0  # set 1 to enable kissing experiment + compliance controller buttons
```

**C2. New button section** after the `Plan Both Arms to Goal (composite)` button (HEAD ~line 1442) and before the `BOARD_VALIDATION` block (~1449):

```python
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
```

**C3. No other monitor changes.** The user's six requested buttons are realized by:

| User wants | Wired by |
|---|---|
| load state | existing `Load Robot Cell State` (HEAD 1466) |
| load trajectory | existing `Load Joint Trajectory` (HEAD 1478) |
| plan to goal | existing `Plan Both Arms to Goal (composite)` (HEAD 1441) |
| move to goal | existing `Exec Both Arm Trajs` (HEAD 1434) |
| ensure joint controller | new `Switch to Joint (BOTH)` |
| start experiment | new `Conduct Kissing Experiment` |

## Verification plan

After implementer finishes, run from `/home/yijiangh/Code/ros2_ws/venv`:

```bash
source /home/yijiangh/Code/ros2_ws/venv/bin/activate
cd /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop

# 1. Static syntax
python -m py_compile husky_assembly_teleop/husky_robot.py \
                     husky_assembly_teleop/husky_world.py \
                     husky_assembly_teleop/husky_monitor.py

# 2. Import smoke (catches missing symbols, bad refs, ROS msg import errors)
python -c "from husky_assembly_teleop import husky_robot, husky_world, husky_monitor; \
           print('ok', \
                 hasattr(husky_robot.HuskyRobotInterface, 'switch_controller'), \
                 hasattr(husky_robot.HuskyRobotInterface, 'zero_ft_sensor'), \
                 hasattr(husky_robot.HuskyRobotInterface, 'set_screw'), \
                 hasattr(husky_world, 'kissing_experiment'), \
                 hasattr(husky_world, 'move_left_linear_z'), \
                 hasattr(husky_world, 'draw_tcp_pose'))"
```

Expected: prints `ok True True True True True True`. Any False = symbol not exported / typo.

Live ROS2 / hardware test = manual, on the rig:
1. `DUAL_ARM_KISSING = 1`, `FAKE_HARDWARE = 0`, `BOARD_VALIDATION = 1`, `CALIBRATION = 0`.
2. Launch monitor; Load Robot Cell State → Load Joint Trajectory → Plan Both Arms (composite) → Exec Both Arm Trajs (move to goal start of kissing).
3. Switch to Joint (BOTH) — verify `active_controller` reports scaled_joint on both arms.
4. Zero Force Sensor (BOTH).
5. Conduct Kissing Experiment — observe insertion + retreat for 3 offsets, JSON dumped to `data/kissing_experiment_data/`.

## After-integration log entry

Append to `tasks/cc_lessons.md` a section "## Dual-arm kissing experiment port from c81e373 — 2026-05" recording:
- Hand-port additive strategy (no cherry-pick).
- New `DUAL_ARM_KISSING` flag pattern.
- Compliance/cartesian/IO/wrench infra all on `HuskyRobotInterface`; topic naming mirrors existing `sub_arms` left/right/single-arm split.
- TCP-pose `correction_transform` is hardcoded `np.deg2rad(-180 if dual_arm else -90)` — assumption baked into v1 tool mounting orientation.

## v3 tool follow-up (for after this lands)

When you transition to Victor's v3 tools, the spots to revisit:
1. **Tool URDF** — wherever `validation_tool_pair` (or its replacement) loads meshes in `common.py:create_end_effector` — point to v3 mesh.
2. **TCP correction transform** in `husky_robot.dynamic_arm_callback` — the `np.deg2rad(-180 if self.dual_arm else -90)` Z-rotation encodes v1 mounting yaw. v3 likely has different mount → expose it as a per-arm constant or read from URDF.
3. **`Z_MOVE_TO_INSERT = 0.035`** in `husky_world.py` — depth for insertion. v3 tip geometry may need a different value.
4. **`compute_bar_pose_from_EE_poses`** — assumes the bar midpoint is the geometric midpoint of the two tool0 poses (no offset). If v3 grippers grasp the bar at non-symmetric offsets, this needs the per-side offset from the URDF.
5. **`kissing_experiment` offsets sweep** — `[0.000 + 0.005 * i, 0.000, 0.00, 0.00] for i in 0..2` is hardcoded. Pull into a config knob.

These five are explicit in the integrated code (constants and functions named above) — easy to grep when ready.

## Critical files to modify (paths)

- `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_robot.py`
- `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_world.py`
- `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py`
- `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/tasks/cc_lessons.md`
