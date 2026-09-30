# 2026-09-30 base_planner plugin

Run: `-p plugins:="['base_planner']"` (pulls in `cell`).

## What
Plan base path current pose -> target, inspect, commit.

Panel (folder `base_planner`), top to bottom:
- status chips: robot, tracked / not tracked, plan state (no plan / ready / stale / sent (stub))
- Robot dropdown
- TARGET: `Target x y yaw°` row + `Gizmo` checkbox (PlanarPoseInput); `Load [Robot] [Cell]`
- PATH: `[Plan] [Play] [Clear]`, `t s` slider, `Commit` button
- details below the buttons (target, path length/duration, pose at t, last message) -> nothing moves under the cursor

3D view (low clutter): small axes at target (only once a target is set), one thin floor line, one see-through ghost robot at slider time. Gizmo only while ticked.

## Files
- `ui/pose_input.py` - `PlanarPoseInput`: reusable x/y/yaw fields + transform gizmo, synced on ROS thread. Also `yaw_to_wxyz`, `yaw_from_xyzw`.
- `plugins/base_planner/path.py` - `BasePath` (times, (x,y,yaw) poses, `sample(t)`); steer = turn/drive/turn (`steer`, `steer_cost`, `steer_points`), `timed_path`, `plan_straight_line`.
- `plugins/base_planner/planner.py` - `PlanningWorld` (private DIRECT pybullet client, copy of every robot, `snapshot` from shared scene, `hit_by`), `plan_birrt` (pp `birrt` with the steer as extend fn).
- `plugins/base_planner/plugin.py` - the plugin; `send_to_onboard_follower` stub.
- `config.find_robot_serial` - robot name ("Alice") -> configured serial.
- `test/test_base_path.py` - path math. `test/test_base_planner_rrt.py` - RRT around a robot in the way.

## Rules
- Start pose from PyBullet scene (last mocap fix, else default pose) -> works offline. Commit needs `base.state.tracked`.
- Plan goes stale if base moves > 5 cm / 3° from plan start, or target != plan goal. Commit refuses stale.
- Cell target = `step.movement.start_state.robot_base_frame`, yaw from its x axis; robot switched to the step's robot.
  Assumes the cell's base frame = URDF root = mocap frame (base_footprint).
- Ghost arms = current measured joints.

## Planner (update, same day)
- pp `birrt` (RRT-Connect + restarts + shortcut smoothing) over (x, y, yaw); extend fn = turn/drive/turn steer so every edge and shortcut is diff-drive feasible; distance = metres + 0.3 * radians.
- Worker thread (job waits on future). Worker only uses raw `p` with explicit client id (pp.CLIENT is the ROS thread's). Snapshot on ROS thread before submit; one search at a time.
- Obstacles = other robots, margin 5 cm. Samples in start/goal box + 2 m. Limit 10 s, 2000 iters, 2 restarts, 100 smoothing iters.
- Start/target in collision -> reported by name. Joined-up smoothed path re-checked.
- Clear / robot change / Stop all abort the worker at its next collision check.

## Obstacles (update, same day, simplified)
- `plugins/obstacles.py`: fixed `BOXES` (placeholder tables/cabinet), added in setup to shared pybullet scene + viser. No UI.
- `base_planner` requires `obstacles`, copies `boxes` into PlanningWorld once in setup.
- `hit_by` pre-filter: skip obstacles further than the robot's reach. 0.46 -> 0.14 ms/check, identical answers.

## TODO
- `send_to_onboard_follower`: publish path to onboard follower via a BaseInterface method (e.g. nav_msgs/Path in world frame).
- Replace placeholder `obstacles.BOXES` with the measured lab furniture.
- Cell structure as obstacles (not in PyBullet yet).
- Maybe "follow cell" mode (retarget automatically when the cell step changes).
