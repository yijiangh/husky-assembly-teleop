# Constrained Button Manual Staging Plan

## Goal
- `Plan & Stage Constrained` should only plan the constrained dual-arm motion.
- The constrained motion start configuration becomes the monitor goal target.
- User manually plans the free approach with existing buttons:
  - `Plan Both Arms to Goal (composite)`
  - `Plan S.Arm to conf target`
- `Display Traj (0=Free,1=Constrained)` remains the selector for visualization/execution.

## Implementation
- In `husky_world.py`, keep constrained start derivation and constrained planning.
- Remove automatic `plan_free_dual_arm()` staging from the constrained button path.
- Store `constrained_start_conf`, `constrained_goal_conf`, and `constrained_trajectory`.
- Clear stale `staging_free_trajectory` after each new constrained plan.
- Set `goal_arm_pose` to constrained start so manual free planning targets that pose.
- In `husky_monitor.py`, wrap existing single-arm and both-arm plan buttons.
- After a manual plan, if the current goal still matches `constrained_start_conf`, cache the result into `staging_free_trajectory` as display slot 0.
- Wrap constrained planning in `pp.LockRenderer()` so PyBullet rendering is paused during the expensive search.
- For manual staging plans, explicitly exclude the active held bar from obstacles and pass a larger BiRRT budget through the free planner wrapper.

## Acceptance
- Clicking `Plan & Stage Constrained` produces no automatic free staging plan.
- After constrained planning, monitor goal is the constrained start.
- Manual free planning populates display slot 0.
- Display slot 1 remains the constrained trajectory.
- Existing execute buttons continue using `planned_arm_trajectory`, refreshed by the display slider.

## Test
- Build package with the ROS workspace venv.
- Load BarAction M1.
- Click `Plan & Stage Constrained`.
- Verify constrained trajectory displays and goal target is the constrained start.
- Click `Plan Both Arms to Goal (composite)` and verify it caches as Display Traj 0.
- If composite planning fails, check whether `initial and end conf not valid` appears; without that line, endpoints were valid and search failed.
- Switch display to 1 and verify constrained trajectory is selected for execution.
- Observe that the GUI stops redrawing during constrained planning and resumes afterward.
