# Merge origin/core-port 732e985 into yh/process_replan_execution

Branch `yh/process_replan_execution`, 29 Oct 2026. Jakob's 12 commits of 23–28 Oct come in; our four themes of
uncommitted work were committed first (b6e1312 search tree, 888fab5 one blue ghost + Plan window, 67692bf
execution and panels, 4982fc9 real lab).

## What comes in

1. Unknown robots, one policy (`tasks/2026-10-23_unknown_robots.md`): lost mocap = untracked base, stale joint
   states = unmeasured arm, set in `take_snapshot`; `world/stand_ins.py` stands another robot not measured where
   last seen, else at its default pose with arms stowed; every planner and clearance uses it and warns. The run's
   recorded robots (beliefs) no longer plan.
2. Commit permission: `CommitConfirm`, a second press within 2 s while other robots are unknown; the
   `unknown_robots_chip` on every planner; `ctx.clearance(..., unknown_robots=...)`.
3. Arm clearance 1 cm: joint jogs while the Clearance box is ticked, every arm planner commit.
4. See-through robot parts: real robots as one-colour meshes, a part the checks only assume at `GUESSED_OPACITY`.
5. Health banner per robot, the info level (blue sim chips), collision shapes in the collisions plugin.
6. Base planner follower model from crl-husky's `base_model`.

## Decisions (user, 29 Oct)

- Beliefs go: stand-ins everywhere, `servo_route` included. The run record keeps `Progress.robots` for the report.
- `ui/visualization.py`: Jakob's drawing; our Visual diagnosis fade (`set_robot_opacity`) is re-done on his parts,
  the see-through twin goes.
- The Assembly tab's Exec rests blue like Commit and turns amber while it waits for its second press.
- The tool0 jog goes through the same Clearance box and guard as the joint jog.
- The SIM chip: Jakob's blue info level, not our green.
- Merge in this checkout; no push (the user pushes).

## Resolution by file

See the plan: `~/.claude/plans/i-want-to-merge-precious-dusk.md` (2a beliefs, 2b visualization, 2c Exec, 2d jog,
2e mechanical). Status is kept below.

## Status

- [x] merge started, conflicts resolved (25 files)
- [x] pytest (default set) green: 1477 passed
- [x] pytest -m "" green (two slow tests fixed to the new defaults: margin 2 cm, Servo by bar on)
- [ ] fake-lab check

## Found on the way

- The second press never reached Exec on core-port either: `AssemblyStepsPlugin._commit` overrides the base's
  `_commit` without calling it. Now `_asks_second_press` is called there, last before the send.
- crl-husky `jg/ros2` ceba934 gives Cindy her `base_model`; the README pin moved to it.
- `HUSKY_DRIVE_ROOT` unset skips 189 tests (the process ones): set it for a real run.
