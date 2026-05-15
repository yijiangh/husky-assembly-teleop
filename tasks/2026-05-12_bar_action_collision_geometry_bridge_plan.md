# Fix BarAction Collision Geometry Bridge

## Summary

Fix the BarAction pp bridge so it preserves RobotCell collision semantics closely enough for the existing pp planners: stationary rigid bodies remain obstacles, RobotCell-attached rigid bodies become moving pp attachments, and future assembly bars named `bar_B*` are filtered correctly.

## Key Changes

- In the BarAction bridge, classify `RigidBodyState`s by state:
  - `active_bar_name`: active manipulated body, handled separately.
  - `attached_to_link` / `attached_to_tool`: moving rigid bodies, excluded from `monitor.static_obstacles`.
  - hidden bodies: excluded from pp collision planning.
  - all remaining rigid bodies: static obstacles.
- Build a BarAction collision context on the monitor:
  - `bar_action_attached_body_attachments`: pp `Attachment`s for attached rigid bodies other than the active bar.
  - `bar_action_attached_body_touch_pairs`: disabled collision pairs from each attached rigid body’s `touch_links`.
  - `bar_action_static_obstacles`: stationary obstacle ids only.
- Keep the two ghost EE attachments only as a fallback interface shim if needed by legacy `ee_list`, but do not use them as the collision representation when real attached rigid bodies exist.

## Planner Integration

- Extend constrained collision setup to accept extra moving attachments and disabled pairs:
  - `get_joint_collision_fn(..., extra_attachments=None, extra_disabled_collisions=None)`.
  - Include active bar attachment plus the RobotCell attached-body attachments.
  - Preserve existing active-bar grasp-mask disabled pairs.
- Extend free staging planning to accept all moving attachments, not just exactly two ghost attachments:
  - Change `plan_free_dual_arm` / `plan_transit_motion` to pass a flat attachment list into `pp.get_collision_fn`.
  - Keep per-arm wrist-to-child disabled pairs for each attachment using its parent link side where possible.
  - For staging, exclude the active bar from both obstacles and attachments, because the staging move occurs before the active bar is held.
- Fix assembly filtering in `husky_world.py`:
  - Replace `^b\d+(_0|_joint_\d+)$` with a predicate covering current RobotCell names, including `bar_B\d+` and `joint_J...`.
  - Only filter future/static assembly elements, not structural fixtures or currently attached tool/joint bodies.

## Tests And Validation

- Update `scripts/inspect_bar_action_collision_geometry.py` so it asserts/report-checks:
  - attached RobotCell rigid bodies are not in pp static obstacles.
  - attached RobotCell rigid bodies appear in pp attachment context.
  - `bar_B1`/`bar_B2`/etc. are filtered by the assembly predicate.
- Run the inspector on `B6.json --movement M1` and confirm:
  - `AssemblyLeftArmToolBody`, `AssemblyRightArmToolBody`, `joint_J3-6_male`, `joint_J4-6_male` are moving attachments, not static obstacles.
  - active bar remains separate.
- Run the headless harness with:
  - `--bar-action B6.json --movement M1 --max-attempts 1 --max-time 1`
  - `--show-collision-setup`
  - optional `--validate` after a successful plan.
- Add focused regression tests if practical as script-level assertions rather than ROS tests, since this path depends on the design-study data and PyBullet.

## Assumptions

- We keep the existing pp planner architecture for now; no full replacement with cfab `check_collision`.
- RobotCell `RigidBodyState.touch_links` and `touch_bodies` are the source of truth for allowed contacts when translating into pp disabled pairs.
- `bar_B*` bodies represent assembly bars that should be excluded from constrained static obstacles in the same spirit as the old `b<N>_*` filter.
