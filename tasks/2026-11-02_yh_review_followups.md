# YH port review: follow-ups

What stays open from the review of Yijiang's port (merge 65053b61, report "YH Port Review"), after the fixes of
29 Oct – 2 Nov. Decisions by JG on 2 Nov.

## Decided, nothing to do

- Rhino obstacles out of the collision checks by default (`OBSTACLES_COLLIDE = False`, process plugin): intended.
  The exported obstacles are wrong (the cabinet stands 0.38 m off); the measured ones (`obstacles` plugin) count.
- The insert's goal comes from the left target only (tamp's linear planner): leave it.
- A bar-held planning try can run ~10 min (`HELD` has no `max_attempts`) and is not cancelled between attempts:
  leave YH's planning UI as it is.

## To do

- Base planner margin: 2 cm, and the same number is the commit's stop distance while the robot rolls ~10 cm after a
  trip (`plugins/base_planner/world.py`, `planner_plugin.py`). Known; see the speed-aware clearance.
- tamp's free search uses compas_fab's full `check_collision` (~56 ms per configuration) where the core's
  `search_check` takes ~1 ms (`external/husky_assembly_tamp/.../motion_planner/api.py`). Likely why free searches
  need 120 s.
- Dense paths are never checked between waypoints: the bar-held transfer, the insert, the retreat and the support line
  (`single_arm.plan_linear`). `collision.swept_check` (0.01 rad, `search_check`) would cost ~1 ms per sample; see the
  estimate in the review answer of 2 Nov before switching it on.
- All planners on the live scene: `scene_for` rebuilds the process at the step's start, keeps only `obstacles/` from
  the snapshot (drops `tracked/*`) and stands unmeasured robots at their design pose. Blockers to remove first: the
  process plugin's drawn state must equal the step's start (the selection can differ), a tracked bar duplicates its
  process body, and the per-step edits (`hide_held`, `ignore_built`, ground halves on the floor) must
  become overlays on the live scene.

- Exec's arm motions under a runtime clearance (1 cm, as the planners' commits) once the live scene carries the
  planner's allowances: a tool on its bar at a grasp's end, a ground bar's feet on the floor, no held bar in the empty
  tools of a workaround transfer; checked on the purged process before switching it on. A trip must then be recorded
  as a clearance trip, not as Stop all.
- The relay's `base_offset`: shown in the monitor (a chip per robot from `/mocap_relay/calibrations`) and recorded in
  a run; today only the relay's log says it is on, and a relay restart drops it.

## Decided 2 Nov (final review)

- A cancelled compliant part (Cancel, Stop all, a plugin closing) holds its arms and sends no controller switch: the
  operator switches them back in robot_control. Closing a plugin waits for no robot.
- assembly_run's second click keeps YH's no time limit; ssik stays optional.

## To investigate

- Handoff: with the anchor more than 10 mm / 2° from the URDF's tool0 it closes the Robotiq with the arm stiff, after
  the operator confirmed a compliant close (`shared/execution/handoff.py`); it also runs without a freeze or a watch on
  the robot holding the bar.
- Spiral: phase A holds compliance with no time limit (`Segment(..., math.inf)`, `shared/execution/spiral.py`) while
  the joint screws keep driving.

## Future work

- mocap_bars runs the old monitor's matcher (`world/marker_match.py`, ~20 ms) on the main thread; JG's label-free fit
  was deleted from core-port and is parked on `jg/mocap-marker-cloud`.

## Duplicates for a later cleanup

- `shared/assembly_planning/arm.plan_arm`: a slimmer fork of `plugins/arm_planner/planner.plan_arm` (one search, no
  best-of-3, no corner rounding, samples the whole URDF ∩ UR range instead of `SAMPLE_SLACK`).
- `shared/execution/retime.py` beside `robot_interface/arm.joint_path_move` / `_cosine_along`: two cosine retimers.
- `plugins/assembly_steps/step_panel_base.PlannerPlugin` beside `shared/planner/PlannerService` / `PlannerPanel`.
- `joint_limits` twice: from the URDF (`shared/assembly_planning/joints.py`) and from the mirror
  (`plugins/arm_planner/planner.py`).
- `world_frame_fk` (compas_fab FK patch) applied by hand at six lend sites; belongs in the core's
  `CompasFabMirror.lend`.
- A second `MirrorPool` in motion_planner: every robot model loaded again (up to 8 worlds, ~0.8 GB).
- `shared/assembly_planning/ik.py` repeats the tool0 FK of `world/tcp.tool0_in_base_link`.
- `single_arm.interpolate_poses` could use `robot_interface/arm.pose_between`.
- `movement_exec/panel.py` computes |F| itself (`shared/execution/force_sensors.force_magnitude`).
- `shared/execution/servo_record.py` types the servo plots' colours; they belong in `colors.py` when YH's plugins are
  converted.
