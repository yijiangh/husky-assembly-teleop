# For Su

# Bar holding accuracy test

ToDo:
1. Reduced Vertical Bar Install to Air
 - try with Cindy old calibration and do a bar reaching in workspace test

3. Measure In existing Structure Workflow
- Assembly Robot test
- Support robot

# Leftover Tasks before FoC
1. [ToReview] double kissing on new tools (integrate into our monitor) - this will force me to 
	1. [x] integrate the new rs485 control into a ros node

3. [] a bar goal reaching test to check the accuracy of mocap2urdf calibration (with our mocap rigs on a bar)
    - test dual-arm transport, making it faster, since no need to get the bar off
    - should be fast to do on site.

4. [] allow users to switch between controlling the dual=arm and the single=arm. and also use the single-arm's compliant controller.
    - should be fast to do on site. but needs to decide how to distnguish which robot a cell_state belongs to


Questions:
- How are the cfab session and pp synchronize collision geometry to be checked.
    - _bridge_cfab_to_pp_for_bar_action
    - how did the collision bodies from one session get copied into the other?

# switch to single arm compliant controller
One thing i am not so sure about is that we decided that we always maintain assembly-robot centric in robot cell state, but if we are saving a state for the holding robot, we need to tell monitor that it should load robot state from the support robot saved as tool in the cell state.
this info needs to be saved by a json that contains a cell state

# Misc
- add the following ros2 pkgs to required installed packages
    - control_msgs
    - crl_control_msgs
    - ur_msgs

- get rid of tracik if not used

# Active

- [] Something is very off when reloading a new bar aciton, the old collision scene of the last bar action is not cleared up
- [] the environment/WalkableGround is not a cfab rigid body atm, but should be

- [] add two more options for the start bar pose sampler for M1, veritcle, and piggyback, horizontal in additional to the current up/down and rotation sampling

9 horizontal (B30)
10 vertical (B33)
13 skewed (B36)

a few bugs appear:
- [x] the joint resolution too corase, bar was slightly bended
- [x] aftter the first vs iter, the planners somehow uses the starting state as the goal state and 
    - [x] did a rapid go back motion without my consent. Need to safe guard for any motion with more than 10 way points or max joint delta > 5 deg.
- in the vs, the joints that is installed on the grasped bar get hidden too

- load bar action first put the bars all out and then make thme disppear. Then load movement will do this again. Can just do it once at bar action leve at bar action level, and it should lock the render to acc


Still some problems with the second vs iteration:
[INFO] [1785839465.393885698] [husky_monitor]: ### SERVOING iteration 2/8
[IK Live Base] start EE frames from FK at start_state.
[goal IK] attempt 1/5: GOAL COLLISION: CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION
[goal IK] attempt 2/5: GOAL COLLISION: CC.1 between robot link 'base_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION
[goal IK] attempt 3/5: GOAL COLLISION: CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION
[goal IK] attempt 4/5: GOAL COLLISION: CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION
[goal IK] attempt 5/5: GOAL COLLISION: CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION
[goal IK] DIAGNOSTIC: IK is reachable WITHOUT collision check but rejected WITH collision check. Last with-CC error: GOAL COLLISION: CC.1 between robot link 'dual_arm_bulkhead_link' and robot link 'left_ur_arm_upper_arm_link' - COLLISION. Likely missing touch-link on the held bar or stale ACM. Inspect monitor.cfab.planner state, the bar's rigid_body_states[...].touch_links, and the start_state passed to IK.

Figure out what this vague msg means:
```
[goal IK] FALLBACK: accepting alt_seed_conf12 verbatim as goal (tool0 world-frame error ~ base_offset). Composite free plan will land the arms on this bar-holding pose.
```

```
ang=0.011 deg | R pos=49.55 mm ang=3.370 deg
[WARN] [1785837365.402086928] [husky_monitor]: [Replan verify] tool0 endpoint MISMATCH: max pos=49.55 mm (tol 5.0 mm), max ang=3.370 deg (tol 1.0 deg). The composite plan likely landed on the IK fallback (alt_seed_conf12 verbatim), which does NOT compensate the arm conf for the base offset -- world-frame tool0 error scales with the base offset. Any downstream linear motion that assumes the authored EE targets should be re-planned.
[WARN] [1785837379.479490164] [husky_monitor]: [transfer validation] 'B36_M3_LM_retreat': joint continuity FAIL (max step 8.42 deg / thresh 1.0); bar-hold OK (max drift 0.00 mm, 0.000 deg)
```

need to also read what the ssik backend is doing with the alt conf12 thing

- mocap offset box should be integrated in dpg
[mocap offset] primary backend is DPG; skipping private offset window.