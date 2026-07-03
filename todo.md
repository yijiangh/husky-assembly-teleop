# For Su

# Bar holding accuracy test

1. [] load goal bar pose from BarAction, load bar action base pose into goal so we can teleop robot over
2. [] live recompute IK from grasp info (needs more codes) and set it as a goal, then write it into start_state?
3. [] use free motion plan to go straight to the goal conf
4. human mount bars
5. read mocap-pair jig data and trigger a compute for fitting a line, and then compare to the goal bar pose from BarAction
    - does BarAction carries bar dimension data?

ToDo:
1. Reduced Vertical Bar Install to Air
 - try with Cindy old calibration and do a bar reaching in workspace test
 - have a strange 140 mm offset. needs debug.

- [] make repo consistent with latest rs_data_structure and tamp backend
- [] load BarAction base pose as goal pose (do we need another shadow robot?) for shadow alignment

3. Measure In existing Structure Workflow
- Assembly Robot test
- Support robot

# Robot-centric movement test

use the double kissing rig to do all movement tests:
    - [x] do Rhino design of joints on the dk jig
    - [x] joint installation on three bars
    - [x] code two LMs, comliant controller integration, rs485 tool intergation
    - [x] switching of movements can be just using a slider (existing)
    - [] hotfix pb acm during the day

    - M0: FM to load bar
    - M1: CDFM to approach
    - M2: CDLM to assembly target
    - M3: LM retreat
    - M4: FM to a fixed home pose (do we need? if so, need a scene-conditioned sampler)

    At night (05-16)
    - need to proof check the ACM in BarAction export, so we can just load them into the planner
    - do proper collision body and acm sync between cfab and pb

4. [x] calibrate Alice and Cindy

# Leftover Tasks before FoC
1. [ToReview] double kissing on new tools (integrate into our monitor) - this will force me to 
	1. [x] integrate the new rs485 control into a ros node
	2. an online LM replanner

2. [ToReview] integrate the dual-arm constrianed planner into monitor, use your new tool model, and see tracking performance of a tracking controller.

[WIP]
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
