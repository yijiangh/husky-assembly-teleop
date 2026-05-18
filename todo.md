# Tasks before FoC
1. [ToReview] double kissing on new tools (integrate into our monitor) - this will force me to 
	1. [ToReview] integrate the new rs485 control into a ros node
	2. an online LM replanner

2. [ToReview] integrate the dual-arm constrianed planner into monitor, use your new tool model, and see tracking performance of a tracking controller.

3. [] a bar goal reaching test to check the accuracy of mocap2urdf calibration (with our mocap rigs on a bar)
    - test dual-arm transport, making it faster, since no need to get the bar off
    - should be fast to do on site.

4. [] allow users to switch between controlling the dual=arm and the single=arm. and also use the single-arm's compliant controller.
    - should be fast to do on site. but needs to decide how to distnguish which robot a cell_state belongs to

# dual-arm constrained planner

ToDo:
1. Reduced Vertical Bar Install to Air
 - try with Cindy old calibration and do a bar reaching in workspace test
 have a strange 140 mm offset. needs debug.

2. Two hand install on Robot Jig (Tomorrow)

A text box to enter global offset to apply on the mocap robot base pose

3. Measure In existing Structure Workflow (Tomorrow)
- Assembly Robot test
- Support robot

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

4. calibrate Alice and Cindy

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

# Tool

for right arm scaffolding tool, a problem:
```
[scaffolding_tool_driver-26] [ERROR] [1778509072.881889439] [a200_0806.right_gripper.scaffolding_tool_driver]: Unexpected response to command 'PING':
[scaffolding_tool_driver-26] [ERROR] [1778509073.883459743] [a200_0806.right_gripper.scaffolding_tool_driver]: Unexpected response to command 'VERSION':
[scaffolding_tool_driver-26] [INFO] [1778509073.883941630] [a200_0806.right_gripper.scaffolding_tool_driver]:
[scaffolding_tool_driver-26] =======================================================================
[scaffolding_tool_driver-26]
[scaffolding_tool_driver-26]   Failed to connect to scaffolding tool
[scaffolding_tool_driver-26]
[scaffolding_tool_driver-26] =======================================================================
```