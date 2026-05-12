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

Questions:"
- what does the max_attempt do in the plan constrained

# switch to single arm compliant controller
One thing i am not so sure about is that we decided that we always maintain assembly-robot centric in robot cell state, but if we are saving a state for the holding robot, we need to tell monitor that it should load robot state from the support robot saved as tool in the cell state.
this info needs to be saved by a json that contains a cell state

# Misc
- add the following ros2 pkgs to required installed packages
    - control_msgs
    - crl_control_msgs
    - ur_msgs

- get rid of tracik if not used