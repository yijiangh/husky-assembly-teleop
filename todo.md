# Tasks before FoC
1. double kissing on new tools (integrate into our monitor) - this will force me to 
	1. integrate the new rs485 control into a ros node
	2. an online LM replanner
2. integrate the dual-arm constrianed planner into monitor, use your new tool model, and see tracking performance of a tracking controller.
3. a bar goal reaching test to check the accuracy of mocap2urdf calibration (with our mocap rigs on a bar)

# dual-arm constrained planner

# Misc
- add the following ros2 pkgs to required installed packages
    - control_msgs
    - crl_control_msgs
    - ur_msgs

- get rid of tracik if not used