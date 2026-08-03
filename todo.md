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

a few bugs appear:
- the joint resolution too corase, bar was slightly bended
- aftter the first vs iter, the planners somehow uses the starting state as the goal state and did a rapid go back motion without my consent. Need to safe guard for any motion with more than 10 way points or max joint delta > 5 deg.
- load bar action first put the bars all out and then make thme disppear. Then load movement will do this again. Can just do it once at bar action leve at bar action level

need to also read what the ssik backend is doing with the alt conf12 thing