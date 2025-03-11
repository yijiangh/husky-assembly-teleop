# Data collection

Make sure that the `base_calibration_file` input for Husky class creation in `husky_world` is set to None before the start of calibration.

## Joint 0 (z axis of the arm base link)
1. Recalibrate the camera, make sure you get "exceptional"
2. Drive the robot to the center of the workspace, jack up the robot base to avoid any movement during calibration
3. Use the goal pose gui to select a start conf, plan the motion. This conf should be at one end of the joint 0. Make sure that the robot is not too stretched, the weight of the links might deform the joints or tilt the base.
4. Execute the motion.
5. Use the goal pose gui to select a goal conf, ONLY move joint 0. 
6. You can plan the motion to visualize if there is any collision, but DO NOT execute it.
7. Click on `Calib joint 0` button. This will auto go through all discretized joint 0 values. You have to click enter to continue in the commandline.
8. Repeat 3-7 to do 10 circles for joint 0. Each circle should have different height by different joint conf. 

## Joint 1 (x axis and origin of the arm base link)
1. Click on `Set joint 0 to zero` button to set a goal conf that zeros joint 0 while keeping others the same.
2. Plan a path, visualiza, and execute it.
3. Use the goal pose gui to select a start conf, plan the motion. This conf should be at one end of the joint 1. Make sure that the robot is not too stretched, the weight of the links might deform the joints or tilt the base.
4. Execute the motion.
5. Use the goal pose gui to select a goal conf, ONLY move joint 1. 
6. You can plan the motion to visualize if there is any collision, but DO NOT execute it.
7. Click on `Calib joint 1` button. This will auto go through all discretized joint 0 values. You have to click enter to continue in the commandline.
8. Repeat 3-7 to do 10 circles for joint 1. Each circle should have different height by different joint conf. 

# Data analysis

## Computing base link transformation
1. Do circle fitting for each circle, check the deviation of the points after the fit.  If the distance deviation is large, something is already wrong at this stage, probably the robot moved.
2. Fit a line for all the circle centers to get the center axis of joint 0. Check the deviation of the circle normal to this line, if this deviation is large, sth is wrong.
3. Do the same circle fitting for joint 1 data. The fitted line should intersect with the joint 0 center axis. Same deviation analysis of 1-2 applies here.
4. With the intersection point, we can get the joint 1 offset from base link from the technical drawing of UR5e. Offset this along the joint 0 axis, we can the true origin of the base link. The local z and x axis are defined by the joint 0 and 1 axes above.
5. Now we can compute the transformation between the mocap base frame and the computed base link frame. Save this. Note that if the mocap frame changes, this needs to be computed again.

## Verification
1. Implement the calibrated transformation in simulation. By loading the calibrated frame json file and input that to the husky class creation in `husky_world.py".
1. To verify the calibration, we can first fix the base, move the arm around, and save a bunch of arm configuration by move the goal, plan the path, execute, wait for the arm to settle, and click `Record current calib conf`.
2. Click on `Export calib conf to json` to save a json file.
3. Now move the base around, do the same for 1-2.
4. Check the consistency between the transformation between the `tool0_fk_pose` and `flange_mocap_pose`. If the calibration is correct, the positional and rotational error should be less than 1 mm and 1 degree across all configurations.

An error less than 1 mm is acceptable.