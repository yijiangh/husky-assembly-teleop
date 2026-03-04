# crl-husky ROS 2

This is the repository for the Husky ROS2 integration.

## Startup

Husky Startup:

- Plug workstation into or connect to the Husky Switch
- Startup Optitrack PC in Ubuntu if you need optitrack (for PW please ask one of the PhDs familiar with the setup)
- Start the Husky:
    - Press the power button
    - Turn on the remote e-stop and press "go" (do not parallelize this step with multiple huskies, sometimes both huskies connect to one e-stop...)
    - Once the communication light on the husky turns green (or yellow, if the e-stop is still active), you can connect the PS controller by pressing the PlayStation logo button and waiting for the backlight to stop blinking and stay on permanently
    - You now can move the husky by pressing L1 or L2 and using the left joystick.
    - Continue in Terminal on your PC
- Start UR5:
    - Move the Husky to a safe spot with the remote (press L1 or L2 + use joystick), where the arm cannot hit any other equipment
    - Press the UR5 startup button to start the UR5. Wait until it has started (see tablet), then power it on via the tablet (left lower corner "Power")
    - Set mode to external control in upper right corner. Starting ros_control.urp is handled by the driver we will start later.
        > ⚠️ Please do not manually start ros_control.urp and make sure to stop ros_control.urp if switching back to local control. Not doing this could break the drivers ability to sync the safety stops of the arms. (E-Stop will be always synced though)
    - Continue in Terminal on your PC

ROS Communication:

- Base:
    - The base controller is already running and should be visible form any ROS2 humble system connected to the husky switch.
    - Control the base using `/a200-0804/cmd_vel` messages

- UR5:
    - Log in to ssh `ssh administrator@192.168.0.113`, pw "clearpath" (`.114` for Belle, `.115` for Cindy).
        - Yijiang finally had enough and changed Cindy's password to "12345678".
    - Launch UR5e controller using `ros2 launch crl-husky crl_single_ur5e.launch.py namespace:='/a200_0804'`
    - For *Cindy the dual arm*, use `ros2 launch crl-husky crl_dual_ur5e.launch.py namespace:='/a200_0806'`
    - Launch UR5e + robotiq 2F-85 controller using `ros2 launch crl-husky crl_single_ur5e_gripper.launch.py namespace:='/a200_0804'`
    - Get joint states from `/a200_0804/ur5e/joint_states`
    - Send trajectories to `/a200_0804/ur5e/scaled_joint_trajectory_controller/follow_joint_trajectory` or other suitable controller.
    - Gripper can be controlled using `/a200_0804/gripper/robotiq_gripper_controller/gripper_cmd`
    - Add `use_fake_hardware:=true` to use simulated ur5e instead of real robot.

## Check ROS2 Connection

Make sure to match the ROS2 domain using `export ROS_DOMAIN_ID=84` and use `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

Run `rqt` to get a useful node graph and topic list.\
![ROS_GRAPH_SVG](data/rosgraph.svg)
Following commands can be used to see the same information in the terminal:\
`ros2 topic list`\
`ros2 topic echo /a200_0804/ur5e/joint_states`\
`ros2 topic hz /a200_0804/ur5e/joint_states`\
`ros2 node list`\

## Connect multiple robots using Zenoh

Zenoh minimizes discovery traffic overhead and is better suited for multirobot setups over WIFI. Robots have different ros domains to isolate cyclone dds traffic between them. 

- Start Zenoh on huskies using `zenoh-bridge-ros2dds -e tcp/192.168.0.7:7447` where 192.168.0.7 is the IP of the workstation PC.
- Start Zenoh on workstation using `zenoh-bridge-ros2dds -l tcp/192.168.0.7:7447`

> ⛔ Zenoh seems to have a [bug](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/issues/368) recreating subscribers. For now avoid restarting your application and if you encounter problems restart Zenoh on huskies and workstation.

## Switching Controllers

Switching controller can be done using rqt > Plugins > Robot Tools > Controller Manager or `ros2 run rqt_controller_manager rqt_controller_manager` First, deactivate the currently running controller (by default this is `Scaled Joint Trajectory Controller`). Second, activate the desired controller.

|Name|Description|Interfaces|
|----|-----------|------|
|Carteisan Motion Controller|PD poisition control of EE in cartesian (base frame) space.|`~/target_frame` topic|
|Carteisan Force Controller|Force control of EE in cartesian (base frame) space.|`~/target_wrench` topic|
|Carteisan Compliance Controller|Force control and PD position control in cartesian (base frame) space. Stiffness of virtual spring determines force vs position weighting.|`~/target_frame` and `~/target_wrench` topic|
|Free Drive Controller|Enables UR5e free drive mode. Needs 2HZ enabling signal. `ros2 topic pub --rate 2 /a200_0804/ur5e/free_drive_controller/enable_freedrive_mode std_msgs/msg/Bool "{data: True}"` |`~/enable_freedrive_mode` topic|
|Force Mode Controller|Interface to use URScript force_mode(...). |`~/start_force_mode` and `~/stop_force_mode` serivce |

See [github](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers) for more information about cartesian controllers.

## Shutdown

Ideally, the husky should be turned off by first connecting via ssh running `sudo shutdown now` and only then pressing the power button. Just removing power could lead to corrupt file systems (as with the previous ROS1 system!) but it should be fine in most cases.

## Restart

Some problems mentioned below can be resolved by a restart. Use `sudo restart now` as it avoids the need to also restart the ur5e arm. (pressing the power button cuts all power, including power for ur5e!)

## Troubleshooting

- Both huskies are paired to one e-stop
    - This can happen when starting both huskies in parallel
    - Still works as e-stop, but connection appears to be less stable. Sudden loss of connection and stops were observed.
    - Turn all huskies and e-stops of (pressing red button until light stop flashing)
    - For each pair at a time:
        - Put e-stop in pairing mode (press and hold on button until both lights start flashing)
        - Power on corresponding husky

- Zenoh doesnt show all topics or does not transmit messages
    - reboot zenoh on all devices

- Husky does not respond to joystick
    - probably a bug in the ros joy node
    - reboot fixes it
    - use teleop keyboard instead `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a200_0806/joy_teleop/cmd_vel`

## Huskies

|Name|Serial|Ip|ROS Domain|Comment|
|----|------|--|----------|-------|
|Alice|0804|192.168.0.113|84| Single-Arm |
|Belle|0805|192.168.0.114|85|  Single-Arm |
|Cindy|0806|192.168.0.115|86|  Dual-Arm   |

> ⚠️ The E-Stop on the husky does not stop the UR5e! Always use the remote E-Stop when using the robot arms.

## Future Work
* Some connectivity issues appear to affect services and actions more than topics. Switching to the topic interface for joint trajectory controller could help. [Docs](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

## TODO
 - Clean Startup instructions
 - Document dual arm topic splitter node
