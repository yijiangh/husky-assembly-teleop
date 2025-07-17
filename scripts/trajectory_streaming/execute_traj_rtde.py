import json
import time
from rtde_control import RTDEControlInterface
import argparse
import os

LEFT_ROBOT_IP = "192.168.131.40"
RIGHT_ROBOT_IP = "192.168.131.41"
DEFAULT_TRAJ_FILE = "planned_trajectory.json"
ratio = 0.5
SPEED = ratio * 1.0      # rad/s
ACCEL = ratio * 2.0      # rad/s^2
PAUSE = 0.1      # seconds between points

def main(arm):
    # Select robot IP and trajectory file based on arm argument
    if arm == 0:
        ROBOT_IP = LEFT_ROBOT_IP
    elif arm == 1:
        ROBOT_IP = RIGHT_ROBOT_IP
    else:
        raise ValueError(f"Invalid arm index: {arm}. Use 0 for left, 1 for right.")

    # Modify TRAJ_FILE to include arm index before extension if present
    base, ext = os.path.splitext(DEFAULT_TRAJ_FILE)
    traj_file_with_arm = f"{base}_arm{arm}{ext}"
    if os.path.exists(traj_file_with_arm):
        traj_file = traj_file_with_arm
    else:
        print(f"Warning: Trajectory file {traj_file_with_arm} not found. Using default {DEFAULT_TRAJ_FILE}.")
        traj_file = DEFAULT_TRAJ_FILE

    # Load trajectory
    with open(traj_file, "r") as f:
        trajectory = json.load(f)

    # Connect to robot
    rtde_c = RTDEControlInterface(ROBOT_IP)

    # Move through trajectory
    for idx, q in enumerate(trajectory):
        print(f"Moving to point {idx+1}/{len(trajectory)}: {q}")
        rtde_c.moveJ(q, speed=SPEED, acceleration=ACCEL)
        time.sleep(PAUSE)

    # Stop control and disconnect
    rtde_c.stopScript()
    print("Trajectory execution complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Execute a planned trajectory on UR robot via RTDE.")
    parser.add_argument("--arm", type=int, default=0, help="Arm index (0 for left, 1 for right)")
    args = parser.parse_args()
    main(args.arm)