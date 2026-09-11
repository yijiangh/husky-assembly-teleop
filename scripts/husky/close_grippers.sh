#!/usr/bin/env bash
# Close the Robotiq grippers (knuckle 0.8 rad = the engine's close command),
# through the same GripperCommand action the open-loop engine uses.
#
#   scripts/husky/close_grippers.sh           # both arms, in parallel
#   scripts/husky/close_grippers.sh left      # one arm
#   scripts/husky/close_grippers.sh right
#
# Needs the gripper stacks running on the Husky (scripts/husky/start_gripper_stacks.sh)
# and this PC's ROS env pointed at them; the env is set below unless already set.
#
# ! The fingers close on whatever is between them, at the driver's own force.
# ! Make sure nothing is there that should not be squeezed.
#
# Expected result per arm:
#   nothing between the pads -> position ~0.789, stalled: true   (the empty-close signature)
#   a part between the pads  -> a smaller position, stalled: true (grasped)
#   reached_goal: true only means the fingers hit the 0.8 target, i.e. they closed on air.

set -uo pipefail
source "$(dirname "$0")/gripper_env.sh"

gripper_goal "${1:-both}" 0.8 "close"
