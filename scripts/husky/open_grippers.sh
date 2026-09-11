#!/usr/bin/env bash
# Open the Robotiq grippers FULLY (knuckle 0.0 rad = 85 mm), through the same
# GripperCommand action the open-loop engine uses.
#
#   scripts/husky/open_grippers.sh            # both arms, in parallel
#   scripts/husky/open_grippers.sh left       # one arm
#   scripts/husky/open_grippers.sh right
#
# Needs the gripper stacks running on the Husky (scripts/husky/start_gripper_stacks.sh)
# and this PC's ROS env pointed at them; the env is set below unless already set.
#
# ! Opening drops whatever the gripper holds. Look before you run it.
#
# Expected result per arm: position ~0.003, stalled: false, reached_goal: true.

set -uo pipefail
source "$(dirname "$0")/gripper_env.sh"

gripper_goal "${1:-both}" 0.0 "open"
