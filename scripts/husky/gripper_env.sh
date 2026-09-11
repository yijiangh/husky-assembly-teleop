#!/usr/bin/env bash
# Shared by open_grippers.sh / close_grippers.sh: the ROS environment that reaches
# Cindy's gripper stacks, and one function that sends a GripperCommand goal to one
# or both arms and prints what came back.
#
# ! Cindy's stacks live in ROS_DOMAIN_ID 86 on Cyclone DDS (doc/rtde_network_setup.md).
# ! A shell without these exports discovers NOTHING and every goal just waits for a
# ! server that never appears -- the failure of 2026-09-10. They are set here only if
# ! the shell has not set them already, so an explicit environment always wins.

# ROS's setup.bash reads variables it never set; suspend nounset just for it.
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/.cyclonedds.xml}"

ROBOT_NS="${ROBOT_NS:-/a200_0806}"
GRIPPER_EFFORT="${GRIPPER_EFFORT:-0.1}"    # never forwarded by the humble driver; kept for the record
GOAL_TIMEOUT_S="${GOAL_TIMEOUT_S:-20}"

# send_one <side> <position> <label>: one goal, result reduced to the lines that matter.
send_one() {
  local side="$1" position="$2" label="$3"
  local action="$ROBOT_NS/${side}_gripper/robotiq_gripper_controller/gripper_cmd"
  local out
  out=$(timeout "$GOAL_TIMEOUT_S" ros2 action send_goal "$action" control_msgs/action/GripperCommand \
        "{command: {position: $position, max_effort: $GRIPPER_EFFORT}}" 2>&1)
  local rc=$?
  if [ $rc -eq 124 ]; then
    echo "$side: NO ANSWER in ${GOAL_TIMEOUT_S}s -- no action server at $action"
    echo "       (stacks down? run scripts/husky/start_gripper_stacks.sh --status;"
    echo "        env wrong? this shell has ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION)"
    return 1
  fi
  # The result block: final position, stalled, reached_goal.
  local pos stalled reached
  pos=$(echo "$out" | awk '/Result:/{f=1} f && /position:/{print $2; exit}')
  stalled=$(echo "$out" | awk '/Result:/{f=1} f && /stalled:/{print $2; exit}')
  reached=$(echo "$out" | awk '/Result:/{f=1} f && /reached_goal:/{print $2; exit}')
  if [ -z "$pos" ]; then
    echo "$side: goal sent but no result parsed -- raw output follows"; echo "$out" | tail -8
    return 1
  fi
  printf '%s %-5s -> position %s rad  stalled %s  reached_goal %s\n' \
         "$side" "$label" "$pos" "$stalled" "$reached"
}

# gripper_goal <left|right|both> <position> <label>
gripper_goal() {
  local which="$1" position="$2" label="$3"
  case "$which" in
    left|right) send_one "$which" "$position" "$label" ;;
    both)
      send_one left  "$position" "$label" &
      send_one right "$position" "$label" &
      wait ;;
    *) echo "usage: $0 [left|right|both]"; return 2 ;;
  esac
}
