#!/usr/bin/env bash
# Stop the gripper-only stacks on a husky (the tmux sessions started by the
# crl_gripper.launch.py recipe in doc/rtde_network_setup.md).
#
# Run it from the laptop -- it ssh's in itself -- or on the husky directly.
#
#   scripts/husky/stop_gripper_stacks.sh            # stop them
#   scripts/husky/stop_gripper_stacks.sh --status   # just report, change nothing
#   HUSKY=administrator@192.168.0.115 scripts/husky/stop_gripper_stacks.sh   # over wifi
#
# ! It only ever touches the two gripper sessions. The Clearpath PLATFORM
# ! ros2_control_node (namespace /a200_0806, params from
# ! /etc/clearpath/platform/config/control.yaml) belongs to the robot's own
# ! boot and is deliberately left alone -- killing it takes the base down.
#
# Stop these before launching the full crl_dual_ur5e.launch.py stack: it starts
# its own tool-communication bridges, and two bridges fight over TCP 54321.

set -uo pipefail

HUSKY="${HUSKY:-administrator@192.168.131.1}"
SESSIONS=(gripper_left gripper_right)
MODE="${1:-stop}"

# The work, as one string, so it runs the same locally or through ssh.
read -r -d '' REMOTE <<REMOTE_SCRIPT
mode="\$1"
echo "=== tmux sessions before ==="
tmux ls 2>&1 || true

if [ "\$mode" != "--status" ]; then
  echo "=== stopping ==="
  for s in ${SESSIONS[*]}; do
    if tmux has-session -t "\$s" 2>/dev/null; then
      tmux kill-session -t "\$s" && echo "  \$s stopped"
    else
      echo "  \$s was not running"
    fi
  done
  sleep 3
  echo "=== tmux sessions after ==="
  tmux ls 2>&1 || true
fi

# Anything the sessions owned that outlived them: the gripper controller
# managers and their socat bridges. Matched by the gripper namespace so the
# platform ros2_control_node is never selected.
echo "=== leftover gripper processes (empty is good) ==="
pgrep -af "left_gripper|right_gripper|ttyUR_left|ttyUR_right" | grep -v pgrep || echo "  none"
echo "=== platform node (should still be running, do not kill) ==="
pgrep -af "ros2_control_node.*platform" | grep -v pgrep | cut -c1-100 || echo "  not running"
REMOTE_SCRIPT

if [ -e /etc/clearpath/setup.bash ]; then
  echo "running locally on the husky"
  bash -c "$REMOTE" _ "$MODE"
else
  echo "connecting to $HUSKY"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$HUSKY" "bash -s -- '$MODE'" <<< "$REMOTE"
fi
