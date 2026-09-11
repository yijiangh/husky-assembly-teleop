#!/usr/bin/env bash
# Start the gripper-only stacks on a husky: both Robotiq 2F-85s, with the tool
# communication bridges, and NO arm drivers -- the combination that leaves the
# arms free for direct ur_rtde control (see doc/rtde_network_setup.md).
#
# Run it from the laptop -- it ssh's in itself -- or on the husky directly.
#
#   scripts/husky/start_gripper_stacks.sh             # start them
#   scripts/husky/start_gripper_stacks.sh --restart   # stop first, then start
#   scripts/husky/start_gripper_stacks.sh --status    # just report, change nothing
#   HUSKY=administrator@192.168.0.115 scripts/husky/start_gripper_stacks.sh   # over wifi
#
# ! The arms must be POWERED ON (RUNNING or IDLE): the tool 24 V the grippers
# ! run on comes through the arm, and the bridge connects to TCP 54321 on it.
# ! Pendants can stay in LOCAL -- a gripper needs no remote mode.
#
# ! It refuses to start while the full crl_dual_ur5e.launch.py stack is up,
# ! because that stack starts its own tool bridges and two bridges to one arm
# ! fight over TCP 54321.
#
# Stop them again with scripts/husky/stop_gripper_stacks.sh.

set -uo pipefail

HUSKY="${HUSKY:-administrator@192.168.131.1}"
MODE="${1:-start}"

# The work, as one string, so it runs the same locally or through ssh.
read -r -d '' REMOTE <<'REMOTE_SCRIPT'
mode="$1"
LEFT_IP=192.168.131.40
RIGHT_IP=192.168.131.41

launch_one() {  # session, namespace, com port, arm ip
  tmux new-session -d -s "$1" "source /etc/clearpath/setup.bash; \
ros2 launch crl_husky crl_gripper.launch.py namespace:=/a200_0806/$2 \
gripper:=robotiq_2F_85 com_port:=$3 start_tool_communication:=true \
robot_ip:=$4 2>&1 | tee /tmp/$1.log"
}

report() {
  echo "=== tmux sessions ==="
  tmux ls 2>&1 || true
  for s in left right; do
    echo "=== $s ==="
    if [ -f "/tmp/gripper_$s.log" ]; then
      n=$(grep -c "Configured and activated" "/tmp/gripper_$s.log" 2>/dev/null || echo 0)
      echo "  controllers activated: $n of 3"
      grep -iE "error|exception|refus|denied" "/tmp/gripper_$s.log" 2>/dev/null \
        | grep -viE "Resending the command|IO Exception: Requested" | tail -2
    else
      echo "  no /tmp/gripper_$s.log"
    fi
  done
  echo "=== gripper action servers ==="
  ( source /etc/clearpath/setup.bash 2>/dev/null
    timeout 15 ros2 action list 2>/dev/null | grep gripper_cmd || echo "  none visible" )
}

if [ "$mode" = "--status" ]; then
  report
  exit 0
fi

# --- preflight ---
if pgrep -af "crl_dual_ur5e|multi_arm_safety_sync" | grep -qv pgrep; then
  echo "REFUSING: the full dual-arm stack is running (it owns the tool bridges)."
  echo "Stop it first, or the two bridges will fight over TCP 54321."
  exit 1
fi

running=0
for s in gripper_left gripper_right; do
  tmux has-session -t "$s" 2>/dev/null && { echo "$s is already running"; running=1; }
done
if [ "$running" = 1 ] && [ "$mode" != "--restart" ]; then
  echo "Nothing started. Use --restart to stop and start them again, or"
  echo "--status to see how they are doing."
  exit 1
fi

if [ "$mode" = "--restart" ]; then
  echo "=== stopping first ==="
  for s in gripper_left gripper_right; do
    tmux kill-session -t "$s" 2>/dev/null && echo "  $s stopped"
  done
  sleep 3
fi

# The 24 V the grippers run on comes through the arms, so an unreachable arm
# means the bridge has nothing to connect to.
for ip in $LEFT_IP $RIGHT_IP; do
  ping -c1 -W2 "$ip" >/dev/null 2>&1 \
    && echo "arm $ip reachable" \
    || echo "WARNING: arm $ip does not answer -- powered off? its gripper will not come up"
done

echo "=== starting ==="
launch_one gripper_left  left_gripper  /tmp/ttyUR_left  $LEFT_IP
launch_one gripper_right right_gripper /tmp/ttyUR_right $RIGHT_IP
echo "  waiting for the controllers to activate ..."
for _ in $(seq 1 20); do
  sleep 1
  ok=0
  for s in left right; do
    [ "$(grep -c 'Configured and activated' /tmp/gripper_$s.log 2>/dev/null || echo 0)" -ge 3 ] \
      && ok=$((ok + 1))
  done
  [ "$ok" = 2 ] && break
done

report
REMOTE_SCRIPT

if [ -e /etc/clearpath/setup.bash ]; then
  echo "running locally on the husky"
  bash -c "$REMOTE" _ "$MODE"
else
  echo "connecting to $HUSKY"
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$HUSKY" "bash -s -- '$MODE'" <<< "$REMOTE"
fi
