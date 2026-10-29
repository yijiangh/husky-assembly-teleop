#!/usr/bin/env bash
# * Read-only checks of the robots from this PC (three-robot manual, Part 3; pane 5 · CHECKS).
# * RUN it, do not source it. It sets each robot's ROS_DOMAIN_ID inside this script only, so
# * the terminal it runs in keeps its own ROS_DOMAIN_ID (and a monitor started there later
# * keeps its robot).
#
#     ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh              # Cindy, Alice, Belle
#     ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh Belle        # one robot
#     ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh Alice Belle  # several
# Robot names in any capitalization (alice = Alice).
#
# Nothing is sent to the robots: it pings their PCs, listens to topics and asks the arms'
# dashboards and controller managers for their state. One line per item, OK or FAIL plus
# what to do; the exit code is the number of FAILs. About 20 s per robot.

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
ORIGINAL_DOMAIN="${ROS_DOMAIN_ID:-unset}"
# The workspace has the tool status message type (crl_husky_msgs); /opt/ros alone does not.
source "$WS/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-file://$HOME/.cyclonedds.xml}"   # pins DDS to eno1

N_FAIL=0
FAILED=()

ok()   { printf '  OK    %s\n' "$1"; }
note() { printf '  NOTE  %s\n' "$1"; }
fail() {
    printf '  FAIL  %s\n        -> %s\n' "$1" "$2"
    N_FAIL=$((N_FAIL + 1))
    FAILED+=("$ROBOT: $1")
}

# * ------------------------------------------------------------- the robots
# Sets DOMAIN, NS (ROS namespace), IP (robot PC), PANE (its ssh pane) and ARMS for robot $1.
robot_vars() {
    case "$1" in
        Cindy) DOMAIN=86; NS=/a200_0806; IP=192.168.0.115; PANE="4 · CINDY"; ARMS="left_ur5e right_ur5e" ;;
        Alice) DOMAIN=84; NS=/a200_0804; IP=192.168.0.113; PANE="2 · ALICE"; ARMS="ur5e" ;;
        Belle) DOMAIN=85; NS=/a200_0805; IP=192.168.0.114; PANE="3 · BELLE"; ARMS="ur5e" ;;
        *) echo "unknown robot '$1' (use Cindy, Alice or Belle)"; exit 2 ;;
    esac
}

# * ------------------------------------------------------------- the checks
# The base driver listens on <ns>/cmd_vel (twist_mux).
check_base() {
    local subs
    subs=$(ros2 topic info "$NS/cmd_vel" --no-daemon --spin-time 4 2>&1 | sed -nE 's/^Subscription count: ([0-9]+)/\1/p')
    if [ "${subs:-0}" -ge 1 ]; then
        ok "base: listens on $NS/cmd_vel"
    elif [ "$ROBOT" = Cindy ]; then
        fail "base: $NS/cmd_vel not visible" "her base driver is down, or this PC is off the robot network (Part 1, network check)"
    else
        note "base: $NS/cmd_vel not visible from this PC (also on 2026-10-16). The monitor does not need it; drive with the joystick"
    fi
}

# A topic's rate. Args: label, topic, lowest OK rate, expected rate, what to do when it fails.
check_rate() {
    local out rate
    out=$(timeout 6 ros2 topic hz "$2" --window 20 2>&1)
    if grep -q 'is invalid' <<<"$out"; then
        fail "$1: message type unknown on this PC" "the workspace is not built or not sourced (Part 1, rebuild)"
        return
    fi
    # The last reading: the first one can be low while the subscriber starts.
    rate=$(grep -oE 'average rate: [0-9.]+' <<<"$out" | tail -1 | awk '{print $3}')
    if [ -z "$rate" ]; then
        fail "$1: no messages on $2" "$5"
    elif awk -v r="$rate" -v m="$3" 'BEGIN { exit !(r < m) }'; then
        fail "$1: only $rate Hz on $2 (expect ~$4 Hz)" "$5"
    else
        ok "$1: $rate Hz"
    fi
}

# What the arm's pendant runs, asked through its dashboard. Args: arm namespace, label.
check_pendant() {
    local arm="$1/dashboard_client" out name state mode remote
    out=$(timeout 8 ros2 service call "$arm/program_state" ur_dashboard_msgs/srv/GetProgramState 2>&1)
    if ! grep -q 'success=True' <<<"$out"; then
        fail "$2 pendant: its dashboard does not answer" \
             "the arm is off, or the robot's launch is not running: pane $PANE, start the launch (Part 2)"
        return
    fi
    name=$(grep -oE "program_name='[^']*'" <<<"$out" | cut -d"'" -f2)
    state=$(grep -oE "state='[A-Z_]+'" <<<"$out" | head -1 | cut -d"'" -f2)
    mode=$(timeout 8 ros2 service call "$arm/get_robot_mode" ur_dashboard_msgs/srv/GetRobotMode 2>&1 \
           | sed -nE "s/.*answer='Robotmode: ([A-Z_]+)'.*/\1/p")
    if [ "$name" = ros_control.urp ] && [ "$state" = PLAYING ] && [ "$mode" = RUNNING ]; then
        ok "$2 pendant: ros_control.urp PLAYING, robot RUNNING (so it is in Remote mode)"
        return
    fi
    remote=$(timeout 8 ros2 service call "$arm/is_in_remote_control" ur_dashboard_msgs/srv/IsInRemoteControl 2>&1 \
             | grep -oE 'remote_control=(True|False)' | cut -d= -f2)
    if [ -n "$mode" ] && [ "$mode" != RUNNING ]; then
        fail "$2 pendant: robot mode $mode" "on the pendant: power on and release the brakes (it shows 'Normal')"
    elif [ "$remote" = False ]; then
        fail "$2 pendant: in Local mode (runs '$name', $state)" \
             "switch it to Remote Control (top-right icon; if missing: Settings > System > Remote Control > Enable). The launch then starts ros_control.urp itself"
    else
        fail "$2 pendant: runs '${name:-?}' ($state), not ros_control.urp" \
             "the launch starts it within ~10 s: run this check again. Still FAIL: restart the launch in pane $PANE. Never start a program by hand"
    fi
}

# The arm's two motion controllers. Args: arm namespace, label.
check_controllers() {
    local out joint compliant
    out=$(timeout 15 ros2 service call "$1/controller_manager/list_controllers" controller_manager_msgs/srv/ListControllers 2>&1 \
          | tr ',' '\n' | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - -)
    joint=$(grep scaled_joint_trajectory_controller <<<"$out" | grep -oE "state='[a-z]+'" | cut -d"'" -f2)
    compliant=$(grep cartesian_compliance_controller <<<"$out" | grep -oE "state='[a-z]+'" | cut -d"'" -f2)
    if [ -z "$out" ]; then
        fail "$2 controllers: none loaded (or no answer)" \
             "the launch probably started before the arm was on and in Remote. Pane $PANE: Ctrl-C the launch, start it again"
    elif [ "$joint" != active ]; then
        fail "$2 controllers: joint controller '${joint:-not loaded}'" \
             "usually the pendant line above (Local mode, program not playing): fix that first. Else 'Switch to Joint (BOTH)' in the monitor"
    elif [ "$compliant" = active ]; then
        fail "$2 controllers: compliance controller still active" "'Switch to Joint (BOTH)' in the monitor, or the command in Part 6 (safety)"
    elif [ -z "$compliant" ]; then
        fail "$2 controllers: compliance controller not loaded" "the insert / retreat / grip handoff cannot go compliant: restart the launch in pane $PANE"
    else
        ok "$2 controllers: joint active, compliance inactive (the monitor turns it on only when needed)"
    fi
}

# A support robot's Robotiq gripper action (its hidden status topic, so no ros2 daemon is needed).
check_gripper_action() {
    local action="$NS/gripper/robotiq_gripper_controller/gripper_cmd"
    if ros2 topic list --no-daemon --spin-time 4 --include-hidden-topics 2>&1 | grep -qx "$action/_action/status"; then
        ok "gripper: action $action is up"
    else
        fail "gripper: no action $action" "the launch line lacks gripper:=robotiq_2F_85, or the gripper driver failed: look in pane $PANE"
    fi
}

# * ------------------------------------------------------------- run
ROBOTS=()
for name in "$@"; do
    name="${name,,}"                 # any capitalization: alice, ALICE, Alice
    ROBOTS+=("${name^}")
done
[ ${#ROBOTS[@]} -eq 0 ] && ROBOTS=(Cindy Alice Belle)
for ROBOT in "${ROBOTS[@]}"; do robot_vars "$ROBOT"; done   # a misspelled name stops here
echo "[check_robots] checking: ${ROBOTS[*]} (to check only some, put their names after the script, e.g. check_robots.sh Alice)"
echo "[check_robots] read-only; this terminal's own ROS_DOMAIN_ID stays '$ORIGINAL_DOMAIN'"

for ROBOT in "${ROBOTS[@]}"; do
    robot_vars "$ROBOT"
    export ROS_DOMAIN_ID=$DOMAIN
    ros2 daemon stop >/dev/null 2>&1   # a stale ros2 daemon can hide topics
    echo
    echo "=== $ROBOT (domain $DOMAIN, $NS; ssh in pane $PANE) ==="
    if ! ping -c1 -W1 "$IP" >/dev/null 2>&1; then
        fail "robot PC $IP does not answer ping" "is $ROBOT on? Is this PC on the robot network (Part 1, network)? Skipping her other checks"
        continue
    fi
    ok "robot PC $IP answers ping"
    # * Anything from this robot at all? Nothing = its launch is not running, or its ROS traffic
    # * leaves through the wrong network port (Alice and Belle on 2026-10-19, Cindy on 2026-10-27).
    if ! ros2 topic list --no-daemon --spin-time 4 2>/dev/null | grep -q "^$NS/"; then
        fail "no topics at all from $NS, although its PC answers" \
             "is the launch running in pane $PANE? If it is, this PC does not receive its traffic: Ctrl-C the launch and start it again with the CYCLONEDDS_URI lines of Part 2. Skipping her other checks"
        continue
    fi
    ok "topics from $NS arrive"
    check_base
    for arm in $ARMS; do
        label=$([ "$arm" = ur5e ] && echo "arm" || echo "${arm%_ur5e} arm")
        check_rate "$label joint states" "$NS/$arm/rate_limiter/joint_states" 40 50 \
                   "the arm driver sends nothing: see the pendant / controllers lines below, else restart the launch in pane $PANE"
        check_pendant "$NS/$arm" "$label"
        check_controllers "$NS/$arm" "$label"
    done
    if [ "$ROBOT" = Cindy ]; then
        for side in left right; do
            check_rate "$side tool status" "$NS/${side}_gripper/tool_status" 15 20 \
                       "that tool's driver has stopped: restart Cindy's launch in pane $PANE (nothing clamped in the tools)"
        done
    else
        check_gripper_action
    fi
done

echo
if [ "$N_FAIL" -eq 0 ]; then
    echo "[check_robots] all OK"
else
    echo "[check_robots] $N_FAIL FAIL:"
    printf '  - %s\n' "${FAILED[@]}"
fi
exit $(( N_FAIL > 255 ? 255 : N_FAIL ))
