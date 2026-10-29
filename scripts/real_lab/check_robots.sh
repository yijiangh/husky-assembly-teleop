#!/bin/bash
# Real lab: read-only checks of this laptop, ROS domain 80 and the robots (manual Part 3; pane checks). Nothing is
# sent to a robot: it pings them, reads their settings over ssh and asks ROS what runs. What the monitor's health
# chips show once it runs (rates, e-stops, batteries, the arms' safety state) is left to them.
#
# Usage: check_robots.sh [alice] [belle] [cindy]    the robots to check, any capitalization; default all three
#
# One line per item: OK, NOTE (worth a look) or FAIL with what to do under it; the exit code is the number of FAILs.
# About 20 s.

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"

n_fail=0
failed=()
# A line that passed, or one worth a look; the arguments are joined with spaces.
ok()   { printf '  OK    %s\n' "$*"; }
note() { printf '  NOTE  %s\n' "$*"; }
# A line that failed: what ($1), and what to do ($2).
fail() {
    printf '  FAIL  %s\n        -> %s\n' "$1" "$2"
    n_fail=$((n_fail + 1))
    failed+=("$who: $1")
}

# The ROS domain a process of this user runs in, from its environment; empty if unknown.
domain_of() {
    tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p'
}

# Whether a process of this user whose command line matches $1 runs in domain 80.
runs_here() {
    local pid
    for pid in $(pgrep -u "$USER" -f "$1"); do
        [ "$(domain_of "$pid")" = 80 ] && return 0
    done
    return 1
}

# * ------------------------------------------------------------- this laptop
check_laptop() {
    who="this laptop"
    echo
    echo "=== this laptop ==="
    local line address port pid
    line=$(ip -4 -o addr show | awk '/ 192\.168\.0\./ {print $2, $4; exit}')
    address=${line#* }
    address=${address%/*}
    if [ -z "$line" ]; then
        fail "not on the lab network (no 192.168.0.x address)" \
             "plug the USB-ethernet dongle into the lab switch; nmcli connection up \"Wired connection 1\" (manual 1.b)"
    elif [[ "$address" =~ ^192\.168\.0\.(20[1-9]|210)$ ]]; then
        ok "on the lab network: $address on ${line% *} (a control-PC slot)"
    else
        note "on the lab network as $address (${line% *}), not in a control-PC slot (.201-.210): the robots take" \
             "no time from this laptop (manual 1.b)"
    fi
    if real_lab_router_up; then
        ok "Zenoh router listening on 7447 (service crl-zenoh-router: $(systemctl --user is-active crl-zenoh-router))"
    else
        fail "no Zenoh router on this laptop: nothing here sees a robot" \
             "install it once: $REAL_LAB_WS/src/crl-husky/crl_husky/scripts/zenoh_router.sh --install (manual 1.c)"
    fi
    if ! command -v chronyc > /dev/null; then
        fail "no time sync with the robots: chrony is not installed" \
             "install it once (manual 1.d); until then the health chips may say a robot's clock is ahead or behind"
    elif chronyc -n tracking 2>/dev/null | grep -q '^Leap status *: Normal'; then
        ok "time: chrony synced, reference $(chronyc -n tracking | sed -n 's/^Reference ID *: //p')"
    else
        note "time: chrony runs but is not synced yet (chronyc -n sources)"
    fi
    for port in 8080 8081; do
        pid=$(ss -Htlnp "( sport = :$port )" 2>/dev/null | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
        if [ -z "$pid" ] && ! ss -Htln "( sport = :$port )" 2>/dev/null | grep -q .; then
            ok "port $port free for the monitor"
        elif [ -n "$pid" ] && [ "$(domain_of "$pid")" = 80 ]; then
            ok "port $port: this lab's monitor runs (pane monitor)"
        else
            local holder="process ${pid:-of another user}, ROS domain $(domain_of "$pid")"
            fail "port $port is taken ($holder): the monitor would refuse to start" \
                 "the fake lab's monitor? Stop it (manual 1.a)"
        fi
    done
    if tmux has-session -t =fake_lab 2>/dev/null; then
        note "the fake lab's tmux session still runs: stop it when you are done with it (manual 1.a)"
    fi
    if [ ! -f "$HUSKY_DRIVE_ROOT/$REAL_LAB_PROCESS/process.json" ]; then
        fail "no $REAL_LAB_PROCESS under the Drive root '${HUSKY_DRIVE_ROOT}'" \
             "is Insync running? Or give REAL_LAB_DRIVE your Drive folder (manual 1.h, 7.h)"
    elif ls -a "$HUSKY_DRIVE_ROOT/$REAL_LAB_PROCESS" | grep -q insyncdl; then
        note "the process folder is still syncing (Insync): wait before Load"
    else
        ok "the process on the Drive: $REAL_LAB_PROCESS"
    fi
    if ping -c1 -W1 "$REAL_LAB_MOTIVE" > /dev/null 2>&1; then
        ok "the Motive PC $REAL_LAB_MOTIVE answers ping"
    else
        fail "the Motive PC $REAL_LAB_MOTIVE does not answer ping" \
             "is it on? Moved to .117 (TIME_SYNC.md)? Then in pane relay: REAL_LAB_MOTIVE=192.168.0.117 relay.sh (7.h)"
    fi
}

# * ------------------------------------------------------------- ROS domain 80, once for all robots
asked="" topics="" nodes="" bodies=""
ask_ros() {
    who="ROS domain 80"
    echo
    echo "=== ROS domain 80 (Zenoh) ==="
    if ! real_lab_router_up; then
        note "not asked: no Zenoh router on this laptop (above)"
        return 1
    fi
    asked=yes
    timeout -s KILL 10 ros2 daemon stop > /dev/null 2>&1  # a stale ros2 daemon can answer for an old graph
    # * The publishers only: on Zenoh a topic someone merely subscribes to is listed too.
    topics=$(timeout -s KILL 20 ros2 topic list -v --no-daemon --spin-time 5 --include-hidden-topics 2>/dev/null \
             | sed -n '/^Published topics:/,/^Subscribed topics:/p' | awk '/ \* / {print $2}')
    nodes=$(timeout -s KILL 20 ros2 node list --no-daemon --spin-time 4 2>/dev/null)
    local relays monitors
    relays=$(grep -cx /mocap_relay <<< "$nodes")
    monitors=$(grep -cx /husky_monitor <<< "$nodes")
    if [ "$relays" = 0 ]; then
        fail "no mocap relay runs: no robot has a base pose, and the followers cannot drive" \
             "pane relay: Up, Enter (relay.sh); its log says why it stopped"
    elif [ "$relays" != 1 ]; then
        fail "$relays mocap relays run in domain 80" "stop all but one: two publish interleaved poses"
    elif runs_here husky_mocap_relay; then
        ok "one mocap relay: this laptop's (pane relay)"
    else
        note "one mocap relay, on another PC: its calibrations are that PC's Drive copy's (fine if it is the lab's)"
    fi
    if [ "$monitors" = 0 ]; then
        ok "no monitor runs yet in domain 80"
    elif [ "$monitors" = 1 ] && runs_here husky_monitor; then
        ok "one monitor: this laptop's (pane monitor)"
    else
        fail "another monitor runs in domain 80 (another PC's)" \
             "find it and stop it before you start yours: two monitors would both command the robots"
    fi
    [ "$relays" = 0 ] && return 0  # no relay: nobody answers for Motive
    bodies=$(timeout -s KILL 15 ros2 service call /mocap/get_streaming_ids crl_husky_msgs/srv/GetStreamingMocapIds \
             "{}" 2>/dev/null | grep -o 'ids=\[[0-9, ]*\]' | grep -oE '[0-9]+' | paste -sd ' ')
    local bar missing=""
    for bar in 1020:B20 1021:B21; do
        grep -qw "${bar%:*}" <<< "$bodies" || missing+="${missing:+, }${bar#*:} (${bar%:*})"
    done
    if [ -z "$bodies" ]; then
        fail "the relay names no rigid body that Motive streams" \
             "is Motive streaming to this laptop? The relay's pane says"
    elif [ -n "$missing" ]; then
        note "Motive streams $bodies; not the bar asset(s) $missing: their servo then aims tool0 only (manual 2.f)"
    else
        ok "Motive streams $bodies (B20, B21 among them)"
    fi
}

# * ------------------------------------------------------------- a robot
# The value after "<$1>=" in what the robot printed, without quotes.
remote_value() {
    sed -n "s/^$1=//p" <<< "$remote" | head -1 | tr -d "\"'"
}

# The robot's settings and crl-husky files, over ssh; sets `files` (this laptop's list) and `remote` (what it printed).
read_robot_pc() {
    # * Its crl-husky code and messages, compared file by file with this laptop's (upload.sh copies files, no git).
    files=$(git -C "$REAL_LAB_WS/src/crl-husky" ls-files crl_husky_msgs crl_husky crl_husky_cpp | grep -v '\.md$')
    remote=$(timeout -s KILL 30 ssh -o BatchMode=yes -o ConnectTimeout=5 "administrator@$ROBOT_IP" bash -s 2>&1 <<EOF
grep -E '^export (RMW_IMPLEMENTATION|ROS_DOMAIN_ID)=' /etc/clearpath/setup.bash
echo "zenoh=\$(dpkg-query -W -f '\${Version}' ros-humble-rmw-zenoh-cpp 2>/dev/null)"
echo "router=\$(systemctl is-active crl-zenoh-router 2>/dev/null)"
echo "launches=\$(pgrep -fc '^[^ ]*python3[^ ]* [^ ]*ros2 launch crl_husky')"
echo "launch=\$(pgrep -fa '^[^ ]*python3[^ ]* [^ ]*ros2 launch crl_husky' | sed -n '1s/.* launch crl_husky //p')"
[ -d ~/workspace/src/crl-husky ] || echo "src=missing"
echo "clock=\$(chronyc -n tracking 2>/dev/null | grep -E '^(Reference ID|System time)' | tr -s ' ' | paste -sd ';')"
cd ~/workspace/src/crl-husky 2>/dev/null && sha1sum $(echo $files) 2>/dev/null
echo "end=ok"
EOF
)
}

check_robot_pc() {
    read_robot_pc
    if ! grep -q '^end=ok$' <<< "$remote"; then
        if grep -q 'Permission denied' <<< "$remote"; then
            note "ssh asks for a password, so her settings are not checked: install this laptop's key once (manual 1.e)"
        elif grep -q 'Host key verification failed' <<< "$remote"; then
            note "ssh does not know her host key yet: ssh administrator@$ROBOT_IP once, answer yes (manual 1.e)"
        else
            fail "ssh to her failed: $(tail -1 <<< "$remote")" "she answers ping: is her PC still booting?"
        fi
        return
    fi
    local rmw domain zenoh router launch launches clock
    rmw=$(remote_value 'export RMW_IMPLEMENTATION')
    domain=$(remote_value 'export ROS_DOMAIN_ID')
    if [ "$rmw" = rmw_zenoh_cpp ] && [ "$domain" = 80 ]; then
        ok "her base services: rmw_zenoh_cpp, domain 80"
    else
        fail "her base services run ${rmw:-?} on domain ${domain:-?}, not Zenoh on 80: the monitor cannot see her" \
             "robot.sh opens a shell on her (pane $who): there crl-husky-mw-zenoh, not while she holds a bar (2.b)"
    fi
    zenoh=$(remote_value zenoh)
    router=$(remote_value router)
    launch=$(remote_value launch)
    launches=$(remote_value launches)
    clock=$(remote_value clock)
    local ours
    ours=$(dpkg-query -W -f '${Version}' ros-humble-rmw-zenoh-cpp 2>/dev/null)
    if [ "$zenoh" = "$ours" ]; then
        ok "rmw_zenoh ${zenoh%%-*}, as on this laptop"
    else
        fail "her rmw_zenoh is ${zenoh:-not installed}, this laptop's $ours" \
             "every machine needs the same version (crl-husky README, Zenoh): apt upgrade the older one"
    fi
    if [ "$router" = active ]; then
        ok "her Zenoh router runs"
    else
        fail "her Zenoh router is ${router:-unknown}" "on her: crl-husky-zenoh-restart"
    fi
    # * The same messages are a must (else a topic between them never decodes); the same code is what was tested.
    local mine diff msgs others
    mine=$(cd "$REAL_LAB_WS/src/crl-husky" && sha1sum $files)
    diff=$(comm -23 <(sort <<< "$mine") <(grep -E '^[0-9a-f]{40}  ' <<< "$remote" | sort) | awk '{print $2}')
    msgs=$(grep -c '^crl_husky_msgs/' <<< "$diff")
    others=$(grep -vc '^crl_husky_msgs/' <<< "$diff")
    if [ "$(remote_value src)" = missing ]; then
        fail "her crl-husky is not in ~/workspace/src/crl-husky: its code cannot be compared" \
             "where does her workspace live? Ask Jakob (manual 1.g)"
    elif [ -z "$diff" ]; then
        ok "her crl-husky: the same code and messages as this laptop's"
    elif [ "$msgs" != 0 ]; then
        local which
        which=$(grep '^crl_husky_msgs/' <<< "$diff" | head -3 | paste -sd ' ')
        fail "her crl_husky_msgs differ from this laptop's ($which): topics between them may not decode" \
             "bring both to one crl-husky commit before the test (manual 1.g)"
    else
        note "her crl-husky differs from this laptop's in $others file(s), e.g." \
             "$(head -2 <<< "$diff" | paste -sd ' '); the messages are the same (manual 1.g)"
    fi
    if [ "${launches:-0}" -gt 1 ]; then
        fail "$launches crl_husky launches run on her: a second driver and controller manager for each arm" \
             "stop all but one (pane $who: robot.sh lists them); then check again"
    elif [[ "$launch" == crl_arms_and_base.launch.py* ]]; then
        ok "her launch runs: $launch"
    elif [ -n "$launch" ]; then
        note "her launch is $launch, not crl_arms_and_base.launch.py: without the follower, Commit cannot drive her"
    else
        fail "no crl_husky launch runs on her" \
             "pane $who: Enter (robot.sh $who), once her pendants are in Remote (manual 2.a)"
    fi
    if [ -n "$clock" ]; then
        ok "her clock (chrony): $clock"
    else
        note "her clock: chronyc gives nothing (chrony not set up on her?)"
    fi
}

# A robot as ROS sees her: what her launch and base publish, her Robotiq's action server, her arms' controllers and
# her Motive body.
check_robot_ros() {
    [ -z "$asked" ] && return  # ROS was not asked: no router
    local wanted="$ROBOT_NS/MultiArmSafetySync/status $ROBOT_NS/base_follower/state $ROBOT_NS/platform/joint_states"
    local arm side missing="" topic
    for arm in $ROBOT_ARMS; do
        wanted+=" $ROBOT_NS/$arm/rate_limiter/joint_states"
    done
    if [ "$who" = cindy ]; then
        for side in left right; do
            wanted+=" $ROBOT_NS/${side}_gripper/tool_status"
        done
    fi
    for topic in $wanted; do
        grep -qx "$topic" <<< "$topics" || missing+=" ${topic#$ROBOT_NS/}"
    done
    if [ -z "$missing" ]; then
        local tools=""
        [ "$who" = cindy ] && tools=" and both tools"
        ok "publishing: the arms' joint states, the safety sync, the follower, the base$tools"
    else
        fail "nothing publishes on:$missing" \
             "platform/: on her, crl-husky-clearpath-restart; the rest: her launch (pane $who) runs? Its log says why"
    fi
    if [ "$who" != cindy ]; then
        # * The action server publishes its goals' status; a client only subscribes. (ros2 action info would leave a
        #   ros2 daemon running and look for half a second only.)
        if grep -qx "$ROBOT_NS/gripper/robotiq_gripper_controller/gripper_cmd/_action/status" <<< "$topics"; then
            ok "the Robotiq's action server is up"
        else
            fail "no Robotiq action server ($ROBOT_NS/gripper/robotiq_gripper_controller/gripper_cmd)" \
                 "her gripper driver failed: read pane $who (crl-husky README, Restart: Robotiq dead after an e-stop)"
        fi
    fi
    local listed joint compliant label
    for arm in $ROBOT_ARMS; do
        label=$([ "$arm" = ur5e ] && echo "arm" || echo "${arm%_ur5e} arm")
        listed=$(timeout -s KILL 15 ros2 service call "$ROBOT_NS/$arm/controller_manager/list_controllers" \
                 controller_manager_msgs/srv/ListControllers 2>/dev/null \
                 | tr ',' '\n' | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - -)
        joint=$(grep "'scaled_joint_trajectory_controller'" <<< "$listed" | grep -oE "state='[a-z]+'" | cut -d"'" -f2)
        compliant=$(grep "'cartesian_compliance_controller'" <<< "$listed" | grep -oE "state='[a-z]+'" | cut -d"'" -f2)
        if [ -z "$listed" ]; then
            fail "$label controllers: no answer" \
                 "her launch started before the arm was on and in Remote? Ctrl-C it in pane $who, start it again"
        elif [ "$joint" != active ]; then
            fail "$label controllers: the joint controller is ${joint:-not loaded}" \
                 "pendant in Remote, program running? The monitor's arm chip says (hover); else Control › Run: Joint"
        elif [ "$compliant" = active ]; then
            fail "$label controllers: the compliance controller is still active" \
                 "Control › $ROBOT_SERIAL › arm › Run: Joint (manual 7.b)"
        elif [ -z "$compliant" ]; then
            fail "$label controllers: no compliance controller loaded: every compliant part would fail at the switch" \
                 "her crl-husky build lacks cartesian_controllers? Ask Jakob; restart her launch"
        else
            ok "$label controllers: joint active, compliance loaded and inactive"
        fi
    done
    if grep -qw "$ROBOT_BODY" <<< "$bodies"; then
        ok "Motive streams her rigid body $ROBOT_BODY"
    elif [ -n "$bodies" ]; then
        fail "Motive does not stream her rigid body $ROBOT_BODY" "in Motive: her asset on (manual 2.f)"
    fi
}

check_robot() {
    real_lab_robot "$1"
    who="$1"
    echo
    echo "=== $1 ($ROBOT_NS, $ROBOT_IP; pane $1) ==="
    if ! ping -c1 -W1 "$ROBOT_IP" > /dev/null 2>&1; then
        fail "her PC $ROBOT_IP does not answer ping" "is she on (manual 2.a)? Her other checks are skipped"
        return
    fi
    ok "her PC answers ping"
    if ss -Htn state established 2>/dev/null | awk '{print $4}' | grep -qx "$ROBOT_IP:7447"; then
        ok "this laptop's Zenoh router is linked to hers"
    else
        fail "this laptop's Zenoh router is not linked to hers" \
             "on her: crl-husky-zenoh-restart; or here: systemctl --user restart crl-zenoh-router (manual 8.b)"
    fi
    check_robot_pc
    check_robot_ros
}

# * ------------------------------------------------------------- run
robots=()
for name in "$@"; do
    name="${name,,}"  # any capitalization: alice, Alice, ALICE
    real_lab_robot "$name" || { echo "unknown robot '$name' (alice, belle or cindy)" >&2; exit 2; }
    robots+=("$name")
done
[ ${#robots[@]} -eq 0 ] && robots=(alice belle cindy)
echo "[check_robots] checking this laptop, ROS domain 80 and ${robots[*]}; read-only, nothing is sent to a robot"
check_laptop
ask_ros
for robot in "${robots[@]}"; do
    check_robot "$robot"
done
echo
if [ "$n_fail" -eq 0 ]; then
    echo "[check_robots] all OK"
else
    echo "[check_robots] $n_fail FAIL:"
    printf '  - %s\n' "${failed[@]}"
fi
exit $((n_fail > 255 ? 255 : n_fail))
