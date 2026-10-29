#!/bin/bash
# Real lab: log in to a robot and run its arms, tools and base follower there (crl-husky's
# crl_arms_and_base.launch.py) until Ctrl-C; then this terminal is back on the laptop, and Up, Enter starts it again.
# A robot that is not on Zenoh in domain 80, not the robot named, or already running a crl_husky launch gets a shell on
# it instead of the launch.
#
# Usage: robot.sh alice|belle|cindy [<argument>:=<value> ...]    more arguments for the launch, e.g. base:=false
#
# ! Start it only once the robot's arms are on and their pendants in Remote (manual 2.a): an arm that is off or in
#   Local mode comes up without its controllers.
# ! Ctrl-C here stops the robot's arm drivers, its tools and its follower: never while it holds a bar.

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/env.sh"
name="${1,,}"  # any capitalization: cindy, Cindy
if ! real_lab_robot "$name"; then
    echo "usage: $0 alice|belle|cindy [<argument>:=<value> ...]" >&2
    exit 2
fi
shift
launch="ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=$ROBOT_GRIPPER${*:+ $*}"

# * Run on the robot, in an interactive shell: its ~/.bashrc sets the robot's middleware, domain, identity and
#   workspace (Clearpath's setup and crl-husky's install_robot_config.sh).
remote=$(cat <<EOF
echo "[\$(hostname)] \$RMW_IMPLEMENTATION, ROS domain \$ROS_DOMAIN_ID, robot \$CRL_HUSKY_SN"
if [ "\$RMW_IMPLEMENTATION" != rmw_zenoh_cpp ] || [ "\$ROS_DOMAIN_ID" != 80 ]; then
    echo "real lab: $name runs \${RMW_IMPLEMENTATION:-no middleware} on domain \${ROS_DOMAIN_ID:-none}, not Zenoh, 80."
    echo "  Switch her here with   crl-husky-mw-zenoh   (it restarts her base services: never while she holds a bar),"
    echo "  then   exit   and start robot.sh $name again (Up, Enter)."
    exec bash -i
fi
if [ "\$CRL_HUSKY_SN" != $ROBOT_SERIAL ]; then
    echo "real lab: this PC is robot \${CRL_HUSKY_SN:-?}, not $name ($ROBOT_SERIAL): not launching. Type exit to leave."
    exec bash -i
fi
# ! A second launch would start a second driver and controller manager for each arm, and a second follower. One can
#   be left from a session whose link dropped (the robot keeps it until its next write fails), or another person's.
#   Anchored at python3: this script's own shell has the launch line in its command line too.
running=\$(pgrep -fa '^[^ ]*python3[^ ]* [^ ]*ros2 launch crl_husky')
if [ -n "\$running" ]; then
    echo "real lab: a crl_husky launch already runs on $name, so not starting another:"
    echo "\$running" | sed 's/^/    /'
    echo "  If it is hers and works (Part 3 OK), keep it: type exit and leave this pane. To restart it: Ctrl-C it"
    echo "  in its own terminal, or here  kill -INT <pid>  and wait until  pgrep -fa 'ros2 launch'  lists nothing;"
    echo "  then exit and start robot.sh $name again (Up, Enter)."
    exec bash -i
fi
echo "real lab: $launch"
exec $launch
EOF
)
echo "real lab: $name, administrator@$ROBOT_IP (asks for her password unless this laptop's key is on her, manual 1.e)"
# * -t: Ctrl-C here reaches the launch. The keepalives give up only after 5 min of silence, so a short WiFi drop
#   does not end the session (and with it the launch).
ssh -t -o ServerAliveInterval=10 -o ServerAliveCountMax=30 "administrator@$ROBOT_IP" "bash -ic $(printf %q "$remote")" \
    || true
echo "real lab: the session on $name ended. A launch in it stops with it, unless the link dropped: then it may still"
echo "  run on her, and robot.sh says so. Up, Enter starts robot.sh again."
