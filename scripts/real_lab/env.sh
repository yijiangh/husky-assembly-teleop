# Real lab: the settings every real-lab script and terminal uses: the three robots, ROS on Zenoh in the robots' domain
# 80, this workspace and the Drive. Sourced by robot.sh, relay.sh, monitor.sh and check_robots.sh, and in the session's
# checks pane.
#
# ! This reaches the real robots: a node started with it talks to them through this laptop's Zenoh router (crl-husky's
#   crl-zenoh-router service, manual 1.c). The fake lab (scripts/fake_lab/) has settings of its own.

#: This checkout and its workspace: scripts/real_lab/ -> the repo -> src/ -> the workspace.
REAL_LAB_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
REAL_LAB_WS="$(cd "$REAL_LAB_REPO/../.." && pwd)"
#: The logs of the monitor and the panes.
REAL_LAB_HOME="${REAL_LAB_HOME:-$HOME/husky_real_lab}"
#: The process the test uses, relative to the Drive root.
REAL_LAB_PROCESS="data_design_study/261008_2ModulesDemo_purged_process"
#: The Motive PC.
REAL_LAB_MOTIVE="${REAL_LAB_MOTIVE:-192.168.0.28}"
# * The Drive: REAL_LAB_DRIVE if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder. The monitor
#   records its run in the process folder there, and the relay reads the base calibrations from it.
if [ -z "$REAL_LAB_DRIVE" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                     "$HOME/Insync/2025-03 Husky Assembly"; do
        [ -d "$candidate" ] && REAL_LAB_DRIVE="$candidate" && break
    done
fi
export REAL_LAB_REPO REAL_LAB_WS REAL_LAB_HOME REAL_LAB_PROCESS REAL_LAB_MOTIVE REAL_LAB_DRIVE
export HUSKY_DRIVE_ROOT="$REAL_LAB_DRIVE"

# Set ROBOT_SERIAL, ROBOT_IP, ROBOT_NS, ROBOT_ARMS (their ROS namespaces), ROBOT_BODY (mocap rigid body) and
# ROBOT_GRIPPER (the onboard launch's) for robot $1: alice, belle or cindy. Any other name returns 1.
real_lab_robot() {
    case "$1" in
        alice) ROBOT_SERIAL=0804 ROBOT_IP=192.168.0.113 ROBOT_BODY=1840 ROBOT_ARMS="ur5e"
               ROBOT_GRIPPER=robotiq_2F_85 ;;
        belle) ROBOT_SERIAL=0805 ROBOT_IP=192.168.0.114 ROBOT_BODY=1850 ROBOT_ARMS="ur5e"
               ROBOT_GRIPPER=robotiq_2F_85 ;;
        cindy) ROBOT_SERIAL=0806 ROBOT_IP=192.168.0.115 ROBOT_BODY=1860 ROBOT_ARMS="left_ur5e right_ur5e"
               ROBOT_GRIPPER=scaffolding_v3 ;;
        *) return 1 ;;
    esac
    ROBOT_NS="/a200_$ROBOT_SERIAL"
}

# Whether this laptop's Zenoh router listens: a node started without one waits for it, silently.
real_lab_router_up() {
    ss -Htln 2>/dev/null | awk '{print $4}' | grep -qx '127\.0\.0\.1:7447'
}

# * ROS: rmw_zenoh in the robots' domain, as every robot and control PC in the lab (crl-husky README, Zenoh). Nodes
#   connect to this laptop's router on localhost:7447, rmw_zenoh's default.
export ROS_DOMAIN_ID=80 RMW_IMPLEMENTATION=rmw_zenoh_cpp
unset ZENOH_CONFIG_OVERRIDE ZENOH_SESSION_CONFIG_URI

# * The workspace: its venv, ROS, then its install (this package, crl-husky).
source "$REAL_LAB_WS/venv/bin/activate"
source /opt/ros/humble/setup.bash
source "$REAL_LAB_WS/install/setup.bash"
