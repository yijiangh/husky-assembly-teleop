# Fake lab: the ROS settings every fake-lab terminal uses. Sourced by router.sh, relay.sh, sim.sh and monitor.sh, in a
# clean shell.
#
# ! Everything stays on this laptop: ROS domain 70 over Zenoh, through a router on the loopback only; never the robots'
#   domains (80, 84-86).
# ! The monitor's Drive root is a scratch folder (~/husky_fake_lab/drive): runs/ never reach the real Drive.

#: This checkout and its workspace: scripts/fake_lab/ -> the repo -> src/ -> the workspace, unless FAKE_LAB_WS names
#: one (for a checkout outside a workspace, e.g. a worktree).
FAKE_LAB_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FAKE_LAB_WS="${FAKE_LAB_WS:-$(cd "$FAKE_LAB_REPO/../.." && pwd)}"
#: Scratch data: the Drive copy (drive/) and the monitor logs (logs/).
FAKE_LAB_HOME="${FAKE_LAB_HOME:-$HOME/husky_fake_lab}"
#: The process the test uses, relative to the Drive root.
FAKE_LAB_PROCESS="data_design_study/261008_2ModulesDemo_purged_process"
export FAKE_LAB_REPO FAKE_LAB_WS FAKE_LAB_HOME FAKE_LAB_PROCESS

# * ROS: rmw_zenoh, as in the lab, in a domain of our own. The lab's nodes meet through Zenoh routers (one per robot,
#   one per PC); here every node meets through the fake lab's single router, router.sh.
#   FAKE_LAB_DOMAIN picks another domain, e.g. for a second fake lab beside yours, with its own router; never a robot's.
export ROS_DOMAIN_ID="${FAKE_LAB_DOMAIN:-70}" RMW_IMPLEMENTATION=rmw_zenoh_cpp
case "$ROS_DOMAIN_ID" in
    80|84|85|86) echo "fake lab: domain $ROS_DOMAIN_ID belongs to the robots; refusing" >&2; return 1 ;;
esac
# ! The router listens on the loopback only, on a port of its own (7400 + the domain, so 7470; the lab's routers use
#   7447), and connects to nothing. Every node connects only to it, with multicast discovery off. So no robot can
#   reach this test, and it reaches no robot.
export FAKE_LAB_ZENOH_PORT=$((7400 + ROS_DOMAIN_ID))
export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"tcp/127.0.0.1:$FAKE_LAB_ZENOH_PORT\"];scouting/multicast/enabled=false"

# * The workspace: its venv, ROS, then its install (crl-husky's simulators, this package).
source "$FAKE_LAB_WS/venv/bin/activate"
source /opt/ros/humble/setup.bash
# ? Only to run without the apt install below: a folder holding the unpacked ros-humble-ur-robot-driver packages.
if [ -n "$FAKE_LAB_UR_PREFIX" ]; then
    export AMENT_PREFIX_PATH="$FAKE_LAB_UR_PREFIX:$AMENT_PREFIX_PATH"
    export CMAKE_PREFIX_PATH="$FAKE_LAB_UR_PREFIX:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$FAKE_LAB_UR_PREFIX/lib:$LD_LIBRARY_PATH" PATH="$FAKE_LAB_UR_PREFIX/bin:$PATH"
    export PYTHONPATH="$FAKE_LAB_UR_PREFIX/local/lib/python3.10/dist-packages:$PYTHONPATH"
fi
source "$FAKE_LAB_WS/install/setup.bash"
# ? Only for a checkout the workspace does not install (e.g. a worktree): an install of this package alone, built from
#   it with --symlink-install, over the workspace's.
if [ -n "$FAKE_LAB_OVERLAY" ]; then
    source "$FAKE_LAB_OVERLAY/local_setup.bash"
fi
# * The core and tamp of this checkout's pins, ahead of the venv's (those of the workspace's checkout).
for package in scaffolding_assembly_core husky_assembly_tamp; do
    if [ -d "$FAKE_LAB_REPO/external/$package/$package" ]; then
        export PYTHONPATH="$FAKE_LAB_REPO/external/$package:$PYTHONPATH"
    fi
done
export HUSKY_DRIVE_ROOT="$FAKE_LAB_HOME/drive"

# ! Every fake-lab node runs on rmw_zenoh (the lab PC needs it too: README, One-time setup).
if [ ! -x /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd ]; then
    echo "fake lab: rmw_zenoh is missing; install it once with:" \
        "sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor" >&2
    return 1
fi
# ! The simulated arms run on the UR driver's fake hardware.
if ! ros2 pkg prefix ur_robot_driver > /dev/null 2>&1; then
    echo "fake lab: the UR driver is missing; install it once with: sudo apt install ros-humble-ur-robot-driver" >&2
    return 1
fi
# ! The relay, the fake Motive and the fake wrench are this package's: an install built before they were added, or
#   from another branch, lacks them and the launch dies at the first missing one.
for script in husky_sim_motive husky_sim_wrench; do
    if [ ! -x "$(ros2 pkg prefix husky_assembly_teleop 2>/dev/null)/lib/husky_assembly_teleop/$script" ]; then
        echo "fake lab: the installed husky_assembly_teleop has no $script; build the workspace from this checkout" \
            "(manual, 0.4), or set FAKE_LAB_OVERLAY" >&2
        return 1
    fi
done
