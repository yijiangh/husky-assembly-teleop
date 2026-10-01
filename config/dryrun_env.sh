# Environment for the FAKE_HARDWARE dry run (doc/support_robot_test_manual.md, Part A).
# SOURCE it, do not execute it:
#     source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 86     # 86 Cindy | 84 Alice | 85 Belle
#
# * What it sets up:
#   - the ros2_ws venv and the workspace install;
#   - CycloneDDS on LOOPBACK only (config/cyclonedds_localhost.xml): works with the robot
#     USB-ethernet adapter unplugged, and nothing can reach a real robot;
#   - the scratch copy of the design problem in ~/husky_dryrun (progress.json and the
#     .live-solved.json files are written there, not into the shared drive);
#   - the calibration folder on this workstation;
#   - the gradient IK backend (ssik is not installed in this venv).
# ! Use a fresh terminal for real-robot runs: this overrides the robot CycloneDDS
# ! config that ~/.bashrc sets.

_DRYRUN_REPO="$HOME/Code/ros2_ws/src/husky-assembly-teleop"
_DRYRUN_GD="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"

cd "$HOME/Code/ros2_ws" || return
source venv/bin/activate
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_DRYRUN_REPO/config/cyclonedds_localhost.xml"
export ROS_DOMAIN_ID="${1:-86}"

export DESIGN_DATA_DIRECTORY="$HOME/husky_dryrun/design"
export DESIGN_PROBLEM_NAME=260920_dryrun
export EXPERIMENT_DATA_DIRECTORY="$_DRYRUN_GD/data_experiment"
export HUSKY_IK_BACKEND=gradient
# ! Start the monitor WITHOUT a pipe (no `| tee`): with its output piped, the monitor's printed
# ! lines stop reaching the terminal partway through start-up. To keep a log, use tmux instead:
# !   tmux pipe-pane -o 'cat >> ~/husky_dryrun/monitor_pane.log'

# An old ros2 CLI daemon may still be using the robot network config.
ros2 daemon stop >/dev/null 2>&1

echo "[dryrun] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (loopback only) | problem $DESIGN_DATA_DIRECTORY/$DESIGN_PROBLEM_NAME"
[ -f "$DESIGN_DATA_DIRECTORY/$DESIGN_PROBLEM_NAME/ActionSchedule.json" ] \
    || echo "[dryrun] ! no ActionSchedule.json there yet -- make the scratch copy first (manual, section A2)"
