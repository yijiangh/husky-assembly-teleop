#!/bin/bash
# Run a command in the jg-sync sidecar: ros2_ws_next venv + ROS + its install, the sidecar and the core sync first on
# PYTHONPATH, a local-only ROS domain (never the lab robots), and the Drive root (read only).
cd /home/yijiangh/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
export HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
W=/home/yijiangh/Code/ros2_ws_next/dev/jg-sync
cd $W
export PYTHONPATH=$W:$W/external/scaffolding_assembly_core:$PYTHONPATH PYTHONDONTWRITEBYTECODE=1
unset CYCLONEDDS_URI
export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
exec "$@"
