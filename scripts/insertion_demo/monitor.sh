#!/bin/bash
# Insertion demo: the monitor with Cindy and the demo's plugins (insertion_demo, robot_control), until Ctrl-C (press
# it once and wait for the final holds). It fills in the demo's process folder (press Load in the Assembly tab).
# Its terminal output is also saved to ~/husky_insertion_demo/logs/monitor_<date>.log.
#
# ! It refuses to start while another monitor runs in domain 80: two monitors would both command the robot.
# ! Workaround for a bug (scripts/lab_common.sh, lab_monitor): the monitor starts with pytest imported, so compas_fab
#   never redirects stdout. Side effect: PyBullet's own messages print in this terminal.
#
# Usage: monitor.sh

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"
DEMO_HOME="${DEMO_HOME:-$HOME/husky_insertion_demo}"
mkdir -p "$DEMO_HOME/logs"
log="$DEMO_HOME/logs/monitor_$(date +%Y%m%d_%H%M%S).log"

# * In a clean environment (lab_common.sh): it exits with the monitor's exit status (pipefail; tee's is 0).
# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean DEMO_ "$here" '
    if ! demo_cindy_up; then
        echo "insertion demo: no Zenoh router at $DEMO_CINDY_IP:7447: is the cable in, this laptop on her subnet" \
            "(manual 1.3), and her launch running?" >&2
        exit 1
    fi
    if [ ! -f "$HUSKY_DRIVE_ROOT/$DEMO_PROCESS/process.json" ]; then
        echo "insertion demo: no $DEMO_PROCESS/process.json under the Drive root" \
            "${HUSKY_DRIVE_ROOT:-(none found)}; set DEMO_DRIVE to your Drive folder" >&2
        exit 1
    fi
    # ! Out of any checkout: python -c imports from the current folder first (an old husky_assembly_teleop).
    cd "$DEMO_HOME"
    # ! Fails closed: without an answer from ROS, nobody knows whether another monitor runs.
    if ! nodes=$(timeout -s KILL 15 ros2 node list --no-daemon --spin-time 4 2>/dev/null); then
        echo "insertion demo: ROS did not say which nodes run in domain 80 (ros2 node list failed): is Cindy" \
            "launched (manual 2.2)?" >&2
        exit 1
    fi
    if grep -qx /husky_monitor <<< "$nodes"; then
        echo "insertion demo: a monitor already runs in ROS domain 80: stop it first" >&2
        exit 1
    fi
    echo "insertion demo: monitor on ROS domain $ROS_DOMAIN_ID via $DEMO_CINDY_IP, Drive root $HUSKY_DRIVE_ROOT"
    echo "insertion demo: open http://localhost:8080; log: $1"
    python3 -u -c "
import pytest  # the workaround above: compas_fab then never redirects stdout
import sys
from husky_assembly_teleop.monitor import main
sys.argv[0] = \"husky_monitor\"
main()" \
        --ros-args -p robots:="[cindy]" -p plugins:="[insertion_demo, robot_control]" \
        -p process_directory:="$DEMO_PROCESS" -p drive_root:="$HUSKY_DRIVE_ROOT" 2>&1 | tee -i "$1"
' "$log"
