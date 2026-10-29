#!/bin/bash
# Real lab: the monitor with Alice, Belle and Cindy and the fake lab's plugins, until Ctrl-C (press it once and wait
# for the final holds). It loads the test's process from the Drive and records the assembly run there.
# Its terminal output is also saved to ~/husky_real_lab/logs/monitor_<date>.log (tee ignores Ctrl-C, so the monitor's
# shutdown lines still reach it).
#
# ! It refuses to start while another monitor runs in domain 80 (another PC's, or this one's twice): two monitors would
#   both command the robots, and neither knows the other's motions. It checks once, at its start: a monitor started
#   later elsewhere goes unnoticed.
# ! Workaround for a bug: compas_fab's stdout redirect closes stdout for a moment around every model it loads; in the
#   monitor, other threads open files at the same time, and the planner then fails with "OSError(16, 'Device or
#   resource busy')". compas_fab skips that redirect when pytest is imported, so this starts the monitor with pytest
#   imported. Side effect: PyBullet's own messages print in this terminal.
#
# Usage: monitor.sh

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The real-lab settings that are set (REAL_LAB_*), for the clean environment.
settings=(); for name in ${!REAL_LAB_*}; do settings+=("$name=${!name}"); done
REAL_LAB_HOME="${REAL_LAB_HOME:-$HOME/husky_real_lab}"
mkdir -p "$REAL_LAB_HOME/logs"
log="$REAL_LAB_HOME/logs/monitor_$(date +%Y%m%d_%H%M%S).log"

# ! A clean environment: your shell's ROS settings (another domain, Cyclone, the fake lab) never leak in.
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
    LANG="${LANG:-C.UTF-8}" DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" \
    "${settings[@]}" \
    bash --noprofile --norc -c '
        source "$0/env.sh" || exit 1
        if ! real_lab_router_up; then
            echo "real lab: no Zenoh router on this laptop (127.0.0.1:7447): install it once (manual 1.c)" >&2
            exit 1
        fi
        if [ ! -f "$HUSKY_DRIVE_ROOT/$REAL_LAB_PROCESS/process.json" ]; then
            echo "real lab: no $REAL_LAB_PROCESS/process.json under the Drive root" \
                "${HUSKY_DRIVE_ROOT:-(none found)}; set REAL_LAB_DRIVE to your Drive folder" >&2
            exit 1
        fi
        # ! Out of any checkout: python -c imports from the current folder first (an old husky_assembly_teleop).
        cd "$REAL_LAB_HOME"
        # ! Fails closed: without an answer from ROS, nobody knows whether another monitor runs.
        if ! nodes=$(timeout -s KILL 15 ros2 node list --no-daemon --spin-time 4 2>/dev/null); then
            echo "real lab: ROS did not say which nodes run in domain 80 (ros2 node list failed), so not starting:" \
                "is the router linked to the robots? check_robots.sh says (manual Part 3)" >&2
            exit 1
        fi
        if grep -qx /husky_monitor <<< "$nodes"; then
            echo "real lab: a monitor already runs in ROS domain 80 (another PC, or another terminal here):" \
                "stop it first; two monitors would both command the robots" >&2
            exit 1
        fi
        # * The plugins of the fake lab, in its order: each after what it needs.
        plugins="[assembly_run, base_planner_panel, assembly_steps, robot_control, arm_planner_panel, obstacles]"
        address=$(ip -4 -o addr show | awk "/ 192\\.168\\.0\\./ {print \$4}" | cut -d/ -f1 | head -1)
        echo "real lab: monitor on ROS domain $ROS_DOMAIN_ID (Zenoh), Drive root $HUSKY_DRIVE_ROOT"
        echo "real lab: open http://localhost:8080 (phone: http://${address:-<this laptop>}:8081); log: $1"
        python3 -u -c "
import pytest  # the workaround above: compas_fab then never redirects stdout
import sys
from husky_assembly_teleop.monitor import main
sys.argv[0] = \"husky_monitor\"
main()" \
            --ros-args \
            -p robots:="[alice, belle, cindy]" \
            -p plugins:="$plugins" \
            -p process_directory:="$REAL_LAB_PROCESS" \
            -p drive_root:="$HUSKY_DRIVE_ROOT" 2>&1 | tee -i "$1"
    ' "$here" "$log"
