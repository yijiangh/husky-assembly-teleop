#!/bin/bash
# Fake lab: the monitor against the simulated robots, until Ctrl-C (press it once and wait for the final holds).
# Its terminal output is also saved to ~/husky_fake_lab/logs/monitor_<date>.log (tee ignores Ctrl-C, so the
# monitor's shutdown lines still reach it).
#
# Usage: monitor.sh [viser_port]    default $FAKE_LAB_PORT, else 8080 (the phone page is the port + 1)
#
# ! Workaround for a bug, test only: compas_fab's stdout redirect closes stdout for a moment around every model it
#   loads; in the monitor, other threads open files at the same time, and the planner then fails with
#   "OSError(16, 'Device or resource busy')". compas_fab skips that redirect when pytest is imported, so this starts
#   the monitor with pytest imported. Side effect: PyBullet's own messages print in this terminal.
# * The measured lab obstacles (`obstacles`, data/lab/main.json) are loaded: the process's own Rhino obstacles count
#   in collision checks only while the process panel's "Rhino obstacles collide" box is ticked (off at start).
# * The plugins' folders follow the order of `plugins` (each after the plugins it needs): base_planner_panel comes
#   after assembly_run and before assembly_steps, so the main column reads as the manual runs: process, run, base,
#   then the steps.

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The fake-lab settings that are set (FAKE_LAB_*), for the clean environment.
settings=(); for name in ${!FAKE_LAB_*}; do settings+=("$name=${!name}"); done
port="${1:-${FAKE_LAB_PORT:-8080}}"
FAKE_LAB_HOME="${FAKE_LAB_HOME:-$HOME/husky_fake_lab}"
mkdir -p "$FAKE_LAB_HOME/logs"
log="$FAKE_LAB_HOME/logs/monitor_$(date +%Y%m%d_%H%M%S).log"

# ! A clean environment: your shell's ROS settings (domain, middleware, the lab workspace) never leak in.
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
    LANG="${LANG:-C.UTF-8}" DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" \
    "${settings[@]}" \
    bash --noprofile --norc -c '
        source "$0/env.sh" || exit 1
        if [ ! -f "$HUSKY_DRIVE_ROOT/$FAKE_LAB_PROCESS/process.json" ]; then
            echo "fake lab: no process copy yet; run $0/drive.sh first" >&2
            exit 1
        fi
        # ! Out of any checkout: python -c imports from the current folder first (an old husky_assembly_teleop).
        cd "$FAKE_LAB_HOME"
        echo "fake lab: monitor on ROS domain $ROS_DOMAIN_ID (Zenoh router 127.0.0.1:$FAKE_LAB_ZENOH_PORT)," \
            "Drive root $HUSKY_DRIVE_ROOT"
        echo "fake lab: open http://localhost:$1 (phone page: port $(($1 + 1))); log: $2"
        python3 -u -c "
import pytest  # the workaround above: compas_fab then never redirects stdout
import sys
from husky_assembly_teleop.monitor import main
sys.argv[0] = \"husky_monitor\"
main()" \
            --ros-args \
            -p robots:="[alice, belle, cindy]" \
            -p plugins:="[assembly_run, base_planner_panel, assembly_steps, robot_control, arm_planner_panel, obstacles]" \
            -p process_directory:="$FAKE_LAB_PROCESS" \
            -p drive_root:="$HUSKY_DRIVE_ROOT" \
            -p viser_port:=$1 2>&1 | tee -i "$2"
    ' "$here" "$port" "$log"
