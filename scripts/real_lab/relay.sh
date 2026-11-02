#!/bin/bash
# Real lab: the mocap relay until Ctrl-C (this checkout's launch/mocap.launch.py): Motive's rigid bodies on ROS, each
# robot's as its calibrated base pose, for the monitor and the robots' followers alike (doc/lab/mocap_relay.md).
#
# ! One relay per ROS domain: it refuses to start while another one runs in domain 80 (another PC's); then use that one.
# ! Stopping it stops mocap for everyone: never while a base drives.
#
# Usage: relay.sh [<argument>:=<value> ...]    more arguments for mocap.launch.py

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# * In a clean environment (lab_common.sh).
# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean REAL_LAB_ "$here" '
    if ! real_lab_router_up; then
        echo "real lab: no Zenoh router on this laptop (127.0.0.1:7447): install it once (manual 1.c)" >&2
        exit 1
    fi
    mkdir -p "$REAL_LAB_HOME" && cd "$REAL_LAB_HOME"  # out of any checkout
    echo "real lab: mocap relay on ROS domain $ROS_DOMAIN_ID, Motive $REAL_LAB_MOTIVE," \
        "calibrations from ${HUSKY_DRIVE_ROOT:-crl-husky only (no Drive found)}"
    exec ros2 launch "$REAL_LAB_REPO/launch/mocap.launch.py" server_ip:="$REAL_LAB_MOTIVE" \
        drive_root:="$HUSKY_DRIVE_ROOT" "$@"
' "$@"
