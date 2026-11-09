#!/bin/bash
# Insertion demo: Cindy's base published standing at the origin (husky_fixed_base), until Ctrl-C. The monitor takes
# a base only from the mocap topic; with no Motive in the hall, this is where the demo assumes her: the process is
# authored with her at the origin, the jig on her base.
#
# ! Never drive her base while this runs: the pose never changes. Never with the mocap relay running: it refuses to
#   start while anything else publishes her body's pose (in the lab her router is joined to the lab's over WiFi, so a
#   relay on a lab PC counts).
#
# Usage: base.sh

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean DEMO_ "$here" '
    demo_need_router || exit 1
    # ! One source per body, and fails closed: only "no publisher" or "no such topic yet" lets it start.
    topic="/mocap/rigid_body/id_$DEMO_BODY/pose"
    info=$(timeout -s KILL 15 ros2 topic info --no-daemon --spin-time 4 "$topic" 2>&1 || true)
    if ! grep -q "Publisher count: 0\|Unknown topic" <<< "$info"; then
        echo "insertion demo: not starting: could not tell that nobody else publishes $topic (the mocap relay," \
            "or another base.sh): ${info:-ROS did not answer}" >&2
        exit 1
    fi
    echo "insertion demo: rigid body $DEMO_BODY (Cindy) fixed at the origin, ROS domain $ROS_DOMAIN_ID via $DEMO_ROUTER"
    exec ros2 run husky_assembly_teleop husky_fixed_base --ros-args -p bodies:="[$DEMO_BODY]"
'
