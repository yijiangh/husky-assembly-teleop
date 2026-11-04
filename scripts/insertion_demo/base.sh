#!/bin/bash
# Insertion demo: Cindy's base published standing at the origin (husky_fixed_base), until Ctrl-C. The monitor takes
# a base only from the mocap topic; with no Motive in the hall, this is where the demo assumes her: the process is
# authored with her at the origin, the jig on her base.
#
# ! Never drive her base while this runs: the pose never changes. Never with the mocap relay running.
#
# Usage: base.sh

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean DEMO_ "$here" '
    if ! demo_cindy_up; then
        echo "insertion demo: no Zenoh router at $DEMO_CINDY_IP:7447: is the cable in, this laptop on her subnet" \
            "(manual 1.3), and her launch running?" >&2
        exit 1
    fi
    echo "insertion demo: rigid body $DEMO_BODY (Cindy) fixed at the origin, ROS domain $ROS_DOMAIN_ID via $DEMO_CINDY_IP"
    exec ros2 run husky_assembly_teleop husky_fixed_base --ros-args -p bodies:="[$DEMO_BODY]"
'
