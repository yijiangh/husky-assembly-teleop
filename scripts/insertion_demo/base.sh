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
    demo_need_router || exit 1
    echo "insertion demo: rigid body $DEMO_BODY (Cindy) fixed at the origin, ROS domain $ROS_DOMAIN_ID via $DEMO_ROUTER"
    exec ros2 run husky_assembly_teleop husky_fixed_base --ros-args -p bodies:="[$DEMO_BODY]"
'
