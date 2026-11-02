#!/bin/bash
# Fake lab: the mocap relay, until Ctrl-C: this checkout's launch/mocap.launch.py, as on the lab PC. It publishes each
# simulated base (the fake Motive's pose, else husky_sim's truth) as the robot's mocap pose; without it the monitor
# sees no base.
#
# ! sim:=true: it reads only the simulation's inputs, never the lab's Motive, even on the mocap network. Calibrations
#   come from the scratch Drive (env.sh), which has none: crl-husky's defaults.
#
# Usage: relay.sh [<argument>:=<value> ...]    more arguments for mocap.launch.py

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"

# * In a clean environment (lab_common.sh).
# shellcheck disable=SC2016  # expanded by the clean shell
lab_exec_clean FAKE_LAB_ "$here" '
    mkdir -p "$FAKE_LAB_HOME" && cd "$FAKE_LAB_HOME"  # out of any checkout
    echo "fake lab: mocap relay on ROS domain $ROS_DOMAIN_ID, Zenoh router 127.0.0.1:$FAKE_LAB_ZENOH_PORT, no Motive"
    exec ros2 launch "$FAKE_LAB_REPO/launch/mocap.launch.py" sim:=true "$@"
' "$@"
