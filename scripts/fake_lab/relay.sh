#!/bin/bash
# Fake lab: the mocap relay, until Ctrl-C: this checkout's launch/mocap.launch.py, as on the lab PC. It publishes each
# simulated base (the fake Motive's pose, else husky_sim's truth) as the robot's mocap pose; without it the monitor
# sees no base.
#
# ! sim:=true: it reads only the simulation's inputs, never the lab's Motive, even on the mocap network. Calibrations
#   come from the scratch Drive (env.sh), which has none: crl-husky's defaults.
#
# Usage: relay.sh [<argument>:=<value> ...]    more arguments for mocap.launch.py

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The fake-lab settings that are set (FAKE_LAB_*), for the clean environment.
settings=(); for name in ${!FAKE_LAB_*}; do settings+=("$name=${!name}"); done

# ! A clean environment: your shell's ROS settings (domain, middleware, the lab workspace) never leak in.
exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
    LANG="${LANG:-C.UTF-8}" "${settings[@]}" \
    bash --noprofile --norc -c '
        source "$0/env.sh" || exit 1
        mkdir -p "$FAKE_LAB_HOME" && cd "$FAKE_LAB_HOME"  # out of any checkout
        echo "fake lab: mocap relay on ROS domain $ROS_DOMAIN_ID, Zenoh router 127.0.0.1:$FAKE_LAB_ZENOH_PORT, no Motive"
        exec ros2 launch "$FAKE_LAB_REPO/launch/mocap.launch.py" sim:=true "${@:1}"
    ' "$here" "$@"
