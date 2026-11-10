# The real lab's and the fake lab's shared parts (scripts/real_lab/, scripts/fake_lab/): sourced by their scripts, and
# in the clean shell each starts (lab_exec_clean). Only definitions: sourcing it changes no setting. Each lab's own
# settings are in its env.sh.

#: The monitor's plugins, in this order: each after what it needs. Both labs run the same ones.
LAB_MONITOR_PLUGINS="[assembly_run, base_planner_panel, assembly_steps, assembly_guide, robot_control, arm_planner_panel, obstacles, \
mocap_streams, doc, stop_recovery]"

# Set the array lab_settings to the settings starting with $1 (REAL_LAB_ or FAKE_LAB_) that are set, as name=value.
lab_collect_settings() {
    local name
    lab_settings=()
    for name in $(compgen -v "$1"); do
        lab_settings+=("$name=${!name}")
    done
}

# Replace this shell by a clean one that runs the bash script $3 under set -euo pipefail, after this file and the
# lab's env.sh. Its $0 is the lab's folder ($2), its $1... the arguments after $3. Only HOME, USER, TERM, LANG, a
# plain PATH and the lab's settings that are set (starting with $1) pass; --display first passes DISPLAY and
# XAUTHORITY too.
# ! A clean environment: your shell's ROS settings (another domain, Cyclone, the other lab) never leak in.
lab_exec_clean() {
    local display=()
    if [ "$1" = --display ]; then
        display=(DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}")
        shift
    fi
    local prefix=$1 folder=$2 script=$3
    shift 3
    lab_collect_settings "$prefix"
    # shellcheck disable=SC2016  # expanded by the clean shell
    exec env -i HOME="$HOME" USER="$USER" PATH=/usr/local/bin:/usr/bin:/bin TERM="${TERM:-xterm}" \
        LANG="${LANG:-C.UTF-8}" "${display[@]}" "${lab_settings[@]}" \
        bash --noprofile --norc -c 'set -euo pipefail
source "$0/../lab_common.sh" && source "$0/env.sh" || exit 1
'"$script" "$folder" "$@"
}

# The monitor with Alice, Belle and Cindy and LAB_MONITOR_PLUGINS, until Ctrl-C; the arguments are more ROS
# arguments (-p <name>:=<value>).
# ! Workaround for a bug: compas_fab's stdout redirect closes stdout for a moment around every model it loads; in the
#   monitor, other threads open files at the same time, and the planner then fails with "OSError(16, 'Device or
#   resource busy')". compas_fab skips that redirect when pytest is imported, so this starts the monitor with pytest
#   imported. Side effect: PyBullet's own messages print in the terminal.
lab_monitor() {
    python3 -u -c "
import pytest  # the workaround above: compas_fab then never redirects stdout
import sys
from husky_assembly_teleop.monitor import main
sys.argv[0] = \"husky_monitor\"
main()" \
        --ros-args \
        -p robots:="[alice, belle, cindy]" \
        -p plugins:="$LAB_MONITOR_PLUGINS" \
        "$@"
}

# * ------------------------------------------------------------- tmux sessions

# Give tmux session $1 the settings starting with $2 that are set: a pane split off later gets them too.
lab_tmux_set_environment() {
    local setting
    lab_collect_settings "$2"
    for setting in "${lab_settings[@]}"; do
        tmux set-environment -t "$1" "${setting%%=*}" "${setting#*=}"
    done
}

# Show each pane's name in its top border, in the format $2 (with #{@name}), for session $1; then pairs of a pane and
# its name. The name is a pane option: a shell's prompt or ssh would overwrite a pane title.
lab_tmux_name_panes() {
    tmux set-option -t "$1" pane-border-status top
    tmux set-option -t "$1" pane-border-format "$2"
    shift 2
    while [ $# -ge 2 ]; do
        tmux set-option -p -t "$1" @name "$2"
        shift 2
    done
}

# Wait until the interactive shell in each of the panes reads its first command line, so that what is then typed in
# shows after its prompt: its line editor has switched the terminal out of canonical mode (stty: -icanon). Gives up
# after 30 s (a shell without a line editor), and the keys go in anyway.
lab_tmux_wait_for_shells() {
    local pane tty waiting deadline=$((SECONDS + 30))
    while [ "$SECONDS" -lt "$deadline" ]; do
        waiting=""
        for pane in "$@"; do
            tty=$(tmux display-message -p -t "$pane" '#{pane_tty}')
            case "$(stty -F "$tty" -a 2>/dev/null)" in
                *-icanon*) ;;
                *) waiting=yes ;;
            esac
        done
        [ -z "$waiting" ] && return 0
        sleep 0.05
    done
    echo "tmux: a pane's shell did not show its prompt within 30 s; typing in anyway" >&2
}

# Attach to tmux session $1, or say how to switch to it from inside tmux.
lab_tmux_attach() {
    if [ -z "${TMUX:-}" ]; then
        exec tmux attach -t "=$1"
    fi
    echo "already inside tmux: switch with  tmux switch-client -t =$1"
}
