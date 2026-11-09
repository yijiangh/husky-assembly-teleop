#!/bin/bash
# Insertion demo: the tmux session "insertion_demo" for the demo with Cindy over the cable; then attach to it.
#
#   ┌──────────────────────────────┬──────────────┐
#   │ monitor                      │ cindy        │
#   │                              │              │
#   ├──────────────────────────────┼──────────────┤
#   │ checks                       │ base         │
#   └──────────────────────────────┴──────────────┘
#
# - "cindy": ssh to her over the cable, typed in: press Enter, then start her launch there (manual 2.2).
# - "base": base.sh, typed in: press Enter once her launch runs; the monitor then sees her base at the origin.
# - "checks": this demo's ROS settings (env.sh) loaded; `ros2 node list` typed in.
# - "monitor": monitor.sh, typed in: press Enter once the base pane runs.
# Each pane's name is in its top border, the scripts are on its PATH, and the panes are logged to
# ~/husky_insertion_demo/logs/. Detach with Ctrl-b d: everything keeps running. Running this again only re-attaches.

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"
source "$here/env.sh" || exit 1
session="${DEMO_SESSION:-insertion_demo}"
logs="$DEMO_HOME/logs"
export PATH="$here:$PATH"
lab_collect_settings DEMO_
# ! Never the dry run here: a tmux server started by sim_tmux.sh hands its DEMO_SIM to every later pane, and env.sh
#   would then take these panes for the dry run. Emptied, env.sh reads them as unset.
environment=(-e "DEMO_SIM=" -e "DEMO_SIM_DOMAIN=")
for setting in "${lab_settings[@]}"; do
    environment+=(-e "$setting")
done

# A pane's start command: its name in bold and a hint, then an interactive shell.
start() {
    printf "printf '\\\\033[1m%%s\\\\033[0m  %%s\\\\n\\\\n' %q %q; exec bash" "$1" "$2"
}

if ! tmux has-session -t "=$session" 2>/dev/null; then
    mkdir -p "$logs"
    stamp=$(date +%Y%m%d_%H%M%S)
    monitor=$(tmux new-session -d -s "$session" -n demo -x 220 -y 60 "${environment[@]}" -P -F '#{pane_id}' \
        "$(start monitor "Press Enter once base.sh runs (manual 2.3). Ctrl-C once stops it.")")
    tmux set-option -t "$session" history-limit 50000
    tmux set-environment -t "$session" PATH "$PATH"
    lab_tmux_set_environment "$session" DEMO_
    cindy=$(tmux split-window -h -l 40% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start cindy "Both pendants in Remote, then Enter: logs in over the cable; start her launch there (manual 2.2).")")
    base=$(tmux split-window -v -l 50% -t "$cindy" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start base "Cindy's base fixed at the origin. Enter once her launch runs. Never drive her base.")")
    checks=$(tmux split-window -v -l 45% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start checks "Domain 80 on Zenoh through Cindy's router. Enter: list her nodes.")")
    lab_tmux_name_panes "$session" " #{@name} " "$monitor" monitor "$checks" checks "$cindy" cindy "$base" base
    for pane in cindy base; do
        tmux pipe-pane -o -t "${!pane}" "cat >> '$logs/${pane}_$stamp.log'"
    done
    lab_tmux_wait_for_shells "$monitor" "$checks" "$cindy" "$base"
    # * --no-daemon: a ROS daemon left from another terminal answers with what it saw under its own settings.
    tmux send-keys -t "$checks" "source env.sh" Enter "ros2 node list --no-daemon"
    tmux send-keys -t "$cindy" "ssh -o ConnectTimeout=5 -o ServerAliveInterval=10 administrator@$DEMO_CINDY_IP"
    tmux send-keys -t "$base" "base.sh"
    tmux send-keys -t "$monitor" "monitor.sh"
    tmux select-pane -t "$monitor"
fi
lab_tmux_attach "$session"
