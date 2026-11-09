#!/bin/bash
# Insertion demo dry run: the tmux session "insertion_demo_sim", the demo's own base.sh and monitor.sh against a
# simulated Cindy on this laptop instead of the cable; then attach to it (doc/lab/insertion_demo_sim_manual.md).
#
#   ┌──────────────────────────────┬──────────────┐
#   │ monitor                      │ sim          │
#   │                              │              │
#   ├──────────────────────────────┼──────────────┤
#   │ checks                       │ base         │
#   │                              ├──────────────┤
#   │                              │ router       │
#   └──────────────────────────────┴──────────────┘
#
# - "router": the dry run's own Zenoh router, on the loopback only (fake_lab/router.sh), started at once. It stands in
#   for Cindy's router.
# - "sim": the simulated Cindy with her arms and scaffolding tools, no Motive (fake_lab/sim.sh robot:=0806), started
#   at once. It stands in for her launch.
# - "base": base.sh, typed in: press Enter once the sim pane goes quiet.
# - "monitor": monitor.sh, typed in: press Enter once the base pane runs.
# - "checks": the dry run's ROS settings (env.sh) loaded; `ros2 node list` typed in.
# The demo's process is copied into the scratch Drive first (sim_drive.sh). Each pane's name is in its top border, the
# scripts are on its PATH, and the panes are logged to ~/husky_insertion_demo_sim/logs/. Detach with Ctrl-b d:
# everything keeps running. Running this again only re-attaches. Stop everything with:
# tmux kill-session -t insertion_demo_sim
#
# Usage: sim_tmux.sh    DEMO_SIM_DOMAIN (71), DEMO_PORT (8080), DEMO_HOME and DEMO_SESSION pick others, for a second
#                       dry run beside this one

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"
# ! Everything this session starts is the dry run: no pane reaches the robot.
export DEMO_SIM=1
source "$here/env.sh" || exit 1
session="${DEMO_SESSION:-insertion_demo_sim}"
logs="$DEMO_HOME/logs"
fake_lab="$DEMO_REPO/scripts/fake_lab"
export PATH="$here:$PATH"
# * The panes' shells come from the tmux server, not from this shell: pass the dry run's settings, and the two the
#   fake lab's router.sh and sim.sh read.
lab_collect_settings DEMO_
environment=(-e "FAKE_LAB_DOMAIN=$FAKE_LAB_DOMAIN" -e "FAKE_LAB_HOME=$FAKE_LAB_HOME")
for setting in "${lab_settings[@]}"; do
    environment+=(-e "$setting")
done

# A pane's start command: its name in bold and a hint, then an interactive shell.
start() {
    printf "printf '\\\\033[1m%%s\\\\033[0m  %%s\\\\n\\\\n' %q %q; exec bash" "$1" "$2"
}

if ! tmux has-session -t "=$session" 2>/dev/null; then
    "$here/sim_drive.sh"
    mkdir -p "$logs"
    stamp=$(date +%Y%m%d_%H%M%S)
    monitor=$(tmux new-session -d -s "$session" -n demo -x 220 -y 60 "${environment[@]}" -P -F '#{pane_id}' \
        "$(start monitor "Press Enter once base.sh runs (manual 1.2). Ctrl-C once stops it.")")
    tmux set-option -t "$session" history-limit 50000
    tmux set-environment -t "$session" PATH "$PATH"
    lab_tmux_set_environment "$session" DEMO_
    tmux set-environment -t "$session" FAKE_LAB_DOMAIN "$FAKE_LAB_DOMAIN"
    tmux set-environment -t "$session" FAKE_LAB_HOME "$FAKE_LAB_HOME"
    sim=$(tmux split-window -h -l 40% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start sim "The simulated Cindy, in place of her launch. Ctrl-C stops it.")")
    base=$(tmux split-window -v -l 50% -t "$sim" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start base "Cindy's base fixed at the origin. Enter once the sim pane goes quiet.")")
    router=$(tmux split-window -v -l 6 -t "$base" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start router "The dry run's own Zenoh router, on the loopback only.")")
    checks=$(tmux split-window -v -l 45% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start checks "ROS domain $ROS_DOMAIN_ID through the dry run's router. Enter: list the nodes.")")
    lab_tmux_name_panes "$session" " #{@name} " \
        "$monitor" monitor "$checks" checks "$sim" sim "$base" base "$router" router
    for pane in sim base router; do
        tmux pipe-pane -o -t "${!pane}" "cat >> '$logs/${pane}_$stamp.log'"
    done
    lab_tmux_wait_for_shells "$monitor" "$checks" "$sim" "$base" "$router"
    tmux send-keys -t "$router" "$fake_lab/router.sh" Enter
    tmux send-keys -t "$sim" "$fake_lab/sim.sh robot:=0806 fake_motive:=false" Enter
    tmux send-keys -t "$checks" "source env.sh" Enter "ros2 node list --no-daemon"
    tmux send-keys -t "$base" "base.sh"
    tmux send-keys -t "$monitor" "monitor.sh"
    tmux select-pane -t "$monitor"
fi
lab_tmux_attach "$session"
