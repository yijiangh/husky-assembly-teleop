#!/bin/bash
# Real lab: the tmux session "real_lab" for the test with the real robots, that you and Claude share; then attach to it.
#
#   ┌──────────────────────────────┬──────────────┐
#   │ monitor                      │ alice        │
#   │                              ├──────────────┤
#   ├──────────────────────────────┤ belle        │
#   │ checks                       ├──────────────┤
#   ├──────────────────────────────┤ cindy        │
#   │ relay                        │              │
#   └──────────────────────────────┴──────────────┘
#
# - "relay": the mocap relay (relay.sh), started at once.
# - "alice", "belle", "cindy": robot.sh <robot>, typed in but not started: press Enter once her pendants are in Remote.
# - "checks": this lab's ROS settings (env.sh) loaded, check_robots.sh typed in.
# - "monitor": monitor.sh, typed in: press Enter once the checks pass.
# Each pane's name is in its top border, the scripts are on its PATH (Up, Enter runs one again), and the relay's and
# the robots' panes are logged to ~/husky_real_lab/logs/. Detach with Ctrl-b d: everything keeps running. Running this
# again only re-attaches.
#
# Claude reads the panes (tmux capture-pane) and never types into them.

set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
session="${REAL_LAB_SESSION:-real_lab}"
logs="${REAL_LAB_HOME:-$HOME/husky_real_lab}/logs"
# * The scripts on the panes' PATH: tmux gives a new pane the PATH of the command that made it.
export PATH="$here:$PATH"
# * The panes' shells come from the tmux server, not from this shell: pass the real-lab settings that are set.
environment=()
for name in ${!REAL_LAB_*}; do
    environment+=(-e "$name=${!name}")
done

# A pane's start command: its name in bold and a hint, then an interactive shell.
start() {
    printf "printf '\\\\033[1m%%s\\\\033[0m  %%s\\\\n\\\\n' %q %q; exec bash" "$1" "$2"
}

if ! tmux has-session -t "=$session" 2>/dev/null; then
    mkdir -p "$logs"
    stamp=$(date +%Y%m%d_%H%M%S)
    hint="Press Enter once the checks pass (manual 4.a). Ctrl-C once stops it."
    # * Said in the pane: attaching clears this terminal. Its monitor would hold port 8080.
    if tmux has-session -t =fake_lab 2>/dev/null; then
        hint="! The fake lab still runs: stop it first (manual 1.a). Then: $hint"
    fi
    monitor=$(tmux new-session -d -s "$session" -n lab -x 220 -y 60 "${environment[@]}" -P -F '#{pane_id}' \
        "$(start monitor "$hint")")
    tmux set-option -t "$session" history-limit 50000
    # * A pane you add later (Ctrl-b ") gets them too.
    tmux set-environment -t "$session" PATH "$PATH"
    for name in ${!REAL_LAB_*}; do
        tmux set-environment -t "$session" "$name" "${!name}"
    done
    alice=$(tmux split-window -h -l 40% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start alice "Her pendant in Remote (manual 2.a), then Enter: logs in and starts her launch.")")
    belle=$(tmux split-window -v -l 67% -t "$alice" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start belle "Her pendant in Remote (manual 2.a), then Enter. She stays parked.")")
    cindy=$(tmux split-window -v -l 50% -t "$belle" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start cindy "Both pendants in Remote (manual 2.a), then Enter. Never Ctrl-C while she holds a bar.")")
    checks=$(tmux split-window -v -l 45% -t "$monitor" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start checks "Domain 80 on Zenoh, as the robots. Enter: the read-only checks (manual Part 3).")")
    relay=$(tmux split-window -v -l 8 -t "$checks" "${environment[@]}" -P -F '#{pane_id}' \
        "$(start relay "The mocap relay. Never stop it while a base drives.")")
    # * Each pane's name in its border, as a pane option: a shell's prompt or ssh would overwrite a pane title.
    tmux set-option -t "$session" pane-border-status top
    tmux set-option -t "$session" pane-border-format " #{@name} "
    for pane in monitor checks relay alice belle cindy; do
        tmux set-option -p -t "${!pane}" @name "$pane"
    done
    for pane in relay alice belle cindy; do
        tmux pipe-pane -o -t "${!pane}" "cat >> '$logs/${pane}_$stamp.log'"
    done
    sleep 1  # the shells' prompts first, so what is typed in shows after them
    tmux send-keys -t "$relay" "relay.sh" Enter
    tmux send-keys -t "$checks" "source env.sh" Enter "check_robots.sh"
    for robot in alice belle cindy; do
        tmux send-keys -t "${!robot}" "robot.sh $robot"
    done
    tmux send-keys -t "$monitor" "monitor.sh"
    tmux select-pane -t "$monitor"
fi
if [ -z "$TMUX" ]; then
    exec tmux attach -t "=$session"
else
    echo "already inside tmux: switch with  tmux switch-client -t =$session"
fi
