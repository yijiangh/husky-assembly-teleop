#!/bin/bash
# Fake lab: the tmux session "fake_lab" that you and Claude share, then attach to it.
#
# One window "lab" with four panes, each named in its top border:
# - "sim", top: the three simulated robots and the fake Motive (sim.sh), started at once;
# - "monitor", middle: the monitor (monitor.sh), typed in but not started. Press Enter once the robots are up (~20 s).
# - "router", bottom left, small: the Zenoh router that every node meets through (router.sh), started first;
# - "relay", bottom right, small: the mocap relay (relay.sh), started next.
#
# Claude reads the panes (tmux capture-pane) and never types into them unless you ask. Detach with Ctrl-b d; the
# session keeps running. Running this again only re-attaches. Stop everything with: tmux kill-session -t fake_lab

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../lab_common.sh"
session="${FAKE_LAB_SESSION:-fake_lab}"
# * The panes' shells come from the tmux server, not from this shell: pass the fake-lab settings that are set.
lab_collect_settings FAKE_LAB_
settings=""
for setting in "${lab_settings[@]}"; do
    settings+="${setting%%=*}=$(printf %q "${setting#*=}") "
done
if ! tmux has-session -t "=$session" 2>/dev/null; then
    tmux new-session -d -s "$session" -n lab -x 220 -y 60
    tmux set-option -t "$session" history-limit 50000
    # * A pane split off later gets them too, so `source env.sh` there finds this session's domain.
    lab_tmux_set_environment "$session" FAKE_LAB_
    tmux split-window -v -t "$session:lab"
    tmux split-window -v -l 5 -t "$session:lab.1"
    tmux split-window -h -t "$session:lab.2"
    lab_tmux_name_panes "$session" " #{pane_index} #{@name} " \
        "$session:lab.0" sim "$session:lab.1" monitor "$session:lab.2" router "$session:lab.3" relay
    lab_tmux_wait_for_shells "$session:lab.0" "$session:lab.1" "$session:lab.2" "$session:lab.3"
    tmux send-keys -t "$session:lab.2" "$settings$here/router.sh" Enter
    tmux send-keys -t "$session:lab.3" "$settings$here/relay.sh" Enter
    tmux send-keys -t "$session:lab.0" "$settings$here/sim.sh" Enter
    tmux send-keys -t "$session:lab.1" "$settings$here/monitor.sh"
fi
lab_tmux_attach "$session"
