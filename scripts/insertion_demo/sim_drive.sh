#!/bin/bash
# Insertion demo dry run: copy the demo's process from your Drive into the dry run's scratch Drive root
# (~/husky_insertion_demo_sim/drive), the only Drive the simulated monitor opens. Reads the real Drive, writes only
# the copy. sim_tmux.sh runs it before it opens the session.
#
# Usage: sim_drive.sh

set -euo pipefail
DEMO_HOME="${DEMO_HOME:-$HOME/husky_insertion_demo_sim}"
process="data_design_study/261104_SRD_insertion/process"

# The real Drive: DEMO_DRIVE if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder.
real="${DEMO_DRIVE:-}"
if [ -z "$real" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                     "$HOME/Insync/2025-03 Husky Assembly"; do
        [ -d "$candidate" ] && real="$candidate" && break
    done
fi
if [ ! -f "$real/$process/process.json" ]; then
    echo "sim_drive.sh: no $process/process.json under '$real'; set DEMO_DRIVE to your Drive folder" >&2
    exit 1
fi
if [ "$(cd "$real" && pwd)" = "$(mkdir -p "$DEMO_HOME/drive" && cd "$DEMO_HOME/drive" && pwd)" ]; then
    echo "sim_drive.sh: the scratch root is the real Drive; refusing" >&2
    exit 1
fi

copy="$DEMO_HOME/drive/$process"
mkdir -p "$copy" "$DEMO_HOME/logs"
# The saved carry (solutions/, planned offline on this copy) is not on the real Drive: keep it.
rsync -a --delete --exclude=/solutions/ "$real/$process/" "$copy/"
echo "sim_drive.sh: $copy is ready ($(du -sh "$copy" | cut -f1))"
