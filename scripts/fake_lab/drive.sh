#!/bin/bash
# Fake lab: copy the test's process folder from your Drive into the scratch Drive root (~/husky_fake_lab/drive), so
# the monitor writes its runs/ there and never into the real Drive. Reads the real Drive, writes only the copy.
#
# Usage: drive.sh           copy the process files (a runs/ folder already in the copy is kept)
#        drive.sh --fresh   also delete the copy's runs/, for a test from the very start
#
# FAKE_LAB_PROCESS names another process than the first three bars' (relative to the Drive root), as for env.sh.
# FAKE_LAB_BENCH names a benchmark data root (the folder holding prm/): its roadmaps and saved carries of the process's
# bars are copied too, to where the monitor looks for them (data_experiment/cdfm_planner_benchmark/prm/ in the copy).

set -euo pipefail
FAKE_LAB_HOME="${FAKE_LAB_HOME:-$HOME/husky_fake_lab}"
process="${FAKE_LAB_PROCESS:-data_design_study/261008_2ModulesDemo_purged_process}"

# The real Drive: $HUSKY_DRIVE_ROOT if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder, under
# the account folder (the laptop) or straight under ~/Insync (the RobotX PC), as scripts/real_lab/env.sh finds it.
real="${HUSKY_DRIVE_ROOT:-}"
if [ -z "$real" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                     "$HOME/Insync/2025-03 Husky Assembly"; do
        [ -d "$candidate" ] && real="$candidate" && break
    done
fi
if [ ! -f "$real/$process/process.json" ]; then
    echo "drive.sh: no $process/process.json under '$real'; set HUSKY_DRIVE_ROOT to your Drive folder" >&2
    exit 1
fi
if [ "$(cd "$real" && pwd)" = "$(mkdir -p "$FAKE_LAB_HOME/drive" && cd "$FAKE_LAB_HOME/drive" && pwd)" ]; then
    echo "drive.sh: the scratch root is the real Drive; refusing" >&2
    exit 1
fi

copy="$FAKE_LAB_HOME/drive/$process"
mkdir -p "$copy" "$FAKE_LAB_HOME/logs"
rsync -a --exclude runs --exclude solutions "$real/$process/" "$copy/"
if [ -n "${FAKE_LAB_BENCH:-}" ]; then
    # * Only what a plan reads: the roadmap, its settings and tracking, the saved carry and loading poses. The
    #   benchmark's scene files, queries and pages stay behind.
    roadmaps="$FAKE_LAB_HOME/drive/data_experiment/cdfm_planner_benchmark/prm"
    mkdir -p "$roadmaps"
    found=0
    for folder in "$FAKE_LAB_BENCH/prm/$(basename "$process")"__*/; do
        [ -f "$folder/roadmap.json.gz" ] || continue
        rsync -a --exclude scenes --exclude queries --exclude dashboard "$folder" "$roadmaps/$(basename "$folder")/"
        found=$((found + 1))
    done
    echo "drive.sh: $found roadmap folder(s) copied to $roadmaps"
fi
if [ "${1:-}" = "--fresh" ]; then
    rm -rf "$copy/runs"
    echo "drive.sh: deleted the copy's runs/"
fi
echo "drive.sh: $copy is ready ($(du -sh "$copy" | cut -f1))"
