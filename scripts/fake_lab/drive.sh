#!/bin/bash
# Fake lab: copy the test's process folder from your Drive into the scratch Drive root (~/husky_fake_lab/drive), so
# the monitor writes its runs/ there and never into the real Drive. Reads the real Drive, writes only the copy.
#
# Usage: drive.sh           copy the process files (a runs/ folder already in the copy is kept)
#        drive.sh --fresh   also delete the copy's runs/, for a test from the very start

set -e
FAKE_LAB_HOME="${FAKE_LAB_HOME:-$HOME/husky_fake_lab}"
process="data_design_study/261008_2ModulesDemo_purged_process"

# The real Drive: $HUSKY_DRIVE_ROOT if set, else the Insync copy of the shared "2025-03 Husky Assembly" folder.
real="${HUSKY_DRIVE_ROOT:-}"
if [ -z "$real" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly"; do
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
if [ "$1" = "--fresh" ]; then
    rm -rf "$copy/runs"
    echo "drive.sh: deleted the copy's runs/"
fi
echo "drive.sh: $copy is ready ($(du -sh "$copy" | cut -f1))"
