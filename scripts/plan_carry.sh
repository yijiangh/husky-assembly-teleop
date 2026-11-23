#!/bin/bash
# Plan one bar's carry again on its roadmap and keep it as the bar's saved carry (carry.json): a loading posture the
# base can drive in with from the robot's station, and the carry from it to the insert start. The roadmap is kept.
#
# Usage: plan_carry.sh <bar> [all|horizontal|vertical|back] [more options of tamp's plan_process_carry.py]
#   the class of loading pose: the bar across the front (horizontal), upright in front (vertical), along the robot
#   over its back (back); all (default) lets the planner choose.
#
# Needs HUSKY_BENCH_ROOT: the folder holding prm/<process>__<bar>/ (the roadmaps). tamp also writes its scene files
# and the plan there, so use a local copy, not the Drive.
# CARRY_PROCESS names the process, relative to the Drive root (default: the two-module demo); CARRY_ROBOT the robot
# whose station in data/lab/main.json the drive in is checked from (default 0806); CARRY_WORKERS the worker processes.
#
# ! Reads the Drive, never writes it. The new carry.json is copied to the fake lab's scratch Drive when that has the
#   bar's roadmap; for the real Drive the copy command is printed, for you to run.
# In the monitor: the guide's travel to load reads the file again at its next Prepare.

set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
workspace="${FAKE_LAB_WS:-$(cd "$repo/../.." && pwd)}"
if [ $# -lt 1 ]; then
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//' >&2
    exit 1
fi
bar="${1#bars/}"
shift
class=all
if [ $# -gt 0 ] && [[ "$1" != -* ]]; then
    class="$1"
    shift
fi
process="${CARRY_PROCESS:-data_design_study/261117_2ModulesDemo_order_bracing}"
robot="${CARRY_ROBOT:-0806}"
if [ -z "${HUSKY_BENCH_ROOT:-}" ]; then
    echo "plan_carry.sh: set HUSKY_BENCH_ROOT to the local folder holding prm/<process>__<bar>/" >&2
    exit 1
fi

# The real Drive: $HUSKY_DRIVE_ROOT if set, else the Insync copy, as scripts/fake_lab/drive.sh finds it.
real="${HUSKY_DRIVE_ROOT:-}"
if [ -z "$real" ]; then
    for candidate in "$HOME"/Insync/*/"Google Drive - Shared with me/2025-03 Husky Assembly" \
                     "$HOME/Insync/2025-03 Husky Assembly"; do
        [ -d "$candidate" ] && real="$candidate" && break
    done
fi
if [ ! -f "$real/$process/process.json" ]; then
    echo "plan_carry.sh: no $process/process.json under '$real'; set HUSKY_DRIVE_ROOT to your Drive folder" >&2
    exit 1
fi
name="$(basename "$process")"
roadmap="$HUSKY_BENCH_ROOT/prm/${name}__$bar"
if [ ! -f "$roadmap/roadmap.json.gz" ]; then
    echo "plan_carry.sh: no roadmap of $bar at $roadmap" >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$workspace/venv/bin/activate"
station="$(python3 -c '
import json, sys
stations = json.load(open(sys.argv[1])).get("stations", {})
if sys.argv[2] not in stations:
    sys.exit(f"plan_carry.sh: no station of {sys.argv[2]} in {sys.argv[1]}")
print(*stations[sys.argv[2]])' "$repo/data/lab/main.json" "$robot")"
echo "plan_carry.sh: $bar of $name, loading class $class, station ($station) of $robot"

cd "$repo/external/husky_assembly_tamp"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
before="$(stat -c %Y "$roadmap/carry.json" 2>/dev/null || echo 0)"
# shellcheck disable=SC2086  # the station is three words
python3 scripts/plan_process_carry.py "$real/$process" --bar "$bar" --cdfm-backend prm --goal frames \
    --prm-workers "${CARRY_WORKERS:-8}" --cdfm-home-anchor "$class" --station $station --save-carry --no-plot \
    --save-dir "$HUSKY_BENCH_ROOT/prm/plans/$name/replanned" --save-tag "$class" "$@" || true
if [ "$(stat -c %Y "$roadmap/carry.json" 2>/dev/null || echo 0)" = "$before" ]; then
    echo "plan_carry.sh: no new carry: the saved carry of $bar is as it was" >&2
    exit 1
fi

inside="data_experiment/cdfm_planner_benchmark/prm/${name}__$bar"
scratch="${FAKE_LAB_HOME:-$HOME/husky_fake_lab}/drive/$inside"
if [ -d "$scratch" ] && [ ! "$roadmap" -ef "$scratch" ]; then
    cp "$roadmap/carry.json" "$scratch/carry.json"
    echo "plan_carry.sh: copied to the fake lab: $scratch/carry.json"
fi
echo "plan_carry.sh: for the real Drive, run:"
echo "  cp '$roadmap/carry.json' '$real/$inside/carry.json'"
