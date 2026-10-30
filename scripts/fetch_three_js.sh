#!/usr/bin/env bash
# Download the three.js files the as-built page builder (scripts/as_built_page.py) carries into every page, once per
# checkout. They are not kept in git (see .gitignore): large, third-party, and pinned here so any machine gets the same.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="$HERE/data/bar_holding_acc_data/vendor/three-r185"
THREE="https://cdn.jsdelivr.net/npm/three@0.185.0"

mkdir -p "$VENDOR/build" "$VENDOR/examples/jsm/controls"

fetch () {  # fetch <url> <destination>
  echo "  $2"
  curl -fsSL "$1" -o "$VENDOR/$2"
}

echo "downloading three.js r185 into $VENDOR"
# three.module.min.js imports three.core.min.js relatively: both are needed, and OrbitControls for the mouse.
fetch "$THREE/build/three.module.min.js"              "build/three.module.min.js"
fetch "$THREE/build/three.core.min.js"                "build/three.core.min.js"
fetch "$THREE/examples/jsm/controls/OrbitControls.js" "examples/jsm/controls/OrbitControls.js"
echo "done."
