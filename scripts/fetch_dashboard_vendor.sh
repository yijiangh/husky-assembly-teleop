#!/usr/bin/env bash
# Download the dashboard's javascript libraries once.
#
# They are deliberately NOT committed (see .gitignore): they are large, they are
# third-party, and this script pins the exact versions so any machine can
# reproduce them. Run it once per checkout, before starting the server.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR="$HERE/husky_assembly_teleop/dashboard/static/vendor"
THREE="https://cdn.jsdelivr.net/npm/three@0.185.0"
PLOTLY="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@3.0.1/plotly.min.js"

mkdir -p "$VENDOR/three-r185/build" \
         "$VENDOR/three-r185/examples/jsm/controls" \
         "$VENDOR/three-r185/examples/jsm/loaders" \
         "$VENDOR/three-r185/examples/jsm/utils"

fetch () {  # fetch <url> <destination>
  echo "  $2"
  curl -fsSL "$1" -o "$VENDOR/$2"
}

echo "downloading the dashboard's javascript into $VENDOR"
# three.module.min.js imports three.core.min.js relatively, and GLTFLoader
# imports BOTH BufferGeometryUtils and SkeletonUtils relatively -- miss one and
# the whole module graph fails to load, with nothing but a console error.
fetch "$THREE/build/three.module.min.js"                     "three-r185/build/three.module.min.js"
fetch "$THREE/build/three.core.min.js"                       "three-r185/build/three.core.min.js"
fetch "$THREE/examples/jsm/controls/OrbitControls.js"        "three-r185/examples/jsm/controls/OrbitControls.js"
fetch "$THREE/examples/jsm/loaders/GLTFLoader.js"            "three-r185/examples/jsm/loaders/GLTFLoader.js"
fetch "$THREE/examples/jsm/utils/BufferGeometryUtils.js"     "three-r185/examples/jsm/utils/BufferGeometryUtils.js"
fetch "$THREE/examples/jsm/utils/SkeletonUtils.js"           "three-r185/examples/jsm/utils/SkeletonUtils.js"
fetch "$PLOTLY"                                              "plotly.min.js"
echo "done."
