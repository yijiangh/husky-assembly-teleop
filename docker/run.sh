#!/bin/bash
# Run the husky-assembly-teleop container

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Detect OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "Detected Linux - using host networking for ROS2 DDS"
    # Allow X11 forwarding
    xhost +local:docker 2>/dev/null || true
    docker compose run --rm --network host husky-teleop-dev
    xhost -local:docker 2>/dev/null || true
elif [[ "$OSTYPE" == "darwin"* ]]; then
    echo "Detected macOS - using bridge networking"
    echo "NOTE: For GUI, start XQuartz and allow network connections"
    export DISPLAY=host.docker.internal:0
    docker compose run --rm husky-teleop-dev
else
    echo "Detected Windows/other - using bridge networking"
    docker compose run --rm husky-teleop-windows
fi
