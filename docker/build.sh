#!/bin/bash
# Build the Docker image for husky-assembly-teleop

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "Updating git submodules..."
git submodule update --init --recursive

echo "Building husky-assembly-teleop Docker image..."
docker compose build husky-teleop-dev

echo ""
echo "Build complete!"
echo ""
echo "To start the container, run: ./docker/run.sh"
