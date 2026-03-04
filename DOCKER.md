# Docker Setup for Husky Assembly Teleop

This guide explains how to run the husky-assembly-teleop ROS2 node in a Docker container on any machine (Windows, Linux, or Mac).

## Prerequisites

1. **Docker Desktop** installed and running
   - Windows: [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
   - Make sure WSL2 backend is enabled on Windows

2. **Git** with submodules initialized:
   ```bash
   git submodule update --init --recursive
   ```

3. **X Server** (for GUI/pybullet visualization):
   - **Windows**: Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [Xming](https://sourceforge.net/projects/xming/)
      - for VcXsrv, run `XLaunch` from the start menu and select "Multiple windows" → Next → Start no client → Disable access control → Finish
   - **Linux**: X11 is usually available by default
   - **Mac**: Install [XQuartz](https://www.xquartz.org/)

## Using Cursor Dev Containers (Recommended for Windows)

The easiest way to develop on Windows is using Cursor's Dev Containers feature:

1. **Ensure submodules are initialized first**:
   ```batch
   git submodule update --init --recursive
   ```

2. Open the project folder in Cursor

3. Press `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"

4. Wait for the container to build (first time takes a while)

5. Once inside, the terminal will have ROS2 and all dependencies ready:
   ```bash
   ros2 run husky_assembly_teleop husky_monitor
   ```

**Note**: The devcontainer configuration uses the Dockerfile directly (not docker-compose) to avoid Windows-specific networking issues.

## Quick Start (Manual Docker)

### Windows

1. **Build the image**:
   ```batch
   docker\build.bat
   ```
   Or using PowerShell/Command Prompt:
   ```batch
   docker compose build husky-teleop-dev
   ```

2. **Run the container**:
   ```batch
   docker\run.bat
   ```
   Or:
   ```batch
   docker compose run --rm husky-teleop-dev
   ```

### Linux/Mac

1. **Build the image**:
   ```bash
   chmod +x docker/*.sh
   ./docker/build.sh
   ```

2. **Run the container**:
   ```bash
   ./docker/run.sh
   ```

## Using VSCode/Cursor Dev Containers

For the best development experience with debugging support:

1. Install the **Dev Containers** extension in VSCode/Cursor
2. **Important**: Initialize submodules first: `git submodule update --init --recursive`
3. Open the project folder in VSCode/Cursor
4. Press `Ctrl+Shift+P` (or `F1`) → "Dev Containers: Reopen in Container"
5. The IDE will build and open the container automatically

### Debugging

Once inside the container, you can:
- Set breakpoints in Python files
- Use the launch configurations in `.vscode/launch.json`
- Run "Python: Husky Monitor" to debug the main node

### First-time Setup Inside Container

On first launch, the container will automatically:
1. Create the Python virtual environment
2. Install external dependencies (compas_fab, pybullet_planning, tracikpy)
3. Build the ROS2 workspace

If you see errors about missing packages, run manually:
```bash
source /ros2_ws/venv/bin/activate
pip install -e /ros2_ws/src/husky_assembly_teleop/external/compas_fab
pip install -e /ros2_ws/src/husky_assembly_teleop/external/pybullet_planning
pip install -e /ros2_ws/src/husky_assembly_teleop/external/tracikpy
cd /ros2_ws && python3 -m colcon build --symlink-install
```

## Inside the Container

Once inside the container, the environment is automatically set up:

```bash
# The venv and ROS2 workspace are sourced automatically

# Run the husky monitor node
ros2 run husky_assembly_teleop husky_monitor

# Or use the alias
husky

# Rebuild the workspace after code changes
cb  # alias for: cd /ros2_ws && python3 -m colcon build --symlink-install

# Rebuild a specific package
cbs husky_assembly_teleop
```

## GUI Support (pybullet visualization)

### Windows with VcXsrv

1. Launch **XLaunch** from Start Menu
2. Select "Multiple windows" → Next
3. Select "Start no client" → Next
4. **Important**: Check "Disable access control" → Next → Finish
5. Run the container with:
   ```batch
   set DISPLAY=host.docker.internal:0.0
   docker compose run --rm husky-teleop-dev
   ```

### Linux

```bash
xhost +local:docker
docker compose run --rm husky-teleop-dev
xhost -local:docker
```

### Mac with XQuartz

1. Start XQuartz
2. Go to Preferences → Security → Check "Allow connections from network clients"
3. Restart XQuartz
4. Run:
   ```bash
   xhost +localhost
   export DISPLAY=host.docker.internal:0
   docker compose run --rm husky-teleop-dev
   ```

## Network Configuration

The container uses `network_mode: host` to:
- Enable ROS2 DDS discovery with other nodes on the network
- Connect to the OptiTrack/Motive server

### Connecting to OptiTrack

Make sure:
1. Your host machine is connected to the same network as the OptiTrack server
2. Update `MOCAP_IP` and `CLIENT_IP` in `husky_monitor.py` as described in the main README

## Troubleshooting

### "No module named 'compas_fab'" or similar

The external dependencies need to be installed. Inside the container:
```bash
pip install -e /ros2_ws/src/husky_assembly_teleop/external/compas_fab
pip install -e /ros2_ws/src/husky_assembly_teleop/external/pybullet_planning
pip install -e /ros2_ws/src/husky_assembly_teleop/external/tracikpy
```

### Build fails with colcon

Clean and rebuild:
```bash
cd /ros2_ws
rm -rf build/ install/ log/
python3 -m colcon build --symlink-install
```

### Cannot connect to X server

- Verify your X server is running
- Check firewall settings allow connections
- On Windows, ensure VcXsrv was started with "Disable access control"

### ROS2 nodes can't communicate

- Verify `ROS_DOMAIN_ID` matches between nodes
- Check that `network_mode: host` is set in docker-compose.yml
- Ensure firewall allows UDP multicast traffic

## File Structure

```
husky-assembly-teleop/
├── Dockerfile              # Production image (Linux with host networking)
├── Dockerfile.dev          # Development image (cross-platform)
├── docker-compose.yml      # Container orchestration (multiple services)
├── .devcontainer/
│   └── devcontainer.json   # VSCode/Cursor Dev Container config
├── .vscode/
│   └── launch.json         # Debug configurations
├── docker/
│   ├── build.bat           # Windows build script
│   ├── build.sh            # Linux/Mac build script
│   ├── run.bat             # Windows run script
│   └── run.sh              # Linux/Mac run script (auto-detects OS)
├── .dockerignore           # Files excluded from Docker build
└── DOCKER.md               # This file
```

## Services in docker-compose.yml

| Service | Description | Use Case |
|---------|-------------|----------|
| `husky-teleop` | Production (Linux only) | Deploy on Ubuntu with host networking |
| `husky-teleop-dev` | Development (cross-platform) | General development |
| `husky-teleop-windows` | Windows-optimized | Windows Docker Desktop |
