# ROS2 Humble on Ubuntu 22.04
FROM ros:humble-ros-base-jammy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Python and pip
    python3-pip \
    python3-venv \
    # tracikpy dependencies
    libeigen3-dev \
    liborocos-kdl-dev \
    libkdl-parser-dev \
    liburdfdom-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    # Build tools
    build-essential \
    cmake \
    git \
    swig \
    # ROS2 build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # GUI support (for pybullet visualization)
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxrender1 \
    libxext6 \
    x11-apps \
    # Useful tools
    vim \
    curl \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws/src

# Copy the entire project (including submodules)
COPY . husky_assembly_teleop/

# Create and configure Python virtual environment
# Using --system-site-packages to access system ROS2 Python packages and colcon
RUN python3 -m venv /ros2_ws/venv --system-site-packages

# Activate venv and install Python dependencies
# Note: The venv activation is done inline since each RUN is a new shell
RUN /bin/bash -c "source /ros2_ws/venv/bin/activate && \
    python3 -m pip install --upgrade pip wheel && \
    python3 -m pip install 'setuptools==68.2.2' 'packaging>=24.2'"

# Install external dependencies from submodules (editable mode)
RUN /bin/bash -c "source /ros2_ws/venv/bin/activate && \
    cd /ros2_ws/src/husky_assembly_teleop/external/compas_fab && \
    python3 -m pip install -e . && \
    cd /ros2_ws/src/husky_assembly_teleop/external/pybullet_planning && \
    python3 -m pip install -e . && \
    cd /ros2_ws/src/husky_assembly_teleop/external/tracikpy && \
    python3 -m pip install -e ."

# Install additional requirements
RUN /bin/bash -c "source /ros2_ws/venv/bin/activate && \
    python3 -m pip install \
    'numpy<1.25.0' \
    'kdtree==0.16' \
    'matplotlib==3.10.3' \
    pybullet \
    roslibpy"

# Build the ROS2 workspace
WORKDIR /ros2_ws
RUN /bin/bash -c "source /ros2_ws/venv/bin/activate && \
    source /opt/ros/humble/setup.bash && \
    python3 -m colcon build --symlink-install"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /ros2_ws/venv/bin/activate\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /ros2_ws/entrypoint.sh && chmod +x /ros2_ws/entrypoint.sh

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
