# husky_assembly_teleop

This is a python package for controlling huskies in the mocap space.

# Installation

Install system package dependencies for [tracikpy](https://github.com/mjd3/tracikpy):
```
sudo apt-get install libeigen3-dev liborocos-kdl-dev libkdl-parser-dev liburdfdom-dev libnlopt-dev libnlopt-cxx-dev
```
If using a Mac, you can install the dependencies using [Homebrew](https://stackoverflow.com/questions/19688424/why-is-the-apt-get-function-not-working-in-the-terminal-on-mac-os-x-v10-9-maver).

Install python dependencies:
```
pip install -r requirements.txt
```

Build and run:
```
cd workspace
colcon build
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor
```

Changes to python scripts need a rebuild too! Use `colcon build && ros2 run husky_assembly_teleop husky_monitor` to run both commands at once. Alternatively, look into `colcon build --symlink-install`.


Read the [Mocap wiki](https://gitlab.inf.ethz.ch/crl/crl-wiki/-/wikis/HW/OptiTrack) for more information on how to create a rigid body in Motive and how to set the IP address of the OptiTrack server.

## Usage

### Initiate network connection
1. Turn on the power for the TPLink, the OptiTrack system (the Netgear router).
2. Connect your computer to the TPLink via an Ethernet cable.
3. Test your Ethernet connection to the Optitrack server computer by pinging the server's IP address. You can find this by running `ipconfig` in the command prompt on the server PC. Update `MOCAP_IP`'s value in the `husky_monitor.py` script.
4. Find our the IP address of your PC, and then confirm this by pinging it from the PC. Update `CLIENT_IP`'s value in the `husky_monitor.py` script.
5. Open the Motive software on the server PC.

### Register a new rigid body in Motive
6. Then, you will need to register a new rigid body by selecting a few markers on the Motive software, follow [the documentation](https://docs.optitrack.com/motive/rigid-body-tracking). Next, note the `Streaming ID` of the rigid body you just registered. You can find this by clicking on the rigid body in the `Assets` panel in Motive. \
\
Alternatively, activate an already existing rigid body in the `Assets` panel.

### Calibrate rigid body

Add the 3D model to the object in motive and move the pivot until the model aligns with the markers. If more precision is needed, use a probe to sample corners on the real object. These additional probe points can be used to improve the alignment of the model.

### Edit the python script to add your rigid body

Add a `TrackedObject(monitor, name, streaming_id, pos, rot, scale, model)` in `husky_world.py`. This object should now be automatically tracked using mocap.

### Tips 💡 
In pybullet's viewer, you can pan the camera by holding `alt` (or `ctrl`) and dragging the mouse. 
`alt + left` click to rotate the camera. 
`alt + right` click to zoom in and out. 
`alt + middle` click to move the camera up and down. 
You can also zoom in and out by scrolling the mouse wheel.

