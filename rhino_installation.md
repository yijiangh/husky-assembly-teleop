The recent rhino 8 support native python, but we need to use the interpreter used by rhino to install `compas_fab`.
```
cd external\compas_fab
C:\Users\<USERNAME>\.rhinocode\py39-rh8\python.exe -m pip install pybullet_planning
C:\Users\<USERNAME>\.rhinocode\py39-rh8\python.exe -m pip install -e .
```
Replace USERNAME to your local folder name. If such a folder does not exists, please type _ScriptEditor command in rhino 8 to initialize the rhino-python environment.

# Getting started with the keyframe interface in Grasshopper
1. Open Rhino 8 and select a new file with meter unit.
2. Type Grasshopper command in the command line to open the Grasshopper editor.
3. Open the `scripts\keyframe_gh_interface\RobotX_demo_keyframe_simplified.gh` file in grasshopper.
4. Follow the step x comments in the file.
5. Run `husky_assembly_teleop\design_interface\reconstruct_state_from_json.py` to reconstruct the robot cell and cell state from the saved json files.

# Tips for Rhino/GH if you are new
Camera moving in Rhino:
- Hold shift and right click the mouse to move the camera.
- Right click to rotate.

For Grasshopper:
- Double click the canvas to invoke the search bar.
- Control shift and drag can move the wire connections.
- Hover over the mouse to component to see the description of the component.
- All python components can be opened by double clicking the component.