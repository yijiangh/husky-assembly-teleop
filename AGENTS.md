- minimize token, broken grammar ok as long as understandble.

- At the end of the plan mode, write a detailed specs and instructions as a log into a markdown file in `tasks/year-month-day_xxxx.md`.

Core principles:
- Simplicity first: make every change as simple as possible. Impact minimal code. Whenever possible, try to reuse existing functions without reinventing the wheels.
- Always write comments in code. Remember code is for both human and agents to read.

Use the following python env to run:

1. if ros package:

```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

2. if standalone script:

```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
python ...
```