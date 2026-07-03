Core principles:
- Simplicity first: make every change as simple as possible. Impact minimal code. Whenever possible, try to reuse existing functions without reinventing the wheels.

# Comments
- write comments whenever you can, the code is for humans to review. Avoid code jargon like no-op, make it plain and easy to understand.
- write google-style docstring for all functions. In the function definition, use type (e.g., fn(a: float)) whenever possible.
- whenever possible, put import at the top of a python file. Also use "from xxx import fn" instead of "import xxx; xxx.fn()".
- I use Better Comments plugin for VScode, use "*, !, ?" etc to highlight important parts of the comments and divide big sections to help me read.

# Run env

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