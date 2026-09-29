# UR arm frames

**Rule:** in our URDFs, each arm's `<arm>_base_link` must be the frame the
robot's own controller calls `base_link`. The robots run the stock
`ur_description`, where the joints below it are fixed:

| Joint | Origin |
|---|---|
| `<arm>_base_link` → `<arm>_base_link_inertia` | `xyz 0 0 0`, `rpy 0 0 π` |
| `<arm>_base_link` → `<arm>_base` | `xyz 0 0 0`, `rpy 0 0 π` |

Calibration goes elsewhere: how the arm sits on the husky into the mount joint
*above* `<arm>_base_link` (`arm_mount_joint`, `right_arm_mount_joint`, ...),
the UR's own calibration into the six revolute joints.

**Why:** the original Alice/Belle URDFs had the 90° mount turn in the two
joints above instead (Cindy's right arm in its `base` joint). The arm was still
drawn in the right place, but `ur_arm_base_link` was 90° off the controller's
`base_link`. Cartesian commands computed in it swung 0804 (e-stop, 2026-09-29).

**Now:**
- The reported TCP (UR Base frame) is turned into the controller's frame by the
  stock 180° (`robot_interface/ur_frames.py`), never through our URDF. Targets
  reach that frame from the husky frame through the URDF's fixed mount chain,
  which is only right because `<arm>_base_link` is stock (the rule above).
- `config.py` uses the fixed `*_StockUrFrames.urdf` files, written by
  `scripts/fix_ur_base_frames.py` (every link stays where it was). Run it again
  after a calibration rewrites the originals.
- A URDF that breaks the rule still *looks* right in the viewer but moves wrong.
  So it is caught twice: `test/test_ur_frames.py` fails for every configured
  URDF that breaks it (`colcon test`), and at startup the monitor logs an error
  and refuses that arm's Cartesian targets (Hold still works).

**Chain from the world to the TCP** (`robot_interface/frames.py`):

```
world ─(mocap)─► base_footprint ─(URDF fixed)─► base_link [husky] ─(URDF fixed, mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0
```

The Cartesian sliders are in the husky's `base_link`. `arm.to_husky` /
`arm.from_husky` and `robot.to_world` / `robot.from_world` convert along it.
