#!/usr/bin/env python3
"""Fix the Robotiq grip force and speed on a husky: apply upstream PR #83 by hand.

Run ON THE HUSKY (copy it over first). It patches the lab's ros2_robotiq_gripper
checkout (2ff8545) so the gripper_speed_multiplier / gripper_force_multiplier
parameters are actually read and scaled, and sets the force multiplier to 0.25
(about 75 N instead of the full 235 N). Both files are backed up next to
themselves as *.bak-20260909, and nothing is written unless every anchor
matches exactly once -- so it is safe to run twice (the second run aborts).

    scp scripts/husky/patch_robotiq_driver.py administrator@192.168.131.1:/tmp/
    ssh administrator@192.168.131.1 'python3 /tmp/patch_robotiq_driver.py'
    ssh administrator@192.168.131.1 'source /opt/ros/humble/setup.bash && cd ~/workspace \\
        && colcon build --packages-select robotiq_driver'

then relaunch the gripper stacks (recipe in doc/rtde_network_setup.md). Why, and
what the three bugs are, is written up in that doc under "Known issue".

! The gripper_speed_multiplier stays at the xacro's 1.0, i.e. FULL speed once the
! fix is in (the grippers crawled at 15 % before). Lower it in the xacro if that
! turns out too brisk for the parts.
"""

import shutil
import sys

CPP = ('/home/administrator/workspace/src/ros2_robotiq_gripper/'
       'robotiq_driver/src/hardware_interface.cpp')
XACRO = ('/home/administrator/workspace/src/ros2_robotiq_gripper/'
         'robotiq_description/urdf/2f_85.ros2_control.xacro')
SUFFIX = '.bak-20260909'

# Each pair is (exact text as it stands in the file, replacement). The anchors
# are the lines read off the husky on 2026-09-09; a checkout that differs will
# simply refuse to patch.
EDITS_CPP = [
    ('''  gripper_speed_ = info_.hardware_parameters.count("gripper_speed_multiplier") ?
                       info_.hardware_parameters.count("gripper_speed_multiplier") :
                       1.0;''',
     '''  // Read the multiplier's VALUE (the old code used count(), i.e. always 1).
  gripper_speed_ = kGripperMaxSpeed * (info_.hardware_parameters.count("gripper_speed_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_speed_multiplier")) :
                                           1.0);'''),
    ('''  gripper_force_ = info_.hardware_parameters.count("gripper_force_multiplier") ?
                       info_.hardware_parameters.count("gripper_force_multiplier") :
                       1.0;''',
     '''  gripper_force_ = kGripperMaxforce * (info_.hardware_parameters.count("gripper_force_multiplier") ?
                                           std::stod(info_.hardware_parameters.at("gripper_force_multiplier")) :
                                           1.0);'''),
    ('''  gripper_speed_ = kGripperMaxSpeed * std::clamp(fabs(gripper_speed_) / kGripperMaxSpeed, 0.0, 1.0);
  write_speed_.store(uint8_t(gripper_speed_ * 0xFF));
  gripper_force_ = kGripperMaxforce * std::clamp(fabs(gripper_force_) / kGripperMaxforce, 0.0, 1.0);
  write_force_.store(uint8_t(gripper_force_ * 0xFF));''',
     '''  // Scale against the constant maxima WITHOUT writing back into the interface
  // storage: the old code folded the constant into gripper_speed_/gripper_force_
  // and pinned the registers at 38 (15 % speed) and 255 (full force) forever.
  const auto speed_fraction = std::clamp(fabs(gripper_speed_) / kGripperMaxSpeed, 0.0, 1.0);
  write_speed_.store(uint8_t(speed_fraction * 0xFF));
  const auto force_fraction = std::clamp(fabs(gripper_force_) / kGripperMaxforce, 0.0, 1.0);
  write_force_.store(uint8_t(force_fraction * 0xFF));'''),
]
# 0.25 of the 20..235 N range is ~74 N: plenty for a 0.75 kg seat and a 10-30 N
# insertion press through rubber pads, gentle on a wooden leg. Tune with a pull test.
EDITS_XACRO = [
    ('<param name="gripper_force_multiplier">0.5</param>',
     '<param name="gripper_force_multiplier">0.25</param>'),
]


def patch(path: str, edits: list):
    """Apply the edits to one file, or abort without touching it.

    Args:
        path (str): The file to patch.
        edits (list): (old, new) pairs; every `old` must occur exactly once.
    """
    text = open(path).read()
    for old, new in edits:
        n = text.count(old)
        if n != 1:
            sys.exit(f'ABORT, nothing written: anchor found {n}x in {path}:\n'
                     f'{old.splitlines()[0]} ...')
        text = text.replace(old, new)
    shutil.copy(path, path + SUFFIX)
    open(path, 'w').write(text)
    print(f'patched {path}\n   backup {path}{SUFFIX}')


if __name__ == '__main__':
    patch(CPP, EDITS_CPP)
    patch(XACRO, EDITS_XACRO)
    print('done -- now: source /opt/ros/humble/setup.bash && cd ~/workspace && '
          'colcon build --packages-select robotiq_driver, then relaunch the gripper stacks')
