"""Conventions of the UR arms every user of the core shares: joint names, and the arm links a mounted tool touches."""

from __future__ import annotations

#: Joint names in the UR driver's order. The URDF has the same names with the arm's prefix ("left_ur_arm_").
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

#: Links of its arm ("<arm>_<suffix>") a mounted tool may touch. The SRDFs don't list these pairs,
#: so collision checkers add them themselves.
TOOL_TOUCHES_ARM_LINKS = ("wrist_2_link", "wrist_3_link", "flange", "tool0")
