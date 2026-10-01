"""`compas_fab.planning_group` picks the group whose base link is nearest the URDF root, as the export does."""

from __future__ import annotations

from pathlib import Path

from compas_fab.robots import RobotCell, RobotSemantics
from design_io_fixtures import write_robot_files

from husky_assembly_teleop.design_io.compas_fab import load_model, planning_group


def test_base_rooted_group_wins_below_a_world_link(tmp_path: Path):
    """With a `world` root above base_footprint (as in the husky URDFs), the base-rooted group still wins."""
    urdf, srdf = write_robot_files(tmp_path)
    urdf.write_text(urdf.read_text().replace(
        '<link name="base_footprint"/>',
        '<link name="world"/><link name="base_footprint"/>'
        '<joint name="world_joint" type="fixed"><parent link="world"/><child link="base_footprint"/></joint>'))
    # * The arm-only group comes first, as `Left arm` does in dual_arm_husky.srdf.
    srdf.write_text("""<?xml version="1.0"?>
<robot name="fake">
  <group name="arm_only"><chain base_link="left_link1" tip_link="left_tool0"/></group>
  <group name="base_left"><chain base_link="base_footprint" tip_link="left_tool0"/></group>
</robot>
""")
    model = load_model(urdf)
    cell = RobotCell(model, RobotSemantics.from_srdf_file(str(srdf), model))
    assert planning_group(cell, "left_tool0") == "base_left"
