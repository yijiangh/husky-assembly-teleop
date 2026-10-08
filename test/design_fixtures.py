"""Shared builders for the design tests: tiny robot files and a small valid two-robot design."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from bar_assembly_core.design import (Action, Attached, BodySpec, Design, Movement, RobotSpec, RobotState, State,
                                      Target, Writer)
from bar_assembly_core.geometry import BoxShape, CylinderShape, Geometry, Pose, TriMesh
from bar_assembly_core.robot import Tool
from bar_assembly_core.design.meshes import write_mesh

WRITER = Writer(1, "design_io", "test", False)
#: Joints a state must list for the fake robot (the wheel is passive, the tool0 joints fixed).
JOINTS = ("left_joint1", "left_joint2", "right_joint1", "right_joint2")

TETRA = TriMesh.from_arrays([[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
                            [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]])


def _arm(side: str) -> str:
    """URDF text of one arm: two revolute joints, then a fixed flange `<side>_tool0`."""
    return f"""
  <link name="{side}_link1"/>
  <link name="{side}_link2"/>
  <link name="{side}_tool0"/>
  <joint name="{side}_joint1" type="revolute">
    <parent link="base_link"/><child link="{side}_link1"/>
    <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="{side}_joint2" type="revolute">
    <parent link="{side}_link1"/><child link="{side}_link2"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <joint name="{side}_tool0_joint" type="fixed">
    <parent link="{side}_link2"/><child link="{side}_tool0"/>
  </joint>"""


def write_robot_files(folder: Path, mesh: str = "meshes/base.obj", write_mesh_file: bool = True) -> tuple[Path, Path]:
    """Write a fake two-arm robot: root base_footprint, a passive wheel, two arms, one base mesh.

    Args:
        folder: Where to write `fake.urdf`, `fake.srdf` and the mesh.
        mesh: The base link's mesh reference, as written in the URDF.
        write_mesh_file: Also write the mesh file (at `folder/mesh`).

    Returns:
        tuple[Path, Path]: The URDF and SRDF.
    """
    folder.mkdir(parents=True, exist_ok=True)
    urdf = folder / "fake.urdf"
    urdf.write_text(f"""<?xml version="1.0"?>
<robot name="fake">
  <!-- a comment the copy keeps -->
  <link name="base_footprint"/>
  <link name="base_link">
    <visual><geometry><mesh filename="{mesh}"/></geometry></visual>
    <collision><geometry><mesh filename="{mesh}"/></geometry></collision>
  </link>
  <link name="wheel_link"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
  </joint>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/><child link="wheel_link"/><axis xyz="0 1 0"/>
  </joint>{_arm("left")}{_arm("right")}
</robot>
""")
    srdf = folder / "fake.srdf"
    srdf.write_text("""<?xml version="1.0"?>
<robot name="fake">
  <group name="left_arm"><chain base_link="base_link" tip_link="left_tool0"/></group>
  <group name="right_arm"><chain base_link="base_link" tip_link="right_tool0"/></group>
  <group name="both"><group name="left_arm"/><group name="right_arm"/></group>
  <passive_joint name="wheel_joint"/>
</robot>
""")
    if write_mesh_file:
        write_mesh(TETRA, folder / mesh)
    return urdf, srdf


def joints(value: float) -> dict[str, float]:
    """Every movable joint of the fake robot at one value."""
    return {name: value for name in JOINTS}


def turned(z: float, w: float) -> Pose:
    """A unit rotation about Z, at (1, 2, 0).

    ? Rounded to the writer's 12 decimals, so a write and read gives back exactly these values.
    """
    norm = (z * z + w * w) ** 0.5
    return Pose((1.0, 2.0, 0.0), (0.0, 0.0, round(z / norm, 12), round(w / norm, 12)))


def build_design(folder: Path) -> Design:
    """A small valid design in memory; robot files are written under `folder/source/`.

    * Covers: a null robot, null joints, an attached body, a pose override, touches, a placeholder,
      target joints and links, a tool movement running on (overlaps_next), a coupled linear movement,
      a mesh shared by two bodies, primitives with origins, a visual differing from collision, notes.

    Args:
        folder: A scratch folder (e.g. pytest's tmp_path).

    Returns:
        Design: Robots `robots/alice` and `robots/cindy`, two actions.
    """
    urdf, srdf = write_robot_files(folder / "source")
    robots = {
        "robots/alice": RobotSpec("robots/alice", urdf, srdf, None, {"left_tool0": "tools/Grip"}),
        "robots/cindy": RobotSpec("robots/cindy", urdf, srdf, "0806", {"left_tool0": "tools/AT3L"}),
    }
    tool_mesh = TriMesh.from_arrays(TETRA.vertices * 2, TETRA.faces)
    tools = {
        "tools/AT3L": Tool("tools/AT3L", Geometry((tool_mesh,), (tool_mesh,)), Pose((0.0, 0.0, 0.12)),
                           "scaffolding_v3", touches=("robots/cindy/left_link2",)),
        "tools/Grip": Tool("tools/Grip", Geometry((BoxShape((0.1, 0.1, 0.2)),), (BoxShape((0.1, 0.1, 0.2)),)),
                           Pose((0.0, 0.0, 0.2)), "robotiq"),
    }
    bar = CylinderShape(0.0125, 0.9, Pose((0.0, 0.0, 0.45)))
    box = BoxShape((0.05, 0.05, 0.05), turned(0.3, 0.9))
    slab = BoxShape((4.0, 4.0, 0.05))
    bodies = {
        "bars/B1": BodySpec("bars/B1", turned(0.0, 1.0), Geometry((bar, box), (bar,)), label="first bar"),
        "bars/B2": BodySpec("bars/B2", Pose((0.5, 0.0, 0.1)), Geometry((bar,), (bar,))),
        "joints/J1_male": BodySpec("joints/J1_male", Pose((0.1, 0.0, 0.0)), Geometry((TETRA,), (TETRA,))),
        "joints/J2_male": BodySpec("joints/J2_male", Pose((0.2, 0.0, 0.0)), Geometry((TETRA,), (TETRA,)),
                                   touches=("bars/B1",)),
        "ground/WG0": BodySpec("ground/WG0", Pose(), Geometry((slab,), (slab,)), touches=("robots/cindy/wheel_link",)),
    }
    everything = frozenset(bodies)
    base_a, base_c = Pose((-1.0, 0.0, 0.0)), turned(0.7071, 0.7071)
    joint_action = Action(
        id="B1_J_joint", type="bar_jointing", robot="robots/cindy", bar="bars/B1", ground=("ground/WG0",),
        movements=(
            Movement(
                id="B1_M0_free", type="free", controller="joint_tracking", arms=("robots/cindy/left_tool0",),
                start=State(robots={"robots/alice": None, "robots/cindy": RobotState(base_c, None)},
                            present=frozenset({"bars/B1", "joints/J1_male", "ground/WG0"}), poses={},
                            attached={"bars/B1": Attached("robots/cindy/left_tool0", Pose((0.0, 0.0, 0.12)))},
                            touches=frozenset({("bars/B1", "tools/AT3L")})),
                target=Target(joints={"robots/cindy": {"left_joint1": 0.5}},
                              links={"robots/cindy/left_tool0": Pose((0.3, 0.2, 0.6))}),
                notes={"lm_distance_mm": 30.0, "approach_axis": [0, 0, -1]}),
            Movement(
                id="B1_M1_tighten", type="tool", controller="none", tools=("tools/AT3L",), tool_action="tighten",
                overlaps_next=True,
                start=State(robots={"robots/alice": None, "robots/cindy": RobotState(base_c, joints(0.25))},
                            present=everything, poses={"joints/J1_male": Pose((0.1, 0.0, 0.01))}, attached={},
                            placeholder=frozenset({"bars/B1"}))),
            Movement(
                id="B1_M2_retreat", type="linear", controller="cartesian_compliant", coupled=True,
                arms=("robots/cindy/left_tool0", "robots/cindy/right_tool0"), label="both arms back",
                start=State(robots={"robots/alice": None, "robots/cindy": RobotState(base_c, joints(-0.5))},
                            present=everything, poses={}, attached={})),
        ))
    hold_action = Action(
        id="B2_H_hold", type="bar_holding", robot="robots/alice", bar="bars/B2", supports_until=("bars/B1",),
        label="hold B2",
        movements=(
            Movement(
                id="B2_M0_manual", type="manual", controller="none",
                start=State(robots={"robots/alice": RobotState(base_a, None),
                                    "robots/cindy": RobotState(base_c, joints(0.0))},
                            present=everything, poses={}, attached={},
                            touches=frozenset({("bars/B2", "robots/alice/left_tool0")})),
                target=Target(joints={"robots/alice": joints(1.0)}, links={})),
        ))
    return Design(folder=None, writer=WRITER, robots=robots, tools=tools, bodies=bodies,
                  schedule=("B1_J_joint", "B2_H_hold"), actions={a.id: a for a in (joint_action, hold_action)})


# --- --- --- --- --- CHANGING A DESIGN --- --- --- --- ---

def with_action(design: Design, action_id: str, **changes) -> Design:
    """The design with one action changed (`dataclasses.replace` fields)."""
    return replace(design, actions={**design.actions, action_id: replace(design.actions[action_id], **changes)})


def with_movement(design: Design, action_id: str, index: int, **changes) -> Design:
    """The design with one movement changed."""
    movements = list(design.actions[action_id].movements)
    movements[index] = replace(movements[index], **changes)
    return with_action(design, action_id, movements=tuple(movements))


def with_start(design: Design, action_id: str, index: int, **changes) -> Design:
    """The design with one movement's start state changed."""
    start = design.actions[action_id].movements[index].start
    return with_movement(design, action_id, index, start=replace(start, **changes))
