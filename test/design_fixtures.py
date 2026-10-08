"""Shared builders for the design tests: tiny robot files and a small valid two-robot design."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from bar_assembly_core.design import (Action, BodySpec, Design, Holder, LineSpec, Movement, Producer, RobotSpec,
                                      RobotState, State, Target, ToolState, Writer)
from bar_assembly_core.geometry import BoxShape, CylinderShape, Geometry, Pose, TriMesh, compose, invert
from bar_assembly_core.kinematics import link_pose
from bar_assembly_core.robot import Tool
from bar_assembly_core.design.meshes import write_mesh

WRITER = Writer(2, "design_io", "test", False)
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


def rounded(pose: Pose) -> Pose:
    """A pose rounded to the writer's 12 decimals, so a write and read gives back exactly these values."""
    return Pose(tuple(round(v, 12) for v in pose.position), tuple(round(v, 12) for v in pose.orientation))


def build_design(folder: Path) -> Design:
    """A small schema 2 design in memory that passes every file and plan check; robot files go to `folder/source/`.

    * The plan: Cindy mounts, grasps and inserts B1 on its ground half (G1 on WG0); Alice closes her gripper on the
      built B1 before Cindy ungrasps and retreats, so B1 has two holders; Cindy mounts B2 from a rack and inserts its
      male half J1 into B1's female half; Alice lets go once B2 is built.
    * Covers: an absent robot, unknown joints and an unknown base, a staged bar with a pose, mounts and mates, parts and
      markers, mount contacts, ground links, a producer, a manual step, tool moves, an insert with a drive and a line,
      a tool-only ungrasp, a retreat with `target: null`, a support gripper on its bar, a mesh shared by three bodies,
      primitives with origins, notes.

    Args:
        folder: A scratch folder (e.g. pytest's tmp_path).

    Returns:
        Design: Robots `robots/alice` and `robots/cindy`, five actions.
    """
    urdf, srdf = write_robot_files(folder / "source")
    robots = {
        "robots/alice": RobotSpec("robots/alice", urdf, srdf, None, {"left_tool0": "tools/Grip"}, ("wheel_link",)),
        "robots/cindy": RobotSpec("robots/cindy", urdf, srdf, "0806", {"left_tool0": "tools/AT3L"}, ("wheel_link",)),
    }
    tool_mesh = TriMesh.from_arrays(TETRA.vertices * 2, TETRA.faces)
    tcp = Pose((0.0, 0.0, 0.12))
    tools = {
        "tools/AT3L": Tool("tools/AT3L", Geometry((tool_mesh,), (tool_mesh,)), tcp, "scaffolding_v3",
                           mount_contacts=("robots/cindy/left_link2",)),
        "tools/Grip": Tool("tools/Grip", Geometry((BoxShape((0.1, 0.1, 0.2)),), (BoxShape((0.1, 0.1, 0.2)),)),
                           Pose((0.0, 0.0, 0.2)), "robotiq"),
    }
    base_a, base_c = Pose((-1.0, 0.0, 0.0)), turned(0.7071, 0.7071)
    flange, grip_flange = "robots/cindy/left_tool0", "robots/alice/left_tool0"
    inserted = {**joints(0.25), "left_joint1": 0.5}

    # * Poses that agree with the geometry (plan check B14): the bars' grasps follow from the tool's TCP, the part
    #   seat and the mount offset; B1's design pose is where Cindy's flange holds it at the end of the insert.
    on_g1, on_j1 = Pose((0.0, 0.0, 0.05)), Pose((0.0, 0.0, 0.0))
    ground_seat = Pose(orientation=(0.0, 0.0, 1.0, 0.0))
    grasp_b1 = rounded(compose(compose(tcp, ground_seat), invert(on_g1)))
    grasp_b2 = rounded(compose(tcp, invert(on_j1)))
    b1 = rounded(compose(link_pose(urdf, base_c, inserted, "left_tool0"), grasp_b1))
    b2 = rounded(compose(link_pose(urdf, base_c, joints(0.35), "left_tool0"), grasp_b2))
    grasp_a = rounded(compose(invert(link_pose(urdf, base_a, joints(1.0), "left_tool0")), b1))

    bar = CylinderShape(0.0125, 0.9, Pose((0.0, 0.0, 0.45)))
    box = BoxShape((0.05, 0.05, 0.05), turned(0.3, 0.9))
    slab = BoxShape((4.0, 4.0, 0.05))
    bodies = {
        "bars/B1": BodySpec("bars/B1", b1, Geometry((bar, box), (bar,)), label="first bar"),
        "bars/B2": BodySpec("bars/B2", b2, Geometry((bar,), (bar,))),
        "joints/G1_ground": BodySpec("joints/G1_ground", rounded(compose(b1, on_g1)), Geometry((TETRA,), (TETRA,)),
                                     part="T20/Ground", mount="bars/B1"),
        "joints/J1_female": BodySpec("joints/J1_female", rounded(compose(b1, Pose((0.0, 0.0, 0.85)))),
                                     Geometry((TETRA,), (TETRA,)), part="T20/Female", mount="bars/B1",
                                     markers={"m1": (0.01, 0.0, 0.02)}),
        "joints/J1_male": BodySpec("joints/J1_male", rounded(compose(b2, on_j1)), Geometry((TETRA,), (TETRA,)),
                                   part="T20/Male", mount="bars/B2"),
        "ground/WG0": BodySpec("ground/WG0", Pose(), Geometry((slab,), (slab,))),
    }
    mates = frozenset({("ground/WG0", "joints/G1_ground"), ("joints/J1_female", "joints/J1_male")})
    rack = {"bars/B2": Pose((2.0, 0.0, 0.1))}
    held_c = {"bars/B1": (Holder(flange, grasp_b1),)}
    held_ca = {"bars/B1": (Holder(flange, grasp_b1), Holder(grip_flange, grasp_a))}
    held_a = {"bars/B1": (Holder(grip_flange, grasp_a),)}
    b2_held = {**held_a, "bars/B2": (Holder(flange, grasp_b2),)}

    def state(cindy, alice, at3l: ToolState, grip: ToolState, attached: dict, built=(), poses=None) -> State:
        """Both robots, WG0 and B2 (on the rack until mounted) present, plus every attached or built bar."""
        return State(robots={"robots/alice": alice, "robots/cindy": cindy},
                     present=frozenset({"ground/WG0", "bars/B2", *attached, *built}),
                     attached=attached, built=frozenset(built), poses=rack if poses is None else poses,
                     tools={"tools/AT3L": at3l, "tools/Grip": grip})

    cindy_at = lambda values, base=base_c: RobotState(base, values)  # noqa: E731
    alice_at = lambda values: RobotState(base_a, values)  # noqa: E731
    released, grip_open = ToolState("open", None), ToolState("open", None)
    on_g1_open, on_g1_closed = ToolState("open", "joints/G1_ground"), ToolState("closed", "joints/G1_ground")
    linear_down = {flange: LineSpec((0.0, 0.0, -1.0), 0.015)}

    b1_joint = Action(
        id="B1_J_joint", type="bar_jointing", robot="robots/cindy", bar="bars/B1", ground=("ground/WG0",),
        movements=(
            Movement(id="B1_M0_mount", ends_on="operator", label="operator mounts B1",
                     start=state(cindy_at(None), None, ToolState(None, None), grip_open, {}),
                     target=Target(attached=held_c)),
            Movement(id="B1_M1_grasp", ends_on="tools", start=state(cindy_at(joints(0.25)), None, on_g1_open, grip_open,
                                                                    held_c),
                     target=Target(tools={"tools/AT3L": "closed"})),
            Movement(id="B1_M2_insert", arms=(flange,), path="linear", controller="compliant", ends_on="tools",
                     line=linear_down, drives={"tools/AT3L": "tighten"},
                     start=state(cindy_at(joints(0.25)), None, on_g1_closed, grip_open, held_c),
                     target=Target(joints={"robots/cindy": {"left_joint1": 0.5}},
                                   links={flange: Pose((0.3, 0.2, 0.6))}, built=frozenset({"bars/B1"})),
                     notes={"approach_axis": "walkable_ground_normal", "approach_offset_mm": 15.0}),
        ))
    b1_hold = Action(
        id="B1_H_hold", type="bar_holding", robot="robots/alice", bar="bars/B1", supports_until=("bars/B2",),
        label="hold B1", notes={"why": "keeps B1 still"},
        movements=(
            Movement(id="B1_H_M0_approach", arms=(grip_flange,), path="free", controller="position",
                     start=state(cindy_at(inserted), alice_at(None), on_g1_closed, grip_open, held_c, ("bars/B1",)),
                     target=Target(joints={"robots/alice": joints(1.0)})),
            Movement(id="B1_H_M1_close", ends_on="tools",
                     start=state(cindy_at(inserted), alice_at(joints(1.0)), on_g1_closed, ToolState("open", "bars/B1"),
                                 held_c, ("bars/B1",)),
                     target=Target(tools={"tools/Grip": "closed"}, attached=held_ca)),
        ))
    grip_on_b1 = ToolState("closed", "bars/B1")
    b1_release = Action(
        id="B1_R_release", type="bar_release", robot="robots/cindy", bar="bars/B1",
        movements=(
            Movement(id="B1_R_M0_ungrasp", ends_on="tools",
                     start=state(cindy_at(inserted), alice_at(joints(1.0)), on_g1_closed, grip_on_b1, held_ca,
                                 ("bars/B1",)),
                     target=Target(tools={"tools/AT3L": "open"}, attached=held_a)),
            Movement(id="B1_R_M1_retreat", arms=(flange,), path="linear", controller="position",
                     line={flange: LineSpec((0.0, 1.0, 0.0), 0.05)}, label="back off",
                     start=state(cindy_at(inserted, None), alice_at(joints(1.0)), on_g1_open, grip_on_b1, held_a,
                                 ("bars/B1",))),
        ))
    on_j1_open, on_j1_closed = ToolState("open", "joints/J1_male"), ToolState("closed", "joints/J1_male")
    b2_joint = Action(
        id="B2_J_joint", type="bar_jointing", robot="robots/cindy", bar="bars/B2",
        movements=(
            Movement(id="B2_M0_mount", ends_on="operator",
                     start=state(cindy_at(joints(0.0)), alice_at(joints(1.0)), released, grip_on_b1, held_a,
                                 ("bars/B1",)),
                     target=Target(attached=b2_held)),
            Movement(id="B2_M1_grasp", ends_on="tools",
                     start=state(cindy_at(joints(0.0)), alice_at(joints(1.0)), on_j1_open, grip_on_b1, b2_held,
                                 ("bars/B1",), {}),
                     target=Target(tools={"tools/AT3L": "closed"})),
            Movement(id="B2_M2_transfer", arms=(flange,), path="free", controller="position",
                     start=state(cindy_at(joints(0.0)), alice_at(joints(1.0)), on_j1_closed, grip_on_b1, b2_held,
                                 ("bars/B1",), {}),
                     target=Target(joints={"robots/cindy": joints(0.3)})),
            Movement(id="B2_M3_insert", arms=(flange,), path="linear", controller="compliant", ends_on="tools",
                     line=linear_down, drives={"tools/AT3L": "tighten"},
                     start=state(cindy_at(joints(0.3)), alice_at(joints(1.0)), on_j1_closed, grip_on_b1, b2_held,
                                 ("bars/B1",), {}),
                     target=Target(joints={"robots/cindy": joints(0.35)}, built=frozenset({"bars/B1", "bars/B2"}))),
        ))
    b2_held_c = {"bars/B2": b2_held["bars/B2"]}
    b1_hold_release = Action(
        id="B1_HR_hold_release", type="bar_holding_release", robot="robots/alice", bar="bars/B1",
        movements=(
            Movement(id="B1_HR_M0_open", ends_on="tools",
                     start=state(cindy_at(joints(0.35)), alice_at(joints(1.0)), on_j1_closed, grip_on_b1, b2_held,
                                 ("bars/B1", "bars/B2"), {}),
                     target=Target(tools={"tools/Grip": "open"}, attached=b2_held_c)),
            Movement(id="B1_HR_M1_retreat", arms=(grip_flange,), path="linear", controller="position",
                     line={grip_flange: LineSpec((0.0, 0.0, 1.0), 0.1)},
                     start=state(cindy_at(joints(0.35)), alice_at(joints(1.0)), on_j1_closed,
                                 ToolState("open", "bars/B1"), b2_held_c, ("bars/B1", "bars/B2"), {}),
                     target=Target(joints={"robots/alice": joints(0.9)})),
        ))
    actions = (b1_joint, b1_hold, b1_release, b2_joint, b1_hold_release)
    return Design(folder=None, writer=WRITER, robots=robots, tools=tools, bodies=bodies,
                  schedule=tuple(action.id for action in actions), actions={a.id: a for a in actions},
                  mates=mates, producer=Producer("rhino", "abc123", False, "RSExportAllBarActions"))


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
