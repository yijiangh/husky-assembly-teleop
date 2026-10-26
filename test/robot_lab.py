"""Alice, Belle and Cindy as fake robots that follow what they are sent, measured live in a `FakeContext`'s snapshot.

The arms are real `ArmInterface`s on fake nodes (`plugin_fakes`): every check before sending is the real one. Cindy
carries two scaffolding tools, Alice and Belle a Robotiq each, all answered by their fake drivers. A robot `track`ed
has its URDF frames and its base seen by mocap, as its compliant parts need.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from planning_fixtures import ALICE, BELLE, CINDY, robot_config
from plugin_fakes import (ArmDriver, FakeContext, RobotiqDriver, ScaffoldingDriver, fake_arm, fake_node, fake_robot,
                          fake_robotiq, fake_scaffolding)

from husky_assembly_teleop.robot_interface.frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.runs.runs import RunSettings
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

#: Arm joints to start at, UR order: Cindy's left and right, a support robot's one arm.
LEFT = [-1.54, -0.04, -2.82, -1.9, -0.01, 1.58]
RIGHT = [1.52, -3.14, 2.78, -1.3, 0.03, -1.57]
SUPPORT = [1.57, -2.97, 2.7, -2.96, 1.57, 0.0]
#: Where the bases stand, by serial.
BASES = {"a200-0806": Pose((3.0, 1.0, 0.0)), "a200-0804": Pose((0.0, 0.0, 0.0)), "a200-0805": Pose((1.0, 0.0, 0.0))}


def joints_of(robot) -> dict[str, float]:
    """Every arm's measured joints, by name."""
    values = {}
    for name, arm in robot.arms.items():
        values.update(zip(arm_joint_names(name), arm.joint_vector().tolist()))
    return values


class RobotLab:
    """The three robots, their drivers ticking with the context, every robot live in its snapshot.

    Attributes:
        ctx: The context; `ctx.world.robots` holds the robots.
        robots: Serial -> robot.
        cindy, alice, belle: The robots.
        tools: Cindy's scaffolding tools by arm.
        bases: Serial -> base pose; change one to move that base.
        live: Whether the snapshot shows every robot measured now (False: its stamps grow old).
    """

    def __init__(self, joints: dict[str, dict[str, list[float]]] | None = None, **driver):
        """Build the robots at `joints` (serial -> arm -> UR-order joints; START values by default).

        Args:
            joints: Where each arm starts.
            driver: Options for every arm's driver (follow, silent_from, speed).
        """
        joints = joints or {}
        self.configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
        # * Named movement_exec: the one plugin that holds robots for an automatic motion takes its leases under it.
        self.ctx = FakeContext(robots=self.configs, step=self.tick, name="movement_exec")
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.tools = {"left_ur_arm": fake_scaffolding(node, "/a200_0806/left_gripper"),
                      "right_ur_arm": fake_scaffolding(node, "/a200_0806/right_gripper")}

        def arm(serial: str, name: str, default: list[float], tool):
            return fake_arm(node, name, joints.get(serial, {}).get(name, default), end_effector=tool)

        self.cindy = fake_robot(node, "a200-0806", [arm("a200-0806", "left_ur_arm", LEFT, self.tools["left_ur_arm"]),
                                                    arm("a200-0806", "right_ur_arm", RIGHT,
                                                        self.tools["right_ur_arm"])])
        self.alice = fake_robot(node, "a200-0804", [arm("a200-0804", "ur_arm", SUPPORT, fake_robotiq(node))])
        self.belle = fake_robot(node, "a200-0805", [arm("a200-0805", "ur_arm", SUPPORT,
                                                        fake_robotiq(node, "/a200_0805/gripper"))])
        self.robots = {robot.config.serial: robot for robot in (self.alice, self.belle, self.cindy)}
        self.ctx.world.robots.update(self.robots)
        self.drivers = [ArmDriver(each, robot, **driver)
                        for robot in self.robots.values() for each in robot.arms.values()]
        self.drivers += [ScaffoldingDriver(tool) for tool in self.tools.values()]
        self.drivers += [RobotiqDriver(robot.arms["ur_arm"].end_effector) for robot in (self.alice, self.belle)]
        self.bases = dict(BASES)
        self.live = True
        #: The robots whose base mocap sees (`track`).
        self.tracked: set[str] = set()
        self.tick(self.ctx.t)

    def track(self, serial: str) -> None:
        """Give a robot its URDF frames and its base to mocap, at `bases[serial]` every tick; its TCP is then reported.

        Under joint control its arms report tool0 by the URDF as their TCP (`ArmDriver`).
        """
        robot, config = self.robots[serial], next(config for config in self.configs if config.serial == serial)
        urdf = config.model.urdf
        robot.config.model = config.model
        robot._husky_in_mocap_frame = fixed_transform(urdf, MOCAP_FRAME, HUSKY_FRAME)
        for arm in robot.arms.values():
            arm.urdf, arm.base_in_husky = urdf, fixed_transform(urdf, HUSKY_FRAME, f"{arm.config.name}_base_link")
        robot.base.mocap_id = len(self.tracked) + 1
        self.tracked.add(serial)
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """Mocap sees the tracked bases, arms and tools move and report; the snapshot is taken from them."""
        for serial in self.tracked:
            state, pose = self.robots[serial].base.state, self.bases[serial]
            state.position, state.orientation = np.array(pose.position), np.array(pose.orientation)
            state.tracked = state.tracking_valid = True
            state.last_update_time = state.last_fix_time = t
            state.marker_errors.append((t, 0.0))
        for driver in self.drivers:
            driver.tick(t)
        models = {config.serial: config.model for config in self.configs}
        stamp = t if self.live else t - 5.0
        self.ctx.scene.snapshot = Scene(robots={
            robot_id(serial): RobotObject(robot_id(serial), models[serial], self.bases[serial], joints_of(robot),
                                          base_time=stamp, joints_time=stamp)
            for serial, robot in self.robots.items()})

    def published(self) -> int:
        """How many trajectories went out to any arm, multi-arm messages included."""
        return sum(len(arm._trajectory.messages) for robot in self.robots.values() for arm in robot.arms.values()) \
            + len(self.cindy._multi_arm.messages)


class FakeRun:
    """assembly_run as other plugins see it: a view, and what they asked."""

    def __init__(self, loaded, current):
        """The run at `current`, every action pending."""
        self.view = SimpleNamespace(process=loaded, current=current, settings=RunSettings(),
                                    statuses={action: "pending" for action in loaded.process.schedule},
                                    beliefs={})
        self.records, self.selected, self.marked = [], [], []

    def request_record(self, event, **fields):
        """Keep the record."""
        self.records.append((event, fields))

    def request_set_current(self, action_id, movement_index=None):
        """Make it current at once."""
        self.selected.append((action_id, movement_index))
        self.view.current = (action_id, movement_index)

    def request_settings(self, **changes):
        """Change the settings at once."""
        self.view.settings = RunSettings(**{**vars(self.view.settings), **changes})

    def request_mark_done(self, action_id=None):
        """Keep the mark."""
        self.marked.append(action_id)

    def files(self) -> list[dict]:
        """The movement records written, in order."""
        return [fields["file"] for _event, fields in self.records if fields.get("file") is not None]
