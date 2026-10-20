"""Real plans on the purged process, timed and dry-run on fake arms: every check of the arms passes, nothing is sent."""

from __future__ import annotations

import asyncio
import random

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, purged_process, robot_config, step
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot

from husky_assembly_teleop.plugins.motion_planner.plugin import MotionPlannerPlugin
from husky_assembly_teleop.shared.execution.send import DRY_RUN, motion_command, send_motion
from husky_assembly_teleop.shared.movement_exec import DEFAULT_TRAJ_TIME, serial_of
from husky_assembly_teleop.shared.planning.ik import ssik_unavailable
from husky_assembly_teleop.shared.planning.joints import arm_joint_names
from husky_assembly_teleop.shared.planning.service import PlanRequest
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

#: The movements planned, and whether the transfer workaround is on.
MOVEMENTS = (("B20_J_transfer", True), ("B20_R_home", False), ("B21_H_approach", False))


def planning_scene(loaded, movement: str, configs) -> Scene:
    """The design's snapshot at a movement's start; an acting robot with no authored start has its arms stowed."""
    scene, at = design_scene(loaded, movement), step(loaded, movement)
    if at.assumed_from != "own target":
        return scene
    serial = serial_of(loaded, at.action.id, configs)
    config = next(config for config in configs if config.serial == serial)
    live = robot_id(serial)
    stowed = {name: value for arm in config.arms for name, value in zip(arm_joint_names(arm.name), arm.stow_joints)}
    robot = scene.robots[live]
    return Scene(robots={**scene.robots, live: RobotObject(live, robot.model, robot.base,
                                                           {**robot.joints, **stowed})})


@pytest.mark.slow
def test_real_plans_pass_every_check_of_the_arms_and_send_nothing():
    """B20_J's transfer (workaround), B20_R's way home, B21_H's approach: planned, timed, dry-run; nothing sent.

    Each is timed at its default motion time and dry-run on arms standing at the plan's start: DRY_RUN, every
    trajectory with as many points as samples and at rest at the end, nothing published.
    """
    if ssik_unavailable() is not None:
        pytest.skip(ssik_unavailable())
    loaded = purged_process()
    configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
    # * Seeded, as the M3 check: tamp's free search is random, and a marginal start (B20_R_home's, near the ground
    #   halves) can miss in all three tries of an unlucky run.
    random.seed(0)
    np.random.seed(0)
    planner = MotionPlannerPlugin()
    planner._log = planner._pool.log = lambda _line: None
    outcomes = []

    async def scenario():
        for movement, transfer_as_free in MOVEMENTS:
            at = step(loaded, movement)
            request = PlanRequest(loaded, at, planning_scene(loaded, movement, configs), configs,
                                  transfer_as_free=transfer_as_free, label=movement)
            route = await planner.request_plan(request)
            assert route.ok, (movement, route.result.reason)
            trajectory = route.result.trajectory
            serial = serial_of(loaded, at.action.id, configs)
            config = next(config for config in configs if config.serial == serial)
            command = motion_command(serial, [arm.name for arm in config.arms], trajectory.joint_names,
                                     trajectory.positions, DEFAULT_TRAJ_TIME[route.routine])
            ctx = FakeContext()
            node = fake_node(ctx.now, ctx.logs)
            start = dict(zip(trajectory.joint_names, trajectory.positions[0]))
            arms = [fake_arm(node, arm.name, [start[name] for name in arm_joint_names(arm.name)])
                    for arm in config.arms]
            robot = fake_robot(node, serial, arms)
            built = []
            for arm in arms:
                build = arm.build_trajectory
                arm.build_trajectory = lambda *args, build=build: built.append(build(*args)) or built[-1]
            outcome = await send_motion(ctx, robot, command, dry_run=True)
            outcomes.append((movement, outcome, command, built, robot, ctx))

    try:
        asyncio.run(scenario())
    finally:
        planner.teardown(None)
    for movement, outcome, command, built, robot, ctx in outcomes:
        assert outcome.status == DRY_RUN, (movement, outcome.reason, ctx.lines("error"))
        samples = len(command.arms[0].positions)
        assert len(built) == len(command.arms) and command.duration >= command.requested - 1e-9
        for message in built:
            assert message is not None and len(message.points) == samples
            assert list(message.points[-1].velocities) == [0.0] * 6
        assert not any(arm._trajectory.messages for arm in robot.arms.values())
        assert robot._multi_arm is None or not robot._multi_arm.messages
