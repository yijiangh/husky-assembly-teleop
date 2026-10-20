"""When a plan goes stale, who may take a robot's lease, and how the operator's prompt takes answers."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot

from husky_assembly_teleop.shared.execution.basis import basis_of, stale_reason
from husky_assembly_teleop.shared.execution.interlock import Busy, busy_reason, give_back, refuse_while_held, take
from husky_assembly_teleop.shared.execution.operator import PROMPT_ARM_S, PromptBox
from husky_assembly_teleop.shared.runs.runs import RunSettings
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

pytestmark = pytest.mark.safety

CINDY, ALICE = "robots/a200-0806", "robots/a200-0804"
NAMES = ("left_ur_arm_shoulder_pan_joint", "left_ur_arm_elbow_joint")
STATUSES = {"B20_J": "pending", "B20_R": "pending"}
LOADED = SimpleNamespace(source=Path("/process"))


def snapshot(cindy_joint: float = 0.0, cindy_x: float = 1.0, alice_x: float = -1.0, alice_joint: float = 0.5,
             cindy_yaw: float = 0.0) -> Scene:
    """Cindy acting at (cindy_x, 0) turned `cindy_yaw`, Alice standing by."""
    model = SimpleNamespace(movable_joints=NAMES)
    turn = Pose((cindy_x, 0.0, 0.0), (0.0, 0.0, math.sin(cindy_yaw / 2), math.cos(cindy_yaw / 2)))
    return Scene(robots={
        CINDY: RobotObject(CINDY, model, turn, {NAMES[0]: cindy_joint, NAMES[1]: 1.0}),
        ALICE: RobotObject(ALICE, model, Pose((alice_x, 0.0, 0.0)), {"ur_arm_elbow_joint": alice_joint})})


def basis(settings: RunSettings = RunSettings()):
    """The basis of a plan for B20_J's transfer (movement 3) made from `snapshot()`."""
    step = SimpleNamespace(action=SimpleNamespace(id="B20_J"), movement_index=3)
    return basis_of(snapshot(), LOADED, step, "a200-0806", NAMES, {NAMES[0]: 0.0, NAMES[1]: 1.0}, settings,
                    STATUSES, False, 12.0)


def reason(scene: Scene, **now) -> str:
    """The stale reason of `basis()` in `scene`, everything else as planned unless given."""
    given = dict(loaded=LOADED, current=("B20_J", 3), transfer_as_free=True, ignore_built=False, statuses=STATUSES)
    return stale_reason(basis(), scene, **{**given, **now})


def test_a_plan_on_the_same_world_is_not_stale():
    """Nothing changed: no reason; the record is JSON values."""
    assert reason(snapshot()) == ""
    record = basis().as_record()
    assert record["base"] == [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] and record["others"][ALICE]["joints"]


@pytest.mark.parametrize("scene, stale, words", [
    (snapshot(cindy_joint=math.radians(1.9)), False, ""),
    (snapshot(cindy_joint=math.radians(2.1)), True, "arm moved 2.1°"),
    (snapshot(cindy_x=1.001), False, ""),
    (snapshot(cindy_x=1.003), True, "base moved 3.0 mm"),
    (snapshot(cindy_yaw=math.radians(0.6)), True, "0.60°"),
    (snapshot(alice_x=-1.009), False, ""),
    (snapshot(alice_x=-1.011), True, f"{ALICE} moved since planning: base 1.1 cm"),
    (snapshot(alice_joint=0.5 + math.radians(1.1)), True, "joints up to 1.1°"),
])
def test_movements_at_their_thresholds(scene, stale, words):
    """The arm at 2°, the base at 2 mm / 0.5°, another robot at 1 cm / 1° and 1° on its joints."""
    found = reason(scene)
    assert bool(found) == stale and words in found


@pytest.mark.parametrize("changes, words", [
    (dict(loaded=SimpleNamespace(source=Path("/process"))), "loaded again"),
    (dict(current=("B20_J", 4)), "current step changed"),
    (dict(current=None), "current step changed"),
    (dict(transfer_as_free=False), "workaround"),
    (dict(ignore_built=True), "ignore built"),
    (dict(statuses={**STATUSES, "B20_J": "done"}), "marked done or reopened"),
])
def test_the_run_and_the_process_changing(changes, words):
    """A reloaded process, another step, other settings or statuses each make the plan stale."""
    assert words in reason(snapshot(), **changes)


def test_the_acting_robot_gone():
    """The acting robot out of the scene."""
    scene = snapshot()
    del scene.robots[CINDY]
    assert "no longer in the scene" in reason(scene)


def world(ctx: FakeContext) -> SimpleNamespace:
    """Cindy and Alice, measured now, not moving."""
    node = fake_node(ctx.now, ctx.logs)
    cindy = fake_robot(node, "a200-0806", [fake_arm(node, "left_ur_arm", [0.0] * 6),
                                           fake_arm(node, "right_ur_arm", [0.0] * 6)])
    alice = fake_robot(node, "a200-0804", [fake_arm(node, "ur_arm", [0.0] * 6)])
    return SimpleNamespace(robots={"a200-0806": cindy, "a200-0804": alice})


def test_one_lease_at_a_time():
    """A lease refuses a second on any robot until given back; while one is held, commands to any robot are refused."""
    ctx = FakeContext()
    robots = world(ctx)
    lease = take(robots, "a200-0806", "movement_exec", "B20_R_home", lambda: None, ctx.t)
    assert robots.robots["a200-0806"].lease is lease
    with pytest.raises(Busy, match="held by movement_exec"):
        take(robots, "a200-0804", "movement_exec", "B21_H_approach", lambda: None, ctx.t)
    lines = []
    assert refuse_while_held(robots, "a200-0806 tool Open", lines.append)
    assert refuse_while_held(robots, "a200-0804 ur_arm Start", lines.append)
    assert lines == ["a200-0806 tool Open refused: a200-0806 is held by movement_exec (B20_R_home), one motion at a "
                     "time; Cancel it first",
                     "a200-0804 ur_arm Start refused: a200-0806 is held by movement_exec (B20_R_home), one motion at "
                     "a time; Cancel it first"]
    assert busy_reason(robots, ctx.t, besides="a200-0806") == ""
    give_back(robots, lease)
    assert not refuse_while_held(robots, "a200-0804 ur_arm Start", lines.append)
    assert robots.robots["a200-0806"].lease is None and busy_reason(robots, ctx.t) == ""
    with pytest.raises(Busy, match="not configured"):
        take(robots, "a200-0805", "movement_exec", "B35_H_open", lambda: None, ctx.t)


def test_a_lease_given_back_late_leaves_a_newer_one():
    """Giving back an old lease clears nothing once the robot holds another."""
    ctx = FakeContext()
    robots = world(ctx)
    old = take(robots, "a200-0806", "movement_exec", "first", lambda: None, ctx.t)
    give_back(robots, old)
    new = take(robots, "a200-0806", "movement_exec", "second", lambda: None, ctx.t)
    give_back(robots, old)
    assert robots.robots["a200-0806"].lease is new


def test_an_arm_running_moving_or_unknown_refuses_a_lease():
    """`is_executing`, a slight motion within 2 s, or joint states over 0.5 s old on any arm refuse a lease."""
    ctx = FakeContext()
    robots = world(ctx)
    state = robots.robots["a200-0804"].arms["ur_arm"].state
    state.is_executing = True
    with pytest.raises(Busy, match="a200-0804 ur_arm is running a trajectory"):
        take(robots, "a200-0806", "movement_exec", "B20_R_home", lambda: None, ctx.t)
    state.is_executing, state.last_slight_motion_time = False, ctx.t - 1.9
    with pytest.raises(Busy, match="a200-0804 ur_arm moved within the last 2 s"):
        take(robots, "a200-0806", "movement_exec", "B20_R_home", lambda: None, ctx.t)
    state.last_slight_motion_time = ctx.t - 2.1
    give_back(robots, take(robots, "a200-0806", "movement_exec", "B20_R_home", lambda: None, ctx.t))
    state.last_update_time = ctx.t - 0.6
    with pytest.raises(Busy, match="can't tell whether a200-0804 ur_arm is moving"):
        take(robots, "a200-0806", "movement_exec", "B20_R_home", lambda: None, ctx.t)


def test_a_confirm_too_soon_is_ignored_and_answers_reach_only_their_prompt():
    """A Confirm within 0.5 s is ignored; a Cancel counts at once; a new prompt drops the old answer."""
    box = PromptBox()
    assert box.answer(True, 0.0) == "nothing to answer"
    first = box.ask("cindy (a200-0806)", "B20_R_home — move", 10.0)
    assert first.line.startswith("cindy (a200-0806): ")
    assert "double click" in box.answer(True, 10.0 + PROMPT_ARM_S / 2)
    assert box.answer(False, 10.1) == ""
    second = box.ask("cindy (a200-0806)", "again", 11.0, warn=True)
    assert second.id != first.id and second.line.startswith("cindy (a200-0806): ")

    async def scenario():
        ctx = FakeContext()
        ctx.t = 11.0
        waiting = asyncio.ensure_future(box.wait(ctx, second))
        await asyncio.sleep(0)
        assert box.answer(True, ctx.t + PROMPT_ARM_S) == ""
        assert box.answer(True, ctx.t + 1.0) == "Confirm ignored: the prompt is already answered"
        return await waiting

    assert asyncio.run(scenario()) is True and box.current is None


def test_a_cancel_wins_over_a_confirm_not_yet_taken():
    """Confirm then Cancel in the same tick: the wait sees Cancel."""
    box = PromptBox()
    prompt = box.ask("alice (a200-0804)", "OPEN the gripper", 0.0)
    box.answer(True, 1.0)
    box.answer(False, 1.0)

    async def scenario():
        return await box.wait(FakeContext(), prompt)

    assert asyncio.run(scenario()) is False


def test_robots_in_the_world_fake_are_arms():
    """The fake world's arms are real arm interfaces, measured now."""
    ctx = FakeContext()
    arm = world(ctx).robots["a200-0804"].arms["ur_arm"]
    assert np.allclose(arm.joint_vector(), 0.0) and arm.state.last_update_time == ctx.t


# --- --- --- --- --- what counts as moving (review M1) --- --- --- --- ---

def moving_lab():
    """The three robots following what they are sent; Belle's driver and arm."""
    from robot_lab import RobotLab
    lab = RobotLab()
    arm = lab.belle.arms["ur_arm"]
    return lab, arm, next(driver for driver in lab.drivers if getattr(driver, "arm", None) is arm)


def test_a_slow_move_is_busy_from_its_start_to_a_second_after_its_end():
    """A 3° joint move over 5 s (robot_control's Start): every tick of it, and a second after, refuses a lease."""
    lab, arm, _driver = moving_lab()

    async def scenario():
        for _ in range(12):
            await lab.ctx.next_tick()
        assert busy_reason(lab.ctx.world, lab.ctx.t) == ""
        start = lab.ctx.t
        duration = arm.send_joint_move(arm.joint_vector() + np.radians([3.0, 0, 0, 0, 0, 0]))
        blind = []
        while lab.ctx.t < start + duration + 1.0:
            await lab.ctx.next_tick()
            if not busy_reason(lab.ctx.world, lab.ctx.t):
                blind.append(round(lab.ctx.t - start, 1))
        return blind, start + duration

    blind, end = asyncio.run(scenario())
    assert blind == []
    lab.ctx.t = end + 2.2
    lab.tick(lab.ctx.t)
    assert busy_reason(lab.ctx.world, lab.ctx.t) == ""


def test_an_arm_whose_joint_states_went_stale_is_busy():
    """Belle's link drops while she moves: "can't tell"; a robot never measured this session is not counted."""
    lab, arm, driver = moving_lab()

    async def scenario():
        arm.send_joint_move(arm.joint_vector() + np.radians([30.0, 0, 0, 0, 0, 0]))
        for _ in range(5):
            await lab.ctx.next_tick()
        driver.silent_from = lab.ctx.t
        for _ in range(8):
            await lab.ctx.next_tick()

    asyncio.run(scenario())
    reason = busy_reason(lab.ctx.world, lab.ctx.t)
    assert "can't tell whether a200-0805 ur_arm is moving" in reason
    with pytest.raises(Busy):
        take(lab.ctx.world, "a200-0806", "movement_exec", "B20_R_home", lambda: None, lab.ctx.t)
    arm.state.last_update_time = None  # never measured this session
    assert busy_reason(lab.ctx.world, lab.ctx.t) == ""


def test_a_joint_velocity_or_a_millimetre_counts_as_moving():
    """A joint state reporting velocity, or a joint 0.002 rad away from where it stood, is motion."""
    from sensor_msgs.msg import JointState
    from husky_assembly_teleop.robot_interface.arm import UR_JOINT_NAMES
    lab, arm, _driver = moving_lab()
    lab.ctx.t += 3.0
    lab.tick(lab.ctx.t)
    here = list(arm.joint_vector())
    arm._on_joint_state(JointState(name=list(UR_JOINT_NAMES), position=here, velocity=[0.0, 0.02, 0, 0, 0, 0]))
    assert "moved" in busy_reason(lab.ctx.world, lab.ctx.t)
    lab.ctx.t += 3.0
    lab.tick(lab.ctx.t)
    assert busy_reason(lab.ctx.world, lab.ctx.t) == ""
    here[3] += 0.002
    arm._on_joint_state(JointState(name=list(UR_JOINT_NAMES), position=here))
    assert "moved" in busy_reason(lab.ctx.world, lab.ctx.t)


def test_a_driving_base_is_busy():
    """Wheels turning, a path being followed, or wheel states gone stale after being live: busy."""
    lab, _arm, _driver = moving_lab()
    base = lab.cindy.base.state
    base.joint_states_update_time, base.wheel_velocities = lab.ctx.t, {"front_left_wheel_joint": 0.0}
    assert busy_reason(lab.ctx.world, lab.ctx.t) == ""
    base.wheel_velocities = {"front_left_wheel_joint": 0.3}
    assert "a200-0806 base is driving" in busy_reason(lab.ctx.world, lab.ctx.t)
    base.wheel_velocities = {"front_left_wheel_joint": 0.0}
    base.follower = SimpleNamespace(state="following", update_time=lab.ctx.t)
    assert "following a path" in busy_reason(lab.ctx.world, lab.ctx.t)
    base.follower = None
    lab.ctx.t += 1.0
    lab.tick(lab.ctx.t)
    assert "can't tell whether a200-0806 base is moving" in busy_reason(lab.ctx.world, lab.ctx.t)
    assert busy_reason(lab.ctx.world, lab.ctx.t, besides="a200-0806") == ""
