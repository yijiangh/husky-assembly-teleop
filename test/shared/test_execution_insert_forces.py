"""The insert's target forces on a fake Cindy: the weight taken off, the push, the stiffness, a tight screw held."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from test_execution_insert import ARMS, LEFT_ARM, RIGHT_ARM, Lab

from husky_assembly_teleop.shared.execution import insert
from husky_assembly_teleop.shared.execution.insert_forces import (WEIGHT_MAX_SIDEWAYS, BarWeight, InsertForces,
                                                                  InsertForceSettings)
from husky_assembly_teleop.shared.execution.stiffness import send_back, set_stiffness
from husky_assembly_teleop.shared.execution.stream import STALLED, measured_tool0

pytestmark = pytest.mark.safety


class WeighingLab(Lab):
    """The Lab, its arms reading a fixed force in the world (turned into each one's measured tool0 every tick).

    `forces` keeps each target force sent, in the world by the arm's measured tool0 then.
    """

    def __init__(self, world_force: dict[str, tuple[float, float, float]], **options):
        """Each arm reads `world_force[arm]`, newtons."""
        self.world_force = {name: np.asarray(force, dtype=float) for name, force in world_force.items()}
        super().__init__(**options)
        self.forces: dict[str, list[np.ndarray]] = {name: [] for name in ARMS}
        for name, arm in self.robot.arms.items():
            arm.wrenches_sent.attach(lambda sent, name=name: self.forces[name].append(self._in_world(name, sent.force)))

    def tick(self, t: float) -> None:
        """As the Lab, then each arm's force reading."""
        super().tick(t)
        for name, force in getattr(self, "world_force", {}).items():
            pose = measured_tool0(self.robot, name)
            if pose is not None:
                tool = Rotation.from_quat(pose.orientation).inv().apply(force)
                self.robot.arms[name].state.wrench = np.concatenate([tool, np.zeros(3)])
                self.robot.arms[name].state.wrench_update_time = t

    def _in_world(self, name: str, force: np.ndarray) -> np.ndarray:
        """A force in the arm's tool0, in the world by its measured tool0 now."""
        return Rotation.from_quat(measured_tool0(self.robot, name).orientation).apply(force)

    def line(self, plan, name: str) -> np.ndarray:
        """The arm's planned compliant line in the world, unit length."""
        line = Rotation.from_quat(self.robot.husky_in_world().orientation).apply(plan.lines[name])
        return line / np.linalg.norm(line)


#: A 9 N bar, the arms squeezing it with 3 N along world x: only the 9 N may come off.
SQUEEZED_BAR = {LEFT_ARM: (3.0, 0.0, -5.0), RIGHT_ARM: (-3.0, 0.0, -4.0)}


def test_the_bars_weight_comes_off_both_arms_and_the_mating_arm_pushes():
    """The squeeze cancels in the sum; each arm lifts half the 9 N; the left pushes 4 N along its line to the end."""
    lab = WeighingLab(SQUEEZED_BAR, left={"screw_stall_after": 6.0})
    plan = lab.plan()
    outcome = lab.run(plan)
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    weight = outcome.record["forces"]["weight"]
    assert weight["vertical_N"] == pytest.approx(-9.0, abs=1e-6) and weight["problem"] == ""
    lift = np.array([0.0, 0.0, 4.5])
    sent = {name: [force for force in lab.forces[name] if np.linalg.norm(force) > 0.0] for name in ARMS}
    np.testing.assert_allclose(sent[RIGHT_ARM], [lift], atol=1e-6)
    np.testing.assert_allclose(sent[LEFT_ARM], [lift + 4.0 * lab.line(plan, LEFT_ARM)], atol=1e-6)
    assert all(np.linalg.norm(lab.forces[name][-1]) == 0.0 for name in ARMS)
    assert any(level == "info" and "tool0 ended" in text and "at the end" in text for level, text in lab.ctx.logs)


class ContactLab(WeighingLab):
    """The squeezed bar, then once the rigid part starts the right arm meets its joint: 15 N back along world x."""

    def tick(self, t: float) -> None:
        """As the WeighingLab; the contact from the first trajectory sent on."""
        if getattr(self, "world_force", None) and self.robot._multi_arm.messages:
            self.world_force[RIGHT_ARM] = np.asarray(SQUEEZED_BAR[RIGHT_ARM]) + (-15.0, 0.0, 0.0)
        super().tick(t)


def test_the_weight_is_measured_before_anything_moves():
    """Measured at the insert's start, the 9 N comes off though the right arm meets its joint during the rigid part.

    Lab 2026-11-11, B26_J: a weight measured after the rigid part read that contact (15 N along the line) instead.
    """
    lab = ContactLab(dict(SQUEEZED_BAR), left={"screw_stall_after": 6.0})
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    weight = outcome.record["forces"]["weight"]
    assert weight["vertical_N"] == pytest.approx(-9.0, abs=1e-6) and weight["problem"] == ""
    assert lab.world_force[RIGHT_ARM][0] == pytest.approx(-18.0), "the contact came during the insert"


def test_a_sum_pointing_sideways_reads_as_a_contact_and_nothing_comes_off():
    """The readings sum more than WEIGHT_MAX_SIDEWAYS sideways: no lift, said loudly; the push still goes."""
    lab = WeighingLab({LEFT_ARM: (WEIGHT_MAX_SIDEWAYS + 1.0, 0.0, -5.0), RIGHT_ARM: (0.0, 0.0, -4.0)},
                      left={"screw_stall_after": 6.0})
    plan = lab.plan()
    outcome = lab.run(plan)
    assert outcome.status == STALLED
    assert "a contact?" in outcome.record["forces"]["weight"]["problem"]
    assert any(level == "warn" and "weight NOT taken off" in text for level, text in lab.ctx.logs)
    assert not any(np.linalg.norm(force) > 0.0 for force in lab.forces[RIGHT_ARM])
    pushed = next(force for force in lab.forces[LEFT_ARM] if np.linalg.norm(force) > 0.0)
    np.testing.assert_allclose(pushed, 4.0 * lab.line(plan, LEFT_ARM), atol=1e-6)


def test_the_runs_stiffness_is_set_for_the_compliant_part_and_set_back():
    """1000/1000/500 N/m on both arms before the switch, the controllers' 500s set back after; no weighing when off."""
    lab = Lab(left={"screw_stall_after": 6.0})
    settings = InsertForceSettings(cancel_weight=False, push_n=0.0, stiffness=(1000.0, 1000.0, 500.0))
    outcome = lab.run(lab.plan(), forces=settings)
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    for name in ARMS:
        service = lab.robot.arms[name].stiffness_service
        assert service.sets == [[1000.0, 1000.0, 500.0], [500.0, 500.0, 500.0]]
        assert outcome.record["stiffness"][name] == {"before": [500.0, 500.0, 500.0], "set": True}
    assert outcome.record["forces"]["weight"] is None and outcome.record["forces"]["push_arms"] == []


def test_a_stiffness_service_that_is_down_is_said_and_the_insert_goes_on():
    """No parameter service (an old controller): warned, nothing set back, the insert runs."""
    lab = Lab(left={"screw_stall_after": 6.0})
    for name in ARMS:
        lab.robot.arms[name].stiffness_service.up = False
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    assert any(level == "warn" and "NOT set" in text for level, text in lab.ctx.logs)
    assert all(lab.robot.arms[name].stiffness_service.sets == [] for name in ARMS)


def test_a_tight_screw_holds_its_arm_while_the_other_spirals_on():
    """Two-mate bar, spiral: the left screw tight at 3 s freezes its target and drops its push; the right searches."""
    lab = WeighingLab({LEFT_ARM: (0.0, 0.0, -4.0), RIGHT_ARM: (0.0, 0.0, -4.0)},
                      left={"screw_stall_after": 3.0}, right={"screw_stall_after": 8.0})
    plan = lab.plan(mating=ARMS, split_mm=30.0, spiral=(10.0, 2))
    outcome = lab.run(plan)
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    tight = outcome.record["forces"]["tight_at"][LEFT_ARM]
    after = {name: [target.position for target in lab.streamed(name) if target.update_time > tight + 0.15]
             for name in ARMS}
    assert len(after[LEFT_ARM]) > 10
    before = [target.position for target in lab.streamed(LEFT_ARM) if target.update_time <= tight]
    np.testing.assert_allclose(after[LEFT_ARM], [before[-1]] * len(after[LEFT_ARM]), atol=1e-12)
    assert np.ptp(np.asarray(after[RIGHT_ARM]), axis=0).max() > 1e-3
    lift = np.array([0.0, 0.0, 4.0])
    left = [force for force in lab.forces[LEFT_ARM] if np.linalg.norm(force) > 0.0]
    np.testing.assert_allclose(left, [lift + 4.0 * lab.line(plan, LEFT_ARM), lift], atol=1e-6)


def test_a_set_not_confirmed_is_set_back_all_the_same():
    """The controllers take the stiffness but never answer: said, and set back at the end anyway."""
    lab = Lab(left={"screw_stall_after": 6.0})
    for name in ARMS:
        lab.robot.arms[name].stiffness_service.confirms = False
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED
    for name in ARMS:
        assert lab.robot.arms[name].stiffness_service.sets == [[1000.0, 1000.0, 500.0], [500.0, 500.0, 500.0]]
    assert any(level == "warn" and "not confirmed" in text for level, text in lab.ctx.logs)


def test_an_arm_whose_stiffness_cannot_be_read_is_not_set():
    """No values to set back: no set goes out, said loudly; the insert runs."""
    lab = Lab(left={"screw_stall_after": 6.0})
    for name in ARMS:
        lab.robot.arms[name].stiffness_service.readable = False
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED and outcome.ok, outcome.reason
    assert all(lab.robot.arms[name].stiffness_service.sets == [] for name in ARMS)
    assert any(level == "warn" and "could not be read" in text for level, text in lab.ctx.logs)


def test_a_cancel_sets_the_stiffness_back_and_keeps_the_weight_off():
    """Cancelled while streaming: the stiffness goes back, and after the hold each arm lifts its share again."""
    lab = WeighingLab(SQUEEZED_BAR, left={"screw_stall_after": 30.0})

    async def scenario():
        task = asyncio.ensure_future(lab.work(lab.plan()))
        while len(lab.streamed(LEFT_ARM)) < 5 and not task.done():
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    for name in ARMS:
        assert lab.robot.arms[name].stiffness_service.sets[-1] == [500.0, 500.0, 500.0]
        np.testing.assert_allclose(lab.forces[name][-1], [0.0, 0.0, 4.5], atol=1e-6)


def test_the_last_screw_to_stall_is_recorded_tight_at_its_stall(monkeypatch):
    """Stalled mid-line, the other arm finishes its line first: the record keeps the stall's time, not the end's."""
    gates = []

    class KeptGate(insert.StallGate):
        """The gate, kept for its times."""

        def __init__(self, *args):
            """As the gate."""
            super().__init__(*args)
            gates.append(self)

    monkeypatch.setattr(insert, "StallGate", KeptGate)
    lab = WeighingLab(SQUEEZED_BAR, left={"screw_stall_after": 4.3})
    outcome = lab.run(lab.plan())
    assert outcome.status == STALLED, outcome.reason
    tight = outcome.record["forces"]["tight_at"][LEFT_ARM]
    assert tight == pytest.approx(gates[0].made + gates[0].stall_times[LEFT_ARM])
    assert lab.ctx.now() - tight > 0.5


def test_an_arm_refusing_its_lift_does_not_cost_the_other_arm_its_lift():
    """A cancel while the mating arm goes back: it refuses, the other arm still gets its share, the log names it."""
    lab = WeighingLab(SQUEEZED_BAR)
    lab.tick(0.0)
    sent = []
    lab.robot.arms[LEFT_ARM].send_target_wrench = lambda force: False
    lab.robot.arms[RIGHT_ARM].send_target_wrench = lambda force: sent.append(force) or True
    weight = BarWeight(np.array([0.0, 0.0, -9.0]), 10, "")
    InsertForces(lab.ctx, lab.robot, ARMS, {}, InsertForceSettings(), weight, "insert").lift_only(ARMS)
    assert len(sent) == 1 and np.linalg.norm(sent[0]) == pytest.approx(4.5)
    assert any(level == "warn" and LEFT_ARM in text and "NOT taken off" in text for level, text in lab.ctx.logs)


def test_a_stiffness_not_confirmed_back_is_the_one_the_next_part_sets_back():
    """The set back is lost: the next part must not take the insert's stiffness for the controller's own."""
    lab = Lab()
    arm, service = lab.robot.arms[LEFT_ARM], lab.robot.arms[LEFT_ARM].stiffness_service
    first, second = {}, {}
    asyncio.run(set_stiffness(lab.ctx, [arm], (1000.0, 1000.0, 500.0), first, label="insert"))
    service.confirms = False
    send_back([arm], first)
    service.confirms, service.stiffness = True, [1000.0, 1000.0, 500.0]
    asyncio.run(set_stiffness(lab.ctx, [arm], (1000.0, 1000.0, 500.0), second, label="insert"))
    assert second[LEFT_ARM]["before"] == [500.0, 500.0, 500.0]
    send_back([arm], second)
    assert service.stiffness == [500.0, 500.0, 500.0] and arm.stiffness_unrestored is None
