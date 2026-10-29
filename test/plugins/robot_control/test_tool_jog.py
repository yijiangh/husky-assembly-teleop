"""The tool0 jog of robot_control: a held button streams points along tool0's own axis, a release holds the arm."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import ArmDriver, FakeContext, fake_arm, fake_node, fake_robot
from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.robot_control.plugin import RobotControlPlugin
from husky_assembly_teleop.plugins.robot_control.teleop_lease import IDLE_AFTER
from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER
from husky_assembly_teleop.robot_interface.robot_lease import Lease
from husky_assembly_teleop.robot_interface.tool_step import _difference, tool0_pose
from husky_assembly_teleop.shared.teleop.dpad import HOLD_TIMEOUT
from husky_assembly_teleop.world.clearance import ARM_STOP_DISTANCE
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety

ARM, SERIAL, OTHER = "ur_arm", "a200-0804", "a200-0805"
#: Belle's URDF, whose arm is ur_arm.
URDF = robot_config_from_serial("0805", DATA).model.urdf
#: An arm pose well away from singularities.
JOINTS = np.radians([10.0, -100.0, 90.0, -80.0, -90.0, 30.0])
TICK = 0.05


class Bench:
    """A robot_control plugin over one or two robots with a real arm each, run by an ArmDriver, on Belle's URDF."""

    def __init__(self, robots: tuple[str, ...] = (SERIAL,), mm_s: float = 5.0):
        """Every arm at JOINTS under the joint controller, the jog speed at `mm_s`, Clearance not built."""
        self.ctx = FakeContext(name="robot_control", step=self._step)
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.plugin, self.arms, self.drivers = RobotControlPlugin(), {}, []
        for serial in robots:
            arm = fake_arm(node, ARM, JOINTS)
            arm.urdf = URDF
            self.ctx.world.robots[serial] = robot = fake_robot(node, serial, [arm])
            self.arms[serial], widgets = arm, SimpleNamespace(
                serial=serial, arm=arm, jog_speed=SimpleNamespace(value=1.0), jog_clearance=None, free_drive_keep=None,
                tool_jog=SimpleNamespace(speed=SimpleNamespace(value=mm_s), readout=SimpleNamespace(content="")))
            self.plugin._arms.append(widgets)
            self.drivers.append(ArmDriver(arm, robot))
        self.anchor = tool0_pose(URDF, ARM, JOINTS)
        self._step(self.ctx.t)

    def _step(self, t: float) -> None:
        for driver in self.drivers:
            driver.tick(t)

    def ticks(self, count: int, press: tuple[int, float] | None = None, serial: str = SERIAL) -> None:
        """Run `count` ticks with `press` (axis, direction) held on robot `serial`'s arm, or nothing held."""
        for _ in range(count):
            self.ctx.t += TICK
            self._step(self.ctx.t)
            if press is not None:
                self.plugin._on_tool_jog_hold(self.ctx, (serial, ARM), *press)
            self.plugin._update_tool_jogs(self.ctx, self.ctx.now())

    def progress(self, serial: str = SERIAL) -> np.ndarray:
        """Tool0 now seen from the press pose: shift (m) then turn (rad) in tool0's own axes."""
        return _difference(tool0_pose(URDF, ARM, self.arms[serial].joint_vector()), self.anchor, local=True)

    def sent(self, serial: str = SERIAL) -> list:
        """Every trajectory message robot `serial`'s arm was sent."""
        return self.arms[serial]._trajectory.messages


def test_a_held_button_streams_one_point_per_tick_and_tool0_moves_along_its_axis():
    """At 5 mm/s for a second, +x: about 5 mm along tool0's x, nothing sideways, no turn; one point per tick."""
    bench = Bench()
    bench.ticks(20, press=(0, 1.0))
    assert len(bench.sent()) == 20 and all(len(message.points) == 1 for message in bench.sent())
    along, beside, turn = bench.progress()[0], bench.progress()[1:3], bench.progress()[3:]
    assert 3e-3 < along <= 5.1e-3, along
    assert np.linalg.norm(beside) < 5e-5 and np.degrees(np.linalg.norm(turn)) < 0.01
    assert "jogging along tool0 x" in bench.plugin._tool_jog_readout((SERIAL, ARM), bench.plugin._arms[0])


def test_a_held_turn_button_turns_tool0_about_its_own_axis():
    """rz+ at 10 mm/s is 2 °/s: after a second tool0 turned about 2° about its z and stayed in place."""
    bench = Bench(mm_s=10.0)
    bench.ticks(20, press=(5, 1.0))
    progress = bench.progress()
    assert np.linalg.norm(progress[:3]) < 5e-5
    assert np.degrees(progress[3:5]).max() < 0.01 and 1.2 < np.degrees(progress[5]) <= 2.05


def test_a_release_holds_the_arm_once_and_the_lease_goes_back_when_idle():
    """Once the reports stop, one hold goes out, the jog is forgotten, and after IDLE_AFTER the lease is released."""
    bench = Bench()
    bench.ticks(4, press=(1, -1.0))
    assert bench.plugin._leases.held(SERIAL) and len(bench.sent()) == 4
    bench.ticks(int(2 * HOLD_TIMEOUT / TICK))
    holds = [message for message in bench.sent() if message.points[0].velocities]
    assert len(holds) == 1 and not bench.plugin._tool_jogging
    assert any("hold sent" in line for _level, line in bench.ctx.logs)
    for _ in range(int(IDLE_AFTER / TICK) + 25):
        bench.ticks(1)
        bench.arms[SERIAL].state.is_executing = False
        bench.plugin._leases.release_idle(bench.ctx, lambda serial: bench.plugin._busy(bench.ctx.now(), serial))
    assert not bench.plugin._leases.held(SERIAL)


def test_a_press_with_clearance_ticked_streams_only_once_its_clearance_checks():
    """With Clearance ticked, the first tick starts a 1 cm clearance and sends nothing; points stream once it checks."""
    bench = Bench()
    bench.plugin._arms[0].jog_clearance = SimpleNamespace(value=True)

    async def scenario() -> None:
        bench.ticks(1, press=(0, 1.0))
        assert not bench.sent(), "the clearance comes first"
        await asyncio.sleep(0)
        assert (SERIAL, ARM) in bench.plugin._jog_checked
        bench.ticks(3, press=(0, 1.0))
        assert len(bench.sent()) == 3
        clearance, = bench.ctx.clearances
        assert (clearance.serials, clearance.margin) == ((SERIAL,), ARM_STOP_DISTANCE)
        bench.plugin._arms[0].jog_clearance.value = False
        await bench.plugin._jog_guards[(SERIAL, ARM)]

    asyncio.run(scenario())
    assert not bench.plugin._jog_checked


def test_a_press_under_the_compliance_controller_is_refused_and_sends_nothing():
    """The jog needs the joint controller: a chip says so, no point goes out."""
    bench = Bench()
    bench.arms[SERIAL].state.controllers.active = CARTESIAN_COMPLIANCE_CONTROLLER
    bench.ticks(3, press=(0, 1.0))
    assert not bench.sent() and bench.plugin._refusals[(SERIAL, ARM)].label == "jog refused"
    assert "joint controller" in bench.plugin._refusals[(SERIAL, ARM)].reason


def test_a_press_on_a_robot_another_plugin_holds_sends_nothing():
    """Leased elsewhere: nothing is sent, and the "lease refused" chip names the holder."""
    bench = Bench()
    Lease("arm_planner: commit plan 2", (bench.ctx.world.robots[SERIAL].lease_slot,))
    bench.ticks(3, press=(0, 1.0))
    assert not bench.sent() and not bench.plugin._leases.held(SERIAL)
    assert "arm_planner" in bench.plugin._refusals[(SERIAL, ARM)].reason


def test_one_arm_jogs_at_a_time():
    """A second robot's press while the first jogs is refused; once the first releases, the second may jog."""
    bench = Bench(robots=(SERIAL, OTHER))
    bench.ticks(3, press=(0, 1.0))
    bench.ctx.t += TICK
    bench._step(bench.ctx.t)
    bench.plugin._on_tool_jog_hold(bench.ctx, (SERIAL, ARM), 0, 1.0)
    bench.plugin._on_tool_jog_hold(bench.ctx, (OTHER, ARM), 0, 1.0)
    bench.plugin._update_tool_jogs(bench.ctx, bench.ctx.now())
    assert not bench.sent(OTHER) and "one arm at a time" in bench.plugin._refusals[(OTHER, ARM)].reason
    bench.ticks(int(2 * HOLD_TIMEOUT / TICK))
    bench.ticks(3, press=(2, 1.0), serial=OTHER)
    assert len(bench.sent(OTHER)) == 3


def test_a_jog_the_solver_refuses_stops_until_the_button_is_released():
    """At the wrist singularity a turn about x cannot be solved: a "jog stopped" chip, and nothing more is sent."""
    bench = Bench()
    singular = JOINTS.copy()
    singular[4] = 0.0
    arm = bench.arms[SERIAL]
    arm.state.joint_positions = {f"{ARM}_{name}": float(value) for name, value in zip(UR_JOINT_NAMES, singular)}
    bench.drivers[0]._times = None
    bench.ticks(5, press=(3, 1.0))
    assert not bench.sent() and bench.plugin._refusals[(SERIAL, ARM)].label == "jog stopped"
    assert bench.plugin._tool_jogging[(SERIAL, ARM)].stopped
