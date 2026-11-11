"""The demo's Return move and its one-click cycle: insert, settle, retreat, then the checked way back."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest
from test_demo_preview import Panel
from test_unscrew_retreat import INSERTED, NAMES, START, Lab

from husky_assembly_teleop.plugins.insertion_demo.parts import COUPLING, JOINT_STEP, return_problem
from husky_assembly_teleop.plugins.insertion_demo.plugin import PREVIEW, SETTLE_DEFAULT, InsertionDemoPlugin
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.world.tcp import tool0_in_base_link
from scaffolding_assembly_core.geometry import Pose, compose, invert

pytestmark = pytest.mark.safety

ARMS = ["left_ur_arm", "right_ur_arm"]
#: How far the bar slides, metres in the husky frame, along the coupled test paths.
SHIFT = (0.02, 0.0, 0.0)


def coupled_rows(lab: Lab, shift: tuple[float, float, float] = SHIFT, n: int = 5) -> np.ndarray:
    """A path of `n` rows from START along which both tool0s slide `shift` together, the grasp kept.

    Each row's joints are solved on the URDF by the fake arms' own least-squares steps.
    """
    rows = [np.array(START, dtype=float)]
    at_start = dict(zip(NAMES, START))
    for s in np.linspace(0.0, 1.0, n)[1:]:
        row = []
        for k, (name, driver) in enumerate(zip(ARMS, lab.drivers[:2])):
            base = lab.robot.arms[name].base_in_husky
            in_husky = compose(base, tool0_in_base_link(lab.urdf, name, at_start))
            there = Pose(tuple(np.add(in_husky.position, np.multiply(shift, s))), in_husky.orientation)
            joints = rows[-1][6 * k:6 * k + 6]
            for _step in range(12):
                joints = driver._follow(joints, compose(invert(base), there))
            row += joints.tolist()
        rows.append(np.array(row))
    return np.array(rows)


def at_start(monkeypatch, rows, lab: Lab | None = None, **planner) -> Panel:
    """The panel on a Cindy at the insert's start, the lab's START, her joint screws stalling 1 s after they start."""
    monkeypatch.setattr(InsertionDemoPlugin, "_insert_start",
                        staticmethod(lambda _demo: (ARMS, dict(zip(NAMES, START)))))
    lab = lab or Lab(at=START, screw=ScaffoldingV3.IDLE, screw_stall_after=1.0)
    return Panel(monkeypatch, rows, lab, **planner)


async def run_to_the_end(panel: Panel, group: str, button: str = "") -> list[tuple[str, float]]:
    """Click, then draw every tick until the task ends; each step the chip showed, with the ROS time it began."""
    panel.widget(group).callback(*([button] if button else []))
    steps: list[tuple[str, float]] = []
    while not panel.plugin._task.done():
        if not steps or steps[-1][0] != panel.plugin.step:
            steps.append((panel.plugin.step, panel.ctx.now()))
        panel.draw()
        await asyncio.sleep(0)
    panel.draw()
    return steps


def motions(panel: Panel) -> int:
    """How many dual-arm joint motions went out."""
    return len(panel.lab.robot._multi_arm.messages)


# --- --- --- --- --- the checks of the way back --- --- --- --- ---

def test_the_way_back_passes_with_the_grasp_kept_and_is_refused_for_a_drift_or_a_jump():
    """A coupled slide is fine; the left arm sliding alone drifts the grasp; a joint jump between waypoints is named."""
    lab = Lab()
    fine = coupled_rows(lab)
    assert return_problem(lab.robot, NAMES, fine, ARMS) == ""
    drifting = fine.copy()
    drifting[:, 6:] = START[6:]
    problem = return_problem(lab.robot, NAMES, drifting, ARMS)
    assert problem.startswith("the arms' relative pose drifts") and f"(over {COUPLING[0] * 1e3:.0f} mm" in problem
    jumping = fine.copy()
    jumping[2:, 3] += 2 * JOINT_STEP
    problem = return_problem(lab.robot, NAMES, jumping, ARMS)
    assert problem.startswith("a joint jumps 10.") and "between waypoints 1 and 2 (over 5°)" in problem
    assert return_problem(lab.robot, NAMES, fine[:1], ARMS) == "the path has fewer than two waypoints"


# --- --- --- --- --- Return --- --- --- --- ---

def test_return_plans_the_insert_backwards_to_the_start_and_execute_runs_it(monkeypatch):
    """Return asks the insert's planner for the start's tool0 poses, shows the path; Execute brings the arms there."""
    shifted = coupled_rows(Lab())
    lab = Lab(at=shifted[-1])
    panel = at_start(monkeypatch, {"demo return": shifted[::-1]}, lab)
    flanges = InsertionDemoPlugin._flanges(panel.plugin._demo(panel.ctx))

    async def scenario():
        assert (await panel.click("Move", "Return")).startswith("return planned (a planner, 5 waypoints")
        request = panel.planner.requests[-1]
        assert request.label == "demo return" and set(request.goal_poses) == set(flanges.values())
        for arm, end in lab.ends().items():
            np.testing.assert_allclose(request.goal_poses[flanges[arm]].position, end.position, atol=1e-9)
        assert "return ready" in panel.widget("").content and panel.lab.nothing_sent()
        assert (await panel.click("Execute")).startswith("return ARRIVED")
        np.testing.assert_allclose(panel.joints(), START, atol=1e-3)

    asyncio.run(scenario())


def test_return_refuses_a_path_that_drifts_or_ends_off_the_start(monkeypatch):
    """A path failing its checks (the path itself, then its end error per flange) is shown, and Execute refuses it."""
    drifting = np.linspace(INSERTED, START, 5)
    panel = at_start(monkeypatch, {"demo return": drifting}, Lab(at=INSERTED))

    async def scenario():
        assert "refused: the arms' relative pose drifts" in await panel.click("Move", "Return")
        planned = panel.plugin._planned
        assert planned.step == "return" and planned.refused.startswith("the arms' relative pose drifts")
        assert "return refused" in panel.widget("").content and panel.plugin._ghost.shown is not None
        assert panel.widget("Preview time").max > 1.0
        refusal = await panel.click("Execute")
        assert refusal.startswith("Execute refused: the return shown is not to run: the arms' relative pose drifts")
        assert panel.plugin._planned is planned and panel.lab.nothing_sent()
        panel.planner.rows = {"demo return": coupled_rows(panel.lab)[::-1]}
        panel.planner.end_error = {InsertionDemoPlugin._flanges(panel.plugin._demo(panel.ctx))["right_ur_arm"]:
                                   (0.02, 0.0)}
        assert "right_ur_arm's tool0 ends 20.0 mm" in await panel.click("Move", "Return")
        assert panel.plugin._planned.refused.startswith("right_ur_arm's tool0 ends") and panel.lab.nothing_sent()

    asyncio.run(scenario())


# --- --- --- --- --- Cycle --- --- --- --- ---

def answers(panel_holder: list) -> dict:
    """The planner's answers for a cycle: the insert's path, and the way back from wherever the arms stand then."""
    return {"demo cycle insert": np.linspace(START, INSERTED, 5),
            "demo return": lambda: np.linspace(panel_holder[0].joints(), START, 5)}


def test_a_cycle_inserts_settles_retreats_and_returns_to_the_start(monkeypatch):
    """One click: tighten to the stall, the pause, loosen and back out, the planned way back; done at the start."""
    holder: list = []
    panel = at_start(monkeypatch, answers(holder))
    holder.append(panel)
    plan_window, exec_window = panel.ctx.panels

    async def scenario():
        assert SETTLE_DEFAULT == 0.0 and panel.widget("Settle s").value == 0.0
        panel.widget("Settle s").value = 5.0
        steps = await run_to_the_end(panel, "Cycle")
        assert panel.plugin.outcome.startswith("cycle done (return ARRIVED"), panel.plugin.outcome
        # * The plan phases pass within a tick here: the fake planner answers at once.
        names = [name for name, _t in steps if name]
        assert names == ["cycle: insert", "cycle: settle", "cycle: retreat", "cycle: rest", "cycle: return"], names
        began = dict(steps)
        assert began["cycle: retreat"] - began["cycle: settle"] >= 5.0 - 0.05
        assert [request.label for request in panel.planner.requests] == ["demo cycle insert", "demo return"]
        kinds = panel.lab.steps()
        assert kinds.index("TIGHTEN M2") < kinds.index("LOOSEN M2")
        assert motions(panel) == 2, "the insert's rigid part, then the return"
        assert panel.lab.running() == [JOINT, JOINT]
        np.testing.assert_allclose(panel.joints(), START, atol=1e-3)
        assert plan_window.calls.count("expand") == 2 and exec_window.calls.count("expand") == 2
        assert panel.plugin._planned is None and "idle" in panel.widget("").content

    asyncio.run(scenario())


def test_a_cycle_stops_when_the_insert_does_not_stall(monkeypatch):
    """The joint screws never stall: the insert ends NO_STALL and the cycle stops there; nothing loosens."""
    holder: list = []
    panel = at_start(monkeypatch, answers(holder), Lab(at=START, screw=ScaffoldingV3.IDLE, screw_stall_after=None))
    holder.append(panel)

    async def scenario():
        await run_to_the_end(panel, "Cycle")
        assert panel.plugin.outcome.startswith("cycle stopped at insert: insert NO_STALL"), panel.plugin.outcome
        assert "LOOSEN M2" not in panel.lab.steps() and panel.lab.running() == [JOINT, JOINT]
        assert len(panel.planner.requests) == 1

    asyncio.run(scenario())


def test_a_cycle_stops_when_the_way_back_fails_its_checks(monkeypatch):
    """The planner's way back drifts: the cycle stops after the retreat, no trajectory goes out after it."""
    holder: list = []
    rows = answers(holder)
    rows["demo return"] = lambda: np.array([holder[0].joints(), START + 2 * JOINT_STEP])
    panel = at_start(monkeypatch, rows)
    holder.append(panel)

    async def scenario():
        await run_to_the_end(panel, "Cycle")
        assert panel.plugin.outcome.startswith("cycle stopped at plan return: return: refused: a joint jumps")
        assert "LOOSEN M2" in panel.lab.steps() and motions(panel) == 1, "only the insert's rigid part"
        assert panel.lab.running() == [JOINT, JOINT]
        # * The refused way back stays in the preview, to look at.
        assert panel.plugin._planned.refused.startswith("a joint jumps")
        assert "return refused" in panel.widget("").content and panel.plugin._ghost.shown is not None

    asyncio.run(scenario())


def test_preview_cycle_shows_the_whole_cycle_as_one_path_and_sends_nothing(monkeypatch):
    """Preview cycle at the insert's start: the insert, the back-out and the return planned and strung together."""
    # * Retreat mm equals Insert mm here: the back-out already ends at the insert's start, the return adds nothing.
    panel = at_start(monkeypatch, {"demo preview insert": np.linspace(START, INSERTED, 5),
                                   "demo preview return": np.array([START, START])})

    async def scenario():
        shown = await panel.click("Preview cycle")
        assert shown.startswith(f"{PREVIEW} shown (insert, back-out, return"), shown
        assert [request.label for request in panel.planner.requests] == ["demo preview insert", "demo preview return"]
        planned = panel.plugin._planned
        assert planned.step == PREVIEW and planned.parts == 3 and planned.refused
        np.testing.assert_allclose(planned.rows[0], START)
        np.testing.assert_allclose(planned.rows[4], INSERTED)
        np.testing.assert_allclose(planned.rows[-1], START)
        assert len(planned.rows) == 6, "the insert's five rows, then the back-out's end"
        assert PREVIEW in panel.widget("").content and panel.plugin._ghost.shown is not None
        assert panel.widget("Preview time").max == pytest.approx(30.0, abs=0.1), "three parts of Motion time each"
        assert (await panel.click("Execute")).startswith(f"Execute refused: the {PREVIEW} shown is not to run")
        assert panel.lab.nothing_sent()

    asyncio.run(scenario())


def test_preview_cycle_away_from_the_start_plans_the_transit_first(monkeypatch):
    """With the arms off the insert's start the preview begins with the transit from where they stand."""
    panel = at_start(monkeypatch, {"demo preview transit": np.linspace(INSERTED, START, 3),
                                   "demo preview insert": np.linspace(START, INSERTED, 5),
                                   "demo preview return": np.array([START, START])}, Lab(at=INSERTED))

    async def scenario():
        shown = await panel.click("Preview cycle")
        assert shown.startswith(f"{PREVIEW} shown (transit, insert, back-out, return)"), shown
        assert panel.planner.requests[0].label == "demo preview transit" and panel.lab.nothing_sent()

    asyncio.run(scenario())


def test_cancel_during_the_settle_ends_the_cycle_and_sends_nothing_more(monkeypatch):
    """Cancel while the arms rest after the insert: cycle cancelled, no loosen, the arms under joint control."""
    holder: list = []
    panel = at_start(monkeypatch, answers(holder))
    holder.append(panel)

    async def scenario():
        panel.widget("Cycle").callback()
        while panel.plugin.step != "cycle: settle":
            panel.draw()
            await asyncio.sleep(0)
        sent = len(panel.lab.events())
        panel.widget("Path").callback("Cancel")
        with pytest.raises(asyncio.CancelledError):
            await panel.plugin._task
        panel.draw()
        assert panel.plugin.outcome == "cycle cancelled" and len(panel.lab.events()) == sent
        assert panel.lab.running() == [JOINT, JOINT]

    asyncio.run(scenario())
