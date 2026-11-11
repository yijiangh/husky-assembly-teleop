"""The demo with the bar carried: Approach and Load replay the saved carry, the cycle runs from the loading pose."""

from __future__ import annotations

import asyncio
import shutil

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from test_demo_cycle import ARMS, answers, at_start, coupled_rows, motions, run_to_the_end
from test_demo_parts import authored_insert, demo_process
from test_demo_preview import Panel
from test_unscrew_retreat import INSERTED, NAMES, START, Lab

from husky_assembly_teleop.plugins.insertion_demo import plugin as plugin_module
from husky_assembly_teleop.plugins.insertion_demo.parts import (demo_steps, line_start, load_step, save_action,
                                                                saved_carry, with_insert_start)
from husky_assembly_teleop.plugins.insertion_demo.plugin import PREVIEW
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.shared.execution.basis import STALE_ARM
from scaffolding_assembly_core.process import read
from scaffolding_assembly_core.process.solutions import (MovementResult, Planner, Solution, Trajectory, solved_against,
                                                         write_solution)

pytestmark = pytest.mark.safety


def a_carry() -> np.ndarray:
    """A carry for the lab's Cindy: nine rows from a loading pose 2 cm off the insert's start to it, the grasp kept."""
    return coupled_rows(Lab(), n=9)[::-1].copy()


def carried(monkeypatch, rows, at: np.ndarray, carry: np.ndarray | str) -> Panel:
    """The panel with the Transfer workaround box unticked, Cindy at joints `at`, `carry` saved (or the refusal)."""
    panel = at_start(monkeypatch, rows, Lab(at=at, screw=ScaffoldingV3.IDLE, screw_stall_after=1.0))
    monkeypatch.setattr(plugin_module, "saved_carry",
                        lambda *_args: carry if isinstance(carry, str) else (list(NAMES), carry.copy()))
    panel.ctx.plugins["process"].transfer_as_free = False
    panel.draw()
    return panel


# --- --- --- --- --- the saved carry --- --- --- --- ---

def test_the_saved_carry_is_read_from_the_solution_unless_missing_or_stale(tmp_path):
    """None saved, saved and read back, no path for the transfer, and stale once the demo's Save moved the start."""
    loaded = demo_process()
    folder = tmp_path / "process"
    shutil.copytree(loaded.source, folder, ignore=shutil.ignore_patterns("solutions"))
    process = read(folder)
    transfer = loaded.steps[demo_steps(loaded, "B6_J")[0]].movement.id
    assert saved_carry(process, "B6_J", transfer).startswith("no carry is saved for B6_J")
    rows = a_carry()
    robot = process.actions["B6_J"].robot
    path = Trajectory(robot, tuple(NAMES), tuple(map(tuple, rows.tolist())))
    write_solution(process, Solution("B6_J", solved_against(process, "B6_J"), Planner("a test"),
                                     {transfer: MovementResult("solved", trajectory=path)}))
    names, read_back = saved_carry(process, "B6_J", transfer)
    assert names == list(NAMES)
    np.testing.assert_allclose(read_back, rows, atol=1e-9)
    assert "holds no path for B6_J_insert" in saved_carry(process, "B6_J", "B6_J_insert")
    _loaded, _transfer, insert, start, end = authored_insert()
    longer = line_start(robot_config(CINDY).model.urdf, ARMS, start, end, 0.060)
    save_action(with_insert_start(process, "B6_J", transfer, insert.id, longer, 0.060), "B6_J")
    assert saved_carry(process, "B6_J", transfer).startswith("the saved carry is stale")


def test_the_travel_to_load_is_the_actions_free_move_without_a_target():
    """B6_J has one, its first movement; B6_R has none."""
    loaded = demo_process()
    assert loaded.steps[load_step(loaded, "B6_J")].movement.id == "B6_J_load"
    assert load_step(loaded, "B6_R") == "B6_R has no travel to load"


# --- --- --- --- --- Approach and Load, the bar held --- --- --- --- ---

def test_held_approach_shows_the_saved_carry_without_a_planner_and_execute_runs_it(monkeypatch):
    """From the loading pose: the carry as saved, no request to the planner; Execute ends at the insert's start."""
    carry = a_carry()
    assert np.max(np.abs(carry[0] - START)) > STALE_ARM, "the loading pose is not the insert's start"
    panel = carried(monkeypatch, {}, carry[0], carry)

    async def scenario():
        assert "bar held" in panel.widget("").content
        shown = await panel.click("Move", "Approach")
        assert shown.startswith("approach planned (the saved carry, 9 waypoints)"), shown
        assert panel.planner.requests == [] and panel.lab.nothing_sent()
        np.testing.assert_allclose(panel.plugin._planned.rows, carry)
        assert "approach ready" in panel.widget("").content
        assert (await panel.click("Execute")).startswith("approach ARRIVED")
        np.testing.assert_allclose(panel.joints(), START, atol=1e-3)

    asyncio.run(scenario())


def test_held_load_runs_the_carry_reversed_from_the_insert_start(monkeypatch):
    """At the insert's start: the carry's rows backwards; Execute ends at the loading pose."""
    carry = a_carry()
    panel = carried(monkeypatch, {}, START, carry)

    async def scenario():
        shown = await panel.click("Move", "Load")
        assert shown.startswith("load planned (the saved carry reversed, 9 waypoints)"), shown
        np.testing.assert_allclose(panel.plugin._planned.rows, carry[::-1])
        assert panel.planner.requests == []
        assert (await panel.click("Execute")).startswith("load ARRIVED")
        np.testing.assert_allclose(panel.joints(), carry[0], atol=1e-3)

    asyncio.run(scenario())


def test_a_carry_is_only_shown_when_the_arms_are_not_at_its_start(monkeypatch):
    """Approach away from the loading pose, Load away from the insert's start: shown, Execute refuses, nothing sent."""
    carry = a_carry()
    panel = carried(monkeypatch, {}, START, carry)

    async def scenario():
        shown = await panel.click("Move", "Approach")
        assert "not to run: the arms do not stand at the loading pose" in shown, shown
        assert "approach refused" in panel.widget("").content and panel.plugin._ghost.shown is not None
        assert (await panel.click("Execute")).startswith("Execute refused: the approach shown is not to run")
        assert panel.lab.nothing_sent()

    asyncio.run(scenario())
    away = carried(monkeypatch, {}, carry[0], carry)

    async def back():
        shown = await away.click("Move", "Load")
        assert "not to run: the arms do not stand at the insert's start" in shown and "Return" in shown, shown
        assert away.lab.nothing_sent()

    asyncio.run(back())


def test_a_carry_that_is_missing_drifts_or_ends_off_the_insert_start_is_refused(monkeypatch):
    """No carry saved; a grasp that drifts along it; an end that is not the insert's start: nothing is shown."""
    carry = a_carry()
    drifting = carry.copy()
    drifting[4, :6] += 0.05
    for saved, why in (("no carry is saved for B6_J", "approach: no carry is saved for B6_J"),
                       (drifting, "approach: the saved carry is refused: "),
                       (carry[:-1], "approach: the saved carry does not end at the insert's start for Insert mm")):
        panel = carried(monkeypatch, {}, carry[0], saved)

        async def scenario():
            shown = await panel.click("Move", "Approach")
            assert shown.startswith(why), shown
            assert panel.plugin._planned is None and panel.lab.nothing_sent()

        asyncio.run(scenario())


def test_unticking_or_ticking_the_box_drops_the_planned_move(monkeypatch):
    """A move planned for one way of travelling is not run the other way."""
    carry = a_carry()
    panel = carried(monkeypatch, {}, carry[0], carry)

    async def scenario():
        await panel.click("Move", "Approach")
        assert panel.plugin._planned is not None
        panel.ctx.plugins["process"].transfer_as_free = True
        panel.draw()
        assert panel.plugin._planned is None and "Transfer workaround" in panel.plugin.outcome
        assert "bar free" in panel.widget("").content

    asyncio.run(scenario())


# --- --- --- --- --- Load, the tools empty --- --- --- --- ---

def test_free_load_plans_the_travel_to_load_to_the_carrys_first_row(monkeypatch):
    """The box ticked: the planner gets the travel-to-load step with the loading joints as the goal; Execute arrives."""
    carry = a_carry()
    panel = Panel(monkeypatch, {"demo load": np.linspace(INSERTED, carry[0], 5)})
    monkeypatch.setattr(plugin_module, "saved_carry", lambda *_args: (list(NAMES), carry.copy()))

    async def scenario():
        assert "bar free" in panel.widget("").content
        assert (await panel.click("Move", "Load")).startswith("load planned (a planner, 5 waypoints)")
        request, = panel.planner.requests
        demo = panel.plugin._demo(panel.ctx)
        assert request.label == "demo load" and request.step.movement.id == "B6_J_load"
        assert request.step is demo.loaded.steps[load_step(demo.loaded, "B6_J")]
        assert request.goal_joints == dict(zip(NAMES, carry[0].tolist()))
        assert (await panel.click("Execute")).startswith("load ARRIVED")
        np.testing.assert_allclose(panel.joints(), carry[0], atol=1e-3)

    asyncio.run(scenario())
    monkeypatch.setattr(plugin_module, "saved_carry", lambda *_args: "no carry is saved for B6_J")
    assert asyncio.run(panel.click("Move", "Load")) == "load: no carry is saved for B6_J"


# --- --- --- --- --- the cycle, the bar held --- --- --- --- ---

def test_a_held_cycle_carries_the_bar_in_and_back_to_the_loading_pose(monkeypatch):
    """From the loading pose: the carry, the insert, the retreat, the return, the carry back; four joint motions."""
    carry = a_carry()
    holder: list = []
    panel = carried(monkeypatch, answers(holder), carry[0], carry)
    holder.append(panel)

    async def scenario():
        steps = await run_to_the_end(panel, "Cycle")
        assert panel.plugin.outcome.startswith("cycle done: back at the loading pose"), panel.plugin.outcome
        names = [name for name, _t in steps if name and name != "cycle: rest" and "plan" not in name]
        assert names == ["cycle: approach", "cycle: insert", "cycle: settle", "cycle: retreat", "cycle: return",
                         "cycle: load"], names
        assert [request.label for request in panel.planner.requests] == ["demo cycle insert", "demo return"]
        assert motions(panel) == 4, "the carry, the insert's rigid part, the return, the carry back"
        np.testing.assert_allclose(panel.joints(), carry[0], atol=1e-3)
        assert panel.plugin._planned is None

    asyncio.run(scenario())


def test_a_held_cycle_away_from_the_loading_pose_stops_before_anything_moves(monkeypatch):
    """At the insert's start the carry is only shown: the cycle stops at its first plan."""
    carry = a_carry()
    panel = carried(monkeypatch, {}, START, carry)

    async def scenario():
        await run_to_the_end(panel, "Cycle")
        assert panel.plugin.outcome.startswith("cycle stopped at plan approach"), panel.plugin.outcome
        assert "loading pose" in panel.plugin.outcome and panel.lab.nothing_sent()

    asyncio.run(scenario())


def test_a_held_preview_cycle_has_the_carry_at_both_ends(monkeypatch):
    """The carry, the insert, the back-out, the return and the carry back, as one path from the loading pose."""
    carry = a_carry()
    panel = carried(monkeypatch, {"demo preview insert": np.linspace(START, INSERTED, 5),
                                  "demo preview return": np.array([START, START])}, START, carry)

    async def scenario():
        shown = await panel.click("Preview cycle")
        assert shown.startswith(f"{PREVIEW} shown (carry, insert, back-out, return, carry back), not to run"), shown
        planned = panel.plugin._planned
        assert planned.parts == 5
        np.testing.assert_allclose(planned.rows[0], carry[0])
        np.testing.assert_allclose(planned.rows[-1], carry[0])
        assert panel.lab.nothing_sent()

    asyncio.run(scenario())
