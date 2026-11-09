"""The demo's panel: a Move button only plans and shows its move; Execute runs the move shown, once.

The plot windows open by themselves: the Plan plots for a planned move, the Exec plots for a compliant part.
"""

from __future__ import annotations

import asyncio
import math
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from test_demo_parts import demo_process
from test_unscrew_retreat import BASE, INSERTED, NAMES, START, Lab

from husky_assembly_teleop.plugins.insertion_demo import plugin as plugin_module
from husky_assembly_teleop.plugins.insertion_demo.parts import back_out_rows, timed_preview
from husky_assembly_teleop.plugins.insertion_demo.plugin import SECONDS_DEFAULT, InsertionDemoPlugin
from husky_assembly_teleop.shared.execution.basis import STALE_ARM

pytestmark = pytest.mark.safety

SERIAL = "a200-0806"


class Ghost:
    """The preview robot: keeps the joints it was last shown at, None while hidden."""

    def __init__(self, *_args):
        """Hidden."""
        self.shown = None

    def show(self, base, joints, parts=None) -> None:
        """Keep where it stands."""
        self.shown = (base, dict(joints))

    def hide(self) -> None:
        """Hidden again."""
        self.shown = None


class Planner:
    """The motion planner: answers every request with `rows` and keeps the requests."""

    def __init__(self, rows: np.ndarray):
        """Nothing asked yet."""
        self.rows, self.requests = rows, []

    async def request_plan(self, request):
        """A solved route along `rows`."""
        self.requests.append(request)
        trajectory = SimpleNamespace(joint_names=list(NAMES), positions=self.rows.tolist())
        return SimpleNamespace(ok=True, result=SimpleNamespace(trajectory=trajectory, reason=""),
                               planner="a planner", waypoints=len(self.rows), warnings=())

    def request_cancel(self) -> None:
        """Nothing to cancel."""


class Panel:
    """The demo's tab on the lab's Cindy (inserted, under joint control), the demo's process loaded."""

    def __init__(self, monkeypatch, rows: np.ndarray):
        """Set the plugin up; the planner answers with `rows`."""
        monkeypatch.setattr(plugin_module, "RobotGhost", Ghost)
        self.lab, self.planner = Lab(), Planner(rows)
        self.ctx = ctx = self.lab.ctx
        loaded = demo_process()
        ctx.config = SimpleNamespace(robots=(robot_config(CINDY),), drive_root=None)
        ctx.world.robots[SERIAL] = self.lab.robot
        # * The plugin takes the robot's lease itself: the lab's own lease is given back first.
        self.lab.robot.lease_slot.lease = None
        ctx.kinematics = SimpleNamespace(base_pose=lambda _serial: BASE, joints=lambda _serial: {})
        ctx.plugins.update(motion_planner=self.planner, process=SimpleNamespace(
            selection=SimpleNamespace(process=loaded.process), obstacles_collide=False,
            request_select=lambda _index: None))
        self.plugin = InsertionDemoPlugin()
        self.plugin.setup(ctx)
        self.draw()

    def widget(self, label: str):
        """The widget with this label."""
        return next(widget for widget in self.ctx.gui.widgets if widget.label == label)

    def draw(self) -> None:
        """One tick of the panel."""
        self.plugin.update(self.ctx)
        self.plugin.draw(self.ctx)

    async def click(self, group: str, button: str = "") -> str:
        """Click a button (of a group, or one of its own), wait for the task it started; the panel's line after."""
        before = self.plugin._task
        self.widget(group).callback(*([button] if button else []))
        if self.plugin._task is not before:
            await self.plugin._task
        self.draw()
        return self.plugin.outcome

    def joints(self) -> np.ndarray:
        """Both arms' measured joints."""
        return np.concatenate([arm.joint_vector() for arm in self.lab.robot.arms.values()])


# --- --- --- --- --- the preview's paths --- --- --- --- ---

def test_a_preview_is_timed_as_its_motion_or_straight_for_the_back_out():
    """A joint motion: the asked time or the arms' minimum, whichever is longer; the back-out: two rows, the time."""
    rows = np.linspace(INSERTED, START, 5)
    path, minimum = timed_preview(rows, 10.0)
    assert path.duration == pytest.approx(10.0, abs=0.1) and 0.0 < minimum < 10.0
    np.testing.assert_allclose(path.sample(0.0), INSERTED, atol=1e-9)
    np.testing.assert_allclose(path.sample(path.duration), START, atol=1e-9)
    assert timed_preview(rows, 0.0)[0].duration == pytest.approx(minimum)
    straight, none = timed_preview(rows, 4.0, limits=False)
    assert straight.duration == 4.0 and len(straight.points) == 2 and np.isnan(none)
    assert timed_preview(np.array([START, START]), 10.0) == "the arms are already there: nothing to move"


def test_the_back_out_path_goes_from_the_live_joints_to_the_insert_start():
    """Two rows over both arms' joints: where the arms stand, then the start."""
    lab = Lab()
    names, rows = back_out_rows(lab.robot, dict(zip(NAMES, START)), ["left_ur_arm", "right_ur_arm"])
    assert names == list(NAMES)
    np.testing.assert_allclose(rows, [INSERTED, START], atol=1e-12)


# --- --- --- --- --- plan, preview, Execute --- --- --- --- ---

def test_approach_only_plans_and_shows_then_execute_runs_it_once(monkeypatch):
    """Approach: the planner is asked, nothing is sent, the preview stands at the end; Execute: the arms arrive."""
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))

    async def scenario():
        planned = await panel.click("Move", "Approach")
        assert planned.startswith("approach planned (a planner, 5 waypoints): the motion takes 10.0 s")
        assert len(panel.planner.requests) == 1 and panel.lab.nothing_sent()
        assert "approach ready" in panel.widget("").content  # the status line is the first unlabelled widget
        slider, ghost = panel.widget("Preview time"), panel.plugin._ghost
        assert slider.max == pytest.approx(10.0, abs=0.1) and slider.value == pytest.approx(slider.max)
        np.testing.assert_allclose([ghost.shown[1][name] for name in NAMES], START, atol=1e-9)
        # * Scrubbing to 0 shows where the arms stand; a longer motion time stretches the preview.
        slider.callback(0.0)
        panel.draw()
        np.testing.assert_allclose([ghost.shown[1][name] for name in NAMES], INSERTED, atol=1e-9)
        panel.widget("Motion time").value = 20.0
        panel.widget("Motion time").callback(20.0)
        panel.draw()
        assert slider.max == pytest.approx(20.0, abs=0.1) and panel.lab.nothing_sent()
        assert (await panel.click("Execute")).startswith("approach ARRIVED")
        np.testing.assert_allclose(panel.joints(), START, atol=1e-3)
        assert ghost.shown is None and "no plan" in panel.widget("").content
        assert "nothing planned" in await panel.click("Execute")

    asyncio.run(scenario())


def test_execute_refuses_a_plan_the_arms_left(monkeypatch):
    """The arms over STALE_ARM from the plan's start: the chip says stale, Execute sends nothing."""
    panel = Panel(monkeypatch, np.linspace(INSERTED + 2 * STALE_ARM, START, 5))

    async def scenario():
        await panel.click("Move", "Approach")
        assert "approach stale" in panel.widget("").content
        assert "plan again" in await panel.click("Execute")
        assert panel.lab.nothing_sent() and panel.plugin._planned is None

    asyncio.run(scenario())


def test_cancel_drops_a_planned_move_and_insert_away_from_its_start_plans_nothing(monkeypatch):
    """Cancel with nothing running forgets the plan; Insert off the insert's start is refused before any planning."""
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))

    async def scenario():
        await panel.click("Move", "Approach")
        assert await panel.click("Path", "Cancel") == "the planned approach dropped"
        assert panel.plugin._planned is None and panel.plugin._ghost.shown is None
        assert "press Approach first" in await panel.click("Move", "Insert")
        assert len(panel.planner.requests) == 1 and panel.plugin._planned is None and panel.lab.nothing_sent()

    asyncio.run(scenario())


def test_retreat_shows_the_back_out_before_anything_is_sent(monkeypatch):
    """Retreat: no planner, a straight preview over the motion time from where the arms stand; nothing sent yet."""
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))

    async def scenario():
        assert (await panel.click("Move", "Retreat")).startswith("retreat planned (the joint screws loosen 6.0 s")
        assert not panel.planner.requests and panel.lab.nothing_sent()
        assert panel.widget("Preview time").max == SECONDS_DEFAULT and "retreat ready" in panel.widget("").content
        panel.widget("Preview time").callback(0.0)
        panel.draw()
        np.testing.assert_allclose([panel.plugin._ghost.shown[1][name] for name in NAMES], INSERTED, atol=1e-9)

    asyncio.run(scenario())


# --- --- --- --- --- the plot windows --- --- --- --- ---

def test_a_planned_move_fills_the_plan_plots_and_opens_their_window_once(monkeypatch):
    """Approach: every joint along the preview, the window forward; a retime redraws only; Execute empties the plots.

    The approach is a joint motion, so the Exec plots stay minimized.
    """
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))
    plan_window, exec_window = panel.ctx.panels
    values, steps = panel.ctx.gui.of_kind("uplot")[:2]
    assert plan_window.calls == exec_window.calls == ["minimize"]

    async def scenario():
        await panel.click("Move", "Approach")
        assert plan_window.calls == ["minimize", "expand"], "a new plan brings the window forward"
        assert len(values.data) == 1 + len(NAMES) and values.data[0][-1] == pytest.approx(10.0, abs=0.1)
        assert [series["label"] for series in values.series[1:3]] == ["L pan", "L lift"]
        assert list(steps.data[0]) == [1.0, 2.0, 3.0, 4.0]
        panel.widget("Motion time").value = 20.0
        panel.widget("Motion time").callback(20.0)
        panel.draw()
        assert values.data[0][-1] == pytest.approx(20.0, abs=0.1) and plan_window.calls.count("expand") == 1
        await panel.click("Execute")
        assert all(len(column) == 1 and math.isnan(column[0]) for column in values.data[1:] + steps.data[1:])
        assert plan_window.calls.count("expand") == 1 and exec_window.calls == ["minimize"]

    asyncio.run(scenario())


def test_the_exec_plots_open_and_fill_while_a_compliant_part_runs(monkeypatch):
    """The retreat's Execute and Compliant each bring the Exec plots forward and add a force sample per tick drawn."""
    # * The lab's Cindy stands at its own insert, not the process's: the back-out goes to the lab's start.
    monkeypatch.setattr(InsertionDemoPlugin, "_insert_start", staticmethod(
        lambda _demo: (["left_ur_arm", "right_ur_arm"], dict(zip(NAMES, START)))))
    panel = Panel(monkeypatch, np.linspace(INSERTED, START, 5))
    _plan_window, exec_window = panel.ctx.panels

    def samples() -> int:
        return len(panel.plugin._exec_plots._compliant.times("force"))

    async def scenario():
        await panel.click("Move", "Retreat")
        assert exec_window.calls == ["minimize"] and samples() == 0, "planning opens the Plan plots only"
        panel.widget("Execute").callback()
        while not panel.plugin._task.done():
            panel.draw()
            await asyncio.sleep(0)
        assert panel.plugin.outcome.startswith("retreat RETREATED")
        retreat = samples()
        assert exec_window.calls == ["minimize", "expand"] and retreat > 10
        # * Compliant starts the plots over; they fill while the arms are held compliant.
        panel.widget("Arms").callback("Compliant")
        for _tick in range(10):
            await asyncio.sleep(0)
            panel.draw()
        assert exec_window.calls.count("expand") == 2 and 0 < samples() < retreat
        panel.widget("Arms").callback("Rigid")
        await panel.plugin._task
        assert panel.plugin.outcome == "both arms back under joint control"

    asyncio.run(scenario())
