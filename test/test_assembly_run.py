"""The assembly_run plugin on a small written process: opening its run, selecting, marking, reopening, new runs."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from planning_fixtures import DATA
from plugin_fakes import FakeContext
from process_fixtures import JOINTS, build_process, with_action, write_robot_files

from husky_assembly_teleop.plugins.assembly_run.panel import RunPanel
from husky_assembly_teleop.plugins.assembly_run.plugin import AssemblyRunPlugin
from husky_assembly_teleop.shared.planning.steps import first_step
from husky_assembly_teleop.shared.process import Selection, load_process
from husky_assembly_teleop.shared.runs import (ASSUMED, DONE, LIVE, PENDING, RunSettings, find_runs, read_events,
                                               read_run)
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import write
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

CINDY, ALICE = "robots/a200-0806", "robots/a200-0804"


@pytest.fixture(autouse=True)
def _no_git(monkeypatch):
    """A run records the commits by running git: left out, to keep the tests quick."""
    monkeypatch.setattr("husky_assembly_teleop.shared.runs.commits", lambda: {"monitor": "", "core": "", "tamp": ""})


class FakeProcess:
    """The process plugin as assembly_run sees it: a selection, and the steps others selected."""

    def __init__(self, loaded):
        """Select the first step of `loaded`."""
        self.loaded, self.index, self.selected = loaded, 0, []

    @property
    def selection(self) -> Selection:
        """The selected step."""
        return Selection(self.loaded, self.index, self.loaded.steps[self.index])

    def request_select(self, index: int) -> None:
        """Note the selection."""
        self.selected.append(index)
        self.index = index


class Lab:
    """A process written to a scratch folder, Cindy configured, live from `live_from` (ROS time) on, never if None."""

    def __init__(self, folder: Path, live_from: float | None = 0.0, alice: bool = False):
        """Write the process and make the context; with `alice`, Alice is in the scene too, always live."""
        self.folder = folder
        urdf, _ = write_robot_files(folder / "robot")
        self.model = SimpleNamespace(urdf=urdf, tools={}, movable_joints=JOINTS)
        config = SimpleNamespace(serial="a200-0806", model=self.model, arms=(), source_urdf=urdf)
        self.live_from, self.alice = live_from, alice
        write(build_process(folder), folder / "p")
        self.ctx = FakeContext(robots=(config,), step=self.tick)
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """Cindy measured now while live; otherwise her last measurements grow old."""
        live = self.live_from is not None and t >= self.live_from
        stamp = t if live else None
        robots = {CINDY: RobotObject(CINDY, self.model, Pose((1.0, 2.0, 0.0)), {name: 0.1 for name in JOINTS},
                                     base_time=stamp, joints_time=stamp)}
        if self.alice:
            robots[ALICE] = RobotObject(ALICE, self.model, Pose((-1.0, 0.0, 0.0)), {name: 1.0 for name in JOINTS},
                                        base_time=t, joints_time=t)
        self.ctx.scene.snapshot = Scene(robots=robots)

    def moving(self, live: str, executing: bool) -> None:
        """Say whether a robot's arm runs a trajectory, as `world.robots[serial].arms[...].state.is_executing`."""
        state = SimpleNamespace(is_executing=executing)
        self.ctx.world.robots[live.split("/", 1)[1]] = SimpleNamespace(arms={"arm": SimpleNamespace(state=state)},
                                                                       lease=None)

    def plugin(self) -> tuple[AssemblyRunPlugin, FakeProcess]:
        """A new plugin, as after a restart, on the process loaded again."""
        process = FakeProcess(load_process(self.folder / "p", DATA))
        self.ctx.plugins["process"] = process
        plugin = AssemblyRunPlugin()
        plugin._ctx = self.ctx
        return plugin, process


async def settle(plugin: AssemblyRunPlugin) -> None:
    """Wait for the plugin's tasks, then for its queued writes."""
    while plugin._tasks:
        await asyncio.gather(*list(plugin._tasks), return_exceptions=True)
    await asyncio.wrap_future(plugin._writer.submit(lambda: None))
    await asyncio.sleep(0)


async def opened(lab: Lab) -> tuple[AssemblyRunPlugin, FakeProcess]:
    """A new plugin that has opened the process's run and selected its current step."""
    plugin, process = lab.plugin()
    plugin.update(lab.ctx)
    await settle(plugin)
    return plugin, process


async def answered(plugin: AssemblyRunPlugin, confirmed: bool) -> None:
    """Wait for the prompt, answer it, and let it finish."""
    while not plugin._prompt:
        await asyncio.sleep(0)
    plugin.answer(confirmed)
    await settle(plugin)


def first_index(process: FakeProcess, action_id: str) -> int:
    """The step index of an action's first step, the transfer workaround on."""
    process_ = process.loaded.process
    return process.loaded.index_of(action_id, first_step(process_, process_.actions[action_id], True))


def runs(lab: Lab) -> list[Path]:
    """The run folders written so far."""
    folder = lab.folder / "p" / "runs"
    return sorted(folder.iterdir()) if folder.is_dir() else []


def test_selects_the_first_pending_action_once_its_robot_is_live(tmp_path):
    """No run yet: the first action becomes current once Cindy is live, a second later; nothing is written."""
    lab = Lab(tmp_path, live_from=11.0)

    async def scenario():
        plugin, process = await opened(lab)
        assert process.selected == [first_index(process, "B1_J_joint")]
        assert 11.0 <= lab.ctx.t < 11.2 and not lab.ctx.lines("warn")
        assert plugin.view.current[0] == "B1_J_joint" and plugin.view.folder is None and runs(lab) == []
        assert set(plugin.view.statuses.values()) == {PENDING}

    asyncio.run(scenario())


def test_selects_anyway_after_15_s_with_a_warning(tmp_path):
    """Cindy never live: after 15 s the first action is selected anyway, with a warning."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        _plugin, process = await opened(lab)
        assert process.selected == [first_index(process, "B1_J_joint")] and lab.ctx.t >= 25.0
        assert any("not live after 15 s" in line for line in lab.ctx.lines("warn"))

    asyncio.run(scenario())


def test_mark_done_live_starts_the_run_and_moves_on(tmp_path):
    """Marking with Cindy live: the run starts, Cindy's end is recorded live, the next action is selected."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        return plugin, process

    plugin, process = asyncio.run(scenario())
    folder, = runs(lab)
    run = read_run(folder)
    record = run.progress.actions["B1_J_joint"]
    assert (record.status, record.assumed, record.ends[CINDY].source) == (DONE, False, LIVE)
    assert run.progress.robots[CINDY].after_action == "B1_J_joint"
    assert run.progress.current == plugin.view.current and plugin.view.current[0] == "B1_H_hold"
    assert process.selected[-1] == first_index(process, "B1_H_hold")
    assert [event["event"] for event in read_events(folder)] == ["new_run", "mark_done"]


def test_mark_done_not_live_asks_first(tmp_path):
    """Cindy not live: Cancel leaves everything as it was; Confirm marks with the export's end, assumed."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await answered(plugin, False)
        assert runs(lab) == [] and plugin.view.statuses["B1_J_joint"] == PENDING
        plugin.request_mark_done()
        assert "not live" in plugin._prompt
        await answered(plugin, True)
        return plugin

    plugin = asyncio.run(scenario())
    run = read_run(runs(lab)[0])
    record = run.progress.actions["B1_J_joint"]
    assert record.status == DONE and record.assumed and record.ends[CINDY].source == ASSUMED
    assert record.ends[CINDY].joints["left_joint1"] == 0.5  # the insert's target
    assert plugin.view.assumed == frozenset({"B1_J_joint"})


def test_reopen_puts_the_action_back(tmp_path):
    """Reopening the marked action: pending again, current again, Cindy's belief from it dropped."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.request_reopen("B1_J_joint")
        await settle(plugin)
        return plugin, process

    plugin, process = asyncio.run(scenario())
    run = read_run(runs(lab)[0])
    assert run.progress.actions["B1_J_joint"].status == PENDING and CINDY not in run.progress.robots
    assert plugin.view.current[0] == "B1_J_joint" and process.selected[-1] == first_index(process, "B1_J_joint")
    assert read_events(run.folder)[-1]["event"] == "reopen"


def test_new_run_after_confirm_keeps_the_old_one(tmp_path):
    """New run, confirmed: a second folder with every action pending; the first run's files unchanged."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        first, = runs(lab)
        before = (first / "progress.json").read_bytes()
        plugin.request_new_run()
        await answered(plugin, True)
        assert (first / "progress.json").read_bytes() == before
        return plugin

    plugin = asyncio.run(scenario())
    first, second = runs(lab)
    assert plugin.view.folder == second and set(plugin.view.statuses.values()) == {PENDING}
    assert {record.status for record in read_run(second).progress.actions.values()} == {PENDING}
    assert plugin.view.current[0] == "B1_J_joint"


def test_a_restart_continues_the_run(tmp_path):
    """A new plugin on the same process continues its run: statuses, beliefs and the current action kept."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        again, process = await opened(lab)
        return plugin, again, process

    plugin, again, process = asyncio.run(scenario())
    assert again.view.folder == plugin.view.folder and again.view.statuses["B1_J_joint"] == DONE
    assert again.view.current == plugin.view.current and process.selected == [first_index(process, "B1_H_hold")]
    assert set(again.view.beliefs) == {CINDY}
    assert [event["event"] for event in read_events(again.view.folder)][-1] == "session"


def test_another_export_is_offered_and_continued(tmp_path):
    """After a re-export the old run is only offered; marking is refused until Continue carries it into a new run."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        changed = with_action(build_process(tmp_path), "B1_H_hold", label="hold B1 tight")
        write(changed, lab.folder / "p", overwrite=True)
        again, _process = await opened(lab)
        assert again.view.offer == plugin.view.folder.name and again.view.folder is None
        again.request_mark_done("B1_H_hold")
        assert "another export" in again._message and len(runs(lab)) == 1
        again.request_continue()
        await settle(again)
        return again

    again = asyncio.run(scenario())
    assert len(runs(lab)) == 2 and again.view.offer == "" and again.view.statuses["B1_J_joint"] == DONE
    info = json.loads((again.view.folder / "run.json").read_text())
    assert info["continued_from"] == runs(lab)[0].name and info["changed_since"] == ["B1_H_hold"]


def test_settings_are_saved_with_the_run(tmp_path):
    """A setting change starts the run and is saved; a restart reads it back; a split out of range is refused."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_settings(spiral=True, insert_split_mm=12)
        await settle(plugin)
        plugin.request_settings(insert_split_mm=40.0)
        assert "must be 0 to 30 mm" in plugin._message
        again, _process = await opened(lab)
        return again

    again = asyncio.run(scenario())
    settings = again.view.settings
    assert (settings.spiral, settings.insert_split_mm, settings.transfer_as_free) == (True, 12.0, True)
    assert read_events(again.view.folder)[1]["event"] == "settings"


def test_view_is_the_same_object_until_something_changes(tmp_path):
    """Two reads without a change give one view; a change gives a new one, the old one unchanged."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        first = plugin.view
        assert plugin.view is first
        plugin.request_set_current("B1_R_release")
        second = plugin.view
        assert second is not first and second.current[0] == "B1_R_release" and first.current[0] == "B1_J_joint"

    asyncio.run(scenario())


def test_record_writes_an_event_and_a_movement_file(tmp_path):
    """Records go into the run: an event with its data, or a movement file the event names."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_record("check_goal", action="B1_J_joint", movement="B1_M2_insert", data={"ok": True})
        await settle(plugin)
        plugin.request_record("planned", action="B1_J_joint", movement="B1_M2_insert", file={"waypoints": 3})
        await settle(plugin)
        return plugin

    plugin = asyncio.run(scenario())
    events = read_events(plugin.view.folder)
    assert events[1]["event"] == "check_goal" and events[1]["data"] == {"ok": True}
    assert events[2]["file"] == "movements/B1_J_joint/B1_M2_insert.1.json"
    assert json.loads((plugin.view.folder / events[2]["file"]).read_text())["waypoints"] == 3


def test_the_panel_shows_the_run_and_its_buttons_ask_the_plugin(tmp_path):
    """Rows show each action's robot and status, coloured; a row click, Mark done and a setting reach the plugin."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin._panel = RunPanel(lab.ctx, plugin)
        gui = lab.ctx.gui
        rows, groups = gui.of_kind("button"), gui.of_kind("button_group")
        groups[1].callback("Mark done")
        await settle(plugin)
        plugin.draw(lab.ctx)
        assert [(row.label, row.color) for row in rows[:3]] == [
            ("1. B1_J_joint · cindy · done", "green"), ("2. B1_H_hold · alice · pending", "blue"),
            ("3. B1_R_release · cindy · pending", "gray")]
        assert rows[5].label == "·" and "cindy     live now" in gui.widgets[-1].content
        rows[2].callback()
        assert plugin.view.current[0] == "B1_R_release"
        gui.of_kind("checkbox")[1].callback(True)
        await settle(plugin)
        assert plugin.view.settings.spiral

    asyncio.run(scenario())


# --- --- --- --- --- REVIEW CASES --- --- --- --- ---

def test_a_movement_the_action_does_not_have_becomes_its_first_step(tmp_path):
    """A movement index the action lacks becomes its first step, with a warning; the panel still draws.

    Both asked for, and carried from another export.
    """
    lab = Lab(tmp_path)

    async def scenario():
        plugin, process = await opened(lab)
        plugin.request_set_current("B1_H_hold", 7)
        assert plugin.view.current == ("B1_H_hold", 0) and process.selected[-1] == first_index(process, "B1_H_hold")
        assert any("no movement 7" in line for line in lab.ctx.lines("warn"))
        plugin.request_mark_done("B1_J_joint")
        await settle(plugin)
        plugin.request_set_current("B1_H_hold", 1)
        await settle(plugin)
        plugin.teardown(lab.ctx)
        base = build_process(tmp_path)
        shorter = with_action(base, "B1_H_hold", movements=base.actions["B1_H_hold"].movements[:1])
        write(shorter, lab.folder / "p", overwrite=True)
        again, process = await opened(lab)
        again._panel = RunPanel(lab.ctx, again)
        again.request_continue()
        await settle(again)
        assert again.view.current == ("B1_H_hold", 0) and process.selected[-1] == first_index(process, "B1_H_hold")
        assert read_run(again.view.folder).progress.current == ("B1_H_hold", 0)
        again.draw(lab.ctx)

    asyncio.run(scenario())


def test_a_saved_movement_out_of_range_is_fixed_on_open(tmp_path):
    """A run saved with a movement its action does not have opens on the action's first step, and saves that."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        path = runs(lab)[0] / "progress.json"
        progress = json.loads(path.read_text())
        progress["current"] = {"action": "B1_H_hold", "movement": 9}
        path.write_text(json.dumps(progress))
        again, process = await opened(lab)
        again._panel = RunPanel(lab.ctx, again)
        again.draw(lab.ctx)
        return again, process

    again, process = asyncio.run(scenario())
    assert again.view.current == ("B1_H_hold", 0) and process.selected == [first_index(process, "B1_H_hold")]
    assert read_run(runs(lab)[0]).progress.current == ("B1_H_hold", 0)


@pytest.mark.parametrize("started", [False, True])
def test_an_interrupted_open_writes_nothing_until_it_opens(tmp_path, started):
    """A soft stop during the open (before or after its first step): nothing is written, then the open is retried."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        again, _process = lab.plugin()
        again.update(lab.ctx)
        if started:
            await asyncio.sleep(0)
        for task in list(again._tasks):  # what a soft stop does
            task.cancel()
        await settle(again)
        again.request_mark_done("B1_H_hold")
        assert "not open" in again._message and len(runs(lab)) == 1
        again.request_record("check_goal", data={"ok": True})
        again.update(lab.ctx)  # the next tick opens again
        await settle(again)
        return again

    again = asyncio.run(scenario())
    folder, = runs(lab)
    assert again.view.folder == folder and again.view.statuses["B1_J_joint"] == DONE
    assert "check_goal" not in [event["event"] for event in read_events(folder)]


def test_a_failed_open_is_retried_after_a_while(tmp_path, monkeypatch):
    """The runs can't be read: the error is shown, nothing written, and the open is tried again 5 s later."""
    lab = Lab(tmp_path)
    failures = [OSError("Drive not mounted")]

    def scan(folder):
        if failures:
            raise failures.pop()
        return find_runs(folder)

    monkeypatch.setattr("husky_assembly_teleop.plugins.assembly_run.plugin.find_runs", scan)

    async def scenario():
        plugin, _process = await opened(lab)
        assert "Drive not mounted" in plugin._message and plugin.view.folder is None
        plugin.request_settings(spiral=True)
        assert not plugin.view.settings.spiral and "not open" in plugin._message
        plugin.update(lab.ctx)
        await settle(plugin)
        assert not plugin._tasks and plugin._state == "not opened"
        lab.ctx.t += 5.0
        plugin.update(lab.ctx)
        await settle(plugin)
        return plugin

    plugin = asyncio.run(scenario())
    assert plugin._state == "open" and plugin._message.startswith("no run yet")


def test_a_reverted_export_offers_the_older_matching_run(tmp_path):
    """Back on an earlier export, its older run is not continued silently: both runs are offered, the operator picks."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        first, = runs(lab)
        write(with_action(build_process(tmp_path), "B1_H_hold", label="tweak"), lab.folder / "p", overwrite=True)
        newer, _process = await opened(lab)
        newer.request_continue()
        await settle(newer)
        newer.request_mark_done("B1_H_hold")
        await settle(newer)
        newer.teardown(lab.ctx)
        write(build_process(tmp_path), lab.folder / "p", overwrite=True)
        back, _process = await opened(lab)
        assert back.view.folder is None and back.view.offer == runs(lab)[1].name and back.view.match == first.name
        back.request_continue(first.name)
        await settle(back)
        return back, first

    back, first = asyncio.run(scenario())
    assert back.view.folder == first and len(runs(lab)) == 2 and back.view.statuses["B1_H_hold"] == PENDING
    assert read_events(first)[-1]["event"] == "session"


def test_a_moving_arm_is_never_recorded(tmp_path):
    """The acting robot moving refuses the mark; another robot moving is left out of the ends."""
    lab = Lab(tmp_path, alice=True)

    async def scenario():
        plugin, _process = await opened(lab)
        lab.moving(CINDY, True)
        plugin.request_mark_done()
        assert "still moving" in plugin._message and runs(lab) == []
        lab.moving(CINDY, False)
        lab.moving(ALICE, True)
        plugin.request_mark_done()
        await settle(plugin)
        assert "not recorded while moving: robots/a200-0804" in plugin._message
        return plugin

    plugin = asyncio.run(scenario())
    assert set(read_run(plugin.view.folder).progress.actions["B1_J_joint"].ends) == {CINDY}


def test_a_robot_held_for_a_motion_refuses_the_mark(tmp_path):
    """While any robot is held by the executor, Mark done is refused, naming the holder."""
    lab = Lab(tmp_path, alice=True)

    async def scenario():
        plugin, _process = await opened(lab)
        lab.moving(ALICE, False)
        lab.ctx.world.robots["a200-0804"].lease = SimpleNamespace(holder="movement_exec", label="B21_H_approach")
        plugin.request_mark_done()
        assert "a200-0804 is held by movement_exec (B21_H_approach)" in plugin._message and runs(lab) == []
        lab.ctx.world.robots["a200-0804"].lease = None
        plugin.request_mark_done()
        await settle(plugin)
        return plugin

    plugin = asyncio.run(scenario())
    assert plugin.view.statuses["B1_J_joint"] == DONE


def test_a_held_robot_refuses_selecting_reopening_and_settings(tmp_path):
    """While a robot is held for a step (an Exec runs), a row click, Reopen and a settings change are refused."""
    lab = Lab(tmp_path, alice=True)

    async def scenario():
        plugin, process = await opened(lab)
        lab.moving(ALICE, False)
        plugin.request_mark_done()
        await settle(plugin)
        lab.ctx.world.robots["a200-0804"].lease = SimpleNamespace(holder="movement_exec", label="B21_H_approach")
        current, selected = plugin.view.current, list(process.selected)
        plugin.request_set_current("B1_J_joint")
        assert "Select refused: a200-0804 is held by movement_exec" in plugin._message
        plugin.request_reopen("B1_J_joint")
        assert "Reopen refused: a200-0804 is held" in plugin._message
        refused = plugin._settings_refused
        plugin.request_settings(transfer_as_free=False)
        assert "Settings refused: a200-0804 is held" in plugin._message and plugin._settings_refused == refused + 1
        return plugin, process, current, selected

    plugin, process, current, selected = asyncio.run(scenario())
    assert plugin.view.current == current and process.selected == selected
    assert plugin.view.statuses["B1_J_joint"] == DONE and plugin.view.settings.transfer_as_free


def test_an_arm_moving_while_the_prompt_waits_refuses_the_confirmed_mark(tmp_path):
    """Cindy not live: the prompt waits; her arm starts moving meanwhile, so Confirm marks nothing."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        lab.moving(CINDY, True)
        await answered(plugin, True)
        return plugin

    plugin = asyncio.run(scenario())
    assert "still moving" in plugin._message and runs(lab) == [] and plugin.view.statuses["B1_J_joint"] == PENDING


def test_a_second_click_waits_for_the_prompt(tmp_path):
    """While a prompt waits, another Mark done, a New run or a Reopen is refused at once."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        prompt = plugin._prompt
        for request in (plugin.request_mark_done, plugin.request_new_run):
            request()
            assert plugin._message.endswith("answer the prompt first") and plugin._prompt == prompt
        await answered(plugin, False)
        return plugin

    plugin = asyncio.run(scenario())
    assert runs(lab) == [] and len(plugin._tasks) == 0


def test_while_runs_are_offered_settings_and_reopen_say_so(tmp_path):
    """Another export's run offered: a setting is refused and put back, Reopen names the offer; nothing is written."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        write(with_action(build_process(tmp_path), "B1_H_hold", label="tweak"), lab.folder / "p", overwrite=True)
        again, _process = await opened(lab)
        refused = again._settings_refused
        again.request_settings(spiral=True)
        assert "Continue or New run first" in again._message and not again.view.settings.spiral
        assert again._settings_refused == refused + 1
        again.request_reopen("B1_J_joint")
        assert "made on another export" in again._message
        await settle(again)

    asyncio.run(scenario())
    assert len(runs(lab)) == 1


def test_settings_that_make_no_sense_are_refused_not_raised(tmp_path):
    """Unknown names, words for numbers, numbers for switches: refused with a reason."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        for changes in ({"bogus": 1}, {"insert_split_mm": "x"}, {"insert_split_mm": float("nan")}, {"spiral": "yes"},
                        {"insert_split_mm": True}):
            plugin.request_settings(**changes)
            assert plugin._message.startswith("Settings refused") and plugin.view.settings == RunSettings()
        await settle(plugin)

    asyncio.run(scenario())
    assert runs(lab) == []


def test_a_step_picked_while_waiting_stops_the_automatic_selection(tmp_path):
    """The operator picks a step in the process panel while the run waits for Cindy: it is not overridden."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        plugin, process = lab.plugin()
        plugin.update(lab.ctx)
        while plugin._go_task is None:
            await asyncio.sleep(0)
        process.index = 4
        plugin.update(lab.ctx)
        await settle(plugin)
        return process

    process = asyncio.run(scenario())
    assert process.selected == [] and process.index == 4 and lab.ctx.t < 25.0


# --- --- --- --- --- RE-REVIEW CASES --- --- --- --- ---

async def reverted(lab: Lab, tmp_path: Path) -> tuple[Path, Path]:
    """Run A on the first export (B1_J done), run B on a second export continuing it (B1_H done), back to the first."""
    plugin, _process = await opened(lab)
    plugin.request_mark_done()
    await settle(plugin)
    plugin.teardown(lab.ctx)
    write(with_action(build_process(tmp_path), "B1_H_hold", label="tweak"), lab.folder / "p", overwrite=True)
    newer, _process = await opened(lab)
    newer.request_continue()
    await settle(newer)
    newer.request_mark_done("B1_H_hold")
    await settle(newer)
    newer.teardown(lab.ctx)
    write(build_process(tmp_path), lab.folder / "p", overwrite=True)
    first, second = runs(lab)
    return first, second


def test_after_continue_match_a_restart_continues_it_and_nothing_is_selected_while_offered(tmp_path):
    """Continue match makes that run the one used last: the next restart continues it without asking.

    While the runs are offered, no action is made current or selected.
    """
    lab = Lab(tmp_path)

    async def scenario():
        first, second = await reverted(lab, tmp_path)
        back, process = await opened(lab)
        assert (back.view.offer, back.view.match) == (second.name, first.name)
        assert back.view.current is None and process.selected == []
        back.request_continue(first.name)
        await settle(back)
        back.request_mark_done("B1_H_hold")
        await settle(back)
        back.teardown(lab.ctx)
        again, _process = await opened(lab)
        return first, again

    first, again = asyncio.run(scenario())
    assert again.view.folder == first and again.view.offer == "" and len(runs(lab)) == 2
    assert again.view.statuses["B1_H_hold"] == DONE


def test_a_second_continue_while_the_first_runs_is_refused(tmp_path):
    """Continue, then at once Continue match: the second is refused; one new run, the matching run untouched."""
    lab = Lab(tmp_path)

    async def scenario():
        first, second = await reverted(lab, tmp_path)
        before = (first / "events.jsonl").read_bytes()
        back, _process = await opened(lab)
        back.request_continue()
        back.request_continue(back.view.match)
        refused = back._message
        await settle(back)
        return first, second, before, back, refused

    first, second, before, back, refused = asyncio.run(scenario())
    assert refused == "Continue refused: a run is being started: wait a moment"
    assert len(runs(lab)) == 3 and read_run(back.view.folder).info.continued_from == second.name
    assert (first / "events.jsonl").read_bytes() == before


def test_a_soft_stop_while_waiting_for_the_robot_still_makes_the_action_current(tmp_path):
    """Stop all while the run waits for Cindy to be live: the first pending action is made current at once."""
    lab = Lab(tmp_path, live_from=None)

    async def scenario():
        plugin, process = lab.plugin()
        plugin.update(lab.ctx)
        while plugin._go_task is None:
            await asyncio.sleep(0)
        for task in list(plugin._tasks):  # what a soft stop does
            task.cancel()
        await settle(plugin)
        return plugin, process

    plugin, process = asyncio.run(scenario())
    assert plugin.view.current[0] == "B1_J_joint" and process.selected == [first_index(process, "B1_J_joint")]
    assert lab.ctx.t < 25.0


def test_a_newer_run_that_cant_be_read_is_not_written_past(tmp_path):
    """A newer run folder can't be read (half synced): the run made on these files is offered, not continued unasked."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        first, = runs(lab)
        broken = first.parent / f"{first.name}_2"
        broken.mkdir()
        (broken / "run.json").write_text('{"schema": "husky_run/1", "kind": "run", "na')
        again, _process = await opened(lab)
        offered = again.view.folder, again.view.offer, again.view.match
        again._panel = RunPanel(lab.ctx, again)
        again.draw(lab.ctx)
        again.request_mark_done("B1_H_hold")
        refused = again._message
        again.request_continue()
        await settle(again)
        return first, again, offered, refused

    first, again, offered, refused = asyncio.run(scenario())
    assert offered == (None, first.name, first.name)
    assert "a newer run folder can't be read" in refused and "Continue or New run first" in refused
    assert again.view.folder == first and len([path for path in runs(lab) if (path / "run.json").is_file()]) == 2


def test_the_re_review_messages(tmp_path, monkeypatch):
    """Continue match without a match, an open raising after the read, true as a movement, a run a setting started."""
    lab = Lab(tmp_path)

    async def scenario():
        plugin, _process = await opened(lab)
        plugin.request_mark_done()
        await settle(plugin)
        plugin.teardown(lab.ctx)
        write(with_action(build_process(tmp_path), "B1_H_hold", label="tweak"), lab.folder / "p", overwrite=True)
        again, _process = await opened(lab)
        again.request_continue(again.view.match)
        assert again._message == "Continue refused: no run made on these files is offered"
        again.request_continue()
        await settle(again)
        again.request_set_current("B1_H_hold", True)
        assert again.view.current == ("B1_H_hold", 0)
        assert any("no movement True" in line for line in lab.ctx.lines("warn"))

        fresh = Lab(tmp_path / "fresh")
        plugin, _process = await opened(fresh)
        plugin.request_settings(spiral=True)
        await settle(plugin)
        assert plugin._message == f"run {plugin.view.folder.name} started"

        def broken(*_args):
            raise RuntimeError("bad hashes")

        monkeypatch.setattr("husky_assembly_teleop.plugins.assembly_run.plugin.continue_run", broken)
        failing, _process = await opened(fresh)
        assert failing._message == "could not open the run (bad hashes): trying again in 5 s" and failing._failed

    asyncio.run(scenario())
