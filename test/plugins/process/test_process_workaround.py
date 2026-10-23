"""The process panel's "Transfer workaround" box: jointing actions listed as they run; authored indices served."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from planning_fixtures import purged_process
from plugin_fakes import FakeContext
from process_fixtures import build_process

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, take_lease
from scaffolding_assembly_core.process import write

#: B22_J as it runs with the workaround: no travel to load, the transfer first.
B22_J_RUN = ["B22_J_transfer", "B22_J_place_bar", "B22_J_grasp", "B22_J_insert"]


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


@pytest.fixture
def small(tmp_path):
    """The small synthetic process, written and loaded as the monitor loads it."""
    write(build_process(tmp_path), tmp_path / "process")
    return load_process(tmp_path / "process", tmp_path)


class _ViserContext(FakeContext):
    """A fake context whose value callbacks are the real ones (`PluginContext.defer_value`), handed viser's events."""

    defer_value = PluginContext.defer_value


def _set_up(process, ctx: FakeContext | None = None) -> tuple[ProcessPlugin, FakeContext]:
    """The plugin with its panel on a fake context, `process` taken."""
    ctx = ctx or FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin._take(process)
    return plugin, ctx


def _ids(plugin: ProcessPlugin, prefix: str = "") -> list[str]:
    """The ids of the listed movements starting with `prefix`, in order."""
    return [movement.id for _action, movement in plugin._movements if movement.id.startswith(prefix)]


def _authored_index(plugin: ProcessPlugin, movement_id: str) -> int:
    """A movement's index in the authored `process.movements()`."""
    return next(index for index, (_action, movement) in enumerate(plugin.process.movements())
                if movement.id == movement_id)


def test_on_at_start_each_jointing_action_is_listed_as_it_runs(purged):
    """On at start: B22_J's load is gone, one movement fewer per jointing action; off gives the authored list again."""
    process = purged.process
    plugin, ctx = _set_up(process)
    authored = tuple(process.movements())
    jointing = sum(action.type == "bar_jointing" for action in process.actions.values())
    assert plugin.transfer_as_free and plugin._transfer.value is True
    assert _ids(plugin, "B22_J_") == B22_J_RUN
    assert len(plugin._movements) == len(authored) - jointing
    assert f"{len(authored) - jointing} movements" in plugin._status_html()

    plugin.set_transfer_as_free(ctx, False)
    assert ctx.lines("info")[-1] == "Transfer workaround off: jointing actions as authored"
    assert len(plugin._movements) == len(authored)
    assert all(shown is each for pair, other in zip(plugin._movements, authored) for shown, each in zip(pair, other))
    plugin.set_transfer_as_free(ctx, True)
    assert ctx.lines("info")[-1] == ("Transfer workaround on: jointing actions shown and run transit, place, grasp, "
                                     "insert")
    assert _ids(plugin, "B22_J_") == B22_J_RUN


def test_a_flip_keeps_the_authored_movement_selected(purged):
    """The insert stays selected both ways, in a new selection; the load, not listed, goes to its action's transit.

    The selection serves the authored process and index, with the action and movement as shown.
    """
    plugin, ctx = _set_up(purged.process)
    insert = _authored_index(plugin, "B22_J_insert")
    plugin.request_select(insert)
    first = plugin.selection
    assert first.process is plugin.process and first.index == insert and first.movement.id == "B22_J_insert"
    assert first.action is plugin._shown.actions["B22_J"]
    plugin.set_transfer_as_free(ctx, False)
    second = plugin.selection
    assert second is not first and second.index == insert and second.action is plugin.process.actions["B22_J"]
    # ! The same process object: assembly_run notices a new process by identity, so a flip never resets the run.
    assert second.process is first.process

    plugin.request_select(_authored_index(plugin, "B22_J_load"))
    plugin.set_transfer_as_free(ctx, True)
    assert plugin.selected[1].id == "B22_J_transfer"
    assert plugin.selection.index == _authored_index(plugin, "B22_J_transfer")


def test_request_select_takes_an_authored_index(purged):
    """Another plugin selects B22_J's load by its authored index: the panel selects the transit, with no bar yet."""
    plugin, _ctx = _set_up(purged.process)
    plugin.request_select(_authored_index(plugin, "B22_J_load"))
    action, transit = plugin.selected
    assert (action.id, transit.id) == ("B22_J", "B22_J_transfer") and "bars/B22" not in transit.start.present
    assert plugin.index == _ids(plugin).index("B22_J_transfer")
    assert plugin.selection.index == _authored_index(plugin, "B22_J_transfer")


def test_the_flip_is_refused_while_a_step_has_a_robots_lease(small):
    """movement_exec has Cindy's lease for a step: the box changes nothing, one line says why, the draw puts it back.

    Another holder's lease (robot_control's teleop) refuses nothing.
    """
    plugin, ctx = _set_up(small)
    slot = LeaseSlot("a200-0806")
    ctx.world.robots["a200-0806"] = SimpleNamespace(lease_slot=slot)
    lease = take_lease([slot], "movement_exec: exec B1_M2_insert")
    revision = plugin.revision
    plugin._transfer.callback(False)
    assert plugin.transfer_as_free and plugin.revision == revision
    assert ctx.lines("warn") == ["Transfer workaround not changed: a200-0806 is leased by movement_exec: exec "
                                 "B1_M2_insert: wait until it is done, or Cancel it"]
    plugin._transfer.value = False
    plugin.draw(ctx)
    assert plugin._transfer.value is True

    lease.release()
    take_lease([slot], "robot_control: teleop")
    plugin._transfer.callback(False)
    assert not plugin.transfer_as_free


def test_only_the_operators_click_flips_the_box(small):
    """The draw's own write-back calls back too, without a client: only a click with a client counts."""
    plugin, _ctx = _set_up(small, _ViserContext())

    def event(client) -> SimpleNamespace:
        """The box's update event, unticked."""
        return SimpleNamespace(client=client, target=SimpleNamespace(value=False))

    asyncio.run(plugin._transfer.callback(event(None)))
    assert plugin.transfer_as_free
    asyncio.run(plugin._transfer.callback(event(object())))
    assert not plugin.transfer_as_free
