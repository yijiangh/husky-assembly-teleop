"""What the process plugin serves other plugins: `selection`, the same object until the step or its id maps change."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.shared.process import LoadedProcess, Step


def _loaded(steps: int) -> LoadedProcess:
    """A loaded process of `steps` steps; the selection only passes the process and movements along."""
    return LoadedProcess(Path("process"), None, tuple(Step(None, 0, index, None) for index in range(steps)))


def test_no_selection_without_steps():
    """Nothing is selected before a load, or in a process without movements."""
    plugin = ProcessPlugin()
    assert plugin.selection is None
    plugin.loaded = _loaded(0)
    assert plugin.selection is None


def test_selection_is_replaced_when_it_changes():
    """Reads between changes return one object; a new step, process or id map gives a new one, never an edit."""
    plugin = ProcessPlugin()
    plugin.loaded = _loaded(3)
    first = plugin.selection
    assert plugin.selection is first
    assert (first.index, first.step) == (0, plugin.loaded.steps[0])

    plugin.select(2)
    second = plugin.selection
    assert second is not first and (second.index, second.step) == (2, plugin.loaded.steps[2])
    assert first.index == 0

    plugin.ids, plugin.process_ids = {"bars/B1": "process/bars/B1"}, {"process/bars/B1": "bars/B1"}
    third = plugin.selection
    assert third is not second and third.ids == {"bars/B1": "process/bars/B1"}

    plugin.loaded = _loaded(3)
    assert plugin.selection is not third


def test_request_select_moves_and_clamps():
    """Another plugin's selection moves the step, clamped to the process; without a process it is ignored."""
    plugin = ProcessPlugin()
    plugin.request_select(2)
    assert plugin.selection is None and plugin.index == 0
    plugin.loaded = _loaded(3)
    plugin.request_select(2)
    assert plugin.selection.index == 2
    plugin.request_select(9)
    assert plugin.index == 2
    plugin.request_select(-1)
    assert plugin.index == 0


def test_index_of_an_actions_movement():
    """A movement's step index: its action's first step plus its index in the action; none for a movement it lacks."""
    steps = tuple(Step(None, action, movement, None) for action, movement in ((0, 0), (0, 1), (1, 0), (1, 1), (1, 2)))
    actions = {"A": SimpleNamespace(movements=(0, 1)), "B": SimpleNamespace(movements=(0, 1, 2))}
    loaded = LoadedProcess(Path("process"), SimpleNamespace(schedule=("A", "B"), actions=actions), steps)
    assert (loaded.index_of("A"), loaded.index_of("B"), loaded.index_of("B", 2)) == (0, 2, 4)
    with pytest.raises(IndexError, match="A has no movement 2"):
        loaded.index_of("A", 2)
