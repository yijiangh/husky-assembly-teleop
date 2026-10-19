"""What the process plugin serves others: `selection`, the same object until the movement or its id maps change."""

from __future__ import annotations

from dataclasses import replace

from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin


def test_no_selection_without_movements(tmp_path):
    """Nothing is selected before a load, or in a process without movements."""
    plugin = ProcessPlugin()
    assert plugin.selection is None
    plugin._take(replace(build_process(tmp_path), schedule=(), actions={}))
    assert plugin.selection is None


def test_selection_is_replaced_when_it_changes(tmp_path):
    """Reads between changes return one object; a new movement, process or id map gives a new one, never an edit."""
    process = build_process(tmp_path)
    movements = list(process.movements())
    plugin = ProcessPlugin()
    plugin._take(process)
    first = plugin.selection
    assert plugin.selection is first
    assert (first.index, first.action, first.movement) == (0, *movements[0]) and first.process is process

    plugin.select(2)
    second = plugin.selection
    assert second is not first and (second.index, second.action, second.movement) == (2, *movements[2])
    assert first.index == 0

    plugin.ids, plugin.process_ids = {"bars/B1": "process/bars/B1"}, {"process/bars/B1": "bars/B1"}
    third = plugin.selection
    assert third is not second and third.ids == {"bars/B1": "process/bars/B1"}

    plugin._take(build_process(tmp_path))
    assert plugin.selection is not third
