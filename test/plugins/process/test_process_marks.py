"""The process panel's rows show the run's marks; joints carried forward are drawn at and after their movement."""

from __future__ import annotations

from types import SimpleNamespace

from plugin_fakes import FakeContext
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.shared.process_contract import MARK_ASSUMED, MARK_DONE, MARK_PENDING, Carry
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import write


class _Drawing(SimpleNamespace):
    """The overlay, keeping the joints of every `show`."""

    def set_bar_lines(self, on: bool) -> None:
        """Keep the switch, as the real drawing does."""
        self.bar_lines = on

    def show(self, state, joints, everything: bool = False, reference: frozenset[str] = frozenset()) -> None:
        """Keep the state and the joints it was asked to draw."""
        self.states.append(state)
        self.shown.append(joints)


def _set_up(tmp_path) -> tuple[ProcessPlugin, FakeContext]:
    """The plugin on a fake context, the small process loaded as authored (no workaround) and drawn by a `_Drawing`."""
    write(build_process(tmp_path), tmp_path / "process")
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin.transfer_as_free = False
    plugin._take(load_process(tmp_path / "process", tmp_path))
    plugin._drawing = _Drawing(shown=[], states=[])
    return plugin, ctx


def test_the_rows_show_the_runs_marks_until_another_load(tmp_path):
    """Done rows are green, assumed ones orange, the selected one blue whatever its mark; a load drops the marks."""
    plugin, ctx = _set_up(tmp_path)
    plugin.request_marks({"B1_J_joint": MARK_DONE, "B1_H_hold": MARK_ASSUMED, "B1_R_release": MARK_PENDING})
    plugin.select(5)  # B1_R_release's first movement
    plugin.draw(ctx)
    assert [(row.label, row.color) for row in plugin._action_rows[:4]] == [
        ("1. B1_J_joint · cindy · 3 mv · done", "green"), ("2. B1_H_hold · alice · 2 mv · done (assumed)", "orange"),
        ("3. B1_R_release · cindy · 2 mv · pending", "blue"), ("4. B2_J_joint · cindy · 4 mv", "gray")]
    plugin._take(plugin.process)
    plugin.draw(ctx)
    assert plugin._action_rows[1].label == "2. B1_H_hold · alice · 2 mv" and plugin._action_rows[1].color == "gray"


def test_the_carry_is_drawn_at_and_after_its_movement_and_dropped_on_load(tmp_path):
    """Carried from B2's transfer: not before, at its target and at the insert's start, not at the insert's target.

    The insert's target has joints of its own, so they show there. A load drops the carry.
    """
    plugin, ctx = _set_up(tmp_path)
    plugin.request_carry(Carry("B2_J_joint", "B2_M2_transfer", "robots/cindy", {"left_joint1": 0.9}))
    index = {movement.id: number for number, (_action, movement) in enumerate(plugin._movements)}

    def cindy(movement_id: str, at_target: bool = False) -> dict[str, float]:
        """Cindy's joints as drawn at a movement's start or target."""
        plugin.select(index[movement_id])
        plugin.at_target, plugin._stale = at_target, True
        plugin.draw(ctx)
        return plugin._drawing.shown[-1]["robots/cindy"]

    assert cindy("B2_M1_grasp", True)["left_joint1"] == 0.0 and cindy("B2_M2_transfer")["left_joint1"] == 0.0
    assert cindy("B2_M2_transfer", True) == {"left_joint1": 0.9, "left_joint2": 0.3, "right_joint1": 0.3,
                                             "right_joint2": 0.3}
    assert cindy("B2_M3_insert")["left_joint1"] == 0.9 and "joints from live IK" in plugin._details.content
    assert cindy("B2_M3_insert", True)["left_joint1"] == 0.35, "its own target joints, not the carry's"
    assert "joints from live IK" not in plugin._details.content
    plugin._take(plugin.process)
    assert cindy("B2_M3_insert")["left_joint1"] == 0.3 and "joints from live IK" not in plugin._details.content


def test_the_carry_is_drawn_on_its_live_base_and_turns_its_movement_to_its_target(tmp_path):
    """Carried with the base it was found at: Cindy drawn there wherever the joints show, the others as authored.

    A carry for the selected movement turns the panel to its target, so the goal shows at once; one for another
    movement leaves the panel as it was.
    """
    plugin, ctx = _set_up(tmp_path)
    index = {movement.id: number for number, (_action, movement) in enumerate(plugin._movements)}
    live = Pose((2.0, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    plugin.select(index["B2_M1_grasp"])
    plugin.request_carry(Carry("B2_J_joint", "B2_M2_transfer", "robots/cindy", {"left_joint1": 0.9}, base=live))
    assert not plugin.at_target, "not the selected movement"
    plugin.select(index["B2_M2_transfer"])
    plugin.request_carry(Carry("B2_J_joint", "B2_M2_transfer", "robots/cindy", {"left_joint1": 0.9}, base=live))
    assert plugin.at_target
    plugin.draw(ctx)
    state = plugin._drawing.states[-1]
    assert state.robots["robots/cindy"].base == live and plugin._drawing.shown[-1]["robots/cindy"]["left_joint1"] == 0.9
    assert "drawn at the joints and base found for B2_M2_transfer" in plugin._details.content, "the chip says so"
    authored = plugin._movements[index["B2_M2_transfer"]][1].start
    assert all(state.robots[robot] == authored.robots[robot] for robot in authored.robots if robot != "robots/cindy")
    plugin.at_target, plugin._stale = False, True
    plugin.draw(ctx)
    assert plugin._drawing.states[-1] is authored, "at its start the carry does not show"
