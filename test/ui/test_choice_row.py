"""A choice row: a click chooses and tells the plugin; new options swap in place, the first chosen."""

from types import SimpleNamespace

from plugin_fakes import FakeGui

from husky_assembly_teleop.ui.choice_row import ChoiceRow


def test_new_options_swap_in_the_same_row_and_the_first_is_chosen():
    """One arm, then Left and Right: the same button group, now two labels, Left filled; a click passes the value."""
    gui, chosen = FakeGui(), []
    ctx = SimpleNamespace(defer_value=lambda _label, run, operator_only=False: run)
    row = ChoiceRow(ctx, gui, "Arm", ("ur_arm",), "ur_arm", on_choose=chosen.append, labels=("Arm",), height=None)
    group, = [widget for widget in gui.widgets if widget.kind == "button_group"]
    row.set_options(("left_ur_arm", "right_ur_arm"), ("Left", "Right"))
    assert [widget for widget in gui.widgets if widget.kind == "button_group"] == [group]
    assert tuple(group.options) == ("Left", "Right") and row.value == "left_ur_arm"
    group.callback("Right")
    assert row.value == "right_ur_arm" and chosen == ["right_ur_arm"]
