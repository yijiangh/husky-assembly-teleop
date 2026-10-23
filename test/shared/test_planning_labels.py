"""Tests for shared.assembly_planning.labels: old monitor names, flowchart kinds and the sentences.

The golden table is the purged two-module process (planning_fixtures.PURGED): every movement of B20-B37 gets the old
monitor's name its action type, routine and real drives give, as the old exports since 261006 named them (e.g.
261016_2ModulesDemo_YH: a ground bar's insert is J_M4_LM_insert). The synthetic tests at the end need no Drive.
"""

from __future__ import annotations

from dataclasses import replace

import pytest
from planning_fixtures import action, movement, purged_process
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.shared.assembly_planning import labels as labels_module
from husky_assembly_teleop.shared.assembly_planning.labels import (BASE, CDFM, CDLM, FM, INSERT_GROUND, INSERT_TIGHTEN,
                                                                   LM, MANUAL, TM, TM_THROUGH, kinds_of,
                                                                   label_movement)
from scaffolding_assembly_core.process import Action, Movement, Target, write

#: The old names of each action type's movements in the purged process, in movement order ("": none); a jointing
#: action's last movement, the insert, depends on its bar (`INSERTS`).
GOLDEN = {
    "bar_jointing": ("J_M0_free_to_load", "J_M1_manual_mount_bar", "J_M2_tool_grasp_bar",
                     "J_M3_CDFM_transfer_to_approach", None),
    "bar_release": ("R_M0_tool_ungrasp_bar", "R_M1_LM_retreat", "R_M2_free_home"),
    "bar_holding": ("H_M0_free_to_approach", "H_M1_gripper_open", "H_M2_LM_to_grasp", "H_M3_gripper_close"),
    "bar_holding_release": ("HR_M0_gripper_open", "HR_M1_LM_retreat", ""),
}
#: Their codes, for the panel's row of chips.
CODES = {
    "bar_jointing": ("J_M0", "J_M1", "J_M2", "J_M3", None),
    "bar_release": ("R_M0", "R_M1", "R_M2"),
    "bar_holding": ("H_M0", "H_M1", "H_M2", "H_M3"),
    "bar_holding_release": ("HR_M0", "HR_M1", "HR_leave"),
}
#: Their flowchart kinds.
KINDS = {
    "bar_jointing": ((FM,), (MANUAL,), (TM,), (CDFM,), None),
    "bar_release": ((TM,), (LM,), (FM,)),
    "bar_holding": ((FM,), (TM,), (LM,), (TM,)),
    "bar_holding_release": ((TM,), (LM,), (FM,)),
}
#: The ground bars of the purged process: their tightens are never sent, so the insert is the old rigid J_M4.
GROUND_BARS = {"B20", "B34", "B46"}
#: Each insert's old name, code and kinds: a ground bar's, and a screwed bar's.
INSERTS = {True: (INSERT_GROUND, "J_M4", (CDLM,)), False: (INSERT_TIGHTEN, "J_M4+5", (CDLM, TM_THROUGH))}


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def test_golden_table(purged):
    """The 70 movements of B20-B37 get their old names, codes and kinds; each a sentence, capitalized."""
    process = purged.process
    count = 0
    for action_id in process.schedule[:18]:
        each = action(purged, action_id)
        labels = [label_movement(process, each, one) for one in each.movements]
        ground = each.bar.split("/")[-1] in GROUND_BARS
        expected = [GOLDEN[each.type], CODES[each.type], KINDS[each.type]]
        if each.type == "bar_jointing":
            expected = [(*rows[:-1], insert) for rows, insert in zip(expected, INSERTS[ground])]
        assert tuple(label.old_name for label in labels) == expected[0], each.id
        assert tuple(label.code for label in labels) == expected[1], each.id
        assert tuple(label.kinds for label in labels) == expected[2], each.id
        for label in labels:
            assert label.text[0].isupper() and label.text.endswith(".") and "{" not in label.text
        count += len(labels)
    assert count == 70


def test_ground_bars_insert_rigid_with_no_screw(purged):
    """B20, B34 and B46 carry tightens on ground halves, which are never sent: J_M4_LM_insert, no tool chip."""
    for bar in sorted(GROUND_BARS):
        label = label_movement(purged.process, *movement(purged, f"{bar}_J_insert"))
        assert (label.old_name, label.code, label.kinds) == (INSERT_GROUND, "J_M4", (CDLM,)), bar
        assert "rigid" in label.text and "no jointing screw runs" in label.text and "tighten" not in label.text


def test_the_panel_says_which_drives_are_never_sent(purged):
    """A ground bar's authored tightens show as not sent in the moves line; a screwed bar's do not."""
    panel = ProcessPlugin()
    panel._take(purged.process)
    for movement_id, marked in (("B20_J_insert", True), ("B22_J_insert", False)):
        panel.select(next(index for index, (_, one) in enumerate(panel._movements) if one.id == movement_id))
        assert ("tighten (not sent)" in panel._details_html(*panel.selected)) == marked, movement_id


def test_a_bar_on_the_ground_at_one_end_that_tightens_at_the_other(purged, monkeypatch):
    """Grounded but with a tighten really sent: the screwed insert's name and sentence, with its tool chip."""
    jointing = action(purged, "B22_J")
    insert = jointing.movements[-1]
    sent = labels_module.mating_drives(purged.process, insert)
    monkeypatch.setattr(labels_module, "mating_drives", lambda process, one: replace(sent, grounded=True))
    label = label_movement(purged.process, jointing, insert)
    assert (label.old_name, label.kinds) == (INSERT_TIGHTEN, (CDLM, TM_THROUGH))
    assert "jointing screws tighten" in label.text


def test_sentences_name_the_robot_the_bar_the_line_and_the_screws(purged):
    """The robot's name, the bar, a straight move's distance and the screws really sent fill the sentences."""
    process = purged.process
    assert label_movement(process, *movement(purged, "B20_J_load")).text.startswith("Cindy's two arms travel")
    assert "B20 is clamped" in label_movement(process, *movement(purged, "B20_J_grasp")).text
    assert "retreats 50 mm along" in label_movement(process, *movement(purged, "B20_R_retreat")).text
    assert label_movement(process, *movement(purged, "B21_H_approach")).text.startswith("Alice's arm")
    # * B21 is one-sided: only AT3L tightens; B22 tightens both.
    assert "while the AT3L jointing screw tightens;" in label_movement(process, *movement(purged, "B21_J_insert")).text
    assert "while the AT3L and AT3R jointing screws tighten;" in \
        label_movement(process, *movement(purged, "B22_J_insert")).text


def test_a_new_exporter_ground_bar_and_its_foundation_fix(purged):
    """The new exporter's ground bar: an insert without drives, then an operator step, J_M5's foundation fix."""
    process = purged.process
    jointing = action(purged, "B20_J")
    insert = replace(jointing.movements[-1], drives={})
    fix = replace(jointing.movements[1], id="B20_J_fix_foundation", label="Operator fixes the foundation")
    ground = replace(jointing, movements=(*jointing.movements[:-1], insert, fix))
    labels = [label_movement(process, ground, one) for one in ground.movements]
    assert [label.old_name for label in labels][3:] == ["J_M3_CDFM_transfer_to_approach", INSERT_GROUND,
                                                        "J_M5_manual_fix_foundation"]
    assert labels[5].code == "J_M5" and labels[5].kinds == (MANUAL,)


def test_an_insert_with_nothing_to_tighten_is_not_called_a_ground_bar(purged):
    """A screwed bar's insert whose drives are gone (an export defect) gets no old name and a neutral sentence."""
    jointing = action(purged, "B22_J")
    insert = replace(jointing.movements[-1], drives={})
    label = label_movement(purged.process, jointing, insert)
    assert (label.old_name, label.code, label.kinds) == ("", "J_insert", (CDLM,))
    assert "ground" not in label.text and "no jointing screw is sent" in label.text


def test_the_process_panel_shows_the_old_name_the_row_and_the_sentence(purged):
    """At a screwed insert: its kinds' chips, the action's codes with this one ringed, the old name and the sentence.

    The transfer workaround is on at start: the row is in run order, the transit first, no travel to load.
    """
    panel = ProcessPlugin()
    panel._take(purged.process)
    panel.select(next(index for index, (_, one) in enumerate(panel._movements) if one.id == "B22_J_insert"))
    html = panel._details_html(*panel.selected)
    assert f"old name {INSERT_TIGHTEN}" in html and "Straight-line insert 15 mm" in html
    for code in ("J_transit", "J_M1", "J_M2", "J_M4+5"):
        assert f">{code}</span>" in html
    assert ">J_M0</span>" not in html and ">J_M3</span>" not in html
    assert "1px dashed #666666" in html and "linear-gradient(90deg,#d5e8d4,#e6d0de)" in html
    # * Only the selected movement is ringed, and every chip keeps one weight, so nothing moves as the ring moves.
    assert html.count("box-shadow:inset 0 0 0 2px #212529;") == 1
    assert "font-weight:800" not in html


# --- --- --- --- --- WITHOUT THE DRIVE --- --- --- --- ---


@pytest.fixture
def small(tmp_path):
    """The small synthetic process (cindy and alice, five actions), written and loaded as the monitor loads it."""
    write(build_process(tmp_path), tmp_path / "process")
    return load_process(tmp_path / "process", tmp_path)


def _movement_like(process, movement_id: str = "X_M", **parts) -> Movement:
    """A movement with the given parts, starting where the process's first movement starts."""
    first = process.actions[process.schedule[0]].movements[0]
    return Movement(id=movement_id, start=first.start, **parts)


def test_kinds_by_parts(small):
    """Kinds come from the parts: arms, path and coupling; an operator end, a base or tools without arms."""
    two = ("robots/cindy/left_tool0", "robots/cindy/right_tool0")
    cases = [(dict(arms=two, path="free"), (FM,)), (dict(arms=two, path="linear"), (LM,)),
             (dict(arms=two, path="free", coupled=True), (CDFM,)),
             (dict(arms=two, path="linear", coupled=True), (CDLM,)),
             (dict(ends_on="operator"), (MANUAL,)), (dict(base=("robots/cindy",)), (BASE,)),
             (dict(ends_on="tools", target=Target(tools={"tools/cindy/AT3L": "closed"})), (TM,)),
             (dict(arms=two[:1], path="linear", target=Target(tools={"tools/cindy/AT3L": "open"})), (LM, TM_THROUGH))]
    for parts, kinds in cases:
        assert kinds_of(small, _movement_like(small, **parts)) == kinds, parts
    assert TM_THROUGH.dashed and not TM.dashed and CDFM.fill_to and not FM.fill_to


def test_support_gripper_opens_and_closes_and_a_bar_placed_by_hand(small):
    """A support robot's gripper step is H_M1 when it opens and H_M3 when it closes; a bar placed by hand is new."""
    holding = Action("X_H", "bar_holding", "robots/alice", "bars/B1", ())
    for grip, old_name in (("open", "H_M1_gripper_open"), ("closed", "H_M3_gripper_close")):
        gripper = _movement_like(small, ends_on="tools", target=Target(tools={"tools/alice/Grip": grip}))
        assert label_movement(small, replace(holding, movements=(gripper,)), gripper).old_name == old_name
    place = _movement_like(small, "X_P_place", ends_on="operator", label="Placed by hand")
    placing = Action("X_P", "bar_placing", "", "bars/B1", (place,))
    label = label_movement(small, placing, place)
    assert (label.old_name, label.code, label.kinds) == ("", "P_place", (MANUAL,))
    assert label.text.startswith("The operator places B1 by hand")


def test_the_transfer_workaround_transit(small):
    """With the workaround, a coupled free movement is the transit: "J_transit", a plain FM, its own sentence."""
    two = ("robots/cindy/left_tool0", "robots/cindy/right_tool0")
    transfer = _movement_like(small, "X_J_transfer", arms=two, path="free", coupled=True)
    jointing = Action("X_J", "bar_jointing", "robots/cindy", "bars/B1", (transfer,))
    label = label_movement(small, jointing, transfer, transfer_as_free=True)
    assert (label.code, label.old_name, label.kinds) == ("J_transit", "", (FM,))
    assert label.text == ("Cindy's two arms travel with empty tools from wherever they are to the insert start, where "
                          "the operator places the bar (FM; the transfer workaround, in place of J_M0 and J_M3).")
    assert label_movement(small, jointing, transfer).code == "J_M3"


def test_the_panel_escapes_rhino_labels(small):
    """A movement's label from Rhino is text, never HTML, in the chips' hover text and the sentence."""
    first_id = small.schedule[0]
    first = small.actions[first_id]
    odd = replace(first, movements=(replace(first.movements[0], label="<b>mount</b> & go"), *first.movements[1:]))
    panel = ProcessPlugin()
    panel._take(replace(small, actions={**small.actions, first_id: odd}))
    panel.select(0)
    html = panel._details_html(*panel.selected)
    assert "<b>mount</b>" not in html and "&lt;b&gt;mount&lt;/b&gt; &amp; go" in html
