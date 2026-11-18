"""PlanMargins: which margin a pair keeps, the smallest, and the texts, without a planning world."""

from __future__ import annotations

from husky_assembly_teleop.shared.planner.plan_margins import ARM_PLAN_MARGINS, Clamp, MarginRule, PlanMargins

FLOOR, BOX, LINK = "obstacles/floor", "t/box", "robots/0804/ur_arm_wrist_1_link"
WALL = "obstacles/boxes/border_x_min"


def test_a_pair_keeps_its_rule_else_the_default():
    """The floor rule gives any pair with the floor 5 cm, whichever side it is on; other pairs keep 1 cm."""
    assert ARM_PLAN_MARGINS.usual(LINK, FLOOR) == ARM_PLAN_MARGINS.usual(FLOOR, LINK) == 0.05
    assert ARM_PLAN_MARGINS.usual(LINK, BOX) == 0.01


def test_a_third_margin_is_one_more_rule_and_the_first_matching_wins():
    """Another rule needs nothing else: its pairs, the smallest and the texts follow; the first matching rule wins."""
    walls = MarginRule("the walls", lambda part: "/border_" in part, 0.03)
    margins = PlanMargins(0.01, (*ARM_PLAN_MARGINS.rules, walls))
    assert margins.usual(LINK, WALL) == 0.03 and margins.usual(WALL, FLOOR) == 0.05
    assert margins.describe() == "1 cm, 5 cm from the floor, 3 cm from the walls"
    tight = PlanMargins(0.01, (MarginRule("the walls", lambda part: "/border_" in part, 0.004),))
    assert tight.smallest == 0.004, "a rule below the default sets the clearance too"


def test_the_smallest_margin_is_the_least_clamp_or_the_default():
    """A floor clamped to 3 cm leaves the smallest at 1 cm; a box clamped to 3 mm takes it below."""
    floor = Clamp(LINK, FLOOR, 0.03, 0.05)
    assert PlanMargins(0.01, ARM_PLAN_MARGINS.rules, (floor,)).smallest == 0.01
    box = Clamp(LINK, BOX, 0.003, 0.01)
    assert PlanMargins(0.01, ARM_PLAN_MARGINS.rules, (box, floor)).smallest == 0.003


def test_the_note_names_the_closest_clamp_against_its_own_usual_margin():
    """Floor at 3 cm and a box at 3 mm: the box first, against its 1 cm; the rest as usual, the floor counted."""
    clamps = (Clamp(LINK, BOX, 0.003, 0.01), Clamp(LINK, FLOOR, 0.03, 0.05))
    margins = PlanMargins(0.01, ARM_PLAN_MARGINS.rules, clamps)
    assert margins.note() == ("keeps only 0.3 cm between ur_arm_wrist_1_link and t/box (and 1 more pair): the start "
                              "or target is that close (usual 1 cm); elsewhere 1 cm, 5 cm from the floor")
    floor_only = PlanMargins(0.01, ARM_PLAN_MARGINS.rules, (Clamp(LINK, FLOOR, 0.03, 0.05),))
    assert floor_only.note().startswith("keeps only 3 cm between ur_arm_wrist_1_link and the floor")
    touching = PlanMargins(0.01, (), (Clamp(LINK, BOX, 0.0, 0.01),))
    assert touching.note() == ("only free of contact between ur_arm_wrist_1_link and t/box: the start or target "
                               "touches (usual 1 cm); elsewhere 1 cm")
    assert PlanMargins(0.01).note() == "keeps 1 cm"
