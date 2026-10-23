"""
Each process movement as the operator knows it: its old monitor name, flowchart kind and colours, and a sentence.

* The flowchart is bar_joint_rhino_design_workflow/docs/action_flowchart_2026.drawio. Its codes (J_M0 to HR_M1) are
  the old monitor's movement names in exports since 261006; older exports numbered the release differently
  (R_M0_tool_untighten_joint to R_M3_free_home). Its legend gives the kinds and their colours.
! A movement is matched by its routine (its parts), its action type and the drives the monitor really sends
  (`mating_drives`), never by its id. A movement the old monitor did not have (a support robot leaving, a bar placed
  by hand) gets no old name.
"""

from __future__ import annotations

from dataclasses import dataclass

from scaffolding_assembly_core.process import Action, Movement, Process

from .movement import Routine, mating_drives, routine_or_none


@dataclass(frozen=True)
class Kind:
    """A kind of movement in the flowchart's legend, with its colours.

    Attributes:
        short: The flowchart's abbreviation, e.g. "CDFM".
        name: What it means, for hover text.
        fill: Background colour.
        border: Border colour.
        fill_to: A second background colour the fill fades into, left to right; "" for none.
        dashed: A dashed border: a tool movement that keeps running through an arm movement.
    """

    short: str
    name: str
    fill: str
    border: str
    fill_to: str = ""
    dashed: bool = False


# * The flowchart's legend, colour for colour.
FM = Kind("FM", "free robotic movement", "#dae8fc", "#6c8ebf")
LM = Kind("LM", "linear robotic movement", "#d5e8d4", "#82b366")
CDFM = Kind("CDFM", "constrained dual-arm free movement: both arms hold one bar", "#dae8fc", "#6c8ebf", "#e6d0de")
CDLM = Kind("CDLM", "constrained dual-arm linear movement: both arms hold one bar, straight-line flanges",
            "#d5e8d4", "#82b366", "#e6d0de")
# ? The flowchart's Manual box has no stroke colour; a mid teal keeps it apart from the ring that marks a selection.
MANUAL = Kind("Manual", "manual movement: an operator step, no planning", "#d0f0f0", "#5f9ea0")
TM = Kind("TM", "tool movement: screws or gripper, no planning", "#f5f5f5", "#666666")
TM_THROUGH = Kind("TM", "tool movement that keeps running through the arm movement", "#f5f5f5", "#666666",
                  dashed=True)
#: Not in the flowchart: a movement that drives a base.
BASE = Kind("base", "base drive", "#ffffff", "#999999")

#: The old monitor's name of each movement, by action type and routine. Operator steps, inserts and a support
#: robot's gripper are told apart further in `_old_name`.
OLD_NAMES = {
    ("bar_jointing", Routine.FREE_TO_LOAD): "J_M0_free_to_load",
    ("bar_jointing", Routine.TOOL_ONLY): "J_M2_tool_grasp_bar",
    ("bar_jointing", Routine.TRANSFER_HELD): "J_M3_CDFM_transfer_to_approach",
    ("bar_jointing", Routine.TRANSFER_FREE): "J_M3_CDFM_transfer_to_approach",
    ("bar_release", Routine.TOOL_ONLY): "R_M0_tool_ungrasp_bar",
    ("bar_release", Routine.RETREAT_LINEAR): "R_M1_LM_retreat",
    ("bar_release", Routine.FREE_HOME): "R_M2_free_home",
    ("bar_holding", Routine.SUPPORT_APPROACH): "H_M0_free_to_approach",
    ("bar_holding", Routine.SUPPORT_LINEAR): "H_M2_LM_to_grasp",
    ("bar_holding_release", Routine.TOOL_ONLY): "HR_M0_gripper_open",
    ("bar_holding_release", Routine.SUPPORT_LINEAR): "HR_M1_LM_retreat",
}
#: The old name of an insert whose jointing screws tighten during it (the old J_M4 and J_M5 in one movement).
INSERT_TIGHTEN = "J_M4_tool_tighten_joint + J_M5_LM_insert"
#: The old name of a ground bar's insert: rigid, no screw runs.
INSERT_GROUND = "J_M4_LM_insert"
#: Each action type's letter in the old names.
PREFIXES = {"bar_jointing": "J", "bar_release": "R", "bar_holding": "H", "bar_holding_release": "HR",
            "bar_placing": "P"}

#: What each old movement does, from the flowchart's boxes: {robot} and {bar} are filled in, {line} with the
#: distance of a straight move and a space ("50 mm "), or nothing, and {screws} with the screws that tighten.
TEXTS = {
    "J_M0_free_to_load": "{robot}'s two arms travel from wherever they are to the bar loading pose (FM, arms "
                         "independent, planned live).",
    "J_M1_manual_mount_bar": "The operator places {bar}, joint halves already installed, into {robot}'s two end "
                             "effectors (Manual).",
    "J_M2_tool_grasp_bar": "Both arm tools drive their grasping screws until they stall: {bar} is clamped (TM).",
    "J_M3_CDFM_transfer_to_approach": "{robot} carries {bar} to the approach pose, just short of the assembled pose "
                                      "(constrained dual-arm FM).",
    INSERT_TIGHTEN: "Straight-line insert {line}to the assembled pose, compliant, while {screws}; the stall ends it "
                    "(TM through a constrained dual-arm LM).",
    INSERT_GROUND: "Straight-line insert {line}of the ground bar to the assembled pose, rigid, ended when the target "
                   "is reached; no jointing screw runs (constrained dual-arm LM).",
    "J_M5_manual_fix_foundation": "The operator fixes the foundation to the ground while {robot} keeps holding "
                                  "{bar} (Manual).",
    "R_M0_tool_ungrasp_bar": "Both tools run the grasping screws backwards: {bar} becomes part of the structure; "
                             "the jointing screws are never reversed (TM).",
    "R_M1_LM_retreat": "Each arm retreats {line}along its joint's -Z axis, arms independent (LM).",
    "R_M2_free_home": "Both arms travel to the fixed dual-arm home configuration (FM, arms independent).",
    "H_M0_free_to_approach": "{robot}'s arm travels from wherever it is to the approach pose, off the grasp along "
                             "the gripper axis (single-arm FM, planned live).",
    "H_M1_gripper_open": "{robot}'s gripper opens to receive {bar} (TM).",
    "H_M2_LM_to_grasp": "Straight-line approach {line}onto the hand-picked grasp pose on {bar} (single-arm LM).",
    "H_M3_gripper_close": "{robot}'s gripper closes: {bar} is held, and {robot} stays here until every bar it "
                          "supports is built (TM).",
    "HR_M0_gripper_open": "{robot}'s gripper opens, letting go of the now stable {bar} (TM).",
    "HR_M1_LM_retreat": "{robot}'s arm retreats straight back {line}to its approach pose, the approach line "
                        "reversed (single-arm LM).",
}
#: What a movement the old monitor did not have does, by routine.
NEW_TEXTS = {
    Routine.INSERT: "Straight-line insert {line}to the assembled pose; no jointing screw is sent (constrained "
                    "dual-arm LM; not in the old monitor).",
    Routine.SUPPORT_LEAVE: "{robot}'s arm moves away from the structure on a free path (single-arm FM; not in the "
                           "old monitor).",
    Routine.OPERATOR: "An operator step, confirmed on the monitor: {label} (Manual; not in the old monitor).",
    Routine.BASE_ONLY: "{robot}'s base drives (no movement planner drives a base yet).",
}
#: A bar_placing action's one movement.
PLACING_TEXT = "The operator places {bar} by hand; no robot acts (Manual; not in the old monitor)."


@dataclass(frozen=True)
class MovementLabel:
    """A movement as the operator knows it.

    Attributes:
        code: Short, for a row of chips: the old code ("J_M3", "J_M4+5"), or the action letter and the id's ending
            ("HR_leave") for a movement the old monitor did not have.
        old_name: The old monitor's name, e.g. "J_M3_CDFM_transfer_to_approach"; "" if it had none.
        kinds: The flowchart's kinds, the arm part first; a tool part that runs through it comes second.
        text: One sentence on what the movement does.
    """

    code: str
    old_name: str
    kinds: tuple[Kind, ...]
    text: str


def kinds_of(process: Process, movement: Movement) -> tuple[Kind, ...]:
    """The flowchart's kinds of a movement, from its parts and the drives the monitor really sends.

    Args:
        process: The process, for the mates that decide which drives are sent.
        movement: The movement.

    Returns:
        tuple[Kind, ...]: One kind; with a second one, TM_THROUGH, when tools act during an arm movement.
    """
    if not movement.arms:
        if movement.ends_on == "operator":
            return (MANUAL,)
        if movement.base:
            return (BASE,)
        return (TM,)
    linear = movement.path == "linear"
    if movement.coupled:
        arm = CDLM if linear else CDFM
    else:
        arm = LM if linear else FM
    # * A ground bar's tightens are never sent: no tool runs through its insert.
    tools = movement.grip_change or (mating_drives(process, movement).kept if movement.drives else {})
    return (arm, TM_THROUGH) if tools else (arm,)


def label_movement(process: Process, action: Action, movement: Movement) -> MovementLabel:
    """A movement's old name, flowchart kinds and description.

    Args:
        process: The process, for the robot's name and the mates.
        action: The action the movement belongs to.
        movement: The movement.

    Returns:
        MovementLabel: The label; a movement whose parts match no routine gets its own id and Rhino's label.

    Example:
        >>> label_movement(process, action, movement).old_name
        'J_M3_CDFM_transfer_to_approach'
    """
    robot = process.robots[action.robot].name.capitalize() if action.robot in process.robots else "the robot"
    words = {"robot": robot, "bar": action.bar.split("/")[-1], "line": _line(movement),
             "label": movement.label or movement.id, "screws": _screws(process, movement)}
    routine = routine_or_none(action, movement)
    old_name = _old_name(process, action, movement, routine) if routine is not None else ""
    if old_name:
        code = "J_M4+5" if old_name == INSERT_TIGHTEN else "_".join(old_name.split("_")[:2])
        text = TEXTS[old_name]
    else:
        prefix = PREFIXES.get(action.type, action.type)
        code = f"{prefix}_{movement.id.removeprefix(action.id + '_')}"
        text = PLACING_TEXT if action.type == "bar_placing" else NEW_TEXTS.get(routine, "{label}")
    sentence = text.format(**words)
    return MovementLabel(code, old_name, kinds_of(process, movement), sentence[:1].upper() + sentence[1:])


def _old_name(process: Process, action: Action, movement: Movement, routine: Routine) -> str:
    """The old monitor's name of a classified movement, or "" if it had none."""
    if action.type == "bar_jointing" and routine == Routine.OPERATOR:
        # ? An operator step after the insert is a ground bar's foundation fix; before it, the bar is placed.
        earlier = action.movements[:action.movements.index(movement)]
        inserted = any(routine_or_none(action, each) == Routine.INSERT for each in earlier)
        return "J_M5_manual_fix_foundation" if inserted else "J_M1_manual_mount_bar"
    if action.type == "bar_jointing" and routine == Routine.INSERT:
        # ? Screws really sent win: a bar on the ground at one end may still tighten at the other.
        drives = mating_drives(process, movement)
        if drives.kept:
            return INSERT_TIGHTEN
        return INSERT_GROUND if drives.grounded else ""
    if action.type == "bar_holding" and routine == Routine.TOOL_ONLY:
        closes = "closed" in movement.grip_change.values()
        return "H_M3_gripper_close" if closes else "H_M1_gripper_open"
    return OLD_NAMES.get((action.type, routine), "")


def _line(movement: Movement) -> str:
    """A straight move's distance and a space, e.g. "50 mm ", when every flange moves as far; else ""."""
    distances = {round(spec.distance * 1000) for spec in movement.line.values()}
    return f"{distances.pop()} mm " if len(distances) == 1 else ""


def _screws(process: Process, movement: Movement) -> str:
    """The tightening the monitor sends, as words: "the AT3L jointing screw tightens", or "" if none."""
    if not movement.drives:
        return ""
    tools = sorted(tool.split("/")[-1] for tool, drive in mating_drives(process, movement).kept.items()
                   if drive == "tighten")
    if not tools:
        return ""
    if len(tools) == 1:
        return f"the {tools[0]} jointing screw tightens"
    return f"the {' and '.join(tools)} jointing screws tighten"
