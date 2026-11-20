"""
The kinds of part the guide walks through: each one's folder of controls, its live chips, and its description.

- One object per kind, built once; the plugin shows the shown part's kind and fills it with that part (`show`).
- Each kind is described the same way: its goal in a line (in the header at the top), what to do (the buttons to
  press, in order; the fix one layer down last, "To fix by hand: …"), what to check (`look_for`: only what this part
  needs). A kind whose texts depend on the part says so in `goal_of` and `todo_of` (a part by hand).
- A part by hand is confirmed by its checklist and Done; Prepare and Go are for parts that move a robot.
- Settings that belong to the run (insert split, spiral, servo by bar, retreat axis) are the run's: shown here and
  changed through `assembly_run`, so the Assembly tab shows the same values.
"""

from __future__ import annotations

import math
from html import escape
from typing import TYPE_CHECKING

import viser

from ...config import short_serial
from ...shared.execution.force_sensors import zero_refusal
from ...shared.guide.step import Step
from ...shared.movement_exec_contract import HANDOFF_PART, MAX_TRAJ_TIME, MovementExecContract, places_bar
from ...shared.runs.runs import (GRASP_CONTROLLERS, GRASP_JOINT, PUSH_RANGE, SPIRAL_SECONDS_RANGE, SPIRAL_TURNS_RANGE,
                                 STIFFNESS_RANGE, RunSettings)
from ...colors import DARK_GREY, INK, LIGHT_GREY
from ...ui.second_press import SecondPress, show_waiting
from ...ui.style import CURRENT, NONE, SECTION_CTRL, block, section, values, wrapped
from ...world.checks import GOOD, PENDING, Check
from .parts import (DRIVE, DRIVE_ARRIVED, DRIVE_YAW_ARRIVED, GRIPPER, HAND, INSERT, MARK, MOVE, PARK, REFUSED,
                    RETREAT, SERVO, GuidePart, floor_pose)

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from .plugin import AssemblyGuidePlugin

#: Rehearse's hint: a dry run of the part.
REHEARSE_HINT = "Run every check of the part and send nothing (a dry run of this part only); Mode Off only"
#: The most boxes a part by hand has.
HAND_BOXES = 4
#: The last box of every part by hand.
HANDS_CLEAR = ("Hands clear", "Nobody's hands, and no loose tool, near the robots")
#: How a `todo` line that says how to fix the part one layer down starts.
FIX_BY_HAND = "To fix by hand: "
#: The description's look: one light card, plain headings, no coloured bars.
CARD = f"background:{LIGHT_GREY}33;border-radius:4px;padding:4px 8px 6px;margin:2px 0;font-size:12px;color:{INK}"
HEADING = f"font-size:11px;font-weight:700;color:{DARK_GREY};margin-top:2px"
LIST = "margin:0 0 2px;padding-left:18px"
#: What every part that moves a robot checks.
E_STOP = "E-stop in hand."
#: The by-hand placing's button that zeroes the robot's force sensors.
ZERO = "Zero force (no bar in tools)"
#: What its first press asks.
ZERO_ASK = ("Only with NO bar in the tools: zeroing tares away whatever they hold, and every later compliant part "
            "pushes with that weight. Press again to zero every arm of the robot.")
#: What it asks once the part says the bar is in the tools: the press still counts, the operator knows best.
ZERO_ASK_HELD = ("The bar looks clamped already (a box is ticked or the part is done): zeroing now tares its weight "
                 "away. Press again to zero anyway.")


def description(kind: Kind, part: GuidePart | None = None) -> str:
    """The kind's text below the controls: a blue "what to do" line, then the steps, checks and fix in one light card.

    The goal is not here: the header at the top says it. `part`: the part shown, for a kind whose texts depend on it.
    """
    todo = kind.todo_of(part) if part is not None else kind.todo
    steps = [line for line in todo if not line.startswith(FIX_BY_HAND)]
    fixes = [line.removeprefix(FIX_BY_HAND) for line in todo if line.startswith(FIX_BY_HAND)]
    html = f'<ol style="{LIST}">' + "".join(f"<li>{escape(line)}</li>" for line in steps) + "</ol>"
    if kind.look_for:
        html += f'<div style="{HEADING}">Check</div><ul style="{LIST}">'
        html += "".join(f"<li>{escape(line)}</li>" for line in kind.look_for) + "</ul>"
    for fix in fixes:
        html += f'<div style="color:{NONE};font-size:11px;margin-top:2px"><b>Fix by hand:</b> {escape(fix)}</div>'
    return section("what to do", CURRENT) + block(f'<div style="{CARD}">{html}</div>')


def hand_items(part: GuidePart) -> list[tuple[str, str]]:
    """The boxes of a part by hand: one per thing the executor asks, then hands clear; each a short label and a hint."""
    items = []
    for plan in part.plans:
        for asked in plan.text.split("; "):
            if asked.startswith("clamp it"):
                items.append(("Screws clamped", asked.removeprefix("clamp it: ").capitalize()))
            elif "places the bar" in asked:
                items.append(("Bar placed", asked))
            else:
                items.append(("Done by hand", asked))
    return [*items[:HAND_BOXES - 1], HANDS_CLEAR]


class Kind(Step):
    """One kind of part: its controls, its live chips, and its description.

    Attributes:
        goal: What the part is for, in a line.
        todo: What to do, in order: the buttons to press; the fix one layer down last.
        look_for: What the operator checks before Go or Done; only what this part needs.
        manual: Whether Done confirms it (by hand, or a record), Prepare and Go staying grey.
    """

    goal = ""
    todo: tuple[str, ...] = ()
    look_for: tuple[str, ...] = ()
    manual = False

    def __init__(self, guide: AssemblyGuidePlugin):
        """Not built yet."""
        self.guide = guide

    def goal_of(self, part: GuidePart) -> str:
        """The goal line for `part`; the kind's own unless its texts depend on the part."""
        return self.goal

    def todo_of(self, part: GuidePart) -> tuple[str, ...]:
        """What to do for `part`; the kind's own unless its texts depend on the part."""
        return self.todo

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """Fill the controls with `part`'s values; called while it is the part shown."""

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """The part's own live chips, e.g. how far the base still is."""
        return []

    def ready(self, ctx: PluginContext) -> list:
        """Nothing keeps Go back but the question itself."""
        return []

    def _rehearse(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Add Rehearse: the part's dry run."""
        self._rehearse_button = gui.add_button("Rehearse", icon=viser.Icon.FLASK, hint=REHEARSE_HINT)
        self._rehearse_button.on_click(ctx.defer("rehearse", lambda: self.guide.prepare(ctx, rehearse=True)))

    def _show_rehearse(self) -> None:
        """Rehearse greyed while it cannot act (Auto, another part shown, a part running)."""
        disabled = not self.guide.can_rehearse
        if self._rehearse_button.disabled != disabled:
            self._rehearse_button.disabled = disabled


class DriveKind(Kind):
    """The drive to the action's start, through the base planner and the robot's follower."""

    name = DRIVE
    goal = "Bring the robot to the action's start."
    todo = ("Prepare: the base planner plans the drive.",
            "Play: watch the ghost drive it.",
            "Go: the robot's follower drives.",
            "Done: next part.",
            "To fix by hand: Plan › Base › Load: Step › Plan › Commit, or drive in Control › the robot › Base; "
            "then Done.")
    look_for = ("The path on the floor is clear of people and of what the scene does not know.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The target."""
        self._where = gui.add_html("")

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """Where the base goes."""
        x, y, yaw = part.base
        self._where.content = block(values(escape(f"target  x {x:+.3f} m  y {y:+.3f} m  "
                                                  f"yaw {math.degrees(yaw):+.1f}°")))

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """How far the base still is from the target."""
        serial = self.guide.serial_for(part)
        if serial is None or serial not in ctx.world.robots:
            return []
        x, y, yaw = part.base
        nx, ny, nyaw = floor_pose(ctx.kinematics.base_pose(serial))
        away, turn = math.hypot(nx - x, ny - y), (nyaw - yaw + math.pi) % (2 * math.pi) - math.pi
        there = away <= DRIVE_ARRIVED and abs(turn) <= DRIVE_YAW_ARRIVED
        return [Check("base", GOOD if there else PENDING,
                      f"The base is at the target within {DRIVE_ARRIVED * 100:.0f} cm and "
                      f"{math.degrees(DRIVE_YAW_ARRIVED):.0f}°" if there else "The base's distance to the target",
                      f"{away * 100:.1f} cm {math.degrees(turn):+.1f}°")]


class ParkKind(DriveKind):
    """A robot the action does not use, out of its way: its arms stowed by the arm planner, then driven to its park."""

    name = PARK
    goal = "Get a robot this action does not use out of its way: arms stowed, base at its parked pose (park.yaml)."
    todo = ("Prepare: the arm planner plans the stow (skipped when stowed).",
            "Play: watch the ghost; Go: the arms stow.",
            "Then the base planner plans the drive to the parked pose; Play, Go: the robot's follower drives.",
            "Done: next part. A robot parked already has nothing to do.",
            "Optional: a park that fails warns \"not parked\" and the guide goes on; Done leaves one with one press.",
            "To fix by hand: Plan › Arm › Load: Stow › Plan › Commit, then Plan › Base › Load: Park › Plan › Commit; "
            "then Done.")
    look_for = ("The stowing arms clear the bars and the other robots.",
                "The path on the floor is clear of people and of what the scene does not know.", E_STOP)

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """Which robot, where it parks."""
        x, y, yaw = part.base
        self._where.content = block(values(escape(f"park  x {x:+.3f} m  y {y:+.3f} m  yaw {math.degrees(yaw):+.1f}°"),
                                           "arms stowed"))

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """Whether the arms are stowed, and how far the base still is."""
        if part.robot not in ctx.world.robots:
            return []
        stowed = self.guide.stowed(ctx, part)
        return [Check("stowed", GOOD, "Every arm is at its stow joints") if stowed else
                Check("not stowed", PENDING, "The arms are away from their stow joints"), *super().live(ctx, part)]


class MoveKind(Kind):
    """An arm move of the process (transit, transfer, home, approach, a straight line, leave), through the executor."""

    name = MOVE
    goal = "Move the arms to this part's goal."
    todo = ("Prepare: plans the move from the live arms (the planner chip above names the planner).",
            "Play: watch the ghost.",
            "Rehearse (optional): every check, nothing sent.",
            "Go: the executor sends it.",
            "Done: next part.",
            "To fix by hand: Plan › Arm › Arm: Both (or the one arm) › Load: Step IK › Plan › Commit, or jog in "
            "Control › the robot "
            "› the arm; then Done.")
    look_for = ("The ghost's arms clear the bars, the tools and the other robots.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The goal found, the motion time, Rehearse."""
        self._goal = gui.add_html("")
        self._time = gui.add_slider("Motion time", min=1.0, max=MAX_TRAJ_TIME, step=0.5, initial_value=10.0,
                                    hint="How long the move takes, seconds; the arms' limits may need longer. "
                                         "Changeable until Go (a plan waiting for Go is timed again, its preview "
                                         "too); greyed while the move runs, which keeps the time it was sent with")
        self._time.on_update(ctx.defer_value("motion time", lambda value: self.guide.set_traj_time(float(value)),
                                             operator_only=True))
        self._rehearse(ctx, gui)

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """The goal line, and the motion time."""
        self._show_rehearse()
        self._goal.content = block(values(escape(self.guide.goal_text or "goal: solved at Prepare")))
        if abs(self._time.value - self.guide.traj_time) > 1e-6:
            self._time.value = self.guide.traj_time
        # * The move sent has its timing: a change then would not reach it.
        moving = self.guide.moving is part
        if self._time.disabled != moving:
            self._time.disabled = moving

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """How far the arms still are from the goal."""
        error = self.guide.goal_error(ctx, part)
        if error is None:
            return []
        there = error <= math.radians(1.0)
        return [Check("arms", GOOD if there else PENDING, "The largest joint's distance to the goal",
                      f"{math.degrees(error):.1f}° from goal")]


class HandKind(Kind):
    """A part done by hand (the bar placed and clamped, or any other): its boxes ticked, Done records it."""

    name = HAND
    manual = True
    goal = "Do this part by hand: what the folder above asks."
    todo = ("Do by hand what the folder above asks.",
            "Tick every box above.",
            "Done: the executor records it, the next part follows.")
    #: The bar placed in the tools and clamped.
    PLACE_GOAL = "Place the bar in the tools and clamp it, by hand."
    PLACE_TODO = (f"{ZERO}, twice, while the tools are still empty: zeroes the arms' force sensors.",
                  "Put the bar into both tools' jaws.",
                  "Tighten each gripper screw with the tool's own buttons until it stalls, one button at a time (B1 "
                  "and B2 together put the arm in free drive).",
                  "Tick every box above.",
                  "Done: the executor records it, the next part follows.")

    @staticmethod
    def places_bar(part: GuidePart) -> bool:
        """Whether the part places a bar in the tools and clamps it (`hand_items`' words)."""
        return any(places_bar(plan) for plan in part.plans)

    def goal_of(self, part: GuidePart) -> str:
        """Placing the bar, or the part's own ask."""
        return self.PLACE_GOAL if self.places_bar(part) else self.goal

    def todo_of(self, part: GuidePart) -> tuple[str, ...]:
        """Placing the bar, or the generic steps."""
        return self.PLACE_TODO if self.places_bar(part) else self.todo

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Zero force, what the executor asks, and the boxes (a pool, labelled per part)."""
        # * Always there, grey but for placing a bar: shown only then, it would move everything below.
        self._zero_press = SecondPress()
        self._zero = gui.add_button(ZERO, disabled=True, icon=viser.Icon.SCALE,
                                    hint=f"Tares every arm's force sensor of the part's robot. ! Only with no bar in "
                                         f"the tools. {self._zero_press.hint}")
        self._zero.on_click(ctx.defer("zero force", lambda: self._press_zero(ctx)))
        self._what = gui.add_html("")
        gui.add_html(section("what to check", SECTION_CTRL))
        self.boxes = [gui.add_checkbox(f"box {index + 1}", False, visible=False) for index in range(HAND_BOXES)]
        self._items: list[tuple[str, str]] = []
        self._for: GuidePart | None = None

    @property
    def ticked(self) -> bool:
        """bool: Whether every box of the part is ticked."""
        shown = self.boxes[:len(self._items)]
        return bool(shown) and all(box.value for box in shown)

    @property
    def count(self) -> tuple[int, int]:
        """tuple: How many boxes are ticked, and how many there are."""
        shown = self.boxes[:len(self._items)]
        return sum(bool(box.value) for box in shown), len(shown)

    def clear(self) -> None:
        """Untick every box, for the next part by hand."""
        for box in self.boxes:
            if box.value:
                box.value = False

    def forget(self) -> None:
        """Untick every box and forget the part they were for: the next part by hand shown starts afresh (Redo)."""
        self.clear()
        self._for = None

    def tick_all(self) -> None:
        """Tick every box (God mode)."""
        for box in self.boxes[:len(self._items)]:
            box.value = True

    def _press_zero(self, ctx: PluginContext) -> None:
        """Zero force's press: the first asks, the second has the executor zero the robot's arms and record it."""
        part, serial = self.guide.part, self.guide.serial
        if part is None or part.kind != HAND or not self.places_bar(part) or serial is None:
            return
        robot = ctx.world.robots.get(serial)
        refusal = (f"{short_serial(serial)} is not connected" if robot is None
                   else zero_refusal(serial, robot, ctx.now()))
        if refusal:
            self.guide._refuse(ctx, ZERO, refusal)
            return
        held = self._looks_held(ctx, part)
        if not self._zero_press.press(ctx.now(), self._zero_key(serial, held)):
            self.guide._refuse(ctx, ZERO, ZERO_ASK_HELD if held else ZERO_ASK, ask=True)
            return
        if held:
            ctx.log_warn(f"{short_serial(serial)}: force sensors zeroed though the bar looks clamped in the tools")
        executor: MovementExecContract = ctx.require("movement_exec")
        executor.request_zero_force(serial, "guide", part.action, part.movement, held).add_done_callback(
            lambda done: self._zeroed(ctx, done.result()))

    def _zeroed(self, ctx: PluginContext, why: str) -> None:
        """Say why the executor's zero did not go through; nothing once it did."""
        if why:
            self.guide._refuse(ctx, ZERO, why)

    def _looks_held(self, ctx: PluginContext, part: GuidePart) -> bool:
        """Whether the part says the bar is in the tools already: a box ticked, or the part done."""
        return self.count[0] > 0 or self.guide.is_done(ctx, part, self.guide.index)

    def _zero_key(self, serial: str | None, held: bool) -> str:
        """What a first press of Zero force asked about: this part's robot, and whether the bar looked held."""
        return f"zero {serial} {self.guide.index} {held}"

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """Zero force live only while placing a bar; what the part asks; its boxes."""
        places = self.places_bar(part) and self.guide.serial is not None
        if self._zero.disabled == places:
            self._zero.disabled = not places
        asked = self._zero_key(self.guide.serial, self._looks_held(ctx, part))
        show_waiting(self._zero, self._zero_press.waiting(ctx.now()) == asked, ZERO, None)
        self._what.content = block(wrapped(*(plan.text for plan in part.plans)))
        if part is self._for:
            return
        # * Another part by hand: its own boxes, unticked.
        self.clear()
        self._for, self._items = part, hand_items(part)
        for index, box in enumerate(self.boxes):
            item = self._items[index] if index < len(self._items) else None
            if item is not None:
                box.label, box.hint = item
            # * Only on another part: every assignment is a message to each browser.
            box.visible = item is not None

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """How many boxes are ticked."""
        ticked, count = self.count
        return [Check("boxes", GOOD if count and ticked == count else PENDING,
                      "Done records the part once every box is ticked", f"{ticked}/{count}")]


class ServoKind(Kind):
    """The mocap servo after a transfer, an approach or a ground bar's insert, run by the executor pass by pass."""

    name = SERVO
    goal = "Correct the arms onto this part's target from what mocap measures."
    todo = ("Prepare: the executor plans the first pass from the live arms.",
            "Go: it runs the pass; a later pass travelling over 5° asks Go again.",
            "Done: next part.",
            "To fix by hand: Cancel, then Plan › Arm or Control; then Done twice to skip.")
    look_for = ("Each pass is small: the Exec plots in the Assembly tab.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Servo by bar."""
        self._by_bar = gui.add_checkbox("Servo by bar", True,
                                        hint="A ground bar held in the tools servos on its mocap fit instead of "
                                             "tool0 (the run's setting); every other bar servos on tool0")
        self._by_bar.on_update(ctx.defer_value("servo by bar", lambda value: self.guide.request_settings(
            servo_by_bar=bool(value)), operator_only=True))

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """The run's setting."""
        wanted = self.guide.settings.servo_by_bar
        if self._by_bar.value != wanted:
            self._by_bar.value = wanted


class InsertKind(Kind):
    """The insert: rigid up to the split, then compliant while the mating screws tighten; a ground bar rigid."""

    name = INSERT
    goal = "Insert the bar into its joint."
    todo = ("Prepare: the motion planner plans the line from where the servo left the flanges.",
            "Rehearse (optional): every compliant target checked, nothing sent.",
            "Go: the executor runs it.",
            "Done: next part.",
            "If it says ARMS MAY BE LEFT under the compliance controller: Control › the arm › Controller: Joint, "
            "before anything else.")
    look_for = ("The bar enters its joint.", "Every held bar stays in its jaws.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The run's insert settings, Rehearse."""
        self._split = gui.add_slider("Insert split mm", min=0.0, max=30.0, step=0.5, initial_value=5.0,
                                     hint="The insert's last millimetres run compliant (the run's setting)")
        self._spiral = gui.add_checkbox("Spiral", False, hint="The compliant part searches in a spiral; only for a "
                                                              "bar with two mating joints")
        self._seconds = gui.add_slider("Spiral s", min=SPIRAL_SECONDS_RANGE[0], max=SPIRAL_SECONDS_RANGE[1], step=1.0,
                                       initial_value=20.0, hint="Seconds the spiral takes to reach its 15 mm radius")
        self._turns = gui.add_slider("Spiral turns", min=SPIRAL_TURNS_RANGE[0], max=SPIRAL_TURNS_RANGE[1], step=1,
                                     initial_value=4, hint="Turns the spiral makes; pitch = 15 mm / turns")
        default = RunSettings()
        self._cancel_weight = gui.add_checkbox("Cancel bar weight", default.insert_cancel_weight,
                                               hint="Take the bar's weight, measured at the insert's start before "
                                                    "anything moves, off the arms (the run's setting)")
        self._push = gui.add_slider("Insert push N", min=PUSH_RANGE[0], max=PUSH_RANGE[1], step=0.5,
                                    initial_value=default.insert_push_n,
                                    hint="Each mating arm pushes this along its line until its screw is tight")
        self._across = gui.add_slider("Stiffness across", min=STIFFNESS_RANGE[0], max=STIFFNESS_RANGE[1], step=100.0,
                                      initial_value=default.insert_stiffness_across,
                                      hint="Compliance across the tool (tool0 x, y), N/m")
        self._along = gui.add_slider("Stiffness along", min=STIFFNESS_RANGE[0], max=STIFFNESS_RANGE[1], step=100.0,
                                     initial_value=default.insert_stiffness_along,
                                     hint="Compliance along the tool (tool0 z), N/m")
        for widget, setting, kind in ((self._split, "insert_split_mm", float), (self._spiral, "spiral", bool),
                                      (self._seconds, "spiral_seconds", float), (self._turns, "spiral_turns", float),
                                      (self._cancel_weight, "insert_cancel_weight", bool),
                                      (self._push, "insert_push_n", float),
                                      (self._across, "insert_stiffness_across", float),
                                      (self._along, "insert_stiffness_along", float)):
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind:
                                             self.guide.request_settings(**{setting: kind(value)}),
                                             operator_only=True))
        self._rehearse(ctx, gui)

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """The run's values; Spiral only for two mating screws."""
        self._show_rehearse()
        settings = self.guide.settings
        two = len(part.plans[0].tools) == 2 if part.plans else False
        for widget, value in ((self._split, settings.insert_split_mm), (self._seconds, settings.spiral_seconds),
                              (self._turns, settings.spiral_turns), (self._spiral, settings.spiral and two),
                              (self._cancel_weight, settings.insert_cancel_weight),
                              (self._push, settings.insert_push_n), (self._across, settings.insert_stiffness_across),
                              (self._along, settings.insert_stiffness_along)):
            if widget.value != value:
                widget.value = value
        if self._spiral.disabled == two:
            self._spiral.disabled = not two


class RetreatKind(Kind):
    """The retreat: the jaws loosen while the tools back straight out, then the rest of the line rigid."""

    name = RETREAT
    goal = "Let go of the bar and back the tools out."
    todo = ("Prepare: the line back out is planned along the tools' live axes.",
            "Rehearse (optional): the screw-out checked, nothing sent.",
            "Go: the executor runs it.",
            "Done: next part.",
            "To fix by hand: Cancel, jog the arms out in Control › the robot › the arm; then Done twice to skip.")
    look_for = ("Both tools let go before they back off.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The run's retreat axis, Rehearse."""
        self._live = gui.add_checkbox("Retreat live axis", True,
                                      hint="Each tool backs out along its own live -z, as the encoders read it; off: "
                                           "along the export's axis (the run's setting)")
        self._live.on_update(ctx.defer_value("retreat live axis", lambda value: self.guide.request_settings(
            retreat_live_axis=bool(value)), operator_only=True))
        self._rehearse(ctx, gui)

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """The run's setting."""
        self._show_rehearse()
        wanted = self.guide.settings.retreat_live_axis
        if self._live.value != wanted:
            self._live.value = wanted


class GripperKind(Kind):
    """A gripper step: the Robotiq opens, or closes on a bar with the arm compliant or stiff (the run's grasp)."""

    name = GRIPPER
    goal = "Open the Robotiq, or close it on the bar with the arm compliant or held stiff (Grasp controller)."
    todo = ("Prepare: the executor asks to run it.",
            "Go: the gripper moves.",
            "Done: next part.",
            "To fix by hand: Control › the robot › the arm's TOOL section › Gripper Open or Close; then Done twice.")
    look_for = ("Before an open at a bar: the gripper stands about 100 mm in front of the bar, centred on it (1-2 cm "
                "off sideways: stop, the base calibration is off).",
                "The other robot's tools are not pulled.",
                "Grasp controller joint: the frame moves as the close pulls the bar; stop if a joint or a post "
                "bends.", E_STOP)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """What the step does; the run's grasp settings, live on a close."""
        self._what = gui.add_html("")
        self._grasp = gui.add_dropdown("Grasp controller", options=list(GRASP_CONTROLLERS),
                                       initial_value=GRASP_CONTROLLERS[0],
                                       hint="The arm's controller during the close (the run's setting): compliance "
                                            "gives way to the bar; joint holds the arm, so the close pulls the bar "
                                            "and the frame to the gripper")
        self._stiffness = gui.add_slider("Grasp stiffness", min=STIFFNESS_RANGE[0], max=STIFFNESS_RANGE[1],
                                         step=100.0, initial_value=500.0,
                                         hint="The compliant grasp's stiffness along tool0 x, y, z, N/m")
        for widget, setting, kind in ((self._grasp, "grasp_controller", str),
                                      (self._stiffness, "grasp_stiffness", float)):
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind:
                                             self.guide.request_settings(**{setting: kind(value)}),
                                             operator_only=True))

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """What the step does; the run's grasp settings, greyed for an open."""
        self._what.content = block(wrapped(*(plan.text for plan in part.plans)))
        settings = self.guide.settings
        closes = any(plan.kind == HANDOFF_PART for plan in part.plans)
        for widget, value in ((self._grasp, settings.grasp_controller), (self._stiffness, settings.grasp_stiffness)):
            if widget.value != value:
                widget.value = value
        self._grasp.disabled = not closes
        self._stiffness.disabled = not closes or settings.grasp_controller == GRASP_JOINT


class MarkKind(Kind):
    """Mark done: the run records the action done and where every live robot stands."""

    name = MARK
    manual = True
    goal = "Record the action done in the run."
    todo = ("Done: the run records the action and where every live robot stands; the next action follows.",
            "By hand: the Assembly tab › Mark done.")
    look_for = ("Every part above is done (the overview's chips).",)

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Nothing: the chips say how many parts are done."""

    def live(self, ctx: PluginContext, part: GuidePart) -> list[Check]:
        """How many of the action's parts are done."""
        parts = [(index, each) for index, each in enumerate(self.guide.parts)
                 if each.action == part.action and each.kind != MARK]
        done = sum(self.guide.is_done(ctx, each, index) for index, each in parts)
        return [Check("parts", GOOD if done == len(parts) else PENDING, "The action's parts done",
                      f"{done}/{len(parts)} done")]


class RefusedKind(Kind):
    """A part the executor cannot run yet."""

    name = REFUSED
    manual = True
    goal = "Do this part outside the guide: the executor cannot run it yet."
    todo = ("Do it by hand in Control, or in the Assembly tab.",
            "Done twice: skip it.")

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Why."""
        self._why = gui.add_html("")

    def show(self, ctx: PluginContext, part: GuidePart) -> None:
        """Why."""
        self._why.content = block(wrapped(part.why or part.label))


#: Every kind, in the order their folders are built.
KINDS = (DriveKind, ParkKind, MoveKind, HandKind, ServoKind, InsertKind, RetreatKind, GripperKind, MarkKind,
         RefusedKind)
