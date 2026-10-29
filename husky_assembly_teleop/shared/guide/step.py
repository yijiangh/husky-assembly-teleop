"""One step of a guided procedure: its result's checks, its state in the overview, and what it needs to act."""

from __future__ import annotations

from typing import TYPE_CHECKING, Sequence

from ...ui.style import note
from ...world.checks import BAD, GOOD, PENDING, Check

if TYPE_CHECKING:
    import viser

    from ...plugin_api.context import PluginContext


def red_checks(checks: Sequence[Check]) -> list[Check]:
    """The BAD checks: while there are any, the step's result fails."""
    return [check for check in checks if check.level == BAD]


def fails(checks: Sequence[Check]) -> bool:
    """Whether any check is BAD: the step's result fails."""
    return bool(red_checks(checks))


def in_short(checks: Sequence[Check]) -> str:
    """The checks as the overview's short text: each one's label and figure, e.g. "takes 4/6, left out 2"."""
    return ", ".join(f"{check.label} {check.value}".strip() for check in checks)


def unfinished(state: Check) -> bool:
    """Whether a step's state (`Step.state`) leaves it unfinished: BAD or PENDING, so Done asks once more."""
    return state.level in (BAD, PENDING)


class Step:
    """One step: what the operator does in it, its result's state, and whether its actions can run.

    - `checks` judge the step's result as it stands now; `state` sums them up for the overview, and Done asks once
      more before leaving a step whose state is unfinished.
    - A precondition step (`precondition`) gathers what later steps `need` to act, and says when it has `expired`.
    - `ready` judges whether the step's actions can run now. Actions refuse while one is red (`refusal`); it never
      keeps the operator in the step.
    - Checks and states are computed on every draw: keep them cheap.
    """

    #: The step's name in the list, and what the operator does in it.
    name = ""
    instructions = ""
    #: Whether the step gathers a precondition for later steps rather than a result: its overview row is dashed.
    precondition = False
    #: The precondition steps, by name, whose precondition must hold before this one acts (`expired`).
    needs: tuple[str, ...] = ()
    #: Whether its procedures can ask for Go; whether Auto and Preview show; whether Cancel shows (in any step while a
    #: procedure runs).
    goes = False
    auto = False
    cancels = False
    #: What the phone page says is done on the laptop; "" for nothing.
    on_laptop = "This step's controls are on the laptop."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Add the step's own widgets; they sit in its folder, shown while it is the current step."""

    def build_phone(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Add the step's widgets on the phone page, for what is done next to the robot; here only `on_laptop`."""
        if self.on_laptop:
            gui.add_html(note(self.on_laptop))

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Judge the step's result now."""
        return []

    def state(self, ctx: PluginContext) -> Check:
        """The step's state for the overview, e.g. Check("passes", GOOD); unfinished (BAD or PENDING) asks at Done.

        The overview shows its label and short `value`; its `detail` says why on hover.
        """
        red = red_checks(self.checks(ctx))
        more = f" (+{len(red) - 1} more: see the step's chips)" if len(red) > 1 else ""
        return (Check("fails", BAD, f"{red[0].label}: {red[0].detail}{more}", in_short(red)) if red else
                Check("passes", GOOD))

    def question(self, ctx: PluginContext) -> str:
        """A question for the operator, shown below the controls on both pages; "" for none."""
        return ""

    def expired(self, ctx: PluginContext) -> str | None:
        """Why this step's precondition has expired for the steps that `need` it; None while it holds or it has none."""
        return None

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Judge whether the step's actions can run now; shown while one is not green."""
        return []

    def refusal(self, ctx: PluginContext) -> str | None:
        """Why the step's actions cannot run now: its red `ready` checks; None if they can."""
        return "; ".join(f"{check.label}: {check.detail}" for check in red_checks(self.ready(ctx))) or None

    def enter(self, ctx: PluginContext) -> None:
        """The step became the current one; ! it changes no data."""

    def leave(self, ctx: PluginContext) -> None:
        """Another step becomes the current one: hide what only this step draws."""

    def draw(self, ctx: PluginContext) -> None:
        """Copy state into the step's widgets and scene nodes; only while it is the current step."""
