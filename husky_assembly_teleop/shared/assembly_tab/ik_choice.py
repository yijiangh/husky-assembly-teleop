"""
A support approach that fails: the pop-up offering the next IK solution, a moved grasp, or giving up.

Also the rule for a plan to an IK solution past the first: slower, and it warns. Used by assembly_steps (the Assembly
tab) and assembly_guide. ! Main thread only; connect the buttons with `ctx.defer`.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING

import viser

from ...colors import rgb
from ...ui.popup import Popup
from ...ui.style import BUSY, block
from ..assembly_planning.movement import Routine

if TYPE_CHECKING:
    from ...ui.visualization import PluginView

NEXT_SOLUTION, MOVE_GRASP, GIVE_UP = "Next IK solution", "Move the grasp", "Give up"
#: Where the operator moves a hold's grasp.
GRASP_HINT = ("Move the grasp: Assembly › process › grasp: Slide mm along the bar, Roll ° about it (the grape "
              "gripper shows where), Apply, then plan again; IK starts again at solution 1")
#: How `solve_ik` says every solution was passed over: Next has nothing left to try.
NO_MORE = "no more solutions"
#: A plan to any IK solution but the first takes at least this many times its routine's default motion time.
FALLBACK_SLOWDOWN = 2.0


def offers_choice(routine: Routine | None, why: str) -> bool:
    """Whether a failure gets the pop-up: a support approach's goal or path, not a cancel."""
    return routine is Routine.SUPPORT_APPROACH and "cancelled" not in why


def fallback_time(traj_time: float, default: float) -> float:
    """The motion time of a plan to an IK solution past the first: at least FALLBACK_SLOWDOWN × the routine's default.

    Args:
        traj_time: The motion time set, seconds.
        default: The routine's default motion time (`DEFAULT_TRAJ_TIME`), seconds.
    """
    return max(traj_time, FALLBACK_SLOWDOWN * default)


def fallback_warning(solution: int, seconds: float) -> str:
    """The warning of a plan to IK solution `solution` (from 1, here at least 2), timed at `seconds`."""
    return (f"The goal is IK solution {solution}, not the one nearest the process's joints: the arm ends in another "
            f"configuration than the process's, timed slower ({seconds:g} s). Play the preview carefully.")


class IkChoice:
    """The pop-up: Next IK solution, Move the grasp, Give up; what failed below them.

    Attributes:
        buttons: The three buttons by label, for the caller to connect.
    """

    def __init__(self, view: PluginView):
        """Make the pop-up, hidden."""
        self._popup = Popup(view, "Approach: no path", viser.Icon.ALERT_TRIANGLE,
                            {NEXT_SOLUTION: rgb(BUSY), MOVE_GRASP: None, GIVE_UP: None})
        self.buttons = self._popup.buttons

    @property
    def is_open(self) -> bool:
        """bool: Whether the pop-up shows."""
        return self._popup.is_open

    def offer(self, label: str, solution: int, why: str) -> None:
        """Show what failed at IK solution `solution` (from 1); Next greyed once no solution is left."""
        more = NO_MORE not in why
        self.buttons[NEXT_SOLUTION].disabled = not more
        nearest = " (the one nearest the process's joints)" if solution == 1 else ""
        then = (f"{NEXT_SOLUTION}: plans to solution {solution + 1}, slower, and asks you to check the preview."
                if more else f"No IK solution is left to try: {NEXT_SOLUTION} is greyed.")
        self._popup.show(block(f"<b>{escape(label)}</b>: IK solution {solution}{nearest} failed: {escape(why)}<br>"
                               f"{escape(then)}<br>{escape(GRASP_HINT)}.<br>{GIVE_UP}: closes this."))

    def close(self) -> None:
        """Hide the pop-up."""
        if self._popup.is_open:
            self._popup.close()
