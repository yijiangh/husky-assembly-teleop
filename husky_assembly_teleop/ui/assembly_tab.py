"""
The side panel's Assembly tab, shared by process, assembly_steps, movement_exec and assembly_run: bands and icons.

The plot windows of assembly_steps and movement_exec float at the bottom left; where is set here too.
Each of them sets `tab_path = (ASSEMBLY_TAB,)` and adds its widgets in its own bands, `with ctx.view.ui(band=STEP_BAND)
as gui:`; the bands sort top to bottom by these numbers, whichever plugin starts first.

! The tab's CSS finds buttons by their icon (`ui/phone.py`): each icon here is on no other desktop button.
"""

from __future__ import annotations

import viser

from .phone import big_buttons

#: The tab's label, the plugins' `tab_path`.
ASSEMBLY_TAB = "Assembly"
#: The tab's icon: the icon of process, which makes the tab, since the other three require it.
ASSEMBLY_ICON = viser.Icon.LIST_CHECK

# * Bands, top to bottom.
#: process: the process (folder, Load, its boxes), its actions with their marks, the movement slider, its details.
PROCESS_BAND = 50.0
#: assembly_steps: status, step and Exec lines, Live IK, Plan, Play / Clear, the preview and the motion time.
STEP_BAND = 100.0
#: assembly_steps: Exec, and its message line.
EXEC_BAND = 200.0
#: movement_exec: the prompt, Confirm | Cancel, the last result.
PROMPT_BAND = 300.0
#: assembly_run: Mark done | Reopen, and its message line.
MARK_BAND = 400.0
#: assembly_steps: the "settings" bar and Ignore built bars.
SETTINGS_BAND = 500.0
#: assembly_run: Insert split mm, Spiral, Servo by bar.
RUN_SETTINGS_BAND = 510.0
#: movement_exec: Dry run.
EXEC_SETTINGS_BAND = 520.0
#: assembly_run: the run's chips, Continue newest / Continue match / New run, where the robots are.
RUN_BAND = 600.0
#: assembly_steps: the Advanced planning folder, closed by default; last, below every button, so opening it moves none.
PLAN_SETTINGS_BAND = 650.0

# * The plot windows float at the bottom left (the console is at the bottom right), minimized to their bars until
#   there is something to plot: assembly_steps' Plan plots, then movement_exec's Exec plots to their right.
#: Each window's size, px.
PLOTS_WIDTH, PLOTS_HEIGHT = 460, 560
#: Where each window sits: x from the page's left edge, y from its bottom (negative, as viser's `float` counts it).
PLAN_PLOTS_X, EXEC_PLOTS_X, PLOTS_Y = 15, 15 + PLOTS_WIDTH + 10, -15

# * Button icons, so the CSS and the buttons agree.
#: Live IK.
GOAL_ICON = viser.Icon.FOCUS_2
#: Check tool0: the live tool0 against the step's authored target.
CHECK_ICON = viser.Icon.RULER_MEASURE
#: Plan.
PLAN_ICON = viser.Icon.ROUTE_2
#: Exec.
EXEC_ICON = viser.Icon.PLAYER_PLAY_FILLED
#: Mark done, and Reopen beside it.
MARK_ICON, REOPEN_ICON = viser.Icon.CIRCLE_CHECK, viser.Icon.ARROW_BACK_UP
#: Confirm and Cancel, here and on the phone's Exec tab.
CONFIRM_ICON, CANCEL_ICON = viser.Icon.CHECK, viser.Icon.X

#: CSS that makes the tab's buttons big, for the whole desktop page; assembly_run adds it, since assembly_steps and
#: movement_exec load it.
TAB_BUTTONS_CSS = big_buttons([GOAL_ICON, CHECK_ICON, PLAN_ICON, EXEC_ICON, MARK_ICON, REOPEN_ICON, CONFIRM_ICON,
                               CANCEL_ICON], 40,
                              18, scope="")
