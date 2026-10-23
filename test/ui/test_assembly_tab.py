"""The Assembly tab's bands and button icons, shared by four plugins."""

from __future__ import annotations

from husky_assembly_teleop.ui.assembly_tab import (ASSEMBLY_ICON, CANCEL_ICON, CONFIRM_ICON, EXEC_BAND, EXEC_ICON,
                                                   EXEC_SETTINGS_BAND, GOAL_ICON, MARK_BAND, MARK_ICON, PLAN_ICON,
                                                   PLOTS_BAND, PROCESS_BAND, PROMPT_BAND, REOPEN_ICON, RUN_BAND,
                                                   RUN_SETTINGS_BAND, SETTINGS_BAND, STEP_BAND)


def test_bands_run_top_to_bottom_and_icons_are_distinct():
    """The bands rise in the order the tab shows them; each button icon is used once, so the CSS finds one button."""
    bands = (PROCESS_BAND, STEP_BAND, EXEC_BAND, PROMPT_BAND, MARK_BAND, SETTINGS_BAND, RUN_SETTINGS_BAND,
             EXEC_SETTINGS_BAND, RUN_BAND, PLOTS_BAND)
    assert list(bands) == sorted(set(bands))
    icons = (GOAL_ICON, PLAN_ICON, EXEC_ICON, MARK_ICON, REOPEN_ICON, CONFIRM_ICON, CANCEL_ICON)
    assert len(set(icons)) == len(icons) and ASSEMBLY_ICON not in icons
