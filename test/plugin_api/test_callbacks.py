"""Widget callbacks: queued for the main thread with the value at the click; with operator_only, an operator's only."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

from husky_assembly_teleop.plugin_api.context import PluginContext

#: A monitor that only logs, into LOGGED, as (message, source).
LOGGED: list = []
MONITOR = SimpleNamespace(log_info=lambda message, source: LOGGED.append((message, source)))


def _drain(ctx: PluginContext) -> list:
    """Run the queued intents, as the monitor does, and return their labels."""
    labels = []
    while not ctx._intents.empty():
        intent = ctx._intents.get_nowait()
        labels.append(intent.label)
        intent.run()
    return labels


def test_a_callback_hands_over_the_value_at_the_click():
    """The value travels with the event; a later change does not reach the queued intent."""
    ctx, seen = PluginContext("p", MONITOR, view=None, scene=None, kinematics=None), []
    widget = SimpleNamespace(value=1.0)
    callback = ctx.defer_value("slider", seen.append)
    asyncio.run(callback(SimpleNamespace(client=object(), target=widget)))
    widget.value = 2.0
    assert _drain(ctx) == ["slider"] and seen == [1.0]


def test_operator_only_skips_the_plugins_own_writes():
    """A value the plugin writes calls back too, with no client: operator_only drops those."""
    ctx, seen = PluginContext("p", MONITOR, view=None, scene=None, kinematics=None), []
    widget = SimpleNamespace(value=3.0)
    callback = ctx.defer_value("slider", seen.append, operator_only=True)
    asyncio.run(callback(SimpleNamespace(client=None, target=widget)))
    asyncio.run(callback(SimpleNamespace(client=object(), target=widget)))
    assert _drain(ctx) == ["slider"] and seen == [3.0]
    touched = []
    asyncio.run(ctx.defer("touch", lambda: touched.append(True), operator_only=True)(
        SimpleNamespace(client=None, target=widget)))
    assert _drain(ctx) == [] and not touched


def test_operator_actions_are_logged_clicks_before_they_run_values_once_settled():
    """Clicks log before they run, in order; a dragged slider logs its last value once settled; the plugin's not.

    A button group's button counts as a click, its label logged with it.
    """
    ctx = PluginContext("p", MONITOR, view=None, scene=None, kinematics=None)
    clock, lines = [10.0], []
    ctx.now, ctx.log_info = lambda: clock[0], lines.append
    widget = SimpleNamespace(value=0.0)

    class GuiButtonGroupHandle:
        """viser's button group, as far as the context looks: its name, and the button clicked as its value."""

        def __init__(self, value: str):
            self.value = value

    class GuiButtonHandle:
        """viser's button, as far as the context looks: its name."""

        value = False

    slider = ctx.defer_value("Motion time", lambda _value: lines.append("ran slider"))
    asyncio.run(ctx.defer("Exec", lambda: lines.append("ran Exec"))(
        SimpleNamespace(client=object(), target=GuiButtonHandle())))
    for name in ("Mark", "Mark"):
        asyncio.run(ctx.defer_value("probe", lambda _value: None)(
            SimpleNamespace(client=object(), target=GuiButtonGroupHandle(name))))
        ctx._drain_intents()
        clock[0] += 0.3  # * two clicks, not a held button
    for value in (1.0, 2.0, 3.0):
        widget.value = value
        asyncio.run(slider(SimpleNamespace(client=object(), target=widget)))
    asyncio.run(slider(SimpleNamespace(client=None, target=widget)))
    ctx._drain_intents()
    assert lines[:4] == ["operator: Exec", "ran Exec", "operator: probe: 'Mark'", "operator: probe: 'Mark'"]
    assert "operator: Motion time = 3.0" not in lines
    clock[0] += 0.6
    ctx._drain_intents()
    assert lines[-1] == "operator: Motion time = 3.0"


def test_a_slider_wired_with_defer_logs_its_settled_value_not_each_step():
    """`defer` on a slider (the arm planner's joints) is a setting: one line with the last value once settled."""
    ctx = PluginContext("p", MONITOR, view=None, scene=None, kinematics=None)
    clock, lines = [10.0], []
    ctx.now, ctx.log_info = lambda: clock[0], lines.append
    widget = SimpleNamespace(value=0.0)
    target = ctx.defer("target", lambda: None, operator_only=True)
    for value in (0.1, 0.2, 0.3):
        widget.value = value
        asyncio.run(target(SimpleNamespace(client=object(), target=widget)))
    ctx._drain_intents()
    assert lines == []
    clock[0] += 0.6
    ctx._drain_intents()
    assert lines == ["operator: target = 0.3"]


def test_a_held_button_logs_once():
    """`on_hold` calls back at 20 Hz: one line per hold; a click after a pause logs again."""
    ctx = PluginContext("p", MONITOR, view=None, scene=None, kinematics=None)
    clock, lines = [10.0], []
    ctx.now, ctx.log_info = lambda: clock[0], lines.append

    class GuiButtonHandle:
        """viser's button, as far as the context looks: its name."""

        value = False

    hold = ctx.defer("hold forward", lambda: None)
    for _ in range(10):
        asyncio.run(hold(SimpleNamespace(client=object(), target=GuiButtonHandle())))
        ctx._drain_intents()
        clock[0] += 0.05
    clock[0] += 1.0
    asyncio.run(hold(SimpleNamespace(client=object(), target=GuiButtonHandle())))
    ctx._drain_intents()
    assert lines == ["operator: hold forward", "operator: hold forward"]
