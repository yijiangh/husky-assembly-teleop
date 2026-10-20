"""Widget callbacks: queued for the main thread with the value at the click; with operator_only, an operator's only."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

from husky_assembly_teleop.plugin_api.context import PluginContext


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
    ctx, seen = PluginContext("p", SimpleNamespace(), view=None, scene=None, kinematics=None), []
    widget = SimpleNamespace(value=1.0)
    callback = ctx.defer_value("slider", seen.append)
    asyncio.run(callback(SimpleNamespace(client=object(), target=widget)))
    widget.value = 2.0
    assert _drain(ctx) == ["slider"] and seen == [1.0]


def test_operator_only_skips_the_plugins_own_writes():
    """A value the plugin writes calls back too, with no client: operator_only drops those."""
    ctx, seen = PluginContext("p", SimpleNamespace(), view=None, scene=None, kinematics=None), []
    widget = SimpleNamespace(value=3.0)
    callback = ctx.defer_value("slider", seen.append, operator_only=True)
    asyncio.run(callback(SimpleNamespace(client=None, target=widget)))
    asyncio.run(callback(SimpleNamespace(client=object(), target=widget)))
    assert _drain(ctx) == ["slider"] and seen == [3.0]
    touched = []
    asyncio.run(ctx.defer("touch", lambda: touched.append(True), operator_only=True)(
        SimpleNamespace(client=None, target=widget)))
    assert _drain(ctx) == [] and not touched
