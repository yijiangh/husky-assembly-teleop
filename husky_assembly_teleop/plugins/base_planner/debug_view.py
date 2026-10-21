"""
Draws a search's `Debug` record: its layers on the floor, a legend and a checkbox per layer, and its notes in the panel.

! Main thread only, like every scene call.
"""

from __future__ import annotations

from html import escape

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...ui.base_markers import FLOOR
from ...ui.style import NONE, block
from .algorithms import Debug

#: Height between layers, metres, so later layers draw over earlier ones.
LAYER_STEP = 0.002
LINE = 0.006         # m
POINT = 0.035        # m


class DebugView:
    """The floor drawing and panel legend of the last search's debug record, rebuilt when the record changes.

    Layers switched off stay off for later records with a layer of the same name.
    """

    def __init__(self, ctx: PluginContext, legend: viser.GuiHtmlHandle, layers: viser.GuiFolderHandle | None = None,
                 notes: viser.GuiHtmlHandle | None = None):
        """Draw nothing yet.

        Args:
            ctx: The plugin's context.
            legend: Panel html for the legend, and the notes unless `notes` is given.
            layers: Panel folder for a checkbox per layer; None for no checkboxes.
            notes: Panel html for the notes, below the checkboxes.
        """
        self._ctx = ctx
        self._legend = legend
        self._layers = layers
        self._notes = notes
        # Scene nodes by layer name, and the panel widgets of the checkboxes.
        self._nodes: dict[str, list] = {}
        self._widgets: list = []
        self._checkboxes: dict[str, viser.GuiCheckboxHandle] = {}
        #: Names of the layers switched off.
        self._hidden: set[str] = set()
        self._shown: Debug | None = None
        if layers is not None:
            layers.visible = False

    def show(self, debug: Debug | None) -> None:
        """Show a record; None, or one recorded with debug off, clears the view."""
        if debug is self._shown:
            return
        self._shown = debug
        for node in (node for nodes in self._nodes.values() for node in nodes):
            node.remove()
        for widget in self._widgets:
            widget.remove()
        self._nodes, self._widgets, self._checkboxes = {}, [], {}
        if debug is None or not debug.on:
            self._legend.content = ""
            if self._notes is not None:
                self._notes.content = ""
            if self._layers is not None:
                self._layers.visible = False
            return
        scene, root = self._ctx.view.scene, f"{self._ctx.view.scene_root}/debug"
        rows = []
        for i, (name, layer) in enumerate(debug.layers.items()):
            height = FLOOR + (i + 2) * LAYER_STEP
            nodes = self._nodes.setdefault(name, [])
            if layer.segments:
                segments = np.asarray(layer.segments)
                segments = np.concatenate([segments, np.full((*segments.shape[:2], 1), height)], axis=2)
                nodes.append(scene.add_line_segments(f"{root}/{i}_lines", segments, layer.color, thickness=LINE))
            if layer.points:
                points = np.column_stack([np.asarray(layer.points), np.full(len(layer.points), height)])
                nodes.append(scene.add_point_cloud(f"{root}/{i}_points", points, layer.color, point_size=POINT,
                                                   point_shape="circle", precision="float32"))
            for node in nodes:
                node.visible = name not in self._hidden
            count = len(layer.segments) + len(layer.points)
            rows.append(f'<div><span style="color:rgb{layer.color}">■</span> {escape(name)} '
                        f'<span style="color:{NONE}">{count}</span></div>')
        notes = "".join(f'<div style="color:{NONE}">{escape(text)}</div>' for text in debug.notes)
        style = '<div style="font-size:11px;line-height:1.5">{}</div>'
        if self._notes is None:
            self._legend.content = block(style.format("".join(rows) + notes))
        else:
            self._legend.content = block(style.format("".join(rows)))
            self._notes.content = block(style.format(notes)) if notes else ""
        if self._layers is not None:
            self._add_checkboxes(debug)

    def _add_checkboxes(self, debug: Debug) -> None:
        """A checkbox per layer in the layers folder, and buttons to show all or none."""
        gui = self._ctx.view.gui
        with self._layers:
            every = gui.add_button_group("Show", ["All", "None"], hint="Show every debug layer, or none")
            every.on_click(self._ctx.defer_value("debug layers", lambda label: self._show_all(label == "All")))
            self._widgets.append(every)
            for name, layer in debug.layers.items():
                count = len(layer.segments) + len(layer.points)
                checkbox = gui.add_checkbox(f"{name} ({count})", initial_value=name not in self._hidden,
                                            hint="Draw this layer on the floor; it stays off for later searches")
                checkbox.on_update(self._ctx.defer_value(
                    "debug layer", lambda value, name=name: self._show_layer(name, bool(value))))
                self._checkboxes[name] = checkbox
                self._widgets.append(checkbox)
        self._layers.visible = True

    def _show_layer(self, name: str, on: bool) -> None:
        """Show or hide one layer, and remember it for later records."""
        if on:
            self._hidden.discard(name)
        else:
            self._hidden.add(name)
        for node in self._nodes.get(name, []):
            node.visible = on
        checkbox = self._checkboxes.get(name)
        if checkbox is not None and checkbox.value != on:
            checkbox.value = on

    def _show_all(self, on: bool) -> None:
        """Show or hide every layer of the record shown."""
        for name in list(self._nodes):
            self._show_layer(name, on)
