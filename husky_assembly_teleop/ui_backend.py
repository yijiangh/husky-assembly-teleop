"""Feature-flagged UI backend for the husky monitor.

Provides PyBulletBackend (legacy `addUserDebugParameter` shim) and
DearPyGuiBackend (real widgets). `make_backend(use_dpg, ...)` selects.
"""
from __future__ import annotations

import logging
import os
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
import pybullet as p

logger = logging.getLogger(__name__)

# Distinct line colors (RGB 0-255) for multi-series live plots. Reused for both
# the plot lines and the matching color chips in the readout table. tab20-style
# so up to 12 joints (dual-arm) stay visually separable. Deliberately no red:
# red is reserved for the out-of-range alert on the readout text.
_MULTI_PLOT_PALETTE = [
    (31, 119, 180),    # blue
    (255, 127, 14),    # orange
    (44, 160, 44),     # green
    (148, 103, 189),   # purple
    (140, 86, 75),     # brown
    (227, 119, 194),   # pink
    (127, 127, 127),   # gray
    (188, 189, 34),    # olive
    (23, 190, 207),    # cyan
    (255, 187, 120),   # light orange
    (152, 223, 138),   # light green
    (174, 199, 232),   # light blue
]

_RAD_TO_DEG = 180.0 / np.pi

# Readout text colors and the joint-limit past which a joint reads as alarming.
_READOUT_TEXT_COLOR = (210, 210, 210, 255)   # normal joint readout text
_READOUT_ALERT_COLOR = (255, 60, 60, 255)    # bold red when out of range
_JOINT_LIMIT_DEG = 345.0                      # |deg| beyond this -> alert

# Bold TTFs (first that exists wins) used to embolden out-of-range readout text.
_BOLD_FONT_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
    "C:/Windows/Fonts/segoeuib.ttf",
)


def _nonzero_range(vmin: float, vmax: float) -> Tuple[float, float]:
    """pybullet's ``addUserDebugParameter`` segfaults (GUI thread div-by-zero)
    when ``rangeMin == rangeMax``. Widen a degenerate range slightly so a
    1-element selection slider is just pinned instead of crashing."""
    vmin, vmax = float(vmin), float(vmax)
    if vmax <= vmin:
        vmax = vmin + 1e-6
    return vmin, vmax


class UIBackend:
    """Abstract UI backend; do NOT instantiate. See PyBulletBackend / DearPyGuiBackend."""

    def add_button(self, label: str, on_click: Callable[[], None]) -> int:
        raise NotImplementedError

    def add_slider_float(self, label: str, vmin: float, vmax: float,
                         default: float, on_change: Callable[[float], None]) -> int:
        raise NotImplementedError

    def add_slider_int(self, label: str, vmin: int, vmax: int,
                       default: int, on_change: Callable[[int], None]) -> int:
        raise NotImplementedError

    def add_slider_group(self, labels: Sequence[str], vmins: Sequence[float],
                         vmaxs: Sequence[float], defaults: Sequence[float],
                         on_change: Callable[[List[float]], None]) -> List[int]:
        raise NotImplementedError

    def add_checkbox(self, label: str, default: bool,
                     on_change: Callable[[bool], None]) -> int:
        raise NotImplementedError

    def add_combo(self, label: str, options: List[str], default_idx: int,
                  on_change: Callable[[int], None]) -> int:
        raise NotImplementedError

    def add_text_input(self, label: str, default: str,
                       on_change: Callable[[Any], None], *,
                       numeric: bool = False, fmt: str = "%.4f") -> int:
        raise NotImplementedError

    def add_file_dialog(self, label: str, on_select: Callable[[str], None], *,
                        base_dir: Optional[str] = None,
                        ext_filter: Optional[str] = None) -> int:
        raise NotImplementedError

    def add_live_plot(self, label: str, source: Callable[[], float],
                      history: int = 200) -> int:
        raise NotImplementedError

    def add_live_multi_plot(self, label: str, source: Callable[[], List[float]],
                            series_labels: Sequence[str], history: int = 200,
                            header_source: Optional[Callable[[], str]] = None, *,
                            parent: Optional[Any] = None,
                            group_size: Optional[int] = None,
                            palette: Optional[Sequence[Any]] = None) -> int:
        raise NotImplementedError

    def add_window(self, label: str, *, tag: str, width: int = 600,
                   height: int = 800, show: bool = True) -> str:
        """Create a floating top-level window (not the primary panel)."""
        raise NotImplementedError

    def set_visible(self, handle: int, visible: bool) -> None:
        """Show or hide a previously-added widget/section by its handle."""
        raise NotImplementedError

    def add_separator(self, label: str) -> int:
        raise NotImplementedError

    def begin_group(self, label: str, *, collapsible: bool = True) -> None:
        raise NotImplementedError

    def end_group(self) -> None:
        raise NotImplementedError

    def poll(self, handle: int, kind: str,
             on_change: Optional[Callable[..., None]] = None) -> None:
        raise NotImplementedError

    def get_value(self, handle: int):
        """Return a widget's current value by handle, or None if unavailable.

        Lets callers read where a slider actually sits rather than relying on
        its on-change callback having fired (callbacks on widgets rebuilt by
        reset_ui can be missed).
        """
        return None

    def set_value(self, handle: int, value: Any) -> None:
        """Write a value into an already-created widget (e.g. blank an entry box).

        Does nothing on backends whose widgets cannot be written to.
        """
        return

    def clear(self) -> None:
        """Remove all widgets created so far so the UI can be rebuilt without
        duplicates (used by HuskyMonitor.reset_ui)."""
        raise NotImplementedError

    def step(self) -> bool:
        raise NotImplementedError

    def shutdown(self) -> None:
        raise NotImplementedError


class PyBulletBackend(UIBackend):
    """Legacy backend using `p.addUserDebugParameter`. Preserves byte-for-byte behavior."""

    def __init__(self) -> None:
        self._next_handle = 1
        self._handles: Dict[int, Dict[str, Any]] = {}
        self._warned_legacy_widgets: set = set()

    def _new_handle(self) -> int:
        h = self._next_handle
        self._next_handle += 1
        return h

    def _warn_once(self, key: str, msg: str) -> None:
        if key not in self._warned_legacy_widgets:
            self._warned_legacy_widgets.add(key)
            logger.warning(msg)

    def add_button(self, label, on_click):
        dbg = p.addUserDebugParameter(label, 1.0, 0.0, 0.0)
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        self._handles[h] = {"kind": "button", "dbg": dbg, "prev": prev, "cb": on_click}
        return h

    def add_slider_float(self, label, vmin, vmax, default, on_change):
        vmin, vmax = _nonzero_range(vmin, vmax)
        dbg = p.addUserDebugParameter(label, vmin, vmax, default)
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_float", "dbg": dbg, "prev": prev, "cb": on_change}
        return h

    def add_slider_int(self, label, vmin, vmax, default, on_change):
        vmin, vmax = _nonzero_range(vmin, vmax)
        dbg = p.addUserDebugParameter(label, vmin, vmax, float(default))
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        wrapped = lambda v, _cb=on_change: _cb(int(round(v)))
        self._handles[h] = {"kind": "slider_int", "dbg": dbg, "prev": prev, "cb": wrapped}
        return h

    def add_slider_group(self, labels, vmins, vmaxs, defaults, on_change):
        ranges = [_nonzero_range(vmn, vmx) for vmn, vmx in zip(vmins, vmaxs)]
        dbgs = [p.addUserDebugParameter(lbl, lo, hi, dv)
                for lbl, (lo, hi), dv in zip(labels, ranges, defaults)]
        prevs = [p.readUserDebugParameter(d) for d in dbgs]
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_group", "dbgs": dbgs, "prev": prevs, "cb": on_change}
        # legacy returns a list of N handles; here we return [h]*N to keep callers happy
        return [h] * len(dbgs)

    def add_checkbox(self, label, default, on_change):
        self._warn_once("checkbox",
                        "checkbox in legacy mode degraded to 0..1 slider; consider USE_DPG_UI=1")
        dbg = p.addUserDebugParameter(label, 0.0, 1.0, 1.0 if default else 0.0)
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        wrapped = lambda v, _cb=on_change: _cb(bool(round(v)))
        self._handles[h] = {"kind": "checkbox", "dbg": dbg, "prev": prev, "cb": wrapped}
        return h

    def add_combo(self, label, options, default_idx, on_change):
        self._warn_once("combo",
                        "combo in legacy mode degraded to int slider; consider USE_DPG_UI=1")
        lo, hi = _nonzero_range(0.0, max(len(options) - 1, 0))
        dbg = p.addUserDebugParameter(label, lo, hi, float(default_idx))
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        wrapped = lambda v, _cb=on_change: _cb(int(round(v)))
        self._handles[h] = {"kind": "combo", "dbg": dbg, "prev": prev, "cb": wrapped}
        return h

    def add_text_input(self, label, default, on_change, *, numeric=False, fmt="%.4f"):
        raise NotImplementedError(
            "text_input widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_file_dialog(self, label, on_select, *, base_dir=None, ext_filter=None):
        raise NotImplementedError(
            "file_dialog widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_live_plot(self, label, source, history=200):
        raise NotImplementedError(
            "live_plot widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_live_multi_plot(self, label, source, series_labels, history=200,
                            header_source=None, *, parent=None, group_size=None,
                            palette=None):
        raise NotImplementedError(
            "live_multi_plot widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_window(self, label, *, tag, width=600, height=800, show=True):
        raise NotImplementedError(
            "separate windows not supported by PyBulletBackend; set USE_DPG_UI=1")

    def set_visible(self, handle, visible):
        # PyBullet debug params can't be individually shown/hidden; nothing to do.
        return

    def add_separator(self, label):
        # PyBullet draws no divider line, so wrap the label in dashes to mark it
        # as a section header (e.g. "---CONTROLLERS---"). DPG keeps the plain label
        # because it draws an actual separator line above the text.
        dbg = p.addUserDebugParameter(f"---{label}---", 0.0, 1.0, 0.0)
        h = self._new_handle()
        self._handles[h] = {"kind": "separator", "dbg": dbg}
        return h

    def begin_group(self, label, *, collapsible=True):
        # legacy: just a decorative separator
        self.add_separator(label)

    def end_group(self):
        # legacy: nothing to do
        pass

    def poll(self, handle, kind, on_change=None):
        rec = self._handles.get(handle)
        if rec is None:
            return
        k = rec["kind"]
        if k == "separator":
            return
        cb = on_change if on_change is not None else rec.get("cb")
        if cb is None:
            return
        if k == "slider_group":
            new_vals = [p.readUserDebugParameter(d) for d in rec["dbgs"]]
            if not np.allclose(new_vals, rec["prev"]):
                rec["prev"] = new_vals
                cb(new_vals)
            return
        new_val = p.readUserDebugParameter(rec["dbg"])
        if new_val != rec["prev"]:
            rec["prev"] = new_val
            if k == "button":
                cb()
            else:
                cb(new_val)

    def get_value(self, handle):
        rec = self._handles.get(handle)
        if rec is None or "dbg" not in rec:
            return None
        return p.readUserDebugParameter(rec["dbg"])

    def set_value(self, handle, value):
        # PyBullet has no setUserDebugParameter(), so a debug slider can only be
        # rewritten by destroying and re-adding it (that is what reset_ui does).
        return

    def clear(self) -> None:
        p.removeAllUserParameters()
        self._handles.clear()

    def step(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass


_DEFAULT_FONT_PATHS = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/System/Library/Fonts/Helvetica.ttc",
    "C:/Windows/Fonts/segoeui.ttf",
)


def bind_default_font(dpg, font_size: int):
    """Bind a TTF font at `font_size` to the current DPG context, falling
    back to scaling the built-in bitmap font. Safe to call once per DPG
    context (i.e. once per create_context()).

    Returns:
        The bound font handle, or None if no TTF was available (bitmap fallback).
    """
    for path in _DEFAULT_FONT_PATHS:
        if not os.path.exists(path):
            continue
        try:
            with dpg.font_registry():
                f = dpg.add_font(path, font_size)
            dpg.bind_font(f)
            return f
        except Exception as e:
            logger.debug(f"font load failed {path}: {e}")
    dpg.set_global_font_scale(max(1.0, font_size / 13.0))
    return None


class DearPyGuiBackend(UIBackend):
    """Real DPG-based backend with full widget support."""

    _DEFAULT_FONT_PATHS = _DEFAULT_FONT_PATHS  # back-compat alias

    def __init__(self, window_title: str = "Husky Monitor",
                 width: int = 600, height: int = 1000,
                 font_size: int = 18) -> None:
        import dearpygui.dearpygui as dpg  # lazy import
        self.dpg = dpg

        dpg.create_context()
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):          # applies to every widget type
                # hover colors (RGBA 0-255)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  (90, 130, 200, 100))
                dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered,  (90, 130, 200, 100))  # selectable / tree / combo items
                dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (70, 90, 130, 100))   # slider / input / checkbox bg
                dpg.add_theme_color(dpg.mvThemeCol_TabHovered,     (90, 130, 200, 100))
        dpg.bind_theme(global_theme)

        dpg.create_viewport(title=window_title, width=width, height=height)
        # Global default font (size = font_size = UI_FONT_SIZE = 16) for all widgets.
        # Keep the handle so we can rebind it to un-bold a readout row.
        self._font_default = self._bind_default_font(font_size)
        # Separators render larger (20); font built here, bound per-item in add_separator.
        self._build_sep_font()
        # Bold font (same size) for emphasising out-of-range joint readouts.
        self._build_bold_font(font_size)
        dpg.setup_dearpygui()

        with dpg.window(tag="root", label=window_title,
                        width=width, height=height, no_close=True):
            pass
        dpg.set_primary_window("root", True)

        self._parent_stack: List[Any] = ["root"]
        self._next_handle = 1
        self._handles: Dict[int, Dict[str, Any]] = {}
        self._live_plots: List[Dict[str, Any]] = []
        self._live_multi: List[Dict[str, Any]] = []
        # Push-based per-iteration plots (add_history_plot). The caller appends
        # samples via history_push(); rendering (set_value + axis fit) happens in
        # step(), the same render-pass path the live plots use, so the axes fit
        # correctly and new points show up.
        self._history_plots: List[Dict[str, Any]] = []
        self._frame_idx = 0

        dpg.show_viewport()

    def _new_handle(self) -> int:
        h = self._next_handle
        self._next_handle += 1
        return h

    def _bind_default_font(self, font_size: int):
        return bind_default_font(self.dpg, font_size)

    def _build_bold_font(self, font_size: int) -> None:
        """Build a bold font at the readout size, bound per-item in step() when a
        joint goes out of range. self._font_bold is None if no bold TTF is found
        (then out-of-range readouts still turn red, just not bold)."""
        dpg = self.dpg
        self._font_bold = None
        path = next((p for p in _BOLD_FONT_PATHS if os.path.exists(p)), None)
        if not path:
            return
        try:
            with dpg.font_registry():
                self._font_bold = dpg.add_font(path, font_size)
        except Exception as e:
            logger.debug(f"bold font load failed {path}: {e}")

    def _build_sep_font(self) -> None:
        """Build a larger font (20 pt) for separator headers; bound per-item in
        add_separator via dpg.bind_item_font. self._font_sep is None if no TTF
        is available (then separators just use the default 16 pt)."""
        dpg = self.dpg
        self._font_sep = None  # default: no special font (separators stay 16 pt)
        # first font file that exists on this machine, or None if none do
        path = next((p for p in _DEFAULT_FONT_PATHS if os.path.exists(p)), None)
        if not path:
            return
        try:
            # DPG requires fonts to be created inside a font_registry context
            with dpg.font_registry():
                self._font_sep = dpg.add_font(path, 20)  # separator header size
        except Exception as e:
            logger.debug(f"separator font load failed {path}: {e}")

    @property
    def _current_parent(self):
        return self._parent_stack[-1]

    def add_button(self, label, on_click):
        dpg = self.dpg
        tag = dpg.add_button(label=label, parent=self._current_parent,
                             callback=lambda *a: on_click())
        h = self._new_handle()
        self._handles[h] = {"kind": "button", "tag": tag}
        return h

    def add_slider_float(self, label, vmin, vmax, default, on_change):
        dpg = self.dpg
        # Label ABOVE the slider: DPG draws the slider's own label inline on the
        # right, so emit the name as a text line first and give the slider an
        # empty label + full width (width=-1).
        # tag = dpg.add_slider_float(label=label, min_value=vmin, max_value=vmax,
        #                            default_value=default, parent=self._current_parent,
        #                            callback=lambda s, app_data, u: on_change(app_data))
        dpg.add_text(label, parent=self._current_parent)
        tag = dpg.add_slider_float(label="", min_value=vmin, max_value=vmax,
                                   default_value=default, parent=self._current_parent,
                                   width=-1, callback=lambda s, app_data, u: on_change(app_data))
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_float", "tag": tag}
        return h

    def add_slider_int(self, label, vmin, vmax, default, on_change):
        dpg = self.dpg
        # Label ABOVE the slider (see add_slider_float).
        # tag = dpg.add_slider_int(label=label, min_value=int(vmin), max_value=int(vmax),
        #                          default_value=int(default), parent=self._current_parent,
        #                          callback=lambda s, app_data, u: on_change(int(app_data)))
        dpg.add_text(label, parent=self._current_parent)
        tag = dpg.add_slider_int(label="", min_value=int(vmin), max_value=int(vmax),
                                 default_value=int(default), parent=self._current_parent,
                                 width=-1, callback=lambda s, app_data, u: on_change(int(app_data)))
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_int", "tag": tag}
        return h

    def add_slider_group(self, labels, vmins, vmaxs, defaults, on_change):
        # Symmetric with PyBulletBackend: one composite handle for the whole group.
        # The shim layer treats a SliderGroup as a single widget for poll dispatch.
        dpg = self.dpg
        tags: List[Any] = []

        def _fan_out(*_a):
            vals = [dpg.get_value(t) for t in tags]
            on_change(vals)

        for lbl, vmn, vmx, dv in zip(labels, vmins, vmaxs, defaults):
            # Label ABOVE each sub-slider (see add_slider_float).
            # t = dpg.add_slider_float(label=lbl, min_value=vmn, max_value=vmx,
            #                          default_value=dv, parent=self._current_parent,
            #                          callback=_fan_out)
            dpg.add_text(lbl, parent=self._current_parent)
            t = dpg.add_slider_float(label="", min_value=vmn, max_value=vmx,
                                     default_value=dv, parent=self._current_parent,
                                     width=-1, callback=_fan_out)
            tags.append(t)
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_group", "tags": tags}
        return [h] * len(tags)

    def add_checkbox(self, label, default, on_change):
        dpg = self.dpg
        tag = dpg.add_checkbox(label=label, default_value=bool(default),
                               parent=self._current_parent,
                               callback=lambda s, app_data, u: on_change(bool(app_data)))
        h = self._new_handle()
        self._handles[h] = {"kind": "checkbox", "tag": tag}
        return h

    def add_combo(self, label, options, default_idx, on_change):
        dpg = self.dpg
        opts = list(options)
        default_val = opts[default_idx] if opts and 0 <= default_idx < len(opts) else ""
        tag = dpg.add_combo(label=label, items=opts, default_value=default_val,
                            parent=self._current_parent,
                            callback=lambda s, app_data, u: on_change(
                                opts.index(app_data) if app_data in opts else 0))
        h = self._new_handle()
        self._handles[h] = {"kind": "combo", "tag": tag}
        return h

    def add_text_input(self, label, default, on_change, *, numeric=False, fmt="%.4f"):
        dpg = self.dpg
        if numeric:
            # step=0 hides the -/+ buttons: these boxes are typed into, and the
            # default 0.1 step is far too coarse for e.g. a metre offset.
            tag = dpg.add_input_float(
                label=label, default_value=float(default or 0),
                parent=self._current_parent, step=0.0, format=fmt,
                callback=lambda s, a, u: on_change(a))
        else:
            tag = dpg.add_input_text(
                label=label, default_value=default or "",
                parent=self._current_parent,
                callback=lambda s, a, u: on_change(a),
                on_enter=True)
        h = self._new_handle()
        self._handles[h] = {"kind": "text_input", "tag": tag}
        return h

    def add_file_dialog(self, label, on_select, *, base_dir=None, ext_filter=None):
        dpg = self.dpg

        def _wrapped(sender, app_data, user_data):
            path = app_data.get("file_path_name") if isinstance(app_data, dict) else None
            if path:
                on_select(path)

        fd_kwargs = dict(directory_selector=False, show=False,
                         callback=_wrapped, modal=True, width=600, height=400)
        if base_dir:
            fd_kwargs["default_path"] = base_dir
        fd_tag = dpg.add_file_dialog(**fd_kwargs)
        if ext_filter:
            dpg.add_file_extension(ext_filter, parent=fd_tag)
        else:
            dpg.add_file_extension(".*", parent=fd_tag)
        btn_tag = dpg.add_button(label=label, parent=self._current_parent,
                                 callback=lambda *a, _t=fd_tag: dpg.show_item(_t))
        h = self._new_handle()
        self._handles[h] = {"kind": "file_dialog", "tag": btn_tag, "fd_tag": fd_tag}
        return h

    def add_live_plot(self, label, source, history=200):
        dpg = self.dpg
        with dpg.plot(label=label, height=120, width=-1, parent=self._current_parent):
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="t")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=label)
            series_tag = dpg.add_line_series([], [], label=label, parent=y_axis)
        h = self._new_handle()
        self._live_plots.append({
            "handle": h,
            "source": source,
            "series_tag": series_tag,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "history": history,
            "x": deque(maxlen=history),
            "y": deque(maxlen=history),
        })
        self._handles[h] = {"kind": "live_plot", "tag": series_tag}
        return h

    def add_window(self, label, *, tag, width=600, height=800, show=True):
        """Create (or refresh) a floating top-level window, not the primary panel.

        build_ui() re-runs on every reset_ui(); because this is a top-level
        window (so it can float/move independently) it is NOT a child of the
        primary "root" panel and therefore survives clear() (which only wipes
        root's children). On a rebuild we would otherwise hit DPG's duplicate-tag
        error, so if the window already exists we delete only its children (the
        stale plot/table) and keep the frame -- this preserves wherever the user
        dragged or resized it. We deliberately do NOT set_primary_window on it.

        Args:
            label (str): Window title bar text.
            tag (str): Stable id used to find/refresh the window on rebuilds.
            width (int): Initial window width in pixels.
            height (int): Initial window height in pixels.
            show (bool): Whether the window starts visible.

        Returns:
            str: The window tag (usable as a parent for other add_* calls).
        """
        dpg = self.dpg
        if dpg.does_item_exist(tag):
            dpg.delete_item(tag, children_only=True)
            dpg.configure_item(tag, show=show)
        else:
            dpg.add_window(tag=tag, label=label, width=width, height=height,
                           show=show)
        return tag

    def add_live_multi_plot(self, label, source, series_labels, history=200,
                            header_source=None, *, parent=None, group_size=None,
                            palette=None):
        """Scrolling multi-series plot + a color-coded rad/deg readout table.

        The plot has a radians y-axis on the left and a passive degrees ruler on
        the right (kept in sync in step()). Its legend sits below the plot so it
        never covers data. Below the plot a table shows one [color chip | text]
        pair per series; series are split into `group_size` column-pairs (e.g.
        left vs right arm). Each series' line color is fixed via a theme so the
        chip can match it.

        Args:
            label (str): Plot title.
            source (callable): source() -> list[float] of length N, polled each step().
            series_labels (list): One legend/readout label per series (len N).
            history (int): Samples kept in the rolling window (oscilloscope width).
            header_source (callable): Optional () -> str shown above the table (robot name).
            parent: Container tag (e.g. a window from add_window) to hold the
                plot + table; defaults to the current parent. set_visible() toggles it.
            group_size (int): Series per column-group (e.g. 6 joints/arm); a
                dual-arm 12-series plot becomes two L | R pairs. Defaults to all-in-one.
            palette (list): RGB tuples per series; defaults to _MULTI_PLOT_PALETTE.

        Returns:
            int: Handle for set_visible().
        """
        dpg = self.dpg
        n = len(series_labels)
        group_size = group_size or n
        n_groups = max(1, n // group_size)
        palette = palette or _MULTI_PLOT_PALETTE
        colors = [tuple(palette[i % len(palette)]) for i in range(n)]
        container = parent if parent is not None else self._current_parent

        series_tags = []
        with dpg.plot(label=label, height=320, width=-1, parent=container):
            # Legend below/outside the axes so it never blocks the plotted lines.
            dpg.add_plot_legend(location=dpg.mvPlot_Location_South,
                                outside=True, horizontal=True)
            # x is a fixed [0, history] sample index (oscilloscope), so it never
            # needs refitting as the trace scrolls.
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="samples",
                                       no_tick_labels=True)
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="joint [rad]")
            # Right-side degrees ruler: no series attached; step() sets its limits
            # to the rad axis limits x (180/pi) so it always reads correctly.
            y_axis_deg = dpg.add_plot_axis(dpg.mvYAxis2, label="joint [deg]",
                                           opposite=True)
            for i, lbl in enumerate(series_labels):
                tag = dpg.add_line_series([], [], label=lbl, parent=y_axis)
                # Pin each line's color so the readout chip can reuse the same
                # RGB (DPG cannot report auto-assigned colormap colors).
                with dpg.theme() as line_theme:
                    with dpg.theme_component(dpg.mvLineSeries):
                        dpg.add_theme_color(dpg.mvPlotCol_Line, (*colors[i], 255),
                                            category=dpg.mvThemeCat_Plots)
                dpg.bind_item_theme(tag, line_theme)
                series_tags.append(tag)

        # Readout: robot name, a divider, then a table with one column per arm
        # group. Each cell is a [chip, text] horizontal pair so the chip sits
        # right next to its own value. Dual arm shares joint r of L and R on a row.
        header_tag = dpg.add_text("", parent=container, color=(200, 200, 160, 255))
        dpg.add_separator(parent=container)
        readout_tags = [None] * n
        table_tag = dpg.add_table(parent=container, header_row=False,
                                  resizable=False, borders_innerV=False,
                                  borders_outerV=False, borders_innerH=False)
        for _g in range(n_groups):
            dpg.add_table_column(parent=table_tag)   # one column per arm group
        for r in range(group_size):
            with dpg.table_row(parent=table_tag):
                for g in range(n_groups):
                    idx = g * group_size + r
                    # Chip and text side by side so they read as one unit.
                    with dpg.group(horizontal=True):
                        dpg.add_color_button(default_value=(*colors[idx], 255),
                                             width=16, height=16, no_alpha=True,
                                             no_drag_drop=True, no_tooltip=True)
                        readout_tags[idx] = dpg.add_text(
                            "", color=_READOUT_TEXT_COLOR)
        h = self._new_handle()
        md = {
            "handle": h,
            "source": source,
            "header_source": header_source,
            "header_tag": header_tag,
            "series_tags": series_tags,
            "labels": list(series_labels),
            "readout_tags": readout_tags,
            "readout_alert": [None] * n,  # last out-of-range state per series
            "x_axis": x_axis,
            "y_axis": y_axis,
            "y_axis_deg": y_axis_deg,
            "history": history,
            "ys": [deque(maxlen=history) for _ in range(n)],
            "visible": True,   # updated by set_visible(); gates recording in step()
            "fitted": False,   # one-time initial fit flag (item 2)
        }
        self._live_multi.append(md)
        self._handles[h] = {"kind": "live_multi", "container_tag": container,
                            "multi": md}
        return h

    def add_history_plot(self, label, series_labels, y_label, *, parent=None,
                         group_size=None, palette=None, history=64):
        """Push-based multi-series plot for per-event data + a value readout table.

        Unlike add_live_multi_plot (polled every step() with a rad/deg readout),
        this plot is fed one sample per explicit history_push() call, its x-axis
        is the event/iteration index, and it carries no unit assumption -- pass a
        plain y_label (e.g. "pos err [mm]" or "rot err [deg]"). Used by the
        visual-servoing tracker, where one point lands after each iteration.

        Rendering happens in step() (not in history_push): history_push only
        appends to the buffers, and step() -- inside DPG's render pass -- pushes
        the buffers to the series and fits the axes. Fitting from history_push
        (which runs in the monitor's task-pump phase, outside the render pass)
        does not take effect, which left points off-screen. Each series also gets
        a circle marker so single points are visible.

        Args:
            label (str): Plot title.
            series_labels (list): One legend/readout label per series (len N).
            y_label (str): Y-axis label including unit (e.g. "tool0 pos err [mm]").
            parent: Container tag (e.g. a window from add_window); defaults to the
                current parent. set_visible() toggles it.
            group_size (int): Series per readout column-group (e.g. 3 axes/arm);
                a 6-series plot lays out as two L | R column-pairs. Defaults to all-in-one.
            palette (list): RGB tuples per series; defaults to _MULTI_PLOT_PALETTE.
            history (int): Max samples kept (a servoing run is only a handful).

        Returns:
            int: Handle for set_visible() / history_push() / history_reset().
        """
        dpg = self.dpg
        n = len(series_labels)
        group_size = group_size or n
        n_groups = max(1, n // group_size)
        palette = palette or _MULTI_PLOT_PALETTE
        colors = [tuple(palette[i % len(palette)]) for i in range(n)]
        container = parent if parent is not None else self._current_parent

        series_tags = []
        with dpg.plot(label=label, height=260, width=-1, parent=container):
            dpg.add_plot_legend(location=dpg.mvPlot_Location_South,
                                outside=True, horizontal=True)
            # x = iteration index; both axes are fit in step() as points arrive.
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="iteration")
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label=y_label)
            for i, lbl in enumerate(series_labels):
                tag = dpg.add_line_series([], [], label=lbl, parent=y_axis)
                # Pin each line's color AND draw a circle marker at every sample,
                # so a lone point (or a two-point line) is clearly visible.
                with dpg.theme() as line_theme:
                    with dpg.theme_component(dpg.mvLineSeries):
                        dpg.add_theme_color(dpg.mvPlotCol_Line, (*colors[i], 255),
                                            category=dpg.mvThemeCat_Plots)
                        dpg.add_theme_style(dpg.mvPlotStyleVar_Marker,
                                            dpg.mvPlotMarker_Circle,
                                            category=dpg.mvThemeCat_Plots)
                        dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 4,
                                            category=dpg.mvThemeCat_Plots)
                dpg.bind_item_theme(tag, line_theme)
                series_tags.append(tag)

        # Readout: a title line, a divider, then one [chip | value] cell per
        # series, grouped into columns exactly like add_live_multi_plot.
        header_tag = dpg.add_text("", parent=container, color=(200, 200, 160, 255))
        dpg.add_separator(parent=container)
        readout_tags = [None] * n
        table_tag = dpg.add_table(parent=container, header_row=False,
                                  resizable=False, borders_innerV=False,
                                  borders_outerV=False, borders_innerH=False)
        for _g in range(n_groups):
            dpg.add_table_column(parent=table_tag)
        for r in range(group_size):
            with dpg.table_row(parent=table_tag):
                for g in range(n_groups):
                    idx = g * group_size + r
                    with dpg.group(horizontal=True):
                        dpg.add_color_button(default_value=(*colors[idx], 255),
                                             width=16, height=16, no_alpha=True,
                                             no_drag_drop=True, no_tooltip=True)
                        readout_tags[idx] = dpg.add_text(
                            "", color=_READOUT_TEXT_COLOR)
        h = self._new_handle()
        md = {
            "handle": h,
            "name": label,
            "series_tags": series_tags,
            "labels": list(series_labels),
            "readout_tags": readout_tags,
            "header_tag": header_tag,
            "x_axis": x_axis,
            "y_axis": y_axis,
            "history": history,
            "xs": deque(maxlen=history),
            "ys": [deque(maxlen=history) for _ in range(n)],
            "next_x": 0,
            "visible": True,   # gates rendering in step(); flipped by set_visible()
            "dirty": True,     # redraw pending (set by history_push / _reset)
        }
        self._history_plots.append(md)
        self._handles[h] = {"kind": "history_plot", "container_tag": container,
                            "history": md}
        return h

    def history_push(self, handle, values, x=None):
        """Append one sample (one value per series) to the buffers; step() draws it.

        A length mismatch is ignored so a mid-run reconfigure can't desync buffers.

        Args:
            handle (int): Handle from add_history_plot().
            values (Sequence[float]): One value per series, same order as labels.
            x (float): Optional explicit x; defaults to an auto-incrementing index.
        """
        info = self._handles.get(handle)
        if not info or "history" not in info:
            return
        md = info["history"]
        if len(values) != len(md["series_tags"]):
            return
        if x is None:
            x = md["next_x"]
        md["next_x"] = x + 1
        md["xs"].append(float(x))
        for i in range(len(md["series_tags"])):
            md["ys"][i].append(float(values[i]))
        md["dirty"] = True

    def history_reset(self, handle):
        """Clear a history plot's buffers so a new run starts from a blank plot."""
        info = self._handles.get(handle)
        if not info or "history" not in info:
            return
        md = info["history"]
        md["xs"].clear()
        md["next_x"] = 0
        for ys in md["ys"]:
            ys.clear()
        md["dirty"] = True

    def set_visible(self, handle, visible):
        """Show or hide a widget/section by handle.

        For a live multi-plot this toggles its container (the separate window)
        and flips the record gate so a hidden plot costs nothing (item 5).
        """
        info = self._handles.get(handle)
        if not info:
            return
        if "multi" in info:
            info["multi"]["visible"] = bool(visible)
        if "history" in info:
            # Re-showing forces a redraw so buffered points appear immediately.
            info["history"]["visible"] = bool(visible)
            if visible:
                info["history"]["dirty"] = True
        tag = info.get("container_tag", info.get("tag"))
        if tag is not None and self.dpg.does_item_exist(tag):
            self.dpg.configure_item(tag, show=bool(visible))

    def add_separator(self, label):
        dpg = self.dpg
        dpg.add_separator(parent=self._current_parent)
        text_tag = dpg.add_text(label, parent=self._current_parent,
                                color=(180, 180, 220, 255))
        # Make the header text larger (20 pt) than the 16 pt default widgets.
        if self._font_sep:
            dpg.bind_item_font(text_tag, self._font_sep)
        h = self._new_handle()
        self._handles[h] = {"kind": "separator", "tag": text_tag}
        return h

    def begin_group(self, label, *, collapsible=True):
        dpg = self.dpg
        if collapsible:
            tag = dpg.add_collapsing_header(label=label, default_open=True,
                                            parent=self._current_parent)
        else:
            dpg.add_text(label, parent=self._current_parent,
                         color=(180, 180, 220, 255))
            tag = dpg.add_group(parent=self._current_parent)
        self._parent_stack.append(tag)

    def end_group(self):
        if len(self._parent_stack) > 1:
            self._parent_stack.pop()

    def poll(self, handle, kind, on_change=None):
        # DPG fires callbacks directly; nothing to poll.
        return

    def get_value(self, handle):
        rec = self._handles.get(handle)
        if rec is None:
            return None
        tag = rec.get("tag")
        if tag is None or not self.dpg.does_item_exist(tag):
            return None
        try:
            return self.dpg.get_value(tag)
        except Exception:
            return None

    def set_value(self, handle, value):
        """Write a value into an existing widget, e.g. to blank an entry box."""
        rec = self._handles.get(handle)
        if rec is None:
            return
        tag = rec.get("tag")
        if tag is None or not self.dpg.does_item_exist(tag):
            return
        self.dpg.set_value(tag, value)

    def clear(self) -> None:
        # Delete only the root window's child widgets so a rebuild doesn't stack a
        # second copy. The "root" window itself, font registries, and the separator
        # font survive (they're not children of the window).
        self.dpg.delete_item("root", children_only=True)
        self._parent_stack = ["root"]
        self._handles.clear()
        self._live_plots.clear()
        self._live_multi.clear()
        self._history_plots.clear()

    def step(self) -> bool:
        dpg = self.dpg
        if not dpg.is_dearpygui_running():
            return False
        self._frame_idx += 1
        for plot in self._live_plots:
            try:
                v = float(plot["source"]())
            except Exception as e:  # source may not be ready yet
                logger.debug(f"live plot source error: {e}")
                continue
            plot["x"].append(self._frame_idx)
            plot["y"].append(v)
            dpg.set_value(plot["series_tag"],
                          [list(plot["x"]), list(plot["y"])])
            # Refit axes occasionally rather than every frame to avoid flicker
            # and reduce per-tick cost.
            if self._frame_idx % 20 == 0:
                dpg.fit_axis_data(plot["y_axis"])
                dpg.fit_axis_data(plot["x_axis"])
        # Multi-series live plots: one scrolling line per value plus a
        # color-chipped table readout showing radians and degrees.
        for plot in self._live_multi:
            # Only record/draw while the section is visible so a hidden window
            # costs nothing (item 5).
            if not plot.get("visible", True):
                continue
            try:
                vals = plot["source"]()
            except Exception as e:  # source may not be ready yet
                logger.debug(f"live multi-plot source error: {e}")
                continue
            # A value/series count mismatch (e.g. mid arm/robot switch) would
            # desync the per-series buffers, so skip until the counts agree.
            if len(vals) != len(plot["series_tags"]):
                continue
            if plot["header_tag"] is not None and plot["header_source"] is not None:
                try:
                    dpg.set_value(plot["header_tag"], str(plot["header_source"]()))
                except Exception as e:
                    logger.debug(f"live multi-plot header error: {e}")
            for i in range(len(plot["series_tags"])):
                plot["ys"][i].append(float(vals[i]))
            # Oscilloscope x: a fixed [0, history] index window that never needs
            # refitting as the trace scrolls (item 2).
            xs = list(range(len(plot["ys"][0])))
            for i, series_tag in enumerate(plot["series_tags"]):
                y = plot["ys"][i]
                dpg.set_value(series_tag, [xs, list(y)])
                v = y[-1]
                deg = np.degrees(v)
                readout_tag = plot["readout_tags"][i]
                dpg.set_value(
                    readout_tag,
                    f"{plot['labels'][i]}: {v:+.3f} rad /{deg:+7.1f} deg")
                # A joint past +/-345 deg turns its readout text bold + red
                # (name, radians and degrees all). Only restyle on a state change
                # to avoid rebinding fonts every frame.
                alert = deg < -_JOINT_LIMIT_DEG or deg > _JOINT_LIMIT_DEG
                if alert != plot["readout_alert"][i]:
                    plot["readout_alert"][i] = alert
                    dpg.configure_item(
                        readout_tag,
                        color=_READOUT_ALERT_COLOR if alert else _READOUT_TEXT_COLOR)
                    # Bold only if both a bold and a default font are available
                    # (so we can reliably revert); otherwise the red alone signals.
                    if self._font_bold is not None and self._font_default is not None:
                        dpg.bind_item_font(
                            readout_tag,
                            self._font_bold if alert else self._font_default)
            # Fit once when data first appears, then never again so the user's
            # manual zoom/pan sticks (item 2).
            if not plot["fitted"] and len(plot["ys"][0]) >= 2:
                dpg.fit_axis_data(plot["y_axis"])
                dpg.set_axis_limits(plot["x_axis"], 0, plot["history"])
                plot["fitted"] = True
            # Mirror the radians axis onto the right-side degrees ruler (item 3),
            # tracking whatever range the user has zoomed to.
            try:
                lo, hi = dpg.get_axis_limits(plot["y_axis"])
                dpg.set_axis_limits(plot["y_axis_deg"], lo * _RAD_TO_DEG,
                                    hi * _RAD_TO_DEG)
            except Exception as e:
                logger.debug(f"deg axis sync error: {e}")
        # Push-based per-iteration plots: redraw from the buffers only when a new
        # sample has been pushed (dirty). Fitting here -- inside the render pass --
        # is what makes the points actually appear (fitting from history_push,
        # which runs in the task-pump phase, does not take).
        for plot in self._history_plots:
            if not plot.get("visible", True) or not plot.get("dirty", False):
                continue
            xs = list(plot["xs"])
            n_pts = len(xs)
            for i, series_tag in enumerate(plot["series_tags"]):
                ys = list(plot["ys"][i])
                dpg.set_value(series_tag, [xs, ys])
                if ys:
                    dpg.set_value(plot["readout_tags"][i],
                                  f"{plot['labels'][i]}: {ys[-1]:+.3f}")
            if plot["header_tag"] is not None:
                dpg.set_value(plot["header_tag"],
                              f"{plot['name']}  (n={n_pts})")
            if n_pts >= 1:
                dpg.fit_axis_data(plot["x_axis"])
                dpg.fit_axis_data(plot["y_axis"])
                # A single point fits to a zero-width range (invisible); widen it.
                if n_pts == 1:
                    x0 = xs[0]
                    dpg.set_axis_limits(plot["x_axis"], x0 - 1.0, x0 + 1.0)
            plot["dirty"] = False
        dpg.render_dearpygui_frame()
        return True

    def shutdown(self) -> None:
        try:
            self.dpg.destroy_context()
        except Exception as e:
            logger.debug(f"dpg.destroy_context error: {e}")


def make_backend(use_dpg: bool, *, window_title: str = "Husky Monitor",
                 width: int = 600, height: int = 1000,
                 font_size: int = 18) -> UIBackend:
    """Factory: returns DearPyGuiBackend if use_dpg else PyBulletBackend.

    Falls back to PyBulletBackend with a logged error if dearpygui isn't installed.
    """
    if use_dpg:
        try:
            return DearPyGuiBackend(window_title=window_title,
                                    width=width, height=height,
                                    font_size=font_size)
        except ImportError as e:
            logger.error(
                f"dearpygui not installed; falling back to PyBulletBackend. "
                f"Install: pip install dearpygui. {e}")
            return PyBulletBackend()
        except Exception as e:
            # DPG init can fail on headless hosts (no display, OpenGL missing,
            # viewport setup errors). Fall back rather than crash the monitor.
            logger.error(
                f"DearPyGuiBackend init failed ({type(e).__name__}: {e}); "
                f"falling back to PyBulletBackend.")
            return PyBulletBackend()
    return PyBulletBackend()
