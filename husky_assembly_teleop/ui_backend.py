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
                       numeric: bool = False) -> int:
        raise NotImplementedError

    def add_file_dialog(self, label: str, on_select: Callable[[str], None], *,
                        base_dir: Optional[str] = None,
                        ext_filter: Optional[str] = None) -> int:
        raise NotImplementedError

    def add_live_plot(self, label: str, source: Callable[[], float],
                      history: int = 200) -> int:
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
        dbg = p.addUserDebugParameter(label, vmin, vmax, default)
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_float", "dbg": dbg, "prev": prev, "cb": on_change}
        return h

    def add_slider_int(self, label, vmin, vmax, default, on_change):
        dbg = p.addUserDebugParameter(label, float(vmin), float(vmax), float(default))
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        wrapped = lambda v, _cb=on_change: _cb(int(round(v)))
        self._handles[h] = {"kind": "slider_int", "dbg": dbg, "prev": prev, "cb": wrapped}
        return h

    def add_slider_group(self, labels, vmins, vmaxs, defaults, on_change):
        dbgs = [p.addUserDebugParameter(lbl, vmn, vmx, dv)
                for lbl, vmn, vmx, dv in zip(labels, vmins, vmaxs, defaults)]
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
        n = max(len(options) - 1, 0)
        dbg = p.addUserDebugParameter(label, 0.0, float(n), float(default_idx))
        prev = p.readUserDebugParameter(dbg)
        h = self._new_handle()
        wrapped = lambda v, _cb=on_change: _cb(int(round(v)))
        self._handles[h] = {"kind": "combo", "dbg": dbg, "prev": prev, "cb": wrapped}
        return h

    def add_text_input(self, label, default, on_change, *, numeric=False):
        raise NotImplementedError(
            "text_input widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_file_dialog(self, label, on_select, *, base_dir=None, ext_filter=None):
        raise NotImplementedError(
            "file_dialog widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_live_plot(self, label, source, history=200):
        raise NotImplementedError(
            "live_plot widget not supported by PyBulletBackend; set USE_DPG_UI=1")

    def add_separator(self, label):
        dbg = p.addUserDebugParameter(label, 0.0, 1.0, 0.0)
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

    def step(self) -> bool:
        return True

    def shutdown(self) -> None:
        pass


class DearPyGuiBackend(UIBackend):
    """Real DPG-based backend with full widget support."""

    # First existing path wins; covers Linux/macOS/Windows defaults.
    _DEFAULT_FONT_PATHS = (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "C:/Windows/Fonts/segoeui.ttf",
    )

    def __init__(self, window_title: str = "Husky Monitor",
                 width: int = 600, height: int = 1000,
                 font_size: int = 18) -> None:
        import dearpygui.dearpygui as dpg  # lazy import
        self.dpg = dpg

        dpg.create_context()
        dpg.create_viewport(title=window_title, width=width, height=height)
        self._bind_default_font(font_size)
        dpg.setup_dearpygui()

        with dpg.window(tag="root", label=window_title,
                        width=width, height=height, no_close=True):
            pass
        dpg.set_primary_window("root", True)

        self._parent_stack: List[Any] = ["root"]
        self._next_handle = 1
        self._handles: Dict[int, Dict[str, Any]] = {}
        self._live_plots: List[Dict[str, Any]] = []
        self._frame_idx = 0

        dpg.show_viewport()

    def _new_handle(self) -> int:
        h = self._next_handle
        self._next_handle += 1
        return h

    def _bind_default_font(self, font_size: int) -> None:
        dpg = self.dpg
        for path in self._DEFAULT_FONT_PATHS:
            if not os.path.exists(path):
                continue
            try:
                with dpg.font_registry():
                    f = dpg.add_font(path, font_size)
                dpg.bind_font(f)
                return
            except Exception as e:
                logger.debug(f"font load failed {path}: {e}")
        # No TTF available — scale the built-in bitmap font instead. Looks
        # blocky but stays readable.
        dpg.set_global_font_scale(max(1.0, font_size / 13.0))

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
        tag = dpg.add_slider_float(label=label, min_value=vmin, max_value=vmax,
                                   default_value=default, parent=self._current_parent,
                                   callback=lambda s, app_data, u: on_change(app_data))
        h = self._new_handle()
        self._handles[h] = {"kind": "slider_float", "tag": tag}
        return h

    def add_slider_int(self, label, vmin, vmax, default, on_change):
        dpg = self.dpg
        tag = dpg.add_slider_int(label=label, min_value=int(vmin), max_value=int(vmax),
                                 default_value=int(default), parent=self._current_parent,
                                 callback=lambda s, app_data, u: on_change(int(app_data)))
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
            t = dpg.add_slider_float(label=lbl, min_value=vmn, max_value=vmx,
                                     default_value=dv, parent=self._current_parent,
                                     callback=_fan_out)
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

    def add_text_input(self, label, default, on_change, *, numeric=False):
        dpg = self.dpg
        if numeric:
            tag = dpg.add_input_float(
                label=label, default_value=float(default or 0),
                parent=self._current_parent,
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

    def add_separator(self, label):
        dpg = self.dpg
        dpg.add_separator(parent=self._current_parent)
        text_tag = dpg.add_text(label, parent=self._current_parent,
                                color=(180, 180, 220, 255))
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
