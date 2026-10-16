"""Tests for the phone page: pinned ports, a plugin's tab and headers, and Stop all."""

from __future__ import annotations

import socket
import threading

import pytest

from husky_assembly_teleop.ui.visualization import Visualization


@pytest.fixture
def viz():
    """Both servers on free ports."""
    viz = Visualization(port=0, phone_port=0)
    yield viz
    viz.stop()


@pytest.fixture
def taken_port():
    """A port some other program is listening on."""
    with socket.socket() as sock:
        sock.bind(("0.0.0.0", 0))
        sock.listen()
        yield sock.getsockname()[1]


@pytest.mark.parametrize("which", ["desktop", "phone"])
def test_taken_port_refuses_to_start(taken_port, which):
    """A taken port stops the monitor from starting, instead of viser quietly moving to the next port."""
    threads = threading.active_count()
    ports = {"port": 0, "phone_port": 0, ("port" if which == "desktop" else "phone_port"): taken_port}
    with pytest.raises(RuntimeError, match=f"Port {taken_port} is taken"):
        Visualization(**ports)
    # Neither server is left running, also when the second one failed.
    assert threading.active_count() == threads


def test_phone_tab_on_first_use_and_removed_by_clear(viz):
    """A plugin gets one phone tab, made when first used; teardown removes the tab and both headers."""
    tabs = viz._phone.tabs
    view = viz.view_for("demo")
    assert tabs._tab_labels == ()
    with view.phone("Demo") as gui:
        gui.add_button("Go")
    with view.phone() as gui:
        gui.add_button("Again")
    with view.phone_header() as gui:
        gui.add_html("status")
    with view.header() as gui:
        gui.add_html("desktop status")
    assert tabs._tab_labels == ("Demo",)
    headers = (view._phone_header, view._header)
    view.clear()
    assert tabs._tab_labels == ()
    assert all(header._impl.removed for header in headers)


def test_phone_stop_all_raises_the_soft_stop(viz):
    """The phone's Stop all raises the same flag as the desktop's."""
    assert not viz.take_stop_request()
    for callback in viz._phone.stop._impl.update_cb:
        callback(None)
    assert viz.take_stop_request()
