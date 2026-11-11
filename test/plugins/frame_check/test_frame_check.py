"""The frame_check plugin: each measured bar's ends against its design, the readouts, and the picture on both pages."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
from planning_fixtures import purged_process
from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins.frame_check.picture import HEIGHT, WIDTH, ordered_ends, render
from husky_assembly_teleop.plugins.frame_check.plugin import (ALL, MAX_AGE, REDRAW_MM, REFRESH_PERIOD,
                                                              FrameCheckPlugin, bar_errors, bar_readout,
                                                              frame_readout, summary_chip)
from husky_assembly_teleop.colors import FAIL, rgb
from husky_assembly_teleop.shared.bars.layouts import bar_layouts
from scaffolding_assembly_core.geometry import Pose

#: The 10 Nov run's lean: B21's top 31 mm towards -y once the frame sprang (rotation about x at its bottom end).
LEAN_RAD = 0.031 / 2.0


def leaned(T: np.ndarray, length: float, angle: float) -> Pose:
    """A bar pose `T` (middle origin, z along it) turned by `angle` about world x around its -z end."""
    c, s = np.cos(angle), np.sin(angle)
    turn = np.eye(4)
    turn[1:3, 1:3] = [[c, -s], [s, c]]
    foot = T @ [0.0, 0.0, -length / 2.0, 1.0]
    to_foot, back = np.eye(4), np.eye(4)
    to_foot[:3, 3], back[:3, 3] = -foot[:3], foot[:3]
    return Pose.from_matrix(back @ turn @ to_foot @ T)


def bench(lean_bar: str = "bars/B21"):
    """The plugin on the purged process; every bar built and fitted at its seat, `lean_bar` leaned like 10 Nov."""
    process = purged_process().process
    layouts = bar_layouts(process)
    fits = {}
    for bar, layout in layouts.items():
        T = process.bodies[bar].assembly_pose.matrix()
        pose = leaned(T, layout.length, LEAN_RAD) if bar == lean_bar else Pose.from_matrix(T)
        fits[bar] = SimpleNamespace(pose=pose, measured_at=10.0, planned=Pose.from_matrix(T))
    selection = SimpleNamespace(process=process)
    ctx = FakeContext(plugins={"process": SimpleNamespace(selection=selection),
                               "mocap_bars": SimpleNamespace(fits=fits)}, name="frame_check")
    plugin = FrameCheckPlugin()
    plugin.setup(ctx)
    return plugin, ctx, process, layouts, fits


def test_a_post_is_named_bottom_to_top_and_a_lying_bar_along_its_main_axis():
    """Ends come low to high whatever order the fit gives, and keep their pairing with the measured ends."""
    post = np.array([[0.0, 0.0, 2.0], [0.0, 0.0, 0.0]])
    design, measured, names = ordered_ends(post, post + [[0.0, 0.02, 0.0], [0.0, 0.01, 0.0]])
    assert names == ("bottom", "top") and design[0, 2] == 0.0
    np.testing.assert_allclose(measured - design, [[0.0, 0.01, 0.0], [0.0, 0.02, 0.0]])
    lying = np.array([[1.8, 0.0, 0.1], [0.0, 0.1, 0.1]])
    design, measured, names = ordered_ends(lying, None)
    assert names == ("-x end", "+x end") and design[0, 0] == 0.0 and measured is None


def test_a_leaning_post_reads_its_top_to_be_moved_back_by_the_lean():
    """B21 leaned 31 mm at its top towards -y: move the top +31 mm in y, the bottom not at all; the rest are fine."""
    _plugin, ctx, process, layouts, fits = bench()
    errors = {e.bar: e for e in bar_errors(process, layouts, fits, ctx.now())}
    b21 = errors["B21"]
    assert b21.end_names == ("bottom", "top")
    np.testing.assert_allclose(-b21.errors_mm[1], [0.0, 31.0, 0.0], atol=0.3)
    np.testing.assert_allclose(b21.errors_mm[0], 0.0, atol=1e-6)
    assert abs(b21.tilt_deg - np.degrees(LEAN_RAD)) < 1e-3
    assert all(e.worst_mm < 1e-6 for name, e in errors.items() if name != "B21")
    assert "+31.0" in bar_readout(b21) and "B21 top 31.0 mm" in summary_chip(list(errors.values()))


def test_a_stale_or_failed_fit_counts_as_not_measured():
    """A fit older than MAX_AGE, or one with no pose, leaves the bar unmeasured, named so in the frame readout."""
    _plugin, ctx, process, layouts, fits = bench()
    fits["bars/B21"].measured_at = ctx.now() - MAX_AGE - 0.1
    fits["bars/B20"].pose = None
    errors = {e.bar: e for e in bar_errors(process, layouts, fits, ctx.now())}
    assert errors["B21"].why == "stale" and errors["B20"].why == "no fit"
    readout = frame_readout(list(errors.values()))
    assert "stale" in readout and "no fit" in readout
    assert "2 not measured" in summary_chip(list(errors.values()))


def test_both_pages_show_the_same_picture_and_send_it_again_only_when_it_changes():
    """Desktop and phone get one picture; an unchanged frame sends none, a chosen bar its own readout on both."""
    plugin, ctx, *_ = bench()
    plugin.update(ctx)
    plugin.draw(ctx)
    images = ctx.gui.of_kind("image")
    assert len(images) == 2 and images[0].image.shape == (HEIGHT, WIDTH, 3)
    assert images[0].image is images[1].image
    first = images[0].image
    ctx.t += REFRESH_PERIOD
    plugin.update(ctx)
    plugin.draw(ctx)
    assert images[0].image is first
    plugin._choose_bar("B21")
    plugin.draw(ctx)
    assert images[0].image is not first
    readouts = [w for w in ctx.gui.of_kind("html") if "move" in w.content]
    assert len(readouts) == 2 and "+31.0" in readouts[0].content


def test_a_bar_gone_from_the_process_falls_back_to_all():
    """The bar row lists the process's bars; a chosen bar no longer there shows every bar again."""
    plugin, ctx, *_ = bench()
    plugin.update(ctx)
    plugin._choose_bar("B99")
    plugin.draw(ctx)
    assert plugin._bar == ALL


def test_the_picture_draws_without_bars_and_in_every_view():
    """No bar: a message instead; every view and magnification draws the same size."""
    assert render([], "side", 10.0, None).shape == (HEIGHT, WIDTH, 3)
    plugin, ctx, process, layouts, fits = bench()
    errors = bar_errors(process, layouts, fits, ctx.now())
    for view in ("side", "front", "top"):
        assert render(errors, view, 30.0, "B21").shape == (HEIGHT, WIDTH, 3)


def test_a_held_or_unbuilt_bar_is_left_out_of_the_frame():
    """A bar the scene shows elsewhere (held) or not at all (not built yet) is neither listed nor in the chip."""
    _plugin, ctx, process, layouts, fits = bench()
    fits["bars/B21"].pose = fits["bars/B21"].planned = Pose((0.5, 0.5, 1.0))  # in a gripper, far off its seat
    fits["bars/B20"].planned = None
    errors = bar_errors(process, layouts, fits, ctx.now())
    assert {e.bar for e in errors} == {"B34", "B35"}
    assert "worst B3" in summary_chip(errors) and "not measured" not in summary_chip(errors)
    assert "no built bar" in summary_chip([])


def test_mocap_noise_alone_never_resends_the_picture_but_a_push_does():
    """Ends jittering 0.04 mm keep the picture; one end moved past REDRAW_MM sends a new one."""
    plugin, ctx, _process, _layouts, fits = bench()
    plugin.update(ctx)
    plugin.draw(ctx)
    image = ctx.gui.of_kind("image")[0]
    first = image.image
    rng = np.random.default_rng(0)
    seats = {bar: fit.pose for bar, fit in fits.items()}
    for _ in range(50):
        for bar, fit in fits.items():
            fit.pose = Pose(tuple(np.add(seats[bar].position, rng.normal(0.0, 4e-5, 3))), seats[bar].orientation)
            fit.measured_at = ctx.t + REFRESH_PERIOD
        ctx.t += REFRESH_PERIOD
        plugin.update(ctx)
        plugin.draw(ctx)
    assert image.image is first
    fits["bars/B21"].pose = Pose(tuple(np.add(seats["bars/B21"].position, [0.0, 2 * REDRAW_MM * 1e-3, 0.0])),
                                 seats["bars/B21"].orientation)
    ctx.t += REFRESH_PERIOD
    plugin.update(ctx)
    plugin.draw(ctx)
    assert image.image is not first


def test_a_fit_counts_as_fresh_as_long_as_mocap_bars_marker_age_says():
    """mocap_bars' marker age (here 3 s) sets when a fit goes stale; without one, MAX_AGE does."""
    plugin, ctx, _process, _layouts, fits = bench()
    fits["bars/B21"].measured_at = ctx.now() - MAX_AGE - 1.0
    ctx.plugins["mocap_bars"].max_marker_age = 3.0
    plugin.update(ctx)
    assert next(e for e in plugin._errors if e.bar == "B21").why == ""


def test_the_magnified_lean_stays_inside_the_picture():
    """B21's 31 mm lean at x30 is drawn whole: its red line is in the picture, off its border."""
    _plugin, ctx, process, layouts, fits = bench()
    errors = bar_errors(process, layouts, fits, ctx.now())
    picture = render(errors, "side", 30.0, "B21")
    red = np.all(picture == rgb(FAIL), axis=2)
    rows, columns = np.nonzero(red)
    assert red.sum() > 100
    assert rows.min() > 2 and columns.min() > 2 and rows.max() < HEIGHT - 3 and columns.max() < WIDTH - 3
