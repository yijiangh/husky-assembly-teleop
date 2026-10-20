"""Tests for the top-down map: footprints and dots land at the right pixels, with world +y up the picture."""

from __future__ import annotations

import numpy as np

from husky_assembly_teleop.ui.top_down import BODY_COLOR, Dot, FloorGrid, footprint, render_top_down
from scaffolding_assembly_core.geometry import BoxShape, Pose

#: 4 x 4 cells of 1 m from (0, 0), 10 px each: (x, y) lands at column 10 x, row 40 - 10 y.
GRID = FloorGrid(origin=(0.0, 0.0), cell_size=1.0, shape=(4, 4), cell_pixels=10)
GREY = np.full((4, 4, 3), 200)


def test_box_footprint_is_its_outline_from_above():
    """A 1 x 0.5 m box turned 90° about z covers 0.5 m along x and 1 m along y."""
    turned = Pose((2.0, 1.0, 0.3), (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)))
    outline = footprint(BoxShape((1.0, 0.5, 0.2)), turned)
    assert np.allclose(np.ptp(outline, axis=0), (0.5, 1.0))
    assert np.allclose(outline.mean(axis=0), (2.0, 1.0))


def test_render_puts_bodies_and_dots_where_they_are():
    """A box at (1.5, 2.5) and a dot at (3.5, 0.5) appear at those spots, the floor everywhere else."""
    box = footprint(BoxShape((0.6, 0.6, 0.6)), Pose((1.5, 2.5, 0.3)))
    picture = render_top_down(GRID, GREY, [(box, BODY_COLOR)], [Dot(3.5, 0.5, (230, 60, 150))])
    assert picture.shape == (40, 40, 3)
    assert tuple(picture[15, 15]) == BODY_COLOR       # row 40 - 25, column 15
    assert tuple(picture[35, 35]) == (230, 60, 150)   # row 40 - 5, column 35
    assert tuple(picture[35, 15]) == (200, 200, 200)  # (1.5, 0.5): floor
