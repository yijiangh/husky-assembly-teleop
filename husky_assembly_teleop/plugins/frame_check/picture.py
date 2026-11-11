"""
How far each measured bar's ends are from the design, and a picture of it from the side, the front or above.

The picture draws every bar's design as a thin grey line and its measured line over it, coloured by its worse end,
with the error from the design magnified, so a 5 mm lean shows on a 2 m frame. It is an RGB array for viser's
`add_image`, which sends it as a PNG.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ...colors import INK, LIGHT_GREY, NONE, OUTLINE, WHITE, rgb
from ...shared.bars.pose import BarGap, gap_color
#: A bar whose axis is closer than this to vertical (cosine) names its ends bottom and top.
VERTICAL_COS = 0.7
#: The views: name -> (world axis drawn to the right, world axis drawn up, where the viewer stands).
VIEWS = {"side": (1, 2, "from +x"), "front": (0, 2, "from -y"), "top": (0, 1, "from above")}
AXIS_NAMES = "xyz"
#: Picture size in pixels; it never changes, so the phone's buttons below it stay put.
WIDTH, HEIGHT = 640, 520
#: Empty border around the design, pixels: room for the magnified errors and the labels.
MARGIN = 56


@dataclass(frozen=True)
class BarError:
    """One bar's design and measured ends, ends in the same order (`end_names`), and its `BarGap`.

    Attributes:
        bar: Its short name, e.g. "B21".
        design: (2, 3) the designed ends, world, metres.
        measured: (2, 3) the measured ends, or None without a good fresh fit.
        end_names: What the operator calls each end: ("bottom", "top"), or ("-x end", "+x end") for a lying bar.
        why: Why there is no measurement ("no fit", "stale"), "" with one.
        gap: `bar_gap` of the fit against the design, which the ends come from; None without a measurement.
    """

    bar: str
    design: np.ndarray
    measured: np.ndarray | None
    end_names: tuple[str, str]
    why: str = ""
    gap: BarGap | None = None

    @property
    def errors_mm(self) -> np.ndarray | None:
        """(2, 3) measured minus design per end, world axes, mm; None without a measurement."""
        return None if self.measured is None else (self.measured - self.design) * 1e3

    @property
    def worst_mm(self) -> float | None:
        """The larger of the two ends' distances from the design, mm."""
        return None if self.gap is None else self.gap.worst_mm

    @property
    def tilt_deg(self) -> float | None:
        """Angle between the measured and the designed axis, degrees."""
        return None if self.gap is None else self.gap.tilt_deg


def ordered_ends(design: np.ndarray,
                 measured: np.ndarray | None) -> tuple[np.ndarray, np.ndarray | None, tuple[str, str]]:
    """Both ends ordered low to high along the bar's main world axis, and their names.

    Args:
        design: (2, 3) designed ends, as `bar_ends` gives them.
        measured: (2, 3) measured ends in the same order, or None.

    Returns:
        tuple: (design, measured, end names), the ends swapped when needed.
    """
    axis = design[1] - design[0]
    direction = axis / np.linalg.norm(axis)
    if abs(direction[2]) >= VERTICAL_COS:
        main, names = 2, ("bottom", "top")
    else:
        main = int(np.argmax(np.abs(direction[:2])))
        names = (f"-{AXIS_NAMES[main]} end", f"+{AXIS_NAMES[main]} end")
    if design[0, main] > design[1, main]:
        design = design[::-1]
        measured = None if measured is None else measured[::-1]
    return design, measured, names


def _label(draw: ImageDraw.ImageDraw, xy: tuple[float, float], text: str, fill: tuple[int, int, int],
           font: ImageFont.FreeTypeFont, anchor: str, taken: list[tuple[float, float, float, float]]) -> None:
    """Write `text` at `xy`, or nudged up or down to the nearest place clear of the labels in `taken`; add its box."""
    step = font.size + 2
    for shift in (0, step, -step, 2 * step, -2 * step, 3 * step, -3 * step):
        box = draw.textbbox((xy[0], xy[1] + shift), text, font=font, anchor=anchor)
        if not any(box[0] < t[2] and t[0] < box[2] and box[1] < t[3] and t[1] < box[3] for t in taken):
            break
    taken.append(box)
    draw.text((xy[0], xy[1] + shift), text, fill=fill, font=font, anchor=anchor)


def render(errors: list[BarError], view: str, gain: float, chosen: str | None) -> np.ndarray:
    """Draw the design and the measured bars seen along one world axis, errors magnified by `gain`.

    Args:
        errors: The bars.
        view: A key of VIEWS.
        gain: How much the errors from the design are magnified.
        chosen: The bar drawn thick and labelled with its ends' errors, or None for all alike.

    Returns:
        np.ndarray: The picture, (HEIGHT, WIDTH, 3) uint8.
    """
    right, up, seen_from = VIEWS[view]
    image = Image.new("RGB", (WIDTH, HEIGHT), rgb(WHITE))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=22)
    small = ImageFont.load_default(size=18)
    if not errors:
        draw.text((WIDTH / 2, HEIGHT / 2), "no bars measured", fill=rgb(NONE), font=font, anchor="mm")
        return np.asarray(image)
    # * The magnified measured ends too, so a large lean at the frame's edge stays in the picture.
    corners = np.concatenate([e.design for e in errors] + [e.design + gain * (e.measured - e.design) for e in errors
                                                           if e.measured is not None])[:, [right, up]]
    low, high = corners.min(axis=0), corners.max(axis=0)
    span = np.maximum(high - low, 0.05)
    scale = min((WIDTH - 2 * MARGIN) / span[0], (HEIGHT - 2 * MARGIN) / span[1])
    centre = (low + high) / 2.0

    def pixel(point: np.ndarray) -> tuple[float, float]:
        """The picture's (column, row) of a world point."""
        u, v = (np.array([point[right], point[up]]) - centre) * scale
        return WIDTH / 2 + u, HEIGHT / 2 - v

    taken: list[tuple[float, float, float, float]] = []
    # * Design first, all of it, so no measured line hides under another bar's design.
    for e in errors:
        draw.line([pixel(p) for p in e.design], fill=rgb(LIGHT_GREY), width=3)
    for e in sorted(errors, key=lambda e: e.bar == chosen):
        thick = e.bar == chosen
        if e.measured is not None:
            shown = e.design + gain * (e.measured - e.design)
            color = rgb(gap_color(e.worst_mm))
            draw.line([pixel(p) for p in shown], fill=color, width=7 if thick else 4)
            for p in shown:
                u, v = pixel(p)
                draw.ellipse([u - 5, v - 5, u + 5, v + 5], fill=color)
        middle = pixel(e.design.mean(axis=0))
        _label(draw, (middle[0] + 8, middle[1]), e.bar, rgb(INK if thick or chosen is None else OUTLINE), font, "lm",
               taken)
        if thick and e.measured is not None:
            for p, error in zip(shown, e.errors_mm):
                u, v = pixel(p)
                _label(draw, (u + 10, v - 10), f"{np.linalg.norm(error):.1f} mm", rgb(INK), small, "ld", taken)
    # * Which way is which, bottom left; the magnification, bottom right.
    x0, y0 = 16, HEIGHT - 16
    draw.line([(x0, y0), (x0 + 40, y0)], fill=rgb(INK), width=2)
    draw.line([(x0, y0), (x0, y0 - 40)], fill=rgb(INK), width=2)
    draw.text((x0 + 44, y0), f"+{AXIS_NAMES[right]}", fill=rgb(INK), font=small, anchor="ls")
    draw.text((x0, y0 - 44), f"+{AXIS_NAMES[up]}", fill=rgb(INK), font=small, anchor="ls")
    draw.text((WIDTH - 16, y0), f"errors x{gain:g}", fill=rgb(OUTLINE), font=small, anchor="rs")
    draw.text((16, 16), f"seen {seen_from}", fill=rgb(OUTLINE), font=small, anchor="lt")
    return np.asarray(image)
