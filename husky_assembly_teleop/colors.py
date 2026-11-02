"""The monitor's colours in normal operation: one value per hue and a name per meaning, for panels, ghosts and markers.

- Blue is "I do stuff": a button that does something is ACTION blue, viser's theme colour, unless its meaning has a
  colour (Go and Confirm OK green, Stop and Cancel FAIL red, Plan teal, pending rows grey). INFO, CURRENT and TARGET
  share the blue: a fact, where a sequence is, where the robot should go.
- Status colours (OK, WARN, FAIL, NONE, INFO) say how something is (NONE also greys debug sections and secondary
  text); CURRENT marks where a sequence is.
- Section colours name a kind of panel section; scene names say what a body or marker is; several names may share
  one value (TARGET and INFO are the same blue), so a meaning never gets its own slightly different shade.
- Plots and debug views (trace plots, the coverage map, the base planner's debug layers) keep their own colours:
  a line `# * Own colours: <why>` in the file lets the colour test skip it.
- No two hues are closer than ΔE2000 12 (`test_colors` checks): two meanings that would get close shades share one
  hue instead, or one gets a clearly different hue.
! Never type a UI or scene colour anywhere else: import it from here (`test/structure/test_colors.py` checks).
"""

from __future__ import annotations


def rgb(color: str) -> tuple[int, int, int]:
    """`color` as (r, g, b) from 0 to 255, e.g. for viser buttons and scene markers."""
    return int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)


def rgb01(color: str, alpha: float | None = None) -> tuple[float, ...]:
    """`color` as (r, g, b) from 0 to 1, with `alpha` appended when given, e.g. for meshes and matplotlib."""
    channels = tuple(value / 255 for value in rgb(color))
    return channels if alpha is None else (*channels, alpha)


# * The hues: every name below is one of these.
GREEN = "#2b8a3e"
AMBER = "#e67700"
RED = "#e03131"
GREY = "#6c757d"
BLUE = "#1971c2"
CYAN = "#0c8599"
GRAPE = "#ae3ec9"
TEAL = "#0ca678"
NAVY = "#1a4599"
YELLOW = "#fab005"
DARK_GREY = "#495057"
LIGHT_GREY = "#adb5bd"
INK = "#212529"
WHITE = "#ffffff"
#: The word for each hue, for text that names a colour (e.g. the health log).
HUE_NAMES = {GREEN: "green", AMBER: "amber", RED: "red", GREY: "grey", BLUE: "blue", CYAN: "cyan",
             GRAPE: "grape", TEAL: "teal", NAVY: "navy", YELLOW: "yellow", DARK_GREY: "dark grey",
             LIGHT_GREY: "light grey", INK: "black", WHITE: "white"}

# * Status: chips put white text on these.
OK = GREEN          # ok, within limit, ready, done; Go and Confirm buttons
WARN = AMBER        # moving or in progress, over a warn limit, stale, waiting for a second press, Clearance off
FAIL = RED          # failed, refused, over a red limit, collision; Stop and Cancel buttons
NONE = GREY         # no data yet, pending, not done; also small secondary text
INFO = BLUE         # a fact or an expected end state (screwed tight, sim), as a filled chip
CURRENT = BLUE      # where a sequence is: a ring around the current step, the current row, progress bars
#: "I do stuff": every button that does something, and viser's theme colour for buttons, sliders and checkboxes.
ACTION = BLUE
#: ACTION as viser's 10-shade theme palette (`ui.style.configure_theme`): Mantine fills with shade 6 in the light
#: theme and 8 in the dark, hovers one darker; the lighter shades are Mantine's blue.
ACTION_SHADES = ("#e7f5ff", "#d0ebff", "#a5d8ff", "#74c0fc", "#4dabf7", "#339af0", ACTION, "#1864ab", ACTION, "#1864ab")

# * Panel sections: the kind of controls below the bar.
SECTION_CTRL = BLUE
SECTION_SENSOR = CYAN
SECTION_TOOL = GRAPE

# * Scene: what a body, ghost or marker is.
TARGET = BLUE       # where the robot should go, and the panel controls that set it
PLAN = GREEN        # where the plan has the robot: plan ghost, sent path
PREVIEW = BLUE      # a path that would be sent: the base planner's preview
#: What the robot really did, drawn as a marker (robots on the phone map): mid grey, seen on the light
#: and the dark scene and lighter than the tools.
ACTUAL = GREY
TRAIL = AMBER       # where the base drove: it moved
MEASURED = CYAN     # mocap measurements: recorded probe points, cameras
FITTED = BLUE       # where a fit puts what was measured: fitted mocap markers, axes and bars
MOUNTED = NAVY      # the joint halves mounted on a fitted bar, darker than it
HELD = AMBER        # a body held by a robot's tool: the old orange, in transit
COLLISION = RED     # what moved into something
HIT = CYAN          # what it moved into (see-through, the red visible inside)
CONTACT = YELLOW    # contact points and the line between them: seen on the see-through HIT, on red and either scene
SELECTED = YELLOW   # what the operator selected, e.g. recorded probe points; far lighter than any other colour
TEMPORARY = GRAPE   # a temporary (session) obstacle
BODY = LIGHT_GREY   # a scene body without its own colour, lab furniture
OUTLINE = DARK_GREY  # outlines and footprints
LAB_BORDER = AMBER  # the lab's border walls (the orange floor tape)
