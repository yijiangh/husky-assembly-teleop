"""
The tool vocabulary (format §5.4): every kind has one lasting state, its grip; some kinds also have drives.

A grip value says what the tool does to the body it is on once a movement is done. A drive runs during a movement and
leaves no state. How a change or a drive ends (stall, timeout) is fixed per kind and belongs to the controller.

! Part of the schema: adding a kind, a grip value or a drive raises `SCHEMA`.
"""

from __future__ import annotations

from typing import Dict, Tuple

#: The values of the one tool state channel, `grip`.
GRIP: Tuple[str, ...] = ("open", "closed")

#: Kind -> the directions of its drive (the jointing screw); empty for kinds without one.
TOOL_KINDS: Dict[str, Tuple[str, ...]] = {
    "scaffolding_v3": ("tighten", "loosen"),
    "scaffolding_v1": (),
    "robotiq": (),
}
