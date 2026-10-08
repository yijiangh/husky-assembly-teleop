"""
The tool state vocabulary (format §5.4): each tool kind's channels and their two planned values.

A value says what the tool does to the body once a movement is done; motor activity is execution data, never a value.
How a transition ends (stall, timeout) is fixed per kind and belongs to the controller, not the design.

! Part of the schema: adding a kind, a channel or a value raises `SCHEMA`.
"""

from __future__ import annotations

from typing import Dict, Tuple

#: Kind -> channel -> its values, the "released" one first (the state of a tool nobody has used yet).
TOOL_CHANNELS: Dict[str, Dict[str, Tuple[str, str]]] = {
    "scaffolding_v3": {"grip": ("open", "closed"), "joint": ("loose", "tight")},
    "scaffolding_v1": {"grip": ("open", "closed")},
    "robotiq": {"grip": ("open", "closed")},
}

#: The key of a tool state naming the body the tool sits on.
ON = "on"


def released(kind: str) -> Dict[str, str]:
    """Every channel of a tool kind at its released value (grip open, joint loose).

    Raises:
        KeyError: For a kind not in the vocabulary.
    """
    return {channel: values[0] for channel, values in TOOL_CHANNELS[kind].items()}
