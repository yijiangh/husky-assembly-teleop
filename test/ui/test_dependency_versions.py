"""The installed viser and yourdfpy are the versions pinned in requirements.txt.

! The UI relies on viser internals that change between releases without notice. Before bumping either pin,
  check each of these against the new release:
  - `FastViserUrdf` (ui/visualization.py) subclasses `ViserUrdf` and reads its private attributes
    (`_urdf`, `_joint_map_values`, `_joint_frames`, `_scale`, `_root_node_name`, `_target`, `_meshes`,
    `_collision_root_frame`); ghost.py and plugins/process/drawing.py use `_meshes` too.
  - ui/checklist.py reads `_impl.uuid` from a checkbox handle.
  - CSS aimed at viser's DOM fails silently: ui/checklist.py, ui/style.py (`data-dock-*`), ui/phone.py,
    shared/teleop/dpad.py and tool_pad.py, shared/guide/panel.py (Mantine class names, `div:has(...)` paths).
    Look at the desktop and phone pages.
  - Tests read `_handle_from_node_name`, `_gui_input_handle_from_uuid` and `_impl` of handles.
"""

from __future__ import annotations

import re
from importlib.metadata import version

import pytest

from repo_paths import REPO


def _pinned(package: str) -> str:
    """The `package==x.y.z` version in requirements.txt."""
    match = re.search(rf"^{package}==([^\s#]+)", (REPO / "requirements.txt").read_text(), re.MULTILINE)
    assert match, f"{package} is not pinned with == in requirements.txt"
    return match.group(1)


@pytest.mark.parametrize("package", ["viser", "yourdfpy"])
def test_installed_version_is_the_pinned_one(package):
    """A different release installed by hand fails here, before the UI breaks in ways no other test sees."""
    pinned = _pinned(package)
    assert version(package) == pinned, (
        f"{package} {version(package)} is installed, requirements.txt pins {pinned}: install {package}=={pinned}, "
        f"or check the internals listed in {__name__}'s docstring before bumping the pin")
