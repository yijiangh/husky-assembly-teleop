"""
Write a copy of a husky URDF whose UR arms use the stock base frames.

Usage:
    python scripts/fix_ur_base_frames.py <in.urdf> [...]

Writes `<in>_StockUrFrames.urdf`. For each arm, the joints below
`<arm>_base_link` are set to the stock rpy 0 0 pi, and the rotation that was
there moves into the mount joint above, so every link except a mis-set
`<arm>_base` stays where it was.
Why: doc/ur_frames.md.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path
from xml.etree.ElementTree import parse

import numpy as np
from scipy.spatial.transform import Rotation

STOCK = Rotation.from_euler("z", np.pi)
STOCK_RPY = "0 0 3.14159265359"


def origin_rotation(joint) -> Rotation:
    """A <joint> element's origin rotation."""
    return Rotation.from_euler("xyz", [float(v) for v in joint.find("origin").get("rpy", "0 0 0").split()])


def set_rpy(text: str, joint_name: str, rpy: str) -> str:
    """Set one joint's origin rpy in URDF text.

    ! Commented-out origins are skipped: the hand-edited files keep the old
      origin commented out next to the real one.
    """
    block = re.search(r'<joint\s+name="' + re.escape(joint_name) + r'".*?</joint>', text, re.S)
    body = block.group(0)
    live = re.sub(r"<!--.*?-->", lambda m: " " * len(m.group(0)), body, flags=re.S)
    tag = re.search(r'rpy="[^"]*"', live)
    body = body[:tag.start()] + f'rpy="{rpy}"' + body[tag.end():]
    return text[:block.start()] + body + text[block.end():]


def fix_file(path: Path) -> None:
    """Write the stock-frames copy of one URDF."""
    by_child = {j.find("child").get("link"): j for j in parse(path).getroot().findall("joint")}
    text = path.read_text()
    for child in by_child:
        if not child.endswith("_base_link_inertia"):
            continue
        arm = child[:-len("_base_link_inertia")]
        mount, inertia, base = by_child[f"{arm}_base_link"], by_child[child], by_child[f"{arm}_base"]
        if (STOCK.inv() * origin_rotation(inertia)).magnitude() > 1e-6:
            # base_link_inertia stays put: mount * inertia == new_mount * stock.
            new_mount = origin_rotation(mount) * origin_rotation(inertia) * STOCK.inv()
            text = set_rpy(text, mount.get("name"), " ".join(f"{v:.12g}" for v in new_mount.as_euler("xyz")))
            print(f"{path.name} {arm}: rotation moved into {mount.get('name')}")
        for joint in (inertia, base):
            if (STOCK.inv() * origin_rotation(joint)).magnitude() > 1e-6:
                text = set_rpy(text, joint.get("name"), STOCK_RPY)
                print(f"{path.name} {arm}: {joint.get('name')} set to stock")
    out = path.with_name(path.stem + "_StockUrFrames" + path.suffix)
    out.write_text(text)
    print(f"  -> {out}")


if __name__ == "__main__":
    for argument in sys.argv[1:]:
        fix_file(Path(argument))
