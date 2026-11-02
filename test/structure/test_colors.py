"""UI and scene colours come from `colors.py` only: no hex strings, Mantine colour names or colour tuples elsewhere.

A plot or debug view keeps its own colours: put a line `# * Own colours: <why>` in its file and it is skipped.
"""

from __future__ import annotations

import ast
import math
import re
from itertools import combinations

from repo_paths import REPO

PACKAGE = REPO / "husky_assembly_teleop"
#: A file with a line starting so keeps its own colours (plots, debug views).
OWN_COLOURS = "# * Own colours:"
HEX = re.compile(r"#[0-9a-fA-F]{6}\b|#[0-9a-fA-F]{3}\b")
#: viser's button colour names (Mantine).
MANTINE = {"dark", "gray", "red", "pink", "grape", "violet", "indigo", "blue", "cyan", "green", "lime", "yellow",
           "orange", "teal", "white", "black"}
#: Not checked: the colours themselves, the old monitor, offline experiments, vendored code, and Yijiang's plugins and
#: the calibration plugin (converted later).
SKIPPED = re.compile(r"^(colors\.py|old/|experiments/|mocap_relay/natnet/"
                     r"|plugins/(assembly_run|assembly_steps|motion_planner|movement_exec|calibration)/"
                     r"|shared/(assembly_planning|execution|process_scene|runs)/)")


def _is_number_tuple(node: ast.AST) -> bool:
    """Whether `node` is a literal tuple or list of 3 or 4 numbers."""
    return (isinstance(node, (ast.Tuple, ast.List)) and len(node.elts) in (3, 4)
            and all(isinstance(e, ast.Constant) and isinstance(e.value, (int, float)) for e in node.elts))


def _colour_literals(tree: ast.AST):
    """(line, text) of each colour typed in `tree`; docstrings may name colours."""
    docs = {id(n.body[0].value) for n in ast.walk(tree)
            if isinstance(n, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
            and n.body and isinstance(n.body[0], ast.Expr) and isinstance(n.body[0].value, ast.Constant)}
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docs:
            if HEX.search(node.value):
                yield node.lineno, node.value[:60]
            elif node.value in MANTINE:
                yield node.lineno, node.value
        elif (isinstance(node, ast.keyword) and node.arg and "color" in node.arg.lower()
              and _is_number_tuple(node.value)):
            yield node.lineno, ast.unparse(node.value)
        elif isinstance(node, ast.Assign) and _is_number_tuple(node.value) and any(
                isinstance(t, ast.Name) and "COLOR" in t.id for t in node.targets):
            yield node.lineno, ast.unparse(node.value)


def test_no_colour_literals():
    """Every colour is a name from `colors.py`, so one meaning never gets two shades."""
    found = []
    for path in sorted(PACKAGE.rglob("*.py")):
        relative = path.relative_to(PACKAGE).as_posix()
        text = path.read_text()
        if SKIPPED.match(relative) or any(line.lstrip().startswith(OWN_COLOURS) for line in text.splitlines()):
            continue
        found += [f"{relative}:{line}: {text}" for line, text in _colour_literals(ast.parse(text))]
    assert not found, "colours typed outside colors.py:\n" + "\n".join(found)


#: The closest two hues of `colors.py` may be, CIEDE2000: closer, they read as one colour that means two things.
MIN_HUE_DISTANCE = 12.0


def _lab(color: str) -> tuple[float, float, float]:
    """CIELAB (D65) of a "#rrggbb" colour."""
    linear = [((c / 255) / 12.92 if c / 255 <= 0.04045 else ((c / 255 + 0.055) / 1.055) ** 2.4)
              for c in (int(color[i:i + 2], 16) for i in (1, 3, 5))]
    xyz = [sum(m * c for m, c in zip(row, linear)) / white for row, white in zip(
        ((0.4124, 0.3576, 0.1805), (0.2126, 0.7152, 0.0722), (0.0193, 0.1192, 0.9505)), (0.95047, 1.0, 1.08883))]
    fx, fy, fz = (v ** (1 / 3) if v > 0.008856 else 7.787 * v + 16 / 116 for v in xyz)
    return 116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)


def _de2000(one: str, two: str) -> float:
    """CIEDE2000 colour difference of two "#rrggbb" colours (Sharma, Wu and Dalal 2005)."""
    (l1, a1, b1), (l2, a2, b2) = _lab(one), _lab(two)
    mean_c = (math.hypot(a1, b1) + math.hypot(a2, b2)) / 2
    g = 0.5 * (1 - math.sqrt(mean_c ** 7 / (mean_c ** 7 + 25 ** 7)))
    c1, c2 = math.hypot(a1 * (1 + g), b1), math.hypot(a2 * (1 + g), b2)
    h1, h2 = (math.degrees(math.atan2(b, a * (1 + g))) % 360 for a, b in ((a1, b1), (a2, b2)))
    dh = 0.0 if c1 * c2 == 0 else (h2 - h1 + 180) % 360 - 180
    big_dh = 2 * math.sqrt(c1 * c2) * math.sin(math.radians(dh / 2))
    mean_l, mean_cp = (l1 + l2) / 2, (c1 + c2) / 2
    mean_h = (h1 + h2 + 360) / 2 if abs(h1 - h2) > 180 else (h1 + h2) / 2
    t = (1 - 0.17 * math.cos(math.radians(mean_h - 30)) + 0.24 * math.cos(math.radians(2 * mean_h))
         + 0.32 * math.cos(math.radians(3 * mean_h + 6)) - 0.2 * math.cos(math.radians(4 * mean_h - 63)))
    sl = 1 + 0.015 * (mean_l - 50) ** 2 / math.sqrt(20 + (mean_l - 50) ** 2)
    sc, sh = 1 + 0.045 * mean_cp, 1 + 0.015 * mean_cp * t
    rt = (-math.sin(math.radians(60 * math.exp(-((mean_h - 275) / 25) ** 2)))
          * 2 * math.sqrt(mean_cp ** 7 / (mean_cp ** 7 + 25 ** 7)))
    dl, dc, dhh = (l2 - l1) / sl, (c2 - c1) / sc, big_dh / sh
    return math.sqrt(dl ** 2 + dc ** 2 + dhh ** 2 + rt * dc * dhh)


def test_hues_are_apart():
    """No two hues of `colors.py` are closer than MIN_HUE_DISTANCE: close meanings share a hue or take a far one."""
    from husky_assembly_teleop import colors
    hues = {word: value for value, word in colors.HUE_NAMES.items()}
    close = [f"{a} {b} {_de2000(hues[a], hues[b]):.1f}" for a, b in combinations(sorted(hues), 2)
             if _de2000(hues[a], hues[b]) < MIN_HUE_DISTANCE]
    assert not close, "hues too close (CIEDE2000):\n" + "\n".join(close)


def test_de2000_matches_the_reference_pairs():
    """The CIEDE2000 formula above: zero for one colour, and the widely published 86.6 for sRGB red against green."""
    assert _de2000("#000000", "#000000") == 0
    assert abs(_de2000("#ff0000", "#00ff00") - 86.6) < 0.5
