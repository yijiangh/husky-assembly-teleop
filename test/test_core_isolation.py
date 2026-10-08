"""Tests for the core's isolation: it imports nothing of the monitor, ROS or viser, and `design` skips compas."""

import ast
import os
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CORE = PACKAGE_ROOT / "bar_assembly_core"
#: Top-level packages the core must never import, not even for type checking.
FORBIDDEN = ("husky_assembly_teleop", "rclpy", "viser", "crl_husky")


def _forbidden_imports(path: Path) -> list:
    """Imports of one core file that reach a FORBIDDEN package, or climb out of the core with dots."""
    package = path.relative_to(PACKAGE_ROOT).parent.parts  # e.g. ("bar_assembly_core", "mirrors")
    found = []
    for node in ast.walk(ast.parse(path.read_text(), str(path))):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names = [node.module or ""]
        elif isinstance(node, ast.ImportFrom):
            # * `from .. import x` in bar_assembly_core/mirrors/ stays inside; one more dot leaves the core.
            if node.level > len(package):
                found.append(f"{path.name}:{node.lineno} {'.' * node.level}{node.module or ''}")
            continue
        else:
            continue
        found += [f"{path.name}:{node.lineno} {name}" for name in names if name.split(".")[0] in FORBIDDEN]
    return found


def test_no_forbidden_imports():
    """No core module imports the monitor, ROS or viser."""
    files = sorted(CORE.rglob("*.py"))
    assert files
    found = [line for path in files for line in _forbidden_imports(path)]
    assert not found, found


def test_import_skips_compas():
    """`import bar_assembly_core.design` loads no compas package."""
    code = ("import sys, bar_assembly_core.design\n"
            "print(sorted(m for m in sys.modules if m.split('.')[0] in ('compas', 'compas_fab', 'compas_robots')))")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(PACKAGE_ROOT), os.environ.get("PYTHONPATH", "")])}
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, timeout=120)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "[]"


def test_no_core_module_reads_notes():
    """No core module reads a note key: notes are for people (the legacy converter reads the old export's only)."""
    found = []
    for path in sorted(CORE.rglob("*.py")):
        if "legacy" in path.parts:
            continue
        for node in ast.walk(ast.parse(path.read_text(), str(path))):
            reads = (isinstance(node, ast.Subscript) and isinstance(node.value, ast.Attribute)
                     and node.value.attr == "notes")
            reads |= (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                      and node.func.attr in ("get", "pop", "__getitem__") and isinstance(node.func.value, ast.Attribute)
                      and node.func.value.attr == "notes")
            if reads:
                found.append(f"{path.relative_to(CORE)}:{node.lineno}")
    assert not found, found
