"""Tests for design_io isolation (T1): it imports nothing of the monitor, and loading it skips compas."""

import ast
import os
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DESIGN_IO = PACKAGE_ROOT / "husky_assembly_teleop" / "design_io"
INSIDE = "husky_assembly_teleop.design_io"


def _outside_imports(path: Path) -> list:
    """Imports of one file that reach husky_assembly_teleop outside design_io (relative ones included)."""
    found = []
    for node in ast.walk(ast.parse(path.read_text(), str(path))):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            names = [node.module or ""]
        elif isinstance(node, ast.ImportFrom):
            # * `from . import x` stays inside design_io; `from .. import x` goes above it.
            if node.level > 1:
                found.append(f"{path.name}:{node.lineno} {'.' * node.level}{node.module or ''}")
            continue
        else:
            continue
        for name in names:
            if name.split(".")[0] == "husky_assembly_teleop" and not (name == INSIDE or name.startswith(INSIDE + ".")):
                found.append(f"{path.name}:{node.lineno} {name}")
    return found


def test_no_imports_from_outside():
    """No module in design_io/ imports husky_assembly_teleop outside design_io."""
    files = sorted(DESIGN_IO.glob("*.py"))
    assert files
    found = [line for path in files for line in _outside_imports(path)]
    assert not found, found


def test_import_skips_compas():
    """`import husky_assembly_teleop.design_io` loads no compas package."""
    code = ("import sys, husky_assembly_teleop.design_io\n"
            "print(sorted(m for m in sys.modules if m.split('.')[0] in ('compas', 'compas_fab', 'compas_robots')))")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(PACKAGE_ROOT), os.environ.get("PYTHONPATH", "")])}
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env, timeout=120)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "[]"
