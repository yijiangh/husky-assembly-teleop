"""Tests for design_io on Python 3.9 (T2), the Python of Rhino 8."""

import ast
import shutil
import subprocess
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DESIGN_IO = PACKAGE_ROOT / "husky_assembly_teleop" / "design_io"
#: Modules that need compas (conversion through legacy); not part of the core that must run on 3.9 alone.
WITH_COMPAS = ("compas_fab", "legacy", "conversion")


def test_syntax_is_python_39():
    """Every design_io file parses as Python 3.9 and postpones annotations."""
    for path in sorted(DESIGN_IO.glob("*.py")):
        source = path.read_text()
        tree = ast.parse(source, str(path), feature_version=(3, 9))
        futures = {alias.name for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == "__future__" for alias in node.names}
        assert "annotations" in futures, f"{path.name} lacks `from __future__ import annotations`"


def test_imports_under_python_39():
    """The core modules import under Python 3.9 with only numpy, scipy and trimesh installed."""
    if shutil.which("uv") is None:
        pytest.skip("uv is not installed")
    modules = sorted(p.stem for p in DESIGN_IO.glob("*.py") if p.stem not in WITH_COMPAS and p.stem != "__init__")
    code = (f"import sys; sys.path.insert(0, {str(PACKAGE_ROOT)!r})\n"
            "import husky_assembly_teleop.design_io\n"
            + "".join(f"import husky_assembly_teleop.design_io.{name}\n" for name in modules)
            + "assert sys.version_info[:2] == (3, 9), sys.version\n"
            "assert 'compas' not in sys.modules\n")
    command = ["uv", "run", "--python", "3.9", "--no-project", "--with", "numpy", "--with", "scipy",
               "--with", "trimesh==4.12.2", "python", "-c", code]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=600, cwd=str(PACKAGE_ROOT))
    except subprocess.TimeoutExpired:
        pytest.skip("uv took too long (offline?)")
    # ? No Traceback means uv itself failed: Python 3.9 not available, or no network to fetch packages.
    if result.returncode != 0 and "Traceback" not in result.stderr:
        pytest.skip(f"Python 3.9 environment unavailable: {result.stderr.strip()[-300:]}")
    assert result.returncode == 0, result.stderr
