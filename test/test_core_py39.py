"""Tests for the core on Python 3.9 (Rhino 8), in an environment without ROS: syntax, and imports via uv."""

import ast
import os
import shutil
import subprocess
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CORE = PACKAGE_ROOT / "bar_assembly_core"
#: Pure-core modules: they must import with numpy, scipy and trimesh alone.
PURE = ("bar_assembly_core", "bar_assembly_core.design_io", "bar_assembly_core.scene", "bar_assembly_core.ur",
        *(f"bar_assembly_core.design_io.{p.stem}" for p in sorted((CORE / "design_io").glob("*.py"))
          if p.stem not in ("__init__", "compas_fab", "legacy", "conversion")))
#: Modules with extra dependencies (`requirements.txt`). legacy and conversion also need rs_data_structure.
EXTRAS = ("bar_assembly_core.kinematics", "bar_assembly_core.mirrors", "bar_assembly_core.mirrors.compas_convert",
          "bar_assembly_core.mirrors.pybullet", "bar_assembly_core.mirrors.compas_fab",
          "bar_assembly_core.design_io.compas_fab")
CORE_PACKAGES = ("numpy", "scipy", "trimesh==4.12.2")
EXTRA_PACKAGES = ("yourdfpy", "pybullet", "compas_robots", "./external/compas_fab", "./external/pybullet_planning")


def test_syntax_is_python_39():
    """Every core file parses as Python 3.9 and postpones annotations."""
    for path in sorted(CORE.rglob("*.py")):
        source = path.read_text()
        tree = ast.parse(source, str(path), feature_version=(3, 9))
        futures = {alias.name for node in tree.body if isinstance(node, ast.ImportFrom)
                   and node.module == "__future__" for alias in node.names}
        assert "annotations" in futures, f"{path.relative_to(CORE)} lacks `from __future__ import annotations`"


def _run_39(packages: tuple, modules: tuple, extra_check: str = "") -> None:
    """Import `modules` under Python 3.9 with only `packages` and no PYTHONPATH (so no ROS); skip without uv."""
    if shutil.which("uv") is None:
        pytest.skip("uv is not installed")
    code = (f"import sys; sys.path.insert(0, {str(PACKAGE_ROOT)!r})\n"
            + "".join(f"import {name}\n" for name in modules)
            + "assert sys.version_info[:2] == (3, 9), sys.version\n"
            "assert 'rclpy' not in sys.modules and 'husky_assembly_teleop' not in sys.modules\n" + extra_check)
    command = ["uv", "run", "--python", "3.9", "--no-project",
               *(arg for package in packages for arg in ("--with", package)), "python", "-c", code]
    env = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=900, cwd=str(PACKAGE_ROOT), env=env)
    except subprocess.TimeoutExpired:
        pytest.skip("uv took too long (offline?)")
    # ? No Traceback means uv itself failed: Python 3.9 not available, or no network to fetch packages.
    if result.returncode != 0 and "Traceback" not in result.stderr:
        pytest.skip(f"Python 3.9 environment unavailable: {result.stderr.strip()[-300:]}")
    assert result.returncode == 0, result.stderr


def test_core_imports_under_python_39():
    """The pure core imports under Python 3.9 with only numpy, scipy and trimesh, and loads no compas."""
    _run_39(CORE_PACKAGES, PURE, "assert 'compas' not in sys.modules\n")


def test_mirrors_import_under_python_39():
    """FK, the mirrors and the compas conversions import under Python 3.9 with their extras installed."""
    if not all((PACKAGE_ROOT / package).is_dir() for package in EXTRA_PACKAGES if package.startswith("./")):
        pytest.skip("external/ submodules not checked out")
    _run_39(CORE_PACKAGES + EXTRA_PACKAGES, PURE + EXTRAS)
