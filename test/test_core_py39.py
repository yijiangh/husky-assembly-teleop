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
PURE = ("bar_assembly_core", "bar_assembly_core.geometry", "bar_assembly_core.ids", "bar_assembly_core.urdf",
        "bar_assembly_core.kinematics", "bar_assembly_core.robot", "bar_assembly_core.scene",
        "bar_assembly_core.design", "bar_assembly_core.legacy", "bar_assembly_core.legacy.timing",
        *(f"bar_assembly_core.design.{p.stem}" for p in sorted((CORE / "design").glob("*.py")) if p.stem != "__init__"))
#: Modules with extra dependencies (`requirements.txt`). legacy.export and legacy.conversion also need
#: rs_data_structure.
EXTRAS = ("bar_assembly_core.mirrors",
          "bar_assembly_core.mirrors.compas", "bar_assembly_core.mirrors.pp_client",
          "bar_assembly_core.mirrors.pybullet",
          "bar_assembly_core.mirrors.compas_fab")
CORE_PACKAGES = ("numpy", "scipy", "trimesh==4.12.2")
EXTRA_PACKAGES = ("pybullet", "compas_robots", "./external/compas_fab", "./external/pybullet_planning")


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
    """The pure core imports and runs under Python 3.9 with only numpy, scipy and trimesh, without compas.

    It writes, reads and plan-checks the test design, and builds a scene of it.
    """
    run = (f"import tempfile, pathlib; sys.path.insert(0, {str(PACKAGE_ROOT / 'test')!r})\n"
           "from design_fixtures import build_design\n"
           "from bar_assembly_core.design import write\n"
           "from bar_assembly_core.design.plan_check import check_plan\n"
           "folder = pathlib.Path(tempfile.mkdtemp())\n"
           "design = write(build_design(folder), folder / 'out')\n"
           "assert check_plan(design).ok\n"
           "scene = design.scene_at(next(movement for _, movement in design.movements()))\n"
           "assert 'bars/B2' in scene.world_poses and scene.robots['robots/cindy'].unmeasured\n"
           "assert 'compas' not in sys.modules and 'yourdfpy' not in sys.modules\n")
    _run_39(CORE_PACKAGES, PURE, run)


def test_mirrors_import_under_python_39():
    """The mirrors and the compas conversions import under Python 3.9 with their extras."""
    if not all((PACKAGE_ROOT / package).is_dir() for package in EXTRA_PACKAGES if package.startswith("./")):
        pytest.skip("external/ submodules not checked out")
    _run_39(CORE_PACKAGES + EXTRA_PACKAGES, PURE + EXTRAS)
