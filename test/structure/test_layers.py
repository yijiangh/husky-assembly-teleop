"""The package's import layers: each package imports only from the layers below it, and plugins not from each other.

Imports under `if TYPE_CHECKING:` are only type hints and do not count; imports inside functions do.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest
from repo_paths import REPO

pytestmark = pytest.mark.safety

PACKAGE = "husky_assembly_teleop"
ROOT = REPO / PACKAGE

#: Lowest first. A module may import from its own layer and the ones below.
LAYERS = (
    ("config", "drive", "tool_urdfs", "plots"),
    ("robot_interface",),
    ("world",),
    ("ui",),
    ("plugin_api",),
    ("shared",),
    ("plugins",),
    ("monitor", "experiments", "mocap_relay"),
)
LAYER = {name: rank for rank, names in enumerate(LAYERS) for name in names}
#: Allowed on purpose: (importing module, imported package).
EXCEPTIONS = {
    # * Plugin discovery imports every plugin to run its @register.
    (f"{PACKAGE}.plugin_api.plugin", f"{PACKAGE}.plugins"),
}


def _module_name(path: Path) -> str:
    """The dotted name of a source file, a package's `__init__.py` named after the package."""
    parts = path.relative_to(ROOT.parent).with_suffix("").parts
    return ".".join(parts[:-1] if parts[-1] == "__init__" else parts)


def _is_type_checking(node: ast.If) -> bool:
    """Whether `node` is `if TYPE_CHECKING:` (or `typing.TYPE_CHECKING`)."""
    test = node.test
    return (isinstance(test, ast.Name) and test.id == "TYPE_CHECKING") or \
        (isinstance(test, ast.Attribute) and test.attr == "TYPE_CHECKING")


def _imports(tree: ast.AST):
    """Every import that runs, skipping `if TYPE_CHECKING:` blocks."""
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.If) and _is_type_checking(node):
            for child in node.orelse:
                yield from _imports(child)
            continue
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            yield node
        yield from _imports(node)


def _targets(module: str, is_package: bool, node: ast.Import | ast.ImportFrom) -> list[str]:
    """The absolute names `node` imports, from `module`."""
    if isinstance(node, ast.Import):
        return [alias.name for alias in node.names]
    if node.level == 0:
        return [node.module]
    base = module.split(".") if is_package else module.split(".")[:-1]
    base = base[:len(base) - (node.level - 1)]
    if node.module:
        return [".".join(base + [node.module])]
    return [".".join(base + [alias.name]) for alias in node.names]


def _problem(module: str, target: str) -> str | None:
    """Why `module` may not import `target`, or None if it may."""
    if not target.startswith(f"{PACKAGE}.") or (module, target) in EXCEPTIONS:
        return None
    source, imported = module.split("."), target.split(".")
    if imported[1] not in LAYER:
        return f"{imported[1]} has no layer in LAYERS"
    if LAYER[imported[1]] > LAYER[source[1]]:
        return f"{source[1]} is below {imported[1]}"
    if source[1] == imported[1] == "plugins" and len(source) > 2 and len(imported) > 2:
        mine, theirs = source[2], imported[2]
        if mine != theirs:
            return f"plugin {mine} imports plugin {theirs}: share code through shared/, data through ctx.require"
    return None


def test_every_package_has_a_layer():
    """A new top-level module or package must be put into LAYERS."""
    names = {path.stem for path in ROOT.glob("*.py")} | {path.name for path in ROOT.iterdir() if path.is_dir()}
    missing = names - set(LAYER) - {"__init__", "__pycache__", "old"}
    assert not missing, f"put these into LAYERS: {sorted(missing)}"


def test_imports_go_down_the_layers():
    """No runtime import points up the layers or from one plugin into another."""
    problems = []
    for path in sorted(ROOT.rglob("*.py")):
        if "old" in path.relative_to(ROOT).parts:
            continue
        module = _module_name(path)
        for node in _imports(ast.parse(path.read_text(), str(path))):
            for target in _targets(module, path.name == "__init__.py", node):
                problem = _problem(module, target)
                if problem:
                    problems.append(f"{path.relative_to(ROOT.parent)}:{node.lineno} imports {target}: {problem}")
    assert not problems, "\n".join(problems)
