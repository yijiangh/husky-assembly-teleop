"""Size on disk, export cell composition and the design round trip, for one old compas_fab export.

    python scripts/design_io_storage.py <export folder> [--json results.json]

* Converts into a temporary folder; the export and its `<export>_design` copy are not touched.
* Loader timing and the loader equality check are in `design_io_equivalence.py`.
! Parsing each export cell takes ~2 s and a few GB of memory.
"""

from __future__ import annotations

import json
import subprocess
import sys
from argparse import ArgumentParser
from collections import defaultdict
from pathlib import Path
from tempfile import TemporaryDirectory

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "test"))
DATA = REPO / "data"


def quiet(_text: str) -> None:
    """Swallow progress messages."""


# --- --- --- --- --- STEPS (each runs in its own process) --- --- --- --- ---

def step_convert(export: str, design: str) -> dict:
    """Convert the export into a design folder."""
    from husky_assembly_teleop.design_io.conversion import convert_export
    convert_export(Path(export), Path(design), DATA, quiet)
    return {}


def step_roundtrip(design: str, tmp: str) -> dict:
    """Validate, then read -> write -> read twice: equal designs, same files, same bytes."""
    from test_design_io_roundtrip import _comparable

    from husky_assembly_teleop.design_io import read, validate, write
    first = read(Path(design))
    a, b = Path(tmp) / "a", Path(tmp) / "b"
    second = write(first, a, package_dirs=[DATA / "husky_urdf"])
    third = write(read(a), b, package_dirs=[DATA / "husky_urdf"])
    files_a = sorted(p.relative_to(a).as_posix() for p in a.rglob("*") if p.is_file())
    files_b = sorted(p.relative_to(b).as_posix() for p in b.rglob("*") if p.is_file())
    return {
        "validate_problems": len(validate(first) or []),
        "read_write_read_equal": _comparable(read(a)) == _comparable(first),
        "second_cycle_equal": _comparable(third) == _comparable(second),
        "files": len(files_a),
        "same_file_list": files_a == files_b,
        "byte_differing_files": [f for f in files_a if f in files_b and (a / f).read_bytes() != (b / f).read_bytes()],
        "counts": {"robots": len(first.robots), "tools": len(first.tools), "bodies": len(first.bodies),
                   "actions": len(first.actions),
                   "movements": sum(len(action.movements) for action in first.actions.values())},
    }


def step_composition(cell_file: str) -> dict:
    """Bytes of one export cell file by part, re-serialized as indented in the file (4 spaces)."""
    def indented(value, depth: int) -> int:
        text = json.dumps(value, indent=4)
        return len(text) + text.count("\n") * depth * 4

    def meshes(links: list, kind: str, depth: int) -> int:
        return sum(indented(link.get("data", link).get(kind, []), depth + 3) for link in links)

    path = Path(cell_file)
    raw = path.read_bytes()
    parsed = json.loads(raw)
    data = parsed["data"]
    parts = {}
    for key, depth, model in ([("robot_model", 2, data["robot_model"])]
                              + [(f"tool_models.{name}", 3, tool) for name, tool in data["tool_models"].items()]):
        links = model.get("data", model)["links"]
        parts[key] = {"bytes": indented(model, depth), "visual": meshes(links, "visual", depth),
                      "collision": meshes(links, "collision", depth)}
    for key in ("rigid_body_models", "robot_semantics"):
        parts[key] = {"bytes": indented(data[key], 2)}
    return {"file": path.name, "bytes": len(raw), "carriage_returns": raw.count(b"\r"),
            "compact_bytes": len(json.dumps(parsed, separators=(",", ":"))), "parts": parts}


# --- --- --- --- --- RUNNING AND REPORTING --- --- --- --- ---

def run_step(name: str, *args: str) -> dict:
    """Run one step in a fresh Python process and return its result."""
    done = subprocess.run([sys.executable, "-W", "ignore", __file__, "--step", name, *args],
                          capture_output=True, text=True)
    lines = [line for line in done.stdout.splitlines() if line.startswith("RESULT ")]
    if done.returncode != 0 or not lines:
        raise RuntimeError(f"step {name} failed:\n{done.stderr[-3000:]}")
    return json.loads(lines[-1][len("RESULT "):])


def sizes(folder: Path) -> dict:
    """Bytes per group of files: top-level files alone, repeated files folded by folder and suffix."""
    groups = defaultdict(list)
    for path in sorted(folder.rglob("*")):
        if not path.is_file():
            continue
        parts = path.relative_to(folder).parts
        if len(parts) == 1:
            key = parts[0]
        elif parts[0] == "robots":
            key = f"robots/{parts[1]}/" + ("meshes/**" if parts[2] == "meshes" else parts[2])
        else:
            key = "/".join(parts[:-1]) + "/*" + path.suffix
        groups[key].append(path.stat().st_size)
    return {"total": sum(map(sum, groups.values())), "files": sum(map(len, groups.values())),
            "groups": {key: {"count": len(v), "bytes": sum(v)} for key, v in groups.items()}}


def print_report(result: dict) -> None:
    """Print the measurements as plain tables."""
    print(f"\n{result['export']}")
    for name in ("export", "design"):
        folder = result["sizes"][name]
        print(f"\n{name}: {folder['total'] / 1e6:.1f} MB in {folder['files']} files")
        for key, group in sorted(folder["groups"].items(), key=lambda item: -item[1]["bytes"]):
            mean = group["bytes"] / group["count"] / 1e3
            print(f"   {key:48}{group['bytes'] / 1e6:10.3f} MB   {group['count']:3} × {mean:9.1f} kB")
    for cell in result["composition"]:
        print(f"\n{cell['file']}: {cell['bytes'] / 1e6:.1f} MB on disk, {cell['compact_bytes'] / 1e6:.1f} MB without "
              f"whitespace, {cell['carriage_returns'] / 1e6:.2f} MB of CR")
        for key, part in cell["parts"].items():
            split = (f"   (visual {part['visual'] / 1e6:.1f}, collision {part['collision'] / 1e6:.1f})"
                     if "visual" in part else "")
            print(f"   {key:40}{part['bytes'] / 1e6:9.1f} MB{split}")
    print(f"\nRound trip: {json.dumps(result['roundtrip'], indent=2)}")


def main() -> None:
    """Run every measurement on the export given and print (and optionally save) the results."""
    parser = ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("export", type=Path, help="folder with ActionSchedule.json and RobotCell*.json")
    parser.add_argument("--json", type=Path, help="also write all results to this file")
    options = parser.parse_args()
    export = options.export.expanduser().resolve()

    result = {"export": export.name}
    with TemporaryDirectory() as tmp:
        design = Path(tmp) / "design"
        run_step("convert", str(export), str(design))
        result["sizes"] = {"export": sizes(export), "design": sizes(design)}
        result["composition"] = [run_step("composition", str(cell)) for cell in sorted(export.glob("RobotCell*.json"))]
        (Path(tmp) / "roundtrip").mkdir()
        result["roundtrip"] = run_step("roundtrip", str(design), str(Path(tmp) / "roundtrip"))

    print_report(result)
    if options.json:
        options.json.write_text(json.dumps(result, indent=1))
        print(f"\nwritten {options.json}")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--step":
        print("RESULT " + json.dumps(globals()[f"step_{sys.argv[2]}"](*sys.argv[3:])))
    else:
        main()
