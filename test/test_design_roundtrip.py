"""Tests for design write and read (T3): a design written and read back is the same design."""

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from design_fixtures import TETRA, build_design, write_robot_files

from bar_assembly_core.design import BodySpec, Design, content_hash, read, write
from bar_assembly_core.geometry import Geometry, Pose, TriMesh
from bar_assembly_core.design.meshes import read_mesh, write_mesh
from bar_assembly_core.urdf import (copy_robot, mesh_references, movable_joints, resolved_urdf_text, srdf_group_tips,
                                    urdf_joints, urdf_links)


def _shapes(shapes) -> list:
    """Shapes as comparable values: meshes by their arrays, primitives as they are."""
    return [("mesh", s.vertices.tolist(), s.faces.tolist()) if isinstance(s, TriMesh) else s for s in shapes]


def _geometry(geometry: Geometry) -> tuple:
    """A geometry as comparable values."""
    return _shapes(geometry.visual), _shapes(geometry.collision)


def _comparable(design: Design) -> dict:
    """Everything of a design except its folder, file paths, writer and mesh object identity."""
    return {
        "robots": {k: (r.id, r.serial, r.tools, r.ground_links) for k, r in design.robots.items()},
        "tools": {k: (t.id, _geometry(t.geometry), t.tcp, t.kind, t.mount_contacts) for k, t in design.tools.items()},
        "bodies": {k: (b.id, b.pose, _geometry(b.geometry), b.label, b.part, b.markers)
                   for k, b in design.bodies.items()},
        "connections": design.connections,
        "producer": design.producer,
        "schedule": design.schedule,
        "actions": design.actions,
    }


def test_write_then_read_is_equal(tmp_path: Path):
    """read(write(d)) equals d, down to every state, tool state, target, line and note."""
    design = build_design(tmp_path)
    back = write(design, tmp_path / "out")
    assert back.folder == (tmp_path / "out").resolve()
    assert _comparable(back) == _comparable(design)
    assert _comparable(read(tmp_path / "out")) == _comparable(design)
    # Robot files now live in the design, with absolute paths.
    for robot in back.robots.values():
        assert robot.urdf == (tmp_path / "out" / "robots" / robot.name / "robot.urdf").resolve()
        assert robot.urdf.is_absolute() and robot.srdf.is_file()


def test_shared_mesh_written_once(tmp_path: Path):
    """Two bodies with one mesh share one file, and read back share one TriMesh object."""
    design = build_design(tmp_path)
    # * A copy with equal content is also written only once.
    copy = TriMesh.from_arrays(TETRA.vertices, TETRA.faces)
    bodies = {**design.bodies, "obstacles/O1": BodySpec("obstacles/O1", design.bodies["bars/B2"].pose,
                                                        Geometry((copy,), (copy,)))}
    back = write(replace(design, bodies=bodies), tmp_path / "out")
    files = sorted(p.relative_to(tmp_path / "out" / "meshes").as_posix()
                   for p in (tmp_path / "out" / "meshes").rglob("*") if p.is_file())
    assert files == ["joints/J1_male.obj", "tools/AT3L.obj"]
    manifest = json.loads((tmp_path / "out" / "design.json").read_text())
    for body in ("joints/J1_male", "joints/J2_male", "obstacles/O1"):
        assert manifest["bodies"][body]["collision"] == [{"mesh": "meshes/joints/J1_male.obj"}]
    meshes = [back.bodies[b].geometry.collision[0] for b in ("joints/J1_male", "joints/J2_male", "obstacles/O1")]
    assert meshes[0] is meshes[1] is meshes[2]
    assert back.bodies["joints/J1_male"].geometry.visual[0] is meshes[0]


def test_visual_left_out_when_equal(tmp_path: Path):
    """`visual` is written only when it differs from `collision`; default keys are left out."""
    write(build_design(tmp_path), tmp_path / "out")
    manifest = json.loads((tmp_path / "out" / "design.json").read_text())
    assert "visual" not in manifest["bodies"]["bars/B2"]
    assert len(manifest["bodies"]["bars/B1"]["visual"]) == 2
    assert "mount_contacts" not in manifest["tools"]["tools/Grip"] and "label" not in manifest["bodies"]["bars/B2"]
    assert "serial" not in manifest["robots"]["robots/alice"] and "part" not in manifest["bodies"]["bars/B2"]
    action = json.loads((tmp_path / "out" / "actions" / "B2_H_hold.json").read_text())
    movement = action["movements"][0]
    assert "coupled" not in movement and "ends_on" not in movement and "notes" not in movement
    assert "carried" not in movement["start"] and "line" not in movement
    assert movement["target"] == {"joints": {"robots/alice": {"left_joint1": 1.0, "left_joint2": 1.0,
                                                              "right_joint1": 1.0, "right_joint2": 1.0}}}
    close = action["movements"][1]
    assert "arms" not in close and "path" not in close and "controller" not in close
    assert close["target"] == {"tools": {"tools/Grip": {"grip": "closed"}}}


def test_writer_block_in_every_file(tmp_path: Path):
    """design.json and each action file carry the writer block of this library."""
    write(build_design(tmp_path), tmp_path / "out")
    files = [tmp_path / "out" / "design.json", *sorted((tmp_path / "out" / "actions").glob("*.json"))]
    assert len(files) == 3
    for path in files:
        writer = json.loads(path.read_text())["writer"]
        assert writer["schema"] == 2 and writer["library"] == "design_io"
        assert isinstance(writer["commit"], str) and isinstance(writer["dirty"], bool)


def test_poses_on_one_line(tmp_path: Path):
    """Poses and joint vectors stay on one line; no NaN."""
    write(build_design(tmp_path), tmp_path / "out")
    text = (tmp_path / "out" / "actions" / "B1_J_joint.json").read_text()
    assert '"offset": [0.0, 0.0, 0.12, 0.0, 0.0, 0.0, 1.0]' in text
    assert "NaN" not in text


def test_content_hash_ignores_writer_and_formatting(tmp_path: Path):
    """The hash covers content only: the same for another writer, indentation or float noise; not for a change."""
    write(build_design(tmp_path), tmp_path / "out")
    path = tmp_path / "out" / "actions" / "B1_J_joint.json"
    before = content_hash(path)
    data = json.loads(path.read_text())
    data["writer"] = {"schema": 2, "library": "design_io", "commit": "elsewhere", "dirty": True}
    data["movements"][2]["line"]["robots/cindy/left_tool0"]["distance"] = 0.015 + 1e-15
    path.write_text(json.dumps(data))
    assert content_hash(path) == before
    data["movements"][2]["line"]["robots/cindy/left_tool0"]["distance"] = 0.02
    path.write_text(json.dumps(data))
    assert content_hash(path) != before


def test_non_empty_folder_refused(tmp_path: Path):
    """Writing into a folder with files needs overwrite=True."""
    (tmp_path / "out").mkdir()
    (tmp_path / "out" / "something.txt").write_text("x")
    with pytest.raises(FileExistsError):
        write(build_design(tmp_path), tmp_path / "out")


def test_overwrite_removes_stale_files(tmp_path: Path):
    """With overwrite, action files and meshes from an earlier write are gone."""
    design = build_design(tmp_path)
    write(design, tmp_path / "out")
    (tmp_path / "out" / "actions" / "OLD.json").write_text("{}")
    (tmp_path / "out" / "meshes" / "old.obj").write_text("")
    (tmp_path / "out" / "solutions").mkdir()
    back = write(design, tmp_path / "out", overwrite=True)
    assert not (tmp_path / "out" / "actions" / "OLD.json").exists()
    assert not (tmp_path / "out" / "meshes" / "old.obj").exists()
    assert (tmp_path / "out" / "solutions").is_dir()
    assert _comparable(back) == _comparable(design)


def test_rewrite_into_own_folder(tmp_path: Path):
    """A design read from a folder can be written back into it: its robot files are kept in place."""
    back = write(build_design(tmp_path), tmp_path / "out")
    again = write(back, tmp_path / "out", overwrite=True)
    assert again.robots["robots/cindy"].urdf == back.robots["robots/cindy"].urdf
    assert _comparable(again) == _comparable(back)


# --- --- --- --- --- ROBOT FILES --- --- --- --- ---

def test_robot_file_queries(tmp_path: Path):
    """Links, joints, movable joints (no fixed, passive or mimic joints) and SRDF group tips."""
    urdf, srdf = write_robot_files(tmp_path)
    text = urdf.read_text().replace("</robot>", """  <link name="finger"/>
  <joint name="finger_joint" type="prismatic">
    <parent link="left_tool0"/><child link="finger"/><mimic joint="left_joint1"/>
  </joint>
</robot>""")
    urdf.write_text(text)
    assert {"base_footprint", "left_tool0", "right_tool0", "finger"} <= urdf_links(urdf)
    assert urdf_joints(urdf)["wheel_joint"] == "continuous"
    assert movable_joints(urdf, srdf) == ("left_joint1", "left_joint2", "right_joint1", "right_joint2")
    assert srdf_group_tips(srdf) == {"left_arm": "left_tool0", "right_arm": "right_tool0"}
    assert mesh_references(urdf) == ["meshes/base.obj", "meshes/base.obj"]


def test_copy_robot_resolves_packages(tmp_path: Path):
    """package:// meshes are found in package dirs and copied below meshes/<package>/."""
    package = tmp_path / "ws" / "husky_description"
    (package / "meshes").mkdir(parents=True)
    (package / "meshes" / "base_link.stl").write_bytes(b"solid x\nendsolid x\n")
    urdf, srdf = write_robot_files(tmp_path / "src", mesh="package://husky_description/meshes/base_link.stl",
                                   write_mesh_file=False)
    for package_dirs in ([tmp_path / "ws"], [package]):
        new_urdf, new_srdf = copy_robot(urdf, srdf, tmp_path / "dest", package_dirs)
        assert mesh_references(new_urdf) == ["meshes/husky_description/meshes/base_link.stl"] * 2
        assert (tmp_path / "dest" / "meshes" / "husky_description" / "meshes" / "base_link.stl").is_file()
        assert new_srdf.read_text() == srdf.read_text()
    with pytest.raises(FileNotFoundError, match="husky_description"):
        copy_robot(urdf, srdf, tmp_path / "dest2", [])


def test_copy_robot_twice_keeps_mesh_paths(tmp_path: Path):
    """Copying a design's own robot again keeps `meshes/<path>`; it does not nest a second `meshes/`."""
    package = tmp_path / "ws" / "husky_description"
    (package / "meshes").mkdir(parents=True)
    (package / "meshes" / "base_link.stl").write_bytes(b"solid x\nendsolid x\n")
    urdf, srdf = write_robot_files(tmp_path / "src", mesh="package://husky_description/meshes/base_link.stl",
                                   write_mesh_file=False)
    first, first_srdf = copy_robot(urdf, srdf, tmp_path / "a", [tmp_path / "ws"])
    second, _ = copy_robot(first, first_srdf, tmp_path / "b")
    assert mesh_references(second) == mesh_references(first) == ["meshes/husky_description/meshes/base_link.stl"] * 2
    assert second.read_bytes() == first.read_bytes()


def test_resolved_urdf_text(tmp_path: Path):
    """Relative mesh paths become absolute against the URDF's folder."""
    urdf, _ = write_robot_files(tmp_path)
    text = resolved_urdf_text(urdf)
    assert f'filename="{(tmp_path / "meshes" / "base.obj").resolve()}"' in text


def test_mesh_file_round_trip(tmp_path: Path):
    """An OBJ written and read keeps exact vertices and faces."""
    mesh = TriMesh.from_arrays(TETRA.vertices + 0.1, TETRA.faces)
    write_mesh(mesh, tmp_path / "a" / "m.obj")
    back = read_mesh(tmp_path / "a" / "m.obj")
    np.testing.assert_array_equal(back.vertices, mesh.vertices)
    np.testing.assert_array_equal(back.faces, mesh.faces)


def test_write_copies_package_meshes(tmp_path: Path):
    """A robot whose source URDF names meshes by package:// is written; the copy uses relative paths."""
    package = tmp_path / "ws" / "husky_description"
    (package / "meshes").mkdir(parents=True)
    (package / "meshes" / "base_link.stl").write_bytes(b"solid x\nendsolid x\n")
    urdf, srdf = write_robot_files(tmp_path / "src", mesh="package://husky_description/meshes/base_link.stl",
                                   write_mesh_file=False)
    design = build_design(tmp_path)
    robots = {key: replace(robot, urdf=urdf, srdf=srdf) for key, robot in design.robots.items()}
    back = write(replace(design, robots=robots), tmp_path / "out", package_dirs=[tmp_path / "ws"])
    assert mesh_references(back.robots["robots/cindy"].urdf) == ["meshes/husky_description/meshes/base_link.stl"] * 2


def test_floats_are_rounded(tmp_path: Path):
    """Written floats keep 12 decimals: numerical noise becomes 0.0, and -0.0 becomes 0.0."""
    design = build_design(tmp_path)
    noisy = Pose((0.5, -3.4e-20, -0.0), (0.0, 0.0, 0.0, 1.0))
    bodies = {**design.bodies, "bars/B2": replace(design.bodies["bars/B2"], pose=noisy)}
    back = write(replace(design, bodies=bodies), tmp_path / "out")
    assert back.bodies["bars/B2"].pose.position == (0.5, 0.0, 0.0)
    manifest = (tmp_path / "out" / "design.json").read_text()
    assert "e-20" not in manifest and "-0.0" not in manifest
