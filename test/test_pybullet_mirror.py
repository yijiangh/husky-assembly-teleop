"""Tests for mirrors.pybullet.PyBulletMirror: snapshots in, a PyBullet world that follows them."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_planning as pp
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from bar_assembly_core.geometry import BoxShape, CylinderShape, Geometry, TriMesh, box_geometry, shape_mesh
from bar_assembly_core.mirrors.pybullet import PyBulletMirror
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import Attachment, Body, Scene
from bar_assembly_core.geometry import Pose

DATA = Path(__file__).resolve().parent.parent / "data"
SERIAL = "0804"
#: The robot's id in scenes.
ROBOT = "robots/a200-0804"


@pytest.fixture(scope="module")
def config():
    """The single-arm robot's stitched config, loaded once."""
    return robot_config_from_serial(SERIAL, DATA)


@pytest.fixture
def mirror():
    """A fresh mirror, closed after the test."""
    mirror = PyBulletMirror()
    yield mirror
    mirror.close()


@pytest.fixture
def resets(monkeypatch) -> list[int]:
    """Count PyBullet base resets: returns the list of body ids reset, in order."""
    calls: list[int] = []
    original = p.resetBasePositionAndOrientation

    def counting(body, *args, **kwargs):
        calls.append(body)
        return original(body, *args, **kwargs)

    monkeypatch.setattr(p, "resetBasePositionAndOrientation", counting)
    return calls


def snapshot(bodies: tuple[Body, ...] = (), robots: dict[str, RobotObject] | None = None) -> Scene:
    """Build a snapshot from bodies, resolving an attachment to the robot's base as the grasp pose.

    Args:
        bodies: Scene bodies; an attached body's world pose is taken to be its grasp.
        robots: Robots by id.

    Returns:
        Scene: The world to sync.
    """
    poses = {body.id: body.placement if isinstance(body.placement, Pose) else body.placement.offset
             for body in bodies}
    return Scene(bodies={body.id: body for body in bodies}, world_poses=poses, robots=robots or {})


def robot_entry(config, joints: dict[str, float] | None = None, base: Pose = Pose(), object_id: str = ROBOT,
                **flags) -> RobotObject:
    """A measured robot standing at `base` with the given joints; `flags` set validity fields."""
    return RobotObject(object_id, config.model, base, dict(joints or {}), **flags)


def two_boxes() -> Geometry:
    """Two 0.2 m cubes 1 m apart along x, merged into one non-convex mesh."""
    mesh = shape_mesh(BoxShape((0.2, 0.2, 0.2)))
    vertices = np.vstack([mesh.vertices - (0.5, 0, 0), mesh.vertices + (0.5, 0, 0)])
    faces = np.vstack([mesh.faces, mesh.faces + len(mesh.vertices)])
    merged = TriMesh.from_arrays(vertices, faces)
    assert not merged.convex
    return Geometry((merged,), (merged,))


def touching(mirror: PyBulletMirror, a: str, b: str) -> bool:
    """Whether any PyBullet body of `a` touches any of `b`."""
    return any(p.getClosestPoints(body_a, body_b, 0.0, physicsClientId=mirror.client_id)
               for body_a in mirror.body_ids(a) for body_b in mirror.body_ids(b))


def test_add_move_remove_and_reused_ids(mirror, resets):
    """Bodies are added, moved only on a real pose change, removed, and a reused PyBullet id maps to the new one."""
    geometry = box_geometry((1.0, 1.0, 1.0))
    mirror.sync(snapshot((Body("t/a", geometry, Pose((1.0, 0.0, 0.0))),)))
    (old,) = mirror.body_ids("t/a")
    assert mirror.id_of(old) == "t/a" and mirror.obstacle_ids() == ["t/a"]

    # An equal pose object (not the same one) is no change.
    resets.clear()
    mirror.sync(snapshot((Body("t/a", geometry, Pose((1.0, 0.0, 0.0))),)))
    assert resets == []
    mirror.sync(snapshot((Body("t/a", geometry, Pose((2.0, 0.0, 0.0))),)))
    assert resets == [old] and mirror.body_ids("t/a") == [old]
    assert p.getBasePositionAndOrientation(old, physicsClientId=mirror.client_id)[0] == (2.0, 0.0, 0.0)

    mirror.sync(snapshot())
    assert mirror.obstacle_ids() == []
    with pytest.raises(KeyError):
        mirror.id_of(old)

    mirror.sync(snapshot((Body("t/b", geometry, Pose()),)))
    assert mirror.body_ids("t/b") == [old], "PyBullet is expected to reuse the freed id"
    assert mirror.id_of(old) == "t/b"


def test_new_geometry_rebuilds(mirror, resets):
    """A new Geometry object rebuilds the body; the same one with a new pose only moves it."""
    small, large = box_geometry((0.1, 0.1, 0.1)), box_geometry((1.0, 1.0, 1.0))
    mirror.sync(snapshot((Body("t/a", small, Pose()),)))
    mirror.sync(snapshot((Body("t/a", small, Pose((0.0, 0.0, 1.0))),)))
    (body,) = mirror.body_ids("t/a")
    low, high = p.getAABB(body, physicsClientId=mirror.client_id)
    assert high[0] - low[0] == pytest.approx(0.1, abs=0.01)

    resets.clear()
    mirror.sync(snapshot((Body("t/a", large, Pose((0.0, 0.0, 1.0))),)))
    (body,) = mirror.body_ids("t/a")
    low, high = p.getAABB(body, physicsClientId=mirror.client_id)
    assert high[0] - low[0] == pytest.approx(1.0, abs=0.01)
    assert low[2] == pytest.approx(0.5, abs=0.01)
    assert resets == [], "a rebuilt body is created at its pose, not moved"


def test_concave_only_while_free(mirror, config):
    """A free non-convex mesh is concave (a probe in its gap is clear); attached or tracked, it is its convex hull."""
    probe = Body("t/probe", box_geometry((0.1, 0.1, 0.1)), Attachment(ROBOT, None, Pose((0.0, 0.0, 5.0))))
    pair = two_boxes()
    robots = {ROBOT: robot_entry(config)}
    mirror.sync(snapshot((Body("t/pair", pair, Pose((0.0, 0.0, 5.0))), probe), robots))
    assert not touching(mirror, "t/pair", "t/probe")

    mirror.sync(snapshot((Body("t/pair", pair, Attachment(ROBOT, None, Pose((0.0, 0.0, 5.0)))), probe), robots))
    assert touching(mirror, "t/pair", "t/probe")

    mirror.sync(snapshot((Body("t/pair", pair, Pose((0.0, 0.0, 5.0))), probe), robots))
    assert not touching(mirror, "t/pair", "t/probe")

    # * A measured object moves every tick: never concave, even standing free.
    mirror.sync(snapshot((Body("tracked/pair", pair, Pose((0.0, 0.0, 5.0))), probe), robots))
    assert touching(mirror, "tracked/pair", "t/probe")


def test_shape_cache(mirror):
    """Equal primitives share one collision shape, meshes are shared by object; unused shapes are dropped."""
    geometry = box_geometry((0.5, 0.5, 0.5))
    mirror.sync(snapshot((Body("t/a", geometry, Pose()), Body("t/b", geometry, Pose((3.0, 0.0, 0.0))))))
    assert len(mirror._shapes) == 1
    (shape_id,) = mirror._shapes.values()
    # * An equal box in a new Geometry is rebuilt as a body, but reuses the shape.
    mirror.sync(snapshot((Body("t/b", box_geometry((0.5, 0.5, 0.5)), Pose()),)))
    assert list(mirror._shapes.values()) == [shape_id]
    pair = two_boxes()
    mirror.sync(snapshot((Body("t/b", box_geometry((0.6, 0.6, 0.6)), Pose()), Body("t/pair", pair, Pose()))))
    assert len(mirror._shapes) == 2 and shape_id not in mirror._shapes.values()
    assert (pair.collision[0], True) in mirror._shapes
    mirror.sync(snapshot())
    assert mirror._shapes == {}


def test_primitives_are_exact_and_offset(mirror):
    """A box and a cylinder are exact PyBullet primitives, placed at their origin inside the body."""
    shifted = BoxShape((0.2, 0.2, 0.2), origin=Pose((1.0, 0.0, 0.0)))
    tall = CylinderShape(0.1, 2.0)
    mirror.sync(snapshot((Body("t/box", Geometry((shifted,), (shifted,)), Pose((0.0, 0.0, 5.0))),
                          Body("t/cyl", Geometry((tall,), (tall,)), Pose((5.0, 0.0, 0.0))))))
    low, high = p.getAABB(mirror.body_ids("t/box")[0], physicsClientId=mirror.client_id)
    np.testing.assert_allclose((np.array(low) + high) / 2, (1.0, 0.0, 5.0), atol=0.02)
    low, high = p.getAABB(mirror.body_ids("t/cyl")[0], physicsClientId=mirror.client_id)
    assert high[2] - low[2] == pytest.approx(2.0, abs=0.02)
    assert high[0] - low[0] == pytest.approx(0.2, abs=0.02)


def test_collisions_and_touches(mirror, config):
    """A box in the chassis is reported unless it touches the robot; a pad under one wheel is allowed per link."""
    robots = {ROBOT: robot_entry(config)}
    chassis = box_geometry((0.2, 0.2, 0.2))
    mirror.sync(snapshot((Body("t/box", chassis, Pose((0.0, 0.0, 0.3))),), robots))
    assert mirror.collisions(ROBOT) == ["t/box"]
    mirror.sync(snapshot((Body("t/box", chassis, Pose((0.0, 0.0, 0.3)), touches=(ROBOT,)),), robots))
    assert mirror.collisions(ROBOT) == []
    assert mirror.allowed(f"{ROBOT}/base_link", "t/box") and mirror.allowed("t/box", f"{ROBOT}/base_link")

    # A thin pad right under the front left wheel only.
    pad = box_geometry((0.1, 0.05, 0.02))
    under_wheel = Pose((0.256, 0.285, -0.012))
    mirror.sync(snapshot((Body("t/pad", pad, under_wheel),), robots))
    assert mirror.collisions(ROBOT, margin=0.01) == ["t/pad"]
    mirror.sync(snapshot((Body("t/pad", pad, under_wheel, touches=(f"{ROBOT}/front_left_wheel_link",)),), robots))
    assert mirror.collisions(ROBOT, margin=0.01) == []
    mirror.sync(snapshot((Body("t/pad", pad, under_wheel, touches=(f"{ROBOT}/rear_left_wheel_link",)),), robots))
    assert mirror.collisions(ROBOT, margin=0.01) == ["t/pad"]


def test_tool_is_a_link_of_its_robot(mirror, config):
    """A mounted tool collides as part of its robot, and `touches` may name it by its tool id."""
    robots = {ROBOT: robot_entry(config)}
    mirror.sync(snapshot(robots=robots))
    body = mirror.robot(ROBOT)
    names = [p.getJointInfo(body, i, physicsClientId=mirror.client_id)[12]
             for i in range(p.getNumJoints(body, physicsClientId=mirror.client_id))]
    tool0 = p.getLinkState(body, names.index(b"ur_arm_tool0"), physicsClientId=mirror.client_id)
    # A small box inside the gripper, 0.14 m out along tool0's z.
    inside = Pose(*p.multiplyTransforms(tool0[4], tool0[5], (0.0, 0.0, 0.14), (0.0, 0.0, 0.0, 1.0)))
    cube = box_geometry((0.02, 0.02, 0.02))
    mirror.sync(snapshot((Body("t/held", cube, inside),), robots))
    assert mirror.collisions(ROBOT) == ["t/held"]
    (tool,) = config.model.tools.values()
    mirror.sync(snapshot((Body("t/held", cube, inside, touches=(tool.id,)),), robots))
    assert mirror.collisions(ROBOT) == []


def test_refuses_robot_that_cannot_act(mirror, config):
    """An untracked base or unknown joints make `collisions` refuse; an absent robot never collides."""
    mirror.sync(snapshot(robots={ROBOT: robot_entry(config, base_tracked=False)}))
    with pytest.raises(ValueError, match="not tracked"):
        mirror.collisions(ROBOT)
    mirror.sync(snapshot(robots={ROBOT: robot_entry(config, unmeasured=frozenset({"ur_arm_elbow_joint"}))}))
    with pytest.raises(ValueError, match="ur_arm_elbow_joint"):
        mirror.collisions(ROBOT)

    other = replace(config.model, name="other")
    robots = {ROBOT: robot_entry(config),
              "robots/other": RobotObject("robots/other", other, Pose((0.3, 0.0, 0.0)), {}, enabled=False)}
    mirror.sync(snapshot(robots=robots))
    assert mirror.collisions(ROBOT) == []


def test_tracked_objects(mirror):
    """A tracked object with geometry is an obstacle "tracked/<name>"; one without geometry isn't built."""
    shape = Body("tracked/bar", box_geometry((1.0, 0.1, 0.1)), Pose((0.0, 0.0, 1.0)))
    frame = Body("tracked/probe", Geometry((), ()), Pose())
    mirror.sync(snapshot((shape, frame)))
    assert mirror.obstacle_ids() == ["tracked/bar"]
    assert mirror.id_of(mirror.body_ids("tracked/bar")[0]) == "tracked/bar"


def test_robot_joints_and_reload(mirror, config, monkeypatch):
    """Base and joints follow the robot; a new RobotModel object reloads it, by the same test as geometry."""
    loads: list[str] = []
    original = p.loadURDF
    monkeypatch.setattr(p, "loadURDF", lambda *args, **kwargs: loads.append(args[0]) or original(*args, **kwargs))

    base = Pose((1.0, 2.0, 0.0))
    mirror.sync(snapshot(robots={ROBOT: robot_entry(config, {"ur_arm_shoulder_pan_joint": 0.5, "no_such": 1.0},
                                                    base)}))
    body = mirror.robot(ROBOT)
    assert mirror.robots == {ROBOT: body} and mirror.id_of(body) == ROBOT
    assert mirror.obstacle_ids() == []
    assert p.getBasePositionAndOrientation(body, physicsClientId=mirror.client_id)[0] == base.position
    pan = next(i for i in range(p.getNumJoints(body, physicsClientId=mirror.client_id))
               if p.getJointInfo(body, i, physicsClientId=mirror.client_id)[1] == b"ur_arm_shoulder_pan_joint")
    assert p.getJointState(body, pan, physicsClientId=mirror.client_id)[0] == pytest.approx(0.5)

    mirror.sync(snapshot(robots={ROBOT: robot_entry(config, {}, base)}))
    assert len(loads) == 1
    mirror.sync(snapshot(robots={ROBOT: RobotObject(ROBOT, replace(config.model), base, {})}))
    assert len(loads) == 2
    assert mirror.id_of(mirror.robot(ROBOT)) == ROBOT
    assert p.getBasePositionAndOrientation(mirror.robot(ROBOT), physicsClientId=mirror.client_id)[0] == base.position

    mirror.sync(snapshot())
    assert mirror.robots == {}


def test_robot_vs_robot(mirror, config):
    """Another robot overlapping counts as a hit, by its robot id."""
    robots = {ROBOT: robot_entry(config), "robots/other": robot_entry(config, base=Pose((0.3, 0.0, 0.0)),
                                                                      object_id="robots/other")}
    mirror.sync(snapshot(robots=robots))
    assert mirror.collisions(ROBOT) == ["robots/other"]
    assert mirror.collisions("robots/other") == [ROBOT]


def test_active_points_pp_at_mirror(mirror):
    """Inside active(), pp.CLIENT is this world; afterwards it is restored."""
    before = pp.CLIENT
    with mirror.active():
        assert pp.CLIENT == mirror.client_id
    assert pp.CLIENT == before


def test_body_without_collision_meshes_never_collides(mirror, config):
    """A body with only visual meshes gets no PyBullet body, so nothing hits it."""
    visual_only = Geometry(box_geometry((1.0, 1.0, 1.0)).visual, ())
    ghost = Body("test/ghost", visual_only, Pose((0.0, 0.0, 0.3)))
    mirror.sync(snapshot((ghost,), {ROBOT: robot_entry(config)}))
    assert mirror.body_ids("test/ghost") == []
    assert mirror.obstacle_ids() == []
    assert mirror.collisions(ROBOT) == []


def test_window_without_display_raises_and_keeps_the_world(monkeypatch):
    """Asking for a window without an X display raises; the world carries on without one."""
    monkeypatch.delenv("DISPLAY", raising=False)
    world = PyBulletMirror()
    world.sync(snapshot((Body("t/a", box_geometry((1.0, 1.0, 1.0)), Pose()),)))
    try:
        with pytest.raises(RuntimeError, match="display"):
            world.set_gui(True)
        assert not world.gui and world.connected
        assert world.obstacle_ids() == ["t/a"], "the world is untouched"
    finally:
        world.close()


def test_disabled_body_stays_built_but_never_collides(mirror, config, resets):
    """A disabled body keeps its PyBullet body (no rebuild), is not an obstacle, and collides again once enabled."""
    robots = {ROBOT: robot_entry(config)}
    box = box_geometry((0.2, 0.2, 0.2))
    mirror.sync(snapshot((Body("t/box", box, Pose((0.0, 0.0, 0.3))),), robots))
    built = mirror.body_ids("t/box")
    mirror.sync(snapshot((Body("t/box", box, Pose((0.0, 0.0, 0.3)), enabled=False),), robots))
    assert mirror.body_ids("t/box") == built
    assert mirror.obstacle_ids() == [] and mirror.collisions(ROBOT) == []
    mirror.sync(snapshot((Body("t/box", box, Pose((0.0, 0.0, 0.3))),), robots))
    assert mirror.body_ids("t/box") == built and mirror.collisions(ROBOT) == ["t/box"]
