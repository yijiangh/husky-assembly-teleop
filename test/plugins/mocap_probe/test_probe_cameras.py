"""The mocap probe's cameras: each drawn where Motive's calibration puts it, its wireframe pointing where it looks."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import viser
from scipy.spatial.transform import Rotation

from test_floor_calibration_plugin import make_ctx, server  # noqa: F401 (the module's viser server fixture)
from husky_assembly_teleop.mocap_relay.body_definitions import parse_descriptions
from husky_assembly_teleop.mocap_relay.node import z_up_camera
from husky_assembly_teleop.plugins.mocap_probe import MocapProbePlugin, camera_frustum
from husky_assembly_teleop.world.mocap_frame import camera_label

#: The lab's ten cameras as Motive 3.0.3.1 gave them (Y-up), 23 Oct 2026.
LAB_REPLY = Path(__file__).parents[2] / "mocap_relay" / "data" / "modeldef_motive_3_0_3_1.bin"
CAMERAS = parse_descriptions(LAB_REPLY.read_bytes(), 4, 0)[1]
#: The Y-up frame turned into the relay's Z-up world.
Z_UP = Rotation.from_euler("x", 90.0, degrees=True)


class Scene:
    """Keeps the frustums and labels added, by name."""

    def __init__(self):
        """Nothing added yet."""
        self.added = {}

    def add_camera_frustum(self, name, **options):
        """Keep it."""
        self.added[name] = options
        return SimpleNamespace(remove=lambda: self.added.pop(name), visible=True)

    def add_label(self, name, text, **options):
        """Keep it."""
        self.added[name] = {"text": text, **options}
        return SimpleNamespace(remove=lambda: self.added.pop(name), visible=True)


@pytest.mark.parametrize("camera", CAMERAS, ids=[camera.name for camera in CAMERAS])
def test_each_wireframe_looks_where_motives_camera_looks(camera):
    """The frustum (viser: along its +z) looks along Motive's camera -z, turned Z-up; the lab's all face the room."""
    scene = Scene()
    camera_frustum(scene, "/cameras/0", z_up_camera(camera))
    w, x, y, z = scene.added["/cameras/0/frustum"]["wxyz"]
    ahead = Rotation.from_quat((x, y, z, w)).apply((0.0, 0.0, 1.0))
    motive = Z_UP.apply(Rotation.from_quat(camera.orientation).apply((0.0, 0.0, -1.0)))
    np.testing.assert_allclose(ahead, motive, atol=1e-9)
    towards_room = -np.asarray(z_up_camera(camera).position) + (0.0, 0.0, 1.0)
    assert ahead @ towards_room / np.linalg.norm(towards_room) > 0.9 and ahead[2] < 0.0
    assert scene.added["/cameras/0/label"]["text"] == camera_label(camera.name)  # * e.g. "TFL #31321"


def test_the_toggle_shows_the_relays_cameras_and_redraws_when_it_announces_others(server, tmp_path):  # noqa: F811
    """Nothing is drawn until ticked; ticked, every camera; another announcement replaces them."""
    ctx = make_ctx(server, tmp_path)
    state = ctx.world.mocap_relay.state
    state.cameras = {camera.name: z_up_camera(camera) for camera in CAMERAS}
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    assert plugin._camera_handles == [] and "10 cameras" in plugin._cameras_status.content
    plugin._show_cameras.value = True
    plugin.draw(ctx)
    # * A frustum and a name per camera, then every view's edge rays and every aim cross, one node each.
    assert len(plugin._camera_handles) == 22 and all(handle.visible for handle in plugin._camera_handles)
    state.cameras = {CAMERAS[0].name: z_up_camera(CAMERAS[0])}
    plugin.draw(ctx)
    assert len(plugin._camera_handles) == 4
    plugin._views_height.value = 1.0
    plugin.draw(ctx)
    assert plugin._drawn_cameras[1] == 1.0
    plugin._show_cameras.value = False
    plugin.draw(ctx)
    assert not any(handle.visible for handle in plugin._camera_handles)


def test_the_views_on_the_floor_are_drawn_by_count_and_cut_again_for_another_height(server, tmp_path):  # noqa: F811
    """One mesh per number of cameras seeing it, covering the coverage map's area; a new height cuts them again."""
    ctx = make_ctx(server, tmp_path)
    ctx.world.mocap_relay.state.cameras = {camera.name: z_up_camera(camera) for camera in CAMERAS}
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin._show_views.value = True
    plugin.draw(ctx)
    assert sum(plugin._views_areas.values()) == pytest.approx(15.0 * 15.0) and max(plugin._views_areas) == 10
    # * A mesh per count, the outlines, and a count on each face big enough to hold one.
    labels = [handle for handle in plugin._views_handles if isinstance(handle, viser.LabelHandle)]
    assert len(plugin._views_handles) == len(plugin._views_areas) + 1 + len(labels) and len(labels) > 20
    assert "seen by 2+" in plugin._cameras_status.content
    floor = dict(plugin._views_areas)
    plugin._views_height.value = 1.0
    plugin.draw(ctx)
    assert plugin._views_areas != floor and plugin._views_for[1] == 1.0
    plugin._show_views.value = False
    plugin.draw(ctx)
    assert not any(handle.visible for handle in plugin._views_handles)


def test_small_camera_moves_draw_nothing_again(server, tmp_path):  # noqa: F811
    """Motive's continuous calibration moves the cameras a little each announcement: the drawing stays."""
    ctx = make_ctx(server, tmp_path)
    state = ctx.world.mocap_relay.state
    state.cameras = {camera.name: z_up_camera(camera) for camera in CAMERAS}
    plugin = MocapProbePlugin()
    plugin.setup(ctx)
    plugin._show_cameras.value = True
    plugin.draw(ctx)
    drawn = list(plugin._camera_handles)
    state.cameras = {name: replace(camera, position=tuple(np.add(camera.position, 0.0002)))
                     for name, camera in state.cameras.items()}
    plugin.draw(ctx)
    assert plugin._camera_handles == drawn
