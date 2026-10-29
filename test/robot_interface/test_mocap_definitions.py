"""Motive's definitions as the relay announces them: body changes told apart, cameras moved, announcements read."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from body_definition_fixtures import MARKERS, body, moved
from husky_assembly_teleop.robot_interface.mocap_definitions import (MATCHES, UNKNOWN, OUT_OF_DATE, MocapCamera,
                                                                     camera_changes, cameras_json, compare,
                                                                     definition_check, definitions_json, match,
                                                                     read_cameras, read_definitions)

pytestmark = pytest.mark.safety

#: A lab camera (Motive 3.0.3.1's reply, turned to Z-up).
CAMERA = MocapCamera("PrimeX 22 #72318", (-3.286, -1.852, 2.188), (-0.186, 0.148, -0.646, 0.725))


@pytest.mark.parametrize("name", ["calibration_tool", "tool body renamed"])
def test_the_same_markers_are_unchanged_whatever_the_name(name):
    """A rename is reported, but it is no data change."""
    change = compare(body(), body(name=name))
    assert change.kind == ("unchanged" if name == "calibration_tool" else "renamed") and not change.data_change


def test_the_same_markers_in_another_order_are_unchanged():
    """The order NatNet lists the markers in is not part of the body."""
    assert compare(body(), body(moved(order=[3, 0, 4, 1, 2]))).kind == "unchanged"


@pytest.mark.parametrize("shift, turn", [((0.002, 0.0, 0.0), (0.0, 0.0, 0.0)), ((0.0, 0.0, 0.0), (0.0, 0.0, 3.0))])
def test_a_pivot_or_orientation_edit_moves_the_frame(shift, turn):
    """The same layout in a moved or turned frame: a data change, with the pivot's move and the turn measured."""
    change = compare(body(), body(moved(shift=shift, turn_degrees=turn)))
    assert change.kind == "pivot moved" and change.data_change and change.markers_moved < 1e-6
    assert change.pivot_moved == pytest.approx(np.linalg.norm(shift), abs=1e-4)
    assert np.degrees(change.turned) == pytest.approx(max(turn), abs=1e-3)


def test_a_refine_moves_markers_a_little_in_the_same_frame():
    """Markers moved by a few tenths of a millimetre around the same pivot: refined."""
    change = compare(body(), body(moved(noise=0.3e-3)))
    assert change.kind == "refined" and change.data_change and 0.05e-3 < change.markers_moved < 0.6e-3


def test_a_body_made_again_from_the_same_markers_is_redefined():
    """A new pivot and orientation, and the markers measured afresh: redefined, not another body."""
    change = compare(body(), body(moved(shift=(0.012, -0.004, 0.003), turn_degrees=(10.0, -25.0, 40.0), noise=0.2e-3,
                                        order=[1, 0, 2, 4, 3])))
    assert change.kind == "redefined" and change.data_change and change.markers_moved < 0.5e-3


@pytest.mark.parametrize("other", [MARKERS * 1.3, MARKERS[:4]])
def test_other_markers_on_the_same_id_are_another_body(other):
    """Another layout, or another number of markers."""
    assert compare(body(), body(other)).kind == "other body"


def test_the_same_markers_under_a_new_id_are_the_same_body():
    """A body that went and one that came with its markers: one event, the id changed; others come and go."""
    other = body(MARKERS * 1.5, streaming_id=1850, name="base")
    after = {1900: body(moved(order=[4, 3, 2, 1, 0]), streaming_id=1900), 1950: body(MARKERS * 2.0, streaming_id=1950)}
    events = match({1866: body(), 1850: other}, after)
    assert {(e.before, e.after, e.change.kind if e.change else None) for e in events} == {
        (1866, 1900, "unchanged"), (None, 1950, None), (1850, None, None)}


def test_the_definitions_read_back_as_announced():
    """Ids, names, offsets and labels come back exactly; problems are not part of the bodies."""
    bodies = [body(), body(MARKERS * 1.5, streaming_id=1850, name="base")]
    read = read_definitions(definitions_json(bodies, ["camera 'x': read 20 of 28 bytes"], 1.0))
    assert read == {1866: bodies[0], 1850: bodies[1]}


def test_the_cameras_read_back_as_announced():
    """By name, position and orientation exact."""
    assert read_cameras(cameras_json([CAMERA], 1.0)) == {CAMERA.name: CAMERA}


@pytest.mark.parametrize("orientation", [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, float("nan"))])
def test_cameras_with_one_without_a_rotation_are_refused(orientation):
    """A floor calibration file or an announcement with such a camera does not read: it would fail where turned."""
    bad = MocapCamera("PrimeX 22 #72318", (1.0, 2.0, 3.0), orientation)
    with pytest.raises(ValueError, match="PrimeX 22 #72318.*no rotation|PrimeX 22 #72318.*not finite"):
        read_cameras(cameras_json([CAMERA, bad], 1.0))


@pytest.mark.parametrize("text", ["[]", "{}", '{"bodies": [{"id": 1}]}', "not json"])
def test_another_format_is_refused(text):
    """Not the relay's announcement: a ValueError naming what it is not."""
    with pytest.raises(ValueError, match="not the relay's body definitions"):
        read_definitions(text)


def moved_camera(shift=(0.0, 0.0, 0.0), turn_degrees: float = 0.0) -> MocapCamera:
    """CAMERA shifted, metres, and turned about its own z."""
    turned = Rotation.from_quat(CAMERA.orientation) * Rotation.from_euler("z", turn_degrees, degrees=True)
    return MocapCamera(CAMERA.name, tuple(np.add(CAMERA.position, shift)), tuple(turned.as_quat()))


@pytest.mark.parametrize("camera, change", [
    (moved_camera((0.0004, 0.0, 0.0)), None),
    (moved_camera(turn_degrees=0.05), None),
    (moved_camera((0.0, 0.0015, 0.0)), "camera moved 1.5 mm, turned 0.00°"),
    (moved_camera(turn_degrees=0.2), "camera moved 0.0 mm, turned 0.20°"),
])
def test_a_camera_moved_over_a_millimetre_or_a_tenth_of_a_degree_is_reported(camera, change):
    """Continuous calibration's small moves are not reported; larger ones are, with the numbers."""
    assert camera_changes({CAMERA.name: CAMERA}, {camera.name: camera}).get(CAMERA.name) == change


def test_cameras_that_come_and_go_are_reported():
    """By name."""
    other = MocapCamera("Prime 13 #31321", (3.8, -5.2, 5.7), (0.0, 0.0, 0.0, 1.0))
    assert camera_changes({CAMERA.name: CAMERA}, {other.name: other}) == {CAMERA.name: "camera gone",
                                                                          other.name: "new camera"}


@pytest.mark.parametrize("current, status, why", [
    ({1866: body()}, MATCHES, "unchanged"),
    ({1866: body(name="tool")}, MATCHES, "renamed"),
    ({1866: body(moved(noise=0.3e-3))}, OUT_OF_DATE, "made for an earlier body: refined"),
    ({1866: body(moved(shift=(0.002, 0.0, 0.0)))}, OUT_OF_DATE, "made for an earlier body: pivot moved"),
    ({1866: body(MARKERS * 1.3)}, OUT_OF_DATE, "made for an earlier body: other body"),
    ({1850: body(streaming_id=1850)}, UNKNOWN, "Motive defines no rigid body 1866"),
    (None, UNKNOWN, "no body definitions from Motive yet"),
])
def test_a_calibration_fits_while_its_body_definition_is_no_data_change(current, status, why):
    """Matches, wrong, or unknown when Motive's definitions are not heard or lack the body."""
    got, detail = definition_check(body(), 1866, current)
    assert got == status and detail.startswith(why)


def test_a_calibration_naming_no_body_definition_is_unknown():
    """crl-husky's defaults and older sessions name none: never judged wrong."""
    assert definition_check(None, 1866, {1866: body()})[0] == UNKNOWN


def brute_force_moved(before, after) -> float:
    """The smallest largest marker left over of any order, each fitted on its own: what `compare` must find."""
    from husky_assembly_teleop.robot_interface.mocap_definitions import _fit, _orders
    return min(float(_fit(before.array(), after.array()[list(order)])[2].max())
               for order in _orders(before.array(), after.array()))


@pytest.mark.parametrize("seed", range(12))
def test_the_order_found_at_once_is_the_best_of_every_order(seed):
    """Random layouts of 3 to 6 markers, moved, nudged and reordered: the same fit as trying each order alone."""
    rng = np.random.default_rng(seed)
    points = rng.uniform(-0.06, 0.06, (3 + seed % 4, 3))
    after = moved(points, shift=rng.normal(0, 0.01, 3), turn_degrees=rng.normal(0, 20, 3), noise=0.4e-3,
                  order=rng.permutation(len(points)), seed=seed)
    before_body, after_body = body(points), body(after)
    assert compare(before_body, after_body).markers_moved == pytest.approx(brute_force_moved(before_body, after_body),
                                                                           abs=1e-9)


def test_a_pair_is_compared_once():
    """Judging the same measurement again on every tick costs a lookup."""
    compare.cache_clear()
    for _ in range(5):
        compare(body(), body(moved(noise=0.3e-3)))
    assert compare.cache_info().misses == 1 and compare.cache_info().hits == 4
