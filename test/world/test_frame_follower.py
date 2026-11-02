"""Following Motive's frame: the state, which fit decides, the refit in use, and what is logged and announced."""

from __future__ import annotations

import re
from pathlib import Path

import numpy as np
import pytest
from mocap_frame_fixtures import (LAB_ANCHORS, LAB_CAMERAS, LOOSE, SHIFT, TURN, anchors_after, cameras_after,
                                  seen_after)
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose
from husky_assembly_teleop.robot_interface.mocap_definitions import (FRAME_ERROR, FRAME_HELD, FRAME_NONE,
                                                                     FRAME_REFIT_PENDING, FRAME_UNCHECKED,
                                                                     FRAME_UNTRUSTED, MocapAnchor,
                                                                     MocapCamera)
from husky_assembly_teleop.world.frame_follower import ANCHOR_GONE_SEC, FrameFollower
from husky_assembly_teleop.world.mocap_frame import FloorCalibration, truss_names

pytestmark = pytest.mark.safety

FLOOR_FILE = "20261024_090000_mocap_floor_calibration.json"
#: Motive's frame after one recalibration.
RECALIBRATED = [(SHIFT, TURN)]
#: How often the relay looks, seconds.
LOOK = 2.0


def moved_alone(camera: MocapCamera, shift) -> MocapCamera:
    """A camera moved on its own by `shift`, metres."""
    return MocapCamera(camera.name, tuple(np.add(camera.position, shift)), camera.orientation)


def follower(anchors: bool = True, floor_from_raw: Pose = Pose()) -> FrameFollower:
    """A follower of a floor calibration made with the lab's cameras (and anchors) where they are; cameras heard."""
    stored = {name: MocapAnchor(name, position) for name, position in LAB_ANCHORS.items()} if anchors else {}
    follower = FrameFollower(use_refit_hint="Use refit")
    follower.use_floor_calibration(FloorCalibration(Path(FLOOR_FILE), floor_from_raw, dict(LAB_CAMERAS), stored))
    follower.cameras = dict(LAB_CAMERAS)
    return follower


def looks(follower: FrameFollower, markers, start: float = 0.0, count: int = 1) -> list:
    """`count` looks LOOK apart from `start`, each seeing `markers` (a list, or a function of the look's index)."""
    events = []
    for index in range(count):
        events += follower.look(markers(index) if callable(markers) else markers, start + index * LOOK)
    return events


def titles(events, level: str | None = None) -> list[str]:
    """The events' titles, of one level or all."""
    return [event.title for event in events if level is None or event.level == level]


def text(events) -> str:
    """Every event's title and lines, whitespace squeezed."""
    return " ".join(" ".join(" ".join([event.title, *event.lines]) for event in events).split())


def recalibrate(follower: FrameFollower) -> list:
    """Motive recalibrated: the cameras and anchors seen from its moved frame, one look."""
    follower.cameras = cameras_after(RECALIBRATED)
    return follower.look(anchors_after(RECALIBRATED), 0.0)


def test_unchecked_until_the_first_look_then_held_by_the_anchors_with_one_table():
    """A start announces unchecked, never held; the first look with anchors and cameras logs one table."""
    frame = follower()
    assert frame.status == FRAME_UNCHECKED and frame.news() and frame.announcement(0.0)["status"] == "unchecked"
    events = looks(frame, [*LAB_ANCHORS.values(), LOOSE])
    assert frame.status == FRAME_HELD and frame.deciding.source == "anchors"
    assert titles(events) == ["frame held by the anchors: poses off now 0.0 mm, within the 1.0 mm limit"]
    assert sorted(frame.anchors) == sorted(LAB_ANCHORS) and frame.news()
    assert looks(frame, [*LAB_ANCHORS.values(), LOOSE], LOOK, 5) == [] and not frame.news()


def test_the_anchors_hold_the_frame_while_their_trusted_fit_decides():
    """Held or a refit pending by the anchors: they hold it; unchecked or decided by the truss cameras: they do not."""
    frame = follower()
    assert not frame.anchors_hold
    looks(frame, [*LAB_ANCHORS.values(), LOOSE])
    assert frame.anchors_hold
    frame.cameras = cameras_after(RECALIBRATED)
    looks(frame, anchors_after(RECALIBRATED), LOOK)
    assert frame.status == FRAME_REFIT_PENDING and frame.deciding.source == "anchors" and frame.anchors_hold
    looks(frame, [], 2 * LOOK, int(ANCHOR_GONE_SEC / LOOK) + 1)
    assert frame.deciding.source == "truss cameras" and not frame.anchors_hold


def test_no_floor_calibration_is_none_and_one_not_read_is_an_error():
    """Nothing to follow: no looks, no events."""
    frame = FrameFollower()
    assert frame.status == FRAME_NONE and frame.look([LOOSE], 0.0) == [] and frame.lab_from_raw() is None
    frame.error = "floor calibration at start not read: half-synced"
    assert frame.status == FRAME_ERROR and frame.announcement(0.0)["error"].endswith("half-synced")


def test_anchors_lost_without_cameras_are_untrusted_not_held():
    """Six anchors stored, no cameras, Motive's frame moved 22 mm: no anchor found, so untrusted, logged as an error."""
    frame = follower()
    frame.cameras = {}
    events = looks(frame, [np.add(position, (0.022, 0.0, 0.0)) for position in LAB_ANCHORS.values()])
    assert frame.status == FRAME_UNTRUSTED and titles(events, "error")[0].startswith("FRAME UNTRUSTED")


def test_looks_without_any_marker_fit_no_anchors_and_report_them_once_they_last():
    """Motive streams no unlabeled markers: no anchors' fit, the truss cameras decide; after 10 s all are missing."""
    frame = follower()
    events = looks(frame, [], 0.0, int(ANCHOR_GONE_SEC / LOOK) + 1)
    assert frame.anchor_fit is None and frame.status == FRAME_HELD and frame.deciding.source == "truss cameras"
    assert titles(events, "warn")[0].startswith("only 0 of 6 anchors seen")


def test_the_truss_cameras_alone_hold_a_floor_calibration_without_anchors():
    """No anchors stored: the truss cameras decide; a 0.6 mm move is held."""
    frame = follower(anchors=False)
    frame.cameras = cameras_after([((0.0006, 0.0, 0.0), 0.0)])
    assert titles(looks(frame, [LOOSE]), "info")[0].startswith("frame held by the truss cameras: poses off now 0.6")


def test_a_recalibration_is_a_refit_pending_until_used_then_held_with_lab_from_raw_back():
    """Warned once with both fits; Use refit puts a fixed point back where the floor calibration has it."""
    frame = follower(floor_from_raw=Pose((0.1, 0.0, 0.0)))
    events = recalibrate(frame)
    assert frame.status == FRAME_REFIT_PENDING and frame.news()
    assert titles(events, "warn") == ["REFIT PENDING by the anchors: poses are off by up to 49.0 mm until the refit "
                                      "is used"]
    assert "anchors trusted 6/6 22.6 mm 0.50° 49.0 ± 0.1 mm 49.0 mm" in text(events)
    assert "use it with Use refit" in text(events)
    assert frame.announcement(0.0)["pending"]["describe"].startswith("poses off now 49.0 mm")
    used, message, logged = frame.use_refit()
    assert used and message.startswith("refit used") and frame.status == FRAME_HELD and frame.news()
    assert titles(logged, "info") == ["REFIT USED: every pose moved up to 49.0 mm, back into the lab frame"]
    point = np.array([1.0, 2.0, 0.5])
    np.testing.assert_allclose(compose(frame.lab_from_raw(), Pose(tuple(seen_after(point, RECALIBRATED)))).position,
                               point + (0.1, 0.0, 0.0), atol=1e-6)
    # * After the refit the table's columns differ: moved since the floor calibration, nothing off now.
    events = frame.look([np.add(position, (0.0012, 0.0, 0.0)) for position in anchors_after(RECALIBRATED)], LOOK)
    assert "since 20261024_090000" in text(events)
    assert re.search(r"anchors trusted 6/6 [\d.]+ mm [\d.]+° [\d.]+ ± [\d.]+ mm 1\.2 mm", text(events))


def test_nothing_pending_and_an_untrusted_fit_are_not_used():
    """Held: no refit pending; three truss cameras moved on their own: untrusted, refused."""
    frame = follower(anchors=False)
    looks(frame, [])
    assert frame.use_refit()[:1] == (False,)
    for index, name in enumerate(truss_names(LAB_CAMERAS)[:3]):
        camera = frame.cameras[name]
        frame.cameras[name] = moved_alone(camera, (0.0, 0.004 * (index + 1), 0.0))
    events = looks(frame, [], LOOK)
    used, message, _ = frame.use_refit()
    assert not used and "not trusted" in message and frame.status == FRAME_UNTRUSTED and titles(events, "error")


def test_jitter_while_a_refit_is_pending_is_logged_and_announced_once():
    """Thirty looks 0.1 mm apart after a recalibration: one warning, one announcement."""
    frame = follower(anchors=False)
    rng = np.random.default_rng(0)
    events, announcements = [], 0
    for index in range(30):
        frame.cameras = {name: moved_alone(camera, rng.normal(0.0, 1e-4, 3))
                         for name, camera in cameras_after(RECALIBRATED).items()}
        events += frame.look([], index * LOOK)
        announcements += frame.news()
    assert len(titles(events, "warn")) == 1 and announcements == 1 and frame.use_refit()[0]


def test_an_anchor_flickering_every_look_logs_nothing():
    """A3 gone every other look for two minutes: never reported, no table after the first."""
    frame = follower()
    events = looks(frame, lambda i: anchors_after([], [n for n in LAB_ANCHORS if n != "A3" or i % 2]), 0.0, 60)
    assert len(events) == 1 and frame.anchors_missing == ()


def test_an_anchor_gone_ten_seconds_is_reported_once_and_its_return_logged():
    """A3 covered: reported after ANCHOR_GONE_SEC with the others holding the frame; seen again: logged."""
    frame = follower()
    looks(frame, list(LAB_ANCHORS.values()))
    count = int(ANCHOR_GONE_SEC / LOOK) + 3
    events = looks(frame, anchors_after([], [n for n in LAB_ANCHORS if n != "A3"]), LOOK, count)
    assert titles(events) == ["anchor A3 not seen for 10 s; the other 5 hold the frame"]
    assert frame.anchors_missing == ("A3",) and frame.announcement(0.0)["anchors_missing"] == ["A3"]
    assert titles(looks(frame, list(LAB_ANCHORS.values()), LOOK * (count + 1))) == ["all 6 anchors seen again"]


def test_under_four_anchors_the_truss_cameras_hold_the_frame():
    """Three anchors seen after a recalibration: the truss cameras decide; reported once they stay away."""
    frame = follower()
    frame.cameras = cameras_after(RECALIBRATED)
    events = looks(frame, anchors_after(RECALIBRATED, ("A1", "A2", "A3")), 0.0, int(ANCHOR_GONE_SEC / LOOK) + 1)
    assert frame.deciding.source == "truss cameras" and frame.status == FRAME_REFIT_PENDING
    assert "only 3 of 6 anchors seen (anchors A4, A5, A6 not for 10 s): the truss cameras hold the frame" in \
        titles(events)
    assert frame.announcement(0.0)["decides"] == "truss cameras"


def test_an_anchor_moved_on_its_own_is_warned_about_and_left_out():
    """A2 3 mm off: one warning naming it; the other five hold the frame."""
    frame = follower()
    moved = {**LAB_ANCHORS, "A2": tuple(np.add(LAB_ANCHORS["A2"], (0.003, 0.0, 0.0)))}
    events = looks(frame, list(moved.values()), 0.0, 3)
    assert titles(events, "warn")[0].startswith("ANCHORS MOVED") and "A2: moved 3.0 mm against the others" in \
        text(events)
    assert frame.status == FRAME_HELD and frame.anchor_fit.used == ("A1", "A3", "A4", "A5", "A6")


def test_an_anchor_left_out_comes_back_only_well_within_its_limit():
    """A2 3 mm off is left out; at 1.8 mm, under the 2 mm limit, still out; at 1.2 mm back in. One warning only."""
    frame = follower()
    events = []
    for index, off in enumerate((0.003, 0.0018, 0.0021, 0.0018, 0.0012)):
        events += looks(frame, list({**LAB_ANCHORS, "A2": tuple(np.add(LAB_ANCHORS["A2"], (off, 0.0, 0.0)))}.values()),
                        index * LOOK)
        assert ("A2" in frame.anchor_fit.bumped) == (off > 0.0015)
        if off == 0.0018:
            assert frame.anchor_fit.bumped["A2"] == "1.8 mm off the others, kept out until back within 1.5 mm"
    assert len([title for title in titles(events, "warn") if title.startswith("ANCHORS MOVED")]) == 1


def test_a_refit_pending_is_held_again_only_well_within_the_limit():
    """Motive's frame 1.2 mm off: pending; at 0.9 and 1.1 mm still pending, logged once; at 0.6 mm held again."""
    frame = follower()
    events, states = [], []
    for index, shift in enumerate((0.0, 0.0012, 0.0009, 0.0011, 0.0009, 0.0006)):
        moved = [((shift, 0.0, 0.0), 0.0)]
        frame.cameras = cameras_after(moved)
        events += looks(frame, anchors_after(moved), index * LOOK)
        states.append(frame.status)
    assert states == [FRAME_HELD] + [FRAME_REFIT_PENDING] * 4 + [FRAME_HELD]
    assert len(titles(events, "warn")) == 1 and "held again once poses are off under 0.8 mm" in text(events)


def test_both_fits_are_tabled_with_how_far_apart_they_place_the_lab():
    """The truss cameras say 3 mm, the anchors nothing: held by the anchors, the cameras' fit beside them."""
    frame = follower()
    frame.cameras = cameras_after([((0.003, 0.0, 0.0), 0.0)])
    said = text(looks(frame, list(LAB_ANCHORS.values())))
    assert "truss cameras trusted 6/6 3.0 mm" in said
    assert "the two fits place the lab's volume up to 3.0 mm apart" in said
    assert frame.status == FRAME_HELD


def test_the_metrics_carry_both_fits_and_how_far_poses_are_off():
    """After a recalibration: off beyond its limit; after the refit is used: nothing off, the refit in the sample."""
    frame = follower()
    recalibrate(frame)
    sample = frame.metrics(1.0)
    assert sample["status"] == "refit_pending" and sample["decides"] == "anchors" and sample["apart"] < 1e-6
    assert sample["refit"] is None and sample["off"] > sample["off_limit"]
    frame.use_refit()
    frame.look(anchors_after(RECALIBRATED), LOOK)
    sample = frame.metrics(2.0)
    assert sample["refit"]["trusted"] and sample["off"] < 1e-6 and sample["status"] == "held"


def test_the_metrics_carry_each_anchors_quality_and_the_raw_inputs():
    """A2 moved 3 mm, A6 unseen: A2 left out and 3 mm off a fit of the others, A6 missing; the rest used, close."""
    frame = follower()
    moved = {**LAB_ANCHORS, "A2": tuple(np.add(LAB_ANCHORS["A2"], (0.003, 0.0, 0.0)))}
    markers = [position for name, position in moved.items() if name != "A6"] + [LOOSE]
    frame.look(markers, 0.0, [(0.0001 * (index + 1), 200) for index in range(len(markers))])
    quality = frame.metrics(1.0)["anchor_quality"]
    assert {name: entry["status"] for name, entry in quality.items()} == {
        "A1": "used", "A2": "bumped", "A3": "used", "A4": "used", "A5": "used", "A6": "missing"}
    assert quality["A2"]["left_out_mm"] == pytest.approx(3.0, abs=0.05) and quality["A2"]["residual_mm"] > 2.9
    assert all(quality[name]["left_out_mm"] < 0.01 for name in ("A1", "A3", "A4", "A5"))
    assert quality["A2"]["raw"] == pytest.approx(list(moved["A2"])) and quality["A2"]["noise_mm"] == 0.2
    assert quality["A2"]["frames"] == 200 and quality["A6"] == {
        "raw": None, "noise_mm": None, "frames": None, "residual_mm": None, "left_out_mm": None, "status": "missing"}
    sample = frame.metrics(1.0)
    assert sample["frames"] == 200 and len(sample["markers"]["raw"]) == 6 and sample["markers"]["noise_mm"][-1] == 0.6
    assert sorted(sample["truss_camera_poses"]) == truss_names(LAB_CAMERAS)


def test_without_a_floor_calibration_a_sample_still_carries_the_markers_and_cameras():
    """Nothing fitted, no anchors named: the raw markers and truss cameras are still there."""
    frame = FrameFollower()
    frame.cameras = dict(LAB_CAMERAS)
    frame.look([LOOSE], 0.0, [(0.0, 10)])
    sample = frame.metrics(1.0)
    assert sample["status"] == FRAME_NONE and sample["anchors"] is None and sample["anchor_quality"] == {}
    assert sample["markers"] == {"raw": [list(LOOSE)], "noise_mm": [0.0], "frames": [10]}
    assert len(sample["truss_camera_poses"]) == 6


def test_anchors_are_found_where_the_floor_calibration_saw_them_after_a_refit_or_by_the_cameras():
    """A second recalibration of 3 cm moves them out of reach: found by the truss cameras' fit."""
    frame = follower()
    second = [(np.array([0.03, 0.0, 0.0]), 0.0)]
    frame.cameras = cameras_after(second)
    frame.look(anchors_after(second), 0.0)
    assert sorted(frame.anchors) == sorted(LAB_ANCHORS) and frame.deciding.source == "anchors"


def test_a_new_floor_calibration_starts_unchecked_without_the_refit():
    """The refit belongs to the floor calibration it was fitted against."""
    frame = follower()
    recalibrate(frame)
    frame.use_refit()
    frame.use_floor_calibration(FloorCalibration(Path("20261025_090000_mocap_floor_calibration.json"), Pose(),
                                                 cameras_after(RECALIBRATED)))
    assert frame.refit is None and frame.status == FRAME_UNCHECKED and frame.lab_from_raw() == Pose()


def test_lab_from_raw_is_the_floor_calibration_then_the_refit():
    """floor_from_raw turned 30° and shifted, then the refit: composed in that order."""
    floor = Pose((1.0, -2.0, 0.0), tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat()))
    frame = follower(floor_from_raw=floor)
    assert frame.lab_from_raw() == floor
    recalibrate(frame)
    frame.use_refit()
    expected = compose(floor, frame.refit.raw_cal_from_raw_now)
    np.testing.assert_allclose(frame.lab_from_raw().position, expected.position, atol=1e-12)
