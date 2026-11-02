"""How Motive's frame moved, from the truss cameras and anchors: recovered, a bumped one left out, residual, files."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose

from repo_paths import DATA
from husky_assembly_teleop.robot_interface.mocap_definitions import MocapAnchor, MocapCamera, cameras_json, read_cameras
from husky_assembly_teleop.world.calibration_sessions import sessions_folder
from husky_assembly_teleop.world.lab import DEFAULT_LAB, lab_file, read_lab
from husky_assembly_teleop.world.mocap_frame import (CAMERA_NAMES, FLOOR_SUFFIX, VOLUME, camera_label,
                                                     camera_serial, find_anchors, fit_anchors, fit_cameras,
                                                     frame_and_cameras, frame_moved, newest_floor_calibration,
                                                     truss_names, volume_moved)

pytestmark = pytest.mark.safety

#: The lab's ten cameras, Z-up, 23 Oct 2026: six on the truss, four on tripods.
LAB = read_cameras((DATA / "sim" / "motive_cameras.json").read_text())
TOP = "Prime 13 #31321"
#: Five anchors spread over the lab, raw.
ANCHORS = {name: MocapAnchor(name, position) for name, position in
           {"A1": (-3.0, -4.0, 0.0), "A2": (3.5, -4.2, 0.0), "A3": (3.4, 5.0, 0.0), "A4": (-3.2, 5.1, 0.0),
            "A5": (0.4, 0.3, 1.9)}.items()}


def recalibrated(cameras: dict, shift=(0.0, 0.0, 0.0), turn_degrees=(0.0, 0.0, 0.0)) -> dict:
    """The cameras as Motive gives them after its frame moved: every pose seen from the moved frame."""
    motion = Rotation.from_euler("xyz", turn_degrees, degrees=True)
    # * The frame moved by (motion, shift): a fixed point p is now at motion⁻¹ (p - shift).
    inverse = motion.inv()
    return {name: MocapCamera(name, tuple(inverse.apply(np.subtract(camera.position, shift))),
                              tuple((inverse * Rotation.from_quat(camera.orientation)).as_quat()))
            for name, camera in cameras.items()}


def bumped(cameras: dict, name: str, shift=(0.0, 0.0, 0.0), turn_degrees: float = 0.0) -> dict:
    """`cameras` with one camera moved on its own: shifted, metres, and turned about its own x."""
    camera = cameras[name]
    turned = Rotation.from_quat(camera.orientation) * Rotation.from_euler("x", turn_degrees, degrees=True)
    return {**cameras, name: MocapCamera(name, tuple(np.add(camera.position, shift)), tuple(turned.as_quat()))}


def back(fit, point) -> np.ndarray:
    """A point in Motive's frame now, taken back to the reference frame by the fit."""
    pose = fit.raw_cal_from_raw_now
    return Rotation.from_quat(pose.orientation).apply(point) + pose.position


def test_only_the_six_truss_cameras_are_reference_cameras():
    """The four on top and the two in the far corners; the tripods never."""
    assert len(truss_names(LAB)) == 6 and "PrimeX 22 #72318" not in truss_names(LAB)


def test_unmoved_cameras_fit_with_no_motion():
    """The same poses: no motion, every truss camera used."""
    fit = fit_cameras(LAB, LAB)
    assert fit.trusted and len(fit.used) == 6 and fit.moved[0] < 1e-9 and fit.moved[1] < 1e-9


@pytest.mark.parametrize("shift, turn", [((0.012, -0.004, 0.002), (0.0, 0.0, 0.8)),
                                         ((0.3, 0.1, 0.0), (1.0, -0.5, 4.0))])
def test_a_moved_frame_is_recovered(shift, turn):
    """Any point measured in the moved frame comes back where it was, to a micrometre."""
    now = recalibrated(LAB, shift, turn)
    fit = fit_cameras(LAB, now)
    floor_point = np.array([1.2, -2.5, 0.0])
    moved_point = recalibrated({"p": MocapCamera("p", tuple(floor_point), (0, 0, 0, 1))}, shift, turn)["p"].position
    assert fit.trusted and not fit.bumped and np.linalg.norm(back(fit, moved_point) - floor_point) < 1e-6


@pytest.mark.parametrize("shift, turn", [((0.003, 0.0, 0.0), 0.0), ((0.0, 0.0, 0.0), 0.5)])
def test_a_camera_bumped_on_its_own_is_named_and_left_out(shift, turn):
    """Shifted 3 mm or turned half a degree: left out, the frame motion still recovered from the other five."""
    now = bumped(recalibrated(LAB, (0.01, 0.0, 0.0), (0.0, 0.0, 0.3)), TOP, shift, turn)
    fit = fit_cameras(LAB, now)
    assert list(fit.bumped) == [TOP] and "left out: TFL #31321 bumped" in fit.describe() and fit.trusted
    assert len(fit.used) == 5
    assert fit.residual < 1e-6


def test_each_camera_is_seen_against_the_moved_frame():
    """The frame moved 2 cm and a tripod 5 mm on its own: the frame's move, and only the tripod's left against it."""
    tripod = "PrimeX 22 #72318"
    fit, seen_before = frame_and_cameras(LAB, recalibrated(bumped(LAB, tripod, (0.005, 0.0, 0.0)), (0.02, 0.0, 0.0),
                                                           (0.0, 0.0, 0.5)))
    assert frame_moved(fit) and abs(fit.moved[0] - 0.02) < 1e-3
    assert {name: round(float(np.linalg.norm(np.subtract(seen_before[name].position, LAB[name].position))), 6)
            for name in LAB if name != tripod} == dict.fromkeys(set(LAB) - {tripod}, 0.0)
    assert abs(np.linalg.norm(np.subtract(seen_before[tripod].position, LAB[tripod].position)) - 0.005) < 1e-6


def test_without_enough_truss_cameras_the_cameras_are_taken_as_they_are():
    """No trusted fit (tripods only): no frame move, every camera's raw move."""
    tripods = {name: camera for name, camera in LAB.items() if name not in truss_names(LAB)}
    now = recalibrated(tripods, (0.02, 0.0, 0.0))
    assert frame_and_cameras(tripods, now) == (None, now) and not frame_moved(None)


def test_three_disagreeing_cameras_are_not_trusted():
    """Half the truss cameras moved on their own: whatever is left is not trusted, and it says so."""
    names = truss_names(LAB)
    now = LAB
    for index, name in enumerate(names[:3]):
        now = bumped(now, name, (0.0, 0.004 * (index + 1), 0.0))
    fit = fit_cameras(LAB, now)
    assert not fit.trusted and "NOT TRUSTED" in fit.describe()


def test_missing_cameras_are_named_and_too_few_give_no_fit():
    """A camera Motive no longer has is missing; with two left there is no fit at all."""
    names = truss_names(LAB)
    fit = fit_cameras(LAB, {name: camera for name, camera in LAB.items() if name != names[0]})
    assert fit.missing == (names[0],) and fit.trusted
    fit = fit_cameras(LAB, {name: LAB[name] for name in names[:2]})
    assert not fit.trusted and fit.describe().startswith("NO FIT")


def test_a_tripod_moved_is_no_frame_move():
    """Only the truss cameras tell the frame: a tripod 5 cm off moves no point of the lab's volume."""
    assert not frame_moved(fit_cameras(LAB, bumped(LAB, "PrimeX 22 #72318", (0.05, 0.0, 0.0))))


@pytest.mark.parametrize("shift, turn_degrees, moved", [((0.0008, 0.0, 0.0), 0.0, False),
                                                        ((0.0012, 0.0, 0.0), 0.0, True),
                                                        ((0.0, 0.0, 0.0), 0.02, True),
                                                        ((0.0, 0.0, 0.0), 0.012, False)])
def test_a_frame_move_counts_by_how_far_it_moves_a_point_of_the_lab(shift, turn_degrees, moved):
    """1 mm anywhere in the volume: a 0.02 deg turn moves its far corner 1.5 mm though the origin stays."""
    assert frame_moved(fit_cameras(LAB, recalibrated(LAB, shift, (0.0, 0.0, turn_degrees)))) == moved


def test_the_volume_is_the_central_cube_inside_the_usable_floor():
    """VOLUME is 3 m along x, -y and z from the lab frame's origin, inside the border of data/lab/main.json."""
    border = read_lab(lab_file(DATA, DEFAULT_LAB)).border
    assert VOLUME == ((0.0, 3.0), (-3.0, 0.0), (0.0, 3.0))
    assert border.x[0] <= VOLUME[0][0] and VOLUME[0][1] <= border.x[1]
    assert border.y[0] <= VOLUME[1][0] and VOLUME[1][1] <= border.y[1]


#: A floor calibration's floor_from_raw: 2 mm and a tilt of 0.25 deg.
FLOOR_FROM_RAW = ((0.002, 0.0, 0.001), tuple(Rotation.from_euler("x", 0.25, degrees=True).as_quat()))


def floor_file(sessions, stamp: str, cameras: dict | None, anchors: dict | None = None, **marks) -> Path:
    """A floor calibration as the probe exports it to its dated session: floor_from_raw, cameras and anchors raw."""
    content = {"kind": "mocap_floor_calibration",
               "floor_from_raw": {"position": FLOOR_FROM_RAW[0], "orientation": FLOOR_FROM_RAW[1]},
               "anchors": {name: list(anchor.position) for name, anchor in (anchors or {}).items()}, **marks}
    if cameras is not None:
        content["cameras"] = json.loads(cameras_json(list(cameras.values()), 1.0))["cameras"]
    path = sessions / stamp[:8] / f"{stamp}{FLOOR_SUFFIX}"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(content))
    return path


def test_the_newest_floor_calibration_with_cameras_is_read_and_old_refit_files_are_not(tmp_path):
    """One without cameras is passed over; a refit file next to it, as the relay once wrote, changes nothing."""
    floor_file(tmp_path, "20261020_100000", LAB, ANCHORS)
    floor_file(tmp_path, "20261024_090000", None)
    (tmp_path / "20261020" / "20261020_100000_mocap_floor_calibration_frame_20261020_120000.json").write_text(
        json.dumps({"floor_from_raw": {"position": [1.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}}))
    frame, skipped = newest_floor_calibration(tmp_path)
    assert frame.floor_file.name.startswith("20261020") and skipped == []
    assert frame.floor_from_raw.position == FLOOR_FROM_RAW[0] and frame.anchors == ANCHORS and frame.cameras == LAB


def test_an_older_floor_calibration_is_read_back_to_raw(tmp_path):
    """floor_from_mocap, cameras and anchors as published under relay_correction: floor_from_raw composed, raw back."""
    content = {"kind": "mocap_floor_calibration",
               "cameras": json.loads(cameras_json(list(LAB.values()), 1.0))["cameras"],
               "floor_from_mocap": {"position": [0.0, 0.0, 0.001], "orientation": [0.0, 0.0, 0.0, 1.0]},
               "relay_correction": [[0.02, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]], "anchors": {"A1": [1.02, 2.0, 0.0]}}
    (tmp_path / "20261027").mkdir()
    (tmp_path / "20261027" / f"20261027_160000{FLOOR_SUFFIX}").write_text(json.dumps(content))
    frame, _skipped = newest_floor_calibration(tmp_path)
    np.testing.assert_allclose(frame.anchors["A1"].position, (1.0, 2.0, 0.0), atol=1e-12)
    np.testing.assert_allclose(frame.floor_from_raw.position, (0.02, 0.0, 0.001), atol=1e-12)


def test_floor_calibrations_are_found_in_the_dated_sessions_beside_the_base_calibrations(tmp_path):
    """The newest across the session folders; one outside a dated session folder never counts."""
    floor_file(tmp_path, "20261020_100000", LAB)
    newest = floor_file(tmp_path, "20261027_090000", LAB)
    (tmp_path / "20261027" / "calibrated_transformation_0804_rhino.json").write_text("{}")
    (tmp_path / "archive").mkdir()
    (tmp_path / "archive" / f"20261030_090000{FLOOR_SUFFIX}").write_text(newest.read_text())
    assert newest_floor_calibration(tmp_path)[0].floor_file == newest


def test_a_simulated_floor_calibration_in_the_robots_folder_is_passed_over(tmp_path):
    """Marked sim in the robots' sessions: passed over with why, never reaching a robot; in a simulation's, read."""
    robots = tmp_path / sessions_folder(False)
    floor_file(robots, "20261020_090000", LAB)
    floor_file(robots, "20261027_090000", LAB, environment="sim")
    frame, skipped = newest_floor_calibration(robots)
    assert frame.floor_file.name.startswith("20261020") and "made in a simulation" in skipped[0]
    floor_file(tmp_path / sessions_folder(True, 70), "20261027_090000", LAB, environment="sim")
    assert newest_floor_calibration(tmp_path / sessions_folder(True, 70))[0] is not None


def test_a_newest_floor_calibration_that_does_not_read_falls_back_to_the_one_before(tmp_path):
    """Half-synced: passed over, with why; the one before it is read, so poses do not stay raw."""
    floor_file(tmp_path, "20261020_100000", LAB)
    floor_file(tmp_path, "20261027_090000", LAB).write_text('{"cameras": [{"name": ')
    frame, skipped = newest_floor_calibration(tmp_path)
    assert frame.floor_file.name.startswith("20261020") and len(skipped) == 1 and "half-synced" in skipped[0]


def test_every_lab_camera_has_its_own_name_that_matches_where_it_hangs():
    """Ten names, none twice; T above 4 m, L/R by x (+x is left); F, M or B along y (+y is back)."""
    assert len(set(CAMERA_NAMES.values())) == len(CAMERA_NAMES) == len(LAB)
    for camera in LAB.values():
        x, y, z = camera.position
        name = CAMERA_NAMES[camera_serial(camera.name)]
        assert name[0] == ("T" if z > 4.0 else "B") and name[2] == ("L" if x > 0.0 else "R")
        # * The middle ones hang within 2 m of the centre; BBR, 1 m back of it, is the back-most on its wall.
        assert name[1] == ("F" if y < -2.0 else "B" if y > 2.0 or name == "BBR" else "M")


def test_a_camera_is_shown_the_labs_way_or_as_motive_names_it():
    """A known serial: its name and serial; an unknown camera: Motive's name."""
    assert camera_label("PrimeX 22 #72318") == "BMR #72318" and camera_label("Slim 3U #1") == "Slim 3U #1"


def test_anchors_are_found_by_the_nearest_marker_within_reach():
    """1 cm off is found, 3 cm off is not; a marker as near two anchors is neither's."""
    lab = {"A": MocapAnchor("A", (0.0, 0.0, 0.0)), "B": MocapAnchor("B", (1.0, 0.0, 0.0)),
           "C": MocapAnchor("C", (2.0, 0.0, 0.0)), "D": MocapAnchor("D", (2.03, 0.0, 0.0))}
    assert find_anchors(lab, [(0.01, 0.0, 0.0), (1.03, 0.0, 0.0), (2.015, 0.0, 0.0)]) == {"A": 0}


def test_a_moved_frame_is_recovered_from_the_anchors_and_a_moved_anchor_left_out():
    """The frame moved 1 cm and turned 0.3 deg, A2 3 mm on its own: four anchors give the motion."""
    now = {name: MocapAnchor(name, camera.position) for name, camera in recalibrated(
        {name: MocapCamera(name, anchor.position, (0.0, 0.0, 0.0, 1.0)) for name, anchor in ANCHORS.items()},
        (0.01, 0.0, 0.0), (0.0, 0.0, 0.3)).items()}
    now["A2"] = MocapAnchor("A2", tuple(np.add(now["A2"].position, (0.003, 0.0, 0.0))))
    fit = fit_anchors(ANCHORS, now)
    assert fit.trusted and fit.source == "anchors" and list(fit.bumped) == ["A2"] and fit.residual < 1e-6
    assert "from 4 anchors" in fit.describe() and "A2 moved 3.0 mm against the others" in fit.describe()
    np.testing.assert_allclose(back(fit, now["A5"].position), ANCHORS["A5"].position, atol=1e-6)


def test_warping_shows_in_the_largest_residual():
    """Motive sees the truss 0.1 % bigger: no rigid fit takes that out, so the residual shows it."""
    centre = np.mean([camera.position for camera in LAB.values()], axis=0)
    grown = {name: MocapCamera(name, tuple(centre + 1.001 * (np.asarray(camera.position) - centre)),
                               camera.orientation) for name, camera in LAB.items()}
    assert fit_cameras(LAB, grown).residual > 2e-3 and fit_cameras(LAB, LAB).residual < 1e-9


def test_the_labs_anchors_recorded_to_about_a_millimetre_all_stay_in_the_fit():
    """27 Oct: the six anchors read again were 0.2 to 1.2 mm off their record: all used, held, ± 2 mm at a corner."""
    recorded = {"A1": (-0.3413, 2.0523, 1.1459), "A2": (-0.3803, 2.6154, 1.1851), "A3": (2.0987, 2.6008, 1.1644),
                "A4": (3.4381, -1.6487, 0.012), "A5": (3.5079, -1.9801, 0.0127), "A6": (2.1075, 2.011, 1.1486)}
    off_mm = {"A1": (0.49, 0.27, -0.24), "A2": (0.18, -0.13, -0.01), "A3": (-0.58, -1.01, 0.06),
              "A4": (-1.09, -0.07, 0.16), "A5": (-0.91, 0.09, 0.35), "A6": (-0.56, -1.04, -0.39)}
    then = {name: MocapAnchor(name, position) for name, position in recorded.items()}
    now = {name: MocapAnchor(name, tuple(np.add(position, np.multiply(off_mm[name], 1e-3))))
           for name, position in recorded.items()}
    fit = fit_anchors(then, now)
    assert len(fit.used) == 6 and not fit.bumped and not frame_moved(fit)
    assert volume_moved(fit.raw_cal_from_raw_now) < 0.002 and fit.uncertainty == pytest.approx(0.0012, abs=3e-4)


#: The lab's six anchors, 27 Oct, lab frame.
LAB_ANCHORS = {"A1": (-0.3394, 2.0528, 1.1446), "A2": (-0.3795, 2.6152, 1.1838), "A3": (2.1021, 2.6017, 1.1636),
               "A4": (3.4379, -1.6486, 0.0116), "A5": (3.5081, -1.9797, 0.0125), "A6": (2.1106, 2.011, 1.1473)}


def anchors_off(off_mm: dict) -> tuple[dict, dict]:
    """The lab's anchors as recorded, and now with some moved on their own by `off_mm` (name: (x, y, z) mm)."""
    then = {name: MocapAnchor(name, position) for name, position in LAB_ANCHORS.items()}
    now = {name: MocapAnchor(name, tuple(np.add(position, np.multiply(off_mm.get(name, (0, 0, 0)), 1e-3))))
           for name, position in LAB_ANCHORS.items()}
    return then, now


def test_one_anchor_bumped_is_left_out_and_the_other_five_trusted():
    """A4 kicked 8 mm: left out, named; the five others agree exactly, the frame held."""
    fit = fit_anchors(*anchors_off({"A4": (8.0, 0.0, 0.0)}))
    assert list(fit.bumped) == ["A4"] and len(fit.used) == 5 and fit.trusted and fit.uncertainty < 1e-3


def test_two_tight_pairs_left_pin_a_turn_far_worse_than_all_six():
    """27 Oct, 18:32: A3 and A6 3 mm off; A1 A2 A4 A5 agree but are two pairs: three times the uncertainty of six."""
    fit = fit_anchors(*anchors_off({"A3": (0.0, 3.0, 0.0), "A6": (3.0, 0.0, 0.0)}))
    six = fit_anchors(*anchors_off({"A3": (0.0, 0.3, 0.0), "A6": (0.3, 0.0, 0.0)}))
    assert sorted(fit.bumped) == ["A3", "A6"] and fit.uncertainty > 3 * six.uncertainty


def test_anchors_that_do_not_agree_are_not_trusted():
    """Three of six moved 5 mm each, apart: no four agree, so not trusted, saying so."""
    fit = fit_anchors(*anchors_off({"A1": (5.0, 0, 0), "A3": (0, 5.0, 0), "A5": (0, 0, 5.0)}))
    assert not fit.consistent and not fit.trusted and "no 4 anchors agree" in fit.describe()


def test_a_scattered_fit_moves_the_frame_only_beyond_twice_its_uncertainty():
    """A fit whose uncertainty is 1.5 mm moving the volume 2 mm: not a move; 4 mm: a move."""
    fit = fit_anchors(*anchors_off({}))
    for shift, moved in ((0.002, False), (0.004, True)):
        assert frame_moved(replace(fit, raw_cal_from_raw_now=Pose((shift, 0.0, 0.0)), uncertainty=0.0015)) == moved


@pytest.mark.parametrize("off_mm", [{}, {"A3": (0.0, 3.0, 0.0), "A6": (3.0, 0.0, 0.0)},
                                    {"A1": (5.0, 0, 0), "A3": (0, 5.0, 0), "A5": (0, 0, 5.0)}])
def test_every_fit_goes_into_json(off_mm):
    """Trusted, too uncertain, or not consistent: it goes into the relay's announcement and the refit file as JSON."""
    entry = json.loads(json.dumps(fit_anchors(*anchors_off(off_mm)).as_dict()))
    assert isinstance(entry["trusted"], bool) and isinstance(entry["consistent"], bool)
