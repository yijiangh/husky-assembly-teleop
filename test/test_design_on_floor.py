"""Checks for the Rhino design placed on the measured floor (mocap_floor + bar_action_io).

The design is drawn on a flat ground (its WalkableGround, -15.55 mm); the real
floor is tilted (0.49 deg on the 22 Oct file, 0.26 deg on the 27 Oct one) and up to
~2 cm off it, so the ground bars' footers float above it or sink into it by
millimetres. With the placement on (``set_world_from_design``, as the live monitor
does), every footer must sit on the measured floor.

* A made-up floor (no files): the placement's maths.
* The real floor file and the 2026-10 exports (skipped where the shared drive is
  not mounted): the footers of 261006 and 261019 OFF vs ON, the sidecar rule
  (MoCap-sourced poses never move), a save + reload round trip, and the Rhino
  markers moving with their bar.
! One RobotCell.json (261019, ~360 MB, ~3 s) is loaded once, only for the ground
! joints' footer meshes; 261006 uses the same joint models.
"""
import os
import shutil

import numpy as np
import pytest
from compas.data import json_load
from compas.geometry import Transformation

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, MOCAP_FLOOR_CALIBRATION_FILE
from husky_assembly_teleop.bar_action_io import (
    MONITOR_GROUND_BODY, get_world_from_design, is_ground_joint_body, parse_bar_action,
    set_world_from_design, write_bar_action,
)
from husky_assembly_teleop.bar_mocap_fit import load_mocap_markers
from husky_assembly_teleop.cfab_session import GROUND_RIGID_BODY_NAME, walkable_ground_path
from husky_assembly_teleop.mocap_floor import MocapFloor, design_ground_height, load_mocap_floor

PROBLEMS = ('261006_3bar_holding_test', '261019_2ModulesDemo')
# The release files start with the bar inserted: every ground joint built so far is placed.
RELEASE_FILES = {'261006_3bar_holding_test': ('B20__R.json', 'B34__R.json'),
                 '261019_2ModulesDemo': ('B20__R.json', 'B34__R.json', 'B46__R.json')}
# Footer bottoms above the measured floor with the design as exported, mm, per floor file:
# on the 22 Oct floor (checked 2026-10-23) the design floated 6 / 14 / 23 mm high; on the
# 27 Oct floor (fitted to the fixed floor points, 0.26 deg) it sits 0-10 mm LOW, most at B20;
# the 28 Oct floor (marks + anchors, 0.37 deg) is 2-5 mm lower there: -6 mm at B20 to +9 mm at B46.
EXPORTED_GAP_MM = {
    '20261022_161428_mocap_floor_calibration.json': {'G20': (5.0, 7.0), 'G34': (13.5, 15.5), 'G46': (21.5, 24.0)},
    '20261027_160043_mocap_floor_calibration.json': {'G20': (-9.6, -6.1), 'G34': (-6.1, -2.6), 'G46': (-2.6, 0.9)},
    '20261028_152335_mocap_floor_calibration.json': {'G20': (-6.2, -0.1), 'G34': (-1.4, 4.7), 'G46': (3.5, 9.6)},
}.get(os.path.basename(MOCAP_FLOOR_CALIBRATION_FILE), {})
CELL_PROBLEM = '261019_2ModulesDemo'
SIDECAR = os.path.join(DESIGN_DATA_DIRECTORY, '261006_3bar_holding_test', 'BarActions',
                       'B20__J.live-solved.json')

needs_real_data = pytest.mark.skipif(
    not (os.path.exists(MOCAP_FLOOR_CALIBRATION_FILE)
         and all(os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, p)) for p in PROBLEMS)),
    reason='the floor calibration or the design-study exports are not on this machine',
)


@pytest.fixture(autouse=True)
def placement_off_after_each_test():
    """Every test starts and ends with the files read as they are (the offline default)."""
    set_world_from_design(None)
    yield
    set_world_from_design(None)


def _made_up_floor(tilt_deg: float = 1.0, height_at_origin: float = 0.010) -> MocapFloor:
    """A floor tilted about the MoCap x axis (rising towards +y), ``height_at_origin`` above the origin."""
    a = np.radians(tilt_deg)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1, 0, 0], [0, np.cos(a), np.sin(a)], [0, -np.sin(a), np.cos(a)]]
    matrix[:3, 3] = -matrix[:3, :3] @ np.array([0.0, 0.0, height_at_origin])
    return MocapFloor(floor_from_mocap=matrix, samples=np.zeros((0, 3)), rms_m=0.0,
                      tilt_deg=tilt_deg, path='made up')


def _apply(matrix: np.ndarray, point) -> np.ndarray:
    """A 4x4 applied to a 3D point."""
    return matrix[:3, :3] @ np.asarray(point, dtype=float) + matrix[:3, 3]


def test_placement_lays_the_design_ground_on_the_floor():
    floor = _made_up_floor()
    ground_z = -0.01555
    C = floor.world_from_design(ground_z)
    assert np.allclose(C[:3, :3] @ C[:3, :3].T, np.eye(3))    # a rigid move
    for x, y in [(0, 0), (3, -4), (-2, 1.5)]:
        # A point of the design's ground lands on the floor; one 0.2 m above it, 0.2 m above the floor.
        assert floor.height_above(_apply(C, [x, y, ground_z])) == pytest.approx(0.0, abs=1e-12)
        assert floor.height_above(_apply(C, [x, y, ground_z + 0.2])) == pytest.approx(0.2, abs=1e-12)
    # The design ground under the MoCap origin only moves up or down, onto the floor there.
    assert np.allclose(_apply(C, [0, 0, ground_z]), [0, 0, floor.z_at(0, 0)], atol=1e-12)


def test_ground_body_names_agree():
    # bar_action_io keeps its own copy (it cannot import cfab_session).
    assert MONITOR_GROUND_BODY == GROUND_RIGID_BODY_NAME


@needs_real_data
def test_design_ground_height_reads_the_flat_ground():
    for problem in PROBLEMS + ('260920_RobArch_demo_revamp_backup',):
        # 260920 also has a wall-like face (z +-3 m): only the flat ones count.
        if os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, problem)):
            assert design_ground_height(walkable_ground_path(problem)) == pytest.approx(-0.01555, abs=1e-6)


@pytest.fixture(scope='module')
def ground_joint_models():
    """The ground joints' rigid-body models, from one cell file (only loaded when a test needs them)."""
    if not os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, CELL_PROBLEM)):
        pytest.skip('the design-study exports are not on this machine')
    cell = json_load(os.path.join(DESIGN_DATA_DIRECTORY, CELL_PROBLEM, 'RobotCell.json'))
    return {name: model for name, model in cell.rigid_body_models.items() if is_ground_joint_body(name)}


def _footer_heights_mm(problem: str, floor: MocapFloor, models: dict) -> dict:
    """Every ground joint's lowest point above the measured floor, mm, as the release files place them.

    Returns:
        dict: ``{joint body name: (min, max) height of its lowest mesh points}``.
    """
    out = {}
    for fname in RELEASE_FILES[problem]:
        state = parse_bar_action(os.path.join(DESIGN_DATA_DIRECTORY, problem, 'BarActions', fname)).movements[0].start_state
        for name, rb in state.rigid_body_states.items():
            if name in out or name not in models or rb.frame is None:
                continue
            model = models[name]
            T = np.array(Transformation.from_frame(rb.frame).matrix)
            V = np.vstack([np.array([m.vertex_coordinates(k) for k in m.vertices()])
                           for m in (model.collision_meshes or model.visual_meshes)]) * (model.native_scale or 1.0)
            W = (T[:3, :3] @ V.T).T + T[:3, 3]
            # The footer's flat bottom: its lowest points (four corners).
            bottom = W[W[:, 2] <= W[:, 2].min() + 1e-4]
            h = floor.height_above(bottom) * 1000.0
            out[name] = (float(h.min()), float(h.max()))
    return out


@needs_real_data
@pytest.mark.parametrize('problem', PROBLEMS)
def test_footers_float_as_exported_and_touch_the_floor_when_placed(problem, ground_joint_models):
    floor = load_mocap_floor()
    off = _footer_heights_mm(problem, floor, ground_joint_models)
    ground_z = design_ground_height(walkable_ground_path(problem))
    set_world_from_design(floor.world_from_design(ground_z))
    on = _footer_heights_mm(problem, floor, ground_joint_models)
    expected = {g for g in EXPORTED_GAP_MM if any(f'_{g}-' in n for n in off)}
    assert expected, 'no ground joint found'
    for name, (lo, hi) in off.items():
        g = name.split('_')[1].split('-')[0]
        print(f"{problem} {name}: as exported {lo:.1f}..{hi:.1f} mm, placed {on[name][0]:+.3f}..{on[name][1]:+.3f} mm")
        # * As exported: off the floor by the measured floor's tilt and offset there.
        assert EXPORTED_GAP_MM[g][0] <= lo <= hi <= EXPORTED_GAP_MM[g][1]
        # * Placed: on the floor (the footer's flat bottom, every corner).
        assert abs(on[name][0]) < 0.01 and abs(on[name][1]) < 0.01


@needs_real_data
def test_sidecar_moves_rhino_poses_only():
    if not os.path.isfile(SIDECAR):
        pytest.skip('no 261006 B20 sidecar on this machine')
    floor = load_mocap_floor()
    C = floor.world_from_design(design_ground_height(walkable_ground_path('261006_3bar_holding_test')))
    as_is = parse_bar_action(SIDECAR)
    set_world_from_design(C)
    placed = parse_bar_action(SIDECAR)
    moved_bars = 0
    for mv_a, mv_p in zip(as_is.movements, placed.movements):
        a, p = mv_a.start_state, mv_p.start_state
        if a is None:
            continue
        # ! MoCap-sourced: the live base, the other robots (progress.json), the monitor's floor.
        assert np.allclose(a.robot_base_frame.point, p.robot_base_frame.point, atol=1e-12)
        for name, tool in a.tool_states.items():
            if tool.frame is not None and not tool.attached_to_group:
                assert np.allclose(tool.frame.point, p.tool_states[name].frame.point, atol=1e-12)
        if MONITOR_GROUND_BODY in a.rigid_body_states:
            assert a.rigid_body_states[MONITOR_GROUND_BODY].frame == p.rigid_body_states[MONITOR_GROUND_BODY].frame
        # Rhino: the bars, by the placement.
        for name, rb in a.rigid_body_states.items():
            if name.startswith('bar_') and rb.frame is not None:
                assert np.allclose(_apply(C, rb.frame.point), p.rigid_body_states[name].frame.point, atol=1e-9)
                moved_bars += 1
    assert moved_bars


@needs_real_data
def test_save_and_reload_round_trips(tmp_path):
    problem_dir = os.path.join(DESIGN_DATA_DIRECTORY, '261006_3bar_holding_test', 'BarActions')
    clean = os.path.join(str(tmp_path), 'B20__J.json')
    shutil.copy(os.path.join(problem_dir, 'B20__J.json'), clean)
    floor = load_mocap_floor()
    set_world_from_design(floor.world_from_design(design_ground_height(walkable_ground_path('261006_3bar_holding_test'))))
    loaded = parse_bar_action(clean)
    # A live value the monitor writes in: a MoCap base.
    loaded.movements[1].start_state.robot_base_frame.point = [1.234, -2.345, 0.0123]
    sidecar = os.path.join(str(tmp_path), 'B20__J.live-solved.json')
    write_bar_action(loaded, sidecar)
    reread = parse_bar_action(sidecar)
    for mv_l, mv_r in zip(loaded.movements, reread.movements):
        sl, sr = mv_l.start_state, mv_r.start_state
        if sl is None:
            continue
        assert np.allclose(sl.robot_base_frame.point, sr.robot_base_frame.point, atol=1e-12)
        for name, rb in sl.rigid_body_states.items():
            if rb.frame is not None:
                assert np.allclose(rb.frame.point, sr.rigid_body_states[name].frame.point, atol=1e-9)
                assert np.allclose(rb.frame.xaxis, sr.rigid_body_states[name].frame.xaxis, atol=1e-9)
        for side, f in (mv_l.target_ee_frames or {}).items():
            assert np.allclose(f.point, mv_r.target_ee_frames[side].point, atol=1e-9)
    # On disk the Rhino poses stay in the export's frame; the MoCap base is stored as it was.
    set_world_from_design(None)
    on_disk, export = parse_bar_action(sidecar), parse_bar_action(clean)
    for mv_d, mv_e in zip(on_disk.movements, export.movements):
        if mv_d.start_state is None:
            continue
        for name, rb in mv_e.start_state.rigid_body_states.items():
            if rb.frame is not None:
                assert np.allclose(rb.frame.point, mv_d.start_state.rigid_body_states[name].frame.point, atol=1e-9)
    assert np.allclose(on_disk.movements[1].start_state.robot_base_frame.point, [1.234, -2.345, 0.0123])
    assert get_world_from_design() is None


@needs_real_data
def test_rhino_markers_move_with_their_bar():
    # MoCapMarkers.json holds each ball in its bar's frame (world = bar frame @ bar_m), so the
    # Rhino markers follow the placed bar: no file to change.
    problem_dir = os.path.join(DESIGN_DATA_DIRECTORY, CELL_PROBLEM)
    bar_m = load_mocap_markers(problem_dir).bars['bar_B20'].points()    # (8, 3), the bar's frame
    path = os.path.join(problem_dir, 'BarActions', 'B20__R.json')
    floor = load_mocap_floor()
    C = floor.world_from_design(design_ground_height(walkable_ground_path(CELL_PROBLEM)))

    def markers_in_world() -> np.ndarray:
        frame = parse_bar_action(path).movements[0].start_state.rigid_body_states['bar_B20'].frame
        T = np.array(Transformation.from_frame(frame).matrix)
        return (T[:3, :3] @ bar_m.T).T + T[:3, 3]

    exported = markers_in_world()
    set_world_from_design(C)
    placed = markers_in_world()
    assert np.allclose(placed, (C[:3, :3] @ exported.T).T + C[:3, 3], atol=1e-9)
    # The balls move by the floor's offset under B20 (y ~ -2.3 m): a few mm, either way
    # depending on the floor file (down on the 22 Oct floor, up on the 27 Oct one).
    assert 0.0 < abs((placed - exported)[:, 2].mean()) < 0.02


@needs_real_data
def test_compare_script_reparses_in_the_takes_frame():
    # 1_compare_to_cell_state.py re-parses the action for takes without a stamped bar pose (or with
    # --bar-action): it must place the design the way the take's monitor did (its world_from_design).
    import importlib.util
    script = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'data', 'bar_holding_acc_data', '1_compare_to_cell_state.py')
    spec = importlib.util.spec_from_file_location('compare_to_cell_state', script)
    compare = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compare)
    path = os.path.join(DESIGN_DATA_DIRECTORY, '261006_3bar_holding_test', 'BarActions', 'B20__R.json')
    C = load_mocap_floor().world_from_design(design_ground_height(walkable_ground_path('261006_3bar_holding_test')))

    def goal(world_from_design):
        _idx, mv, action, _path = compare._resolve_take_movement_placed(path, 0, world_from_design)
        return compare.goal_bar_pose_from_movement(mv, f"bar_{action.active_bar_id}")[0]

    as_exported, placed = goal(None), goal(C.tolist())
    assert np.allclose(placed[0], _apply(C, as_exported[0]), atol=1e-9)
    # The placement never leaks into a later parse.
    assert get_world_from_design() is None


@pytest.mark.skipif(not os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, '261023_2ModulesDemo'))
                    or not os.path.exists(MOCAP_FLOOR_CALIBRATION_FILE),
                    reason='261023 or the floor calibration is not on this machine')
def test_261023_footers_stand_15_mm_above_the_floor(ground_joint_models):
    # * 261023 = 261019 moved by (+585.7, +1387.1, +15.55) mm: its footers stand at z = 0 while its
    # * WalkableGround (and the robots' planned bases) stayed at -15.55 mm. User decision 2026-10-23:
    # * keep WalkableGround as the design ground, so every footer ends 15.55 mm ABOVE the real floor
    # * (safer: never mashed); the three ground bars are checked on site afterwards.
    problem = '261023_2ModulesDemo'
    floor = load_mocap_floor()
    ground_z = design_ground_height(walkable_ground_path(problem))
    assert ground_z == pytest.approx(-0.01555, abs=1e-6)
    RELEASE_FILES[problem] = ('B20__R.json', 'B34__R.json', 'B46__R.json')
    off = _footer_heights_mm(problem, floor, ground_joint_models)
    set_world_from_design(floor.world_from_design(ground_z))
    on = _footer_heights_mm(problem, floor, ground_joint_models)
    assert len(on) == 6
    for name, (lo, hi) in on.items():
        print(f"{problem} {name}: as exported {off[name][0]:.1f}..{off[name][1]:.1f} mm, placed {lo:.3f}..{hi:.3f} mm")
        assert lo == pytest.approx(15.55, abs=0.01) and hi == pytest.approx(15.55, abs=0.01)


@pytest.mark.skipif(not os.path.isdir(os.path.join(DESIGN_DATA_DIRECTORY, '261023_2ModulesDemo'))
                    or not os.path.exists(MOCAP_FLOOR_CALIBRATION_FILE),
                    reason='261023 or the floor calibration is not on this machine')
def test_261023_ground_raise_lowers_the_footers(ground_joint_models):
    # * DESIGN_GROUND_RAISE_MM (2026-10-23: 23 mm, from B20's measured ~23 mm gap): the WalkableGround
    # * is raised before it is laid on the floor, so the design sits that much lower. By the model the
    # * footers then sit 15.55 - 23 = -7.45 mm against the fitted floor plane (on the real floor at B20).
    from husky_assembly_teleop import DESIGN_GROUND_RAISE_MM
    problem = '261023_2ModulesDemo'
    floor = load_mocap_floor()
    ground_z = design_ground_height(walkable_ground_path(problem)) + DESIGN_GROUND_RAISE_MM / 1000.0
    RELEASE_FILES[problem] = ('B20__R.json', 'B34__R.json', 'B46__R.json')
    set_world_from_design(floor.world_from_design(ground_z))
    on = _footer_heights_mm(problem, floor, ground_joint_models)
    assert len(on) == 6
    for name, (lo, hi) in on.items():
        assert lo == pytest.approx(15.55 - DESIGN_GROUND_RAISE_MM, abs=0.01)
        assert hi == pytest.approx(15.55 - DESIGN_GROUND_RAISE_MM, abs=0.01)
