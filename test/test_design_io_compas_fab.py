"""T5: the producer's own export, converted to schema 1 and back to compas_fab, gives the same cells.

For every movement of the 260814 export: each body's hidden flag, attachment and frame, each other
robot's placement and joints, and the acting robot's base and joints match the original start state;
and compas_fab's `check_collision` finds the same pairs in both cells.

Skipped when the export is not under the Drive root (HUSKY_DRIVE_ROOT) on this machine.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop.drive import drive_root

EXPORT = (drive_root() or Path("/missing")) / "data_design_study/260814_RobArch_support_ik"
pytestmark = [pytest.mark.skipif(not (EXPORT / "ActionSchedule.json").is_file(), reason="export not on this machine"),
              pytest.mark.slow]

DATA = Path(__file__).resolve().parent.parent / "data"


def _same_frame(a, b, tolerance: float = 1e-6) -> bool:
    """Whether two compas frames are equal within a tolerance (points and axes)."""
    return all(np.allclose(list(x), list(y), atol=tolerance)
               for x, y in ((a.point, b.point), (a.xaxis, b.xaxis), (a.yaxis, b.yaxis)))


@pytest.fixture(scope="module")
def converted(tmp_path_factory):
    """The export converted and written, read back, with the original actions and cells."""
    from compas.data import json_load
    from bar_assembly_core.design_io.conversion import convert_export
    from bar_assembly_core.design_io import read

    folder = tmp_path_factory.mktemp("design")
    design = read(convert_export(EXPORT, folder, DATA, report=lambda _line: None))
    cells = {cell.robot_model.name: cell
             for cell in (json_load(str(path)) for path in sorted(EXPORT.glob("RobotCell*.json")))}
    originals = {}
    import json
    schedule = json.loads((EXPORT / "ActionSchedule.json").read_text())
    for entry in schedule["schedule"]:
        originals[entry["action_id"]] = json_load(str(EXPORT / entry["file"]))
    return design, cells, originals


def _legacy_names(design, acting: str):
    """Maps from the export's names to ours, for one acting robot's cell."""
    from bar_assembly_core.design_io.legacy import body_id

    def name(key: str) -> str:
        if key.startswith("ObstacleRobot"):
            return f"robots/{key[len('ObstacleRobot'):].lower()}"
        tools = {tool_id.split("/")[-1]: tool_id for tool_id in design.robots[acting].tools.values()}
        if key in tools:
            return tools[key]
        return body_id(key)
    return name


def test_states_match(converted):
    """Every start state of every movement comes back with the same frames and flags."""
    from bar_assembly_core.design_io.compas_fab import PARKED_POSITION, to_cell_state, to_robot_cell

    design, cells, originals = converted
    new_cells = {robot: to_robot_cell(design, robot) for robot in design.robots}
    checked = 0
    for action, movement in design.movements():
        original = next(m for m in originals[action.id].movements if m.movement_id == movement.id).start_state
        cell = new_cells[action.robot]
        ours = to_cell_state(design, action.robot, movement.start, cell)
        name = _legacy_names(design, action.robot)

        assert _same_frame(ours.robot_base_frame, original.robot_base_frame), movement.id
        if original.robot_configuration is None:
            assert ours.robot_configuration is None, movement.id
        else:
            configuration = original.robot_configuration
            for joint, value in zip(configuration.joint_names, configuration.joint_values):
                assert ours.robot_configuration[joint] == pytest.approx(value, abs=1e-9), (movement.id, joint)

        for key, body in original.rigid_body_states.items():
            mine = ours.rigid_body_states[name(key)]
            assert mine.is_hidden == body.is_hidden, (movement.id, key)
            if body.is_hidden:
                continue
            assert mine.attached_to_link == body.attached_to_link, (movement.id, key)
            if body.attached_to_link:
                assert _same_frame(mine.attachment_frame, body.attachment_frame), (movement.id, key)
            else:
                assert _same_frame(mine.frame, body.frame), (movement.id, key)
            assert set(mine.touch_links) >= set(body.touch_links), (movement.id, key)
            assert set(mine.touch_bodies) >= {name(other) for other in body.touch_bodies}, (movement.id, key)

        for key, tool in original.tool_states.items():
            mine = ours.tool_states[name(key)]
            if key.startswith("ObstacleRobot"):
                if np.allclose(list(tool.frame.point), PARKED_POSITION, atol=1e-3):
                    assert np.allclose(list(mine.frame.point), PARKED_POSITION), (movement.id, key)
                    continue
                assert _same_frame(mine.frame, tool.frame), (movement.id, key)
                for joint, value in zip(tool.configuration.joint_names, tool.configuration.joint_values):
                    assert mine.configuration[joint] == pytest.approx(value, abs=1e-9), (movement.id, key, joint)
            else:
                # ? Group names may differ (arm-only vs base-rooted, format R17); the flange must not.
                assert (cell.get_end_effector_link_name(mine.attached_to_group)
                        == cells[originals[action.id].robot_id].get_end_effector_link_name(tool.attached_to_group))
        checked += 1
    assert checked == 224


def test_collisions_match(converted):
    """compas_fab finds the same colliding pairs in the original and the converted cells."""
    from compas_fab.backends import CollisionCheckError, PyBulletClient, PyBulletPlanner

    from bar_assembly_core.design_io.compas_fab import to_cell_state, to_robot_cell

    design, cells, originals = converted
    not_ground = lambda body_id: not body_id.startswith("ground/")  # the export has no ground  # noqa: E731

    def pairs(planner, state, names) -> set:
        """Colliding pairs as our ids."""
        try:
            planner.check_collision(state, {"full_report": True})
        except CollisionCheckError as error:
            return {tuple(sorted((names(a), names(b)))) for a, b in error.collision_pairs}
        return set()

    compared = 0
    for robot in design.robots:
        movements = [(a, m) for a, m in design.movements() if a.robot == robot and m.start.robots[robot].joints]
        if not movements:
            continue
        legacy_cell = cells[originals[movements[0][0].id].robot_id]
        new_cell = to_robot_cell(design, robot, include=not_ground)
        name = _legacy_names(design, robot)
        with PyBulletClient("direct", verbose=False) as old_client, \
                PyBulletClient("direct", verbose=False) as new_client:
            old_planner, new_planner = PyBulletPlanner(old_client), PyBulletPlanner(new_client)
            old_planner.set_robot_cell(legacy_cell)
            new_planner.set_robot_cell(new_cell)
            old_models = [*legacy_cell.tool_models.items(), *legacy_cell.rigid_body_models.items()]
            old_ids = {id(value): name(key) for key, value in old_models}
            new_ids = {id(v): k for k, v in [*new_cell.tool_models.items(), *new_cell.rigid_body_models.items()]}

            def old_name(model):
                return old_ids.get(id(model), f"{robot}/{model.name}")

            def new_name(model):
                return new_ids.get(id(model), f"{robot}/{model.name}")

            for action, movement in movements:
                original = next(m for m in originals[action.id].movements if m.movement_id == movement.id).start_state
                old = pairs(old_planner, original, old_name)
                new = pairs(new_planner, to_cell_state(design, robot, movement.start, new_cell), new_name)
                assert old == new, (movement.id, sorted(old ^ new))
                compared += 1
    assert compared > 100
