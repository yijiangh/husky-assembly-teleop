"""The examples keep their chips in a handle of their own, so live numbers beside them never drop a chip's hover."""

from types import SimpleNamespace

from husky_assembly_teleop.plugins.examples.robot_state import ExampleRobotStatePlugin


def _robot(x: float) -> SimpleNamespace:
    """A robot whose base is tracked at `x` with a fresh fix, and one arm under a controller."""
    arm = SimpleNamespace(state=SimpleNamespace(controllers=SimpleNamespace(active="scaled_jtc")),
                          joint_vector=lambda: None)
    return SimpleNamespace(base=SimpleNamespace(state=SimpleNamespace(position=(x, 0.0, 0.0), last_update_time=9.9,
                                                                      tracked=True)),
                           arms={"ur_arm": arm})


def test_robot_state_chips_stay_while_the_pose_changes():
    """Only the numbers' handle changes when the base moves; the chips' handle keeps its HTML, each chip a title."""
    plugin = ExampleRobotStatePlugin()
    plugin._chips, plugin._rows = {"0806": SimpleNamespace(content="")}, {"0806": SimpleNamespace(content="")}
    drawn = []
    for x in (1.0, 1.2):
        plugin.draw(SimpleNamespace(now=lambda: 10.0, world=SimpleNamespace(robots={"0806": _robot(x)})))
        drawn.append((plugin._chips["0806"].content, plugin._rows["0806"].content))
    (chips, numbers), (chips_after, numbers_after) = drawn
    assert chips == chips_after and numbers != numbers_after
    assert 'title=""' not in chips
