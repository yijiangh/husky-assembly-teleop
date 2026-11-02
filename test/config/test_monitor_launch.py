"""monitor.launch.py offers every monitor parameter and passes on only the ones given, typed."""

import importlib.util

import pytest
from repo_paths import REPO

from husky_assembly_teleop.config import _parse_tools, config_from_ros_parameters

_SPEC = importlib.util.spec_from_file_location("monitor_launch", REPO / "launch" / "monitor.launch.py")
LAUNCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(LAUNCH)


class _Declared(Exception):
    """Raised at the first read, once every parameter is declared."""


class _DeclaringNode:
    """Keeps the names the monitor declares."""

    def __init__(self):
        self.names = []

    def declare_parameter(self, name, _default):
        self.names.append(name)

    def get_parameter(self, _name):
        raise _Declared


def test_the_launch_offers_every_monitor_parameter():
    """An argument per parameter config.py declares, each once."""
    node = _DeclaringNode()
    with pytest.raises(_Declared):
        config_from_ros_parameters(node)
    names = [name for name, _kind, _description in LAUNCH.PARAMETERS]
    assert sorted(names) == sorted(node.names) and len(set(names)) == len(names)


def test_empty_arguments_leave_the_monitors_defaults():
    """An argument left empty is not passed on."""
    assert LAUNCH.parameters({"robots": "", "viser_port": " "}) == {}


def test_lists_read_with_spaces_commas_or_in_the_ros_args_form():
    """Spaces, commas and the --ros-args form give the same lists; [] gives no robots."""
    expected = {"robots": ["alice", "belle"], "plugins": ["robot_control"],
                "tools": ["alice:scaffolding_v3", "cindy:scaffolding_v3:none"]}
    for robots, tools in (("alice belle", "alice:scaffolding_v3  cindy:scaffolding_v3:none"),
                          ("alice, belle", "alice:scaffolding_v3,cindy:scaffolding_v3:none"),
                          ("['alice','belle']", "['alice:scaffolding_v3', 'cindy:scaffolding_v3:none']")):
        assert LAUNCH.parameters({"robots": robots, "plugins": "robot_control", "tools": tools}) == expected
    assert LAUNCH.parameters({"robots": "[]"}) == {"robots": [""]}


def test_tools_entries_give_each_robot_one_tool_per_arm():
    """The monitor reads the launch's tools entries per robot, in arm order, none for a bare arm."""
    entries = LAUNCH.parameters({"tools": "alice:scaffolding_v3 cindy:scaffolding_v3:none"})["tools"]
    assert _parse_tools(tuple(entries)) == {"0804": ("scaffolding_v3",), "0806": ("scaffolding_v3", None)}


def test_numbers_and_flags_are_typed():
    """Ports are ints, timeouts floats, flags true or false and nothing else."""
    assert LAUNCH.parameters({"viser_port": "8095", "ghost_timeout": "0", "no_default_plugins": "True"}) == {
        "viser_port": 8095, "ghost_timeout": 0.0, "no_default_plugins": True}
    with pytest.raises(ValueError):
        LAUNCH.parameters({"no_default_plugins": "yes"})
