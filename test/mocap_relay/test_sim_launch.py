"""sim.launch.py offers every argument of the launches it starts, as they declare them."""

import importlib.util

from launch.actions import DeclareLaunchArgument
from repo_paths import REPO


def _arguments(path) -> dict:
    """A launch file's arguments: default as text, by name."""
    spec = importlib.util.spec_from_file_location(path.name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {entity.name: "".join(part.text for part in entity.default_value)
            for entity in module.generate_launch_description().entities if isinstance(entity, DeclareLaunchArgument)}


def test_the_simulation_launch_offers_sim_robots_and_the_fake_motives_arguments():
    """Its arguments are sim_robot's and sim_mocap's with their defaults, plus fake_motive (off)."""
    sim = _arguments(REPO / "launch" / "sim.launch.py")
    module = importlib.util.spec_from_file_location("sim_launch", REPO / "launch" / "sim.launch.py")
    paths = importlib.util.module_from_spec(module)
    module.loader.exec_module(paths)
    expected = {**_arguments(paths.SIM_ROBOT), **_arguments(paths.FAKE_MOTIVE), "fake_motive": "false"}
    assert sim == expected and {"tool", "joint_offsets", "offsets", "delay_sec"} <= set(sim)
