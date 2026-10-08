"""
What a run is recorded with: its setup, read from ROS at Start, and its own folder for the recording.

- Sim or real: real unless a SIM_NODE runs in the robot's namespace; then its parameters are the sim model.
- Controller: the follower's node, as it names itself in BaseFollowerState, with all its parameters.
- Folder: <Drive root>/RECORDING_FOLDER/<experiment name>/<date>_<time>_<robot>_<template>_<sim|real>/
  recording.npz; `report.py` adds experiment.json and overview.png beside it.
- Every run also records CONDITIONS, which this experiment holds fixed (base, arm, floor).
- Runs with equal `setup_key` were made with the same environment, sim model, controller tuning and conditions;
  `environment_label` sorts them into the real robot, the ideal simulator, and a simulator with a model.
"""

from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path

import numpy as np

from ...plugin_api.context import PluginContext
from ...plugin_api.trace import Trace
from ...robot_interface.base import BaseInterface, parameter_value
from .plan import Plan

#: Recordings, under the Drive root (`drive.py`); ! create it in Drive, it is never created here.
RECORDING_FOLDER = Path("data_experiment/base_exp")
#: The simulator's node; running in the robot's namespace means the run is simulated.
SIM_NODE = "husky_sim"
#: Seconds to wait for a node's parameters.
PARAMETER_TIMEOUT = 1.0
#: The simulator parameters that make its model, with their ideal values; the rest (topics, start pose) do not
#: change results.
MODEL_PARAMETERS = {"xICR": 0.0, "speed_efficiency": 1.0, "steering_efficiency": 1.0, "cmd_delay_sec": 0.0,
                    "mocap_delay_sec": 0.0, "mocap_noise_position_m": 0.0, "mocap_noise_yaw_rad": 0.0}
#: Conditions held fixed for every run; ! change them here when the robot or the place changes (e.g. castors).
CONDITIONS = {"base": "skid steer (stock Husky wheels)", "arm": "stowed", "floor": "RobotX hall, tiles"}
#: Controller parameters that do not change how it drives.
NOT_TUNING = ("use_sim_time", "cmd_topic", "mocap_topic", "path_topic", "state_topic")


async def setup(ctx: PluginContext, base: BaseInterface) -> dict:
    """Where and with what the robot drives now: sim or real (and the sim model), controller and its parameters."""
    nodes = base.node_names()
    follower = base.state.follower
    controller = follower.controller if follower is not None and follower.controller else "unknown"
    result = {"environment": "sim" if SIM_NODE in nodes else "real",
              "controller": {"name": controller, "parameters": await parameters(ctx, base, controller)},
              "nodes": nodes}
    if SIM_NODE in nodes:
        result["sim_model"] = await parameters(ctx, base, SIM_NODE)
    return result


async def describe(ctx: PluginContext, base: BaseInterface, plan: Plan, **more) -> dict:
    """The run's record: its setup (see `setup`), robot, conditions, battery at start, the plan's settings.

    Args:
        ctx: The plugin's context, to wait for ROS.
        base: The robot's base.
        plan: The path about to be sent.
        more: Further entries, e.g. the clearances used or where the run stands in an automated series.

    Returns:
        dict: Plain values, ready for JSON.
    """
    linear, angular = plan.speed_caps
    battery = {"percentage": base.state.battery_percentage, "voltage": base.state.battery_voltage}
    return {"started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "robot": plan.serial, "conditions": dict(CONDITIONS),
            "battery": battery,
            **await setup(ctx, base),
            "template": plan.settings, "speed_caps": {"linear": linear, "angular": angular},
            "expected_duration": plan.expected, **more}


def sim_model(experiment: dict) -> dict[str, float] | None:
    """The simulator's model parameters of a run (ideal values where it did not have one), or None if real."""
    model = experiment.get("sim_model")
    if experiment.get("environment") != "sim" or not isinstance(model, dict):
        return None
    return {key: round(float(model.get(key, ideal)), 4) for key, ideal in MODEL_PARAMETERS.items()}


def environment_label(experiment: dict) -> str:
    """The environment's label: real, sim ideal (the model the controllers assume), or sim model (any other)."""
    model = sim_model(experiment)
    if model is None:
        return "real"
    return "sim ideal" if model == {key: float(ideal) for key, ideal in MODEL_PARAMETERS.items()} else "sim model"


def setup_key(experiment: dict) -> tuple:
    """What makes two runs comparable: environment, sim model, controller and its tuning, conditions (hashable)."""
    model = sim_model(experiment)
    model = tuple(model.values()) if model is not None else None
    controller = experiment.get("controller", {})
    tuning = controller.get("parameters")
    tuning = (tuple(sorted((k, str(v)) for k, v in tuning.items() if k not in NOT_TUNING))
              if isinstance(tuning, dict) else None)
    conditions = tuple(sorted(experiment.get("conditions", {}).items()))
    return experiment.get("environment"), model, controller.get("name"), tuning, conditions


def current_key(setup_now: dict) -> tuple:
    """The `setup_key` runs started now would get, from `setup`, as it reads back from their experiment.json."""
    return setup_key(json.loads(json.dumps({**setup_now, "conditions": dict(CONDITIONS)})))


def earlier_runs(recordings: Path, experiment_name: str, key: tuple) -> list[dict]:
    """The experiment.json of every saved run of an experiment with setup `key`, oldest first."""
    found = []
    for data in sorted((recordings / experiment_name).glob("*/experiment.json")):
        experiment = json.loads(data.read_text())
        if setup_key(experiment) == key:
            found.append(experiment)
    return found


async def parameters(ctx: PluginContext, base: BaseInterface, node: str) -> dict | str:
    """All parameters of a node in the robot's namespace by name, or why they could not be read."""
    listing = base.list_parameters(node)
    if listing is None:
        return f"{node} did not answer"
    try:
        names = list((await asyncio.wait_for(ctx.ros(listing), PARAMETER_TIMEOUT)).result.names)
        getting = base.get_parameters(node, names)
        if getting is None:
            return f"{node} did not answer"
        values = (await asyncio.wait_for(ctx.ros(getting), PARAMETER_TIMEOUT)).values
    except asyncio.TimeoutError:
        return f"{node} did not answer in time"
    return {name: parameter_value(value) for name, value in zip(names, values)}


def save(recordings: Path, rec: Trace, plan: Plan, experiment: dict, outcome: str,
         logs: dict[str, np.ndarray] | None = None) -> Path:
    """Save a run's recording, with the path sent and its setup, into a new folder of its own.

    Args:
        recordings: The experiments' folder, RECORDING_FOLDER under the Drive root.
        logs: Message logs over the run, saved as they are: "commands" (every follower report: received, stamp, v, w,
            state code) and "wheel_odometry" (received, stamp, v, w); see `BaseInterface.follower_log`.

    Returns:
        Path: The recording.npz written.

    Raises:
        OSError: If it cannot be written.
    """
    folder = (recordings / experiment.get("experiment_name", "default")
              / f"{time.strftime('%Y%m%d_%H%M%S')}_{plan.serial}_{plan.slug}_{experiment['environment']}")
    extra = {"path_poses": plan.poses, "path_t": np.array([]) if plan.t is None else plan.t,
             "path_label": np.array(plan.label), "outcome": np.array(outcome),
             "speed_caps": np.array(plan.speed_caps), "expected_duration": np.array(plan.expected),
             "experiment": np.array(json.dumps(experiment)), **(logs or {})}
    return rec.save(folder / "recording.npz", extra)
