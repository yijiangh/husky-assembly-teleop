"""
What a run is recorded with: its setup, read from ROS at Start, and its own folder for the recording.

- Sim or real: real unless the base's simulator announces itself (`BaseInterface.simulation`); then its node's
  parameters are the sim model, and a fake Motive's (`BaseInterface.mocap_simulation`) give its mocap delay and
  noise, 0 without one.
- Controller: the follower's node, as it names itself in BaseFollowerState, with all its parameters.
- Folder: <Drive root>/RECORDING_FOLDER/<experiment name>/<date>_<time>_<robot>_<template>_<sim|real>/ with
  experiment.json (this setup, the path's label, the outcome) and recording.npz (`shared/recordings/runs.py`);
  `report.py` adds report.json and overview.png beside them.
- Every run also records CONDITIONS, which this experiment holds fixed (base, arm, floor).
- Runs with equal `setup_key` were made with the same environment, sim model, controller tuning and conditions;
  `environment_label` sorts them into the real robot, the ideal simulator, and a simulator with a model.
"""

from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path

import numpy as np
from crl_husky.base_model import A200_LIMITS, LIMIT_KEYS, BaseModel

from ...drive import started_at
from ...plugin_api.context import PluginContext
from ...plugin_api.trace import Trace
from ...robot_interface.base import BaseInterface, parameter_value
from ...robot_interface.simulated_parts import body_values
from ...shared.recordings.runs import DESCRIPTION, run_folder, save_recording, write_description
from .plan import Plan

#: Recordings, under the Drive root (`drive.py`); ! create it in Drive, it is never created here.
RECORDING_FOLDER = Path("data_experiment/base_exp")
#: Seconds to wait for a node's parameters.
PARAMETER_TIMEOUT = 1.0
#: The simulator parameters that make its model, with their ideal values; the rest (topics, start pose) do not
#: change results.
MODEL_PARAMETERS = {"xICR": 0.0, "speed_efficiency": 1.0, "steering_efficiency": 1.0, "cmd_delay_sec": 0.0,
                    **{key: 0.0 for key in LIMIT_KEYS},
                    "mocap_delay_sec": 0.0, "mocap_noise_position_m": 0.0, "mocap_noise_yaw_rad": 0.0}
#: The sim model's mocap values from the fake Motive's parameters: (parameter, element of its per-body value or None
#: for a number, factor).
MOCAP_PARAMETERS = {"mocap_delay_sec": ("delay_sec", None, 1.0), "mocap_noise_position_m": ("noise", 0, 1.0),
                    "mocap_noise_yaw_rad": ("noise", 1, math.pi / 180.0)}
#: Conditions held fixed for every run; ! change them here when the robot or the place changes (e.g. castors).
CONDITIONS = {"base": "skid steer (stock Husky wheels)", "arm": "stowed", "floor": "RobotX hall, tiles"}
#: Controller parameters that do not change how it drives.
NOT_TUNING = ("use_sim_time", "cmd_topic", "mocap_topic", "path_topic", "state_topic")


async def setup(ctx: PluginContext, base: BaseInterface) -> dict:
    """Where and with what the robot drives now: sim or real (and the sim model), controller and its parameters."""
    follower = base.state.follower
    controller = follower.controller if follower is not None and follower.controller else "unknown"
    simulation = base.simulation
    result = {"environment": "sim" if simulation is not None else "real",
              "controller": {"name": controller, "parameters": await parameters(ctx, base, controller)},
              "nodes": base.node_names()}
    if simulation is not None:
        result["sim_model"] = await parameters(ctx, base, simulation.node)
        mocap = base.mocap_simulation
        if mocap is not None:
            result["sim_mocap"] = await parameters(ctx, base, mocap.node)
            result["sim_mocap_body"] = base.mocap_id
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
    return {"started": started_at(), "robot": plan.serial, "conditions": dict(CONDITIONS),
            "battery": battery,
            **await setup(ctx, base),
            "template": plan.settings, "speed_caps": {"linear": linear, "angular": angular},
            "expected_duration": plan.expected, **more}


def sim_model(experiment: dict) -> dict[str, float] | None:
    """The simulator's model parameters of a run (ideal values where it did not have one), or None if real."""
    model = experiment.get("sim_model")
    if experiment.get("environment") != "sim" or not isinstance(model, dict):
        return None
    # * Older runs have the mocap values in the simulator's parameters, newer ones from the fake Motive's.
    values = {**model, **mocap_values(experiment.get("sim_mocap"), experiment.get("sim_mocap_body"))}
    return {key: round(float(values.get(key, ideal)), 4) for key, ideal in MODEL_PARAMETERS.items()}


def platform(experiment: dict) -> BaseModel:
    """The limits of the base that drove a run: the A200's on the real robot, else the simulator's (older: none)."""
    model = sim_model(experiment)
    return BaseModel(**(A200_LIMITS if model is None else {key: model[key] for key in LIMIT_KEYS}))


def mocap_values(fake_motive, body: int | None) -> dict[str, float]:
    """The sim model's mocap delay and the base body's noise from a fake Motive's parameters; none without one."""
    if not isinstance(fake_motive, dict):
        return {}
    values = {}
    for key, (name, element, factor) in MOCAP_PARAMETERS.items():
        value = fake_motive.get(name)
        if value is not None and element is None:
            values[key] = float(value) * factor
        elif value is not None and body is not None:
            values[key] = body_values(str(value), int(body), 2)[element] * factor
    return values


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
    for data in sorted((recordings / experiment_name).glob(f"*/{DESCRIPTION}")):
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


def save(recordings: Path, rec: Trace, plan: Plan, experiment: dict, outcome: str, file_format: str = "npz") -> Path:
    """Save a run into a new folder of its own: its description, and its recording with the path sent.

    The recording holds the tick signals and, per message, "<serial>_mocap", "<serial>_odometry", and
    "<serial>_follower" (path runs) or "<serial>_twist" (constant-command runs).

    Args:
        recordings: The experiments' folder, RECORDING_FOLDER under the Drive root.
        rec: The run's trace.
        plan: The path sent.
        experiment: The run's description (`describe`).
        outcome: How the run ended.
        file_format: The recording's format, "npz" or "json".

    Returns:
        Path: The recording written.

    Raises:
        OSError: If it cannot be written.
    """
    folder = run_folder(recordings, experiment.get("experiment_name", "default"), plan.serial, plan.slug,
                        experiment["environment"])
    write_description(folder, {**experiment, "path_label": plan.label, "outcome": outcome})
    return save_recording(folder, rec, plan.serial,
                          {"path_poses": plan.poses, "path_t": np.array([]) if plan.t is None else plan.t}, file_format)
