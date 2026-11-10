"""
Replay an arm search the monitor saved when it found no path (`plugins/arm_planner/problem.py`).

The monitor logs the folder with the failure ("no path for ...: saved to ~/.ros/arm_planner/<day>/<time>_<robot>").
This runs the same search again in the same scene and prints how it ends and what blocked it; `--set` changes goal
joints first, to try a fix. Run with the venv and the workspace sourced:

    python3 scripts/replay_arm_plan.py ~/.ros/arm_planner/20261109/170351_0806 --tries 2
    python3 scripts/replay_arm_plan.py <folder> --set left_ur_arm_shoulder_lift_joint=-185.1 --window
"""

from __future__ import annotations

import argparse
import math
import threading
import time

from husky_assembly_teleop.plugins.arm_planner import planner
from husky_assembly_teleop.plugins.arm_planner.problem import load_problem
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.mirrors import MirrorPool


def main() -> None:
    """Load the problem, apply `--set`, and search `--tries` times."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("folder", help="the folder the monitor saved the failed search in")
    parser.add_argument("--tries", type=int, default=1, help="searches to run (each random)")
    parser.add_argument("--seconds", type=float, default=planner.TIME_LIMIT, help="time limit of each search")
    parser.add_argument("--set", action="append", default=[], metavar="JOINT=DEG",
                        help="change one goal joint, in degrees; may repeat")
    parser.add_argument("--window", action="store_true", help="watch the search in PyBullet's own window")
    args = parser.parse_args()

    problem = load_problem(args.folder)
    names = [name for arm in problem.arms for name in arm_joint_names(arm)]
    goal = list(problem.goal)
    for change in args.set:
        name, _, degrees = change.partition("=")
        goal[names.index(name)] = math.radians(float(degrees))
    print(f"{problem.serial} {' + '.join(problem.arms)}: saved as failed: {problem.reason}")
    if problem.detail:
        print(f"  {problem.detail}")
    for name, start, end in zip(names, problem.start, goal):
        print(f"  {name:36s} {math.degrees(start):+8.1f} -> {math.degrees(end):+8.1f} deg")

    planner.TIME_LIMIT = args.seconds
    pool = MirrorPool()
    try:
        if args.window:
            pool.set_gui(True, problem.scene, problem.serial)
        for attempt in range(args.tries):
            began = time.time()
            result = planner.plan_arms(pool.sync(problem.scene, problem.serial), problem.scene, problem.arms, goal,
                                       threading.Event())
            if result.path is None:
                print(f"try {attempt + 1}: no path in {time.time() - began:.1f} s: {result.reason}")
                if result.detail:
                    print(f"  {result.detail}")
            else:
                print(f"try {attempt + 1}: path in {time.time() - began:.1f} s, {result.path.duration:.1f} s long, "
                      f"{planner.margin_note(result.path.margin, result.path.close)}")
    finally:
        pool.close()


if __name__ == "__main__":
    main()
