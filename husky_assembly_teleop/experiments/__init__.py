"""
Offline analysis of recorded experiments, run from `scripts/`; the monitor never imports it.

It may import any package below it, plugins included; nothing imports it except scripts and tests.

  base_exp/      the base experiment's reports over all runs, and the robot model fitted to them
  base_planner/  the base planner experiment's report, one run's report and one attempt's picture
  assembly_plan_check.py  every arm movement of a process's first actions planned headless, at the authored or
                          perturbed bases (`python -m husky_assembly_teleop.experiments.assembly_plan_check PROCESS`)
"""
