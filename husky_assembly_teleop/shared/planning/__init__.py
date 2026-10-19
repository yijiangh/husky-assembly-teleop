"""
What the planner plugins share; registers no plugin itself.

Contract, what a planner offers others:
  planner_contract.py  Plan, PlannerContract, and each planner's request and contract (arm, base)
Base classes:
  planner_service.py   PlannerService: a headless planner plugin that plans, cancels and commits on request
  planner_panel.py     PlannerPanel: the panel plugin that drives a planner through its contract
Helpers:
  planner_search.py    Search, PlanResult, the search time limit, and planning from guessed robot states
  rrt_connect.py       RRT-Connect with abort and the straight-move shortcut
  timed_path.py        TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds

! No collision world here: each planner brings its own.
"""
