"""
Base classes and helpers for the planner plugins; registers no plugin itself.

Each planner's contract is its own file in `shared/` (`arm_planner_contract.py`, `base_planner_contract.py`).

  plan.py              Plan, PlanRequest, and the generic PlannerContract each planner's contract fixes the request of
  planner_service.py   PlannerService: a headless planner plugin that plans, cancels and commits on request
  planner_panel.py     PlannerPanel: the panel plugin that drives a planner through its contract
  planner_search.py    Search, PlanResult, the search time limit, and planning from guessed robot states
  rrt_connect.py       RRT-Connect with abort, the straight-move shortcut, and the shortest of several searches
  timed_path.py        TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds

! No collision world here: each planner brings its own.
"""
