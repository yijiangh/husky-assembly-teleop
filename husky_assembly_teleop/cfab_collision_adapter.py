"""Adapter: expose cfab's PyBulletCheckCollision as a (conf12) -> bool predicate.

Lets external planners (the free composite planner via
`plan_transit_motion` and the constrained Stage-3 planner via
`get_joint_collision_fn`) consume cfab's 5-step collision contract without
knowing about RobotCellState.

Per-call protocol:
  1. st = template_state.copy()
  2. write 12 arm joint values into st.robot_configuration
  3. planner.set_robot_cell_state(st)
  4. planner.check_collision(st, opts); catch CollisionCheckError -> True

Caller responsibilities:
  - Provide a template state with the correct attachments (tool_states,
    rigid_body_states) and `robot_base_frame` synced to the live mocap base.
  - For Stage-3 with a held bar: ensure the bar's RigidBodyState has
    attached_to_tool set AND touch_links covers the grasp-mask links.
"""
from __future__ import annotations

from typing import Callable, Iterable, Optional

import numpy as np

from compas_fab.backends import CollisionCheckError

from husky_assembly_teleop.utils import HUSKY_DUAL_UR5e_JOINT_NAMES


def make_cfab_collision_fn(
    cfab_session,
    template_state,
    *,
    joint_names_12: Optional[Iterable[str]] = None,
    cc_options: Optional[dict] = None,
    set_state_each_call: bool = True,
) -> Callable[..., bool]:
    """Build a collision predicate that returns True iff in collision.

    Matches `pp.get_collision_fn` semantics (True == colliding) so it is a
    drop-in for callers in plan_transit_motion / plan_pose_rrt.

    Accepts extra **kwargs (e.g. `diagnosis=True` from pp.check_initial_end)
    and ignores them.
    """
    planner = cfab_session.planner
    names = list(joint_names_12) if joint_names_12 is not None else (
        list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    )
    if len(names) != 12:
        raise ValueError(f"joint_names_12 must have length 12, got {len(names)}")
    base_state = template_state.copy()
    base_opts = {
        "verbose": False,
        "full_report": False,
        "_skip_set_robot_cell_state": True,
    }
    if cc_options:
        base_opts.update(cc_options)

    def _check(conf12, **_kw) -> bool:
        q = np.asarray(conf12, dtype=float).reshape(-1)
        if q.shape[0] != 12:
            raise ValueError(f"conf12 must be length 12, got {q.shape[0]}")
        st = base_state.copy()
        for n, v in zip(names, q):
            st.robot_configuration[n] = float(v)
        if set_state_each_call:
            planner.set_robot_cell_state(st)
        try:
            planner.check_collision(st, base_opts)
            return False
        except CollisionCheckError:
            return True

    return _check
