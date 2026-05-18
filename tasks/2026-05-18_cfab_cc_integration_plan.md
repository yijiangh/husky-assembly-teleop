# cfab `PyBulletCheckCollision` Integration into Free / Constrained Dual-Arm Planners

**Date:** 2026-05-18
**Status:** Plan only — not yet implemented.
**Scope:** Wire `compas_fab.backends.pybullet.backend_features.pybullet_check_collision.PyBulletCheckCollision` into:
1. Free composite planner (`plan_free_dual_arm` → `plan_transit_motion`)
2. Constrained dual-arm planner (`plan_pose_rrt` Stage 3 only)

## Context

Both target planners currently use **pybullet-planning** (`pp.get_collision_fn`) with their **own** URDF/SRDF load, *separate* from the cfab `PyBulletClient` already maintained by `CfabSession` in `husky_assembly_teleop/cfab_session.py`. The cfab session already provides a richer collision contract (5 CC steps: robot-self, robot-tool, robot-rigid_body, attached_rb-other_rb, tool-rigid_body) with proper SRDF + per-state `touch_links`/`touch_bodies` exceptions, and is already used elsewhere in this package (`husky_monitor.py:1074`, `husky_world.py:1839`).

Goal: make `PyBulletCheckCollision` available as a drop-in `(conf12) -> bool` collision predicate inside these planners with minimal architectural change.

## Critical invariants (verified by exploration)

1. **Constrained-planning path: cfab and pp share the same PyBullet world.**
   `husky_world.plan_and_stage_constrained` sets `pp.CLIENT = monitor.cfab.client.client_id` (~line 1942). `scene["robot"]` is `monitor.cfab.client.robot_puid`; `monitor.active_bar_body` is `cfab.client.rigid_bodies_puids[bar_name][0]`. So pp `get_collision_fn` and cfab `check_collision` read the same bodies. Swapping the predicate is essentially zero-scene-prep.

2. **Free-planning path: cfab and pp are *independent* worlds.**
   In `plan_both_arms_to_goal` (composite branch), `scene["robot"]` is `monitor.huskies[...].object.robot` — that lives in the monitor's pp world. cfab's world has its own husky URDF/tools/bodies (visually hidden). cfab CC will check the **cfab** robot driven by `set_robot_cell_state`, *not* the pp scene the caller built. This is fine **iff** the caller's `template_state.robot_base_frame` matches the live base. Both existing replan paths already sync this (`husky_world.py:1693, 1899`).

3. **Joint name source of truth:** `husky_assembly_teleop.utils.HUSKY_DUAL_UR5e_JOINT_NAMES` (utils.py:62–73). Existing helpers `conf_from_12vec` / `vec12_from_conf` use exactly `HUSKY_DUAL_UR5e_JOINT_NAMES[0] + [1]`. The 12 arm joints are a *subset* of cfab's full `Configuration`; base/wheel/gripper joints stay at template defaults.

4. **Template `RobotCellState` is already available:** `monitor.movement_start_state` is set on movement load (husky_monitor.py:1041). It has tool_states and (for stage-3) the active bar's `RigidBodyState` with `attached_to_tool`/`attachment_frame` already wired — that's the exact state cfab `set_robot_cell_state` consumes.

5. **`_augment_tool_touch_links_for_v3` (husky_world.py:1654–1670) is the existing pattern** for translating SRDF-disable-style allowances into cfab `touch_links`. We extend it for `STAGE3_GRASP_MASK_LINKS` on the bar's `RigidBodyState.touch_links`.

## API contract for `PyBulletCheckCollision`

- Input: `RobotCellState` with `robot_configuration` (full Configuration).
- Output: `None` on success; raises `CollisionCheckError` on collision.
- Options: `verbose`, `full_report`, `_skip_set_robot_cell_state`, `_skip_cc1..._skip_cc5`.
- 5 CC steps: (1) robot self, (2) robot↔tool, (3) robot↔rigid_body, (4) attached_rb↔other_rb, (5) tool↔rigid_body.
- Honors SRDF disabled-collisions and per-state `touch_links` / `touch_bodies` exceptions.
- Reference: `external/compas_fab/src/compas_fab/backends/pybullet/backend_features/pybullet_check_collision.py:19-300`.

## Design

### Step 1 — New adapter module (Phase 1)

**Path:** `husky_assembly_teleop/cfab_collision_adapter.py` (NEW)

Single public function — returns a `Callable[[np.ndarray12], bool]` (True == in collision, matching `pp.get_collision_fn` semantics).

```python
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
) -> Callable[[np.ndarray], bool]:
    """Build a (conf12) -> bool collision predicate using cfab PyBulletCheckCollision.

    Per-call protocol:
      1. st = template_state.copy()
      2. write 12 arm joint values into st.robot_configuration
      3. planner.set_robot_cell_state(st)  (only if set_state_each_call=True)
      4. planner.check_collision(st, opts); catch CollisionCheckError -> True
    """
    planner = cfab_session.planner
    names = list(joint_names_12) if joint_names_12 is not None else (
        list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    )
    if len(names) != 12:
        raise ValueError(f"joint_names_12 must have length 12, got {len(names)}")
    base_state = template_state.copy()
    base_opts = {"verbose": False, "full_report": False,
                 "_skip_set_robot_cell_state": True}
    if cc_options:
        base_opts.update(cc_options)

    def _check(conf12) -> bool:
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
```

**Notes:**
- Does NOT accept `diagnosis=...` kwarg; wrap at boundary if a callsite passes one.
- Does NOT do `pp.WorldSaver` — cfab's `set_robot_cell_state` mutates its world; constrained path uses `pp.WorldSaver()` at the planner boundary which restores cfab when `pp.CLIENT == cfab.client.client_id`. Document the side-effect for the free path (cfab world is left at the last sampled conf).
- `template_state.copy()` per call protects against cross-query pollution. If `RobotCellState.copy()` is shallow on `tool_states`/`rigid_body_states`, swap to `copy.deepcopy` (see Open Q §2).

### Step 2 — Constrained Stage-3 wiring (Phase 1)

**File A:** `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/dual_arm_task_space_rrt/core.py`

Add a thin helper next to `get_joint_collision_fn` (~line 349):

```python
def get_joint_collision_fn_cfab(cfab_session, template_state, *, joint_names_12=None):
    """cfab equivalent of get_joint_collision_fn for Stage 3 RRT.

    Caller must ensure template_state has the held bar attached to the left
    tool with STAGE3_GRASP_MASK_LINKS listed in touch_links of the bar's
    RigidBodyState.
    """
    from husky_assembly_teleop.cfab_collision_adapter import make_cfab_collision_fn
    return make_cfab_collision_fn(cfab_session, template_state,
                                  joint_names_12=joint_names_12)
```

`plan_pose_rrt` itself **does not change** — it already accepts `joint_collision_fn` (core.py:944, 1013–1023). The endpoint check, `extend_toward`, `endpoint_dual_arm_ik`, `reconstruct_joint_path_for_pose_path`, `smooth_dual_arm_pose_path` all pass `next_conf` positionally — adapter signature matches.

**File B:** `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/api.py` — `plan_constrained_dual_arm` (lines 168–323)

Add optional kwargs `cfab_session=None`, `cfab_template_state=None`. When both provided AND `stage == 3` AND `enforce_collision`:

```python
joint_collision_fn = None
if enforce_collision:
    if cfab_session is not None and cfab_template_state is not None:
        from .dual_arm_task_space_rrt.core import get_joint_collision_fn_cfab
        joint_collision_fn = get_joint_collision_fn_cfab(cfab_session, cfab_template_state)
    else:
        joint_collision_fn = get_joint_collision_fn(
            robot=scene["robot"], arm_joints=scene["arm_joints"],
            obstacle_bodies=list(scene["obstacles"]),
            tool_link_left=scene["tool_link_left"], bar_body=bar_body,
            grasp_bar_from_left=grasp_bar_from_left,
        )
```

Stage 1 / Stage 2 stay on `get_pose_collision_fn` (pp). Floating-bar pose check is not a natural fit for cfab without extra rigging — defer.

**File C:** `husky_assembly_teleop/husky_world.py` — `_plan_and_stage_body` (the constrained body path with two `plan_constrained_dual_arm` callsites at lines 2296 and 2320; share kwargs via `plan_kwargs`):

```python
cfab_for_constrained = None
cfab_template = None
if getattr(monitor, "use_cfab_collision_for_constrained", False) \
        and getattr(monitor, "cfab", None) is not None \
        and monitor.cfab.planner is not None \
        and monitor.movement_start_state is not None:
    cfab_for_constrained = monitor.cfab
    cfab_template = monitor.movement_start_state.copy()
    bar_name = monitor.active_bar_name
    bar_rb_state = cfab_template.rigid_body_states.get(bar_name)
    if bar_rb_state is not None:
        from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import \
            STAGE3_GRASP_MASK_LINKS
        existing = list(getattr(bar_rb_state, "touch_links", []) or [])
        for ln in STAGE3_GRASP_MASK_LINKS:
            if ln not in existing:
                existing.append(ln)
        bar_rb_state.touch_links = existing
    _augment_tool_touch_links_for_v3(cfab_template, husky)

plan_kwargs["cfab_session"] = cfab_for_constrained
plan_kwargs["cfab_template_state"] = cfab_template
```

Toggle defaults **OFF** so first land is opt-in via `monitor.use_cfab_collision_for_constrained = True` from a debug command.

### Step 3 — Free composite planner wiring (Phase 2)

**File A:** `husky_assembly_teleop/utils.py` — `plan_transit_motion` (lines 267–378)

Add `cfab_collision_fn: Optional[Callable] = None`. After existing `transit_collision_fn` is built (~line 347), if `cfab_collision_fn is not None`:

```python
if cfab_collision_fn is not None:
    def _adapted(q, **_kw):  # swallow diagnosis= kwarg pp may pass
        return bool(cfab_collision_fn(np.asarray(q, dtype=float)))
    transit_collision_fn = _adapted
```

Everything else (sample_fn / distance_fn / extend_fn) stays pp.

**File B:** `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/api.py` — `plan_free_dual_arm` (lines 64–127)

Thread `cfab_collision_fn` through to `plan_transit_motion`.

**File C:** `husky_assembly_teleop/husky_world.py` — `plan_both_arms_to_goal` composite branch (lines 1598–1639). Build `cfab_cf` analogously, gated by `monitor.use_cfab_collision_for_free` (default False).

### Step 4 — CC-step selection per planner

| Planner / situation                                                            | cc1 self | cc2 robot↔tool | cc3 robot↔rb | cc4 rb↔rb | cc5 tool↔rb |
|---|---|---|---|---|---|
| Free composite (parity with current pp `obstacles=[]`)                         | ON       | ON             | skip         | skip      | skip        |
| Free composite (full env-aware, follow-up)                                     | ON       | ON             | ON           | ON        | ON          |
| Constrained Stage 3 (parity: self + robot↔world + robot↔bar + bar↔world)       | ON       | ON             | ON           | ON        | ON          |
| Stage 1 / 2                                                                    | (keep pp `get_pose_collision_fn`) |

Free composite parity options: `{"_skip_cc3": True, "_skip_cc4": True, "_skip_cc5": True}`.

### Step 5 — Worked joint-name mapping

```
names = (
  "left_ur_arm_shoulder_pan_joint",  "left_ur_arm_shoulder_lift_joint",
  "left_ur_arm_elbow_joint",         "left_ur_arm_wrist_1_joint",
  "left_ur_arm_wrist_2_joint",       "left_ur_arm_wrist_3_joint",
  "right_ur_arm_shoulder_pan_joint", "right_ur_arm_shoulder_lift_joint",
  "right_ur_arm_elbow_joint",        "right_ur_arm_wrist_1_joint",
  "right_ur_arm_wrist_2_joint",      "right_ur_arm_wrist_3_joint",
)
st = template_state.copy()
for n, v in zip(names, q12):
    st.robot_configuration[n] = float(v)
planner.set_robot_cell_state(st)
planner.check_collision(st, {"_skip_set_robot_cell_state": True})
```

Proven by `husky_world.py:1830–1839` (dual-arm IK goal solving).

## Edge cases / gotchas

1. **`pp.get_collision_fn` callsites pass `diagnosis=...`** (`pp.check_initial_end` inside `plan_transit_motion`). Wrap the cfab predicate to absorb `**_kw`.
2. **Constrained extend / IK callsites pass `next_conf` positionally** — no wrapping needed.
3. **Free-path world divergence:** cfab world ≠ pp world. Caller MUST sync `template_state.robot_base_frame` to the live base (pattern already at husky_world.py:1693, 1899).
4. **`set_robot_cell_state` overhead.** Repositions tool/rigid-body meshes each call. ≈ equivalent to `pp.set_joint_positions` + `Attachment.assign()`. Expect 1.0–1.5× overhead. Profile in Phase 2.
5. **Configuration is full** (base/wheels/grippers); we touch only 12 arm joints, others inherit from template.
6. **`monitor.movement_start_state` may be `None`** (no movement loaded). Guard and fall back to pp.
7. **Bar held during free planning:** if `movement_start_state.rigid_body_states[bar].attached_to_tool` is set, cfab CC will check the bar — *more* correct than the pp path which only attaches grippers. Acceptable divergence.
8. **`extra_disabled_collisions` mapping:** pp uses `(body, link)` tuples; cfab uses SRDF + per-state `touch_links`/`touch_bodies`. SRDF covers the husky's hard ACM. The `_augment_tool_touch_links_for_v3` pattern is the translation template. For Stage-3 bar masks, populate `bar_rb_state.touch_links` with `STAGE3_GRASP_MASK_LINKS`.
9. **`pp.WorldSaver()` in `plan_constrained_dual_arm`** (api.py:229) restores cfab state on exit (constrained path shares `pp.CLIENT`). Good.
10. **`RobotCellState.copy()` depth** — see Open Q §2.

## Phasing

**Phase 1 (lands first):**
- (a) `husky_assembly_teleop/cfab_collision_adapter.py` (NEW)
- (b) `get_joint_collision_fn_cfab` in `dual_arm_task_space_rrt/core.py`
- (c) `plan_constrained_dual_arm` new kwargs (`cfab_session`, `cfab_template_state`)
- (d) `_plan_and_stage_body` callsite plumbing, gated by `monitor.use_cfab_collision_for_constrained` (default **False**)
- (e) Smoke test: extend `scripts/smoke_constrained_api.py` with a cfab-mode stage-3 run (if RobotCell.json is reachable headlessly), OR opt-in run inside live monitor.

**Phase 2 (follow-up):**
- (f) Free planner wiring: `plan_transit_motion` + `plan_free_dual_arm` kwarg, `plan_both_arms_to_goal` callsite, `monitor.use_cfab_collision_for_free` toggle (default False).
- (g) UI toggles on monitor; verbose / `full_report=True` on goal-state rejection.
- (h) Performance profiling pass; decide replace vs keep both.
- (i) (Optional) Stage 1/2 cfab pose-only equivalence — probably not worth it vs `pp.get_floating_body_collision_fn`.

## Verification

- **Phase-1 smoke (headless if possible):** spin up `CfabSession`, build cfab collision fn from a stage-3 movement, assert (i) clean conf passes (ii) hand-crafted self-collision (both `shoulder_pan == 0`) fails.
- **Live A/B in monitor:** plan a real `plan_and_stage_constrained_bar_action` twice with the toggle flipped. Both should produce a trajectory. Cross-check both reject the same hand-crafted collisions via direct `check_collision`.
- **Regression:** `scripts/smoke_constrained_api.py` (pp-only) keeps passing — `plan_constrained_dual_arm` falls back to pp when `cfab_session=None`.
- Run colcon build from venv per CLAUDE.md after edits.

## Critical files

| File | Why |
|---|---|
| `husky_assembly_teleop/cfab_collision_adapter.py` | NEW adapter (Phase 1) |
| `external/husky_assembly_tamp/.../motion_planner/api.py` | `plan_constrained_dual_arm` 168–323; `plan_free_dual_arm` 64–127 |
| `external/husky_assembly_tamp/.../motion_planner/dual_arm_task_space_rrt/core.py` | add `get_joint_collision_fn_cfab` near 349 |
| `husky_assembly_teleop/husky_world.py` | `plan_both_arms_to_goal` 1522–1645; `_plan_and_stage_body` callsites at 2296 / 2320 |
| `husky_assembly_teleop/utils.py` | `plan_transit_motion` 267–378 — add `cfab_collision_fn` kwarg |
| `husky_assembly_teleop/cfab_session.py` | reference: `self.client`, `self.planner`, `self.robot_cell` |
| `external/compas_fab/.../pybullet_check_collision.py` | API reference |

## Open questions (must answer before / during implementation)

1. **`monitor.cfab.robot_cell` contents** — are grippers registered as `ToolState`s? Does stage-3 `movement_start_state.rigid_body_states[bar]` carry `attached_to_tool=<left tool id>` already? Decide whether Phase 1 needs any scene prep.
2. **`RobotCellState.copy()` depth** on `tool_states` / `rigid_body_states`. If shallow, the touch_links mutation will leak into `monitor.movement_start_state` — switch to `copy.deepcopy`.
3. **`PyBulletPlanner.set_robot_cell_state` short-circuit on same state?** Determines whether to add per-call gating in adapter.
4. **Default for toggles** — recommend `False` in Phase 1 (opt-in A/B), flip after parity validated.
5. **Keep pp `get_joint_collision_fn` forever** as fallback (no cfab JSON needed for headless tests), or remove once cfab is the default? Recommend keep both; tag pp side as deprecated-after-default-flip.
6. **Phase-1 smoke test data path** — is a `RobotCell.json` checked in for headless tests, or is live-monitor the only verification surface?
