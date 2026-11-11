# Unified clearance (4 Nov 2026)

Every button that moves a robot keeps a clearance the same way, set by the kind of move; the Clearance box only turns
it on or off.

## Rules

- ! A clearance is always below the margin the move was planned or checked with.

  | Move | Plan / check margin | Clearance |
  |---|---|---|
  | Arm, planned (Commit, calibration moves, sweeps, Exec's free-space parts) | 1 cm (`PLAN_DISTANCE`) | 5 mm (`ARM_STOP_DISTANCE`) |
  | Arm, not planned (jogs, tool0 jog, tool pad) | - | 5 mm |
  | Base, planned (Commit, base_exp) | the plan's margin, slider 10 cm at start | 5 cm (`STOP_DISTANCE`) |
  | Base, not planned (D-pads) | - | 5 cm |

- A plan whose smallest margin is below the usual one (arm 1 cm, base 10 cm) gets half its smallest margin as its
  clearance, with an amber warning chip ("clearance 3 mm: the plan comes within 6 mm").
  - Arm: the planner measures the start's and target's distance to everything; when one is under 1 cm, the whole
    path is planned at the smaller one (not at contact), so the plan's smallest margin is known (`ArmPath.margin`).
  - Base: the plan's margin is the slider's, unless the planner lets the start or target be closer (to check).
- Order for every send: lease, clearance (box ticked), a start already too close refused, wait for the first check,
  send. Nothing goes out before a clear check.
- Other robots not measured are always checked as their stand-ins.
  - Arm moves: robots whose only unknown parts are arms out of the moving arm's reach (`out_of_reach`) are taken
    without asking, for every arm send.
  - Base moves: never exempt (driving changes the distance to everyone). Commit and Start ask for a second press;
    held buttons (D-pads) do not, the amber chip warns.
- One box per set of buttons (`ClearanceToggle`), shared by laptop and phone. Only calibration free drive has no
  clearance (hand guided).
- One start chip and button colouring for all (robot_control's: red chip and grey buttons while a start would be
  refused, amber while unticked), in `ClearanceToggle`.
- Base drives are not checked against themselves (`check_self=False`): the arms stand still.
- Not in scope: stopping in time at speed (`clearance at speed`, a separate problem).

## Done (jg/unified-clearance)

- `world/clearance.py`: `MoveKind` (`ARM_MOVE`, `BASE_DRIVE`), `stop_for`, `close_note`; `STOP_DISTANCE` 5 cm,
  `BASE_PLAN_DISTANCE` 10 cm.
- Planners: `PlannerService.kind`, `plan_margin`, `stop_distance`, `close_note` (also on `PlannerContract`);
  `commit_clearance` takes its margin from them; panels' `clearance_chips` (start chip + amber "small clearance").
  Arm plans keep `endpoint_margin` (bisected to 0.5 mm) instead of falling back to contact. Base Margin slider starts
  at 10 cm (algorithms' own default `COLLISION_MARGIN` stays 2 cm); base commits not checked against themselves.
- Calibration: sweeps checked at 1 cm, stopped at 5 mm (`ARM_CLEARANCE` removed); D-pad refuses a close start, waits
  for the first check, own arms may be unknown; Go's prompt says a small clearance.
- base_exp: `_clearance` (refuse_first, first check, no self check), stop 5 cm; Start asks twice for other robots not
  measured too. No box (always on).
- Exec: plain motions under `motion_clearance` (contact only, `PLANNED_MARGIN`), none with Ignore built bars or dry
  run; box below Exec; refused start = REFUSED, trip = FAILED; chips "small clearance" / "no clearance".
- Docs: plugins.md "Clearance" section for operators, plugin sections, phone, short/test lab and calibration manuals,
  glossary.

## Left open

- The assembly planner still checks contact only: give tamp's check the 1 cm margin, then Exec's clearance becomes
  5 mm (`PLANNED_MARGIN`).
- Retreat tails (replanned free motion) run without a clearance.
- robot_control keeps its own start chip and colouring; moving them into `ClearanceToggle` is not done.
- Stopping in time at speed (separate problem).
