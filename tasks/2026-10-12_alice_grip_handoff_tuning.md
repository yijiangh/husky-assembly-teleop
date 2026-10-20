# Alice's grip handoff — open tuning questions (2026-10-12, pick up here)

The gripper close step of a support hold (`B21_H_M3_gripper_close`, button
`Exec gripper step: CLOSE + compliant handoff`) runs `husky_world.grip_with_compliant_handoff`.
It was tried in the air and on a bar on Alice on 2026-10-12. Two problems remain.
**Nothing below is implemented yet; the user wants to decide first.**

Operator manual for the run: `doc/three_robot_assembly_test_manual.md` (Part 4: air checks;
5.4: the hold procedure; 5.7: Alice's hold of B21, 5.9: Belle's hold of B35). Handoff description: `doc/support_robot_plan_skeleton_manual.md`
("The grip handoff").

---

## 0. What already changed on 2026-10-12 (uncommitted at the time of writing)

1. **No action feedback from the gripper.** Alice's `gripper_cmd` (humble `gripper_action_controller`)
   sends only a final result, never feedback. The old handoff waited for feedback to reach 80 % of
   the stroke, so on hardware it always fell back to a stiff close.
2. **Compliant BEFORE the close** (user's choice). Order now: joint controller → zero the F/T sensor
   (0.5 s) → capture tool0 (the "anchor") and check it against the TCP the arm reports → switch to
   `cartesian_compliance_controller` → send the close → publish the anchor + zero wrench every tick
   until the gripper result → always back to `scaled_joint_trajectory_controller`.
   Log lines `phase 1/3` .. `phase 3/3`.
3. **Frame bug fixed** (the "sudden arm movement"). The monitor's arm frame on the single-arm
   huskies (`ur_arm_base_link` of the calibrated URDF) is UR's `base_link` turned 90° about z.
   The anchor was published without conversion, i.e. a target 559 mm away; the arm jumped 21.5 mm
   before the drift check stopped it. Now `HuskyRobotInterface._to_controller_frame` converts
   every target in `send_arm_cmd_cartesian` (Cindy: unchanged, her frames coincide).
   Test: `test/test_controller_frame.py`.
4. **Safety limits tightened:** anchor vs reported TCP must agree within 10 mm / 2°
   (`GRIP_HANDOFF_ANCHOR_TOL_M`, `GRIP_HANDOFF_ANCHOR_TOL_DEG`; measured 0.0 mm on Alice), else a
   stiff close; the hold ends if the arm moves more than 10 mm from the anchor
   (`GRIP_HANDOFF_MAX_DRIFT_M`, was 20 mm).

After 3 the jump is gone (user, 2026-10-12). The two observations below came from that run.

---

## 1. Observation: compliance is switched off before the arm has time to react

The user replaced the Robotiq fingertips with custom ones that form-fit the bar. When they close,
the gripper should guide the arm into the form fit, but the arm goes back to joint tracking first.

**Why (checked, read-only, on Alice's gripper controller):**
- The hold ends as soon as the gripper reports a result.
- The controller reports "stalled" **1.0 s after the fingers stop moving** (`stall_timeout: 1.0`,
  `stall_velocity_threshold: 0.001` rad/s) and, with `allow_stalling: false`, **aborts** the goal
  (seen in the log: `Gripper 0 done: ABORTED, ... stalled True`).
- With form-fit tips the fingers stop on the bar almost at once, so the result comes about 1 s after
  contact. The arm, with its slow compliance response (section 2), has barely started to move.
- The gripper **keeps squeezing after the abort**: the controller does not change the close command
  on a stall (`gripper_action_controller_impl.hpp`, `check_for_success`). So the force that seats the
  arm is still there after the result; the monitor just stops listening to it.

**Options for ending the hold (decision 1):**

| | end the hold when ... | for | against |
|---|---|---|---|
| **A** (recommended) | the arm has settled: tool0 moved < 0.5 mm and < 0.2° over the last 1 s; at least 1 s after the gripper result; at most ~10 s (then switch back anyway, with a warning) | adapts to however long the seating takes; uses the live joints + FK the drift check already reads | thresholds to pick |
| B | a fixed wait after the result (e.g. 3 s) | simplest | a guess; too short or wastes time |
| C | the operator clicks `Confirm Exec` while watching the arm, with the same maximum | best while tuning | a routine pause (the user generally dislikes those) |
| D | the wrist force / torque stop changing | measures the contact directly | noisier than motion |

With any option: log every 0.5 s during the hold the tool0 drift from the anchor (mm, deg) and the
wrist force / torque, so tuning is based on numbers. Keep the 10 mm drift limit until the logs show
how far a real seating moves the arm. (Lateral room is about half of "open width − bar diameter":
open = 0.426 rad ≈ 39 mm.)

Switching back to `scaled_joint_trajectory_controller` holds the arm where it is at that moment,
so the seated pose is kept.

---

## 2. Observation: the compliance controller holds its position too much

The user wants the arm **not to sag under gravity**, but **to follow the grasping force** and
reconfigure itself into the form fit.

**Alice's current settings** (read-only `ros2 param dump /a200_0804/ur5e/cartesian_compliance_controller`;
`compliance_ref_link`, `end_effector_link`, `ft_sensor_ref_link`: `tool0`; `robot_base_link: base_link`;
`hand_frame_control: true`; `ik_solver: forward_dynamics`):

| | translation (x, y, z) | rotation (x, y, z) |
|---|---|---|
| `stiffness` | 500 N/m | 20 Nm/rad |
| `pd_gains.*.p` / `.d` | 0.05 / 0.005 | 1.5 / 0.0 |
| `solver.error_scale` | 0.2 | |
| `solver.iterations`, `solver.forward_dynamics.link_mass` | 1, 0.1 | |

**Reasoning:**
- **Settled, 500 N/m is already soft:** 10 N sideways moves the arm 20 mm once settled. The "holds its
  position" impression is probably mostly the **slow response** (gain 0.05, then scaled by 0.2) cut
  off after ~1 s by observation 1. Fixing 1 may already help a lot.
- **Seating into form-fit tips is mostly small rotations.** At 20 Nm/rad, 1 Nm turns the arm only
  about 3°; lowering the rotation stiffness should help most.
- **Sag under gravity:** the F/T sensor is zeroed right before the close with the gripper hanging in
  that pose, so the gripper's own weight is cancelled. B21 is carried by the structure and Cindy, so
  after the grasp the vertical load is small. Remaining sources: the share of the bar's weight Alice
  takes, drift of the sensor zero, and the gripper's weight pointing a little differently if the
  wrist turns. Sag ≈ that load / translation stiffness.
- So: **lower rotation stiffness, raise the response gains, keep translation stiffness around
  300–500.** If needed later, stiffness is per `tool0` axis, so it could be soft only along the
  finger-closing direction.

**Suggested first values to try** (in the air first, hand on Alice's e-stop; raise gains in small
steps, too high oscillates): rotation stiffness 5, translation stiffness 300, translation
`pd_gains.p` 0.1 (then 0.2).

**Tuning by hand, no code change** (checks pane, domain 84). The FZI controller documents stiffness
and gains as parameters that can be changed while it runs. **Not yet verified on the robot** that a
change while running takes effect:

```bash
C=/a200_0804/ur5e/cartesian_compliance_controller
for a in rot_x rot_y rot_z; do ros2 param set $C stiffness.$a 5.0; done
for a in trans_x trans_y trans_z; do ros2 param set $C stiffness.$a 300.0; done
for a in trans_x trans_y trans_z; do ros2 param set $C pd_gains.$a.p 0.1; done
# back to the values of 2026-10-12:
for a in rot_x rot_y rot_z; do ros2 param set $C stiffness.$a 20.0; done
for a in trans_x trans_y trans_z; do ros2 param set $C stiffness.$a 500.0; ros2 param set $C pd_gains.$a.p 0.05; done
```

**Where the chosen values live (decision 2):**
- **(a)** The monitor sets the handoff's values just before going compliant and puts the previous
  values back afterwards (a parameter client on the controller). Other uses of Alice's compliance
  keep theirs. Needs the runtime change to work (check by hand first).
- **(b)** Edit Alice's controller config on her PC (`crl_husky` package). Permanent; affects every
  use of Alice's compliance; needs a relaunch.

---

## 3. Decisions to make

1. **Ending the hold:** A (arm settled, recommended), B (fixed wait), C (operator confirm) or D
   (wrist force settled)? Thresholds for A: 0.5 mm / 0.2° over 1 s, min 1 s after the result, max 10 s?
2. **Stiffness:** tune by hand first with the commands above; then (a) the monitor sets and restores
   them per handoff, or (b) change Alice's config on her PC?
3. Optional: anisotropic stiffness (soft only along the finger-closing axis) if sag shows up.

## 4. Other open points around the handoff

- The wrench sent with the target (`send_arm_cmd_cartesian_force`) is **not** converted into the
  controller's frame. Only a zero wrench is ever sent; check `hand_frame_control` semantics before
  sending a non-zero one to a single-arm husky.
- Belle presumably shares Alice's 90° frame turn (same code path); unverified.
- Alice's Robotiq driver may still have the 15 % speed / full-force register bug (fixed on Cindy
  2026-09-09); the close then takes ~4 s at full force.
- Scratch tools used (session scratchpad, not in the repo): `alice_frame_check.py` (tool0 in the
  monitor vs the controller frame at the live joints), `run_alice_handoff_order.py` (headless
  handoff cases). Recreate from this note if needed.
