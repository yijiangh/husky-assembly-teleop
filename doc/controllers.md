# The two controllers of the open-loop engine: math and algorithms

**Written 2026-09-10.** What `husky_assembly_teleop/open_loop_engine.py`,
`open_loop_traj.py` and `open_loop_insertion.py` actually compute, symbol by symbol, with
every symbol mapped to its code name. Operator notes are in
[open_loop_engine_handover.md](open_loop_engine_handover.md); the work state of the
insertion skill is in [compliant_insertion_handover.md](compliant_insertion_handover.md).

> **Status of what is described.** The joint tracker (section 1) has run on Cindy for short
> trajectories. The insertion skill (section 2) is documented **as implemented and tested
> against a fake robot only** (`scripts/fake_rtde.py`); it has never touched hardware, so
> nothing below is a validated claim about the real joint. The execution-speed scaling in
> section 1.3 is implemented and fake-tested, not yet run on the robot.

Math renders in VS Code's Markdown preview (`Ctrl+Shift+V`) and on GitHub.

---

## 0. What these two controllers are

**The "speedJ controller"** is a *joint-space trajectory tracker with velocity feed-forward
and proportional position feedback* — in control terms a **kinematic (velocity-resolved)
tracking controller**. It never computes torques: it chooses a joint velocity every cycle and
the UR's own servo loop realises it (`speedJ`). Valentin's C++ original calls this
"ReferencePath mode" (`robot_ipc_control-dev-vh/controller/impedance_controller.cpp:424-445`).
It is closed-loop on joint position; the "open-loop" in the engine's name refers to the
*plan* — it has no feedback about contact.

**The insertion skill** is a **hybrid position/force controller** in the sense of
Raibert–Craig: the two axes across the insertion direction are position-controlled (that is
what the search steers), the axis along it is force-controlled (that is what presses a part
home without knowing exactly how far away home is). It is realised as a *velocity-resolved
admittance* — forces are turned into velocities, never into torques — sent as `speedL`, so
again the UR does the inverse kinematics. On top of it sits a model-based search: an
Archimedean spiral across the rim of the hole.

---

## 1. Joint-space tracker (`_arm_tracker`, `open_loop_traj.OpenLoopTraj.sample`)

### 1.1 The reference

The planner exports samples $(t_k, q_k, \dot q_k)$ at $\Delta t_s = 0.05$ s for 12 joints.
Per arm, the loader builds one cubic **Hermite** spline through them
(`open_loop_traj._build_splines`, `scipy.interpolate.CubicHermiteSpline`):

$$
q_{ref}(\tau) = H\big(\tau;\ \{t_k, q_k, \dot q_k\}\big), \qquad
\dot q_{ref}(\tau) = \frac{d}{d\tau} q_{ref}(\tau)
$$

Position **and** velocity are interpolation constraints, so the spline reproduces the
authored $q_k$ and $\dot q_k$ exactly at every knot and is $C^1$ in between — there is no
re-fitting drift, unlike a plain cubic spline through positions alone.

`sample(arm, τ, t_end, brake_time)` clamps both ends:

- $\tau \le 0$: returns $(q_0,\ 0)$ — the start pose, at rest. The engine relies on this for
  its pre-roll (below).
- past a cutoff $\tau_e$ (`t_end`, the "run until sample" slider): the reference **brakes
  along its own path** instead of jumping to a standstill, which would make the controller
  fight the arm's momentum. With $T_b$ = `BRAKE_TIME_S` = 0.4 s and $u = (\tau - \tau_e)/T_b$:

$$
\dot q_{ref} = \dot q_e\,(1-u), \qquad
q_{ref} = q_e + \dot q_e\,T_b\left(u - \tfrac{u^2}{2}\right), \qquad 0 \le u < 1
$$

  and for $u \ge 1$ it holds $q_e + \tfrac12 \dot q_e T_b$ with zero velocity — exactly the
  point a linear deceleration ends at.

### 1.2 The control law

Every cycle (period $\Delta t = 1/f$, $f$ = `--frequency` = 125 Hz), each arm's thread reads
the shared trajectory time $\tau$, samples its reference, reads the actual joints $q$ from
RTDE, and commands

$$
\dot q_{cmd} \;=\; \operatorname{clip}\Big(\dot q_{ref}(\tau) \;+\; K_p\,\big[\,q_{ref}(\tau) - q\,\big],\ \ -\dot q_{max},\ +\dot q_{max}\Big)
$$

$$
\texttt{speedJ}\big(\dot q_{cmd},\ a,\ \Delta t\big)
$$

(written here at unit playback speed; with the speed slider the feed-forward carries a factor
$s$, derived in §1.3.)

| symbol | code | default |
|---|---|---|
| $K_p$ (per arm, scalar) | `--p-gain L R` | 1.0, 1.0 (Valentin used 0.5 / 2.0) |
| $\dot q_{max}$ | `--max-joint-vel` | 2.0 rad/s |
| $a$ (speedJ acceleration limit, not a feed-forward) | `--joint-accel` | 3.0 rad/s² |
| $f$ | `--frequency` | 125 Hz |
| $e_{abort}$ | `--err-abort` | 0.25 rad |

If the UR tracked the commanded velocity perfectly, the error $e = q_{ref} - q$ would obey

$$
\dot e = \dot q_{ref} - \dot q = \dot q_{ref} - \big(\dot q_{ref} + K_p e\big) = -K_p\, e,
$$

a first-order decay with time constant $1/K_p$ — one second at the default gain. The
feed-forward term is what makes the tracker follow a moving reference with no lag at all in
that ideal case; the P term only mops up disturbance and the UR's own tracking imperfection.
There is no integral term (the UR's inner loop has its own) and no acceleration feed-forward
(`speedJ`'s `a` is a limit the UR applies, not a command).

**Safety net.** If $\lVert e \rVert_\infty > e_{abort}$ on either arm, that thread raises,
sets the shared stop event, and **both** arms `speedStop` — reality has drifted too far from
the plan for an open-loop replay to be trusted. Before a run, `Check start pose` requires
$\lVert q - q_0 \rVert_\infty \le$ `--start-tol` (0.05 rad).

**Velocity guard at connect.** The loader evaluates the velocity splines on a 4× denser grid
and refuses to execute if $\max_\tau \lVert \dot q_{ref} \rVert_\infty > \dot q_{max}$: a
clipped feed-forward would not slow the path, it would *distort* it.

### 1.3 The shared clock, lockstep, and time scaling

Both arm threads compute the same $\tau$ from one number set before either thread starts:

$$
\tau(t) = t - t_0, \qquad t_0 = t_{\text{launch}} + T_{pre}
$$

with $T_{pre}$ = `START_PREROLL_S` = 0.5 s. For $t < t_0$, $\tau < 0$ and `sample()` returns
the start pose at rest, so thread-start jitter can never desynchronise the arms: whoever
starts first simply holds. **That single clock is the whole synchronisation** — there is no
message passing between the arms.

A phase that resumes mid-file (after an insertion) sets
$t_0 = t_{\text{launch}} + T_{pre} - \tau_{start}$, so $\tau$ picks up at $\tau_{start}$.

The execution-speed slider $s \in [0.1, 2]$ makes this a piecewise-affine clock
(`open_loop_traj.TrajClock`), shared by both threads:

$$
\tau(t) = \tau_0 + s\,(t - t_0),
$$

where a scale change at wall time $t_c$ rebases $(t_0, \tau_0) \leftarrow (t_c, \tau(t_c))$ so
$\tau$ is continuous, and the feed-forward becomes $s\,\dot q_{ref}(\tau)$ by the chain rule
($\tfrac{d}{dt} q_{ref}(\tau(t)) = s\,\dot q_{ref}$). Because both arms read the same triple
$(t_0, \tau_0, s)$, swapped in one atomic assignment, the largest disagreement between them is
one control period times the scale step — lockstep is preserved (measured: 3 ms apart at
$s = 0.5$). The slider's cap is $\dot q_{max} / \max\lVert\dot q_{ref}\rVert$, the same guard
as at connect.

Everything else measured in trajectory seconds — the cutoff brake $T_b$, the end settle, the
resume blend $T_{blend}$ — stretches by $1/s$ in wall time, because they are all evaluated
against $\tau$. A force-controlled insertion is the exception: it runs at $s = 1$, since its
contact thresholds, stall timer and budget are quantities in real seconds.

**The operator's pause (Space) is a ramp on that same rate.** `TrajClock.ramp_to` makes $s$ a
linear function of wall time between two end points, over $T_r$ = `PAUSE_RAMP_S` = 0.5 s:

$$
s(t) = s_a + (s_b - s_a)\,\frac{u}{T_r}, \qquad u = \operatorname{clamp}(t - t_c,\ 0,\ T_r),
$$

and since trajectory time is the area under that rate, $\tau$ is quadratic while the ramp
runs and affine on either side of it:

$$
\tau(t) = \tau_c + s_a u + \tfrac{1}{2}(s_b - s_a)\frac{u^2}{T_r}
\quad\text{for } 0 \le u \le T_r, \qquad
\tau(t) = \tau_c + \tfrac{1}{2}(s_a + s_b) T_r + s_b\,(t - t_c - T_r) \ \text{ after.}
$$

Pausing sets $s_b = 0$, resuming sets $s_b$ back to the slider's value, and both start from
$s_a = s(t_c)$, the rate at that instant — so reversing a ramp halfway turns it around from
where it is, keeping both $s$ and $\tau$ continuous. Braking therefore covers
$\tfrac{1}{2} s_a T_r$ of trajectory time (0.25 s at $s_a = 1$), and the joint deceleration
it asks for is $s_a\lVert\dot q_{ref}\rVert / T_r$ — at the file's 0.73 rad/s peak that is
1.5 rad/s², inside the 3.0 rad/s² `--joint-accel`, though at the $s = 2$ cap with the 2 rad/s
velocity clamp it would reach 4 rad/s² and `speedJ` would lag briefly while the P term caught
up. At $s = 0$ the reference stands still, the feed-forward term vanishes entirely and
$q_{ref}(\tau)$ is held by the P term alone.

! The ramp is stored as $(t_c, \tau_c, s_a, s_b, T_r)$ — one tuple, one atomic assignment,
exactly like the plain map — and evaluated from the wall clock on every read. Nobody has to
step it forward, which is what makes it independent of the UI's frame rate; see the pause
section of [open_loop_engine_handover.md](open_loop_engine_handover.md) for why that matters
(DearPyGui can tick at 1 Hz). Both arms read the same tuple, so lockstep holds through a ramp
just as it does through a step change.

### 1.4 Resume blend and hold mode (used around an insertion)

After an insertion the arms are wherever the skill left them, not on the plan. Snapping back
would be a jerk, so the difference is blended out over $T_{blend}$ (`--resume-blend`, 2 s):

$$
q_{ref}'(\tau) = q_{ref}(\tau) + \Delta q \cdot \max\!\Big(0,\ 1 - \frac{\tau - \tau_r}{T_{blend}}\Big),
\qquad \Delta q = q_{live} - q_{ref}(\tau_r)
$$

The arm holding the part being inserted *into* runs the same loop in **hold mode**:
$q_{ref} \equiv q_{hold}$, $\dot q_{ref} = 0$, and it watches its own wrist wrench. Its guard
is on the **change** since the hold began, $\lVert F - F_{hold,0} \rVert > F_{guard}$,
because that arm is already carrying the part's weight — only the press reaction matters.

### 1.5 Gripper events

Close/open commands are rising edges in the file, fired by the 20 Hz UI tick when
$\tau \ge t_{ev}$ — so at most 50 ms late, ample against the 0.5 s `grasp_wait` still period
the planner bakes around every close. The command is a `GripperCommand` action; what its
result means is in section 3.

---

## 2. Insertion skill (`open_loop_insertion.InsertionController`)

### 2.1 Frames and coordinates

Everything is in the **UR base frame** of the inserting arm, and the funnel line is taken
from the robot's *own* forward kinematics of the two planned configurations that bracket
the mate (the funnel mouth $q_{start}$ and the seated pose $q_{open}$, found by
`open_loop_traj.find_insertions`):

$$
p_{start} = \mathrm{FK}(q_{start}),\quad p_{open} = \mathrm{FK}(q_{open}),\quad
\hat a = \frac{p_{open} - p_{start}}{\lVert p_{open} - p_{start} \rVert},\quad
D = \lVert p_{open} - p_{start} \rVert
$$

$\hat u, \hat v$ are two unit vectors perpendicular to $\hat a$ (`frame_axes`). Every
measured TCP position $p$ is decomposed into an **axial** travel and a **lateral** offset:

$$
z = (p - p_{mouth}) \cdot \hat a, \qquad
\ell = \big[(p - p_{mouth})\cdot\hat u,\ (p - p_{mouth})\cdot\hat v\big]
$$

The orientation reference $R_{ref}$ is frozen at the funnel mouth for the whole skill: the
planner's mate is a pure translation, and letting the tool turn while a tenon is in a mortise
is how it wedges.

> Why the UR's FK and not the repo's ssik FK: ssik returns tool0 in `<side>_ur_arm_base_link`,
> which is the UR base frame turned 180° about z. Mixing the two puts the funnel in the
> wrong place. Using `getForwardKinematics(q, tcp)` keeps the reference, the pose feedback
> and the wrench in one frame.

### 2.2 Sensing

The wrench is the UR5e's built-in wrist sensor, read as `getActualTCPForce()` — the
generalised force at the TCP, in the base frame *(asserted from UR's documentation; the signs
have not yet been checked on Cindy)*. Its quoted accuracy is ±4 N and ±0.3 Nm, and that sets
the dead-bands.

- **Zeroing.** `zeroFtSensor()` is called at the start of every insertion, with the part
  already in the gripper and the arm still. The earlier ROS compliance controller skipped
  this and *sagged*: an uncompensated tool + bar weight of ~10 N read as an external force,
  and 10 N ÷ 500 N/m of stiffness is the 20 mm it drifted.
- **Low-pass.** A first-order filter on every cycle,

$$
F_{lp} \leftarrow F_{lp} + \alpha\,(F_{raw} - F_{lp}), \qquad \alpha = \frac{\Delta t}{\max(\tau_f, \Delta t)}, \quad \tau_f = 0.05\ \text{s}
$$

- **Dead-band.** Shrinks a value toward zero by the sensor's accuracy, continuous at the
  threshold:

$$
\operatorname{db}(x, w) = \operatorname{sign}(x)\,\max(|x| - w,\ 0), \qquad w_F = 4\ \text{N},\ w_\tau = 0.3\ \text{Nm}
$$

- The axial force is $f_a = F_{lp}\cdot\hat a$. Pressing the part **in** produces a reaction
  **against** $\hat a$, so "the part is pressing on something" is $-f_a > F_c$.

### 2.3 The hybrid law (`_command`)

Lateral (position-controlled, yielding only to forces above the noise):

$$
v_\ell = K_p\,(\ell_{ref} - \ell) + A\,\operatorname{db}(F_\ell,\ w_F)
$$

Axial (force-controlled, by phase; $h = 0.5$ after a retry, else 1):

$$
v_a =
\begin{cases}
v_{app} + A\,\operatorname{db}(f_a, w_F) & \text{approach} \\[4pt]
\min\!\big(A\,(f_a + F_{push}),\ v_{app}\big) & \text{search} \\[4pt]
\min\!\big(A\,(f_a + F_{push}),\ h\,v_{ins}\big) & \text{insert} \\[4pt]
-\,v_{ins} & \text{backoff}
\end{cases}
\qquad\text{and } v_a \le 0 \text{ once } z \ge D
$$

Rotation (a spring to the frozen $R_{ref}$ plus torque yield; $\log$ = rotation vector of
$R_{ref}R^\top$):

$$
\omega = K_r \log\!\big(R_{ref} R^\top\big) + A_r\,\operatorname{db}(\tau_{lp},\ w_\tau)
$$

Then $v = v_a \hat a + v_{\ell,1}\hat u + v_{\ell,2}\hat v$, clamped to $\lVert v\rVert \le h\,v_{max}$,
$\lVert \omega \rVert \le \omega_{max}$, and sent as `speedL([v, ω], a, Δt)`.

Three things worth understanding about this law:

1. **Approach stalls at contact by itself.** In free space $f_a = 0$ and the part descends
   at $v_{app}$. On contact the reaction grows until $A\,\operatorname{db}(f_a) = -v_{app}$,
   i.e. $f_a = -(v_{app}/A + w_F) \approx -14$ N — comfortably past the 8 N contact threshold,
   so contact is *noticed* rather than pushed through.
2. **Search and insert regulate the push directly.** The equilibrium of
   $A\,(f_a + F_{push}) = 0$ is $f_a = -F_{push}$; in free space the same term gives an
   approach speed of $A\,F_{push}$ = 0.002 × 10 = 20 mm/s, capped. The dead-band is
   deliberately **not** applied here: a 10 N target stands well clear of the 4 N noise, and
   dead-banding a regulated force only biases it by the dead-band's width.
3. **Nothing in the law knows where the hole is.** The lateral reference $\ell_{ref}$ is
   supplied by the search below; the law just goes where it is told and yields to walls.

### 2.4 The search: an Archimedean spiral walked by arc length

$$
r = \frac{p}{2\pi}\,\theta, \qquad
\sigma(\theta) \approx \frac{p}{4\pi}\,\theta^2 \;\Rightarrow\; \theta = \sqrt{\frac{4\pi\,\sigma}{p}},
\qquad \sigma \mathrel{+}= v_{search}\,h\,\Delta t
$$

$$
\ell_{ref} = \ell_c + r\,(\cos\theta,\ \sin\theta)
$$

Parameterising by arc length $\sigma$ rather than angle keeps the tool speed constant from
the first turn to the last (measured step spread 1.03× over a 50 mm sweep). The
$\theta^2$ approximation of the arc length is exact in the limit and only wrong in the
innermost millimetre, which the approach has already probed. $\ell_c$ is the lateral
position where contact happened.

The **pitch** $p$ = 1.5 mm is chosen against the joint: a 30 × 22 mm tenon in a 32 × 32 mm
mortise leaves 1 mm per side on the tight axis, 2 mm diametral — the spiral's turns are
closer together than that, so it cannot step over the hole. With $r_{max}$ = 8 mm the sweep
covers ~±8 mm of pickup-plus-grasp error and takes about 13 s at 10 mm/s.

### 2.5 The state machine (`_advance`)

Let $\text{remaining} = D - z$ and $\text{pressing} = (-f_a > F_c)$.

| phase | what it does | leaves when |
|---|---|---|
| `zero` | hold still for $T_{settle}$, then `zeroFtSensor()` | → `approach` |
| `approach` | track the planned line at $v_{app}$, compliant across it | $\text{remaining} \le \delta_D$ → **seated** (no contact at all); pressing with $\text{remaining} \le d_{catch} + \delta_D$ → **seated_contact**; pressing earlier → `search` (touched the rim, $z_c := z$) |
| `search` | spiral across the rim while pressing at $F_{push}$ | $z - z_c > d_{catch}$ → `insert` (dropped into the mouth); $\lVert\ell_{ref} - \ell\rVert > c$ → *constrained* (see below); $r > r_{max}$ → **search_exhausted** |
| `insert` | press at $F_{push}$, lateral zero-force compliant | $\text{remaining} \le \delta_D$ → **seated**; a jam → `backoff` (first time) or **stalled** (second); stalled within $d_{catch}+\delta_D$ of $D$ → **seated_contact** |
| `backoff` | retreat at $v_{ins}$ | retreated $\ge b$ and not pressing → `search` again, re-centred, at half speed |

A **jam** is "pressing, but less than $\delta_{stall}$ of axial progress for $T_{stall}$".
**Constrained** means the spiral is running away from the part: it cannot move sideways, so
it is *inside* the joint and blocked — that is a jam, not a hole still to be found. Without
this test a fouled pocket reads as `search_exhausted`.

Every cycle, in every phase: $\lVert F_{lp} \rVert > F_{guard}$ → **wrench_guard**;
$t > T_{budget}$ → **budget**. Any outcome other than the two seated ones stops the arm
(`speedStop`) and holds; the engine then parks both arms in its `held` state for the
operator. Only a seated outcome is ever followed by opening the gripper.

The two seated outcomes mirror the planner's simulated skill (`fixtureless-assembly/sim.py`,
`_SkillServo.seat`): *seated* = reached the planned depth, *seated_contact* = stopped by
contact within tolerance of it. The one-retry back-off is copied from it too.

### 2.6 The fake joint the tests run against (`scripts/fake_rtde.VirtualMortise`)

A stateless contact model, so the logic above can be exercised without a robot. With
half-extents $h^{peg}$ (15 × 11 mm) and $h^{hole}$ (16 × 16 mm), the overhang per lateral
axis is $\mathrm{over}_k = |\ell_k| + h^{peg}_k - h^{hole}_k$ and

$$
F =
\begin{cases}
-k\,z\,\hat a & \text{overhanging and } z \le \text{lip: resting on the rim} \\[4pt]
-\sum_{k:\ \mathrm{over}_k>0} k\,\mathrm{over}_k \operatorname{sign}(\ell_k)\,\hat e_k \;-\; k\,\max(z - z_{floor}, 0)\,\hat a & \text{otherwise: walls and floor}
\end{cases}
$$

with $k$ = 50 kN/m, so a 10 N press sinks 0.2 mm — well under $d_{catch}$, which is what
keeps "pressing on the rim" from being mistaken for "dropped in". A square rim gives **no**
lateral guidance (the real parts' chamfer is far too small to steer with), so the search has
to find the hole geometrically. There is no friction, no mass and no gripper compliance in
it; a skill that passes here has proven its *logic*, nothing more.

### 2.7 What the math does not cover

- **Orientation error.** The tool holds $R_{ref}$; there is no angular search. A seat held
  2° off makes a 25 mm-deep insertion bind on the far wall. The rotational compliance
  $A_r$ lets the tenon *follow* the walls once inside, which is the only correction there is.
- **Friction and grasp slip.** A 10–30 N press through rubber pads: the gripper held ~55 N of
  pull in a hand test (see `rtde_network_setup.md`), so slip > guard > push holds by a small
  margin only.
- **The holding arm moves in the plan** (38 mm on one mate of `open-loop.json`) but is frozen
  here; the mate is unaffected, the assembly's world pose is.

---

## 3. Grasp verdict (`grasp_verdict`, `jaw_width_mm`)

The gripper action reports `stalled` (fingers stopped before the target) and `reached_goal`
(within tolerance of it). A part in the jaws stops the fingers early, so a grasp is
`stalled` — but so is an **empty** close: the fingers' mechanical limit is at 0.7894 rad and
the 0.8 rad target overshoots it by more than the controller's 0.01 rad goal tolerance
(measured on both of Cindy's grippers). The verdict therefore uses the final knuckle angle
$\theta$:

$$
\text{close: } \begin{cases} \text{MISSED} & \theta \ge \theta_{empty} = 0.77 \\ \text{GRASPED} & \text{stalled, } \theta < \theta_{empty} \end{cases}
\qquad
w_{jaw} \approx 85\,\text{mm}\cdot\Big(1 - \frac{\theta}{0.8}\Big)
$$

The jaw width is a linear approximation of the 2F-85's linkage, good to a couple of
millimetres — enough to tell a 22 mm leg (stalls at ~0.597 rad, 21.5 mm) from a 35 mm seat
plate (~0.47 rad). A MISSED close stops both arms unless `--no-grasp-abort`.

---

## 4. Parameters

### 4.1 Tracker (`open_loop_engine.py` CLI)

| name | default | meaning |
|---|---|---|
| `--frequency` | 125 Hz | control cycle; also the RTDE rate |
| `--p-gain L R` | 1.0 1.0 | $K_p$ per arm |
| `--joint-accel` | 3.0 rad/s² | speedJ acceleration limit |
| `--max-joint-vel` | 2.0 rad/s | clamp on $\dot q_{cmd}$; also the connect-time guard |
| `--err-abort` | 0.25 rad | tracking error that stops both arms |
| `--start-tol` | 0.05 rad | how close to sample 0 START requires |
| `--approach-vel` | 0.25 rad/s | speed of the planned approach to sample 0 |
| `START_PREROLL_S` / `END_SETTLE_S` / `BRAKE_TIME_S` | 0.5 / 1.0 / 0.4 s | constants in the engine |
| `PAUSE_RAMP_S` | 0.5 s | $T_r$, how long Space takes to coast the arms to a stop |
| `--resume-blend` | 2.0 s | $T_{blend}$ |
| `--release-wait` | 1.0 s | hold after opening on a seated part |

### 4.2 Insertion (`InsertionParams`; `--ins-*` flags and the three UI sliders override)

| symbol | field | default | where it comes from |
|---|---|---|---|
| $v_{app}$ | `approach_speed` | 20 mm/s | sim `INSERT_CONTACT_SPEED` |
| $v_{ins}$ | `insert_speed` | 10 mm/s | half of it, as the sim does after a retry |
| $v_{search}$ | `search_speed` | 10 mm/s | — |
| $p$ | `search_pitch` | 1.5 mm | < the tenon's 2 mm diametral clearance |
| $r_{max}$ | `search_radius` | 8 mm | expected pickup + grasp error |
| $F_c$ | `contact_force` | 8 N | 2× the sensor's ±4 N accuracy |
| $F_{push}$ | `push_force` | 10 N | the planner's `insert_force` |
| — | `max_force` | 30 N | **declared but not used by the law** (the only cap is $F_{guard}$) |
| $F_{guard}$ | `guard_force` | 40 N | below the ~55 N grasp slip |
| $d_{catch}$ | `catch_drop` | 2 mm | > the 0.2 mm a press sinks into the rim |
| $\delta_D$ | `depth_tol` | 2 mm | sim `tol` |
| $\delta_{stall}$, $T_{stall}$ | `stall_progress`, `stall_time` | 0.2 mm, 0.3 s | sim `INSERT_PROGRESS` / 2.5× `INSERT_STALL_TIME` |
| $b$ | `backoff` | 3 mm | sim back-off |
| $c$ | `constrained_lateral` | 3 mm | 2× the pitch |
| $K_p$, $K_r$ | `kp`, `kr` | 4.0, 2.0 1/s | sim `kp` |
| $A$, $A_r$ | `admittance`, `rot_admittance` | 0.002 m/s/N, 0.05 rad/s/Nm | $A F_{push}$ = the approach speed |
| $w_F$, $w_\tau$ | `deadband_force`, `deadband_torque` | 4 N, 0.3 Nm | UR5e sensor accuracy |
| $\tau_f$ | `lowpass_tau` | 0.05 s | — |
| $v_{max}$, $\omega_{max}$ | `vmax`, `wmax` | 0.05 m/s, 0.3 rad/s | sim `vmax` |
| $a$ | `accel` | 0.5 m/s² | speedL acceleration |
| $T_{settle}$, $T_{budget}$ | `settle_s`, `budget_s` | 0.3 s, 30 s | — |
| — | `tcp_offset` | (0, 0, **0.1632** m) | ! the engine passes `utils.TOOL0_FROM_GRIPPER_TCP` (0.164 m) and `bench_insertion.py` uses 0.164 — three values for one gripper; reconcile before hardware |

Sim references: `~/Code/fixtureless-assembly/sim.py:124-129` (`INSERT_*`) and `:2517-2520`
(`_SkillServo` gains).
