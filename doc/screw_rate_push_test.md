# Release test: how fast the grasp screws back out (Cindy)

Since 2026-10-28 Cindy's retreat runs **stiff** by default
(`doc/three_robot_assembly_test_manual.md` 5.3 a): when it starts, both tools loosen their grasp
screws while the arms back both tools straight out at the screws' own speed, so the screws neither
push nor pull the bar. That speed is `RETREAT_SCREW_RPM` in `husky_assembly_teleop/husky_world.py`:
**72 rpm** (2.4 mm/s) since 2026-10-28, measured with this test.

**!** Result of 2026-10-28 (3 cycles on B20, its footers only placed, not fixed; videos
`doc/IMG_5761.MOV`, `IMG_5762.MOV`, `IMG_5763.MOV`): once the thread lets go, B20 ticks once per screw
turn, every 0.81-0.84 s = 71-74 rpm. The rate came from the videos. The wrist force sensors did
**not** feel the push (B20 slid away with less than 3 N), so the rates and the
`RETREAT_SCREW_RPM = R` suggestion the test prints are wrong: don't apply them.

The test is three real B20 cycles: **place B20, release it, take it out by hand**. Each release is
one button: the monitor records B20, loosens both tools, waits, stops them, records B20 again and
prints the numbers. Nothing to count or time by hand. No Motive export: MoCap is recorded by the
monitor, like the as-built record of the assembly.

## Words used here

| Word | Meaning |
|---|---|
| grasp screws | the two screws (one per tool, 2 mm pitch, about 8 mm of thread engaged) that hold the bar in Cindy's tools; the tool's **gripper motor** turns them |
| LOOSEN | the gripper motor turning a grasp screw out of its thread |
| release test | button [Release test (BOTH)]: the measurement on this page |
| as-built record | a MoCap measurement of every bar, saved to the day's `<date>-asbuilt/` file (manual Part 9); the test takes one before and one after |
| push | while the screws turn out of their thread with the arms still, they push B20 away from the tools (on 2026-10-28 for about 2 s; the wrist force sensors did not feel it) |
| travel | how far B20 moved at a tool, along the tool's axis (MoCap, before vs after) |
| rate | travel / push time, in mm/s: the speed the stiff retreat must not exceed |

## Where the controls are

In the monitor's control panel (manual 0.5):

```
MOVEMENT EXE         [Exec Selected Mv Traj (auto)]  [Confirm Exec] [Cancel Exec]
CONTROLLERS          [Switch to Joint (BOTH)]
SCAFFOLDING V3       [Loosen Grippers (BOTH)] [Stop All (BOTH)] [Release test (BOTH)]
(Cindy)              L / R: Stop All, Tighten / Loosen Gripper, Tighten / Loosen Joint
```

**!** While both tools hold a bar, use only the (BOTH) buttons. One tool letting go or stopping
alone bends the bar.

## a) Before (once)

- [ ] pane 1: Cindy's bring-up as in the manual, Parts 1-2, and the monitor with problem
      `261023_2ModulesDemo` (manual 5.1)
  - **Expect:** at start-up, in green: `retreat screw-out: STIFF (joint controller, both tools straight out 12 mm at the screw rate, 72 rpm)`.
- [ ] Motive: B20's 8 balls stream (manual 2.e)
  - **Expect:** on any as-built record, `bar_B20 ... fit ok 8/8`.

## b) Each cycle (3 times)

**b1) Place B20**
- [ ] slider [Action (idx)] → B20_J → button [Load action], then run it as usual through its insert
      and the servo after it (manual 5.2)
  - **Expect:** B20 lies on its footers, held by both tools. Do **not** load B20_R.

**b2) Release B20**
- [ ] button [Release test (BOTH)]
  - **Expect:** about 10 s in all, in this order:
    1. `[release test] before: recording the bar (as-built, MoCap).` and
       `[as-built] ... manual: bar_B20 fit ok 8/8 | ...`;
    2. `[scaffolding] release test: LOOSEN gripper motor on L arm (arm 0)` and `... on R arm (arm 1)`,
       then `[release test] LOOSEN both sent; sampling 8 s, the arms stay still.`
       B20 slides away from both tools within about 2 s (on 2026-10-28: 2-3 mm at the left tool,
       about 6 mm at the right); the arms do not move;
    3. `[scaffolding] L Stop All ...`, `R Stop All ...`, then
       `[release test] stop both sent; settling 1 s, then the after-record.`;
    4. `[as-built] ... manual: bar_B20 fit ok 8/8 | ...` (B20 after);
    5. one line per tool, e.g.
       `[release test] left: motor start 0.10 s, push start 0.35 s after the LOOSEN, push 3.40 s (force 3.40 s, current 3.30 s), bar moved 7.6 mm at the tool (MoCap) -> 2.24 mm/s`;
    6. the summary over this monitor run's cycles, e.g.
       `[release test] 1 run(s), 2 rate(s): slowest 2.20 mm/s (mean 2.22) -> set the stiff screw-out to 1.98 mm/s: husky_world.RETREAT_SCREW_RPM = 59`;
    7. `[release test] saved .../<date>-asbuilt/release_test_<HHMMSS>.json and .png`.
  - **If not:** B20 does not move while the motors run: something holds it (a footer, another
    bar, the tools) → button [Cancel Exec]: both tools stop at once and the after-record is still
    taken. Look before the next cycle.
  - **If not:** a tool's line ends `[... no push seen]` or `[no clear push ...]`: its force sensor
    did not feel the screw push B20 (B20 too free, or already loose). That tool gives no rate in
    this cycle; note it.
  - **If not:** `(design, no MoCap fit)` in a line, or `which bar? ...`: MoCap did not fit B20
    before or after (balls hidden). The travel falls back to the design's 8 mm; free the balls
    for the next cycle.
  - **If not:** `note: the arms moved N deg during the test: not a clean push`: the arms were not
    still. That cycle does not count: button [Switch to Joint (BOTH)], then repeat it.

**b3) Take B20 out**
- [ ] by hand: take B20 out of the tools and put it back where B20_J picks it up (its manual mount).

What each part of a result line means:

| Part | Meaning |
|---|---|
| `motor start` | LOOSEN sent → the tool reports its gripper motor LOOSENING |
| `push start ... after the LOOSEN` | → the axial wrist force (tool0 z) leaves its value from before the LOOSEN: the screw starts pushing B20 |
| `push` | → the force is back at the empty-tool level: the thread is out. The longer of `force` and `current` (the motor's current falling to its free-running level), so a wrong guess gives a lower rate |
| `bar moved ... at the tool (MoCap)` | B20's MoCap fit after vs before, at that tool's flange, along the tool's axis |
| `-> ... mm/s` | the rate: travel / push |

## c) After the third cycle

**!** Since 2026-10-28 the suggestion below is known to be wrong (top of this page). Keep the
lines as a record; `RETREAT_SCREW_RPM` stays 72.

- [ ] pane 1: read the last `[release test] 3 run(s), ...` line
  - **Expect:** `-> set the stiff screw-out to X mm/s: husky_world.RETREAT_SCREW_RPM = R`: the slowest
    rate of all tools and cycles, minus 10 %, so a stiff tool lags the screws a little (they push
    B20 slightly toward its joint) and never pulls the thread.
  - **If not:** it also says `the push started up to N s after the LOOSEN: worth a start delay`:
    send that line; the screw-out can then start that much after the LOOSEN.
- [ ] in `husky_assembly_teleop/husky_world.py`: `RETREAT_SCREW_RPM = R` (one line), then restart the
      monitor (the package is built with `--symlink-install`: no rebuild)
  - **Expect:** at start-up, `retreat screw-out: STIFF (..., R rpm)`.
- [ ] the first stiff retreat (manual 5.3 a): watch the force plot
  - **Expect:** the axial force stays about flat during the screw-out.

**?** The saved cycles print their lines and the summary again (pane 1, venv + ROS sourced):
`python3 -m husky_assembly_teleop.screw_rate_test <.../release_test_*.json ...>`.

## What to send back

- the three `release_test_<HHMMSS>.json` and `.png` (in the day's `<date>-asbuilt/` folder);
- the `[release test]` lines of pane 1, and anything an **If not** above asked to note;
- optionally a phone video of one cycle, filmed from the side of a tool.
