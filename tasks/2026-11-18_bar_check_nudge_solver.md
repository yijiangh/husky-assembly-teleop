# Bar check: which joint needs nudging

Goal: after measuring outer-edge distances between joints on one bar, tell the operator which joint is wrong, by how
much and in which direction, and which single reading to take next when that cannot be told yet. No reading from the
bar's ends: the ends are not trusted and are hard to reach.

Starting data (B20, 8 joints, whole-mm tape):

| Pair | Design mm | Measured mm | Error mm |
|---|---|---|---|
| B169 + B25 | 594.702 | 595 | +0.298 |
| B22 + B21 | 395.098 | 394 | -1.098 |
| ground-0 + ground-1 | 1110.000 | 1110 | 0.000 |

## Model

Each joint `i` sits `e_i` mm from its design position along the bar (stored direction). A reading between joints `i`
(left) and `j` (right) gives

    measured - design = e_j - e_i + reading error

Only differences are observable: shifting every joint together changes nothing. Joint width errors are not modelled;
they appear as reading error.

Readings are taken to 1 mm or 0.5 mm, chosen in the window. A set of `e` explains the readings when every residual is
within the limit for that resolution: 1.0 mm for 1 mm readings, 0.75 mm for 0.5 mm readings (half a division plus
0.5 mm for scatter on joints and slips).

An error is pinned on one joint when its estimate is at least 1.5 mm, how precisely the jig places a joint, or just over
the tolerance when three or more readings agree on it. Below that the window names the pair and says the tape cannot
tell which joint, with no nudge arrow.

## How the solver decides

Run on the newest valid reading of each pair of one bar.

1. **Best single move:** for each joint, find the move that removes the most error from all its readings, taken from
   the readings that agree on it. An odd reading therefore does not pull the move.
2. **Repeat:** apply the best joint's move and search again until every reading is explained, so several wrong joints
   are found one after the other.
3. **Rivals:** when another joint's move removes a similar error and the best move explains it away, the two explain
   the same readings. The solver stops there and asks for the reading that tells them apart.
4. **Verdict:**
   - Any joint that stood out with an error of at least the smallest nudge and the tolerance: named, with its nudge.
     **Confirmed** when it has readings to two joints that also have a reading with each other; else **probable**.
   - Every reading within the "Good within" tolerance: no joint is shown to be wrong.
   - A bar with only two joints: the pair is off, move either.
   - A loop of readings that misses closing by more than reading errors explain: a reading is wrong. Checked first; the
     window names the loop and the reading to measure again, or the reading that tells which one.
   - A reading out of tolerance while no joint alone is: the error is shared; list single moves that bring every
     reading within tolerance.
   - Otherwise too small to pin on a joint, or undecided between the rivals.
5. **Next reading:** the unread pair telling the rivals apart (predictions at least 0.9 mm apart), the shortest pair on
   a tie. For a probable verdict, the pair that closes the triangle. With no joint wrong and unread joints, the shortest
   pair reaching one.

## After a nudge

"I moved this joint" (second press confirms) appends a `moved` row to the log. Readings of that joint from before it
stop counting, also after a restart; the rows stay in the file. The solver then asks for fresh readings of that joint.

## Window

- New tab "Which joint?" beside the existing key: verdict in words, the nudge ("Move B21 3 mm towards ground-0"), the
  reading to take next with a button that selects it (`n`) and "I moved this joint". "Read to" 1 or 0.5 mm sits by the
  measured distance.
- The nudge is an arrow with its amount on the joint in the drawing, correct when the bar is flipped.
- The direction is named after a neighbouring joint, not left or right.

## Files

- `scripts/bar_check_solver.py`: the solver, numpy only.
- `scripts/bar_check.py`: the window, the tab, the arrow, the log in `<process>/runs/bar_check.csv`.
- `scripts/bar_check_3d.py`: the 3D view.
- `doc/lab/bar_check_manual.md`: how to use it.
- `test/scripts/test_bar_check_solver.py`: ladder cases, B20's real readings, and a simulation with whole-mm readings
  that reports how often the verdict is right.

## Out of scope

Readings from the bar's ends, automatic fixes, comparing bars with each other, modelling joint widths. A width bias
shared by all joints does matter: at 0.3 mm a simulated good bar was reported clean only 45% of the time. Caliper a few
blocks; a common offset of 0.2 mm or more needs handling before trusting verdicts at 0.5 mm.
