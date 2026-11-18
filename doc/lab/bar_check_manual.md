# Bar check manual

The bar check is a window for measuring the joints on a process's built bars with a tape. It shows which distances
matter, logs what you read, marks which bars are good enough, and names the joint to nudge on a bar that measures wrong.

## Start it

The window needs Tk (`sudo apt install python3-tk`, once per machine). Run it from the repo with the venv active:

```bash
python3 scripts/bar_check.py "$HUSKY_DRIVE_ROOT/data_design_study/<process>"
```

| Argument | Default | Meaning |
|---|---|---|
| `--tolerance` | 1 | A reading this close to the design is good, mm. "Good within" in the window changes it. |
| `--resolution` | 1 | How finely you read the tape, 1 or 0.5 mm. "Read to" in the window changes it. |
| `--log` | `<process>/runs/bar_check.csv` | Keep the log in another file. |

## Measure a bar

Always measure between the two joints' **outer edges**. Never measure from the bar's ends.

1. Type the bar's number and press Enter, or pick it from the list. The window opens on the bar's most important pair.
2. Press `m`, type the distance and press Enter. The reading is logged and the next pair is selected.
3. Carry on until the bar's row turns green or yellow.

A reading more than 20 mm from the design takes a second Enter, in case of a typo or the wrong edges.

| Key | Does |
|---|---|
| Digits, then Enter | Go to that bar |
| Left, right | Previous, next bar |
| Up, down | Previous, next pair |
| `m` | Enter a measured distance, also from the bar number box |
| `n` | Select the reading the solver asks for |
| `f` | Flip the bar's drawing |
| `v` | Open the 3D view |
| Esc | Show every distance |

While the cursor is in the Measured box, keys go to that box: only Enter, up, down and Esc do anything else. After a
reading the cursor stays there for the next pair. Press Esc to leave the box and use the other keys.

Click two joints to measure any other pair, in the drawing or in the 3D view. Clicking another bar in the 3D view goes
to that bar.

## The bar list

"Only bars with triangles" is on when the window opens, so bars outside a triangle are hidden. Untick it to list every
bar. Going to a hidden bar by its number also lists every bar. Bars in a triangle are bold.

| Row | Meaning |
|---|---|
| Green OK | The required pairs are measured and every reading is within tolerance. |
| Yellow OK | The required pairs are good, but another reading is off by up to 5 mm. |
| Red | A required pair is out of tolerance, or another reading is more than 5 mm off. On a bar with no required pairs, any reading out of tolerance. |
| Grey | Some readings are in, required pairs are still missing. |
| Blue with TRI | Nothing measured yet, part of a triangle. |
| Greyed out | Nothing measured yet, not part of a triangle. |

The required pairs are the bar's triangle pairs. A bar without triangles needs its ground pairs, or else all its pairs.

## Which joint is wrong?

The "Which joint?" tab compares all readings of the bar. It looks for the single joint move that removes the most error
from the readings, applies it and looks again, so it can find several wrong joints on one bar.

- **A named joint** comes with its nudge, for example "Move B42 20 mm towards B169", and an arrow on the drawing.
  *Confirmed* means a closed triangle of readings supports it. *Probable* means one more reading would confirm it.
- **A reading to take next** appears when the readings cannot tell two joints apart. Press `n` to select it.
- **"A reading is wrong"** means a loop of readings does not close. The tab names the reading to measure again.

After moving a joint, press the "I moved" button, which names the joint, and press it again within 5 seconds. The button
works when the solver names a joint or when you click one in the drawing. The joint's earlier readings stop counting and
stay in the log. Measure that joint again.

A joint is named from an error of about 1.5 mm, or from just over the tolerance when three or more readings agree on it.
A smaller error is not pinned on one joint: the tab reports the pair, and lists a single move when one would bring every
reading within tolerance.

## The log

Every reading and every moved joint is one new row in `runs/bar_check.csv` in the process folder. Rows are never
changed. The newest reading of a pair counts.

| Column | Holds |
|---|---|
| `time` | Date and time with the time zone |
| `process`, `process_hash` | The process and the version it was measured against |
| `bar`, `joint_a`, `joint_b`, `pair_type` | What was measured; `pair_type` is TRI, QUAD, GND or PICK |
| `kind` | `outer` for a reading, `moved` for a joint that was moved |
| `design_mm`, `measured_mm`, `error_mm` | The distance as designed, as read, and the difference |
| `tolerance_mm`, `within_tolerance` | The tolerance at the time and whether the reading met it |

The log is on the shared Drive, so measure from one PC at a time: a window reads the log only when it opens.

The first time a process opens, a log it has in `~/bar_check_logs/<process>.csv` is copied into the process folder. The
old file stays where it is and is not used again.
