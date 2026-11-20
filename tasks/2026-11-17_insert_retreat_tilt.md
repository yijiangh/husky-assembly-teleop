# Insert and retreat of B20, B34, B21, B35, B25 on 10 Nov, frame by frame (17 Nov 2026)

A companion page to the 10 Nov as-built page (https://claude.ai/artifact/84Qir6eSHVDCLEAhpWeYQs) on two things seen in
the lab: the ground bars end up turned in plan (left end further from Cindy, right end back toward her), and the
vertical bars' tops are off from the start.

## Data (Drive, read only)

- Records: `data_experiment/bar_action_live_replan_exe/20261110-asbuilt/as_built_261110_SRD.jsonl`; B25 is read back
  from asset 1039 (`relabel_assets`, `B25=1039`).
- 20 Hz: `data_experiment/assmebly_observer/20261110/20261110_161902_session` (`chunk_*.npz`, `events.jsonl`).
  - `object_fitted_bar_<B>_pose` is Motive's rigid-body pivot, not the bar frame: map it with one constant per bar,
    `C = inv(T_obs) · T_record`, and check it against every record (ends within about 0.5 mm).
  - B25 streamed as `object_fitted_bar_B39_pose` (from 20:09:38), fitted to B39's pattern: its constant comes from
    the relabelled records.

## What to show

- Ground bars (B20 16:38, B20 18:43, B34 19:16): left and right ends along Cindy's line through the foundation fix,
  retreat and home; split into common shift and left−right turn; line up with each jaw's loosening and each
  flange's back-out; the held load before the release (difference = push between the arms, sum = load on the bar).
- Vertical bars (B21's six inserts, B35, B25): foot and top at the insertion start against the authored pose; the bar
  the flanges imply against MoCap; foot, top and flanges through the insert, the switch to compliance and the screw's
  stall; whether a straight bar through the two grips explains the top; what the retreats moved.

## Rules

- Nothing written to the Drive; the script stays in the session's scratchpad.
- A reviewer subagent checks every cause against the raw data before the page is published; before and after must
  read the same balls.

## Done (17 Nov)

- Published: https://claude.ai/artifact/YEQUKzno3VtYHSQ4Hrkmkz (private until shared); the 10 Nov page links to it.
- Two review rounds by an independent subagent; round 1 corrected the joint (the male is on the vertical bar, the
  receiver is the ground bar's ball plate), B21's try 1 (a receiver turned 6–7° about B20), the short tops (the upper
  flange settles under weak compliance, not cut off by the stall), Belle's release (frozen finger feed, reactivation
  opens and closes once) and the MoCap frame (untrusted 16:40–17:48, refits before tries 4–5).
- Open: the 10 Nov page still says the ground bar's release moves it "a different way each time" and leaves B21's and
  Belle's causes open; the companion page refines all three.

## Follow-up (17 Nov): a timeline per insert, and a 3D replay

Asked: a 3D page, and in the report a diagram like the 10 Nov one that says when each movement runs and what the
error is at that moment.

- One timeline per episode: B20 run 1, B20 run 2, B34; B21 tries 1–6, B35, B25; Alice and Belle letting go. Lanes from
  the records and `events.jsonl`: steps (records' start and end), motions (`trajectory sent` and its duration, Confirm
  waits), controller, jaw and joint-screw motors, the servo's own error at each iteration, MoCap refits and alerts.
  Under them the error (ground: each end across the bar and its height above the seat; vertical: foot, top and
  receiver along the insert, foot and top sideways), each arm's force, and the as-built records as dots.
- Stretches over about 6 s where nothing moves are shortened to a labelled break.
- The 20 Hz lines are pinned to the records: the offset between stream and record, interpolated between records.
- 3D page: the five bars measured and designed, errors magnified; Cindy, Alice and Belle from their joint streams
  through the core's forward kinematics mirrored in JS (meshes deduplicated, coarsened to a 3 mm grid); the same
  timeline as the scrubber. Reuses `viewer_common.js` and `page.bundle_three_js` without changing the repo.
- Checks: the JS kinematics against the records' `tool0_fk`; every line through its records' dots; a reviewer
  subagent on the lane labels and the 3D page.

### Done (17 Nov, follow-up)

- Report: a timeline section after the answers (tabs per episode, `#tl-<id>` opens one), its method entry, and the B25
  release row corrected to "Cindy, Alice holding" (Alice closed at 20:13:00, before Cindy's retreat at 20:13:18).
- 3D replay published: https://claude.ai/artifact/MmC6wm8V4QPw3R5dsMqJ3v (private until shared). Exported kinematics
  reproduce `link_pose` at tool0 within 0.0002 mm; all twelve episodes load without errors.
- On the vertical timelines the foot is the bar's axis at the male's height (as on the receiver table), so foot against
  receiver is the "male short of seated" gap.
- Review (independent subagent) applied: base drives count as motion; waits end at their cancel, holds stop
  trajectories; both of Cindy's arms on the controller row; the servo row shows what it steered by (tool0 on vertical
  bars); every robot's e-stops, idle arms and lost streams in the alerts row; breaks named by what filled them; one-tick
  glitches dropped; the 3D pinned per coordinate (agrees with the timeline to 0.005 mm), far bars hidden; summary numbers
  matched to the records; release row "Belle opens" corrected to "Belle's gripper reactivated".
- Both pages republished (now shared by link).
- Report: "Where the bar and the jaws were, against the design", drawn after the lab's sketch (debug/IMG_6443.HEIC):
  design and real bar, design and real holding points, yellow (holding point) and red (end) offsets in mm, drawn ×40,
  for every insert at its key moments (`drawings.py` from the records; `#dr-<id>` opens one).
