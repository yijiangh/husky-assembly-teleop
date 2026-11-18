# Narrow passages: open points (6 Nov 2026)

What the narrow-passage base planner (`jg/narrow-passages`, crl-husky `1956556`) leaves open after three reviews.

## Before driving robots

- ! Rebuild `crl_husky_msgs` and `crl_husky` on Alice, Belle and Cindy. Until then they get the plain `BasePath`
  with the whole path at the slowest scale (safe, slow); no message layout changed.
- ! Try one slowed turn on the spot on a robot. At the slowest scale the follower turns at 0.045 rad/s, below its
  usual `min_turn_speed` 0.08: a skid steer may stall there, and the turn then runs until the run's time limit.
- Time the B37_J drive (261103_SRD, build placed at -0.418, -0.078 m) in the monitor: offline it plans in about
  16 s of the 40 s; if the monitor is still about 3 times slower, it may not finish.

## Planner

- The search back from the target flips driving directions (`flip`), but the drift slack after a turn
  (`_settled`), the drive after a big turn (`_cuts_short`, `node.big`, `piece`) and the finishing join's `_fragile`
  still count in search order. Safe (the check as followed decides in driving order), but that search can spend its
  time on paths the check refuses, or refuse good ones. Fix: carry "distance until the next big turn" when flipped.
- `PlanResult.seconds` leaves out the time `planner_plugin.narrowed` takes.

## Robot interface

- A stop goes out on both `base_path` and `base_path_scaled`, which keep no order between them: a new scaled path
  sent milliseconds after a stop can be overtaken by the plain stop. Not seen in practice (planning sits in between).
- Scaled paths are chosen by any subscriber on `base_path_scaled` (`get_subscription_count`): a `ros2 topic echo`
  there makes an old follower get nothing (fails safe: the run ends silent).
- The planner tells a whole-path-slow send by `BaseInterface.scaled_as` text; a flag would be sturdier.
- An old follower still turns at its `min_turn_speed` when that is above the lowered angular cap.

## Repro

`workspace/base_planner_b37/repro_knobs.py` (env DX, DY, DYAW, NARROW, SPREAD, TL, EXACT) plans B37_J offline from
the run's `progress.json` with the build placement applied.
