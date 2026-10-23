# The Assembly tab and the process panel's workaround box, on core-port

Status: done on side branch `yh/assembly-tab` (off `origin/core-port` 16e8864), two commits; not landed or pushed,
waiting for the user's OK.

## Why

`core-port` is the main branch now: `origin/yh/core-port` (eaebeec) merged with `jg/viser-cleanup-next`. Three commits
made on the old `yh/core-port` after eaebeec were never pushed. Two of them move here; one is dropped.

| Old commit | What it did | Here |
|---|---|---|
| aec124f | the "Transfer workaround" box moves into the process panel, on at start; jointing actions listed and drawn as they run | commit 1, adapted to robot leases and the glossary |
| 4045e96 | main-column folders start collapsed | dropped: every plugin is a side tab now, only health keeps a main-column folder |
| 59760fe | one "Assembly" side tab: process, steps, exec and run in bands; marks in the action list; Live IK; plan plots | commit 2, rebuilt on the plugin-tab API |

Decisions (2026-10-23):
- One Assembly tab, as in 59760fe, instead of Assembly › Run / Exec / Steps and a separate Process tab.
- The base planner's panel stays in Plan › Base.

## Commit 1: the workaround box

Cherry-pick aec124f, then:
- The hold lock (`shared/execution/interlock.held`) is gone. The process box refuses a change while a step runs, read
  from robot leases the way `RunWrites._step_running` (assembly_run/writes.py) does; messages say "has the lease".
- The steps panel's box goes, and with it 136eac3's put-back. The process box keeps that idea: when a change is
  refused, it shows the real value again.
- Wording from `doc/glossary.md`: the operator places the bar (not "mounts"); "not supported yet" instead of milestone
  numbers.

## Commit 2: the Assembly tab

- `PluginView.ui(band=None)` (ui/visualization.py): a band is an unlabelled folder with `order=band` in the plugin's
  tab, so plugins sharing one tab sort top to bottom by band number. `clear()` removes this plugin's bands, skipping
  any its tab's maker already removed.
- process, assembly_steps, movement_exec and assembly_run: `tab_path = (ASSEMBLY_TAB,)`; bands, button icons and
  the big-button CSS from `ui/assembly_tab.py`, in 59760fe's order (process 50, step 100, Exec 200, prompt 300, mark
  400, settings 500–520, run 600, plots 700).
- `ASSEMBLY_TAB = "Assembly"` and `ASSEMBLY_ICON` (LIST_CHECK) live in `ui/assembly_tab.py`; the plugins import them
  from there, and `shared/assembly_run_contract.py` no longer defines the label. process makes the tab (the other three
  require it), so its `icon` is ASSEMBLY_ICON.
- New run, and Mark done with the action's robot not live, take a second click within 10 s (assembly_run's own
  Confirm/Cancel row is gone); both are refused while movement_exec has a robot's lease (`step_running`), as are
  core-port's other refusals. Cancel's hint says "hold every arm where it is".
- The run's current step follows a pick in the process panel, except while movement_exec has a robot's lease.
- base_planner_panel keeps core-port's Plan › Base.
- README, the fake-lab manual, plugin_roadmap and the phone manual follow. monitor.sh is unchanged: its plugin list
  already makes Assembly the first side tab (Assembly, Collisions, Plan, Control; Obstacles minimized).

## Checks

- `pytest -m ""` on plain core-port: 1133 passed, 3 skipped, 1 xfailed, 3 failed (crl-husky 934fc9a's `tool_truth`
  needs `urdf_parser_py`: `sudo apt install ros-humble-urdfdom-py`, done). On the branch: 1166 passed, 3 skipped,
  1 xfailed.
- Review of both commits: no motion path past a lease, a freeze, Stop all or the stale-plan checks. Its findings are
  fixed in the commits:
  - nothing above Exec, Confirm and Cancel changes height: the process panel's error is one line, and its details
    have the same rows before a Load;
  - a pick made while a step has a robot's lease only shows that movement: the run stays on its step, and when the
    step ends the panel goes back to the run's step;
  - a refused second click ends the wait;
  - a late Live IK answer carries nothing once the step changed;
  - a refused workaround flip shows a red "workaround refused" chip for 3 s, first on its chip line.
- Fake lab on a private slot (domain 72, headless Chrome):
  - The side tabs are Assembly, Collisions, Plan, Control, with Obstacles minimized, and the bands are in order.
  - The buttons keep their place across a Load.
  - A pick sets the run's step.
  - Cindy driven to B21 with Plan › Base, then Live IK, Plan (63 waypoints, 15.4 s) and both plots.
  - Exec, then Confirm: the free move "arrived, end within 0.00°", then Cancel at the by-hand step.
  - While Exec had the leases, a pick did not move the run's step, and a workaround flip was refused with its reason.

## Found on core-port, not changed here (the user's decision)

- B20_J's Live IK finds no goal: "all 64 solutions collide; the nearest: process/joints/G20-T20-0_ground with
  obstacles/floor", with the workaround on or off. The floor (1b168a0) checks what the robots hold, and B20's ground
  half sits at floor level at the insert start.
- The process panel's Load is not refused while a step runs. It sits right above Exec and Confirm now.
- A pick during a step replaces the process's scene bodies. This does nothing today, but would matter once
  movement_exec uses a clearance guard.

## Landing

Bring the branch up to date with core-port, `pytest -m ""`, merge `--no-ff` into core-port, push. Only after the
user's OK.
