# Panels never jump (2026-10-22)

Status: the plain fixes are done (1–6, 8, 11, 12, 13's one-line summary, 15–18, 20–25 in place, phone chip lines
24 px high). Waiting on the user, because they move or rebuild things (user: ask first when a case differs from the
plain pattern): 7, 9, 10, 13's folder reuse, 14, 16's warnings, 19. 26 waits for the other session's calibration work.
Shared helpers: `ui.style.chip_line`, and `ui.style.note_line` for a gray message that comes and goes.

## Goal

While a view shows, nothing above a button changes height, on the laptop and on the phone (rule in AGENTS.md, "Panels:
nothing jumps"). Chips that can appear sit on a line that is always there (`ui.style.chip_line`). Text that comes and
goes sits below the buttons. Three read-only audits found the places below; the user asked for all of them to be fixed.

## A. robot_control, health, runtime

1. `robot_control/status.py` phone base line: the refusal chip appears while a D-pad button is held and wraps → `chip_line`.
2. `robot_control/status.py` phone arm line, above Gripper and Free drive (hold) → `chip_line`.
3. `robot_control/status.py` laptop base status (chips above Run, Speed, D-pad) → `chip_line` for the chips.
4. `robot_control/status.py` laptop arm status (varying chip count above Run and Jog) → `chip_line`.
5. `robot_control/status.py` tool status above Grip, Screw and Target (up to 8 chips) → `chip_line`.
6. `robot_control/arm_panel.py` compliance pose plan: 1 note line becomes 2 value lines → always 2 lines, with
   placeholders.
7. `robot_control/plugin.py`: controller folders shown or hidden (`show_only_running`) move SENSOR and TOOL → build
   SENSOR and TOOL before the controller panel.
8. `health.py` robot rows above the Robot buttons, where labels grow → `render_line` returns a `chip_line`.
9. `health.py`: the SOFT STOPPED chip goes into the section heading → put it on the base line instead.
10. `ui/visualization.py`: the "Monitor broken" banner turns visible above Stop all → move it below the fixed top
    controls.

## B. Planners, collisions

11. `base_planner/panel_plugin.py` status (first widget; varying chips, long labels) → `chip_line`.
12. `arm_planner/panel_plugin.py` status: a chip's text swaps → `chip_line`.
13. `collisions.py`: `replace` removes a source's folder and adds it again at the bottom → reuse the folder in place;
    the summary note becomes one line.
14. `base_planner` Advanced debug legend above the Debug layers folder → create the layers before the legend.
15. (optional) `process/plugin.py` and `obstacles/plugin.py` status chips → `chip_line`.

## C. Experiments, mocap, examples, calibration (laptop)

16. `base_exp/plugin.py` status: 3–6 chips, and two warnings above all controls → `chip_line`; warnings into
    `_details`.
17. `mocap_bars.py` readout of 1–3 lines → always 3 lines.
18. `mocap_probe.py` `_result` note appears and wraps → one always-present line.
19. `mocap_probe.py`: the Samples list grows above the coverage controls → coverage section first.
20. `mocap_probe.py` phone `_phone_result` between Record point and Map coverage → below them, or a one-line line.
21. `mocap_probe.py` phone big box (`min-height`) can grow as Record point is tapped → fixed content.
22. `examples/pybullet_scene.py`: hits add lines above Move → one fixed line.
23. `examples/lease_and_freeze.py` chips (outcome, refusal) → `chip_line` (a template others copy).
24. `examples/sequence.py` chips → `chip_line`.
25. `examples/recording.py` empty result line → "—" placeholder.
26. Calibration, laptop (waits: another session edits calibration and the guide; tell it to apply this):
    - the check-chip rows wrap when a value grows → the same fixed-height `.guide-chips` box as on the phone;
    - Tip offset and Punch readouts grow above Go/Cancel → fixed line counts;
    - Validate's spots row → `chip_line`.

## Verification

- Area tests while fixing; `pytest -m ""` before each commit.
- One commit per area (A, B, C).
- Screenshots of the phone (robot_control Control tab, calibration) with `phone_mock/shot.py`.
