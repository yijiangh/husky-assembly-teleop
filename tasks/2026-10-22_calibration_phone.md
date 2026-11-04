# Calibration on the phone (2026-10-22)

Status: done, untried on a real phone and robot (tested with fakes, screenshots of the real plugin).

Changed while building: on the phone the step list is one row of number chips (nine lines pushed the controls off the screen).

## Goal

Calibration gets a tab on the phone page (second viser server, `ui/phone.py` `PhonePage`, `PluginView.phone()`).
The user can then follow and advance the whole guided run from next to the robot. Decisions taken with the user:

- **Scope:** the whole guide on the phone (step list, chips, Back/Next). Step controls only where you stand at the
  robot. Laptop-only steps say so.
- **Go:** pressed only on the laptop, after previewing the ghost. The phone gets **Cancel**, plus the Go question as
  text with "Go on the laptop". Preview and Auto stay laptop-only.
  - Consequence: phone buttons never start a procedure that a Go gates (Record take, Sample poses here). With Auto on,
    those would move the arm with nobody previewing.

## 1. Guide on two pages (`shared/guide/`)

No plugin has a helper that writes one value to several handles: robot_control builds separate phone handles and
writes them twice. The guide has many handles, so a small helper keeps the steps' `draw` unchanged.

- **`shared/guide/both_pages.py` (new): `BothPages`, one widget on the laptop page and the phone page.** Not
  "mirror": that word means the core's scene mirrors.
  - Holds handles, each with an optional transform for `content` (e.g. `ui.phone.phone_text` on the phone).
  - Setting an attribute writes every handle, only where the value changed (the `set_changed` idea). Reading returns
    the first handle's value.
  - `on_both(existing, handle, transform=None)` adds the phone's handle to a desktop handle or a `BothPages`.
  - Steps then write `self._html.content = ...` as now, and it reaches both pages.
- **`Step.build_phone(ctx, gui)`**, defaults to a note "on the laptop".
- **`GuidePanel.build_phone(ctx, gui)`**, laid out top to bottom like the desktop with fixed lines:
  - title and step list, on both pages;
  - Back/Next, big (60 px) and side by side (`grid_css` + `big_buttons`, as in the phone mock);
  - the status chips, on both pages and wrapped in `phone_text`;
  - a folder per step that calls `step.build_phone`, shown with the current step (`show` sets both folders);
  - Cancel (Go, Auto and Preview not built);
  - the notes, with the Go question plus "Go on the laptop";
  - the instructions.
  - `_draw_controls` labels, colours, disables and shows both pages' Back/Next/Cancel through `BothPages`. The phone
    Back/Next call the same `navigate`.
- **`Checklist.build` callable once per page.**
  - Each tick by the operator (`defer_value(operator_only=True)`) sets the same box on the other page, then calls the
    step's `on_tick`.
  - `check()` and `tick()` work as now.
  - Setup passes its existing `_on_tick` instead of wiring each box itself.

## 2. Calibration's phone tab

- **`CalibrationPlugin.setup`:** `with ctx.view.phone("Calibration", <icon>) as phone: self.panel.build_phone(ctx,
  phone)`. It builds only while `ctx.view` has a phone (`PluginView.phone()` exists in the runtime).
- **`CalibrationStep.build_phone_movers`:** the phone twin of `build_movers`.
  - `build_velocity_inputs` with the phone gui (its own speed, as robot_control).
  - `HandMoves.build_inputs` (Free drive (hold), Keep free drive on, which is already kept in `HandMoves._boxes`, and
    the tool pad with its own Step size).
- **Per step on the phone**, besides the readout on both pages:

| Step | Phone controls |
|---|---|
| Setup | the checklist (synced with the laptop); arm, run and tool body stay on the laptop |
| Marker error | D-pad, Recheck (refining needs Motive: laptop) |
| Sweeps | Check collisions (user, after the first build): rechecking after moving the base moves nothing |
| Solve, Apply | none: "on the laptop" |
| Record, Validate | Validate's D-pad (drive to the next spot); Record take and Sample poses here stay on the laptop (Go-gated) |
| Tip offset | Free drive (hold), Keep free drive on, tool pad, Record tip pose |
| Punch | D-pad, Free drive (hold), Keep free drive on, tool pad, Record punch take (TCP offset entry on the laptop) |

- **Phone CSS** (one `<style>` in the tab):
  - `PHONE_DPAD_STYLE` for the D-pad;
  - `big_buttons` for the tool pad's arrow and rotate icons (about 52 px) and Back/Next (60 px);
  - Record buttons at 56 px.
  - Free drive keeps `HAND_GRAB`. Its 64 px from robot_control's page-wide rule is what we want anyway; set it here
    too, so the size doesn't depend on robot_control.
  - `PAD_STYLE` from `tool_pad.py` comes with the pad.
  - New icons must stay out of other grids (the `grid_css` rule).
- **Readouts** (`self._html`) and Setup's run line become `BothPages`; the steps' `draw` code is unchanged.

## 3. Docs

- `doc/calibration_manual.md`: an "On the phone" section with what each step offers there, and that Go stays on the
  laptop.
- `doc/plugin_roadmap.md`: drop "Not yet: a phone tab".
- `plugins/calibration/plugin.py` docstring: one line about the phone tab.

## 4. Tests

- `test/plugins/calibration/test_calibration_plugin.py`: `make_ctx` gets `view.phone`, a context manager yielding a
  second module-level `viser.ViserServer` gui. New tests:
  - the phone tab builds and shows only the current step's folder;
  - phone Next/Back navigate, and its label and colour follow the desktop's;
  - a phone tick ticks the desktop box and the run's checklist time;
  - the status and readout html match the desktop's, wrapped in `phone-text`;
  - Sweeps has Check collisions;
  - Tip offset's phone Free drive (hold) drives `HandMoves` (`hold_free_drive`);
  - no Go button on the phone, and Cancel cancels the procedure.
- `test/shared/test_guide.py`: `BothPages` (writes both, only changed, transform); the panel's phone build shows the status
  and controls; Checklist sync between two pages.

## Verification

- `pytest -n 0 test/shared test/plugins/calibration` while iterating.
- `pytest -m ""` and `--old-scripts` for calibration before each commit; linters green.
- **Phone screenshot:**
  - A scratchpad serve script runs the real `CalibrationPlugin` with the test's `make_ctx` fakes and a real
    `PhonePage` on a free port.
  - `phone_mock/shot.py` (Playwright, iPhone size) captures Setup, Tip offset and Punch.
  - Check: big buttons, nothing clipped, Back/Next fixed at the top. Then compare with the desktop screenshot.
- **Commits** (user's style; `git commit -- <paths>`, other sessions' files left out):
  1. "guide panel on the phone" (`BothPages`, panel and checklist on two pages);
  2. "calibration phone tab" (steps, plugin, docs, tests).
- Not pushed. Untried on a real phone and robot: say so.
