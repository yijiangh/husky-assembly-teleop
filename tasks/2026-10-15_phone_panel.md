# Phone panel (2026-10-15)

A phone-friendly page for walking next to the robots: Stop all on top, then one tab per plugin that has something
for the phone. It is part of the core and always on, like the desktop page; plugins add their own tab. The first two
tabs: **Health** and **Drive** (robot_control's bases).

Code: `config.py` (port), `ui/visualization.py` (second server, Stop all, tabs, `PluginView.phone()`),
`plugins/health.py`, `plugins/robot_control/plugin.py`.

## Network

The phone joins the robot WiFi (no internet) and opens `http://<control PC LAN IP>:<phone port>`. The router
bridges the wired LAN and the robot WiFi, so no internet and no ROS on the phone. viser listens on `0.0.0.0` by
default; only the control PC's firewall may need the port opened. Anyone on the LAN can open it, as with the desktop
page.

## Decisions

- **Second viser server, owned by `Visualization`** (user picked over a phone layout of the main viser): the phone
  gets only a few HTML strings and button events, not the robot meshes and the 20 Hz scene stream over the robots'
  WiFi; the desktop UI stays untouched. Port: `MonitorConfig.phone_port`, default `viser_port + 1` (8081).
- **Core feature, not a plugin** (user): the core owns the page, its CSS and Stop all; plugins only add tabs.
- **Stop all = the core soft stop**: the phone's Stop all calls the same `Visualization._request_stop` as the desktop
  button and Esc, so it also cancels every plugin task. No new context API.
- **One tab per plugin** (user), mirroring `PluginView.ui()`:
  - `PluginView.phone(label, icon)`: a context manager that adds widgets inside this plugin's phone tab, created on
    first use (a plugin that never calls it gets no tab). Removed in `PluginView.clear()`, like the folder and panels.
  - `PluginView.phone_header()`: widgets above the tabs, under Stop all, so a status shows on every tab. Only health
    uses it, for its problems banner. ? Place the tab group with a fixed high `order` so header widgets land above it.
  - Tabs appear in plugin load order. The label is fixed at creation (viser can change a tab's icon, not its label).
- **Each plugin builds its own tab from its own state**, so no plugin reads another's internals:
  - **Health**: the banner in the header, the per-robot rows (`section` + `render_line`) in its tab. `draw` writes
    the same strings it writes to the desktop.
  - **Drive** (robot_control): a sub-tab per robot (tab group inside the tab), each with a controller chip, its own
    speed slider and the D-pad from `build_velocity_inputs` (`robot_control/dpad.py`, takes any `GuiApi`). Holds go
    to the existing `_on_hold`, so `update` stays the one place that streams the twist and sends the stop on release.
    ! The hold must remember which speed slider it came from (desktop or phone): store the slider with the hold and
    read it in `update`. Drives only while `platform_velocity_controller` is active, as on the desktop. The 0.25 s
    `HOLD_TIMEOUT` stops the base when the phone drops off or the tab is backgrounded.
- **Threading**: the phone server's callbacks follow the same rule as the main one: only `ctx.defer` /
  `ctx.defer_value`, never touch world state, the scene or ROS.
- **Freeze**: the desktop's freeze skips every `draw`, so phone tabs freeze too. Accepted.
- **Layout via one `<style>`** from the core (as `DPAD_STYLE` does): see the prototype findings below.

## Page, top to bottom

1. **Stop all**: full width, red, big.
2. Header: health's problems banner.
3. Tabs, one per plugin with a phone tab: **Drive** (sub-tab per robot: controller chip, speed, D-pad) and
   **Health** (per-robot rows of base and arm chips).

## Later tabs (mocked up, not in this task)

Mocked to check the design carries them; each is its plugin's own tab.

- **Tip** (Cartesian TCP jog for point calibration: line up the arm's tip with a fixed tip, as on the teach
  pendant): arm dropdown, controller chip, step size (0.1, 0.5, 1, 5 mm, or hold at 2 mm/s), frame (base or tool),
  a jog pad (XY cross and a Z column, labelled +X … −Z), the TCP x y z in mm, Record point, Undo, the last points.
  A tap moves one step; in hold mode it moves while held, stopped by the same 0.25 s timeout. Steps stay within
  `CARTESIAN_MAX_STEP` (10 mm). Needs `cartesian_compliance_controller`.
- **Steps** (a step-by-step process, e.g. calibration swings): the process and step n/N, the step list (done,
  current, to do), the current step's instruction and quality chips (e.g. samples, coverage, markers, residual),
  Back and Next, and a **Step controls** folder whose contents change with the step (e.g. Start/Stop swing,
  amplitude). Next is refused with a notification while the gating metrics are short. A reusable helper in `ui/` would
  serve every such process.
- **Mocap** (mocap_probe): the big status block (one colour and one word, x y z), Label, Record point, Map coverage,
  the last points.

Shared need: big buttons in a grid (D-pad, jog pad, Back/Next). The prototype generalises `_dpad_css` to
`grid_css(cells, columns)` plus a big-button rule; the core should offer that, not each plugin.

## Steps

1. Spike: done in the prototype (headless Chromium, touch: 20 holds/s). Still check on an iPhone and an Android
   phone that holding with a finger keeps sending holds and stops on release.
2. Core: `phone_port` in `MonitorConfig`; in `Visualization` the second server, its CSS, Stop all and the tab group;
   `PluginView.phone()` / `phone_header()`; stop the server on close. Note it in the plugin rules
   (`plugins/__init__.py`).
3. Health: header banner and tab.
4. robot_control: Drive tab with sub-tabs; holds carry their speed slider.
5. Tests: `Visualization` with both servers on `port=0`; a phone tab created on first use and removed by `clear`; a
   phone hold reaching robot_control's `update` at the phone slider's speed; phone Stop all raising the soft stop.
   README: how to open it from a phone.

## Prototype findings (mock server, outside the repo)

- Layout as above; Stop all and the banner stay visible on every tab. Tab groups (`add_tab_group`), also nested,
  render fine in the phone's bottom sheet.
- viser shows the phone layout (a bottom sheet, inline styles: 20em wide, max 60 % high) only below 576 px width;
  landscape phones get the desktop layout. Find the sheet with an empty marker element:
  `.mantine-Paper-root:has(#marker)`, then override with `!important`. Hide `canvas`.
- Button icons sit in a 1em box: grow them with `font-size` on `.mantine-Button-section`, and make the `svg`
  `position: static; transform: none`, or the icon starts at the box's middle and overlaps the label. For the D-pad
  hide the empty `.mantine-Button-label` and zero the section margin, or the arrow sits off-centre.
- Grids: `repeat(n, minmax(0, 1fr))` and slim button padding, or 4 columns overflow a 390 px phone. Sheet-wide rules
  that use `:has(#marker)` count as an id selector: prefix specific rules with the same sheet selector or they lose.
- Holds use pointer events and end on `pointercancel`: keep `touch-action: none` on buttons. Headless Chromium with
  touch: 20 holds/s while held.

## Open

- Several drivers at once (desktop and phone): no lock for now; each hold only drives for 0.25 s.
- Answered: Stop all switches every base controller off; switching it back on stays on the laptop (user: operators
  may have to go back to it). The phone shows which controller is active and refuses with a notification.
