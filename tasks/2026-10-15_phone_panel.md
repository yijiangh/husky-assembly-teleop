# Phone panel (2026-10-15)

A phone-friendly page for walking next to the robots: one full-screen panel with Stop all, health, and a D-pad to
drive a base. Code: new `plugins/phone.py`; small hooks in `plugin_api/context.py`, `plugins/health.py` and
`plugins/robot_control/plugin.py`.

## Network

The phone joins the robot WiFi (no internet) and opens `http://<control PC LAN IP>:<phone port>`. The router
bridges the wired LAN and the robot WiFi, so no internet and no ROS on the phone. viser listens on `0.0.0.0` by
default; only the control PC's firewall may need the port opened.

## Decisions

- **Second viser server, owned by the `phone` plugin** (user picked over a phone layout of the main viser): the
  phone gets only a few HTML strings and button events, not the robot meshes and the 20 Hz scene stream over the
  robots' WiFi; the desktop UI stays untouched. Port: `viser_port + 1` (8081 by default).
- **Content** (user): Stop all, health, D-pad drive. Nothing else.
- **Not loaded by default**: add `phone` to `plugins`. `requires = ("health", "robot_control")`.
- **Reuse, don't copy**:
  - D-pad: `build_velocity_inputs` (`robot_control/dpad.py`) already takes any `GuiApi`. Its 0.25 s
    `HOLD_TIMEOUT` stops the base when the phone drops off or the tab is backgrounded.
  - Health: render with `render_banner` / `render_line` / `section` from `health.py`, from the checks the health
    plugin already computed this tick (expose them as a read-only property; don't recompute).
  - Driving: hand holds to `robot_control` (a public `hold(serial, linear, angular, speed)`), so its `update`
    keeps the one place that streams the twist and sends the stop on release. Drives only while
    `platform_velocity_controller` is active, as on the desktop.
- **Stop all = the core soft stop**: add `ctx.request_soft_stop()` that sets the same flag as the desktop button
  and Esc (`Visualization._request_stop`), so it also cancels every plugin task.
- **Threading**: the phone server's callbacks follow the same rule as the main one: only `ctx.defer` /
  `ctx.defer_value`, never touch world state, the scene or ROS.
- **Layout via one `<style>`** (as `DPAD_STYLE` does): bottom sheet full height, canvas hidden or empty, big
  buttons, and `user-select: none; touch-action: none; -webkit-touch-callout: none` so a long press doesn't
  select text or open the browser menu.

## Panel, top to bottom

1. **Stop all**: full width, red.
2. **Health**: banner, then one line per robot (base and arms chips, battery). Rewritten in `draw`, like health.
3. **Robot**: a dropdown of serials; shows whether its velocity controller is active.
4. **Speed** slider and the **D-pad**, big.

## Steps

1. Spike first: a bare second `ViserServer` with one `on_hold` button; check on an iPhone and an Android phone that
   holding with a finger keeps sending hold events at 20 Hz and stops when released. If not, fix the hold before
   building the rest (it affects every viser option).
2. `ctx.request_soft_stop()`; health checks property; `robot_control.hold`.
3. `plugins/phone.py`: server in `setup`, widgets, `draw`, `server.stop()` in `teardown`.
4. CSS for the phone layout; check on both phones in portrait.
5. Test: setup/teardown with the server on `port=0` (as `test_scene_view.py`); a hold reaching
   `robot_control`; Stop all raising the soft stop. README: plugin list and how to open it from a phone.

## Open

- Several drivers at once (desktop and phone): no lock for now; each hold only drives for 0.25 s.
- Whether to add a button that switches the base to `platform_velocity_controller`, or leave that to the desktop.
