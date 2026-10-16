# Phone panel (2026-10-15)

A phone-friendly page for walking next to the robots: Stop all on top, then one tab per plugin that has something
for the phone. It is part of the core and always on, like the desktop page; plugins add their own tab. First
implementation: **Health**, **Control** (robot_control's bases and arms) and **Mocap** (mocap_probe). A mock of the
page, with later tabs, was tried on a phone first (prototype findings below).

## Network and ports

The phone joins the robot WiFi (no internet) and opens `http://<control PC LAN IP>:8081`. The router bridges the
wired LAN and the robot WiFi, so no internet and no ROS on the phone. viser listens on `0.0.0.0`; only the control
PC's firewall may need the port opened. Anyone on the LAN can open it, as with the desktop page.

**Pinned ports** (user, for bookmarks): desktop 8080, phone 8081.
- viser silently takes the next free port when the one asked for is busy (`infra/_infra.py`: `port_attempt += 1`);
  the README even documents that. Pinned means: if the port is taken, the monitor refuses to start and says which
  port and how to pick another, instead of moving the page where no bookmark finds it.
- ! `viser_port` is not a ROS parameter today (`config_from_node` never declares it): the base_exp manual's
  `-p viser_port:=8095` does nothing, and that second monitor only worked through the silent fallback. Declare
  `viser_port` (8080) and `phone_port` (0 = `viser_port + 1`), so a second monitor needs one parameter and gets
  8095/8096.

## Decisions

- **Second viser server, owned by `Visualization`** (user picked over a phone layout of the main viser): the phone
  gets only a few HTML strings and button events, not the robot meshes and the 20 Hz scene stream over the robots'
  WiFi; the desktop UI stays untouched.
- **Core feature, not a plugin** (user): the core owns the page, its CSS and Stop all; plugins only add tabs.
- **Stop all = the core soft stop**: the phone's Stop all calls the same `Visualization._request_stop` as the desktop
  button and Esc, so it also cancels every plugin task. No new context API. Switching a controller back on after
  it stays on the laptop (user).
- **One tab per plugin** (user), mirroring `PluginView.ui()`:
  - `PluginView.phone(label, icon)`: a context manager that adds widgets inside this plugin's phone tab, created on
    first use, so a plugin that never calls it gets no tab. Tabs appear in the order plugins first call it (setup
    runs in dependency order). The label is fixed at creation (viser can change a tab's icon, not its label).
  - `PluginView.phone_header()`: widgets above the tabs, under Stop all, shown on every tab. Only health uses it, for
    its problems banner. The core makes one unlabelled folder above the tab group (`add_folder(None)` draws only its
    children: no title, no border); each plugin's header is its own unlabelled folder inside it.
  - `PluginView.clear()` removes the plugin's tab and header folder, like its desktop folder and panels.
- **Each plugin builds its tab from its own state**, so no plugin reads another's internals; `draw` writes the same
  strings to the desktop and phone widgets.
- **Threading**: the phone server's callbacks follow the same rule as the main one: only `ctx.defer` /
  `ctx.defer_value`, never touch world state, the scene or ROS.
- **Freeze**: the desktop's freeze skips every plugin `draw`, so phone tabs freeze too. Accepted.
- **CSS lives in one core module** (`ui/phone.py`), not in each plugin: the sheet and touch rules, and helpers for
  big buttons in a grid that the D-pad (and later the jog pad and Back/Next) use.

## Page, top to bottom

1. **Stop all**: full width, red, big.
2. Header: health's problems banner.
3. Tabs, in plugin setup order:
   - **Health**: per-robot rows of base and arm chips, and the tracked-objects row.
   - **Control**: a sub-tab per robot, labelled as on the desktop ("0804"): controller chip, speed, D-pad; below it a
     folder per arm with gripper Open/Close and Free drive (hold).
   - **Mocap**: big probe status, Label, Record point, Map coverage, a top-down map while mapping, the last points.

## Plan

### 1. Core: the phone page (`ui/phone.py`, `ui/visualization.py`, `config.py`, `monitor.py`)

- `config.py`: `MonitorConfig.phone_port: int = 8081`; `config_from_node` declares `viser_port` and `phone_port`
  (0 = `viser_port + 1`). Docstring: both are pinned.
- `ui/phone.py` (new): `PHONE_STYLE`, the `<style>` from the prototype (marker element, full-height sheet, hidden
  canvas, no text selection or long-press menu, `touch-action: none` on buttons, 44 px buttons, static button icons,
  larger chips); `grid_css(cells, columns)`, generalised from `dpad.py`'s `_dpad_css` with `minmax(0, 1fr)`
  columns; `big_buttons(icons, height, icon_size, labels=True)`. `dpad.py` builds `DPAD_STYLE` with `grid_css`.
- `Visualization.__init__(port, phone_port)`:
  - Start both servers through one helper that pins the port: if `get_port()` differs from a non-zero `port`, stop
    the server and raise `RuntimeError("port 8080 is taken; is another monitor running? Pick another with
    -p viser_port:=...")`. ! If the phone server fails, stop the desktop one before raising: `Monitor._release`
    only stops a `Visualization` that finished constructing.
  - Phone page: `configure_theme(show_logo=False, show_share_button=False)`, `<title>`, the marker and
    `PHONE_STYLE`, Stop all (big, `_request_stop`), the header folder, the tab group.
  - `view_for` passes the phone page to `PluginView`; `stop()` stops both servers.
- `PluginView`: `phone(label=None, icon=None)` and `phone_header()`, created on first use; `clear()` removes both.
- `monitor.py`: `Visualization(port=..., phone_port=...)`; log both URLs at startup.
- `plugins/__init__.py`: one line in the rules: a plugin may add a phone tab with `ctx.view.phone()`; keep it to
  what an operator needs next to the robots.

### 2. robot_control: Control tab (renamed from Drive) (`plugins/robot_control/plugin.py`)

- In `setup`, after the desktop panel: `with ctx.view.phone("Control", viser.Icon.DEVICE_GAMEPAD_2)`, a tab group with a
  sub-tab per robot, each with a status line and `build_velocity_inputs` (the same D-pad and speed slider; viser
  keeps its own copy per server). Plus `big_buttons` for the D-pad icons, labels off.
- `_BaseWidgets` gets `phone_status` and `phone_speed`.
- Holds remember where they came from: `_on_hold(ctx, serial, phone, linear, angular)` stores
  `(linear, angular, time, phone)`; `update` multiplies by `phone_speed` or `speed`. Everything else in `update`
  stays: the 0.25 s timeout, the stop on release, the controller check.
- `draw`: the phone status line: "velocity controller active" (green), or "<controller> active: D-pad off; switch
  on the laptop" (amber). The D-pad stays visible (the layout must not jump); holds are dropped by `update`, as on
  the desktop.

### 3. health: header and tab (`plugins/health.py`)

- `setup`: the banner in `ctx.view.phone_header()`; `ctx.view.phone("Health", viser.Icon.HEART_RATE_MONITOR)` with
  one row per robot and the objects row. No Unlock / Resume / Reconnect on the phone (laptop).
- `draw`: assign the same strings to the phone widgets as to the desktop ones.

### 4. Tests and docs

- `test/test_phone.py`, with ports 0 (as `test_scene_view.py`):
  - a taken port (a bound socket) makes `Visualization` raise, and no server is left running;
  - `PluginView.phone()` adds no tab until first used, one tab after, and `clear()` removes the tab and header;
  - the phone's Stop all makes `take_stop_request()` true;
  - robot_control `update` with a phone hold sends the twist at the phone slider's speed, and one stop after the
    timeout (fake base with `send_twist` and `controllers.is_active`).
- README: "Open it on a phone" (URL, firewall, pinned ports, what the tabs do); fix the "next free port" line.
  `doc/base_exp_manual.md`: the second monitor's `-p viser_port:=8095` now works and gives the phone 8096.
- Manual: the simulator (its own domain, see the sim notes), then a real robot: hold to drive from an iPhone and an
  Android phone, release stops, backgrounding the browser stops, Stop all from the phone stops everything.

### Status (2026-10-16): first implementation done

Steps 1–4 done, plus, on request:
- **Control** (renamed from Drive, as it has the arms too): per arm, gripper Open/Close and **Free drive (hold)**.
  Holding switches the arm to `free_drive_controller`; releasing (0.25 s without a report) switches it to the joint
  controller, which holds the arm, but only while free drive still runs, so a release after Stop all starts nothing.
  A refused switch is asked again after 1 s.
- **Mocap** tab (mocap_probe): big status (24vh high), Label + Record point (each page records with its own Label),
  the last action's message, Map coverage (the phone's checkbox sets the desktop one; `_set_mapping` sets the
  phone's back), the last 5 points, and a top-down map while mapping: `ui/top_down.py` draws the coverage cells,
  the footprint of every scene part (convex hull of its collision shapes), the points and the probe with PIL, on a
  worker thread (a robot's first footprints took 1.2 s on the main thread), at most every 0.5 s, sent as a PNG only
  when it changed.
- **Desktop header**: `PluginView.header()`, an unlabelled folder under Stop all, Live and Collision, above every
  plugin's folder. Health's banner moved there, so it shows with the health folder collapsed.

Tests: `test/test_phone.py` (pinned ports, tab and headers, Stop all, D-pad speed, free drive) and
`test/test_top_down.py`; quick tests and linters green. Checked against the real monitor on an isolated domain in
headless Chromium at phone size (the simulator has no controller manager, so no driving or free drive switch). The
user opened the page on a phone over the offline WiFi. The rest is in the Todo below.

## Todo: test on real phones and robots

Open `http://192.168.0.202:8081` on the offline WiFi, mobile data off. Do each on an iPhone and an Android phone.

### Page
- [ ] Loads, fills the screen in portrait; Stop all, the banner and the tabs (Health, Control, Mocap) all show.
- [ ] Stop all from the phone stops every robot (as the laptop's button), and the banner shows it.
- [ ] Long press on any button: no text selection, no browser menu, the page does not scroll.

### Control: base
- [ ] With `platform_velocity_controller` on (switched on the laptop): holding an arrow drives, releasing stops.
- [ ] The phone's Speed slider sets the speed, not the laptop's.
- [ ] Mid-hold, switch apps or lock the phone: the base stops within 0.25 s.
- [ ] Mid-hold, slide the finger off the button: it stops.
- [ ] With another controller running: "D-pad off: switch on the laptop", and the D-pad does nothing.

### Control: arms
- [ ] Gripper Open and Close work, on each arm of a dual-arm robot (0806 Left and Right).
- [ ] Free drive (hold): the chip shows "Free" and the arm can be pushed by hand while held.
- [ ] Release: the joint controller takes over and holds the arm without a jump (keep a hand near it).
- [ ] Mid-hold, switch apps or lock the phone: free drive ends and the arm is held.
- [ ] Stop all during free drive, then release: no controller starts.

### Mocap
- [ ] With the relay running: the big status shows TRACKED / NOISY / NO FIX and live x y z.
- [ ] Record point with a Label: the blue "RECORDED" flash, the point in the list and on the laptop.
- [ ] Record point without a fix: refused, with the reason under the button.
- [ ] Map coverage on the phone ticks it on the laptop and back; the map appears only while mapping.
- [ ] Walking the probe around: cells turn red to green, the probe dot follows, points show in pink.
- [ ] The map's orientation matches the room (+x right, +y up) and the robots sit where they are.

### Network
- [ ] On eduroam, `http://<eduroam IP>:8081` does not load (ufw lets only 192.168.0.0/24 in).

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
- Landscape: the phone layout needs a portrait phone (below 576 px). Accept for now.
- Tab order: tabs come in setup order, so Health (a default plugin) comes first; viser has no tab order or
  default-tab setting. Keep, or let `phone()` take an order?
