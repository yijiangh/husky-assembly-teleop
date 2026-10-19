# MoCap bar pose test (B20 by eye): Rhino pose vs MoCap fit vs the real bar

> **Runs on any PC on the mocap network** (`192.168.0.x`, the Motive PC is `192.168.0.28`).
> Set `HUSKY_PC` at the top of the PC block (section 0.0) and paste it in every terminal. Every
> command below then uses its variables; nothing else is PC-specific.
>
> | `HUSKY_PC` | PC | workspace | shared drive | mocap network port |
> |---|---|---|---|---|
> | `laptop` | the Razer laptop (user `yijiangh`) | `~/Code/ros2_ws` | `~/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly` | the USB-ethernet dongle (its name changes per dongle; found by its `192.168.0.x` address) |
> | `robotpc` | `CRLRobotxPC` (user `su`) | `~/ros2_ws` | `~/Insync/2025-03 Husky Assembly` | `eno1` (found the same way) |
>
> The tmux tips of `doc/three_robot_assembly_test_manual.md` section 1 apply as they are.

**What this test shows.** Bar **B20** of the design problem **`261016_2ModulesDemo_YH`** carries
four `T20_MoCap` joints, each with a plate holding two OptiTrack balls (8 balls). The monitor
finds those 8 balls in Motive's stream, matches them to the Rhino marker table
(`MoCapMarkers.json`) and fits B20's full pose. PyBullet then shows two see-through copies of
B20 **with all its joints**. The same is done for **every bar Motive streams that has a marker
table**, whichever action is loaded; today that is B20 and B21:

| copy | colour | where | label |
|---|---|---|---|
| Rhino | **pink** | the Rhino (assembled) pose from the BarAction export | `B20 Rhino` |
| MoCap | **cyan** | the pose fitted from the balls | `B20 MoCap 8/8 rms 0.9 mm` |

You compare three things:
- **pink vs cyan**: what the log prints as `dpos`, `dang` and `dx / dy / dz(along)`;
- **cyan vs the real bar**: every white cross (a ball Motive sees) sits on a ball of the cyan
  plates. Then the real bar's ends, plates and heights match the cyan copy (section 4.2);
- **pink vs the real bar**: where you put the bar compared with where Rhino wants it.

**No robot is needed.** The monitor runs with `FAKE_HARDWARE = 1`: it connects to no arm,
gripper or tool, and nothing moves. Motive must stream.

**B20 at its Rhino pose** (from the export; the monitor prints the same numbers):

| | |
|---|---|
| bar | 1800 mm long, lying flat, its centre line **150 mm above the floor** |
| start end | at x = **1326**, y = **−2278** mm from the Motive origin |
| other end | at x = **−474**, y = −2278 mm (the bar runs along −x) |
| joints along the bar, from the start end | J20-22 mocap **75 mm**, G20-T20-0 ground 265, J20-21 mocap **430**, J20-42 female 650, J20-169 mocap **1135**, G20-T20-1 ground 1325, J20-44 female 1470, J20-25 mocap **1670** |
| mocap plates | J20-22, J20-21 and J20-25 face **+y**; **J20-169 faces −y**. On each plate one ball sits about 24 mm above the bar's centre line and one 24 mm below it |

So the **start end** is the end whose nearest mocap joint is 75 mm from the tip; at the other
end the nearest mocap joint (J20-25) is 130 mm from the tip.

Every code block below is meant to be pasted as is. Tick the boxes as you go; anything that does
not match "expect" is worth a note (log lines + a photo).

---

## 0. Once, before the session

### 0.0 First, in every terminal: the PC block

Set `HUSKY_PC` in the first line, then paste the whole block. It goes to the workspace, sources the
venv and ROS, finds the mocap network port and this PC's address on it, and sets the data paths
and the design problem.

```bash
export HUSKY_PC=laptop        # laptop = this laptop (yijiangh, USB-ethernet dongle) | robotpc = CRLRobotxPC (su, eno1)
case "$HUSKY_PC" in
  laptop)  export HUSKY_WS="$HOME/Code/ros2_ws"
           export HUSKY_DRIVE="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly" ;;
  robotpc) export HUSKY_WS="$HOME/ros2_ws"
           export HUSKY_DRIVE="$HOME/Insync/2025-03 Husky Assembly" ;;
  *)       echo "! HUSKY_PC must be laptop or robotpc" ;;
esac
# * The port on the mocap network (192.168.0.x; a dongle's name changes per dongle) and this PC's address there.
export HUSKY_NET_IF=$(ip -4 -o addr | awk '$4 ~ /^192\.168\.0\./ {print $2; exit}')
export MOCAP_CLIENT_IP=$(ip -4 -o addr show "$HUSKY_NET_IF" 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
# * ROS: the workspace, Cindy's domain (her cell = the marker table's), CycloneDDS pinned to that port.
cd "$HUSKY_WS" && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$HUSKY_NET_IF\"/></Interfaces><AllowMulticast>default</AllowMulticast></General></Domain></CycloneDDS>"
# * The design problem and where the data lives.
export DESIGN_DATA_DIRECTORY="$HUSKY_DRIVE/data_design_study"
export EXPERIMENT_DATA_DIRECTORY="$HUSKY_DRIVE/data_experiment"
export DESIGN_PROBLEM_NAME=261016_2ModulesDemo_YH
export HUSKY_MONITOR_PY="$HUSKY_WS/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py"
ros2 daemon stop >/dev/null 2>&1
echo "[$HUSKY_PC] ws $HUSKY_WS | mocap port ${HUSKY_NET_IF:-NONE} = ${MOCAP_CLIENT_IP:-NONE} | domain $ROS_DOMAIN_ID | problem $DESIGN_PROBLEM_NAME"
```

- [ ] Expect one line like
      `[laptop] ws /home/yijiangh/Code/ros2_ws | mocap port enxa0cec86472dd = 192.168.0.29 | domain 86 | problem 261016_2ModulesDemo_YH`.
      `mocap port NONE = NONE`: no port of this PC is on the `192.168.0.x` network. Check the
      cable / dongle (`ip -4 -brief addr`) before going on.

What the variables do:
- `MOCAP_CLIENT_IP` is the address Motive streams to. The monitor reads it instead of the
  `CLIENT_IP` written in `husky_monitor.py`, so the file is not edited per PC.
- `CYCLONEDDS_URI` pins ROS to the same port, written inline, so `~/.cyclonedds.xml` and its
  dongle name do not matter.
- `DESIGN_DATA_DIRECTORY`, `EXPERIMENT_DATA_DIRECTORY` and `DESIGN_PROBLEM_NAME` point the monitor
  at this PC's copy of the shared drive.
- `ROS_DOMAIN_ID=86` is Cindy's: the monitor then loads `RobotCell.json`, the file the marker
  table was exported with. With `FAKE_HARDWARE = 1` the monitor does not talk to Cindy, so she
  may stay off.

### 0.1 Monitor flags

The switches are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`); no rebuild after editing, restart the monitor.

```bash
F="$HUSKY_MONITOR_PY"
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|MOCAP_BAR_FAKE_MARKERS|BAR_ACTION_LIVE_REPLAN_EXE|BAR_ACTION_MOCAP_ACCURACY_TEST|AUTO_LOAD_ACTION|MOCAP_AXIS_CONVENTION) *=" "$F"
grep -nE "^MOCAP_IP *=" "$F"
```

| flag | value for this test |
|---|---|
| `FAKE_HARDWARE` | **1** (no robot; ! set it back to 0 afterwards, section 7) |
| `USE_MOCAP` | **1** |
| `MOCAP_BAR_FAKE_MARKERS` | **0** (1 only for the desk rehearsal, section 2) |
| `MOCAP_AXIS_CONVENTION` | `"rhino"` |
| `BAR_ACTION_LIVE_REPLAN_EXE` | 1 (the plan skeleton panel loads the action) |
| `BAR_ACTION_MOCAP_ACCURACY_TEST` | **1** (shows `Save markerset data`, used in 4.6) |
| `AUTO_LOAD_ACTION` | 1 (action 0, `B20_J_joint`, loads by itself) |
| `MOCAP_IP` (module level) | `192.168.0.28` |
| `CLIENT_IP` (module level) | not edited: `MOCAP_CLIENT_IP` from the PC block overrides it |

To flip `FAKE_HARDWARE` from the shell:

```bash
F="$HUSKY_MONITOR_PY"
sed -i 's/^    FAKE_HARDWARE = 0/    FAKE_HARDWARE = 1/' "$F"
grep -n "^    FAKE_HARDWARE" "$F"     # expect: FAKE_HARDWARE = 1
```

### 0.2 The problem folder and its marker table

```bash
P="$DESIGN_DATA_DIRECTORY/$DESIGN_PROBLEM_NAME"
ls "$P"                       # ActionSchedule.json BarActions MoCapMarkers.json RobotCell.json RobotCell_Alice.json RobotCell_Belle.json ...
ls -a "$P" | grep -c insyncdl # 0 = Insync has finished syncing
python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print('schema', d['schema_version'], '| units', d['units'], '|', {b: len(v['markers']) for b, v in d['bars'].items()})" "$P/MoCapMarkers.json"
```

- [ ] Expect: `schema 2 | units m | {'bar_B20': 8, 'bar_B21': 8}`.

  `schema 1`: an old export. Re-export from Rhino (`RSExportRobotCell`); the monitor refuses
  schema 1.

### 0.3 Motive

B20 is a rigid-body asset in Motive with streaming ID **1020**. The lab convention is that bar
B*xx* streams as 10*xx*, so B21 is 1021. The monitor uses the convention by itself and reads the
8 balls Motive labels under that asset.

- [ ] Streaming on, to this PC, as for the robots' rigid bodies. In the streaming settings,
      **Labeled Markers** (and **Asset Markers**, if listed) and **Unlabeled Markers** are
      **on**. The unlabelled markers are the fallback when the asset is off or lost (section 6).
- [ ] The B20 asset (ID **1020**) is enabled and tracked, with all **8** balls. Motive's
      rigid-body pose itself is not used, only the balls. On 2026-10-16 Motive listed the bar
      assets as `B20` = 1020, `B21` = 1021, `B34` = 1034, `B35` = 1035 (NatNet 4.0), with B20's 8
      balls streaming.
- [ ] No other asset uses ID 1020.

### 0.4 The bar

- [ ] B20 with its four mocap joints mounted: J20-22, J20-21, J20-169, J20-25 (balls 19 mm).
      Check by eye that the plates sit as in the table above: three on one side, J20-169's on
      the other.
- [ ] Something to set B20 down on near its Rhino spot: its two ground joints (G20-T20-0 at
      265 mm, G20-T20-1 at 1325 mm) on their foundations, if they are fixed for this export;
      else two supports that hold the centre line about 150 mm above the floor.
- [ ] A tape measure.

### 0.5 Rebuild the package (after pulling)

```bash
cd "$HUSKY_WS" && source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

---

## 1. Monitor pane — paste before each monitor run

The PC block of section 0.0 (it also stops the ROS daemon). The monitor is started in section 3.

Keep a log (from another pane, once): `mkdir -p ~/husky_real && tmux pipe-pane -t husky_real:0.0 -o 'cat >> ~/husky_real/mocap_bar.log'`.

---

## 2. Desk rehearsal (optional; no Motive, no bar)

This rehearses the panel and the drawing anywhere, before going to the lab. The monitor makes up
B20's 8 balls at its Rhino pose, **slid 30 mm along the bar and rolled 2° about it**, plus one
stray marker.

- [ ] Set `MOCAP_BAR_FAKE_MARKERS = 1` (and `FAKE_HARDWARE = 1`); `USE_MOCAP` may be 0 or 1.

```bash
F="$HUSKY_MONITOR_PY"
sed -i 's/^    MOCAP_BAR_FAKE_MARKERS = 0/    MOCAP_BAR_FAKE_MARKERS = 1/' "$F"
grep -n "^    MOCAP_BAR_FAKE_MARKERS" "$F"     # expect: MOCAP_BAR_FAKE_MARKERS = 1
```

- [ ] Start the monitor (section 3), wait for the action to load, then click
      `MoCap bar: measure + draw (streaming bars)`. Made-up balls exist only for the loaded
      bar. Expect (numbers vary by a few tenths):

```
[mocap bar] measuring 1 bar(s): bar_B20 (fake markers) | instrumented but not streaming: bar_B21
[mocap bar] bar_B20 | source fake (age 0 ms) | 8/8 matched | rms 0.7 mm, max 1.2 mm | dpos 30.1 mm, dang 1.77 deg | in the bar frame: dx 0.1, dy -0.0, dz(along) 30.1 mm | ok
[mocap bar] residuals (mm): J20-22_mocap_M1 0.3, J20-22_mocap_M2 0.2, ... | missing: none | stray sensed: 1
[mocap bar] bar ends (world, mm): Rhino start [1326 -2278 150] end [-474 -2278 150] | MoCap start [1296 -2278 150] end [-504 -2278 150]
```

  - [ ] PyBullet: the pink and the cyan B20, each with its 8 joints. The cyan copy sits 30 mm
        towards the far end (−x) and turned slightly about the bar.
  - [ ] Eight white crosses on the cyan plates' balls, each joined to its ball by a short black
        line. One red cross 0.3 m away (the stray marker).
  - [ ] Labels `B20 Rhino` and `B20 MoCap 8/8 rms ... mm`. The pink line
        (`[BarAction] pink line = bar_B20 at its assembled pose ...`) runs through the pink copy.
  - [ ] Untick `Rhino bar lines (pink)` → every pink line and pink label goes (the pink
        see-through copy stays); tick it → they come back.

  The 1.77° instead of 2° comes from the 0.5 mm noise added to the made-up balls.
- [ ] Set the flag back to 0, **before** the lab test:

```bash
sed -i 's/^    MOCAP_BAR_FAKE_MARKERS = 1/    MOCAP_BAR_FAKE_MARKERS = 0/' "$F"
grep -n "^    MOCAP_BAR_FAKE_MARKERS" "$F"     # expect: MOCAP_BAR_FAKE_MARKERS = 0
```

! With the flag at 1 the made-up balls take over whenever Motive's markers are missing or older
than 0.5 s (the log then says `source fake`), and could be recorded as a take.

---

## 3. Start the monitor

**Set up:** section 0 done, Motive streaming. Monitor pane: the PC block (0.0), then:

```bash
ros2 daemon stop      # ! every time, or the monitor may miss its topics
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] `mocap client connected: True (this PC 192.168.0.x, Motive 192.168.0.28)`; the first
      address is the PC block's `MOCAP_CLIENT_IP`. `False`: see section 8.
- [ ] `[Plan skeleton] robot Cindy (domain 86) | problem 261016_2ModulesDemo_YH | progress k/78 done`.
- [ ] `[Plan skeleton] auto-load at start-up: action 0 (B20_J_joint, Cindy), the first pending one.`
      If another action is first (progress left over), select action 0 or 1 (`B20_R_release`)
      and click `Load action`. Any J or R action of B20 gives the same Rhino pose.
- [ ] The marker table:

```
[mocap bars] 2 instrumented bar(s): bar_B20 (8), bar_B21 (8) | .../261016_2ModulesDemo_YH/MoCapMarkers.json
[mocap bar] Motive streaming IDs of the instrumented bars: bar_B20 = id 1020, bar_B21 = id 1021 (a bar whose asset does not stream is read from the unlabelled markers)
[mocap bars] bar_B20: the marker table agrees with this action's joint frames (max 0.007 mm over 8 markers).
```

  A warning `the marker table is off this action's joint frames by up to ... mm` means
  `MoCapMarkers.json` and the BarActions come from different exports. The balls then cannot sit
  on the drawn plates: re-export, stop here.
- [ ] Loading the action takes seconds, but up to a few minutes on the first start after a sync
      (3.5 min once on the desk PC); the panel does not respond meanwhile. It ends with
      `[BarAction] pink line = bar_B20 at its assembled pose (M3 start), centre [0.426, -2.278, 0.15]; the pink lines of all 1 bar(s) loaded so far stay ('Rhino bar lines (pink)' shows / hides them).`
      Every action loaded in this run adds its bar's pink line and label: a ruler of the Rhino
      poses.
- [ ] Only if Motive streams bar assets that the Rhino export has no mocap joints on:
      `[mocap bar] Motive also streams B34 (id 1034), B35 (id 1035), but MoCapMarkers.json has no marker table for them (no T20_MoCap joints in the Rhino export): they cannot be measured.`
      These bars are not drawn. Add their mocap joints in Rhino and re-export to measure them.
- [ ] Panel, section **`MoCap bar (joint markers)`** (above `CONTROLLERS`):
      `Rhino bar lines (pink)` (a checkbox, ticked), `MoCap bar: measure + draw (streaming bars)`,
      `MoCap bar: follow live (10 Hz)` (a checkbox), `MoCap bar: clear drawing`,
      `MoCap bar: record take`.

! Do not click `Mark action done` in this test: it would record B20 as built in
`progress.json`.

---

## 4. The test: B20 by eye

### 4.1 B20 at its Rhino spot, first measurement

- [ ] Set B20 down at its Rhino pose (table at the top): start end at x ≈ 1326 mm, the bar along
      −x, centre line about 150 mm up, J20-169's plate facing −y. It does not have to be exact:
      the test measures where it is.
- [ ] Nothing (hands, tools, cables) covers a ball; Motive shows the 8 balls.
- [ ] `MoCap bar: measure + draw (streaming bars)`. Expect a header line, then three lines per
      bar (B21's three too if its asset streams):

```
[mocap bar] measuring 1 bar(s): bar_B20 (asset 1020) | instrumented but not streaming: bar_B21
[mocap bar] bar_B20 | source asset (age 8 ms) | 8/8 matched | rms 0.9 mm, max 1.6 mm | dpos 4.2 mm, dang 0.31 deg | in the bar frame: dx 1.2, dy -0.8, dz(along) 3.9 mm | ok
[mocap bar] residuals (mm): J20-22_mocap_M1 0.6, J20-22_mocap_M2 0.4, ... | missing: none | stray sensed: 0
[mocap bar] bar ends (world, mm): Rhino start [1326 -2278 150] end [-474 -2278 150] | MoCap start [...] end [...]
```

  - [ ] `source asset` (the balls of Motive's B20 asset, ID 1020), `8/8 matched`, `ok`, `rms`
        about 1 mm, every residual below about 2 mm. `dpos` / `dang` are how far you put the bar
        from its Rhino pose (section 5).
  - [ ] PyBullet (zoom on B20): the cyan copy lies on (or next to) the pink one. The eight white
        crosses sit on the balls of the cyan plates, with very short black lines.

### 4.2 The cyan copy is the real bar

- [ ] **Which end:** the cyan copy's J20-22 plate (75 mm from its start tip) is at the same end
      as the real bar's J20-22.
- [ ] **Which side:** the cyan J20-169 plate faces the other way from the three others, as on the
      real bar.
- [ ] **Heights:** tape-measure from the floor to the bar's centre line (the top of the bar minus
      half its diameter) at both tips. Compare with the `z` of `MoCap start [...]` and
      `end [...]` in the last line (mm above Motive's floor). Expect agreement within a few mm.
- [ ] **Position:** if the floor has marks for the Motive origin or the foundations, tape-measure
      the real start tip against `MoCap start [x y z]`.
- [ ] Photo of the bar and of the PyBullet view from about the same direction.

### 4.3 Follow live: move the bar, the cyan copy follows

- [ ] Tick `MoCap bar: follow live (10 Hz)`: every streaming bar's cyan copy and crosses are
      re-fitted ten times a second. While all of a bar's balls are seen, each refresh starts from
      the bar's last pose (about 1 ms per bar). With a ball hidden, or after a jump, it runs the
      full search (about 25 ms per bar, the same as a click). The log prints one line per bar only
      when its fit turns ok or fails, e.g.
      `[mocap bar] bar_B20 live: fit ok (8/8 matched, rms 0.9 mm, dpos 4.2 mm, dang 0.31 deg)`.
      A click on `measure + draw` always runs the full search, and its numbers are the ones to
      note.
- [ ] **Slide** the bar about 50 mm along its axis towards the far end (−x). Click
      `measure + draw` → `dz(along)` grows by about **+50 mm** (towards the start end: −50).
      `dx`, `dy` and `dang` barely change.
- [ ] **Roll** it about 90° about its axis → the cyan plates turn with the real ones;
      `measure + draw` → `dang` about **90 deg**, `dpos` small.
- [ ] **Lift one end** about 20 mm → the cyan copy tilts; that end's height in `bar ends` goes
      up by about 20 mm.
- [ ] Put the bar back at its Rhino spot. Untick `follow live` (the last drawing stays).

### 4.4 Hidden balls

A covered ball of a tracked asset still streams from Motive, flagged *occluded*, at Motive's
guess from the asset. The fit never uses such a ball: it counts as missing.

- [ ] Cover **one** ball with a hand → `measure + draw` → `7/8 matched`, `ok`; that ball's name
      in `missing: ...` and as a red label on the cyan copy.
- [ ] Cover **three** balls → `FAILED: only 5 of 8 design markers matched (need 6)`, possibly
      followed by `(3 marker(s) occluded in Motive)`. The cyan copy disappears, and a red
      `B20 MoCap FAILED: ...` label shows. If Motive loses the asset, the log may say
      `source unlabeled` instead; the result is the same.

### 4.5 Optional: B21 too

- [ ] Put B21 (its asset 1021 enabled) in view near its Rhino pose, standing upright, with its
      bottom end about 50 mm above the floor. Click `measure + draw` → the header names
      `bar_B21 (asset 1021)`, and B21 gets its own pink and cyan copies and three lines. Nothing
      needs loading: any action of this export gives every bar's Rhino pose.
- [ ] B20's fit is unchanged: each bar is read from its own asset.
- [ ] Disable B21's asset in Motive with `follow live` ticked →
      `[mocap bar] bar_B21: its Motive asset stopped streaming; its drawing is removed.`

### 4.6 Keep the data

- [ ] `MoCap bar: record take` once at the Rhino spot, once after the 50 mm slide, once rolled.
      Each click records one take per streaming bar:
      `[mocap bar] take k recorded: bar_B20 (unsaved; 'Save markerset data' writes it)`.
- [ ] `Save markerset data` → `Bar holding acc data saved to .../bar_holding_acc_<date>_<time>.json`.
      Each take holds its bar's raw markers, the fit, the bar's marker table and its Rhino pose,
      so it can be re-fitted later.

---

## 5. Reading the numbers

All in the **Rhino bar frame**: its z runs along the bar from the start end to the far end.

| value | means |
|---|---|
| `dpos`, `dang` | how far the MoCap pose is from the Rhino pose: distance of the bar origins (mm) and the total turn (deg) |
| `dz(along)` | slid along the bar (+ = towards the far end) |
| `dx`, `dy` | moved sideways, across the bar (for B20 at its Rhino pose: −dx = world +y, dy = up) |
| `rms`, `max`, `residuals` | how well the 8 balls fit the Rhino marker table once the bar is placed. About 1 mm is Motive's noise. A single ball far off (> 2 mm) points at a plate mounted off its Rhino spot, or a ball knocked loose |
| `dang` about 180 deg with a good rms | the bar lies end for end compared with Rhino (check 4.2's "which end") |
| `AMBIGUOUS` / `two different marker assignments fit equally well` | too few balls in view to tell the plates apart (e.g. both balls of J20-169 hidden); show more balls |

---

## 6. Optional: without the asset (the unlabelled fallback)

When B20's asset is off or lost, the monitor reads the bar's balls from Motive's unlabelled
markers and matches them itself. Stray markers then show too.

- [ ] Bar at its Rhino spot. **Disable** the B20 asset (1020) in Motive → `measure + draw` →
      `source unlabeled`, still `8/8`, `ok`, the same pose as with the asset within about 1 mm.
      Note in the log whether the balls kept arriving: this answers whether Motive streams a
      disabled asset's balls as unlabelled.
- [ ] A loose marker (a spare ball) near the bar → a **red** cross, `stray sensed: 1`, the fit
      stays `ok`.
- [ ] Optional: B21's balls in view too (its asset disabled) → red crosses, `stray sensed: 8`,
      B20's fit stays `ok`.
- [ ] **Enable** the B20 asset again → `measure + draw` → `source asset`.

An asset with another ID than 10*xx* is still found when it is named `B20` (or `bar_B20`), as
Motive's bar assets are. Otherwise set its ID in `MOCAP_BAR_STREAMING_IDS`
(`husky_assembly_teleop/husky_world.py`, e.g. `{'bar_B20': 1020}`). Either way restart the
monitor: it reads Motive's asset list when it connects.

---

## 7. Afterwards: flags back

```bash
F="$HUSKY_MONITOR_PY"
sed -i 's/^    FAKE_HARDWARE = 1/    FAKE_HARDWARE = 0/; s/^    MOCAP_BAR_FAKE_MARKERS = 1/    MOCAP_BAR_FAKE_MARKERS = 0/' "$F"
grep -nE "^    (FAKE_HARDWARE|MOCAP_BAR_FAKE_MARKERS) *=" "$F"     # expect both 0
```

Before the next three-robot run, open a new terminal or set `DESIGN_PROBLEM_NAME` back to its
problem; the section 1 blocks of that manual do. `unset MOCAP_CLIENT_IP` too if that run uses
another PC's address.

---

## 8. If something goes wrong

| situation | do |
|---|---|
| the PC block prints `mocap port NONE = NONE` | no port of this PC is on `192.168.0.x`: plug the cable / dongle in, wait for DHCP (`ip -4 -brief addr`), paste the block again |
| `mocap client connected: False (this PC ..., Motive 192.168.0.28)` | the first address is not this PC's on the mocap network (an old `MOCAP_CLIENT_IP`: paste the PC block again), or the Motive PC moved (`ping 192.168.0.28`; `doc/calibration_manual.md` 1.4) |
| `rmw_create_node: failed to create domain` / `... does not match an available interface` | `CYCLONEDDS_URI` names a port that is not there (e.g. `~/.cyclonedds.xml` with another dongle's name): paste the PC block again, it pins the port it found |
| `[mocap bars] this design problem has no MoCapMarkers.json` | wrong `DESIGN_PROBLEM_NAME` or `HUSKY_PC` (section 0.0), or the file has not synced (0.2) |
| `[mocap bars] ... schema_version 1 ...` (refused) | an old export: re-export from Rhino (0.2) |
| `[mocap bar] nothing to measure: bar_XX carries no MoCap markers and no instrumented bar streams from Motive (instrumented: bar_B20, bar_B21).` | the loaded action's bar has no mocap joints, and neither B20's nor B21's asset streams: enable them in Motive (0.3) |
| a bar with balls on it is not drawn, and the start-up says `Motive also streams Bxx (id 10xx), but MoCapMarkers.json has no marker table for them ...` | the Rhino export has no T20_MoCap joints on that bar: add them in Rhino and re-export |
| `[mocap bar] no markers from Motive yet, neither from bar_B20's asset (id 1020) nor unlabelled: ...` | nothing has ever arrived: `mocap client connected: False` (rows above), Motive not streaming, or the B20 asset is not 1020 and **Unlabeled Markers** is off (0.3, section 6) |
| `FAILED: the marker snapshot is ... s old (limit 0.5 s); is Motive streaming?` | the markers stopped arriving: Motive stopped streaming, or the asset was lost while **Unlabeled Markers** is off (0.3) |
| `... (k marker(s) occluded in Motive)` after a FAILED reason | Motive does not see those balls of the asset (it only guesses them): free them, or check Motive's view of the bar |
| `FAILED: only k of 8 design markers matched (need 6)` | balls hidden: look at `missing: ...` and the red labels; free those balls |
| `FAILED: no three sensed markers match any design triangle (wrong bar, or wrong unit / axis convention?)` | the balls in view are not B20's (another bar loaded?), or `MOCAP_AXIS_CONVENTION` is not `"rhino"` |
| `FAILED: too many markers in view (N, at most 24)` | only when reading unlabelled markers (`source unlabeled`): many loose markers in the volume (the robots' balls arrive under their own assets and do not count). Enable the B20 asset (1020), or clear the volume |
| `... marker(s) dropped: size outside 12-26 mm or error above 3 mm` | Motive sees the balls badly (merged, or seen by too few cameras): check Motive's view of the bar |
| `source fake` in the lab | `MOCAP_BAR_FAKE_MARKERS` is still 1: set it to 0 (section 2), restart |
| `[mocap bar] the Motive asset bar_B20 streams as id N, which is already '...'` | that ID belongs to another tracked body in the monitor: give the B20 asset its own ID (1020 by the convention) |
| `the marker table is off this action's joint frames by up to ... mm` | `MoCapMarkers.json` and the BarActions come from different exports: re-export both from Rhino |
| the cyan / pink copies are not drawn, only the lines and labels | PyBullet's mesh buffer is full with this export: the lines, crosses and labels still carry the check; note it |
| the drawing vanished after `Remove all drawing` | click `measure + draw` again (the pink assembled lines come back by themselves within a second while `Rhino bar lines (pink)` is ticked) |

---

## 9. What to send back

- The monitor log (`~/husky_real/mocap_bar.log`, or copy the terminal): the `[mocap bars]` lines
  at the load and every `[mocap bar]` block.
- The saved take file (4.6).
- Photos (4.2) of the real bar and the PyBullet view from the same direction, at the Rhino spot.
- For 4.2: the tape-measured heights at both tips next to the logged `MoCap start / end` z.
- For 4.3: the `dz(along)` after the 50 mm slide and the `dang` after the 90° roll.
- Answers to the open points in `tasks/2026-10-16_bar_mocap_fit.md` (NatNet is 4.0, seen
  2026-10-16):
  - whether the asset's balls arrive unlabelled when the asset is disabled (section 6);
  - the largest residual seen with all 8 balls in view (if above 2 mm, the matcher's 2 mm side
    tolerance should go up).
