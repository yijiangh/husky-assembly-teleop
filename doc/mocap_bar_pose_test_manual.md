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
gripper or tool, and nothing moves. Motive must stream. The exception is section 10, which runs on
Cindy: the visual servo of a grounded bar aims the bar by this MoCap fit instead of tool0.

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
      `MoCap bar: record take`, `As-built: record now`. The servo's checkbox
      `Servo feedback: MoCap bar markers (unticked = tool0)` sits under `MOVEMENT EXE`, below the
      spiral box (only used with Cindy in section 10). `As-built: record now` writes one line of
      the as-built record of an assembly run (`doc/three_robot_assembly_test_manual.md` section 9);
      this test does not need it.

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

---

## 10. With Cindy: the servo on the MoCap bar (grounded B20)

Sections 0–9 run without a robot. This section runs **action 0, `B20_J_joint`, on Cindy**, twice:
once with the servo aiming the **bar** by its MoCap fit, once aiming **tool0** as before. Both runs
print, and save, the same comparison of three bar poses:

| name in the log | what it is |
|---|---|
| **design** | where Rhino wants the bar at that moment: the insert's start (15 mm above B20's assembled pose) during the first servo, the assembled pose during the second |
| **FK** | where the robot thinks the bar is: mocap base + arm joints (FK) → tool0, composed with the Rhino grasp (tool0 → bar) |
| **MoCap** | where the bar is: the fit of its 8 joint balls (sections 4–5) |

`MoCap-vs-FK` is the error the tool0 servo cannot see. It is the sum of the base calibration, the
arm, the tool mount on the flange and the joint mount on the bar. With the box ticked the servo
moves both flanges by the one rigid correction that puts the MoCap bar on its design pose, so
`MoCap-vs-design` goes under 0.5 mm / 0.1°. Unticked, `FK-vs-design` goes under 0.2 mm (tool0)
and `MoCap-vs-design` stays what the 20261001 test measured.

Only a **grounded** bar (held on its ground joints, nothing to mate with) is aimed by MoCap. Every
other bar keeps the tool0 servo, with the MoCap bar recorded alongside when it has a marker table.
In this export only B20 and B21 have one. B34, the other grounded bar (action 7), has none, so on
B34 the box falls back to tool0 and says so.

### 10.1 Set up

- [ ] The robot as for `doc/three_robot_assembly_test_manual.md`:
  - Cindy brought up (its 0.3 Cindy) and checked (2.1 Cindy);
  - safety as in its section 4;
  - the flags of its 0.1, **except** that `DESIGN_PROBLEM_NAME` stays `261016_2ModulesDemo_YH`
    (this manual's PC block, with `HUSKY_PC=robotpc` on the robot-control PC). The three-robot
    manual's own `261006_3bar_holding_test` works too: it borrows B20's markers from
    `261016_2ModulesDemo_YH` (`MOCAP_MARKERS_PROBLEM_NAME`; run that way 2026-10-20).
- [ ] Flags (section 0.1 of this manual) differ from the by-eye test:

| flag | value for this section |
|---|---|
| `FAKE_HARDWARE` | **0** (Cindy moves) |
| `USE_MOCAP` | **1** |
| `MOCAP_BAR_FAKE_MARKERS` | **0** (with 1 the servo is told `fake markers ... never steered on` whenever Motive misses a ball) |
| `CONNECT_COMPLIANT_CONTROLLER`, `LIST_CONTROLLER_SERVICES` | 1 |

```bash
F="$HUSKY_MONITOR_PY"
sed -i 's/^    FAKE_HARDWARE = 1/    FAKE_HARDWARE = 0/; s/^    MOCAP_BAR_FAKE_MARKERS = 1/    MOCAP_BAR_FAKE_MARKERS = 0/' "$F"
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|MOCAP_BAR_FAKE_MARKERS) *=" "$F"     # expect 0, 1, 0
```

- [ ] Motive streams B20's asset (1020) with its 8 balls (0.3). B20 is on its loading spot, its four
      mocap plates mounted (0.4), and nothing will cover the balls while the arms hold it.
- [ ] Start the monitor (section 3). Under `MOVEMENT EXE`, below the spiral box, is the checkbox
      **`Servo feedback: MoCap bar markers (unticked = tool0)`**, unticked by default; the start-up
      log says `servo feedback: tool0` in yellow. Since 2026-10-22 the box also works for a screwed
      bar that carries markers (B21, B35); that use has not run on Cindy yet.
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)`, as in the
      three-robot manual's 5.2 a).

### 10.2 Run 1: box ticked (the servo aims the bar)

Tick `Servo feedback: MoCap bar markers (unticked = tool0)` (the log says `servo feedback: MoCap bar`
in green), then run action 0 as the three-robot manual's
5.2 describes: IK check, `Plan Movement`, `Exec Selected Mv Traj (auto)` on the transfer, then the
mount pause. What is new:

- [ ] At the exec: `[transfer B20_J_M3_CDFM_transfer_to_approach] servo feedback: MoCap bar`.
- [ ] After the mount pause, the insert is loaded and the servo starts with
      `[servo] feedback: the MoCap bar bar_B20 (converges at 0.5 mm / 0.1 deg).`
- [ ] Every reading prints one bar line next to the tool0 line, e.g.
      `[servo 0] bar bar_B20: FK-vs-design 0.05 mm/0.002°, MoCap-vs-design 4.31 mm/0.125°, MoCap-vs-FK 4.29 mm/0.124° (fit 8/8, rms 0.6 mm, asset)`.
      After it: `[IK Live Base] target is BAR-CORRECTED (servo on the MoCap bar): ...`.
- [ ] PyBullet: the **pink** B20 copy stands at the insert's start (15 mm above its assembled
      pose) while the servo runs. The **cyan** copy (the MoCap fit) is redrawn at every reading
      and closes on the pink one.
- [ ] The first move pauses for `Confirm Exec` as before (a few mm, the bar held). Later passes
      run alone unless a path travels more than 5°.
- [ ] The `Servoing Live Tracker`'s tool0 curves level off at the FK-vs-MoCap gap instead of going
      to 0. That is expected: tool0 is moved off its Rhino target on purpose. Read the bar lines.
- [ ] It ends with one of:
      - `### SERVOING converged on the MoCap bar at iter N: dpos 0.31 mm, dang 0.042 deg (< 0.5 mm / 0.1 deg)`;
      - `### SERVOING stopped at iter 1: the MoCap bar is already on its target (...)`;
      - `### SERVOING ran all 8 iterations without converging.` (note the last bar line; the bar
        stays where the last pass put it).

      Then **the line this test is about**:
      `### bar pose at termination (bar_B20): FK-vs-design ..., MoCap-vs-design ..., MoCap-vs-FK ...`,
      followed by `[servo] the MoCap-bar correction is carried into the plan of B20_J_M4_LM_insert.`
- [ ] **Insert**: `Plan Movement` →
      `[insert plan] MoCap-bar correction carried: B20_J_M4_LM_insert left goal X mm / Y deg from the authored end.`
      X is about the `MoCap-vs-FK` gap: the insert makes Rhino's 15 mm move down from where the
      servo put the bar, not from the Rhino start.
- [ ] `Exec Selected Mv Traj (auto)` → the rigid insert as in the three-robot manual's 5.2 d) (`grounded bar ... rigid insert done (no tighten).`). Then, new:
      `[insert B20_J_M4_LM_insert] servo to the start of B20_J_M5_manual_fix_foundation (the insert's end), feedback: MoCap bar; straight-line corrections only.`
      The manual step `B20_J_M5_manual_fix_foundation` is now loaded, and the **second servo** runs
      on the assembled pose (the pink copy moves down to it):
  - [ ] The first correction pauses for `Confirm Exec`: scrub `Traj viz time` (sub-mm to a few mm
        with B20 on the floor), then confirm.
  - [ ] Watch the load lines
        `[servo N] load L=... N / ... Nm (+... N vs iter 0) | R=... | force change since iter 0 L=... R=... N`.
        The **force change** is what counts. Pressing on the floor can make the printed load
        *smaller*, because the floor takes the bar's weight, while the change still grows. The servo
        stops by itself when the change at either flange passes 20 N:
        `### SERVOING stopped at iter N: the force on an arm changed by ... N since iter 0 (limit 20 N); the bar may be pushed against something.`
        `Cancel Exec` stops it any time. The insert had finished, so where the arms are is still
        carried forward:
        `[insert B20_J_M4_LM_insert] 'Cancel Exec' during the servo after the insert: the insert itself had finished, so where the arms are now is carried forward.`
  - [ ] It ends with `converged on the MoCap bar` (or `already on its target`), then the
        `### bar pose at termination (bar_B20): ...` line for the **placed** bar.
  - `[transfer plan] the straight-line bar move could not be solved (IK or collision along the line); straight lines only here, so nothing is planned.`
    then `Servoing: replan produced no trajectory; stopping.`: the correction on the floor could
    not be planned. Note the lines; B20 stays where the insert put it.
  - `[insert B20_J_M4_LM_insert] no force reading: the servo's load guard is OFF. ...`: the force
    sensors are not streaming. Watch the arms closely, or `Cancel Exec`.
- [ ] **Fix foundation** as in the three-robot manual's 5.2 e). Before `Mark action done`, keep the two records (10.4).

### 10.3 Run 2: box unticked (tool0, as before)

Redo action 0: un-mount B20 and put it back on its loading spot, then `Reopen action (reload clean)`
(three-robot manual 7.a). Untick the box and run it exactly as in 10.2.

- [ ] `[transfer ...] servo feedback: tool0`, then
      `[servo] feedback: tool0 (0.2 mm); the MoCap bar bar_B20 is recorded alongside.`
- [ ] The bar lines are printed at every reading as in run 1, but the loop aims tool0. It ends with
      `### SERVOING converged at iter N (< 0.2 mm)` (or `nothing left to plan`), then the
      `### bar pose at termination (bar_B20): ...` line. Seen 2026-10-20 instead, in both servos:
      `### SERVOING ran all 8 iterations without converging.`: the right flange stays 1–3 mm from its
      Rhino target (the bar as mounted by hand) while the left one is within 0.2 mm; the bar moved
      under 0.5 mm after the first pass.
- [ ] No `correction carried` line: the insert plans to Rhino's end.
- [ ] After the insert the second servo runs too, on tool0 (`feedback: tool0`).

### 10.4 Reading and keeping the numbers

| in run 1 (bar) | in run 2 (tool0) |
|---|---|
| `MoCap-vs-design` < 0.5 mm / 0.1° | `MoCap-vs-design` = the old placement error (about 4 mm in 20261001) |
| `FK-vs-design` ≈ the gap: the robot believes the bar is off | `FK-vs-design` < 0.2 mm: the robot believes the bar is on target |
| `MoCap-vs-FK`: the robot's own error, unchanged | `MoCap-vs-FK`: about the same value |

`MoCap-vs-FK` should come out about the same in both runs and both servos: it is a property of
the robot (calibration, arm, tool and joint mounts), not of the loop. If it changes a lot between
the servo at the insert's start and the one on the floor, the bar moved in the grippers when its
feet touched down.

Records, under `$EXPERIMENT_DATA_DIRECTORY/bar_action_live_replan_exe/<date>-servoing/`:
- `B20_J_M4_LM_insert_servoing_data_<ts>.json` / `_performance_<ts>.png`: the servo before the insert;
- `B20_J_M4_LM_insert_end_servoing_data_<ts>.json` / `_performance_<ts>.png`: the servo after it.

Each JSON names its mode (`servo_feedback`: `mocap_bar` or `tool0`) and its bar (`bar_name`). Each
reading in `servoing_data` carries a `bar` entry:
- `fk_vs_design`, `mocap_vs_design`, `mocap_vs_fk`, each as `dpos_mm`, `dang_deg`, and the same
  split in the bar's frame;
- the three 4x4 poses `T_design`, `T_fk`, `T_mocap`;
- the fit;
- `usable` and `reason`.

The PNG adds the bar's distance from its design pose, dashed for MoCap and dotted for FK.

The same run also writes the **as-built record** (every MoCap bar at the start and end of every
step, `<date>-asbuilt/` next to `<date>-servoing/`); its page shows the transfer's end as
`before insert` and the insert's end as `after insert`, with the robot-visible and robot-blind
parts of the error side by side. See `doc/three_robot_assembly_test_manual.md` section 9.

### 10.5 If something goes wrong

| the log says | do |
|---|---|
| `'Servo feedback: MoCap bar markers' is ticked, but this run aims tool0: bar_B20 carries no MoCap markers.` | no marker table for B20: neither `DESIGN_PROBLEM_NAME` nor `MOCAP_MARKERS_PROBLEM_NAME` (default `261019_2ModulesDemo`) has a `MoCapMarkers.json` with B20 (0.2; three-robot manual 9.1) |
| `[servo N] bar bar_B20: ... not usable: fit failed: only k of 8 design markers matched (need 6)` | balls hidden by the arms or the tools: look at Motive; the loop aims tool0 until the fit is good |
| `... not usable: fake markers (desk rehearsal; Motive does not stream the bar), never steered on` | `MOCAP_BAR_FAKE_MARKERS` is 1 and Motive's B20 balls are missing: set it to 0 (10.1), check the asset |
| `... not usable: MoCap-vs-design ... is beyond the 30 mm / 3 deg gate ...` | the fit found another bar, or B20 sits far off its target (e.g. a wrong mount): check the cyan copy against the real bar (4.2) |
| `### SERVOING stopped at iter N: the MoCap bar fit is lost (...); the arms stay at the last bar-corrected pose.` | the balls went out of view after the loop had started on them: free them, then `Load Movement` on the step and re-run, or carry on |
| `### SERVOING converged at iter N on tool0 (< 0.2 mm): the MoCap bar fit was never usable, so the loop aimed tool0.` | the bar was never measured well enough: this run is a tool0 run (its record says `servo_feedback: tool0`), redo it |
| `### SERVOING stopped at iter N: nothing left to plan (...); the MoCap bar is ... from its target` | the correction left is too small to move the flanges (a turn about the bar's own axis barely moves them): fine, the bar line says where the bar is |
| `[insert ...] the flanges stopped X mm from the end of the insert (limit 3.0 mm): no servo after it.` | the insert did not run to its end (e.g. the arms were not under `scaled_joint_trajectory_controller`): look at the insert's own lines above it |
