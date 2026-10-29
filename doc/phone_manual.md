# Phone manual

The monitor serves a second page for a phone, for walking next to the robots: **Stop all**, the health banner, and a
tab per plugin to drive a base, open or close a gripper, free drive an arm, or record mocap points. Everything else
stays on the laptop.

> **Safety.** **Stop all** on the phone is the monitor's soft stop, not an emergency stop: keep the e-stop in reach.
> Every hold button (D-pad, Free drive) acts only while you hold it; when you let go, or the phone loses the page,
> the robot stops within 0.25 s.

## 1. Once per laptop

The phone reaches the laptop over the offline network (the robot WiFi is bridged to the wired lab LAN, 192.168.0.x).
If the laptop runs a firewall, let that network in (and keep everything else out, e.g. eduroam):

```bash
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from 192.168.0.0/24
sudo ufw enable
```

## 2. Start the monitor

Start it as usual (README, "Start the monitor"). The tabs come from the plugins you load:

| Tab | Needs |
|---|---|
| **Control** | `robot_control` |
| **Health** | always there (`health` is a default plugin) |
| **Mocap** | `mocap_probe` |
| **Exec** | `movement_exec` (loaded with `assembly_steps`) |

The ports are fixed so a bookmark keeps working: desktop **8080**, phone **8081**. If one is taken, usually by a
monitor that is still running, the monitor refuses to start and says which port. A second monitor on purpose:
`-p viser_port:=8095` (its phone page is then 8096).

## 3. Open the page on the phone

1. Join the robot WiFi.
2. Find the laptop's address on the offline network, the one starting with `192.168.0.`:

   ```bash
   ip -4 -br addr
   ```

   Not the eduroam address (10.x): the robot WiFi does not reach it, and it changes from day to day.
3. On the phone, open `http://192.168.0.<n>:8081`. Type the `http://`: some browsers try `https://` first and fail
   without saying why.
4. Hold the phone upright. The phone layout needs a narrow screen; turned sideways, the page shows the laptop
   layout.
5. Bookmark it.

The robot WiFi has no internet. If the phone asks whether to stay connected, say yes, and if the page does not load,
turn off mobile data: phones send everything over mobile data when the WiFi has no internet.

## 4. The page

From top to bottom:

- **Husky Phone** (top left): a green cloud means the page is linked to the monitor. Red dots mean the link is lost:
  nothing you press reaches the robots then, and the page reconnects by itself.
- **Stop all**: stops every arm's program, every tool and every base, and cancels the plugin tasks that may move a
  robot, as the laptop's Stop all and Esc do. The robots refuse every motion command until Resume.
- **Health banner**: "ALL GREEN", or how many problems there are; shown on every tab.
- **Tabs**: one per plugin, below.

### Control

One sub-tab per robot, labelled with its serial (`0804`, `0805`, `0806`).

- **Status line**: "D-pad drives" in green, or the running base controller and "D-pad off: switch on the laptop".
- **Speed**: a fraction of the full speed (0.3 m/s, 0.5 rad/s). Separate from the laptop's slider.
- **D-pad**: forward, back, turn left, turn right; drives while held. It drives only while the base runs its
  velocity controller (**Vel**); switch to it in the laptop's robot panel.
- One folder per arm (**Arm**, or **Left** and **Right**):
  - The running arm controller: **Joint**, **Cart**, **Free**, or "no ctrl".
  - **Gripper**: **Open**, **Close** and **Stop** (no Stop on the v1 tool, which cannot stop halfway), one command
    per press, the same as on the laptop. The screw and Reactivate are on the laptop.
  - **Free drive (hold)**: while you hold it, the arm can be pushed by hand (its chip shows **Free**). When you let
    go, the joint controller takes over and holds the arm where it is; it does not go back to the Cartesian
    controller. Check the payload on the pendant first: a wrong one makes the arm sag or rise. Free drive interrupts
    a move the arm is running.

### Health

The same chips as the laptop's health panel: one row per robot (base and arms) and one for the tracked objects.
Phones have no hover, so a chip's detail is only on the laptop. Unlock, Resume and Reconnect are on the laptop too.

### Mocap

For recording points with the mocap probe (`mocap_probe`); choose its rigid body on the laptop.

- **Big status**: one colour and one word (TRACKED, NOISY, NO FIX), the probe's x y z and its marker error; it
  flashes RECORDED with the point's name. Below it two tabs, **Points** and **Floor**.
- **Label** and **Record point**: points are numbered per label (`table_1`, `table_2`, …). Refused while the probe
  has no fix; a noisy fix records, with its marker error.
- **Delete all points**: deletes every recorded point. The first tap turns it red ("Tap again: delete N"); a second
  tap within 3 s deletes. Export and delete single points on the laptop.
- **Map coverage**: the same switch as in the laptop's Debug folder. While it is on, a top-down map shows the coverage map with 1 m grid lines
  (light grey until visited, then by mean marker error: green up to 1 mm, deeper the lower,
  amber to deep orange up to 2 mm, red above, greyer where often lost, grey if never tracked), obstacles in dark grey, robots
  in blue, recorded points in pink and the probe as a white dot (grey without a fix); +x right, +y up. It is redrawn
  at most twice a second.
- The last points.
- **Floor** (floor calibration): **Floor point** records a `floor` sample; spread them over the area. The plane is
  refitted after each one: points, rms and max (the point farthest from the plane), tilt from the mocap z and how
  well the normal is known (± per axis). The chip turns green from 6 points with the normal known to 0.02° per
  axis, and red (no export) while the points sit in one spot or on a line. **Mark** (Origin, Mark 1–3) records the
  mark on the floor under the tip; it is fitted into the plane too, and probed again it averages. The lab frame is
  fitted to the marks probed and the anchors read, against their places stored in git (two at least); the
  chip shows how many, and turns red when a mark lands more than 2 mm from where it is stored (hit badly,
  or the mark moved). The anchors stored in git are found by themselves, each the marker within 10 cm of its stored
  place, read once the probe is 0.5 m away, while the relay applies a floor calibration. **Anchor**: only for one the
  "anchors mm" line asks for ("point": not found near its stored place) or a new one: point the tip at the marker and
  press; the tip is recorded, and the anchor is the marker nearest it (within 10 cm), fitted with the marks. An
  anchor over 2 mm from where it is stored has moved: it is left out of the fit (amber chip, "moved"), and the export
  keeps it for the relay, with its new place in `anchors_in_floor` for updating the lab frame file. The "marks mm"
  line shows each mark's distance from where it is stored, on the floor ("new" when probed but not stored), "anchors
  mm" each anchor's, in 3D ("–" while not read, "point" to point at it). "marks vs
  anchors" says how far the marks alone and the anchors alone place the lab apart. "poses off mm" checks the frame
  the relay applies, at the lab volume's worst corner: how far the marks alone place the lab from it ("marks"), how
  far the relay's trusted fit does ("relay", its drift estimate); "marks vs relay" how far those two are apart. To
  sanity check the calibration, probe a few floor points and the four marks (no export needed): "marks vs relay"
  should stay under about a millimetre. The last line shows the probe's height above the plane, live. **Export floor calibration** saves `floor_from_raw` to today's calibration session
  on the Drive (`data_experiment/calibration_data/<YYYYMMDD>/`), on a second tap within 3 s like the delete buttons;
  refused (saying why) while the frame cannot be placed or the relay announces no cameras. With a warning (a mark or
  anchor off, an anchor not read) it says why and turns orange: a second press within 2 s exports anyway, on the
  laptop too.
  The last result shows in full at the bottom of the tab. To drop a bad floor point, select only it in the laptop's
  list and Delete: the plane refits. **Delete floor points** deletes the floor points, marks and anchors only, with
  the same second tap.

### Exec

For running an assembly step next to the robot (`movement_exec`); the step is started with **Exec** on the laptop.

- The status line: what runs, and whether a prompt waits.
- The prompt, naming the robot, then **Confirm** (do what it says) and **Cancel** (answer no; with no prompt open,
  stop the running step and hold every arm where it is).
- The last result.

## 5. After a stop

- **Stop all**: the arms' programs are stopped, the base controllers are off, and the robots refuse every motion
  command. **Resume** in the laptop's health panel starts both again; the phone cannot. Letting go of a hold button
  after Stop all starts nothing again.
- **E-stop**: the same, and the Robotiq gripper may stay dead even after the arm runs again (crl-husky README,
  "Restart"). Wait until the arm runs, press Reactivate on the laptop, and only then use Open and Close: presses while
  the gripper is dead are queued on the robot and may all run at once when it comes back.

Several people can drive at once, from the laptop and from phones: they share one lease (robot_control's), so nothing keeps them apart. Each press drives for only
0.25 s after its last report, so talk to each other.

## 6. When something does not work

| What you see | Likely cause | What to do |
|---|---|---|
| The page does not load | Wrong address (eduroam, or it changed), mobile data, firewall | Section 3; check the address on the laptop; turn off mobile data; section 1 |
| The page shows the laptop layout | The phone is sideways, or the screen is wider than 576 px | Hold it upright |
| Red dots instead of the green cloud | The phone lost the WiFi, or the monitor stopped | Wait for it to reconnect; check the monitor's terminal |
| D-pad does nothing | The base does not run **Vel** (status line amber) | Switch it in the laptop's robot panel |
| Free drive does nothing | The arm's controller manager is not reachable (arm chip "no ctrl"), or the safety sync does not vouch for the arm (no status, not operational, program stopped) | Check Health; the monitor asks again every second while you hold |
| Gripper does not react | After a stop or e-stop the gripper link is down | Section 5 |
| Nothing changes on the page | The laptop's panels are paused (its **Live** button shows **Paused**) | Press it on the laptop: pausing stops the phone's updates too |
