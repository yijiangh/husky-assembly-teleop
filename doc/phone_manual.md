# Phone manual

The monitor serves a second page for a phone, for walking next to the robots: **Stop all**, the health banner, and a
tab per plugin to drive a base, open or close a gripper, free drive an arm, record mocap points or make the floor
calibration, and confirm an assembly step. Everything else stays on the laptop. This manual connects the phone and
says what each tab is for; the plugins' own manuals have the rest.

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

| Tab | Needs | For |
|---|---|---|
| **Control** | `robot_control` | driving a base, the grippers, free drive |
| **Health** | always there (`health` is a default plugin) | the health chips |
| **Mocap** | `mocap_probe` | recording points, the floor calibration ([floor_calibration_manual.md](floor_calibration_manual.md)) |
| **Exec** | `movement_exec` (loaded with `assembly_steps`) | confirming an assembly step next to the robot ([real_lab_test_manual.md](real_lab_test_manual.md)) |

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

Buttons that delete or export act on a second tap: the first turns them red, and a second within 3 s does it.

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

For the mocap probe (`mocap_probe`; choose its rigid body on the laptop). On top a big status, readable at arm's
length: TRACKED, NOISY or NO FIX, the probe's x y z and marker error, flashing RECORDED with each point's name. Below
it two tabs:

- **Points**: **Label** and **Record point** (numbered per label: `table_1`, `table_2`, …; refused without a fix),
  **Delete all points**, **Map coverage** with a top-down map of where the probe is seen well (colours: README,
  `mocap_probe`; obstacles dark grey, robots blue, points pink, the probe a white dot, grey without a fix; +x right,
  +y up), and the last points. Export and delete single points on the laptop.
- **Floor**: the floor calibration, made walking from mark to mark: [floor_calibration_manual.md](floor_calibration_manual.md).

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
