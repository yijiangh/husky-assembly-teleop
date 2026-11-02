# Drive layout: sim marked in the file, calibrations together

## Decisions (JG, 27 Oct)

- Simulated and real data share their folders; every file says which: `"environment": "sim" | "real"` and
  `"ros_domain"` (`drive.environment`). Files without the mark count as real.
- Only what the relay applies gets a folder of its own for simulations: base and floor calibrations.
- Base and floor calibrations live together in the dated sessions `data_experiment/calibration_data/<YYYYMMDD>/`
  (floor calibration flat: `<date>_<time>_mocap_floor_calibration.json`); a simulation's in
  `data_experiment/sim/calibration_data/domain_<id>/<YYYYMMDD>/` (`calibration_sessions.sessions_folder`).
- Sim is decided by where the data came from, never by the ROS domain: a simulated robot (calibration plugin), a
  simulation's relay (probe, mocap_bars). A real floor calibration made from domain 70 is real.
- A relay never mixes real and simulated inputs (28 Oct): `sim:=false` (default) reads Motive over NatNet only,
  with the robots' sessions; `sim:=true` (refused in the robots' domain) reads only a simulation's inputs (fake
  Motive over simulator truth per body), with the simulation's sessions, base and floor calibrations alike. Fixed for
  the relay's run. `~/calibrations` announces `simulated`, `~/frame` the sessions `folder`, where the probe writes.
  This drops the "simulated robot among the lab's real objects" setup; a simulated robot under Motive's relay gets no
  mocap pose.
- Probe measurements (points, coverage) stay in `data_experiment/mocap_probe/`, marked by the relay's mode.
- Run folder names keep their `_sim`/`_real` part (browsing); the file is what counts.

## What changed

- `drive.py`: `environment`, `marked_sim`, `ros_domain`.
- `world/calibration_sessions.py`: `sessions_folder`, `misplaced_problem` (a sim-marked file outside the simulations'
  folders is refused); `SIM_FOLDER`, `SIM_SESSIONS_FOLDER` gone.
- `world/mocap_frame.py`: `newest_floor_calibration(sessions)` searches the dated session folders; `FLOOR_FOLDER`,
  `SIM_FLOOR_FOLDER`, `floor_folder` gone.
- Relay: `sim` parameter (`launch/mocap.launch.py` too; `node.simulated_mode` guards it), one calibration set,
  `simulated` and the sessions folder announced (`MocapRelayState.simulated`, `FloorCalibrationState.folder`); no
  NatNet client with `sim:=true`. The fake lab's relay runs with `sim:=true`.
- Probe, calibration plugin (run file, calibration file), mocap_bars, example recording: marked.
- Docs: README (Drive section), mocap_relay.md, simulation_manual.md, calibration_manual.md, base_exp_manual.md,
  base_planner_manual.md, phone_manual.md, plugin_roadmap.md, lab check.

## Drive migration

Mock Drive (this PC) done with the move list below. On the real Drive, the same moves by hand:

- `mocap_probe/*_mocap_floor_calibration.json` → `calibration_data/<date>/`.
- `sim/mocap_probe/domain_<id>/` floor calibrations: made on the lab's Motive → `calibration_data/<date>/`; from a
  fake Motive → `sim/calibration_data/domain_<id>/<date>/`. Old refit files (`*_frame_*.json`) are no longer read:
  they can stay beside their floor calibration or go.
- Then remove the empty `sim/mocap_probe/`.
