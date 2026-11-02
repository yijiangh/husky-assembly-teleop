# Experiment data on the Drive

Experiment data lives in the project's Google Drive folder, synced to your PC (e.g. with Insync), not in this repo.
Point the monitor and the analysis scripts at your local copy of the `2025-03 Husky Assembly` folder itself; the path
above it differs per user.

```shell
# In ~/.bashrc, or each terminal: read by the monitor and the scripts
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"

# Or for one monitor run; the parameter wins over the variable
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" -p plugins:="['base_exp']" \
    -p drive_root:="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

Paths are given relative to this folder, e.g. `data_design_study/260814_RobArch_support_ik`; absolute paths still
work, for data elsewhere. `base_exp` does not start without the root and its `data_experiment/base_exp/` folder;
`process` needs the root to load a relative process folder.

## Where runs go

Every plugin saves its runs the same way (`shared/recordings/`): under its own folder `data_experiment/<plugin>/`
(e.g. `data_experiment/base_exp/`), a folder per experiment name, and in it a folder per run,
`<date>_<time>_<robot>_<what>_<sim|real>/`, with `experiment.json` (what the plugin knew: setup, conditions,
outcome), `recording.npz` (the signals, per tick and per message; `recording.json` with `recording_format:=json`)
and, once analysed, `report.json` and pictures. Load a run with `load_recording(folder)`. A new plugin needs only its
`data_experiment/<plugin>/` folder, made in Drive.

An assembly run is different: `assembly_run` keeps it in `runs/<YYYYMMDD_HHMMSS>/` inside the process folder
([run_record.md](run_record.md)).

## Simulated and real data

Simulated and real data share their folders; every file says which (`"environment": "sim" | "real"` and
`"ros_domain"`, `drive.environment`). The one exception is what the mocap relay applies, the base and floor
calibrations: both go to the dated sessions `data_experiment/calibration_data/<YYYYMMDD>/`, and a simulation's to
`data_experiment/sim/calibration_data/domain_<ROS_DOMAIN_ID>/<YYYYMMDD>/`, so a simulated calibration never reaches a
robot (one marked sim in the robots' folder is refused). What counts as simulated is where the data came from (a
simulated robot, a fake Motive), not the ROS domain.

## Who creates folders

Folders directly under the root and its data folders (e.g. `data_experiment/<plugin>/`) are never created by the
code: create them in Google Drive and sync them instead. The reason: with selective sync, a folder can exist in Drive but be
missing on your PC, because it is not synced there. If the code then created it locally, Insync would upload it as a
new folder, and Google Drive, unlike a normal file system, allows several folders with the same name side by side. You
would end up with two `base_exp` folders, the runs split between them. The code only creates new folders inside an
existing one (an experiment, a run), which cannot clash.

One exception: a relay started with `sim:=true` and the Drive in use creates
`data_experiment/sim/calibration_data/domain_<ROS_DOMAIN_ID>/` and any missing folder above it, so make sure
`data_experiment/sim/calibration_data/` exists in Drive first.
