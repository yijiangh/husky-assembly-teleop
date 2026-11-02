# Documentation

To install and start the monitor, begin with the [README](../README.md). `setup/` and `lab/` hold everything else
needed to run the new monitor.

## setup/: getting a PC ready

| Doc | What it covers |
|---|---|
| [monitor_pc_setup.md](setup/monitor_pc_setup.md) | For a coding agent setting up the new monitor on another lab PC, beside the old monitor: what is new (repos, apt and Python packages, the Zenoh router, time sync, ssh keys, the Drive), the steps, the checks and what to report. |
| [experiment_data.md](setup/experiment_data.md) | The project's Drive: pointing the monitor at it, where runs and calibrations go, sim and real data, who creates folders there. |
| [old_monitor.md](setup/old_monitor.md) | What changed from the old monitor, running it, and migrating a workspace to the new one. |

## lab/: running the robots, real or simulated

| Doc | What it covers |
|---|---|
| [plugins.md](lab/plugins.md) | The UI's panels, your own layout, the Assembly tab, the phone page, launch examples, and every plugin in detail. |
| [real_lab_test_manual.md](lab/real_lab_test_manual.md) | The real robots from a laptop on the lab network: the one-time setup (router, time sync, ssh keys), the robots' launches, read-only checks, tests in the air, then the first three bars step by step; `scripts/real_lab/tmux.sh`. |
| [fake_lab_test_manual.md](lab/fake_lab_test_manual.md) | The same test against three simulated robots, a fake Motive and the relay on one laptop, isolated on Zenoh; known issues; `scripts/fake_lab/tmux.sh`. |
| [real_lab_short_manual.md](lab/real_lab_short_manual.md) | The real-lab test, short: what runs where, the plain commands to start it (`scripts/lab_tmux.sh real` types them into tmux panes), the steps of the first three bars. |
| [fake_lab_short_manual.md](lab/fake_lab_short_manual.md) | The fake-lab test, short: the plain commands on a laptop in its own domain, a scratch Drive, what differs from the lab. |
| [lab_tools_test_manual.md](lab/lab_tools_test_manual.md) | The operator's tools (the tool0 jog, the live plots, the relay's base offset, Check tool0, Rhino bar lines, the Exec plots, the compliant parts and more), tested in simulation and on a robot. |
| [simulation_manual.md](lab/simulation_manual.md) | Simulated robots and mocap: one robot, several, realistic mocap, base experiments, every argument, what is not simulated. |
| [phone_manual.md](lab/phone_manual.md) | The phone page: connecting a phone, what each tab is for, what to do after a stop, troubleshooting. |
| [mocap_relay.md](lab/mocap_relay.md) | The mocap relay: topics, calibrations, the lab frame and how it follows Motive's frame, rigid bodies in Motive, launch arguments, simulated inputs, what changed from the old monitor's direct NatNet. |
| [ur_frames.md](lab/ur_frames.md) | The UR arm frames our URDFs must keep, the pendant's TCP at tool0, what the health `tcp` and `URDF` chips mean and how to fix them. |
| [floor_calibration_manual.md](lab/floor_calibration_manual.md) | The floor calibration with the mocap probe: when, the steps, what every chip and number means, after a Motive recalibration, troubleshooting. |

## experiments/: planning, recording and analysing experiments

| Doc | What it covers |
|---|---|
| [experiment_design_guide.md](experiments/experiment_design_guide.md), [experiment_report_guide.md](experiments/experiment_report_guide.md) | JG's Experiment Design Guide: how to plan, build and analyse an experiment, and how its generated reports are written. |
| [base_exp_manual.md](experiments/base_exp_manual.md) | Base controller experiments: goal and hypotheses, the three experiments (robot model, model over conditions, controllers), how to run them and where the results are. |
| [base_planner_manual.md](experiments/base_planner_manual.md) | Base planner experiments: goals, tasks, how attempts are recorded, collected and analysed. |

## design/: how the code is built

| Doc | What it covers |
|---|---|
| [refactor_rationale.md](design/refactor_rationale.md) | Why the runtime is built the way it is. |
| [plugin_roadmap.md](design/plugin_roadmap.md) | Which plugins exist, and which old features are still to be ported. |
| [scene_refactor_plan.md](design/scene_refactor_plan.md) | The scene: one backend-free scene, copied once per tick, mirrored into each planner. |
| [run_record.md](design/run_record.md) | What an assembly run writes, how, and how the state moves from step to step. |
| [representation_old_vs_new.md](design/representation_old_vs_new.md) | The old monitor's action, movement and state data and the new, side by side, live and on disk. |

The process file format is in the core's `doc/process_format.md` (`external/scaffolding_assembly_core/`).

## Other

- [glossary.md](glossary.md): the project's words and what each means where.
- [calibration_manual.md](calibration_manual.md): **outdated**, written for the old monitor; it says at the top what
  is missing or different now.
- `archive/`: outdated docs, kept for reference: manuals written for the old monitor, Su's notes and to-do list, the
  superseded journal plan, the RobArch 2026 paper, the old code layout diagram.
- `../tasks/`: specs written while building a feature, dated; they describe the code as it was then.
