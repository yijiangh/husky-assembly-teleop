# Simulated parts announce themselves (2026-10-19)

Status: **built, not committed**. A whole robot simulated on a laptop works (`sim_robot.launch.py robot:=0806`,
checked 19 Oct); not tried yet on a robot.

## Decisions (user, 19 Oct)

- Every simulated part of a robot announces itself on one topic per robot, `/<robot>/simulated_parts`, as a
  `crl_husky_msgs/SimulatedPart` (part, node, kind). Replaces finding the simulator by node name in the ROS graph.
- A heartbeat, not a service or a latched topic: once a second; a part counts while heard in the last 3 s
  (`robot_interface/simulated_parts.SIMULATED_FOR`). Not heard from means real, with every check.
- Parts: "base", announced by `husky_sim`; each arm by its namespace under the robot's ("ur5e", "left_ur5e"),
  announced by `multi_arm_safety_sync`.
- The safety sync works in simulation too. `crl_single_ur5e` / `crl_dual_ur5e` pass `use_fake_hardware` to it as
  `simulated`, read once at startup (never from a topic). Each arm is then a `SimulatedArmConnection` instead of the
  dashboard connection; the real path is unchanged:
  - operational without a dashboard; Stop switches off every controller but the broadcasters, as controller_stopper
    does on a real arm, and keeps them off; starting switches them back on.
  - a "simulated" arm whose dashboard services appear is a real driver: not operational, so every arm stops.
- Simulations run from the repo's config: `sim_robot.launch.py robot:=<serial>` (base and arms) and `sim_all.launch.py`
  (every robot), from `config/robots/<serial>/`. The arms read their UR calibration only from there, on the robots
  too (the fallback to `/home/administrator/config/ur/` is gone); a launch left without one stops before any node.
- Simulations refuse the robots' ROS domain (from every `robot.yaml`): the sim launches, `crl_single`/`crl_dual` with
  `use_fake_hardware:=true`, and every simulator that announces itself (`announce` raises).
- `urscript_interface` runs only on real hardware: with fake hardware it still reached for the real arm's IP.
- The monitor always needs the sync's recent, operational status. Only a missing dashboard is accepted, and only for
  an arm announced as simulated; health then shows its `SIM` chip instead of the dashboard parts.

## Built

- crl-husky (`jg/ros2`): `SimulatedPart.msg`; `crl_husky/simulated_parts.py` (`announce`); `husky_sim` announces
  "base"; the sync's `simulated` parameter, `SimulatedArmConnection` and announcements; both arm launches pass
  `use_fake_hardware`; `test/test_safety_sync_simulated.py`; README.
- teleop: `robot_interface/simulated_parts.py`; `HuskyRobotInterface.simulated`, `BaseInterface.simulation`,
  `ArmInterface.simulation`; the arm's safety check; base_exp and base_planner record sim or real from it (the sim
  model from the announced node's parameters); health's `SIM` chip and a red `NO DASHBOARD` for a real arm; tests.

## Todo

- [x] `sim_robot.launch.py robot:=0806` on an isolated domain: base and both arms announced, the sync runs both arms,
      Stop switches `scaled_joint_trajectory_controller` off on both, Resume back on.
- [ ] The monitor against `sim_robot.launch.py`: `SIM` chips, a joint move and a base path go out.
- [ ] On a robot: the arms start with the repo's calibration (CRL_HUSKY_SN set) and the sync is not simulated.
- [ ] A real arm with `multi_arm_safety_sync` stopped: motion refused with "its safety is not known".
- [ ] Rebuild crl-husky everywhere a simulator, fake arm or the sync runs; push crl-husky first.
