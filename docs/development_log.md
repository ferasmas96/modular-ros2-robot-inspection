# Development Log

## 2026-11-12 — DOC-001

- **Date:** 2026-11-12
- **Task ID:** DOC-001
- **Objective:** Create the thesis project documentation structure and initialize empty documentation templates.
- **Actions performed:** Created the requested documentation, evidence, configuration, and experiment directories; added the documentation guide, topic templates, and decision-log template.
- **Files created or modified:** Created `docs/README.md`, `docs/01_environment.md` through `docs/11_evaluation.md`, `docs/decisions.md`, and this log. Created `evidence/screenshots/`, `evidence/logs/`, `evidence/command_outputs/`, `configs/`, and `experiments/`. No existing files were modified.
- **Verification status:** PASS — all requested paths exist, and all 11 numbered documents contain the required sections and unfilled-section markers.
- **Problems encountered:** The selected workspace is not itself a Git repository; its four nested source repositories were clean when checked. The project root was confirmed with the user before creating files.
- **Next planned task:** Not documented yet

## 2026-11-12 — ENV-001

- **Date:** 2026-11-12
- **Task ID:** ENV-001
- **Objective:** Inspect and document the existing WSL2 robotics development environment as a verified baseline.
- **Actions performed:** Inspected Ubuntu/WSL, CPU/RAM/disk, ROS 2 and its temporary sourced workspace overlay, Gazebo version, installed Clearpath and ROS–Gazebo packages, development tools, NVIDIA/WSLg devices and prior graphics-log excerpts, APT repositories, project Git status, and the four nested repository remotes/commits. No simulation or GUI application was started; no software, source repository, or persistent configuration was changed.
- **Files created or modified:** Modified `docs/01_environment.md` and this log. Created `evidence/command_outputs/2026-11-12-system-wsl.txt`, `evidence/command_outputs/2026-11-12-ros-gazebo.txt`, `evidence/command_outputs/2026-11-12-tools-gpu-apt.txt`, and `evidence/command_outputs/2026-11-12-clearpath-repositories.txt`.
- **Verification results:** PASS — ROS CLI and package list worked in a temporary Jazzy/overlay shell; `gz sim --version` reported 8.15.0; GPU, WSLg, tool versions, APT sources, and sanitized Git metadata were captured. `git diff --check` passed; final status contains only the two requested documentation modifications and four new evidence outputs. No commit or push was made. NOT TESTED — simulation behavior, ROS runtime interfaces, RViz launch, and hardware-backed OpenGL.
- **Problems encountered:** ROS was not sourced in the initial inspection shell; temporary sourcing exposed it without persistent changes. The first nested-remote parser assumed tab-separated output and stopped; a whitespace-safe read-only check then recorded all four repositories. `gh` and `glxinfo` were unavailable.
- **Outstanding tasks:** Independently verify the Husky simulation and ROS interfaces in a later task; directly verify which GPU renderer is used if needed.

## 2026-11-12 — SIM-001 — Husky A300 Simulation Inspection

- **Date:** 2026-11-12
- **Task ID:** SIM-001
- **Objective:** Inspect and document the existing Husky A300 Gazebo simulation configuration, launch procedure, world, robot description, and teleoperation paths.
- **Actions performed:** Inspected installed `clearpath_gz` launch/world files, workspace Clearpath description and control sources, `/home/feras/clearpath/robot.yaml` and generated setup files, launch arguments, shell history, and a prior launch log. With user approval, launched the existing simulation with `generate:=false`, confirmed startup/spawn/controller evidence in logs, checked the active ROS nodes, and received user confirmation that the GUI, robot, and W/A/S/D control worked. Stopped the simulation after verification. No source, launch, world, or robot configuration was changed.
- **Files created or modified:** Modified `docs/02_husky_simulation.md` and this log. Created `evidence/command_outputs/2026-11-12-simulation-inspection.txt`, `evidence/logs/2026-11-12-prior-husky-launch.log`, and `evidence/logs/2026-11-12-sim-001-runtime.log`. No screenshot was saved.
- **Verification results:** PASS — inspected launch/config/world, successful runtime world startup, robot entity creation, controller activation, visible Gazebo window, and user-confirmed keyboard control. NOT TESTED — topics, odometry, TF, RViz2, and sensors. All four nested source repositories remained clean and generated setup-file timestamps were unchanged.
- **Problems encountered:** The PS4 joystick device `/dev/input/ps4` was unavailable; this was separate from the working GUI keyboard path. Screenshot capture failed with an X11 `XGetImage` error. The controller reported a 0.02 s update period versus the 0.003 s Gazebo period. Gazebo also required SIGTERM escalation after not exiting within 5 seconds of SIGINT; remaining processes were stopped by their recorded PIDs. No configuration was changed.
- **Outstanding tasks:** Task 4 — inspect ROS 2 communication, velocity/odometry interfaces, and TF; RViz2 and sensor integration remain untested.

## 2026-11-12 — ROS-001 — ROS 2 Communication Inspection

- **Date:** 2026-11-12
- **Task ID:** ROS-001
- **Objective:** Inspect the running Husky A300 simulation's ROS 2 nodes, topics, endpoint relationships, QoS, and basic message flow.
- **Actions performed:** Verified Jazzy and the workspace overlay in temporary sourced shells; recorded unset `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`. After user approval, launched the existing simulation with `generate:=false`, discovered nodes/topics, inspected key node and topic endpoint information, received bounded one-message samples for clock, joint states, odometry, and diagnostics, and—at the user's request—observed a keyboard-generated velocity message. Stopped the simulation and cleaned up its remaining Gazebo processes.
- **Files updated:** Modified `docs/03_ros2_communication.md` and this log. Created date-prefixed node/topic lists, node/topic details, one-message command outputs, and a concise runtime log excerpt under `evidence/`.
- **Verification results:** PASS — 14 ROS 2 nodes and 34 topics discovered; basic clock, joint-state, odometry, diagnostics, and keyboard velocity communication observed. NOT TESTED — TF correctness, sensor streams/accuracy, SLAM, navigation, and gamepad data.
- **Problems encountered:** The PS4 device `/dev/input/ps4` was unavailable. Several configured mux inputs had no publisher in the runtime graph. Gazebo required SIGTERM escalation after the SIGINT shutdown timeout; all remaining processes were stopped by their recorded PIDs.
- **Remaining tasks:** Task 5 — inspect TF transformations; Task 8 — inspect sensor integration. Investigate the currently unconnected command/safety interfaces if required by later system design.
