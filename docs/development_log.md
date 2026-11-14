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

## 2026-11-12 — TF-001 — Odometry and TF Inspection

- **Date:** 2026-11-12
- **Task ID:** TF-001
- **Objective:** Inspect raw and EKF-filtered odometry and validate the available namespaced TF tree.
- **Actions performed:** Inspected the generated EKF configuration and active parameters; after user approval, launched the existing simulation with `generate:=false`. Captured stationary raw/filtered odometry and bounded frequency measurements, inspected the actual publisher/QoS endpoints, generated a TF diagram with installed `tf2_tools`, and used explicitly remapped `tf2_echo` queries. After separate user approval, the user manually drove forward; post-movement odometry and TF samples were captured. Stopped the simulation and cleaned up remaining Gazebo processes.
- **Files modified:** Updated `docs/04_odometry_tf.md` and this log. Created date-prefixed odometry, EKF, TF, and runtime evidence; saved the TF diagram and Graphviz source under `evidence/`.
- **Verification results:** PASS — raw and filtered odometry changed from near-zero stationary position to approximately 3.088 m after manual movement and agreed closely. Their frames matched the connected `odom → base_link → chassis_link → base_footprint` tree. TF timestamps advanced; measured odometry rates were approximately 44–48 Hz against a configured 50 Hz.
- **Problems encountered:** Root-namespace TF tools initially saw an empty graph because TF is published under `/a300_00000`; explicit remappings resolved it. Direct bounded echo of `/a300_00000/tf_static` timed out, although tf2 tools resolved fixed transforms. Gazebo required SIGTERM escalation on shutdown. No configuration or source was changed.
- **Outstanding tasks:** TF and odometry accuracy against independent ground truth remain untested. Sensor-frame validation remains blocked by the absence of configured sensor topics; proceed only with the user's next task.

## 2026-11-12 — GZ-001 — ROS-Gazebo Integration Inspection

- **Date:** 2026-11-12
- **Task ID:** GZ-001
- **Objective:** Inspect and verify existing Gazebo Harmonic / ROS 2 integration, Gazebo Transport topics, controller integration, command routing, and simulation clock.
- **Actions performed:** Inspected installed Clearpath Gazebo launch files, generated platform launch/configuration, Clearpath ROS 2 control launch files, and the simulated A300 Gazebo control Xacro. After user approval, launched the existing simulation with `generate:=false`; inspected ROS and Gazebo topic names, types, endpoints, bridge startup directions, and selected `use_sim_time` parameters. After user approval, the user manually operated the keyboard; captured matching command samples on the Gazebo GUI topic, ROS command topics, and the bridge's Gazebo output. Stopped the simulation and verified its processes had exited.
- **Files modified:** Updated `docs/05_ros_gz_integration.md` and this log. Added date-prefixed ROS/Gazebo command outputs and a concise runtime log excerpt under `evidence/`.
- **Verification results:** PASS — Gazebo `/clock` bridged to ROS `/clock`; selected ROS nodes used simulation time. PASS — the user's forward keyboard command was observed on Gazebo `/a300_00000/cmd_vel`, ROS `/a300_00000/cmd_vel`, and `/a300_00000/platform/cmd_vel`; the platform velocity controller subscribed to the latter. The ROS→Gazebo bridge emitted a matching message, but its Gazebo topic had no subscriber. NOT VERIFIED — the configured pose-to-TF Gazebo topic had no publisher.
- **Problems encountered:** The Gazebo CLI was initially invoked before sourcing ROS and was not on `PATH`; sourcing `/opt/ros/jazzy/setup.bash` resolved it. Gazebo reported no publisher for the configured `/model/a300_00000/robot/tf` bridge input, despite publishing `/model/a300_00000/robot/pose`; the ROS→Gazebo command output likewise had no Gazebo subscriber. A bounded ROS clock echo printed message-loss notices before receiving a sample. Gazebo required cleanup by verified PIDs after the launcher's SIGINT timeout. No source, launch, bridge, controller, or robot configuration was changed.
- **Outstanding tasks:** Clarify whether the unmatched pose-to-TF and ROS-to-Gazebo command topics are intended/required before modifying any integration. Sensor bridges and later thesis tasks were not started.

## 2026-11-14 — RVIZ-001 — RViz2 Visualization Verification

- **Date:** 2026-11-14
- **Task ID:** RVIZ-001
- **Objective:** Verify RViz2 visualization of the Husky A300 model, namespaced TF, and filtered odometry, including user-operated movement.
- **Actions performed:** Inspected installed RViz2 and Clearpath `clearpath_viz` launch/configuration files before runtime. After approval, launched the existing simulation with `generate:=false` and launched `view_robot.launch.py` using `robot.rviz`, namespace `a300_00000`, and simulation time. Verified runtime robot-description, TF, clock, and odometry interfaces. The user confirmed the model and TF display were visible, added a temporary Odometry display for filtered odometry, and manually drove/released W. Captured stationary and post-movement raw/filtered odometry and a remapped odom-to-base TF sample. Attempted a WSLg screenshot and stopped the simulation and RViz afterward.
- **Files modified:** Updated `docs/06_rviz_visualization.md` and this log. Added dated ROS command outputs and a concise runtime log excerpt under `evidence/`. No RViz config, launch, robot, sensor, TF, or EKF configuration was changed.
- **Verification results:** PASS — user reported model and TF frames/axes visible; RViz subscribed to `/a300_00000/robot_description` and namespaced TF was resolved with the Clearpath launch remappings. PASS — filtered odometry display was subscribed by RViz and updated during movement. Stationary odometry was near zero; after movement raw/filtered position was approximately `(-1.063, 2.305, 0) m`, matching the resolved `odom → base_link` transform. User reported visual movement consistent between Gazebo and RViz.
- **Problems encountered:** The first graph query preceded robot spawn; the settled graph was correct. RViz's default config does not include an Odometry display, which the user added temporarily. A short filtered odometry rate sample showed a long gap and the EKF logged missed update-rate deadlines; not investigated or modified. `ImageGrab` failed with `X get_image failed: error 8 (73, 0, 1080)`, so no screenshot was saved. No WSLg/GPU settings were changed.
- **Outstanding tasks:** Raw odometry was not added as a separate RViz display; sensor visualization and ground-truth accuracy remain untested. Do not start Task 8 without the user's next request.
