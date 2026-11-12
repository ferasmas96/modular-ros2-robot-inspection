# 02. Husky Simulation

### 1. Objective
Inspect how the existing Clearpath Husky A300 Gazebo simulation is configured, launched, and controlled. The inspection distinguishes file-based findings from behavior confirmed during the approved runtime check.

### 2. System Configuration
- **Inspection date:** 2026-11-12
- **Environment:** Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy; Gazebo Sim (Harmonic) 8.15.0; Clearpath Simulator 2.9.4.
- **Installed simulation package:** `ros-jazzy-clearpath-gz`; its launch files and `warehouse.sdf` are under `/opt/ros/jazzy/share/clearpath_gz/`.
- **Workspace sources:** Clearpath description and control sources are in `src/clearpath_common/`; the Clearpath configuration source is in `src/clearpath_config/`.
- **User configuration:** `/home/feras/clearpath/robot.yaml`, outside this repository. It identifies serial/model `a300-00000`, namespace `a300_00000`, ROS domain 0, `rmw_fastrtps_cpp`, PS4 teleoperation, `diff_4wd`, outdoor front/rear wheels, bumpers and top plate. Camera, GPS, IMU, 2D LiDAR and 3D LiDAR arrays are empty.
- No robot, world, launch, or source configuration was modified. The runtime launch used `generate:=false` to avoid regenerating files in `/home/feras/clearpath/`.

### 3. Implementation or Inspection Procedure
Inspected the installed Clearpath launch files and warehouse world, workspace Clearpath Xacro/control sources, user robot configuration and generated setup files, Gazebo GUI configuration, shell history, and a pre-existing launch log. Queried launch arguments with `--show-args` without starting a simulation.

The historical shell command is `ros2 launch clearpath_gz simulation.launch.py`. The top-level launch includes the Gazebo world and robot-spawn launch files. Its defaults include `world:=warehouse`, `setup_path:=$HOME/clearpath/`, `rviz:=false`, `use_sim_time:=true`, `auto_start:=true`, pose `(0, 0, 0.3)` and `generate:=true`. The latter runs generators that write into the setup path.

With the user's approval, runtime verification was performed using the existing generated setup and `generate:=false`. The simulation was then stopped. Only `ros2 node list` was used for runtime ROS inspection; no topics or TF were tested.

### 4. Results
- **Launch and architecture:** `clearpath_gz/launch/simulation.launch.py` starts Gazebo and the robot-spawn launch. Gazebo loads the installed `warehouse.sdf`; Clearpath launch generates or reuses the robot description and starts the robot's ROS nodes, bridges, and controllers. `ros_gz_sim create` spawns the robot from `robot_description`.
- **Robot description and model:** `/home/feras/clearpath/robot.urdf.xacro` is a generated top-level description using Clearpath's `clearpath_platform_description` A300 and common macros. The source A300 Xacro is `src/clearpath_common/clearpath_platform_description/urdf/a300/a300.urdf.xacro`; the generic Gazebo Xacro adds the Gazebo ROS 2 control plugin and pose publisher. The description launch sets simulation time. A generated `/home/feras/clearpath/robot.urdf` was not present when inspected.
- **World:** `warehouse` is the installed Clearpath world, not a local customization. It uses ODE, 0.003 s maximum step, 1.0 real-time factor, 1000 Hz update rate, and gravity `(0, 0, -9.8)`. It includes Physics, UserCommands, SceneBroadcaster, Sensors (Ogre2), IMU, and NavSat plugins, a ground plane, and Fuel-provided warehouse/environment models.
- **Velocity control:** The Gazebo GUI configuration enables its Teleop plugin. `gz_sim.launch.py` adds `/<namespace>/cmd_vel` to a temporary GUI configuration. The generated platform launch includes the command bridge; `teleop_base.launch.py` starts `twist_mux` and remaps its output to `platform/cmd_vel`, consumed by the platform velocity controller and Gazebo ROS 2 control. The GUI's individual W/A/S/D key mapping was not present in the inspected configuration.
- **Separate gamepad path:** The configuration also starts `joy_linux_node` and `teleop_twist_joy` for the configured PS4 controller. This is distinct from Gazebo GUI keyboard teleoperation.
- **Runtime observation:** The launch log records Gazebo starting in `warehouse`, robot state publisher initialization, successful entity creation (`a300_00000/robot`), and controller activation. The user confirmed that the GUI opened, the Husky was visible, and keyboard control worked during this run.

### 5. Verification
- **PASS** — Static launch, robot configuration, description, world, and teleoperation paths inspected.
- **PASS** — Approved launch using `generate:=false`; logs show successful robot entity creation and controller activation. The user confirmed the GUI, robot visibility, and W/A/S/D control.
- **PASS** — WSLg exposed a Gazebo Sim window; ROS 2 nodes were visible during the run.
- **NOT TESTED** — ROS topics, odometry, TF, RViz2, and sensor streams; these belong to later tasks.
- **NOT CAPTURED** — Screenshot. The available Pillow X11 capture failed with `X get_image failed: error 8 (73, 0, 1080)`. No screenshot file was saved.
- The simulation was stopped after verification. Existing generated setup file timestamps remained unchanged, and all four nested source repositories remained clean.

### 6. Problems and Solutions
- `joy_linux_node` reported that `/dev/input/ps4` could not be opened and would retry. This is an unavailable gamepad device; it did not prevent the user-confirmed GUI keyboard control.
- The controller logged that its 0.02 s update period is slower than the 0.003 s Gazebo period. No change was made; this warning is recorded for later evaluation.
- During shutdown, Gazebo did not exit within 5 seconds after SIGINT; the launch escalated to SIGTERM. Remaining Gazebo GUI/server processes were stopped by their recorded PIDs and then confirmed absent.
- Screenshot capture failed as noted above. No capture utility was installed and no graphics settings were changed.

### 7. Evidence
- [Command and configuration inspection](../evidence/command_outputs/2026-11-12-simulation-inspection.txt)
- [Prior launch log excerpt](../evidence/logs/2026-11-12-prior-husky-launch.log)
- [Approved runtime launch log excerpt](../evidence/logs/2026-11-12-sim-001-runtime.log)
- No screenshot was captured.

### 8. Reproducibility
In a terminal with the existing workspace installed, run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py generate:=false
```

This reuses the existing generated files under `$HOME/clearpath/` and uses the default `warehouse` world. The historical command omitted `generate:=false`; the override is recorded here to avoid regenerating setup files. Do not remove the override unless regeneration is intended.

### 9. Thesis Writing Notes
This provides a reproducible baseline architecture: Clearpath-generated A300 description and configuration, Gazebo Harmonic world, ROS-Gazebo bridges, a four-wheel differential controller, and distinct GUI-keyboard and gamepad command paths. The current robot configuration has no configured camera, IMU, GPS, or LiDAR arrays, so sensor integration remains a separate future stage. Cite the inspected package versions and the user-confirmed runtime result separately from static configuration findings.
