# 04. Odometry and TF

### 1. Objective
Inspect representative stationary and post-movement raw/filtered odometry, the configured EKF, and the Husky A300 TF tree. No robot, EKF, or TF configuration was changed.

### 2. System Configuration
- **Inspection date:** 2026-11-12
- **Environment:** Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy; Gazebo Harmonic 8.15.0; Husky namespace `a300_00000`.
- The existing simulation was launched after user approval with `generate:=false`; movement was performed manually by the user after separate approval.
- **Generated EKF config:** `/home/feras/clearpath/platform/config/localization.yaml` (outside the repository). The generated platform launch enables the EKF and sets `use_sim_time:=true`.
- **ROS environment:** `ROS_DISTRO=jazzy`; `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` were unset in the temporary sourced shell.

### 3. Implementation or Inspection Procedure
ROS 2 Jazzy and the existing workspace overlay were sourced in temporary shells. Inspected EKF parameters/configuration, odometry topic metadata and samples, bounded `ros2 topic hz` measurements, and TF using `tf2_echo` and the already-installed `tf2_tools view_frames`.

The simulation exposes TF at `/a300_00000/tf` and `/a300_00000/tf_static`, not at root `/tf` and `/tf_static`. TF tools therefore required explicit remappings. Stationary samples were captured first. After asking for and receiving approval, the user drove forward briefly with W; raw and filtered odometry and the odom-to-base transform were sampled again. No commands were published by the inspection.

### 4. Results
#### Odometry topics and representative measurements

| Topic | Type | Publisher | Frames |
|---|---|---|---|
| `/a300_00000/platform/odom` | `nav_msgs/msg/Odometry` | `platform_velocity_controller` | `header.frame_id=odom`, `child_frame_id=base_link` |
| `/a300_00000/platform/odom/filtered` | `nav_msgs/msg/Odometry` | `ekf_node` | `header.frame_id=odom`, `child_frame_id=base_link` |

Both stationary messages contained pose and twist covariance arrays. Representative stationary values were:

- **Raw:** position approximately `(-6.42e-18, -5.18e-35, 0) m`; quaternion approximately `(0, 0, 5.86e-18, 1)`; linear `x` velocity approximately `4.03e-19 m/s`; angular `z` velocity approximately `-1.89e-20 rad/s`. Pose and twist covariance diagonal entries were approximately `(0.001, 0.001, 0.001, 0.001, 0.001, 0.01)`.
- **Filtered:** position approximately `(-6.31e-18, -5.04e-35, 0) m`; quaternion `(0, 0, 0, 1)`. Covariance arrays were present; pose diagonal entries included approximately `0.000626` for x/y and `0.00298` for yaw. These are sample values, not an accuracy assessment.

Short bounded `ros2 topic hz` windows measured raw odometry at approximately **44.5–47.4 Hz** and filtered odometry at approximately **43.7–48.0 Hz**. The EKF is configured for 50 Hz. The rate command was stopped at its 8-second timeout; these are observed short-window rates, not a long-term benchmark.

After the user-operated W movement, the raw odometry position was approximately `(3.088386, 0, 0) m`; filtered odometry was approximately `(3.088386, 0, 0) m`. The samples agreed closely. This does not establish that the filtered result is more accurate; no independent ground truth was used. Post-movement velocity samples were near zero after the key was released.

#### EKF configuration and runtime

The generated configuration is `/home/feras/clearpath/platform/config/localization.yaml`; its Clearpath launch source is `src/clearpath_common/clearpath_control/launch/localization.launch.py`.

| Setting | Configured/observed value |
|---|---|
| Input | `odom0: platform/odom`, resolving to `/a300_00000/platform/odom` |
| Active input state variables | x, y, yaw, vx, vy, and yaw rate |
| Other sensor inputs | No `imu0` configured |
| Output | `odometry/filtered`, remapped to `/a300_00000/platform/odom/filtered` |
| Frames | `odom_frame=odom`, `base_link_frame=base_link`, `world_frame=odom` |
| Filter options | `two_d_mode=true`, `publish_tf=true`, `use_sim_time=true` |
| Frequency / queue | `frequency=50.0`, `odom0_queue_size=10`, `odom0_differential=false` |

These settings were confirmed with live `ros2 param get` queries. Live topic introspection found `platform_velocity_controller` publishing the configured raw input, and `ekf_node` publishing the filtered output. The filtered topic had no subscriber at inspection time.

#### TF frames, connectivity, and timestamps

The live graph contained `/a300_00000/tf` (`tf2_msgs/msg/TFMessage`) with publishers `robot_state_publisher`, `ekf_node`, `odom_base_tf_bridge`, and `platform_velocity_controller`. `/a300_00000/tf_static` had a `robot_state_publisher` publisher and a transient-local TF listener. No sensor frames were found.

The remapped `view_frames` output showed a connected tree. Its key chain is:

```text
odom
└── base_link
    └── chassis_link
        ├── base_footprint
        ├── suspension / motor / wheel links
        ├── front_bumper_mount → front_bumper_link
        ├── rear_bumper_mount → rear_bumper_link
        └── default_mount → top_plate_link → top-plate mounts
```

`tf2_echo odom base_link` succeeded after TF data arrived. While stationary, the transform was approximately identity at the origin. After the user's movement, it reported x translation approximately `3.088 m`, consistent with the raw and filtered odometry samples. `tf2_echo base_link base_footprint` resolved a fixed z offset of approximately `0.165 m`.

The `view_frames` graph reported the dynamic `odom → base_link` transform at approximately 45.1 Hz and wheel transforms at approximately 14.9 Hz; fixed links had timestamp zero. `tf2_echo` timestamps advanced during the simulation. The post-movement raw and filtered odometry stamps were simulation times `478.086 s` and `478.380 s`; the TF listener subsequently reported `479.199 s`. Frame names and simulation-time progression were consistent, but no precise latency measurement was made.

### 5. Verification
- **PASS** — Raw and filtered odometry messages were received while stationary and after user-approved keyboard movement; both used `odom` / `base_link`.
- **PASS** — Raw and filtered positions changed from near zero to approximately 3.088 m after movement and agreed closely.
- **PASS** — EKF configuration and active runtime parameters were inspected; the raw odometry publisher and filtered output publisher were present.
- **PASS** — With explicit topic remappings, `tf2_echo` resolved `odom → base_link` and `base_link → base_footprint`; `view_frames` produced a connected tree.
- **PASS** — Dynamic TF timestamps advanced; the tree showed fixed and dynamic frame relationships.
- **NOT TESTED** — Odometry accuracy against ground truth, sensor-frame transforms, sensor data, SLAM, or navigation.
- **NOT VERIFIED BY DIRECT ECHO** — A bounded `ros2 topic echo --once` on `/a300_00000/tf_static` timed out even with transient-local QoS. The TF buffer nevertheless resolved fixed transforms and `view_frames` included fixed links.

### 6. Problems and Solutions
- The first `tf2_echo` and `view_frames` attempts listened on root `/tf`; the simulation publishes namespaced TF. They reported missing frames/empty graph. Explicitly remapping `/tf` and `/tf_static` to `/a300_00000/tf` and `/a300_00000/tf_static` resolved the frame queries; no configuration change was made.
- The direct bounded `/tf_static` echo did not return a message, despite the live transient-local publisher/subscriber and fixed transforms received by the TF tools. The reason for this CLI echo discrepancy was not established.
- `tf2_echo` briefly reported an invalid frame while its buffer was waiting for initial data; it then returned repeated transforms. No persistent connectivity error was observed with the correct remappings.
- Frequency commands ended at their bounded timeout by design. During shutdown Gazebo again required SIGTERM escalation after the SIGINT timeout; the remaining processes were stopped by their recorded PIDs.

### 7. Evidence
- [Stationary raw and filtered samples](../evidence/command_outputs/2026-11-12-tf-001-odom-samples.txt), [post-movement raw sample](../evidence/command_outputs/2026-11-12-tf-001-odom-after-motion.txt), and [post-movement filtered sample](../evidence/command_outputs/2026-11-12-tf-001-filtered-after-motion.txt)
- [Odometry rates](../evidence/command_outputs/2026-11-12-tf-001-odom-frequency.txt), [live parameters](../evidence/command_outputs/2026-11-12-tf-001-ekf-params.txt), and [odometry/TF endpoint QoS](../evidence/command_outputs/2026-11-12-tf-001-odometry-and-tf-info.txt)
- [Remapped TF frame graph output](../evidence/command_outputs/2026-11-12-tf-001-view-frames-remapped.txt), [TF diagram](../evidence/screenshots/2026-11-12-tf-001-frame-tree.pdf), and [Graphviz source](../evidence/command_outputs/2026-11-12-tf-001-frame-tree.gv)
- [Odometry-to-base transform samples](../evidence/command_outputs/2026-11-12-tf-001-tf2-echo-remapped.txt) and [base-to-footprint sample](../evidence/command_outputs/2026-11-12-tf-001-tf2-base-footprint.txt)
- [TF topic samples](../evidence/command_outputs/2026-11-12-tf-001-tf-topic-samples.txt), [static-topic echo result](../evidence/command_outputs/2026-11-12-tf-001-tf-static-echo.txt), [runtime launch excerpt](../evidence/logs/2026-11-12-tf-001-runtime.log), and [inspection notes](../evidence/logs/2026-11-12-tf-001-inspection-notes.log)

### 8. Reproducibility
Start the existing simulation as previously verified:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py generate:=false
```

Inspect odometry with bounded commands:

```bash
timeout 8s ros2 topic echo --once /a300_00000/platform/odom
timeout 8s ros2 topic echo --once /a300_00000/platform/odom/filtered
timeout 8s ros2 topic hz /a300_00000/platform/odom
```

For this namespaced simulation, use explicit TF remappings:

```bash
timeout 8s ros2 run tf2_ros tf2_echo odom base_link \
  --ros-args -r /tf:=/a300_00000/tf -r /tf_static:=/a300_00000/tf_static
ros2 run tf2_tools view_frames \
  --ros-args -r /tf:=/a300_00000/tf -r /tf_static:=/a300_00000/tf_static
```

To check motion-dependent changes, first obtain stationary samples, ask the operator to drive manually, then sample again. Do not publish commands from the terminal.

### 9. Thesis Writing Notes
The live results establish that the configured single-input 2D EKF receives wheel odometry, publishes filtered odometry, and contributes to a connected `odom`-rooted TF tree. Raw and filtered outputs tracked the same manually observed displacement in this run; that is a consistency check, not an accuracy evaluation. Record the namespaced TF topics and remapping requirement in future launch/RViz instructions. Sensor frames and independent localization ground truth remain unavailable.
