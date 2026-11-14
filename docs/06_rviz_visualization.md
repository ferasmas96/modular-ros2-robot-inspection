# 06. RViz2 Visualization

### 1. Objective
Launch the existing Clearpath RViz2 visualization alongside the Husky A300 simulation and verify the robot model, namespaced TF, filtered odometry, and response to user-operated movement. No robot, sensor, TF, EKF, or RViz configuration files were changed.

### 2. Environment Configuration
- **Inspection date:** 2026-11-14
- **Environment:** Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy; Gazebo Harmonic 8.15.0; Husky A300 namespace `a300_00000`; world `warehouse`.
- `rviz2` and Clearpath's `clearpath_viz` are installed under `/opt/ros/jazzy`.
- The existing robot description is published as `std_msgs/msg/String` on `/a300_00000/robot_description`.
- TF topics are `/a300_00000/tf` and `/a300_00000/tf_static`; the fixed frame is `odom`.
- ROS 2 and the workspace overlay were sourced only in the launch/inspection shells. No persistent environment changes were made.

### 3. RViz2 Launch Procedure
Inspected the installed Clearpath package and reused its `robot.rviz` config without editing it. The Clearpath `view_robot.launch.py` pushes the requested namespace to RViz, passes the `use_sim_time` parameter, and remaps `/tf` and `/tf_static` to namespace-relative topics. With `namespace:=a300_00000`, those resolve to the robot's namespaced TF topics.

The existing config has RobotModel and TF displays enabled and sets fixed frame `odom`. It reads the relative `robot_description` topic, which resolves to `/a300_00000/robot_description` under the pushed namespace. It does not include an Odometry display; that display was added interactively for this session and was not saved to the installed config.

The simulation was started after approval using the existing command and `generate:=false`. RViz was launched with:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_viz view_robot.launch.py \
  namespace:=a300_00000 use_sim_time:=true config:=robot.rviz
```

### 4. Visualization Results
#### Robot model
- The live publisher was `/a300_00000/robot_state_publisher`; RViz subscribed to `/a300_00000/robot_description`.
- The user confirmed that the Husky model was visible in RViz and appeared to follow the Gazebo robot during manual movement.
- No custom or generated robot description was introduced for visualization.
- No simulated sensor topics were active in prior inspection; this visualization does not verify sensor displays or sensor data.

#### TF
- The RViz TF display from `robot.rviz` is enabled with frame names, arrows, and axes; the user confirmed that frames and axes were visible.
- Fixed frame: `odom`.
- A bounded `tf2_echo odom base_link` using the same namespaced TF remappings as RViz resolved the transform. It initially warned that `odom` was not yet in its buffer, then returned repeated transforms.
- After movement, the transform was approximately `(-1.063, 2.305, 0) m`, yaw `12.373°`; frame and timestamp data were available in simulation time.

#### Odometry
The user added an RViz Odometry display and selected `/a300_00000/platform/odom/filtered`. RViz then appeared as a subscriber to the filtered topic, and the user confirmed that the display updated during motion. The existing raw topic remained available but was not added as a second RViz display.

| Sample | Raw odometry position (m) | Filtered odometry position (m) | Frame |
|---|---:|---:|---|
| Stationary baseline | approximately `(0, 0, 0)` | approximately `(0, 0, 0)` | `odom` / `base_link` |
| After user-operated movement | `(-1.06313, 2.30461, 0)` | `(-1.06313, 2.30461, 0)` | `odom` / `base_link` |

The post-movement filtered pose agreed with the `odom → base_link` transform. This is a consistency check, not an accuracy assessment; no independent ground truth was available. The user manually pressed W and released it, and reported that Gazebo and the RViz robot model, TF, and filtered odometry display moved consistently.

### 5. Verification
- **PASS** — Existing Clearpath RViz launch started with the existing configuration and displayed the robot model.
- **PASS** — Runtime graph confirmed RViz subscribed to `/a300_00000/robot_description`; robot-state publisher provided the description.
- **PASS** — The existing configuration used fixed frame `odom`; TF remappings connected RViz to namespaced transforms. The user confirmed visible TF frames and axes; `tf2_echo` resolved `odom → base_link`.
- **PASS** — RViz `use_sim_time` was `true`; it subscribed to `/clock`.
- **PASS** — User-added filtered Odometry display subscribed to `/a300_00000/platform/odom/filtered`; messages were received and the user confirmed display updates.
- **PASS** — After user-operated movement, raw and filtered odometry changed from near zero to approximately `(-1.063, 2.305, 0) m`; the resolved odom-to-base TF matched.
- **NOT TESTED** — Raw odometry as a separate RViz display, independent pose ground truth, sensor visualization, and SLAM/map displays.
- **NOT CAPTURED** — Screenshots. The WSLg/X11 desktop capture attempt failed with `X get_image failed: error 8 (73, 0, 1080)`. No screenshot file was created; graphics settings were not changed.

### 6. Problems and Limitations
- The first ROS graph query ran before startup/spawning completed and did not yet find the robot-state publisher or robot-description topic. A later settled query found both, and entity creation was logged as successful.
- Before the Odometry display was added, the filtered topic had zero subscribers. After the user confirmed adding the display with the requested topic, the topic endpoint showed one subscriber, `/a300_00000/rviz2`.
- The Clearpath `robot.rviz` file includes TF frame entries for several optional sensor links, but the current robot configuration has no active camera, LiDAR, IMU, or GNSS topics. No sensor visualization was claimed.
- During the RViz session, the EKF log reported missed update-rate deadlines, and a short `ros2 topic hz` interval later showed a long inter-message gap. The source and effect of this runtime load were not investigated or changed. The user still observed the filtered odometry display updating.
- The TF listener emitted transient “Invalid frame ID” messages while waiting for initial data; it then resolved repeated transforms with the namespace remappings.
- Screenshot capture failed as described above; no WSLg or GPU configuration changes were attempted.
- The simulation and RViz were stopped after verification.

### 7. Evidence
- [ROS graph and robot description endpoint](../evidence/command_outputs/2026-11-14-rviz-001-robot-description.txt)
- [RViz node and simulation-time settings](../evidence/command_outputs/2026-11-14-rviz-001-rviz-node.txt), [final RViz/filtered odometry interfaces](../evidence/command_outputs/2026-11-14-rviz-001-final-interfaces.txt), and [filtered topic subscription after display configuration](../evidence/command_outputs/2026-11-14-rviz-001-odom-display-endpoints-after-config.txt)
- [Stationary odometry samples](../evidence/command_outputs/2026-11-14-rviz-001-odom-before-movement.txt), [post-movement samples](../evidence/command_outputs/2026-11-14-rviz-001-odom-after-movement.txt), and [TF echo](../evidence/command_outputs/2026-11-14-rviz-001-tf-after-movement.txt)
- [TF endpoints](../evidence/command_outputs/2026-11-14-rviz-001-tf-endpoints.txt), [clock/RViz time check](../evidence/command_outputs/2026-11-14-rviz-001-clock-rviz-time.txt), and [runtime log excerpt](../evidence/logs/2026-11-14-rviz-001-runtime.log)
- [Screenshot capture limitation](../evidence/logs/2026-11-14-rviz-001-screenshot.log)

The visual appearance and movement observations were reported directly by the user; no screenshot could be saved as independent visual evidence.

### 8. Reproducibility Commands
Start the existing simulation in one sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py generate:=false
```

Start the existing Clearpath RViz config in a second sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_viz view_robot.launch.py \
  namespace:=a300_00000 use_sim_time:=true config:=robot.rviz
```

This launch remaps `/tf` and `/tf_static` into `/a300_00000/tf` and `/a300_00000/tf_static`. The installed config selects fixed frame `odom` and enables RobotModel and TF. To display filtered odometry temporarily, add an RViz Odometry display and select `/a300_00000/platform/odom/filtered`; do not save unless a reusable project configuration is separately approved.

### 9. Thesis Writing Notes
The existing Clearpath RViz package is sufficient for the base model and TF view when launched with the correct namespace and simulation-time setting. Filtered odometry can be added interactively without changing upstream config. Report the movement comparison as observed consistency, not localization accuracy. A screenshot could not be captured in this WSLg session, so retain that evidence limitation explicitly.
