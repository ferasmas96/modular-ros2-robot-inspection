# 05. ROS 2–Gazebo Integration

### 1. Objective
Inspect and validate the existing communication paths between Gazebo Harmonic and ROS 2 Jazzy, including `ros_gz_bridge`, Gazebo Transport topics, the ROS 2 control integration, and simulation time. No bridges, launch files, controller parameters, or robot configurations were changed.

### 2. System Configuration
- **Inspection date:** 2026-11-12
- **Environment:** Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy; Gazebo Harmonic 8.15.0; Husky A300 namespace `a300_00000`; world `warehouse`.
- **Launch:** The existing simulation was launched after user approval using `ros2 launch clearpath_gz simulation.launch.py generate:=false`. The user manually operated W/A/S/D for the command-flow sample.
- The Gazebo CLI is available after sourcing `/opt/ros/jazzy/setup.bash`; in an unsourced shell `gz` was not found. No persistent environment changes were made.

### 3. Existing Integration Components

#### Launch and bridge configuration

| File | Role |
|---|---|
| `/opt/ros/jazzy/share/clearpath_gz/launch/simulation.launch.py` | Includes Gazebo startup and robot-spawn launch files; defaults to the `warehouse` world. |
| `/opt/ros/jazzy/share/clearpath_gz/launch/gz_sim.launch.py` | Starts Gazebo, configures resource paths and GUI teleoperation topic, and starts the `/clock` bridge. |
| `/opt/ros/jazzy/share/clearpath_gz/launch/robot_spawn.launch.py` | Starts platform/sensor services and uses `ros_gz_sim create` to spawn the robot from `robot_description`. |
| `/home/feras/clearpath/platform/launch/platform-service.launch.py` | Generated per-robot launch file. Starts the platform stack, command bridge, and pose-to-TF bridge. This file is outside the workspace. |
| `../src/clearpath_common/clearpath_control/launch/platform.launch.py` | Includes description, control, EKF, and teleoperation launches under the robot namespace. |
| `../src/clearpath_common/clearpath_control/launch/control.launch.py` | Loads controller configuration and spawns the joint-state broadcaster and velocity controller. |
| `../src/clearpath_common/clearpath_control/launch/teleop_base.launch.py` | Starts `twist_mux`, which routes selected ROS velocity inputs to `platform/cmd_vel`. |
| `../src/clearpath_common/clearpath_platform_description/urdf/generic/gazebo.urdf.xacro` | Adds the Gazebo Sim ROS 2 control plugin and its ROS topic remappings to the simulated robot description. |
| `/home/feras/clearpath/platform/config/control.yaml` | Generated controller parameters, including `DiffDriveController` and simulation-time settings. This file is outside the workspace. |

#### `ros_gz_bridge` mappings

| Gazebo Transport topic and type | ROS 2 topic and type | Direction / process | Runtime result |
|---|---|---|---|
| `/clock` — `gz.msgs.Clock` | `/clock` — `rosgraph_msgs/msg/Clock` | Gazebo → ROS; `/clock_bridge` | **PASS.** Bridge startup log, Gazebo publisher/subscriber information, and a ROS clock message were observed. |
| `/a300_00000/cmd_vel` — `gz.msgs.Twist` | `/a300_00000/cmd_vel` — `geometry_msgs/msg/TwistStamped` | Gazebo → ROS; `/a300_00000/cmd_vel_bridge` | **PASS.** Gazebo and ROS samples both showed the user's forward command (`linear.x=1.0`). |
| `/model/a300_00000/robot/cmd_vel` — `gz.msgs.Twist` | `/a300_00000/platform/cmd_vel` — `geometry_msgs/msg/TwistStamped` | ROS → Gazebo; `/a300_00000/cmd_vel_bridge` | The bridge published a Gazebo message matching the ROS command, but Gazebo reported **no subscriber** on this transport topic. The ROS topic is separately subscribed to by `platform_velocity_controller`. |
| `/model/a300_00000/robot/tf` — `gz.msgs.Pose_V` | `/a300_00000/tf` — `tf2_msgs/msg/TFMessage` | Configured Gazebo → ROS; `/a300_00000/odom_base_tf_bridge` | **NOT VERIFIED.** The bridge subscription was present, but Gazebo reported no publisher on its configured input topic. |

The bridge directions above were confirmed from the runtime bridge startup logs, topic endpoint inspection, and command samples. The ROS remappings in the generated launch file account for the final namespaced ROS topic names.

### 4. Gazebo Transport and Controller Results
The observed and configured data paths can be summarized as:

```text
Gazebo GUI Teleop
  -> Gazebo /a300_00000/cmd_vel (gz.msgs.Twist)
  -> cmd_vel_bridge (Gazebo -> ROS)
  -> ROS /a300_00000/cmd_vel
  -> twist_mux
  -> ROS /a300_00000/platform/cmd_vel
  -> platform_velocity_controller (ROS 2 control)
  -> gz_ros2_control plugin / simulated robot
  -> ROS /a300_00000/platform/odom

ROS /a300_00000/platform/cmd_vel
  -> cmd_vel_bridge (ROS -> Gazebo)
  -> Gazebo /model/a300_00000/robot/cmd_vel [no Gazebo subscriber observed]

Gazebo /clock (gz.msgs.Clock)
  -> clock_bridge (Gazebo -> ROS)
  -> ROS /clock
  -> selected ROS nodes with use_sim_time=true

Gazebo /model/a300_00000/robot/pose (gz.msgs.Pose_V) [publisher observed]
Configured /model/a300_00000/robot/tf (gz.msgs.Pose_V)
  -> odom_base_tf_bridge (Gazebo -> ROS /a300_00000/tf)
  [no Gazebo publisher observed on the configured input]
```

The live `gz topic -l` output included:

| Gazebo topic | Type / endpoint observation | Purpose and result |
|---|---|---|
| `/clock` | `gz.msgs.Clock`; publisher and subscriber present | Source for the ROS `/clock` bridge. |
| `/world/warehouse/clock` | `gz.msgs.Clock`; publisher, no subscribers | World-scoped clock exists but is not the topic configured for the ROS bridge. |
| `/a300_00000/cmd_vel` | `gz.msgs.Twist`; publisher and subscriber present | Gazebo GUI teleoperation output and input to the GZ→ROS command bridge. |
| `/model/a300_00000/robot/cmd_vel` | `gz.msgs.Twist`; publisher, no subscribers | ROS→GZ bridge output; no Gazebo consumer was discovered. |
| `/model/a300_00000/robot/pose` | `gz.msgs.Pose_V`; publisher, no subscribers | Gazebo robot/link pose output was observed. It is not the input topic named in the pose-to-TF bridge configuration. |
| `/model/a300_00000/robot/tf` | `gz.msgs.Pose_V`; no publisher, bridge subscriber present | Configured pose-to-TF bridge source was not publishing during inspection. |
| `/world/warehouse/stats` | `gz.msgs.WorldStatistics`; publisher and subscribers present | World statistics; a sample reported simulation time, iteration count, and real-time factor. |

#### Controller integration (not a `ros_gz_bridge` topic mapping)
The A300 Xacro selects `gz_ros2_control/GazeboSimSystem` when simulation is enabled. The generic Gazebo Xacro loads `libgz_ros2_control-system.so` and remaps controller interfaces, including `platform/cmd_vel`, `platform/odom`, joint states, and TF. At runtime, `/a300_00000/gz_ros_control`, `/a300_00000/controller_manager`, and `/a300_00000/platform_velocity_controller` were present. The live ROS graph showed `twist_mux` publishing `/a300_00000/platform/cmd_vel` and `platform_velocity_controller` subscribing to it. This controller path is integrated through the Gazebo ROS 2 control plugin rather than requiring the outgoing `ros_gz_bridge` topic to drive the controller.

The user's W-key sample appeared as forward `linear.x=1.0` on the Gazebo GUI topic, ROS `/a300_00000/cmd_vel`, ROS `/a300_00000/platform/cmd_vel`, and the bridge's Gazebo `/model/a300_00000/robot/cmd_vel` output. This verifies the command message path and the controller subscriber connection; no command was published by the inspection.

### 5. Simulation Clock
- Gazebo published `gz.msgs.Clock` on `/clock`; `gz topic -i` showed both a publisher and the bridge subscriber.
- The installed Clearpath launch starts `ros_gz_bridge/parameter_bridge` with `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock`, which is Gazebo → ROS. Its startup log explicitly reported the `/clock` bridge.
- ROS `/clock` had one publisher, `/clock_bridge`, and 11 subscribers in the live graph. A bounded `ros2 topic echo --once /clock` received a message.
- Runtime `use_sim_time` was `True` on the checked EKF, robot-state publisher, controller manager, platform velocity controller, twist mux, and both per-robot bridge nodes. `/clock_bridge` reported `use_sim_time=False`; it is the clock publisher and was not a tested clock consumer.
- `/world/warehouse/clock` was separately present in Gazebo but had no subscribers; it is not the configured bridge source.

### 6. Verification
- **PASS** — Existing launch completed, entity creation succeeded, and controller activation was logged.
- **PASS** — ROS and Gazebo topic discovery, message types, bridge directions, and endpoint availability were inspected.
- **PASS** — Gazebo clock data and ROS `/clock` messages were received; selected ROS consumer nodes used simulation time.
- **PASS** — During user-operated keyboard control, the same forward command was observed on Gazebo's GUI command topic and both corresponding ROS command topics. The ROS velocity controller was a subscriber to the mux output.
- **PASS, LIMITED** — The ROS→Gazebo command bridge published a matching message on its configured Gazebo topic; that topic had no Gazebo subscriber, so its downstream use was not verified.
- **NOT VERIFIED** — The configured Gazebo pose-to-TF input had no Gazebo publisher. The bridge's effective output data was therefore not demonstrated in this check.
- **NOT TESTED** — Sensor bridges/data, autonomous control, and any change or repair to the unmatched interfaces.

### 7. Problems and Limitations
- The configured pose-to-TF bridge listens on `/model/a300_00000/robot/tf`, where Gazebo reported no publisher. Gazebo did publish `Pose_V` data on `/model/a300_00000/robot/pose`. This is a runtime topic discrepancy; its intended resolution was not investigated and no files were changed.
- The ROS→Gazebo command bridge emitted messages on `/model/a300_00000/robot/cmd_vel`, but Gazebo reported no subscriber. The ROS 2 controller's direct subscription through `gz_ros2_control` was present; the purpose or expected consumer of the additional Gazebo topic was not determined.
- One bounded ROS clock echo printed CLI “message was lost” notices before it received a message. The bridge and both clock endpoints were present; this inspection did not establish the cause of the CLI notices.
- An initial `gz topic` attempt was made before sourcing ROS and failed with `gz: command not found`. Sourcing `/opt/ros/jazzy/setup.bash` exposed the installed CLI, and subsequent topic inspection succeeded. No installation or environment change was made.
- The joystick device `/dev/input/ps4` remained unavailable. The Gazebo controller also warned that its desired 0.02 s update period was slower than the 0.003 s Gazebo simulation period. Neither configuration was changed.
- Gazebo did not exit within the launcher's SIGINT timeout; the launch shut down its ROS processes and remaining Gazebo processes were stopped by their verified PIDs.

### 8. Evidence
- [Gazebo transport topic list](../evidence/command_outputs/2026-11-12-gz-001-gz-topics.txt), [topic types and endpoints](../evidence/command_outputs/2026-11-12-gz-001-gz-topic-info.txt), and [bounded Gazebo samples](../evidence/command_outputs/2026-11-12-gz-001-gz-samples.txt)
- [ROS graph and bridge topic endpoints](../evidence/command_outputs/2026-11-12-gz-001-ros-graph.txt), [simulation-time parameters](../evidence/command_outputs/2026-11-12-gz-001-use-sim-time.txt), and [additional time parameters](../evidence/command_outputs/2026-11-12-gz-001-additional-time-params.txt)
- Manual command samples: [Gazebo GUI topic](../evidence/command_outputs/2026-11-12-gz-001-gz-keyboard-command.txt), [ROS keyboard topic](../evidence/command_outputs/2026-11-12-gz-001-ros-keyboard-command.txt), [ROS controller topic](../evidence/command_outputs/2026-11-12-gz-001-ros-controller-command.txt), and [Gazebo bridge output](../evidence/command_outputs/2026-11-12-gz-001-gz-command-output.txt)
- [ROS clock and odometry samples](../evidence/command_outputs/2026-11-12-gz-001-ros-clock-odom.txt), [pose/TF endpoint status](../evidence/command_outputs/2026-11-12-gz-001-pose-tf-topic-status.txt), and [runtime bridge/controller log excerpt](../evidence/logs/2026-11-12-gz-001-runtime.log)

### 9. Reproducibility
Use the existing generated robot configuration and the previously verified launch command:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py generate:=false
```

In another sourced shell, list and inspect Gazebo topics and the ROS graph:

```bash
source /opt/ros/jazzy/setup.bash
gz topic -l
gz topic -i -t /clock
gz topic -i -t /a300_00000/cmd_vel

source /home/feras/clearpath_ws/install/setup.bash
ros2 node list
ros2 topic info /clock --verbose
ros2 topic info /a300_00000/platform/cmd_vel --verbose
timeout 8s ros2 topic echo /clock --once
```

To observe keyboard flow, start bounded `gz topic -e` and `ros2 topic echo --once` subscriptions on the already-discovered topics, then operate the Gazebo GUI manually. Do not publish a velocity command from the terminal.

### 10. Thesis Writing Notes
Describe two distinct command mechanisms: the Gazebo GUI keyboard topic is bridged into ROS and selected by `twist_mux`; the mux output is received directly by the ROS 2 diff-drive controller hosted through `gz_ros2_control`. The additional ROS→Gazebo bridge topic exists and published in this run, but had no Gazebo subscriber. Likewise, the configured pose-to-TF bridge source did not have a Gazebo publisher, while a separate Gazebo robot pose topic did. Keep these observed limitations explicit; do not present either unmatched bridge as a verified data path. Sensor integration and subsequent tasks were not started.
