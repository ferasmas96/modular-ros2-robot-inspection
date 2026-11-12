# 03. ROS 2 Communication

### 1. Objective
Inspect the runtime ROS 2 node and topic graph for the existing Husky A300 simulation and verify basic message flow without testing TF transforms, sensor performance, or SLAM.

### 2. System Configuration
- **Inspection date:** 2026-11-12
- **Environment:** Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy; Gazebo Harmonic 8.15.0; Clearpath Simulator 2.9.4.
- **Workspace:** `/home/feras/clearpath_ws`; robot namespace: `a300_00000`.
- **ROS environment:** `ROS_DISTRO=jazzy`; `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` were unset in the sourced shell. ROS 2 executable: `/opt/ros/jazzy/bin/ros2`.
- ROS and the existing workspace overlay were sourced only in temporary inspection shells. No persistent environment or simulation configuration was changed.

### 3. Implementation or Inspection Procedure
The simulation was not running at inspection start. After user approval, it was launched with the previously verified command and `generate:=false`; it was stopped after inspection. Commands used included:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 node info <discovered_node>
ros2 topic info <discovered_topic> --verbose
timeout 8s ros2 topic echo <discovered_topic> --once
```

Node and topic endpoint information was collected from the live ROS graph. Bounded one-message subscriptions checked `/clock`, joint states, odometry, and diagnostics. With the user's participation, a one-message subscription also observed keyboard teleoperation on `/a300_00000/cmd_vel`. No velocity commands were published by the inspection.

### 4. Results
#### Discovered nodes

`ros2 node list` returned **14 nodes**:

| Runtime node | Observed role |
|---|---|
| `/a300_00000/cmd_vel_bridge` | ROS-Gazebo velocity bridge; publishes `/a300_00000/cmd_vel` and subscribes to `/a300_00000/platform/cmd_vel`. |
| `/a300_00000/controller_manager` | ROS 2 controller manager; subscribes to `robot_description` and manages the controllers. |
| `/a300_00000/ekf_node` | State-estimation node; subscribes to raw odometry and publishes filtered odometry and diagnostics. |
| `/a300_00000/gz_ros_control` | Gazebo ROS 2 control integration node. |
| `/a300_00000/joint_state_broadcaster` | Publishes joint-state topics. |
| `/a300_00000/joy_node` | Gamepad input node; its configured PS4 device was unavailable during this run. |
| `/a300_00000/odom_base_tf_bridge` | Publishes on `/tf`; transform values were not inspected. |
| `/a300_00000/platform_velocity_controller` | Subscribes to platform velocity commands and publishes odometry. |
| `/a300_00000/robot_state_publisher` | Publishes robot description and subscribes to joint states. |
| `/a300_00000/teleop_twist_joy_node` | Converts gamepad input to velocity messages. |
| `/a300_00000/transform_listener_impl_59f17541eeb0` | Runtime transform-listener helper; no transform data was inspected. The suffix is generated at runtime. |
| `/a300_00000/twist_mux` | Subscribes to velocity and stop inputs and publishes the selected platform command. |
| `/a300_00000/twist_server_node` | Interactive-marker command source. |
| `/clock_bridge` | Publishes the simulation clock to ROS 2. |

#### Communication inventory

Endpoint names and counts below are from live `ros2 topic info --verbose` / node introspection. “No subscriber” or “no publisher” describes the discovered graph at inspection time, not necessarily a fault.

| Class | Topic | Message type | Publisher → subscriber(s) | Status |
|---|---|---|---|---|
| Control | `/a300_00000/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `cmd_vel_bridge` → `twist_mux` | **PASS** — one message observed during user-operated keyboard control. |
| Control | `/a300_00000/platform/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `twist_mux` → `cmd_vel_bridge`, `platform_velocity_controller` | **PASS** — both subscribers and the publisher discovered. |
| Control | `/a300_00000/joy_teleop/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `teleop_twist_joy_node` → `twist_mux` | Graph endpoints discovered; gamepad input not verified. |
| Control | `/a300_00000/twist_marker_server/cmd_vel` | `geometry_msgs/msg/TwistStamped` | `twist_server_node` → `twist_mux` | Graph endpoints discovered; no command test performed. |
| Control | `/a300_00000/rc_teleop/cmd_vel` | `geometry_msgs/msg/TwistStamped` | No publisher → `twist_mux` | **NOT TESTED** — subscription exists, no publisher was discovered. |
| Control | `/a300_00000/platform/emergency_stop` | `std_msgs/msg/Bool` | No publisher → `twist_mux` | No publisher discovered. |
| Control | `/a300_00000/platform/safety_stop` | `std_msgs/msg/Bool` | No publisher → `twist_mux` | No publisher discovered. |
| Odometry | `/a300_00000/platform/odom` | `nav_msgs/msg/Odometry` | `platform_velocity_controller` → `ekf_node` | **PASS** — one message received. |
| Odometry | `/a300_00000/platform/odom/filtered` | `nav_msgs/msg/Odometry` | `ekf_node` → no subscriber | Publisher discovered; no sample separately echoed. |
| Robot state | `/a300_00000/platform/joint_states` | `sensor_msgs/msg/JointState` | `joint_state_broadcaster` → `robot_state_publisher` | **PASS** — one message received. |
| Robot state | `/a300_00000/dynamic_joint_states` | `control_msgs/msg/DynamicJointState` | `joint_state_broadcaster` → no subscriber | Publisher discovered; no sample separately echoed. |
| Robot status | `/a300_00000/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `joy_node`, `controller_manager`, `ekf_node`, `twist_mux` → no subscriber | **PASS** — one message received; it included an EKF diagnostic reporting normal operation. |
| Robot description | `/a300_00000/robot_description` | `std_msgs/msg/String` | `robot_state_publisher` → `controller_manager` | Graph endpoints discovered. |
| Simulation | `/clock` | `rosgraph_msgs/msg/Clock` | `clock_bridge` → 11 subscribers | **PASS** — one message received. |
| TF (names/types only) | `/a300_00000/tf` | `tf2_msgs/msg/TFMessage` | 4 publishers → one listener | Topic metadata discovered; transform contents/correctness **NOT TESTED**. |
| TF (names/types only) | `/a300_00000/tf_static` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` → one listener | Topic metadata discovered; transform contents/correctness **NOT TESTED**. |
| Gamepad input | `/a300_00000/joy_teleop/joy` | `sensor_msgs/msg/Joy` | `joy_node` → `teleop_twist_joy_node` | Graph endpoints discovered; device data **NOT TESTED**. |

The topic list contained **34 topics** total. Other entries included controller-manager statistics and lifecycle events, interactive-marker topics, `/parameter_events`, and `/rosout`.

#### QoS observations

Selected endpoints reported `RELIABLE` publishers with `BEST_EFFORT` subscribers on `/a300_00000/cmd_vel`, `/a300_00000/platform/cmd_vel`, raw odometry, and joint states. The publisher on `/clock` was `RELIABLE`; its subscribers were `BEST_EFFORT`. QoS was inspected only; no QoS settings were changed.

#### Sensors and command flow

No camera, image, laser scan, point cloud, IMU, or GPS topics appeared in the topic list. The gamepad input topic was `/a300_00000/joy_teleop/joy` (`sensor_msgs/msg/Joy`); the configured PS4 device could not be opened. This matches the existing robot configuration's empty camera/GPS/IMU/LiDAR sensor arrays.

The user pressed W and A during the manual check and confirmed the robot moved. The one-shot echo on `/a300_00000/cmd_vel` captured a `TwistStamped` with `linear.x: 1.0` and zero angular velocity. The live graph directly showed `cmd_vel_bridge` publishing that topic to `twist_mux`, and `twist_mux` publishing `/a300_00000/platform/cmd_vel` with subscribers `cmd_vel_bridge` and `platform_velocity_controller`. The physical GUI-to-Gazebo transport segment is described by the inspected Clearpath launch/configuration; ROS-side endpoints and the keyboard message were directly observed.

### 5. Verification
- **PASS** — ROS 2 Jazzy CLI and workspace overlay sourced in a temporary shell; `ros2 node list` and `ros2 topic list -t` completed.
- **PASS** — 14 nodes and 34 topics discovered; detailed publisher/subscriber and QoS information obtained for the relevant interfaces.
- **PASS** — One-message reads succeeded for `/clock`, `/a300_00000/platform/joint_states`, `/a300_00000/platform/odom`, and `/a300_00000/diagnostics`.
- **PASS** — During user-operated keyboard input, `/a300_00000/cmd_vel` carried a nonzero forward command; the user confirmed movement.
- **NOT TESTED** — TF values/correctness, sensor data or accuracy, SLAM, autonomous navigation, and the gamepad data stream.

### 6. Problems and Solutions
- `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` were unset in the inspection shell. No persistent environment changes were made.
- The launch log reports that `/dev/input/ps4` could not be opened. The gamepad nodes/topics were present, but hardware input was unavailable; keyboard control was separately confirmed.
- `/a300_00000/rc_teleop/cmd_vel`, `platform/emergency_stop`, and `platform/safety_stop` had subscribers but no discovered publishers. These interfaces were recorded as currently unconnected, not modified.
- The filtered odometry, dynamic joint-state, and controller output topics had publishers but no discovered subscribers at inspection time.
- During shutdown, Gazebo did not exit within five seconds after SIGINT; the launch escalated to SIGTERM. Remaining Gazebo processes were stopped by their recorded PIDs and confirmed absent.

### 7. Evidence
- [Node list](../evidence/command_outputs/2026-11-12-ros-001-node-list.txt), [topic list](../evidence/command_outputs/2026-11-12-ros-001-topic-list.txt)
- [Node details](../evidence/command_outputs/2026-11-12-ros-001-node-info.txt), [topic endpoints and QoS](../evidence/command_outputs/2026-11-12-ros-001-topic-info.txt)
- One-message checks: [clock](../evidence/command_outputs/2026-11-12-ros-001-echo-clock.txt), [joint states](../evidence/command_outputs/2026-11-12-ros-001-echo-joints.txt), [odometry](../evidence/command_outputs/2026-11-12-ros-001-echo-odom.txt), [diagnostics](../evidence/command_outputs/2026-11-12-ros-001-echo-status.txt), and [keyboard velocity](../evidence/command_outputs/2026-11-12-ros-001-echo-keyboard-cmd-vel.txt)
- [Runtime launch log excerpt](../evidence/logs/2026-11-12-ros-001-runtime.log)

### 8. Reproducibility
In a terminal with the existing workspace installed, run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py generate:=false
```

After startup, use `ros2 node list`, `ros2 topic list -t`, and `ros2 topic info <topic> --verbose`. To check a message without waiting indefinitely, use `timeout 8s ros2 topic echo <topic> --once`. To observe keyboard velocity, start an echo on `/a300_00000/cmd_vel` and manually press a key in the Gazebo window. Do not publish a command from the terminal for this check.

### 9. Thesis Writing Notes
The simulation exposes a working ROS-side command path, raw and filtered odometry publishers, joint states, diagnostics, and simulation time. Record which links were observed from the live ROS graph versus those inferred from Clearpath launch/configuration. The current topic graph exposes no robot camera, LiDAR, IMU, or GPS streams; integrating those remains future work. TF topic presence is documented only—the transform graph and correctness belong to Task 5.
