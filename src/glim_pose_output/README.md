# SLAM-004A: GLIM local base-pose output

Small ROS 2 extension for the installed GLIM **1.2.2** ABI. It uses GLIM's
`create_extension_module` factory and `ExtensionModuleROS2::create_subscriptions(Node&)`
hook to create a publisher on the existing host node. It creates no ROS node,
subscription, timer, executor, viewer, TF listener or TF broadcaster.

`OdometryEstimationCallbacks::on_update_new_frame` supplies the newest IMU state
after correction by the point-cloud observation. This is **local odometry**;
global mapping/loop-closure corrections are not applied. The initial frame does
not invoke this hook; expect output from subsequent corrected frames after IMU
initialization. The rate follows processed clouds, nominally **10 Hz** for this
setup, and can fall with processing backlog or rejected inputs. It is not a
100 Hz IMU prediction stream or a fixed-rate heartbeat.

| Output property | Meaning |
|---|---|
| Topic | `/a300_00000/slam/glim/pose` |
| Type | `geometry_msgs/msg/PoseStamped` |
| `header.frame_id` | `glim_odom`: this GLIM process's local estimator reference |
| Pose body | Husky `base_link` (PoseStamped has no child-frame field) |
| Timestamp | `EstimationFrame::stamp`, rounded from seconds to nanoseconds |
| QoS | Best effort, volatile, keep last 10; subscribers must accept best effort |

`glim_odom` is not Clearpath `odom`, `map`, or Gazebo world, and no transform to
those frames is established or broadcast. A future switching manager must first
establish the required frame/time alignment and health criteria. No covariance,
confidence, global consistency or accuracy is asserted by this message.

The separate `config_pose_output.json` contains the verified static transform
`T_base_imu = [0, 0, 0.294759, 0, 0, 0, 1]` in TUM order. The conversion is:

```text
T_glim_odom_base = T_world_imu * inverse(T_base_imu)
```

Here GLIM's variable name `T_world_imu` refers to its local odometry estimate.
The sensor setting `T_lidar_imu = [0, 0, -0.0475, 0, 0, 0, 1]` is unchanged.
The base transform follows the verified Xacro chain, independently of the LiDAR
extrinsic. Changes to robot mounting would require a reviewed new configuration.

Frame values are copied inside the odometry callback; no mutable GLIM frame is
retained. Missing/invalid extrinsics fail initialization. Invalid stamps or
nonfinite/nonrigid poses are rejected without publication. Publication exceptions
are contained and reported (first and every 100th rejection). No estimator state,
covariance request, optimizer or mapping options are changed. Publishing adds
small in-thread work; its actual processing cost remains a runtime test item.

The callback uses weak state because GLIM clears extensions before destroying its
asynchronous estimator. It becomes a no-op after extension destruction; a callback
already in flight keeps its own publisher/state alive. GLIM 1.2.2 retains loaded
library handles for process lifetime. Its unsynchronized callback vector is not
modified during teardown. This is a process-lifetime extension, not a hot-unload
facility; enable one instance per GLIM process, before live inputs.

Files: `src/extension.cpp` supplies the publisher/factory/hook; `src/pose_output.cpp`
and `include/glim_pose_output/pose_output.hpp` implement conversion and validation;
`test/test_pose_output.cpp` contains ten deterministic tests without ROS
initialization; `test/check_configuration.py` audits snapshots and shared-library
resolution without constructing GLIM or ROS nodes. CMake links `glim::glim`,
`rclcpp`, and `geometry_msgs`, never `glim_ros` or `rviz_viewer`.

See the [build, checks and proposed runtime procedure](../../docs/experiments/slam004a_pose_output.md).
Runtime publication and live TF authority checks remain **NOT TESTED**.
