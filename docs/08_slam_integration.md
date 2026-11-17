# 08. SLAM Integration

### 1. Objective

Prepare a project-owned PointCloud2 adapter for a later GLIM interface test. The adapter appends a recognized per-point time field containing zero seconds for every point. This is an instantaneous/global-shutter simulation approximation; it does not model a spinning LiDAR's acquisition timing or motion distortion.

SLAM-001A statically prepared the adapter. Gazebo was launched for the bounded SLAM-001B runtime check; GLIM was not launched. Live adapter publication was verified, but GLIM runtime compatibility remains untested.

### 2. System Configuration

- ROS 2 Jazzy; workspace package: `husky_a300_sensor_extensions`.
- Input: `/a300_00000/sensors/helios_16p/points`.
- Separate output: `/a300_00000/sensors/helios_16p/points_glim`.
- Node: `pointcloud2_timing_adapter.py`, using `rclpy`, `sensor_msgs/msg/PointCloud2`, and sensor-data QoS.
- Launch: [`pointcloud2_timing_adapter.launch.py`](../src/husky_a300_sensor_extensions/launch/pointcloud2_timing_adapter.launch.py), with `input_topic`, `output_topic`, and `use_sim_time` arguments.
- Adapter implementation: [`pointcloud2_timing_adapter.py`](../src/husky_a300_sensor_extensions/scripts/pointcloud2_timing_adapter.py).

The LiDAR cloud previously observed in LiDAR-001C was organized as 360 × 16, little-endian, with `point_step=32`, `row_step=11520`, and 184,320 payload bytes. Its fields were `x/y/z` (`FLOAT32`, offsets 0/4/8), `intensity` (`FLOAT32`, offset 16), and `ring` (`UINT16`, offset 24). It had no recognized point-time field. See [LiDAR-001C runtime evidence](../evidence/command_outputs/2026-11-15-lidar-001c-runtime-validation.txt).

### 3. Implementation or Inspection Procedure

The selected GLIM source revisions were inspected before implementation:

- The [PointCloud2 converter](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/include/glim/util/ros_cloud_converter.hpp) recognizes `t`, `time`, `time_stamp`, and `timestamp`. It reads `UINT32` times as nanoseconds and `FLOAT32`/`FLOAT64` as seconds. It requires floating-point XYZ fields.
- The [time settings](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/util/time_keeper.cpp) treat a present point-time vector as relative when auto-detection sees a maximum below one. An all-zero vector has matching point count, is neither negative nor greater than one second, and remains zero relative to the cloud header timestamp. If no recognized time field is present, this revision instead estimates pseudo-times from scan intervals and point order. Cloud header stamps must remain monotonic; the adapter preserves them.
- The [sensor configuration](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/config/config_sensors.json) documents `perpoint_time_scale=1.0` as seconds and `global_shutter_lidar=true` as zeroing point times and disabling deskewing. Enable that setting in a future GLIM test configuration; no GLIM configuration was changed here.
- The [GLIM cloud preprocessor](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/preprocess/cloud_preprocessor.cpp) adds and sorts the times, sets scan end to the header stamp plus the last relative offset, and in global-shutter mode zeroes times after filtering. With this adapter, scan end equals the header stamp; no scan duration or firing sequence is represented. Sorting equal zero times does not preserve or encode physical point order. The inspected [GTSAM Points sampling code](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/src/gtsam_points/types/point_cloud_cpu_funcs.cpp) retains point times; voxel averaging of zero values remains zero.
- The [GLIM ROS 2 callback](https://github.com/koide3/glim_ros2/blob/4d4ec524ccf1b02aa09b0af2af767ecc54343798/src/glim_ros/glim_ros.cpp) subscribes to PointCloud2 and Imu and calls `TimeKeeper::process` before cloud preprocessing. For this sensor, future GLIM configuration must use the observed LiDAR/IMU frames and a verified `T_lidar_imu`; the default example extrinsic must not be assumed correct. GLIM's default `ring_field` is empty, so set it to `ring` only if ring data is needed.

The adapter copies the incoming message into a separate output without changing the source topic. It preserves the header, frame ID, dimensions, endianness, density flag, existing field descriptors, and each original point's bytes. It appends an aligned `time` field (`FLOAT32`, count 1) with zero-valued bytes. Existing per-point padding and row padding are retained; new alignment padding is zero-filled. NaN and infinite coordinates are copied, not filtered. Malformed layouts and clouds already containing a recognized time field are rejected with an explicit ROS error.

### 4. Results

#### SLAM-001A — PointCloud2 Timing Adapter Preparation

- The output field order is the original fields followed by `time`. For the LiDAR-001C layout, `time` is at offset 32 and `point_step` becomes 36 bytes. With the observed tightly packed 360 × 16 input, expected output `row_step` is 12,960 bytes and payload length is 207,360 bytes.
- Every `time` value is IEEE-754 `FLOAT32` zero, interpreted as seconds relative to the original cloud header stamp. The adapter does not generate scan firing times or alter input point order; GLIM's subsequent equal-time sort does not encode timing order.
- The original LiDAR cloud, IMU, TF, bridge, EKF, and robot configuration are not modified. The new launch is separate and is not included in the Husky simulation launch.
- At completion of SLAM-001A, runtime publication and GLIM compatibility were **NOT TESTED**. SLAM-001B later verified live publication; GLIM runtime compatibility remains **NOT TESTED**.

#### SLAM-001B — Live PointCloud2 Timing Adapter Validation

- **Runtime setup:** After user approval, sourced `/opt/ros/jazzy/setup.bash` and `/home/feras/clearpath_ws/install/setup.bash`, loaded the private ROS domain from the test setup without recording its value, and set `ROS2CLI_DISABLE_DAEMON=1`. Launched `/home/feras/clearpath_lidar11b_test` using `clearpath_gz simulation.launch.py` with `generate:=false`; started the existing `rs_helios_16p_bridge.launch.py` and the separate `pointcloud2_timing_adapter.launch.py`. The adapter used its defaults (`input_topic`, `output_topic`, `use_sim_time:=true`); no robot movement command, generator, build, or configuration change was made.
- **Topics and message layout:** The original `/a300_00000/sensors/helios_16p/points` remained available as `sensor_msgs/msg/PointCloud2`; the adapter published separately to `/a300_00000/sensors/helios_16p/points_glim`. Five message pairs matched exactly by header timestamp and frame. Both were 360 × 16, little-endian, and `is_dense=false`. Original fields were `x/y/z FLOAT32` at offsets 0/4/8, `intensity FLOAT32@16`, and `ring UINT16@24`; original `point_step=32`, `row_step=11520`, and data length 184,320 bytes. The adapted message appended `time FLOAT32`, count 1, at offset 32; adapted `point_step=36`, `row_step=12960`, and data length 207,360 bytes.
- **Time field and preservation:** All 28,800 sampled `time` values were exactly zero, interpreted as relative seconds from the preserved simulation-time header stamp. In all five matched pairs, the header stamp, frame, dimensions, original field descriptors, and original per-point bytes (including XYZ, intensity, ring, and existing padding) were unchanged. This included non-finite XYZ values; the adapter did not filter them. The original source topic and stream were not replaced.
- **QoS and rates:** The bridge publisher used RELIABLE QoS; the adapter input subscription and adapted publisher used BEST_EFFORT QoS. The adapter's `use_sim_time` was true. In the fixed-window measurement, both clouds ran at approximately 10.00 Hz in simulation time and 9.99 Hz by wall-clock arrivals. `/clock` measured 333.3 Hz; the IMU delivered 501 samples in five seconds (100.0 Hz); raw and filtered odometry measured 47.6 and 47.4 Hz. Message gaps were not independently counted; processing latency was not measured.
- **TF and EKF regression:** TF resolved from `base_link` to `helios_16p_link` (0, 0, 0.342 m) and `imu_0_link` (0, 0, 0.295 m), both with identity rotation. The IMU frame and raw/filtered odometry remained available. Runtime EKF parameters showed `odom0=platform/odom`; `odom1` and `imu0` were unset, confirming the existing wheel-odometry-only configuration. The active setup matched its protected backup; combined IMU/EKF configuration and the validated IMU setup matched the Task 11B baseline.
- **Resource observations:** A five-second process sample measured the adapter at 11.8% of one logical CPU and 75.7 MiB RSS, and the bridge at 4.0% and 57.6 MiB RSS. Gazebo server and GUI measured 153.3%/1,075.9 MiB and 446.6%/1,205.5 MiB respectively. This is a single loaded WSL2 sample, not a benchmark or estimate of incremental adapter cost. No trustworthy processing-latency measurement was taken.
- **Logs and caveats:** No adapter data-processing errors or bridge warnings/errors were observed while streaming. On the requested SIGINT shutdown, the adapter's `rclpy.spin` emitted a `KeyboardInterrupt` traceback and the launch reported exit code -2; the process exited and no Task 11E process remained. Review shutdown handling before routine repeated use; no implementation change was made here. Gazebo/Clearpath emitted controller initialization and update-period/deprecation warnings, an `imu_0` hardware-info association warning, repeated velocity-timeout braking notices while no command was sent, and a joystick-device error. The hardware-info warning did not prevent the separately bridged IMU stream from publishing at 100 Hz. The other warnings did not prevent sensor publication. The high sampled Gazebo resource use means simulator scheduling remains a limitation. No full cloud payload was saved.
- **GLIM readiness:** The observed XYZ datatypes and explicit `time FLOAT32` seconds field match the inspected converter's schema, timestamps use simulation time, and both sensor frames are connected. This is a candidate stream for a controlled GLIM test, not verified GLIM compatibility. Before that test, confirm topic settings, the correctly directed LiDAR-to-IMU extrinsic, `global_shutter_lidar=true` for the explicit zero-time approximation, IMU acceleration/angular-velocity compatibility, BEST_EFFORT QoS compatibility, and adequate simulator performance. Preserve a separate comparison against the original no-time cloud to evaluate GLIM TimeKeeper inference.
- **Limitations and thesis implications:** Zero-valued point times represent an instantaneous/global-shutter approximation only; they do not model scan firing order, physical LiDAR timing, robot motion distortion, or motion compensation. The 10 Hz simulation cloud is synthetic and is not evidence of physical RS-Helios-16P behavior. Report the adapter experiment separately from GLIM's inferred pseudo-times and from any hardware evaluation.
- **Reproduction and evidence:** With Jazzy and the workspace overlay sourced and the private test domain loaded without printing it, run the isolated simulation with `generate:=false`, then launch `husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py` and `husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py`. Sanitized measurements and observations are in [SLAM-001B runtime evidence](../evidence/command_outputs/2026-11-15-slam-001b-runtime-validation.txt). All Task 11E processes were stopped after capture.

### 5. Verification

- PASS — targeted `colcon build --packages-select husky_a300_sensor_extensions`.
- PASS — eight deterministic pytest cases via `colcon test`; zero errors or failures.
- PASS — installed executable is listed by `ros2 pkg executables`.
- PASS — launch file parses and reports its three arguments with `ros2 launch ... --show-args`.
- PASS — organized and unorganized layouts, little- and big-endian byte preservation, point and row padding, header/field/dimension preservation, NaN/infinity preservation, intensity/ring bytes, zero times, empty clouds, and malformed inputs are covered by unit tests.
- PASS — the active setup matches `/home/feras/clearpath_imu001a_backup`; the validated IMU setup matches `/home/feras/clearpath_lidar11b_backup`.
- PASS — SLAM-001B live ROS publication, matched-message preservation, exact zero times, rates, sensor TF, IMU/odometry continuity, wheel-only EKF inputs, and protected configuration comparisons.
- NOT TESTED — GLIM ingestion or mapping, and physical LiDAR timing/motion-compensation accuracy.

### 6. Problems and Solutions

- The inspected GLIM converter indexes data as `point_step * point_index` and does not use PointCloud2 `row_step`. It therefore cannot safely consume row-padded organized clouds. The adapter preserves valid row padding as required; the observed Gazebo cloud has no row padding, and its adapted output remains tightly packed.
- The converter uses typed pointer reads without byte swapping based on `is_bigendian`. The adapter preserves byte order and adds zero bytes correctly, but this GLIM revision should only be tested with its native little-endian input path. LiDAR-001C observed little-endian data.
- GLIM's inspected time-keeper checks the first and last time values without guarding an empty cloud. The adapter returns a valid empty PointCloud2, but empty outputs must not be sent to this GLIM revision without a separately validated guard.
- GLIM preprocessing filters non-finite/out-of-range points later; the adapter itself preserves them. For the zero-time cloud, scan end equals the cloud timestamp, so it provides no motion-compensation evidence.

### 7. Evidence

Sanitized source findings, build/test results, launch parsing, and protected-setup comparisons are recorded in [SLAM-001A static-validation evidence](../evidence/command_outputs/2026-11-15-slam-001a-static-validation.txt).
Live adapter results, matched message checks, regression measurements, and runtime caveats are recorded in [SLAM-001B runtime evidence](../evidence/command_outputs/2026-11-15-slam-001b-runtime-validation.txt).

### 8. Reproducibility

From `/home/feras/clearpath_ws`, with ROS 2 Jazzy sourced:

```bash
colcon build --packages-select husky_a300_sensor_extensions
source install/setup.bash
colcon test --packages-select husky_a300_sensor_extensions
colcon test-result --verbose
ros2 pkg executables husky_a300_sensor_extensions
ros2 launch husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py --show-args
```

These are static/package checks. Do not interpret `--show-args` as a node launch or a runtime publication test.

### 9. Thesis Writing Notes

Describe this as a generic, instantaneous/global-shutter timing approximation used to prepare a GLIM interface test. Do not claim physical RS-Helios-16P firing timing, scan motion, or motion-compensation accuracy. A later evaluation must distinguish this approximation from any real spinning-LiDAR experiment and must verify the GLIM configuration, timestamps, frame/extrinsic, QoS, empty-cloud behavior, and runtime output independently.

### 10. SLAM-002A — GLIM Installation and Configuration Preflight

#### Scope and verified environment

This was a read-only installation/configuration preflight. GLIM was **not installed, built, or launched**, Gazebo was not launched, and no APT source, workspace, robot, sensor, EKF, or TF configuration was changed.

- Ubuntu 24.04.5 on WSL2; ROS 2 Jazzy and the existing `/home/feras/clearpath_ws/install` overlay resolve correctly. The workspace contains no GLIM or GTSAM source repository, and no GLIM/GTSAM ROS or system packages are installed.
- Available resources at inspection: 16 logical CPUs, 7.3 GiB RAM (about 5.3 GiB available), and about 948 GiB free disk.
- The WSL NVIDIA interface exposes an RTX 4060 (8 GiB VRAM, compute capability 8.9) and `libcuda.so.1`; `nvcc` and `libcudart` are absent. GPU visibility alone does not provide the CUDA development toolchain required for a CUDA build.
- Installed dependencies/tools include CMake 3.28.3, GCC 13.3, colcon, make, git, rosdep, Eigen 3.4, PCL 1.14, OpenCV 4.6, Boost 1.83, TBB 2021.11, fmt 9.1, spdlog 1.12, nanoflann 1.5.4, and OpenMP. Relevant ROS 2 Jazzy message, TF, rosbag, and image packages are present.
- Missing for a source build: GTSAM, GTSAM Points, and `libmetis-dev`. The optional viewer development dependencies Iridescence, GLFW, and GLM are absent. Ninja is absent, but make is available. These are inventory findings, not a request to install them.

#### Compatibility and installation decision

The [official installation guide](https://koide3.github.io/glim/installation.html), upstream CI, and current Ubuntu 24.04 PPA package index support the ROS 2 Jazzy / Ubuntu 24.04 combination. The inspected upstream revisions and PPA package versions are:

| Component | Inspected source | PPA CPU package/version |
|---|---|---|
| GLIM core | [`koide3/glim` `1a197c999e4865f5cab7c9cd25474128413ebbca`](https://github.com/koide3/glim/tree/1a197c999e4865f5cab7c9cd25474128413ebbca) | `ros-jazzy-glim` 1.2.2 |
| GLIM ROS 2 | [`koide3/glim_ros2` `4d4ec524ccf1b02aa09b0af2af767ecc54343798`](https://github.com/koide3/glim_ros2/tree/4d4ec524ccf1b02aa09b0af2af767ecc54343798) | `ros-jazzy-glim-ros` 1.2.2-0noble |
| GTSAM Points | [`koide3/gtsam_points` `8e94654558dfbe7ddb0341de300daaf17e25ee02`](https://github.com/koide3/gtsam_points/tree/8e94654558dfbe7ddb0341de300daaf17e25ee02) | `libgtsam-points-dev` 1.2.2 |
| GTSAM | [`borglab/gtsam` 4.3.0 commit `71a25ca36c084cbad1f872e812d6d97fbadfdb05`](https://github.com/borglab/gtsam/tree/71a25ca36c084cbad1f872e812d6d97fbadfdb05) | `libgtsam-notbb-dev` 4.3.0 |

These are source revisions inspected for compatibility and reproducibility; the PPA package index does not establish that its binaries were built from those exact commits. The PPA is not configured in this environment, so these versions are **not** current APT candidates and were not installed. Its live package index is newer than the PPA changelog text inspected; check `apt-cache policy` and a simulated install transaction before any future installation.

**Recommended for Task 12B:** use the official PPA's CPU package `ros-jazzy-glim-ros`, after reviewing the repository/key setup and a simulated APT transaction. This is the least complex supported route, avoids building large dependencies with only about 5.3 GiB RAM available, and leaves the Clearpath source workspace unchanged. It does add packages to the system ROS installation, so it is not an isolated ROS prefix. Keep project configs and run outputs in a sibling directory such as `/home/feras/glim_ws/`, not in `clearpath_ws` or upstream package files.

If system packages are not acceptable, the documented source route can use a separate sibling ROS workspace with the exact source commits above; do not clone/build GLIM inside `clearpath_ws` or track floating `master`. For a CPU-only source fallback, configure the GLIM core with `BUILD_WITH_CUDA=OFF`, `BUILD_WITH_VIEWER=OFF`, `BUILD_WITH_OPENCV=OFF`, and `BUILD_WITH_MARCH_NATIVE=OFF`; configure `glim_ros2` with `BUILD_WITH_CUDA=OFF`, `BUILD_WITH_VIEWER=OFF`, and `BUILD_WITH_CV_BRIDGE=OFF`. GLIM's core defaults otherwise enable CUDA, viewer, and OpenCV. Upstream CI at the inspected revisions includes Ubuntu Noble/Jazzy CPU builds; GPU builds are separate. The PPA also publishes CUDA variants, but they are not recommended for this first WSL test without a validated CUDA toolkit/compiler.

#### ROS executable, configuration, topics, and QoS

- The ROS package is `glim_ros`; its main executable is `glim_rosnode` (`ros2 run glim_ros glim_rosnode`). The inspected source also defines `glim_rosbag` and `validator_node`; viewer-enabled builds may provide `offline_viewer` and `map_editor`. No upstream ROS launch file was found.
- `glim_rosnode` takes `config_path` (an absolute project-owned directory is supported) and `dump_path`; set the standard ROS parameter `use_sim_time:=true`. Installed defaults are under the GLIM package's `share/glim/config`. Keep separate copied configurations for the two timing modes, and do not edit files under `/opt/ros`.
- The relevant JSON files/keys are `config.json` (`global.config_odometry`, `global.config_sub_mapping`, and `global.config_global_mapping`; the defaults select GPU modules). For CPU, select `config_odometry_cpu.json`, `config_sub_mapping_cpu.json`, and `config_global_mapping_cpu.json`. `config_ros.json` contains `points_topic`, `imu_topic`, `imu_qos`, `points_qos`, `imu_time_offset`, `points_time_offset`, `acc_scale`, `ang_scale`, `imu_frame_id`, `lidar_frame_id`, `base_frame_id`, `map_frame_id`, `odom_frame_id`, `publish_imu2lidar`, `enable_local_mapping`, `enable_global_mapping`, and `extension_modules`. `config_sensors.json` contains `T_lidar_imu`, `intensity_field`, `ring_field`, `autoconf_perpoint_times`, `autoconf_prefer_frame_time`, `perpoint_relative_time`, `perpoint_time_scale`, and `global_shutter_lidar`. Set both time offsets to zero for these aligned simulation streams; use SI scales (`acc_scale=1`, `ang_scale=1`) after confirming the live IMU units. Set `intensity_field` to `intensity` and `ring_field` to `ring` if those channels are consumed.
- Use absolute input names so the node namespace does not rewrite them: the IMU is `/a300_00000/sensors/imu_0/data`; Mode A's cloud is `/a300_00000/sensors/helios_16p/points_glim`; Mode B's is `/a300_00000/sensors/helios_16p/points`. Set the configured sensor frame IDs to `imu_0_link` and `helios_16p_link`, respectively.
- The inspected ROS subscriptions default to sensor-data QoS (BEST_EFFORT, volatile; IMU depth defaults to 1000, and points use the sensor-data profile default). A BEST_EFFORT subscriber is compatible with both the bridge's RELIABLE cloud publisher and the adapter's BEST_EFFORT publisher. Verify the live endpoints again before runtime; do not assume future publisher QoS remains unchanged. The viewer extension's map publisher uses RELIABLE keep-last QoS with depth 1.
- ROS map/odometry topics are provided by the `librviz_viewer.so` extension, not by the core subscriber alone. It publishes the private `~/map` PointCloud2 and odometry/pose topics such as `~/odom`, `~/lidar_odom`, and corrected variants; their resolved names depend on the node namespace/name. Global map publication is demand-driven and computationally costly (the inspected implementation warns about it and updates at roughly 10-second intervals when subscribed). Initially leave this extension and RViz2 disabled and do not subscribe to the global map.
- **TF ownership is an unresolved integration risk.** The inspected viewer extension broadcasts GLIM map/odom/base transforms; Clearpath/EKF already own `odom -> base_link`. Its default `publish_imu2lidar` can also add a sensor edge to a TF tree that already connects both sensor frames through `base_link`. Set `publish_imu2lidar=false`, but that alone does not resolve the odometry TF ownership conflict. Do not enable the ROS viewer/output extension with the current TF publishers until a non-conflicting ownership/remapping plan is validated. Headless startup can be tested without that extension; ROS map/odometry publication and TF integration then remain untested.

#### LiDAR-to-IMU transform direction

GLIM's `T_lidar_imu` maps coordinates expressed in the IMU frame into the LiDAR frame: `p_lidar = T_lidar_imu * p_imu`. It is therefore **not** the transform that expresses the LiDAR origin in IMU coordinates.

From the measured fixed chain, `T_base_imu` is approximately identity rotation with translation `[0, 0, +0.295] m`, and `T_base_lidar` is approximately identity rotation with `[0, 0, +0.342] m`. Thus:

```text
T_imu_lidar = inverse(T_base_imu) * T_base_lidar
            ≈ [I, (0, 0, +0.047 m)]   (direct reported TF: about +0.048 m)
T_lidar_imu = inverse(T_imu_lidar)
            ≈ [I, (0, 0, -0.047 to -0.048 m)]
```

The provisional GLIM config should therefore use identity rotation and a **negative Z** translation, approximately `[0, 0, -0.048, 0, 0, 0, 1]` in `[x,y,z,qx,qy,qz,qw]` order. Its rotation matrix is identity. The 0.001 m difference between the rounded base offsets and the directly reported sensor-to-sensor transform is rounding; read the unrounded TF before fixing the runtime config. This is only the simulated mount, not a physical sensor calibration.

#### IMU compatibility and timing modes

- The ROS callback consumes `linear_acceleration`, `angular_velocity`, and the message timestamp; orientation and message covariance are not used in the inspected callback. Acceleration is scaled by `acc_scale` (auto-detection when configured as zero); angular velocity uses `ang_scale` (default 1). The simulated stationary `+Z` acceleration of about `9.8 m/s²` is consistent with SI specific force. Preserve SI units (`acc_scale=1`, `ang_scale=1`) unless a live check disproves this. The callback assumes compatible sensor coordinate conventions; the extrinsic does not correct a handedness/units mismatch.
- TimeKeeper uses message stamps, expects monotonic input, and reports large gaps. The simulated sensor's simplified noise and zero covariance do not establish hardware-grade noise behavior; covariance is ignored by this callback. Keep the robot stationary through the CPU configuration's approximately 3-second IMU initialization window, and verify rate, units, monotonic timestamps, and gravity convention before interpreting results.

Use independent config directories and separate runs:

| Mode | `points_topic` | Time configuration | Interpretation |
|---|---|---|---|
| A — explicit zero-time adapter | `/a300_00000/sensors/helios_16p/points_glim` | `autoconf_perpoint_times=false`, `perpoint_relative_time=true`, `perpoint_time_scale=1.0`, `global_shutter_lidar=true` | Adapter's `time FLOAT32` values are zero relative seconds. Global-shutter preprocessing also zeroes point times and disables deskewing. This is an instantaneous approximation, not physical scan timing. |
| B — native TimeKeeper inference | `/a300_00000/sensors/helios_16p/points` | No point-time field; `global_shutter_lidar=false` | TimeKeeper estimates scan intervals from cloud header stamps and assigns pseudo-times by point order. At about 10 Hz, this is roughly a 0.1 s interval, not GPU-LiDAR firing timing. The first cloud has no prior interval; estimates improve only as intervals accumulate. |

Both use `/a300_00000/sensors/imu_0/data`, simulation time, the same validated extrinsic, and compatible sensor-data QoS. Do not feed an empty cloud to this inspected GLIM converter; its empty-time path is not safely guarded. Record the two modes independently and do not claim either is more accurate without a controlled comparison.

#### Resource assessment, next steps, and rollback

Previous simulation evidence measured LiDAR around 10 Hz and IMU around 100 Hz, with occasional multi-second scheduling gaps, EKF missed-update warnings, and high Gazebo CPU/RSS use in WSL2. GLIM plus Gazebo, RViz2, and the interactive GLIM viewer is not a prudent first combination on the observed 7.3 GiB RAM system. Start with the CPU package/modules, a small thread count, no RViz2, no GLIM viewer extension, and no map subscriber. Monitor wall-clock and simulation-time rates, message gaps, memory, CPU, and GLIM warnings. Only proceed to a short user-controlled movement run after a stationary startup succeeds; do not issue automatic movement.

Recommended implementation sequence (not executed):

1. In a separate approved task, review the official PPA key/repository setup, check `apt-cache policy`, and run a simulated install transaction. Install only the supported CPU ROS package if its candidate/dependency closure matches the inspected versions; then verify `ros2 pkg executables glim_ros` and the installed config directory.
2. Copy the installed default config into two project-owned directories outside `clearpath_ws`; select CPU modules, set the exact topics/QoS/frames, provisional inverse extrinsic, time mode, simulation time, and unique per-run dump paths.
3. Recheck live topic types, non-empty cloud layout/endian/row stride, publisher QoS, IMU units/rate/stamps, and the fixed TF chain. Preserve the existing robot/ EKF configuration.
4. Run one bounded stationary headless startup for each mode, separately, with the robot held still through initialization. Record logs and resource/timing samples; do not enable ROS viewer TF until its conflict is resolved.
5. After successful stationary runs and explicit user control, perform a short movement test; inspect GLIM dump/map/trajectory artifacts and, only after TF ownership is solved, ROS odometry/map topics. Compare modes without treating pseudo-times as measured physical firing times.
6. Preserve sanitized configs, command outputs, package versions, logs, resource samples, and the exact simulation/setup identity. Do not save full point-cloud payloads or private domain values.

For rollback, remove only GLIM packages and the PPA source/key introduced by that future task, after checking package ownership/dependencies; do not run broad `autoremove`. Keep the Clearpath workspace and pre-existing ROS packages intact. Remove only explicitly created GLIM config/result directories if cleanup is requested.

**Status at SLAM-002A completion (2026-11-15):** GLIM installed — **NO**. GLIM runtime tested — **NO**. Task 12B had not started. The user later completed the CPU package installation manually; see SLAM-002B below. Detailed preflight findings and source references are in [SLAM-002A preflight evidence](../evidence/command_outputs/2026-11-15-slam-002a-glim-preflight.txt).

### 11. SLAM-002B — GLIM CPU Package Installation Verification

Task 12B was completed manually by the user in the WSL Ubuntu terminal. The user reported that the official GLIM Ubuntu 24.04 PPA was configured, APT metadata updated, the install transaction simulated, the official CPU packages installed, and `sudo ldconfig` completed. The user reported ten new packages, zero upgrades, and zero removals. No APT command or privileged operation was run during this configuration task.

Independent read-only verification in the current shell:

- Ubuntu 24.04.5 / ROS 2 Jazzy resolve; `source /opt/ros/jazzy/setup.bash` and `source /home/feras/clearpath_ws/install/setup.bash` expose the packages.
- `dpkg-query` confirms `ros-jazzy-glim-ros` 1.2.2-0noble, `ros-jazzy-glim` 1.2.2-0noble, `libgtsam-notbb-dev` 4.3.0, `libgtsam-points-dev` 1.2.2, and `libiridescence-dev` 1.0.2.
- `ros2 pkg executables glim_ros` lists `glim_rosnode`, `glim_rosbag`, `validator_node`, `offline_viewer`, and `map_editor`.
- `ros2 pkg prefix glim_ros` and `ros2 pkg prefix clearpath_gz` resolve to `/opt/ros/jazzy`; `husky_a300_sensor_extensions` resolves to `/home/feras/clearpath_ws/install/husky_a300_sensor_extensions`.
- With Jazzy and the workspace overlay sourced, `ldd` reports the required GLIM ROS/core libraries and selected CPU modules resolved, with no `not found` entries.

The ten-package count and zero-upgrade/zero-removal transaction result above are user-reported; they were not independently reconstructed from APT history. Executable discovery and package versions/prefixes were independently verified. These checks do not prove that the GLIM node has started successfully.

### 12. SLAM-002C — GLIM CPU Configuration Preparation

#### Installed configuration and package-version cross-check

The installed default configuration is `/opt/ros/jazzy/share/glim/config`. The installed package contains 15 JSON-with-comments files, including `config.json`, `config_ros.json`, `config_sensors.json`, `config_preprocess.json`, CPU and GPU odometry/mapping variants, logging, and viewer config. These system files were not edited. Each private mode directory contains its own copy of all 15 files.

The installed 1.2.2 files are authoritative for these configurations and differ from the earlier inspected floating/master snapshot: `config_odometry_cpu.json` specifies `initialization_mode: "LOOSE"` (not the previously summarized `"ROBUST"`); `config_sensors.json` uses the single `imu_bias_noise` key; and ROS configuration has no `ang_scale` setting. The v1.2.2 ROS callback passes angular velocity through in its existing rad/s value, so no unsupported `ang_scale` key was added. Acceleration is explicitly scaled by `acc_scale: 1.0`. Version-specific behavior was cross-checked against the upstream GLIM and GLIM ROS 2 v1.2.2 tags; that source check is not a cryptographic provenance statement about the PPA build.

#### Project-owned configuration layout and CPU choices

Created the new owner-only workspace `/home/feras/glim_ws/` with mode 0700 directories:

```text
/home/feras/glim_ws/
├── configs/
│   ├── mode_a_zero_time/       # 15 private copies of installed defaults
│   └── mode_b_timekeeper/      # 15 private copies of installed defaults
├── runs/
│   ├── mode_a/
│   └── mode_b/
└── evidence/
```

Copied configuration files are mode 0600. Both `config.json` files select the installed CPU files `config_odometry_cpu.json`, `config_sub_mapping_cpu.json`, and `config_global_mapping_cpu.json`; the active odometry library is `libodometry_estimation_cpu.so`. All selected library names exist under `/opt/ros/jazzy/lib`. The config trees retain unused GPU example files as part of the verbatim installed-default copies, but none is referenced by the active `config.json`.

The installed `config_ros.json` explicitly documents that if both `enable_local_mapping` and `enable_global_mapping` are false, odometry estimation runs. Both modes use that smallest documented processing path; their config still selects the CPU mapping files so a later, separately tested mapping experiment can enable those components. The CPU odometry config uses two threads and a 3.0-second initialization window. GLIM standalone/ROS viewer extensions, RViz, local mapping, and global mapping are disabled.

#### Common ROS settings

Both mode configs set:

- `imu_topic`: `/a300_00000/sensors/imu_0/data`
- `imu_frame_id`: `imu_0_link`; `lidar_frame_id`: `helios_16p_link`; `base_frame_id`: `base_link`
- `imu_time_offset: 0.0`, `points_time_offset: 0.0`, and `acc_scale: 1.0`
- `intensity_field: "intensity"` and `ring_field: "ring"`
- ROS `imu_qos` and `points_qos`: `sensor_data` profile (BEST_EFFORT, VOLATILE). IMU queue depth is 1000; point QoS is explicitly BEST_EFFORT, VOLATILE, KEEP_LAST depth 5, matching the sensor-data profile default.
- `enable_local_mapping: false`, `enable_global_mapping: false`, `extension_modules: []`, and `publish_imu2lidar: false`

Prior live evidence records a RELIABLE, VOLATILE original cloud publisher and BEST_EFFORT adapted cloud publisher; a BEST_EFFORT GLIM subscription is compatible with both. Prior IMU validation recorded message type, frame, and rate but not the live publisher QoS. Confirm actual IMU and cloud endpoint QoS with `ros2 topic info --verbose` before a future runtime test; configuration is not proof of live endpoint compatibility.

`use_sim_time` is a ROS node parameter, not a GLIM JSON key. The future command must pass `-p use_sim_time:=true`. Input topic names are absolute, so the node namespace does not rewrite subscriptions.

#### Exact simulated extrinsic from the configured model

The static chain in `/home/feras/clearpath_lidar11b_test/robot.urdf.xacro`, the installed A300 and top-plate Xacros, the MicroStrain IMU macro, and the project LiDAR Xacro is:

```text
base_link -> chassis_link                 [0, 0, 0]
chassis_link -> default_mount             [0, 0, 0.229559]
default_mount -> top_plate_link           [0, 0, 0]
top_plate_link -> top_plate_default_mount [0, 0, 0.0127]
top_plate_default_mount -> imu_0_base_link[0, 0, 0.05]
imu_0_base_link -> imu_0_link             [0, 0, 0.0025]
top_plate_default_mount -> helios_16p_link[0, 0, 0.10]
```

All joint rotations are identity. Therefore:

```text
T_base_imu =
[[1,0,0,0], [0,1,0,0], [0,0,1,0.294759], [0,0,0,1]]

T_base_lidar =
[[1,0,0,0], [0,1,0,0], [0,0,1,0.342259], [0,0,0,1]]

T_imu_lidar = inverse(T_base_imu) * T_base_lidar
            = translation [0, 0, +0.0475], identity rotation
```

`T_imu_lidar` expresses the LiDAR frame/origin in IMU coordinates. Installed GLIM v1.2.2 documents `T_lidar_imu` as the transform mapping IMU-coordinate points into the LiDAR frame, `p_lidar = T_lidar_imu * p_imu`; GLIM therefore requires the inverse:

```text
T_lidar_imu = inverse(T_imu_lidar)
            = translation [0, 0, -0.0475], identity rotation
            = [0.0, 0.0, -0.0475, 0.0, 0.0, 0.0, 1.0]
```

The serialized order is TUM `[x, y, z, qx, qy, qz, qw]`, confirmed by installed GLIM's config conversion header. This is the exact simulated Xacro transform, not a physical sensor calibration. The prior rounded runtime TF figures (+0.295 m and +0.342 m) obscure the exact 0.0475 m difference; no robot description or TF publisher was modified.

#### Mode A — adapted zero-time cloud

Directory: `/home/feras/glim_ws/configs/mode_a_zero_time/`

- `points_topic`: `/a300_00000/sensors/helios_16p/points_glim`
- `autoconf_perpoint_times: false`
- `perpoint_relative_time: true`; `perpoint_time_scale: 1.0` seconds
- `global_shutter_lidar: true`
- Exact `T_lidar_imu` above

The installed v1.2.2 TimeKeeper leaves explicit zero relative offsets at zero when auto-configuration is disabled. Preprocessing in global-shutter mode also zeroes point times and disables deskewing. This is an instantaneous/global-shutter approximation only; it does not represent physical firing times or demonstrate motion compensation.

#### Mode B — native TimeKeeper inference

Directory: `/home/feras/glim_ws/configs/mode_b_timekeeper/`

- `points_topic`: `/a300_00000/sensors/helios_16p/points`
- `autoconf_perpoint_times: true` (installed default); `global_shutter_lidar: false`
- Exact `T_lidar_imu` above

In the v1.2.2 TimeKeeper, absent point-time values are synthesized from point order and a scan duration estimated from LiDAR header-stamp intervals. The first cloud has no prior interval and receives zero offsets. Subsequent intervals contribute to a running median; 1000 samples lock the estimate. Durations below 0.01 s or above 1.0 s are rejected with a warning and zero pseudo-times; gaps above 0.5 s warn, and timestamp rewinds cause the cloud to be skipped. At the observed nominal 10 Hz the inferred interval is about 0.1 s, not the synthetic GPU-LiDAR's measured firing time. Do not equate this mode with physical timing or compare it as more accurate without a controlled experiment.

The inspected v1.2.2 `TimeKeeper::process` accesses the first and last time entries without an empty-cloud guard. Exclude empty clouds from both future tests unless a separately validated guard is added. No such guard or code change was made here.

#### TF/output isolation and expected results

The v1.2.2 ROS node itself loads ROS output/TF behavior through extension modules. `librviz_viewer.so` creates the `~/map`, `~/odom`, pose/cloud publishers and broadcasts `map -> odom -> base_frame`; it can conflict with Clearpath's existing `odom -> base_link`. Both prepared `extension_modules` arrays are empty, and `publish_imu2lidar` is false. Thus the prepared headless configs do not load that extension or create GLIM ROS TF publishers. Do not enable the extension until TF ownership is separately resolved.

For a future run with these configs, expected work is GLIM CPU odometry internally and a GLIM save/dump under the chosen `dump_path` at shutdown. Because mapping and ROS viewer extensions are disabled, do **not** expect GLIM map or `~/odom` ROS output topics from these configs. No GLIM or ROS output was launched or observed in this task.

#### Future stationary headless commands (documented, not executed)

Use a fresh unique dump directory for each run. These commands do not start Gazebo; a separately authorized task must first establish the already-validated simulation and sensor topics without running a generator.

Common shell setup:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export ROS_DOMAIN_ID="$(python3 -c 'import yaml; d=yaml.safe_load(open("/home/feras/clearpath_lidar11b_test/robot.yaml")); print(d["system"]["ros2"]["domain_id"])')"
export ROS2CLI_DISABLE_DAEMON=1
```

Mode A:

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_mode_a \
  -p use_sim_time:=true \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_zero_time \
  -p dump_path:=/home/feras/glim_ws/runs/mode_a/run_001
```

Mode B:

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_mode_b \
  -p use_sim_time:=true \
  -p config_path:=/home/feras/glim_ws/configs/mode_b_timekeeper \
  -p dump_path:=/home/feras/glim_ws/runs/mode_b/run_001
```

The installed v1.2.2 `glim_rosnode` source declares `config_path` (absolute directory accepted; relative paths resolve under the installed GLIM share directory) and `dump_path` parameters. Keep the robot stationary for the CPU odometry config's 3-second initialization window. Verify live nonempty point clouds, simulation-time progression, monotonic stamps, IMU units/rate, and all endpoint QoS before launch. Do not run these commands while another GLIM instance subscribes to the same mode or until a future task explicitly authorizes runtime startup.

#### Static validation and remaining limitations

- **PASS:** All 15 copied JSON-with-comments files in each mode parse using a comment-aware parser consistent with GLIM's nlohmann JSON comment-ignoring behavior.
- **PASS:** Every filename referenced by the active `config.json` exists in the mode directory; selected CPU module shared libraries exist.
- **PASS:** Active odometry uses `libodometry_estimation_cpu.so`; no active config path selects a GPU module.
- **PASS:** Topics, frames, scales, offsets, QoS, timing flags, exact directed extrinsic, and extension/TF safeguards match the intended settings.
- **PASS:** The two mode trees differ only in `points_topic`, `autoconf_perpoint_times`, and `global_shutter_lidar`; all other parsed settings are equal.
- **PASS:** No GLIM static validator was executed. `validator_node` is a ROS executable and was not run; JSON/config checks do not prove that GLIM loads or runs successfully.
- **PASS:** GLIM CPU libraries resolved with no missing shared dependencies when the ROS and workspace environments were sourced.
- **NOT TESTED:** GLIM startup, odometry output, mapping, ROS topic output, GPU/CUDA, live QoS compatibility for the IMU, performance with GLIM, or stationary behavior under the simulation.

Prior runtime evidence measured LiDAR around 10 Hz and IMU around 100 Hz, with occasional multi-second scheduling gaps and high Gazebo CPU/RSS use. The first separately authorized runtime task should be stationary, headless, CPU-only, one mode at a time, with the simulator already running and no RViz/viewer/map subscription. No Clearpath, IMU, LiDAR, EKF, TF, adapter, or Git configuration was changed by this preparation. Sanitized validation evidence is in [SLAM-002B/C evidence](../evidence/command_outputs/2026-11-17-slam-002b-c-glim-config-static.txt).

### 13. SLAM-002D — Stationary GLIM CPU Runtime Validation (Mode A)

#### Objective and setup

Ran one authorized stationary Mode A test using the combined isolated setup at `/home/feras/clearpath_lidar11b_test`, with `generate:=false`. GLIM 1.2.2-0noble used `/home/feras/glim_ws/configs/mode_a_zero_time`; the run directory was `/home/feras/glim_ws/runs/mode_a/run_20261117T125100_stationary/`. The command selected `libodometry_estimation_cpu.so`, set `use_sim_time:=true`, and supplied a new `glim_dump` path. No robot movement command, RViz, viewer, mapping, Mode B, or GLIM TF output was enabled.

The GLIM process started at 2026-11-17 12:57:16.343 and received SIGINT at 12:58:59.690 (about 103.35 seconds wall time). A bounded observer recorded 25.002 seconds of `/clock` progression over 25.27 seconds wall time after startup. The robot remained stationary; no commands were published to move it.

#### Live input and TF checks

- **PASS:** Original cloud `/a300_00000/sensors/helios_16p/points` and adapted cloud `/a300_00000/sensors/helios_16p/points_glim` were both published as `sensor_msgs/msg/PointCloud2`. In a 12-second matched capture, each ran at approximately 9.99 Hz. The adapted publisher and GLIM's BEST_EFFORT/VOLATILE subscription were present.
- **PASS:** The original and adapted layouts were 360×16, little-endian and nonempty. The original had `point_step=32`, `row_step=11520`, and 184,320 payload bytes; the adapted stream had `point_step=36`, `row_step=12960`, and 207,360 bytes. XYZ were FLOAT32 at offsets 0/4/8; intensity was FLOAT32 at offset 16; ring was UINT16 at offset 24; adapted `time` was FLOAT32 at offset 32.
- **PASS:** Across 120 pairs matched by identical header timestamps, every original field's bytes were identical, including non-finite XYZ values. Headers, frame IDs (`helios_16p_link`), dimensions, field descriptors and endianness were preserved. All 5,760 adapted point times in each pair were exactly zero.
- **PASS:** `/a300_00000/sensors/imu_0/data` published RELIABLE/VOLATILE; GLIM's BEST_EFFORT/VOLATILE subscription was compatible. IMU frame was `imu_0_link`; the 12-second capture contained 1,197 messages with strictly increasing simulation timestamps, mean Z acceleration 9.800000 m/s² (range 9.79999999995–9.80000000005), and maximum angular-speed magnitude 6.49×10⁻¹³ rad/s.
- **PASS:** Both cloud streams, IMU stamps and `/clock` advanced monotonically. During the bounded observation, GLIM's adapted-cloud and IMU subscriptions were visible and the observer received 232 clouds and 2,459 IMU messages. Runtime TF resolved `base_link` to `imu_0_link` at +0.295 m Z and to `helios_16p_link` at +0.342 m Z, both with identity rotation. The configured exact simulated `T_lidar_imu` remained `[0, 0, -0.0475, 0, 0, 0, 1]`.
- **PASS:** The existing EKF continued to subscribe to wheel odometry and publish `/a300_00000/platform/odom/filtered`; no IMU input was added to the EKF. The robot setup and protected IMU/EKF files were not edited. The combined setup's `imu_0.yaml`, localization and control files matched its saved baseline. The IMU-only setup's existing `imu_0.yaml` matched the saved combined sensor baseline; its localization/control files matched their saved baseline. The active `/home/feras/clearpath` and its saved baseline both have no `sensors/config/imu_0.yaml`; their localization/control files matched.

#### GLIM results and limitations

- **PASS:** GLIM loaded the requested Mode A config and logged `load libodometry_estimation_cpu.so`. The `/a300_00000/glim_mode_a` node stayed alive through the 25.002-second simulation-time observation and subscribed to the configured cloud and IMU topics. The endpoint QoS was compatible.
- **PASS:** GLIM logged `estimate initial IMU state` followed by `initial IMU state estimation result`, with 93 initialization values. This confirms stationary IMU initialization, not successful LiDAR odometry.
- **PARTIAL / NOT CONFIRMED:** The live cloud stream and GLIM subscription were verified, but GLIM 1.2.2 emitted no per-scan callback, preprocessing, or odometry-state evidence during this run. Its only later odometry-related log was `waiting for odometry estimation` on shutdown. Therefore LiDAR frames processed through the odometry pipeline and an odometry estimate are **not confirmed**; do not treat node survival or the IMU initialization as proof.
- **PASS:** No GLIM warning, exception, or crash was observed. Sampled GLIM process usage was about 22.9–23.1% CPU and 172–174 MiB RSS. Available memory before GLIM was 3.3 GiB; Gazebo had already shown high CPU use (about 441% GUI and 150% server in a process snapshot), so this is not a performance benchmark.
- **NOT PRODUCED:** No `glim_dump` directory or trajectory/odometry artifact was created. With mapping and ROS output extensions disabled, no GLIM map or odometry ROS topics were expected; none appeared.
- **SHUTDOWN:** GLIM exited after SIGINT. The bridge exited cleanly. The adapter's known SIGINT `KeyboardInterrupt` traceback occurred; it exited and was not modified. The Gazebo launch escalated to SIGTERM after its five-second shutdown timeout; ownership of the remaining `gz sim` wrapper/server/GUI processes was checked, and only those processes from this run were sent SIGINT. All Task 12D processes were subsequently absent.

#### Reproduction and evidence

After sourcing ROS Jazzy and the existing workspace overlay, derive the ROS domain from the private combined setup without recording its value. Start the existing simulation with `setup_path:=/home/feras/clearpath_lidar11b_test generate:=false`, then the existing LiDAR bridge and timing-adapter launch files. The exact GLIM command used was:

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_mode_a \
  -p use_sim_time:=true \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_zero_time \
  -p dump_path:=/home/feras/glim_ws/runs/mode_a/run_20261117T125100_stationary/glim_dump
```

Sanitized measurements are in [SLAM-002D runtime evidence](../evidence/command_outputs/2026-11-17-slam-002d-runtime-validation.txt). Bounded logs and observer output are kept in the private run directory above; no point-cloud payloads or ROS domain value were saved in Git evidence. This was Mode A only: Mode B, mapping, trajectory quality, and successful LiDAR odometry remain untested. The next task should add a version-supported way to observe GLIM scan/odometry progress, then repeat a short stationary test without changing the existing sensor or robot configuration.

### 14. SLAM-002E — GLIM Odometry Processing Diagnosis

#### Static diagnosis and conclusion

This phase was read-only with respect to the simulation and GLIM runtime: neither Gazebo nor GLIM was launched. The evidence from SLAM-002D confirms active sensor publishers, compatible QoS, a valid sensor TF chain, GLIM subscriptions, and initial IMU-state estimation. It does **not** show that GLIM's cloud callback, conversion, preprocessing, CPU odometry, or per-scan state update ran.

The missing odometry topic and dump are explained by the active configuration: local/global mapping and extension modules are disabled. `GlimROS::timer_callback()` retrieves estimator results but has no default ROS odometry publisher. With no mapping or extension consumer, the configured `dump_path` does not create an odometry trajectory. `waiting for odometry estimation` is emitted by the ordinary shutdown `wait()` path and is not an error or proof of a stalled estimator. Thus, the known issue is lack of observability; whether scan processing succeeds remains unverified.

#### GLIM 1.2.2 processing path and potential blockers

The matching GLIM ROS 2 and GLIM source revisions were inspected: `glim_ros2` `4d4ec524ccf1b02aa09b0af2af767ecc54343798` and `glim` `1a197c999e4865f5cab7c9cd25474128413ebbca`.

1. The ROS `PointCloud2` subscription calls `GlimROS::points_callback()`. It traces the cloud timestamp, calls `extract_raw_points()`, then `TimeKeeper::process()`, preprocessing, and asynchronous `insert_frame()`. Unsupported/missing required point fields can make extraction fail with `failed to extract points from message`. The TimeKeeper rejects a rewinding cloud stamp; it logs several malformed-time conditions but does not generally reject solely because relative point times are zero. An empty cloud is a hazard: the inspected TimeKeeper path accesses timestamp `front()`/`back()` without an empty-vector guard.
2. The preprocessor logs input and output point counts at TRACE. It downsamples, removes non-finite points and points outside configured 0.5–100 m range, and warns below 100 points; that warning alone does not reject a cloud. The LOOSE initializer skips a frame below 50 points and requires its configured three-second initialization window and sufficient IMU data. For this 5,760-point cloud and target 10,000, the sampling ratio is above 0.99, so the GTSAM Points random-grid sampler clones rather than downsamples the cloud. Global-shutter mode zeros point times; with Mode A's explicit zeros, `scan_end_time` equals the cloud stamp. Intensity is carried into the CPU point cloud; ring is parsed but is not used by the CPU geometric odometry path.
3. `AsyncOdometryEstimation` buffers frames until the newest IMU stamp reaches the frame's `scan_end_time`. It logs a debug wait message and warns if more than ten frames accumulate. The CPU estimator's `insert_frame()` trace confirms a frame reached odometry. Between scans, fewer than two integrated IMU samples produce explicit warnings. The CPU estimator then deskews/transforms points, builds the GICP factors, updates the smoother, and updates frame pose/velocity/bias from the smoother estimate. Its `frames updated` TRACE line follows those state updates. Stationary geometry may affect estimate quality, but it does not by itself prove a callback or processing failure.
4. GLIM's ROS timer polls estimator results. With the existing empty extension list and disabled mapping, it does not publish ROS odometry or consume results into a trajectory output.

#### Least-invasive observation method

Use the supported ROS parameter `debug:=true` for the one proposed diagnostic run. In this exact GLIM ROS 2 revision it raises the logger to TRACE and emits:

- `points: sec.nanosec` for callback entry;
- `failed to extract points from message` or subsequent preprocessing traces, distinguishing failed conversion from successful extraction;
- `preprocessing input: ... points` and `preprocessed: ... -> ... points`;
- CPU logger `insert_frame points=... times=... ~ ...`, proving the preprocessed scan reached CPU odometry;
- `waiting for IMU data ...` / queue warnings, if applicable;
- `frames updated` after a subsequent scan's smoother state update.

The callback's `workload=` is only a queued-work count; it is not evidence that odometry completed. `frames updated`, corroborated by the source path that calculates and stores the smoother estimates, is the intended evidence of internal per-scan state production. It does not expose a numeric ROS odometry message.

The optional `librviz_viewer.so` is not a safe diagnostic output for this system: its ROS viewer registers odometry callbacks but also owns a TF broadcaster that publishes GLIM's `map -> odom` and `odom -> base_frame` transforms. Setting `publish_imu2lidar=false` disables only the IMU-to-LiDAR TF. Clearpath already owns `odom -> base_link`; do not load this extension or introduce a competing TF publisher. No safe viewer TF-isolation setting was established, and none is needed for the TRACE-based test.

The debug parameter writes an additional hard-coded `/tmp/glim_log.log`. It was absent during this preflight. The existing Mode A `config_logging.json` writes/rotates module logs under `/tmp`, where `/tmp/glim_odom.log` already exists. To protect that file, a separate private configuration was prepared at `/home/feras/glim_ws/configs/mode_a_diagnostic/`; it is a copy of Mode A whose only difference is `config_logging.json`'s `log_dir`, redirected to `/home/feras/glim_ws/runs/mode_a/run_20261117T122103Z_task12e_diagnostic/`. The validated baseline `/home/feras/glim_ws/configs/mode_a_zero_time/` is unchanged. Before an authorized launch, recheck that `/tmp/glim_log.log` is still absent; if not, do not start GLIM. After an authorized run, preserve the debug log in the private run directory without overwriting existing files.

#### Static validation and runtime gate

- **PASS:** All 15 diagnostic config files match the Mode A baseline byte-for-byte except `config_logging.json`; only its `log_dir` value differs. That JSON parses, points to the unique private run directory, and preserves all other logging settings.
- **PASS:** Existing `/tmp/glim_odom.log` remains present and unchanged by this static phase; `/tmp/glim_log.log` was absent at inspection.
- **NOT VERIFIED at the end of the static phase:** Cloud callback execution, conversion, preprocessing, CPU frame insertion, per-scan smoother update, and odometry quality.
- **PASS (prior Task 12D only):** The simulation publisher rates, GLIM subscriptions, QoS compatibility, sensor timestamps, TF, and initial IMU-state estimation were verified as documented in SLAM-002D.
- **NOT DONE:** No viewer/output extension, GLIM source/build change, sensor/robot/EKF/TF change, mapping, RViz, movement command, Mode B run, commit, or push.

#### Separately approved runtime verification

The user separately approved the existing combined simulation/input launch and then one GLIM diagnostic. The simulation used `/home/feras/clearpath_lidar11b_test` with `generate:=false`, the existing Helios bridge, and one existing timing adapter. No Clearpath generator, movement command, RViz, viewer extension, mapping, or Mode B was used.

Before GLIM started, the original and adapted cloud topics were active at approximately 10 Hz, IMU at approximately 100 Hz, and `/clock` ranged approximately 282–339 Hz in bounded windows. Endpoint QoS was compatible: cloud RELIABLE publisher to BEST_EFFORT adapter subscription; adapted cloud BEST_EFFORT publisher; IMU RELIABLE publisher to BEST_EFFORT reader. Parallel rate probes briefly showed multi-second scheduling gaps. During the GLIM run, callback timestamps advanced by 117.900 simulation seconds for LiDAR and 119.190 seconds for IMU.

The single stationary Mode A CPU run used the diagnostic config, `use_sim_time:=true`, and `debug:=true`. GLIM started at 13:32:44.002 and received SIGINT at 13:34:55.285. No robot motion was commanded.

| Processing stage | Status | Evidence |
|---|---|---|
| PointCloud2 publisher active | PASS | Both cloud topics and the IMU were active before the GLIM approval gate; live endpoint/QoS inspection and rate samples succeeded. |
| GLIM cloud callback executed | PASS | 1,180 `points:` TRACE entries; cloud stamps increased from 222.402 to 340.302. |
| PointCloud2 conversion succeeded | PASS | 1,180 preprocessing-input entries followed the cloud callbacks; no `failed to extract points` warning. |
| Cloud preprocessing succeeded | PASS | 1,180 outputs, each `5760 -> 5484 points`; no invalid-cloud skip or GLIM warning/error. |
| CPU odometry received scans | PASS | 1,180 CPU `insert_frame points=5484 times=0 ~ 0` traces. |
| IMU initialization succeeded | PASS | One `initial IMU state estimation result`; the initial scan wait ended after the configured initialization window. |
| Odometry estimate produced | PASS | 1,149 `frames updated` traces after initialization. In the tagged source this trace follows smoother estimates being calculated and written into frame pose/velocity/bias. The initial-state scan has its separate initialization path. |
| Trajectory or odometry observable | PARTIAL | Internal estimate updates are observable in TRACE logs, but this configuration publishes no ROS odometry topic and saves no trajectory/dump. |
| Conflicting TF publishers avoided | PASS | No viewer/output extension was loaded; mapping and `publish_imu2lidar` remained disabled. |
| Protected files preserved | PASS | The diagnostic copy was used; Mode A baseline and `/tmp/glim_odom.log` remained unchanged. |

Across the captured run, 1,180 clouds spanned 117.900 simulation seconds (about 10 Hz); 11,920 IMU callbacks spanned 119.190 seconds (about 100 Hz). Thirty scans waited for the initial IMU-state estimate; initialization then succeeded, followed by 1,149 per-scan state updates. No GLIM conversion failure, invalid-cloud skip, IMU-data wait, warning, error, or exception was found in the captured logs. One in-run resource sample showed 23.5% CPU and 176,232 KiB RSS; available memory before GLIM was 3.1 GiB. This is a single sample, not a peak or benchmark. The `values: 93` line in SLAM-002D was an optimizer summary, not a scan counter.

The run ended on SIGINT with exit status 0. The expected adapter `KeyboardInterrupt` occurred during SIGINT shutdown. The bridge stopped cleanly; after the Gazebo launch exited, its remaining server/GUI child processes were verified as belonging to this run and stopped. No task-owned simulation, bridge, adapter, or GLIM process remained. GLIM's hard-coded `/tmp/glim_log.log` was absent before launch and was moved into the private run directory afterward; the pre-existing `/tmp/glim_odom.log` remained 460 bytes with its original timestamp. No `glim_dump` was created, as expected with mapping/extensions disabled.

**Conclusion:** The success criterion is met for actual scan receipt, conversion, preprocessing, CPU odometry insertion, and internal odometry-state updates. This does not establish trajectory accuracy or provide a ROS odometry output. Sanitized runtime measurements are in [SLAM-002E runtime evidence](../evidence/command_outputs/2026-11-17-slam-002e-runtime-diagnostic.txt); full logs remain in `/home/feras/glim_ws/runs/mode_a/run_20261117T122103Z_task12e_diagnostic/`. Static findings and config validation remain in [SLAM-002E static evidence](../evidence/command_outputs/2026-11-17-slam-002e-static-diagnosis.txt).
