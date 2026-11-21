# 08. SLAM Integration

**Current status (2026-11-21):** SLAM-003E completed user-driven GLIM Mode A CPU mapping, graceful native saving and read-only native reload: 9,579 frames, one submap and 7,203 finite points. Four valid trajectories show approximately 4.58 m net estimated displacement. This is a functional moving-map result with deviations: actual Teleop commands reached 1.0 m/s and 0.5 rad/s instead of the intended 0.10 limits, and three IMU-prediction warning blocks occurred. A verified upload archive is ready and task-owned processes are stopped. Low-speed control, mapping accuracy, loop closure and Mode B remain unverified. Earlier milestones retain their historical status.

### 1. Objective

Prepare a project-owned PointCloud2 adapter for a later GLIM interface test. The adapter appends a recognized per-point time field containing zero seconds for every point. This is an instantaneous/global-shutter simulation approximation; it does not model a spinning LiDAR's acquisition timing or motion distortion.

SLAM-001A statically prepared the adapter. SLAM-001B verified live adapter publication without GLIM; SLAM-002E subsequently verified actual GLIM Mode A processing. SLAM-003A prepares mapping configuration only, as recorded in section 15.

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

Correction established by SLAM-002E: these odometry-only configs run GLIM internally but do not save an odometry dump merely because `dump_path` is supplied. Native saving needs a global-mapping instance or an extension save consumer. Because mapping and ROS viewer extensions are disabled, do **not** expect GLIM map or `~/odom` ROS output topics from these configs. No GLIM or ROS output was launched or observed during SLAM-002C.

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

### 15. SLAM-003A — Prepare GLIM Mode A CPU Mapping Configuration

**Date:** 2026-11-17. **Result:** PASS for static preparation only. No Gazebo, GLIM, RViz, generator, build, installation or runtime validator was run. No movement command, commit or push was issued.

#### Starting point and inspected saving mechanism

SLAM-002E is the verified processing baseline: 1,180 cloud callbacks/conversions/preprocessing outputs/CPU insertions and 1,149 subsequent state updates. This task does not repeat that experiment or establish mapping success.

Before configuration changes, inspected installed GLIM/core ROS package versions (both `1.2.2-0noble`), `/opt/ros/jazzy/share/glim/config/` CPU mapping defaults and their parameter descriptions, installed mapping/ROS headers, and installed library symbols/embedded export filenames. Cross-checked behavior against upstream **v1.2.2**, rather than the earlier floating-source revisions. This corroborates API availability, not exact source provenance of the PPA binaries.

The standalone `glim_rosnode` handles shutdown by waiting for estimator/mapping queues, then calling `save(dump_path)`. `GlimROS::save` invokes the global mapper's save method without requiring a viewer extension. Local mapping passes marginalized odometry frames into submaps; global mapping consumes those submaps. End-of-sequence processing can flush an incomplete submap at shutdown, so a stationary run need not reach 15 distinct keyframes before saving. This remains a source-supported expectation, not a runtime result.

Native save references:

- [Standalone shutdown and dump_path](https://github.com/koide3/glim_ros2/blob/v1.2.2/src/glim_rosnode.cpp)
- [ROS module selection, result forwarding, wait and save](https://github.com/koide3/glim_ros2/blob/v1.2.2/src/glim_ros/glim_ros.cpp)
- [Submap end-of-sequence flush](https://github.com/koide3/glim/blob/v1.2.2/src/glim/mapping/sub_mapping.cpp), [asynchronous queue drain](https://github.com/koide3/glim/blob/v1.2.2/src/glim/mapping/async_sub_mapping.cpp)
- [Native graph and trajectory saving](https://github.com/koide3/glim/blob/v1.2.2/src/glim/mapping/global_mapping.cpp), [submap saving](https://github.com/koide3/glim/blob/v1.2.2/src/glim/mapping/sub_map.cpp)

#### Isolated files and exact changes

Operational configuration: `/home/feras/glim_ws/configs/mode_a_cpu_mapping/`, copied from `mode_a_zero_time` (15 files, directory 0700/files 0600). A portable snapshot is in [`configs/glim/mode_a_cpu_mapping/`](../configs/glim/mode_a_cpu_mapping). Following the portability review, its only difference from the operational copy is `logging.log_dir="logs/glim"`, resolved relative to the process working directory. The original byte-identical snapshot was preserved outside Git at `/home/feras/glim_ws/evidence/slam003a_initial_repository_snapshot`. Retained inactive GPU/viewer example files are inherited copies; active references select CPU configs and no extensions.

Only these three values differ from the validated Mode A baseline:

| File / key | Baseline | Mapping configuration |
|---|---|---|
| `config_ros.json`: `glim_ros.enable_local_mapping` | false | true |
| `config_ros.json`: `glim_ros.enable_global_mapping` | false | true |
| `config_logging.json`: `logging.log_dir` | `/tmp` | `/home/feras/glim_ws/runs/mode_a_mapping/slam003b_001/logs/glim` |

The other 13 files are byte-identical to Mode A. Both CPU mapping files also match the installed defaults byte-for-byte:

| Component | Selected library / principal options |
|---|---|
| Odometry | `libodometry_estimation_cpu.so`; existing LOOSE/3 s initialization and two threads |
| Local/submapping | `libsub_mapping.so`; CPU `VGICP`, IMU enabled, local optimization **false**, OVERLAP strategy, at most 15 keyframes, minimum 500 points, overlap threshold 0.6, downsample resolution 0.3 m |
| Global mapping | `libglobal_mapping.so`; CPU `VGICP`, IMU and global optimization **true**, GICP between factors enabled, sampling rate 0.2, voxel resolution 0.5 m |

`sub_mapping.enable_optimization=false` is the installed CPU default: submaps are still created from odometry. It does not disable the local mapping module. No mapping tuning was introduced.

Preserved settings include adapted cloud `/a300_00000/sensors/helios_16p/points_glim`, IMU `/a300_00000/sensors/imu_0/data`, sensor-data QoS, frame names, zero timestamp offsets, `acc_scale=1`, and exact `T_lidar_imu=[0,0,-0.0475,0,0,0,1]`. Sensors/preprocessing/odometry files are unchanged. Mode A retains explicit zero relative seconds and `global_shutter_lidar=true`; this is not physical LiDAR timing.

`extension_modules=[]` and `publish_imu2lidar=false` remain unchanged. Mapping alone does not enable the viewer's ROS publishers/TF broadcaster. No live ROS map/odometry topic is expected. Clearpath/EKF retains its existing TF ownership.

#### Reserved outputs and expected native artifacts

Prepared a new owner-only run directory `/home/feras/glim_ws/runs/mode_a_mapping/slam003b_001/` containing:

- `run_manifest.json`: prepared/not-run status, package versions, config hashes, exact ROS parameters, output paths, simulation identity and expected artifacts.
- Empty `logs/glim/` and `logs/ros/` directories. Proposed console log: `logs/glim-console.log`, currently absent.
- Reserved native output path `glim_dump/`, deliberately **not created**. GLIM must create it during native save; an empty directory is not success evidence.

`config_logging.json` controls module log files. `ROS_LOG_DIR` and `dump_path` are separate runtime settings recorded in the manifest; they are not invented GLIM JSON keys. The proposed standalone run uses `debug=false` to avoid the hard-coded `/tmp/glim_log.log`, and `dump_on_unload=false` to avoid the destructor's separate `/tmp/dump` fallback. Standalone shutdown still calls native save.

If mapping succeeds, expect `graph.bin`, `values.bin`, `graph.txt`, `config/`, numbered submap directories such as `000000/` (metadata `data.txt`, `imu_rate.txt`, and compact point-cloud data), plus:

- `odom_lidar.txt`, `odom_imu.txt`: odometry-frame poses.
- `traj_lidar.txt`, `traj_imu.txt`: mapping-adjusted poses.

Trajectory rows use `timestamp x y z qx qy qz qw` (seconds/metres/quaternion). Native saving does not promise a merged PCD/PLY file or ROS map publication. Sensor trajectories are not automatically `base_link` trajectories or ground truth; their GLIM world/odometry origins must be aligned before later accuracy evaluation. `imu_rate.txt` content is conditional on saved frame data.

Never reuse this run once logs/artifacts exist. For a later run, create a fresh configuration copy with a new log destination, a new manifest and a new dump path; retain this configuration/snapshot as the prepared first-run record. ROS/simulator raw logs stay private; do not copy robot.yaml, domain/network values, full sensor payloads or bulk map data into Git.

#### Static validation results

- **PASS:** 15 JSON-with-comments files parsed per configuration tree, rejecting duplicate keys and non-finite constants; active references resolve.
- **PASS:** Exact change set is the two mapping booleans and logging directory; topic/time/QoS/extrinsic/TF safeguards are preserved.
- **PASS:** CPU libraries exist, export module factories and have no unresolved `ldd` dependencies in the sourced Jazzy/workspace environment. No `dlopen`, GLIM executable or validator was run.
- **PASS:** CPU mapping options match installed defaults; no active GPU factor type or viewer/output extension is selected.
- **PASS:** Before portability review the snapshot was byte-identical; the current portable snapshot differs from the operational copy only in `logging.log_dir`. All 15 operational manifest SHA256 values match; output log directories are empty and `glim_dump` is absent.
- **PASS:** Original Mode A/B/diagnostic trees, all five protected Clearpath setups, installed defaults and project sensor package match pre-task file-content hashes. Nested repositories remain clean; `main` and HEAD are unchanged.
- **NOT TESTED:** Startup/module loading, mapping/submap generation, native save, trajectory content, runtime TF isolation, mapping quality and performance. JSON/linkage checks do not establish runtime compatibility.

Evidence: [SLAM-003A static validation](../evidence/command_outputs/2026-11-17-slam-003a-static-validation.txt), superseded for repository logging-path equality by the [portable snapshot review](../evidence/command_outputs/2026-11-17-slam-003a-portable-review.txt).

#### Proposed SLAM-003B procedure — not executed; requires authorization

1. Recheck `codex-work`, protected hashes, installed versions, operational configuration/manifest hash agreement and the documented logging-only difference from the portable snapshot, empty reserved logs, absent dump/console output and available resources. Stop on any drift or occupied output path. No second GLIM, adapter or competing TF instance should be running.
2. In each runtime terminal, source Jazzy and the existing workspace; load the private test domain without printing its value. Set `ROS_LOG_DIR` to the reserved `logs/ros` path. Start only the existing combined setup with `setup_path:=/home/feras/clearpath_lidar11b_test generate:=false`, then its existing project LiDAR bridge and timing adapter. Do not launch RViz or a viewer, and do not regenerate files.
3. Before GLIM, inspect live cloud/IMU type, frame, nonempty tightly packed little-endian layout, zero point times, monotonic simulation stamps, QoS and rates (approximately 10/100 Hz), fixed sensor TF and existing EKF inputs. Verify no extra TF authority. Use bounded sequential probes to limit scheduling load; record simulation-time and wall-clock behavior separately.
4. Start one stationary headless GLIM instance using the command below. Require all three CPU module-load messages and IMU initialization. Observe up to 30 simulation seconds after initialization, with a 120-second wall-clock ceiling from GLIM startup. No motion is part of this smoke test. Stop on errors, non-finite states, unexpected TF, sustained input gaps/backlog or resource exhaustion; do not tune/retry automatically.
5. Send SIGINT to GLIM first while inputs remain available. Allow normal odometry/local/global queue draining and native save; monitor shutdown for up to 60 seconds. If it stalls, preserve logs and report the incomplete result before any forceful cleanup decision. Then stop the adapter, bridge and simulation; inspect task-owned leftovers individually.
6. Require exit status 0, loaded CPU modules, no processing/save exceptions, `graph.txt` reporting at least one submap and positive frame count, nonempty graph/value binaries, submap metadata and compact point data, saved configuration and all four nonempty trajectory files. Check eight numeric columns, finite values, increasing timestamps within each sensor trajectory, near-unit quaternions and plausible stationary poses. Record counts/ranges/resource samples and protected hashes. File existence alone does not pass validation.

Common terminal preparation (future only):

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export ROS_DOMAIN_ID="$(python3 -c 'import yaml; d=yaml.safe_load(open("/home/feras/clearpath_lidar11b_test/robot.yaml")); print(d["system"]["ros2"]["domain_id"])')"
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR=/home/feras/glim_ws/runs/mode_a_mapping/slam003b_001/logs/ros
```

Existing inputs, each in its own prepared terminal after authorization:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_lidar11b_test generate:=false
ros2 launch husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py
ros2 launch husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py
```

GLIM command in another prepared terminal, only after input checks:

```bash
mapping_run=/home/feras/glim_ws/runs/mode_a_mapping/slam003b_001
test ! -e "$mapping_run/glim_dump" || exit 1
test ! -e "$mapping_run/logs/glim-console.log" || exit 1
set -o noclobber
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_mode_a_mapping \
  -p use_sim_time:=true -p debug:=false -p dump_on_unload:=false \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_cpu_mapping \
  -p dump_path:="$mapping_run/glim_dump" \
  > "$mapping_run/logs/glim-console.log" 2>&1
mapping_exit=$?
printf 'GLIM exit status: %s\n' "$mapping_exit"
```

The observation/shutdown bounds above are operator/agent monitoring limits, not an automatic kill timer that could interrupt native saving. With OVERLAP keyframes a stationary submap may appear only on shutdown; this is a persistence smoke test, not multi-submap mapping, loop-closure or accuracy evaluation. No SLAM-003B action has been authorized or performed in SLAM-003A.

### 16. SLAM-003B — Stationary GLIM CPU Mapping Smoke Test

**Date:** 2026-11-17. **Result:** PASS for stationary CPU mapping, native persistence, numeric pose validation and cleanup. This is not an accuracy experiment or moving-robot mapping validation.

#### Checkpoint, authorization and preserved configuration

Reviewed all pending SLAM-003A changes; no unrelated files were found. Clarified the portable repository logging path in the documentation, then committed exactly 20 configuration/documentation/sanitized-evidence files on `codex-work`:

```text
136ca1d35404a7d4f0c361985ea2215b76e2d43e
SLAM-003A: Add validated GLIM CPU mapping configuration
```

The working tree was clean after this checkpoint. `main` remains `ca1140ef760bd6e28c32229347e75d896bfb38b4`; nothing was pushed or merged. Runtime results were not included in the checkpoint and remain uncommitted for review.

The user explicitly authorized Gazebo plus the existing inputs, then separately authorized one GLIM run after live input verification. Used the existing combined `/home/feras/clearpath_lidar11b_test` setup with `generate:=false`, existing Helios bridge and existing timing adapter. No generator, source/configuration change, build, package installation, robot command, RViz, viewer, Mode B or Nav2 was used.

Operational config: `/home/feras/glim_ws/configs/mode_a_cpu_mapping`. Private experiment: `/home/feras/glim_ws/runs/mode_a_mapping/slam003b_001`. The operational hash manifest matched; the portable repository configuration was not loaded. The actual GLIM invocation retained the command in section 15, including simulation time, `debug=false`, `dump_on_unload=false`, the operational absolute config path and unique private dump/log paths. Output files were created exclusively without overwriting existing run files.

All three CPU shared libraries loaded during preflight without calling factories, then actual GLIM startup logged `libodometry_estimation_cpu.so`, `libsub_mapping.so` and `libglobal_mapping.so`. Local/global mapping flags were true; local optimization retained the installed false default, while global optimization remained true. Sensor topics, QoS, timestamps, frames and exact `T_lidar_imu=[0,0,-0.0475,0,0,0,1]` were preserved. Extensions stayed empty and `publish_imu2lidar=false`.

#### Input verification and bounded observation

- Original/adapted clouds were approximately 9.998/9.999 Hz in simulation time; IMU was 100 Hz and `/clock` 333.3 Hz in the 12-second input check. All sensor/clock timestamps advanced monotonically.
- Adapted clouds were nonempty 360×16, little-endian and tightly packed (`point_step=36`, `row_step=12960`, 207,360 bytes). Five timestamp-matched pairs preserved original point bytes and all 28,800 sampled point times were zero.
- Original cloud and IMU publishers were RELIABLE/VOLATILE; adapted cloud was BEST_EFFORT/VOLATILE. GLIM's actual cloud/IMU BEST_EFFORT/VOLATILE subscriptions were observed and compatible.
- Sensor TF resolved with identity rotations and exact simulated heights: IMU 0.294759 m, LiDAR 0.342259 m above `base_link`. `odom → base_link` remained near identity.
- Two initial read-only inspection probes failed on unsupported rclpy publisher-GID access. A corrected probe using supported message/graph APIs passed; the simulation and sensor processes were not restarted. This was inspection-code failure, not sensor/GLIM failure.
- GLIM initialized the IMU after 3.524 observer wall seconds. The clock advanced from 270.897 to 300.897 s during the subsequent 30.000-second observation; the 120-second wall ceiling was not reached.
- Observation lasted 33.544 monotonic wall seconds from GLIM start; total through wrapper exit was 33.882 s. Console timestamps record config load at 18:21:50.841, IMU initialization at 18:21:54.046 and native save completion at 18:22:27.170.
- The independent observer received 347 clouds and 3,542 IMUs over its full discovery/run/shutdown capture. These are observer counts, not GLIM callback counters: TRACE was intentionally disabled. Actual retained mapping frames are established by saved structures below.

#### Mapping and native artifact validation

SIGINT was sent to the identified GLIM node first while sensor inputs remained active. GLIM drained odometry, local and global mapping queues, serialized the output, logged `saved`, and exited with status **0**. No forced GLIM termination was needed. The stationary partial submap was flushed at end of sequence; no motion was needed to produce valid structures.

`graph.txt` reports `num_submaps=1`, `num_all_frames=300`, `num_matching_cost_factors=0`. Submap metadata and each main trajectory independently contain 300 frames. The single submap `000000` contains 2,648 finite points, with matching finite normal/intensity/covariance arrays; all 604 stored rigid-transform matrices checked were finite and coherent. Zero inter-submap matching factors is expected here and does not demonstrate loop closure.

All artifacts below exist under `glim_dump/`; sizes are bytes:

| Artifact | Size | Content validation |
|---|---:|---|
| `graph.bin` | 1,463 | Nonempty Boost archive header; full deserialization/reload NOT TESTED |
| `values.bin` | 573 | Nonempty Boost archive header; full deserialization/reload NOT TESTED |
| `graph.txt` | 64 | Positive submap/frame counts match metadata and trajectories |
| `odom_lidar.txt` | 23,987 | 300 valid pose rows |
| `traj_lidar.txt` | 23,987 | 300 valid pose rows |
| `odom_imu.txt` | 24,287 | 300 valid pose rows |
| `traj_imu.txt` | 24,287 | 300 valid pose rows |
| `000000/data.txt` | 162,060 | Parsed frame metadata and rigid transforms |
| `000000/imu_rate.txt` | 267,166 | 3,300 finite poses; boundary timestamp repeats described below |
| `000000/points_compact.bin` | 31,776 | 2,648 finite XYZ float32 triples |
| `000000/normals_compact.bin` | 31,776 | Matching finite, near-unit normal triples |
| `000000/intensities_compact.bin` | 10,592 | 2,648 finite float32 intensities |
| `000000/covs_compact.bin` | 63,552 | Matching six-float symmetric covariance records; positive eigenvalues |
| `config/config.json` | 573 | Valid JSON; resolved config path and detected frame metadata |
| `config/config_ros.json` | 911 | Parsed settings match operational config |
| `config/config_sensors.json` | 1,039 | Parsed settings match operational config |
| `config/config_preprocess.json` | 589 | Parsed settings match operational config |
| `config/config_odometry_cpu.json` | 744 | Parsed settings match operational config |
| `config/config_sub_mapping_cpu.json` | 711 | Parsed settings match operational config |
| `config/config_global_mapping_cpu.json` | 619 | Parsed settings match operational config |
| `config/config_logging.json` | 202 | Parsed settings match operational config |
| `config/config_viewer.json` | 706 | Parsed settings match operational config; viewer remains disabled |

Total native output: **22 files, 671,664 bytes**. Binary compact layouts were checked against [GTSAM Points v1.2.2 saving code](https://github.com/koide3/gtsam_points/blob/v1.2.2/src/gtsam_points/types/point_cloud.cpp). This verifies numerical contents and dimensions, not full native dump reload. No merged PCD/PLY or ROS map/odometry output was produced or required.

The four main trajectories contain eight finite columns (`timestamp x y z qx qy qz qw`), strictly increasing timestamps from 270.801 to 300.702 s, and 0.099–0.102 s intervals. Quaternion norms are within approximately 2.1×10⁻¹⁰ of one. Odometry and mapping-adjusted files are byte-identical per sensor in this single-submap run. Maximum displacement from the first pose is about 1.0543 mm; this is a stationary numerical observation, not accuracy against ground truth. GLIM sensor-frame poses are not base-link poses.

`imu_rate.txt` contains 3,300 finite, near-unit-quaternion poses with nondecreasing timestamps. Its 299 adjacent repeated timestamps occur exactly at joins between 11-row scan segments; none decrease. Downstream consumers may require scan-boundary deduplication. Saved `config.json` resolves the absolute operational `config_path` and adds detected `imu_0_link`/`helios_16p_link` metadata; other references match the operational input.

#### TF, resources, warnings and cleanup

Namespaced TF publisher sets were identical before, during and after GLIM: four existing dynamic publishers and two existing static publishers. GLIM appeared in none; root `/tf` and `/tf_static` had no publishers. Existing EKF/controller TF settings were unchanged. Captured transforms had no conflicting parents. This checks that GLIM added no competing TF authority; it is not an exhaustive upstream per-transform duplicate-authority audit.

Thirty-four approximately one-second samples measured GLIM at 19.96–24.98% of one logical CPU (mean 21.99%) and 89.09–179.23 MiB RSS. System available memory stayed at 2,775.67–2,883.95 MiB with zero swap use. These are observation samples, not benchmark results or guaranteed save-time peaks. Maximum observed cloud/IMU wall gaps were 0.119/0.020 s. No memory-pressure abort, processing exception, severe backlog/input stall or EKF missed-update message was observed.

GLIM logged four absent-option/default pairs from the installed CPU config set: `submap_target_num_points=-1`, `submap_voxel_resolution_max=0.5`, `submap_voxel_resolution_dmin=5`, and `submap_voxel_resolution_dmax=20`. These defaults did not prevent mapping; no tuning/restart was attempted. Gazebo/Clearpath retained known missing-joystick, IMU hardware-info, controller-period, SDF `gz_frame_id`/gravity, rendering/remapping and idle-braking warnings.

After GLIM saved, stopped the adapter, bridge and simulation with SIGINT. The adapter had its known `KeyboardInterrupt`/exit -2; the bridge exited cleanly; the Gazebo wrapper reported signal-related exit -2 while server/render threads shut down. No SIGTERM escalation was needed. All launch sessions exited, and an independent host process check found no remaining task-owned runtime processes.

Post-run hashes confirm **187 files across 11 protected roots unchanged**, including all five Clearpath setups, the three validated GLIM modes, operational mapping config, installed defaults and project sensor source. Nested repositories and `main` remain unchanged. Private raw logs, helper scripts, JSON observations/validation results and map payloads remain in the experiment directory; no private domain/network values or sensor payloads were added to Git.

Evidence: [sanitized SLAM-003B runtime validation](../evidence/command_outputs/2026-11-17-slam-003b-runtime-validation.txt). Runtime documentation/evidence remain uncommitted. Recommended next step: review these results and separately verify native dump reload before any explicitly authorized movement/multi-submap experiment. No further experiment was started.

### 17. SLAM-003C-B — Headless Native Dump Reload Validation

**Date:** 2026-11-17. **Updated status:** PASS — native graph/value and submap/compact-cloud loading are now verified for the stationary SLAM-003B dump. This closes the native-reload limitation recorded at the earlier SLAM-003B milestone. Accuracy, motion mapping, loop closure and Mode B remain outside scope.

SLAM-003C inspected installed `offline_viewer`/`map_editor` arguments and found no headless CLI path; no GUI was launched. SLAM-003C-B explicitly authorized a minimal standalone C++ caller of the installed GLIM 1.2.2 APIs. It is a validation utility, not a ROS package, parser or SLAM component: [source/build instructions](../tools/glim_dump_reload/README.md), [C++ caller](../tools/glim_dump_reload/validate_dump.cpp).

#### Inspection and read-only execution

Before code creation, verified `codex-work`, checkpoint `136ca1d35404a7d4f0c361985ea2215b76e2d43e`, existing SLAM-003B changes and all 22 original artifact hashes. Existing documentation/evidence changes were copied byte-for-byte to the new private validation directory. These existing records are preserved; only this section and a development-log entry are appended after successful validation.

Inspected installed GlobalMapping/SubMap/callback/configuration/logging headers, exported targets, and native `ISAM2ExtDummy`, and cross-checked the v1.2.2 `GlobalMapping::load`, `SubMap::load`, `PointCloudCPU::load`, configuration and logging implementations. Native loading reads the saved graph/values and compact submaps, rebuilds in-memory voxel maps and updates its backend. Native errors can be caught and recovery attempted, so a true load return alone is insufficient. Module logging can otherwise append existing files.

The caller therefore registers console-only `glim`/`global` loggers before native initialization and uses `GlobalConfig::instance(dump + "/config")`. It sets only the in-memory `GlobalMappingParams.enable_optimization=false`, selecting the installed `ISAM2ExtDummy` backend. The pre-update callback verifies that exact backend; its update stores deserialized factors/values without nonlinear optimization. The native informational line `optimize` is still emitted by the load call site and does not mean optimization was enabled. No saved configuration is edited, no async worker/viewer/ROS node is instantiated, and no save/configuration-dump method is called.

Additionally, a fail-closed Landlock ruleset is installed before GLIM object construction. ABI 7 was available; write-open, truncation, creation, removal and rename/link operations are denied. A denied creation probe confirms enforcement. New stdout/stderr captures are opened exclusively by the parent before execution; the validator itself uses inherited streams only. No input permissions were changed. There were no errors or corruption/recovery diagnostics during loading.

#### Separate build and one bounded reload

Project source: `tools/glim_dump_reload/`; build/results: `/home/feras/glim_ws/validation/slam003c_b_001/`. CMake consumes installed `glim::glim` and its already-installed dependencies, with C++17. No colcon/package rebuild, new dependency, installation or GLIM source modification was needed.

The installed config under `/opt/ros/jazzy/share/glim/cmake` initially failed because it includes a missing `glim-targets.cmake`. The intact installed export under `/opt/ros/jazzy/share/glim` resolved this packaging issue without editing installed files. The corrected standalone build succeeded. This was a build-configuration correction; the dump reload itself ran exactly once.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S /home/feras/clearpath_ws/tools/glim_dump_reload \
  -B /home/feras/glim_ws/validation/slam003c_b_001/build \
  -DCMAKE_BUILD_TYPE=Release \
  -Dglim_DIR=/opt/ros/jazzy/share/glim
cmake --build /home/feras/glim_ws/validation/slam003c_b_001/build --parallel 2
```

With working directory `/home/feras/glim_ws/validation/slam003c_b_001`, `OMP_NUM_THREADS=2` and exclusive stdout/stderr capture by the parent process:

```bash
timeout --signal=TERM --kill-after=5s 30s \
  /home/feras/glim_ws/validation/slam003c_b_001/build/glim_dump_reload \
  /home/feras/glim_ws/runs/mode_a_mapping/slam003b_001/glim_dump \
  1 300 2648
```

The single call returned **0** in approximately **0.0134 wall seconds**, without timeout or restart. Linkage inspection found no missing dependencies. Build output and full execution/hash records remain private; no existing run output was reused or overwritten.

#### Results and integrity

| Check | Result |
|---|---|
| Native `GlobalMapping::load` | PASS — returned true |
| Native graph/value deserialization | PASS — 4 non-null factors and 4 values observed through the native pre-update callback; factor keys resolve |
| Native `SubMap::load` / compact-cloud loading | PASS — reached through global loading, without a custom parser |
| Loaded submaps / retained frames / points | PASS — 1 / 300 / 2,648 |
| Loaded frame stamps and poses | PASS — finite; stamps strictly increasing |
| Loaded normals, covariances, intensities and points | PASS — attributes present and finite |
| Post-load submap pose/update | PASS — native callback received expected count and finite poses |
| Native in-memory `export_points()` | PASS — 2,648 points; no file export |
| Native errors / recovery | PASS — 0 errors; no corruption/recovery detected |
| Optimization / filesystem writes | Disabled in memory / Landlock write denial verified |
| Original dump integrity | PASS — all 22 file hashes, sizes and path set unchanged |

Three optional global-mapping settings emitted missing-option/default pairs (six warning lines): `submap_voxel_resolution_max=0.5`, `submap_voxel_resolution_dmin=5`, `submap_voxel_resolution_dmax=20`. No corrupted-file, missing-library or deserialization exception appeared.

The four trajectory text files are not consumed by the native map loader. Their hashes are unchanged, so the previous SLAM-003B text-trajectory validation remains applicable; this task does not claim a separate trajectory import or accuracy result. The 187 protected configuration/source contents, original SLAM-003B evidence, `main` and checkpoint remain unchanged.

Evidence: [SLAM-003C-B native reload result](../evidence/command_outputs/2026-11-17-slam-003c-b-native-reload.txt). Only appended documentation and the new utility/evidence are pending review; no commit, stash, reset, clean, push, simulation, live mapping, GUI, movement or next experiment was performed.

### 18. SLAM-003D-A — Controlled Moving Experiment Preparation

**Result:** PASS for static preparation only; no Gazebo, GLIM, recorder, reference bridge or velocity publisher was launched. No robot movement or new native reload execution. Three separate new authorizations are required for the proposed runtime.

Reviewed exactly seven pending project-owned SLAM-003B/C-B files (two documents, two sanitized evidence files, standalone validator source/CMake/README); no unrelated or private runtime files. Created the single authorized local checkpoint `60c39e30bf50b72b62d2e9c51988482790685c3f`, `SLAM-003B-C: Validate GLIM mapping and native dump reload`. Working tree was clean immediately afterward. The SLAM-003A commit remains its parent; `main` is unchanged. No push/merge. Historical statements about pending review above describe the earlier milestones.

Prepared [route/recording plan and configuration](../configs/experiments/slam003d/README.md), a separate [portable moving configuration](../configs/glim/mode_a_cpu_mapping_moving/), and the [SLAM-003D-B procedure with exact commands](experiments/slam003d_b_procedure.md). Operational configuration is `/home/feras/glim_ws/configs/mode_a_cpu_mapping_slam003d_001`; reserved run is `/home/feras/glim_ws/runs/mode_a_mapping/slam003d_001`. Its dump and bag paths do not yet exist. Only `logging.log_dir` differs from the stationary operational mapping config; 14 of 15 files remain byte-identical. All 15 portable files equal the committed portable CPU mapping baseline; runtime explicitly selects the new operational copy.

The working stamped mux interface is `/a300_00000/cmd_vel` (`geometry_msgs/msg/TwistStamped`), not a new controller. Proposed route: at least 10 simulation seconds stationary for initialization, forward 8 s at 0.15 m/s, zero 2 s, left turn 8 s at 0.20 rad/s, zero 2 s, forward 8 s at 0.15 m/s, final zero/stationary 8 s. Commands run at 20 Hz with refreshed simulation stamps. Nominal duration 46 s before discovery/approval delays; nominal commanded travel 2.4 m and yaw 1.6 rad. Actual motion remains untested. Route clearance must be established live before authorization/execution.

Historical Gazebo evidence verifies `gz.msgs.Pose_V` on `/model/a300_00000/robot/pose`, including the world-to-model pose and model-relative base pose. Installed PosePublisher and source semantics support an independent simulator reference; current-run availability must be rechecked after authorization. A prepared one-way native bridge records individual stamped transforms on `/a300_00000/reference/gazebo_pose`, never the live TF topics. Filtered and raw wheel odometry are recorded as dependent wheel-based estimates, not ground truth. The procedure defines sensor-to-base conversion and origin/time alignment explicitly.

One rosbag2 recorder is planned for 11 explicit topics: both clouds, IMU, raw/filtered odometry, clock, mux command input/output, reference poses and namespaced dynamic/static TF. BEST_EFFORT sensor subscriptions preserve publisher settings; static TF uses transient-local durability. An 8 MiB cache per buffer bounds recorder memory. No all-topic recording or extra cloud-rate processes.

Safety procedure specifies three separate approval gates; zero commands and measured-stop verification between stages; per-command, route, observation and saving bounds; sensor/clock gaps, resource, recording, processing and TF abort criteria; graceful native saving and recorder flush; actual artifact/trajectory/retained-frame validation and one bounded native reload of the new dump. There is no prescribed submap or loop-closure count. All task-owned processes must stop and protected hashes must match afterward.

Static checks passed: 30 JSON-with-comments files, native rosbag2 parsing of 11 QoS overrides, bridge schema and exported converter, CPU module references and RTLD_NOW dependency resolution with Jazzy sourced, preserved mapping/extrinsic/TF settings, logging-only semantic delta, route arithmetic and `bash -n` for all command blocks. Protected comparisons preserve 187 files, all 22 stationary dump files and 41 native reload build/result files. New preparation remains uncommitted.

Evidence: [SLAM-003D-A static preparation](../evidence/command_outputs/2026-11-17-slam-003d-a-preparation.txt). Stop here; SLAM-003D-B requires review and new authorizations.

### 19. SLAM-003D-B — Controlled Motion Attempt Stopped at Clearance Preflight

**Result: STOPPED BEFORE GLIM OR MOVEMENT.** The requested moving-mapping experiment was not completed. Only the separately authorized simulation, existing sensor pipeline, dedicated reference bridge and stationary recording ran. No GLIM or movement approval was requested, no velocity publisher was created, and no alternate route, spawn or configuration was substituted.

Started on `codex-work` at preserved checkpoint `60c39e30bf50b72b62d2e9c51988482790685c3f`; protected `main` remained unchanged. Preflight verified the prepared configuration, unused outputs, 187 protected files, 22 original stationary dump artifacts and 41 native-reload build/result files. No conflicting host runtime processes. Initial available RAM 5,324.6 MiB and free disk 945.9 GiB. The reviewed [procedure](experiments/slam003d_b_procedure.md), route, portable files and operational configuration were preserved.

#### Clearance blocker and decision

The actual warehouse scene includes `Person 1 - Standing` / `MaleVisitorOnPhone` at `(1, -1, 0)` m. Finite live LiDAR points near that location enter the required clearance envelope during the proposed in-place turn centered nominally at `(1.2, 0)` m. A point at approximately `(1.13704, -0.97501, 0.21122)` m in the stationary base frame lies about **0.486875 m** from the projected installed chassis collision footprint at nominal turn yaw **1.104 rad**. This is below the reviewed **0.5 m** margin.

The check used the convex footprint of installed `chassis_collision.stl`, finite points in the chassis-height band and the reviewed nominal turn sweep; it did not move the robot. This is a conservative projected clearance check, not a full 3D collision prediction or semantic identification of every point. It cannot certify the required margin, especially with motion/slip uncertainty and other robot parts. The correct outcome is to stop for route review, not reduce the margin or silently change the route. No turn or forward segment was attempted.

#### Authorized stationary inputs, reference and recording

Used the exact existing combined setup with `generate:=false`, Helios bridge and timing adapter. Input preflight measured original/adapted clouds about **10.001/10.002 Hz**, IMU **99.991 Hz** and clock **333.333 Hz**. Five header-matched cloud pairs preserved all original point bytes and zero times. Adapted clouds remained nonempty, little-endian and tightly packed. Fixed sensor TF exactly matched base-to-IMU Z=0.294759 m and base-to-LiDAR Z=0.342259 m, identity rotations; no conflicting TF parents were observed.

Live Gazebo inspection confirmed `gz.msgs.Pose_V` on the prepared reference topic. The prepared native one-way bridge worked on `/a300_00000/reference/gazebo_pose`, separate from TF. Offline native rosbag2/CDR reading verified **4,586** messages with same-stamp `warehouse → a300_00000/robot → a300_00000/robot/base_link` poses, finite transforms and strictly increasing simulation timestamps. The model/base transform was identity throughout this stationary capture. This confirms independent reference availability; wheel odometry was not substituted as ground truth.

The unchanged 11-topic MCAP recorder subscribed to all requested topics, flushed on SIGINT and exited **0**. The bag contains **63,613 messages over 96.516 simulation seconds**, from receipt simulation time 171.102 to 267.618 s. Recorder wrapper lifetime was approximately 110.094 wall seconds including startup/flush; the 90-second GLIM observation cap was never applicable because GLIM was not started. This stationary preflight capture is not the planned 46-second moving dataset. It contains zero input/output command messages, as expected without a command publisher.

| Topic suffix under `/a300_00000` unless absolute | Messages |
|---|---:|
| `sensors/helios_16p/points` | 963 |
| `sensors/helios_16p/points_glim` | 957 |
| `sensors/imu_0/data` | 9,652 |
| `platform/odom` | 4,581 |
| `platform/odom/filtered` | 4,587 |
| `reference/gazebo_pose` | 4,586 |
| `/clock` | 32,170 |
| `tf` / `tf_static` | 6,115 / 2 |
| `cmd_vel` / `platform/cmd_vel` | 0 / 0 |

Offline validation read all messages and matched metadata counts. All recorded clouds passed layout and zero-time checks; five additional matched pairs preserved original bytes. Recorded cloud rates were about 9.990/9.979 Hz and IMU 100.000 Hz. Maximum cloud stamp gaps were 0.198/0.300 s; IMU 0.012 s. Reference stamps span 171.108–267.603 s with maximum gap 0.231 s. No decreasing sensor/reference stamps. Mixed TF/static messages can repeat receipt stamps and are not sensor trajectories.

The recorder reported **three transport-layer message losses**, without per-topic attribution. Discovery/recording boundaries and gaps mean this is not a lossless evaluation dataset. A supplemental seven-second read-only probe collected 292 valid reference rows, but failed before its summary because a parameter-service future for `twist_mux` had not completed. Its code issued the request before confirming service discovery. No motion endpoint was exercised; live parameter verification remains incomplete. The failed probe was not used as successful evidence; the already-recorded bag supplied the completed reference validation. No simulator, input or recorder restart was made.

#### Results, resources and cleanup

Recorded reference displacement and yaw change are both **0** over the stationary interval. Raw/filtered odometry speeds remained effectively zero (below 1.2×10⁻¹⁸ m/s); tiny wheel-integration values are numerical noise, not movement. GLIM initialization, moving scans, map/submap/frame/point counts, four GLIM trajectories, native saving and new-map reload are **NOT RUN**, rather than mapping failures. `glim_dump` is absent; the old native validator was not executed or modified.

RAM observations are snapshots only: 5,324.6 MiB available before launch and 2,838.8 MiB before recorder stop; swap used zero. Continuous CPU/RSS samples were not preserved because the supplemental probe exited before writing its summary. No GLIM CPU/RSS measurement exists. No resource-pressure abort was observed; clearance was the stop reason.

All five task-owned launch wrappers exited 0 after SIGINT: recorder, reference bridge, adapter, Helios bridge and simulation. The adapter child retained its known `KeyboardInterrupt` / exit -2 shutdown diagnostic; wrapper 0 does not erase that child status. No forced termination was needed. Host process/group inspection found no remaining task-owned or conflicting robotics processes. Existing Gazebo warnings included unavailable joystick, IMU hardware-info association, controller-period mismatch, SDF/remapping and Collada notices; no EKF missed-update message was found.

Post-run checks preserve all **187 protected files, 22 SLAM-003B dump artifacts, 41 native-reload build/result files and the prepared operational configuration**. All 24 preexisting SLAM-003D-A project files matched their preflight hashes before these explicitly requested documentation updates. The prepared procedure/configurations/evidence remain unchanged; only current status and appended runtime records are added to the documentation. No commit, push, stash, reset, clean, generator, package change, Mode B, Nav2 or switching experiment.

Private dataset: `/home/feras/glim_ws/runs/mode_a_mapping/slam003d_001/bag/`:

- `bag_0.mcap`: **394,399,741 bytes**.
- `metadata.yaml`: **10,764 bytes**.

Full commands/launch timing, world/software identity, configuration hashes, reference rows, clearance data, failed-probe diagnostics, bag validation, raw logs and cleanup/preservation results remain in that private run. Sanitized [SLAM-003D-B preflight-abort evidence](../evidence/command_outputs/2026-11-17-slam-003d-b-preflight-abort.txt) contains the report without bulk data or private domain values.

**Next action:** review the clearance evidence and prepare a separately reviewed route with the required margin. Do not reuse this consumed bag/log reservation; a future authorized experiment needs fresh run/configuration paths. No route/spawn/margin change or second attempt was made here. Moving-robot GLIM mapping remains untested.

### 20. SLAM-003D-R — Static Safe-Route Redesign

**Decision: no route verified; no new experiment reservation or runtime.** The 0.5 m clearance requirement is retained. The earlier 0.486875 m chassis/LiDAR turn result remains unchanged; this review extends the inspection to all robot collision parts and installed world collision geometry.

Reviewed the original stationary bag, clearance evidence and prepared route, preserving all existing pending work. Native offline reading inspected all 957 adapted clouds: one unique XYZ geometry / 5,620 finite returns, not new viewpoints. Existing xacro was evaluated in memory without a Clearpath generator, resolving all 16 robot collision parts. Parsed 25 cached world models / 49 collision geometries, applying model/link/collision poses and scales. Full robot footprint is about 0.9954×0.6801 m, including wheels and bumpers; the point-robot and chassis-only approximations are insufficient.

The standing-person collision OBJ at `(1,-1,0)` m extends closer than the visible point surface. Full-height projected geometry gives an initial whole-robot margin of 0.493693 m. Checked the entire A/B/C sweeps, with exact convex translation-sweep cross-checks and a bound on sampled rotation gaps:

| Candidate | Proposed motion | Total nominal simulation time including 10 s initial / 8 s final holds | Minimum projected world clearance | Result |
|---|---|---:|---:|---|
| A | Forward 0.10 m/s × 3 s | 21 s | 0.327963 m | FAIL to certify 0.5 m |
| B | A; zero 2 s; left yaw 0.10 rad/s × 2 s | 25 s | 0.327963 m | FAIL to certify 0.5 m |
| C | Reverse 0.10 m/s × 3 s | 21 s | 0.493693 m at the initial pose | FAIL to certify 0.5 m |

These conservative projected bounds do not prove a 3D collision. Full-height projection includes upper-body overhang; a separate person-only 3D convex separation diagnostic gives approximately 0.449747 m for A/B and 0.583320 m for C at sampled poses. It is not a complete full-world continuous 3D certificate and is not used to override the failed retained projected margin. Cloud-only minima (about 0.618 m for A/B and 0.763 m for C) would miss the more restrictive collision-envelope evidence. Explicit position/yaw/geometry/tilt and discretization allowances further reduce conservative bounds; no unmeasured controller guarantee or reduced safety margin is assumed.

No candidate is selected. `slam003d_002` and a new operational configuration were **not created**. The existing run/configuration and historical runtime procedure remain unchanged. A justified continuous 3D envelope and tracking-error analysis would be needed to resolve whether reverse motion can be certified with the same 0.5 m physical requirement; there is no authorization for a changed metric, person relocation, new spawn or runtime test.

The independent reference bridge and explicit recording approach are retained as the reference design. The three earlier recorder losses remain a data-quality limitation. Any later approved recording must report loss totals/unknown attribution, native read counts versus metadata, per-topic stamp coverage/gaps and matched cloud headers over a common interval; discovery boundaries must be separated from proven transport loss. No QoS/recording architecture was silently changed.

See [full route review, candidate commands and uncertainty](experiments/slam003d_r_route_review.md), [geometry comparison figure](../evidence/figures/2026-11-17-slam-003d-r-route-comparison.png), [offline inspection utility](../tools/slam_route_clearance/README.md) and [sanitized evidence](../evidence/command_outputs/2026-11-17-slam-003d-r-static-route-review.txt). Detailed arrays/hashes remain private under `/home/feras/glim_ws/planning/slam003d_r_001`. Six analytic geometry checks and exact translation-sweep cross-checks passed; this does not certify runtime motion. Existing data, protected configurations and checkpoints are preserved; no commit, push, runtime, generator, installation or movement.

### 21. SLAM-003E — User-Driven Husky Mapping, Native Save and Upload Package

**Date:** 2026-11-21. **Result:** native moving-map creation/save/reload PASS **with deviations**; intended low-speed limits were not achieved. No autonomous route or assistant motion publisher was created.

The user selected spawn `x=-4,y=3,z=0.3,yaw=0`, explicitly authorized the existing warehouse simulation with `generate:=false rviz:=false`, and confirmed the GUI looked correct. The live model settled at `(-4,3,0.135969836)` m with identity orientation; both controllers were active and clock advanced. Existing LiDAR bridge and timing adapter then passed live verification: both clouds about 10.002 Hz, IMU 100.018 Hz, monotonic stamps, five matching byte-preserved zero-time pairs, compatible QoS and exact sensor TF. The combined Clearpath setup and installed files were unchanged.

After separate user approval, started node `glim_manual_mapping_001` using `/home/feras/glim_ws/configs/mode_a_cpu_mapping_manual_001`. This is a new private 15-file copy of validated `mode_a_cpu_mapping`; 14 files are byte-identical and only absolute `logging.log_dir` changed. CPU odometry/local/global mapping, sensor topics/QoS/timing, exact `T_lidar_imu=[0,0,-0.0475,0,0,0,1]`, `use_sim_time=true`, extensions `[]` and `publish_imu2lidar=false` were preserved. Runtime used `debug=false`, `dump_on_unload=false` and unique native output `/home/feras/glim_ws/runs/mode_a_mapping/slam003e_manual_001/glim_dump`.

All three CPU modules loaded and IMU initialization succeeded. The user confirmed configuring the existing Teleop panel and clear space, then separately approved manual driving. The assistant sent no nonzero or zero velocity commands. W/A/S/D were operated by the user through the existing bridge/mux; a read-only observer recorded sensor stamps, wheel odometry, commands, TF endpoint names and resources. After the user confirmed completion and authorized saving, observed velocities were near zero for at least two seconds, then SIGINT was sent to the identified GLIM node while sensor inputs remained active. All queues drained, native `saved` was logged and the process exited **0**.

#### Saved result and validation

- **9579 frames / 1 submap / 7203 finite points**. Compact points/normals/intensities/covariances have matching dimensions and finite values; nine saved JSON files parse successfully. Native output is 22 files totaling **17,286,200 bytes**.
- All four trajectory files contain 9,579 finite eight-column pose rows with strictly increasing simulation stamps **617.901–1575.702 s**, span **957.801 s**, intervals **0.099–0.102 s**, and near-unit quaternions. The long stationary setup/interaction prefix is part of this run.
- Main wheel-observed motion spans **1504.692–1547.826 s**, elapsed **43.134 simulation seconds** including stops. **432 saved trajectory rows** lie in that interval; output covers its boundaries. LiDAR trajectory net displacement is **4.583 m**, net yaw **−1.023727 rad** (about −58.65 degrees), numeric polyline length **19.789 m**. These are numerical pose observations, not ground-truth accuracy metrics.
- Filtered wheel odometry estimates **20.147 m** path length, **4.298 m** net displacement and **−1.072008 rad** net yaw. It is not independent ground truth. Only endpoint Gazebo poses were sampled; no complete reference trajectory or ROS bag was recorded. GLIM sensor poses must not be treated as `base_link` or automatically aligned to Gazebo world.
- One bounded execution of the unchanged native reload validator used actual arguments **`1 9579 7203`**. It returned PASS/exit 0 in **0.0824 s**, with 4 factors/4 values, all expected submaps/frames/points and zero errors/recovery. Landlock read-only protection remained active; every original native file hash was unchanged.
- `num_matching_cost_factors=0`. One submap is acceptable for this first moving experiment; no loop closure or map-accuracy result is claimed.

#### Deviations and limitations

**Speed limits were not enforced at the message level.** Although the user confirmed intended GUI values of 0.10 m/s and 0.10 rad/s, observed input/mux commands were ±1.0 m/s and ±0.5 rad/s. Wheel peaks were 1.047 m/s and 0.507 rad/s. The read-only monitor recorded the mismatch but did not stop it; the assistant did not catch it during the drive. This was not a validated conservative-speed experiment. Before any further driving, verify actual command messages with a separately approved brief user-operated check; do not assume displayed settings have taken effect. No configuration change or retry was made after this discovery.

Two brief nonzero input events were also observed during setup before explicit manual-drive approval; odometry was stationary at approval. Existing GUI bridge command headers were zero, so their observed simulation clock and valid odometry/sensor stamps were used for timing. The working control/bridge implementation was not changed. These observations do not attribute the GUI setting failure to a specific cause.

GLIM emitted **three IMU-prediction validation warning blocks**, including suggestions about extrinsic/bias accuracy. The cause was not diagnosed; the verified extrinsic was preserved. Optional mapping settings used installed defaults. No fatal GLIM processing error or restart occurred. Zero-time Mode A remains a synthetic global-shutter approximation rather than hardware scan timing fidelity.

Over **837** approximately one-second samples, GLIM used **18.90–59.81%** of one CPU core (mean **21.71%**) and **180.6–322.8 MiB RSS**. Available RAM stayed above **3059.3 MiB** with zero swap use. Observer sensor/clock stamps had no rewinds; maximum observed cloud/IMU wall gaps were about 0.135/0.024 s. These samples are not a benchmark. No GLIM-named TF publisher appeared and root TF topics had no publishers; a single sample contained transient unresolved DDS endpoint names, so no exhaustive per-transform authority claim is made.

#### Archive and cleanup

Prepared and verified **`/home/feras/glim_ws/runs/mode_a_mapping/slam003e_manual_001/glim_manual_mapping.tar.gz`**, **1,368,466 bytes**. It contains only unmodified `glim_dump/` plus `experiment_summary.txt` (23 regular files verified by SHA256). Raw logs, helper sources, observation JSONL, private ROS domain/network configuration, robot.yaml and ROS bags are excluded. Original native files remain unchanged. The archive has not been uploaded; the user uploads it manually.

Open its folder from WSL:

```bash
cd /home/feras/glim_ws/runs/mode_a_mapping/slam003e_manual_001
explorer.exe .
```

After packaging, stopped adapter, bridge and simulation with SIGINT. GLIM and all three launch wrappers exited 0; the adapter child retained its known SIGINT KeyboardInterrupt/-2 diagnostic. The observer had already exited after GLIM. Host process inspection found no task-owned simulation/GLIM/input/command or discovery helpers remaining. All 187 protected contents, validated/new mapping configs, previous stationary map/reload artifacts and earlier run/planning files were unchanged. Existing Git work was preserved; no commit/push, generator, package upgrade, source change, Mode B or Nav2.

Evidence: [SLAM-003E sanitized validation](../evidence/command_outputs/2026-11-21-slam-003e-manual-mapping.txt). Recommended next action: upload this archive for visualization, inspect IMU warnings, and separately resolve/verify Teleop speed application before another drive. No further experiment was started.


**Reusable figure archive (2026-11-21):** The user-supplied ChatGPT top-down map, pose-derived motion-speed plot and interactive HTML map have been copied from Downloads into the [SLAM-003E figure gallery](../evidence/figures/SLAM-003E/README.md). Original downloads and native experiment output remain unchanged; SHA256 provenance is recorded. The gallery includes captions and the experiment's limitations; the external plotting method/alignment has not been independently reproduced. New figures from later tasks follow the [task figure archive workflow](../evidence/figures/README.md).

### 22. SLAM-004A — TF-safe live local base-pose output (2026-11-21)

**Implementation/static validation PASS; live publication and TF checks NOT TESTED.**
Added the separate project-owned [glim_pose_output package](../src/glim_pose_output/README.md),
without editing installed GLIM or the validated Mode A mapping configuration.
The existing RViz publisher was unsuitable because its pose-output path also
enables TF broadcasts without an independent disable switch. Inspection of the
installed 1.2.2 headers/libraries and matching upstream implementations is recorded
with source links in the [SLAM-004A procedure](experiments/slam004a_pose_output.md).

The extension publishes `/a300_00000/slam/glim/pose` as
`geometry_msgs/msg/PoseStamped` using
`OdometryEstimationCallbacks::on_update_new_frame`. It reads the newest state
after point-cloud correction, copies it inside the callback, and uses its sensor
timestamp rounded to nanoseconds. Expected cadence follows processed clouds,
approximately 10 Hz after IMU initialization, rather than IMU rate. Output QoS is
best effort/volatile, keep last 10; consumers must request compatible QoS.

The pose body is **base_link** in a **GLIM-specific local `glim_odom` reference**:
`T_glim_odom_base = T_world_imu * inverse(T_base_imu)`, with verified
`T_base_imu = [0,0,0.294759,0,0,0,1]`. GLIM's `T_world_imu` variable here is its
local odometry estimate. This is not a globally corrected mapping pose, not an
alignment to Gazebo world/Clearpath odom/map, and not a confidence/covariance
estimate. No TF connects this reference to existing frames. A future switching
manager will need separate alignment and health handling; none is implemented.

No TF broadcaster/listener, new node, timer or subscription is created; the
publisher uses the existing GLIM host node. It links no TF or RViz library.
Clearpath/EKF retains `odom → base_link`. Exact LiDAR/IMU extrinsic, sensor topics,
QoS, timing and CPU estimation/local/global mapping settings are unchanged.
Invalid inputs are rejected without publication; output exceptions are contained.
Weak callback ownership handles host teardown without changing GLIM's
unsynchronized callback list while its worker can still be active.

Created isolated portable [configuration and exact differences](../configs/glim/mode_a_cpu_mapping_pose_output/README.md)
and private operational `/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output`.
Each has 16 JSON files; 12 original files remain byte-identical. Changes are the
new config reference, extension list, log destination and added base-extrinsic
file. Existing baseline configurations were preserved byte-for-byte. Only the
new package was built under `/home/feras/glim_ws/validation/slam004a_001`.
Ten deterministic GTest cases passed, actual operational Config API loading
passed, and JSON/module/factory/dependency/TF-symbol checks passed for both copies.
These tests did not start ROS nodes or exercise live host loading/publication.

All 262 snapshotted private/protected files were unchanged, including baseline
configurations, Clearpath files and previous native maps/archive. Starting worktree
was clean on `codex-work`, HEAD `f2d9d5b`; checkpoint commits and `main` are preserved.
The [proposed stationary runtime commands](experiments/slam004a_pose_output.md#proposed-stationary-runtime-test--authorization-required)
use the operational config, isolated library overlay, unique unused outputs,
simulation time and no viewer. Runtime requires separate authorization. No Gazebo,
live GLIM, movement, package change, generator, commit or push occurred.

Evidence: [sanitized SLAM-004A static validation](../evidence/command_outputs/2026-11-21-slam-004a-pose-output-static.txt).

### 23. SLAM-004B — Stationary runtime validation of live pose output (2026-11-21)

**Functional stationary runtime PASS; verbose CLI publisher listing INCONCLUSIVE.**
One separately authorized GLIM run loaded the CPU odometry, CPU local/global
mapping modules and project-owned pose extension from the isolated SLAM-004B
build. IMU initialization completed successfully. The robot was not driven;
the standard viewer remained disabled. The unchanged operational configuration
was `/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output`.

The fresh private run used the directory prepared by SLAM-004A,
`/home/feras/glim_ws/runs/mode_a_mapping/slam004a_pose_001`, matching its absolute
logging path. The manifest identifies this experiment as **SLAM-004B**. Native
output is in its `glim_dump/`; raw logs, observers and detailed samples remain
private. No earlier experiment or configuration was overwritten.

#### Observed live output

Received **301** `geometry_msgs/msg/PoseStamped` messages on
`/a300_00000/slam/glim/pose`, all with reference `glim_odom` and the documented
`base_link` body convention. Stamps **926.502–956.502 s** span exactly **30.000
simulation seconds**: **10.000 Hz** by sensor time, **9.735 Hz** by wall reception.
Every stamp matches an observed adapted-cloud stamp; all are strictly increasing,
with intervals **0.099–0.102 s**, no interval above 0.15 s and no nonfinite poses.
Reception lag against the observer clock was **0.096–0.414 s**; maximum wall
reception gap was **0.412 s**, without a sustained stall.

| Quantity | Minimum | Maximum |
|---|---:|---:|
| Position X (m) | −0.0013081263 | −0.0010817108 |
| Position Y (m) | −0.0005069518 | −0.0003797625 |
| Position Z (m) | −0.2977601116 | −0.2960021982 |
| Quaternion X | −2.2487537e−5 | −6.3184267e−7 |
| Quaternion Y | −9.6395606e−7 | 3.3278109e−5 |
| Quaternion Z | 7.1160971e−7 | 4.7582788e−6 |
| Quaternion W | 0.9999999991864 | 0.9999999999991 |

Quaternion norms were **0.9999999999999999–1.0000000000000002**. Maximum position
change from the first live pose was **0.001758 m**; maximum relative rotation was
**8.14734e−5 rad**. Both pass the predeclared stationary tolerances of 0.10 m and
0.10 rad. Negative Z is expressed in GLIM's local reference after the verified
IMU-to-base conversion, not Gazebo world or the existing Clearpath odom frame.

The independent lack-of-movement check observed **2,169 filtered wheel-odometry
messages**, maximum displacement **2.41e−17 m**, linear speed **3.09e−18 m/s** and
angular speed **2.39e−18 rad/s**. No command messages or nonzero commands were
observed during the monitor window. Absolute coordinates were not compared
between wheel odometry and GLIM. During this window, adapted clouds measured
**10.000 Hz**, IMU **100.000 Hz** and clock **333.333 Hz** by simulation stamps.
The earlier sensor preflight measured 9.999/97.954 Hz for cloud/IMU reception;
best-effort observation counts are not exact middleware loss counters.

#### TF isolation

Before/during publisher sets were identical: namespaced dynamic TF had
`robot_state_publisher`, `ekf_node`, `odom_base_tf_bridge` and
`platform_velocity_controller`; static TF had `robot_state_publisher` and
`imu_0_static_tf`, all under `/a300_00000`. Root `/tf` and `/tf_static` had no
publishers. No GLIM TF publisher or conflicting parent appeared.

A private read-only native C++ probe matched each received TF message's
`rclcpp::MessageInfo` DDS publisher GID to graph endpoint GIDs. Across the baseline
and runtime windows, all **4,337** observed `odom → base_link` transforms resolved
exclusively to **`/a300_00000/ekf_node`**. This avoids relying only on node names
or the Python MessageInfo API, which lacks that GID here. EKF `publish_tf=true`
and controller `enable_odom_tf=false` were confirmed by live parameter queries.
The registered Gazebo TF bridge had no publisher on its upstream Gazebo topic.
Exact sensor TF remained Z=0.294759/0.342259 m, identity rotations.

The initial reused sensor probe globally remapped TF topics, so its root endpoint
queries also resolved to the namespaced topics. A separate unremapped baseline
and unremapped runtime observers supplied the authoritative comparison above;
the original probe output was retained with this limitation documented.

#### Stability, save and cleanup

GLIM used **21.94–31.99% of one CPU core**, mean **24.35%**, and **59.39–178.25
MiB RSS**, including startup. Available memory stayed at **2028.5–2257.2 MiB**,
with zero swap use. These are sampled observations, not a benchmark. No GLIM
error, rejected pose, IMU-prediction warning block or safety abort occurred.

After 30 simulation seconds of live output, the guard sent SIGINT at **34.643 s
wall time**. GLIM drained odometry/local/global queues, logged `saved`, and exited
**0** after **34.944 s**. It saved **23 files / 676,332 bytes**, including
`graph.bin` (1,463 B), `values.bin` (573 B), `graph.txt` (64 B), ten config files,
one numbered submap with **301 retained frames / 2,699 finite compact points**,
and four valid finite, ordered trajectory files of 301 rows each. Live output
stamps were 926.502–956.502; saved trajectories were 926.4–956.4, so their boundary
windows differ by approximately one scan. No new binary-deserialization test or
mapping-accuracy/loop-closure claim is made.

Simulation, bridge and adapter launch wrappers exited **0** after SIGINT. The
adapter child emitted its known `KeyboardInterrupt`/−2 during rclpy destruction.
Both observers and the CLI-created discovery daemon were stopped; host `/proc`
inspection confirmed no task-owned runtime processes remained.

#### Tooling limitations and preservation

The first Gazebo/sensor attempt was blocked by sandbox networking (`getifaddrs:
Operation not permitted`; the sandboxed bridge child aborted). All those
processes were cleaned up before launching with the approved host transport;
separate logs preserve both attempts. **GLIM ran exactly once**, with no retry or
configuration change. The successful simulation emitted expected stationary
velocity-timeout/braking messages. GLIM warned about four missing optional
mapping settings and their installed defaults, plus generic extension/license
and missing optional `glim_ext` notices. None prevented extension loading.

The requested `ros2 topic info ... --verbose` query returned `Unknown topic`;
the daemon-free repeat near shutdown also did not capture a live listing. This
CLI check is **INCONCLUSIVE**. The best-effort typed `topic echo --once` succeeded
(stamp 938.502); direct observer graph snapshots identify the sole live publisher
`/a300_00000/glim_pose_output_test` with best-effort/volatile QoS, and the 301 typed
messages establish actual publication. No extra run was made to repair CLI
evidence. Future CLI inspection should use direct discovery with enough discovery
time early in an authorized run.

All **413** snapshotted private/protected file hashes matched after cleanup,
including original maps/archive, Clearpath files and all operational/baseline
configs. All **221** existing project files matched before documentation changes;
this task only appends to this integration document and the development log and
adds two sanitized evidence files. Prior pending work, `codex-work`, HEAD
`f2d9d5b`, original `main` and checkpoints are preserved; the index remains empty.
No commit, push, movement, viewer, generator, installation, RTAB-Map, Nav2 or
switching implementation occurred. No subsequent experiment was started.

Evidence: [runtime report](../evidence/command_outputs/2026-11-21-slam-004b-live-pose-runtime.txt)
and [structured measurements/artifact inventory](../evidence/command_outputs/2026-11-21-slam-004b-live-pose-runtime.json).

### 24. SLAM-004C — Manual live localization, partial validation after region abort

**Outcome: ABORTED_REGION_LIMIT_PARTIAL_VALIDATION (2026-11-21).** One separately
authorized user-operated moving run verified live base-pose tracking and hard
command caps, then the observer stopped the simulator at the reviewed center
region. The planned final stationary bookend/full sequence completion is not
verified. No retry or subsequent experiment was started.

Private run: `/home/feras/glim_ws/runs/mode_a_mapping/slam004c_manual_001`.
The separate operational GLIM copy changed only its log destination; the
successful SLAM-004B CPU settings, pose extension, sensor extrinsic/topics/QoS,
timestamps and TF isolation were unchanged. Sixteen new manual-control/API tests
and four analytic evaluation tests passed. The [procedure](experiments/slam004c_manual_pose_procedure.md)
records authorization gates and consumed paths; it must not be re-executed into
those existing outputs.

#### Control and recorded movement

The earlier GUI speed problem was addressed with a hard-capped, default-disarmed
[manual relay](../tools/manual_drive_guard/README.md). Only task-owned unbounded
command bridge, joystick-command and marker-command processes were stopped;
the existing mux/controller/EKF and protected files remained intact. Native
replacement input bridging fed an isolated raw topic. The actual disconnected
W/A check measured 1.0 m/s and 0.5 rad/s raw peaks,0.10/0.10 preview peaks, zero robot
commands and stationary wheels/reference. The first finite observation window
expired before recording key inputs; its files were preserved and a continuous
capture verified the repeated exercise. Initial setup/API errors were corrected
before arming and covered by additional installed-endpoint tests.

After separate GLIM/manual approvals, the native recorder subscribed to all
twelve explicit small measurement topics and captured a verified 12.006 s initial
stationary hold. The user alone supplied nonzero input after the explicit armed
message. Actual input included multiple forward/left bursts and combined
linear/yaw commands. All recorded guard/mux and controller commands respected
**0.10 m/s /0.10rad/s**, with other components zero. Physical wheel-response peaks
were 0.103242 m/s and 0.101488 rad/s, separate from commanded limits. No automatic
movement sequence or unsupported SLAM algorithm was introduced.

At simulation stamp **3740.877 s**, reference center radius reached **1.2501397 m**,
crossing the reviewed 1.25 m region. The observer disarmed and signalled task-owned
simulation shutdown; GLIM then drained/saved. Reference motion continued for
**1.029 s /0.092405m** during shutdown, ending at radius 1.340265 m and total path
1.370244 m. This is an observed stopping tail to review before further driving.
The region guard's abort is not evidence of a collision or a relaxed clearance
threshold. The nominal full-body projected region bound was 2.261005 m above the
unchanged 0.5 m criterion, with documented 3D/dynamic/illustrative-allowance limits.

#### Live output and synchronized metrics

The closed 25.17 MB MCAP contains **100,353 messages across twelve topics**, including
**1,554 live poses** and 7,449 native Gazebo reference messages. Live pose stamps
**3585.501–3740.802 s** increase strictly at about **10.000 Hz**; all positions and
quaternions are finite, with norms effectively one. Maximum adjacent displacement
was 13.27 mm and rotation 0.009773 rad; no jump exceeded the predeclared criteria.
There were 54 poses after the last turn command. Final stationary validation is
unavailable because the safety abort occurred while moving.

The [offline evaluator](../tools/slam_trajectory_eval/README.md) composes native
same-stamp `warehouse → model → base_link` reference poses. The published GLIM
pose already describes base_link; no second sensor conversion is applied.
Every live pose matches a reference interpolation bracket≤0.021 s, with linear
translation/SLERP and no extrapolation. One fixed metric SE(3), based on 101
initial stationary pairs, aligns local GLIM to world for analysis only; no scale,
whole-route fit or operational TF transform was used. Wheel odometry is not
ground truth.

| Captured motion, actual commands 3723.012–3740.829 s | Result |
|---|---:|
| Translation ATE RMSE /maximum | **16.293 mm /32.274mm** (178 poses) |
| Orientation error RMSE |0.29353° |
| One-second RPE translation RMSE | **6.615 mm** (168 pairs) |
| One-second RPE rotation RMSE | **0.11679°** |
| RPE interval tolerance |±0.03 s |

Whole-window ATE is 5.515 mm because it includes a long stationary prefix; the
moving-interval result above is the primary diagnostic. Native reference extends
to 3741.906 s, leaving **1.104 s without a GLIM estimate** after the last recorded
pose; that tail is not evaluated. These are captured-interval measurements in a
short synthetic Mode A trial, not a completed-sequence or general accuracy claim.

#### TF, resources, saving and preservation

Before/during TF publisher sets remained four dynamic/two static under
`/a300_00000`, with no root TF or GLIM/reference TF publisher. Native per-message
GID matching resolved all **11,295** sampled `odom → base_link` transforms solely
to EKF. Sensor transforms and exact `T_lidar_imu` remained unchanged. After
simulator shutdown the guard's graph check reflects disappearing mux endpoints;
no unexpected active input publisher was observed.

Over 238 GLIM resource samples, CPU was 11.98–33.93% of one core, mean 22.95%; RSS
46.2–213.5 MiB including startup; available RAM stayed above 1717.1 MiB with zero
swap. One undiagnosed IMU-prediction warning block and normal optional/default
extension notices appeared; no GLIM error/exception occurred. Recorder logged
no transport/cache loss notices, which does not prove zero DDS loss. Raw wheel
odom had 48 duplicate stamps and zero decreases; the filtered wheel, pose and
reference streams used for checks increase strictly. GUI raw headers remain
zero; raw-input timing uses bag receipt simulation time. Clock freeze on shutdown
also produces repeated zero guard-command stamps in the retained tail.

GLIM exited 0 after 238.314 wallseconds, saved 23 files /4,245,666 bytes, one submap,
2,323 frames and 2,699 finite compact points. Graph/values/config were saved;
four finite ordered 2323-row trajectories were validated. Binary graph/value
reload was not repeated. Recorder flushed/closed and exited 0. All task-owned
simulation, mapping, control, bridge and observer processes stopped; host process
inspection found no runtime leftovers. All 561 original private/protected hashes,
earlier experiments, validated configurations, GLIM source and checkpoints remain
unchanged. `codex-work`/HEAD `f2d9d5b` and original `main` are preserved; no staging,
commit/push, installation, generator, Nav 2, RTAB-Map or switching implementation.

Three [diagnostic figures with PNG/SVG, CSV data and editable source](../evidence/figures/SLAM-004C/README.md)
are archived for documentation. Full [sanitized report](../evidence/command_outputs/2026-11-21-slam-004c-manual-live-pose.txt)
and [structured measurements/hashes](../evidence/command_outputs/2026-11-21-slam-004c-manual-live-pose.json)
are available. Review the region exit, stopping tail and IMU warning before any
separately authorized shorter manual trial. Stop after SLAM-004C; no next run.

### 25. SLAM-005A — GLIM checkpoint and independent RGB-D SLAM planning

Completed GLIM work was reviewed and committed first as
`f2bad11511214a858d0e2dcf56b7016e5eb0e0d5`
(`feat: complete GLIM mapping and live localization validation`): 62 appropriate
project files, with private configurations, bags, native dumps and binaries
excluded. Fresh checks passed 10 GLIM, 16 manual-guard/API and four trajectory
tests. Created/switched to **`adding-rtab`** at that commit, initially clean;
`codex-work` remains at the checkpoint and `main` is unchanged. No push.

The [SLAM-005A plan](experiments/slam005a_rtabmap_integration_plan.md) documents
static camera/package inspection, verified release interfaces and the exact
SLAM-005B implementation gates. The current protected robot setup has **no RGB-D
camera, camera topics, optical frames or camera mount**; its in-memory model
expansion contains only IMU and Helios LiDAR sensors. Camera description/bridge
assets are installed, but RTAB-Map is absent. Local APT candidates are wrappers
0.23.13 and core 0.23.14. No packages were installed or refreshed.

Propose RGB-D-only native visual odometry feeding its own mapper in
`/a300_00000/slam/rtabmap`, with separate `rtabmap_odom`/`rtabmap_map` frames and
private database. A later tiny TF-free adapter would publish base-link local
visual odometry as PoseStamped on `/a300_00000/slam/rtabmap/pose`; native corrected
map poses remain separate. No GLIM, LiDAR or EKF pose feeds into the new estimator.
Both native nodes must disable TF sending, and the mapper must consume its own
odometry topic with `odom_frame_id` empty. Existing EKF authority is preserved.
The plan also identifies native TF-lookup fallback/wait behavior for measurement
without adding an odometry transform.

The plan flags ROS GTSAM 4.2 versus GLIM's local 4.3a1 for library-resolution
inspection before future runtime. The observed WSL Ryzen host is not the target
NUC; simultaneous performance is unmeasured. **No further movement is permitted
until the SLAM-004C region-abort/1.029 s shutdown tail is addressed.**

Only planning documentation and [sanitized static evidence](../evidence/command_outputs/2026-11-21-slam-005a-assessment.txt)
were added after the checkpoint, left unstaged/uncommitted. Original protected
files, GLIM configurations and experiment contents were hash-checked. No RTAB
implementation, adapter, configuration, run reservation, runtime or movement was
created. Stop after assessment; SLAM-005B needs a separate instruction.

### 26. SLAM-005B — Independent RGB-D RTAB-Map stationary integration

**Result (2026-11-21): functional stationary odometry/mapping PASS, with warnings
and post-run corrections validated statically only.** Remain on `adding-rtab`,
HEAD/checkpoint `f2bad11511214a858d0e2dcf56b7016e5eb0e0d5`; no new commit/push.
See the [full implementation/results report](experiments/slam005b_rgbd_rtabmap_integration.md)
and [sanitized evidence](../evidence/command_outputs/2026-11-21-slam-005b-rgbd-rtabmap.txt).

The APT simulation proposed 36 new packages and zero upgrades/removals. However,
candidate ROS GTSAM 4.2 shares `libgtsam.so.4` with GLIM's 4.3a1 and produces 54
unresolved GLIM symbols if selected. The user approved a **private package
installation**, instead of system APT/dpkg installation, under
`/home/feras/rtabmap_jazzy_isolated/slam005b_001`. Native core 0.23.14/ROS wrappers
0.23.13 run only through a scoped environment; normal GLIM still resolves local
4.3a1 with no unresolved symbols. No global linker or GLIM configuration change.

Approved forward-facing front-left D435 bottom-screw mount: base coordinates
(0.35,0.18,0.35) m; optical origin (0.3606,0.2125,0.3625) m. The isolated setup
`/home/feras/clearpath_rgbd005b_001` adds the native RGB-D sensor and support post
without changing the original 16 collision parts, 37 link transforms or IMU/LiDAR
sensor definitions. Original setup/world/generators remain untouched. Project
[package](../src/rtabmap_husky_integration/package.xml) and [tools/commands](../tools/rtabmap_isolated/README.md)
provide reproducible configuration, provisioning and launch.

After separate camera and RTAB-Map approvals, one stationary run produced
**379 valid local base PoseStamped messages at 12.643 Hz**, stamps 485.037–514.935 s,
on `/a300_00000/slam/rtabmap/pose` in `rtabmap_odom`. Position remained constant,
quaternions unit, timestamps strict. RGB-D odometry uses camera data, static TF
and simulation clock only; no GLIM, LiDAR, wheel/EKF pose or ground-truth input.
Mapper topic odometry, empty `odom_frame_id`, `odom_sensor_sync=false`, zero
lookup wait and disabled estimator TF sending kept EKF authority unchanged.
Native GID checks found **1460** `odom → base_link` records in the observed runtime
window, solely from EKF; the two native idle broadcaster endpoints sent no TF.

Saved private database: `runs/rtabmap/slam005b_001/database/rtabmap.db`,
**5,402,624 bytes**, read-only integrity check `ok`, 29 stored sensor records and
28 stored links. Online graph retained one node, with 28 negative-weight
stationary records; do not interpret those as separate viewpoints. The native
cloud contains **2411 finite XYZ points**, mostly visible floor. A reusable
[PNG/SVG preview, CSV and plotting source](../evidence/figures/SLAM-005B/README.md)
are archived; native DB/raw data remain private. No binary reload or moving
accuracy test was performed.

VO/map mean CPU 35.68/11.14% of one core, peak RSS 237.62/356.29 MiB; minimum
available RAM 1499.10 MiB, peak swap about 40 KiB. Logged VO mean delay 39.56 ms,
mapper 68.48 ms. This measures the Ryzen WSL host, not NUC or parallel performance.
Best-effort observers missed image messages; no loss-free transport claim.

Native VO/map saved and exited cleanly; the adapter's repeated-SIGINT teardown
raised KeyboardInterrupt/−2. Cleanup now handles it and passes a deterministic
test (12 adapter tests total). The deprecated grid parameter and native PNG
fallback were also made explicit; these final changes were **not runtime-retested**.
Expected missing-odometry-TF fallback warnings (278) and the optional validation
cloud's rendering-axis/half-pixel convention are recorded in the report. Late
CLI parameter dumps failed; startup logs/launch/static ELF/native TF checks are
the available evidence. All task-owned processes were stopped.

Original GLIM tests (10) and all 602 protected/private hashes passed; SLAM-005A
planning contents remain intact. No movement, GLIM launch, simultaneous-SLAM
test, Nav2 or switching was performed. **The SLAM-004C stopping delay remains
unresolved and blocks further driving.** Review these results, then separately
authorize a fresh stationary cleanup/config confirmation before parallel tests.
