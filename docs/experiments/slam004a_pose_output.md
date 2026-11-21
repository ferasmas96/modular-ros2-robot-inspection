# SLAM-004A — TF-safe GLIM live pose output (2026-11-21)

Implementation and static validation **PASS**. Live publication, rate, cost and
live TF checks are **NOT TESTED**. No simulation, live GLIM or movement was started.

## Inspection and choice

Installed `ros-jazzy-glim` and `ros-jazzy-glim-ros` are both **1.2.2-0noble**.
The installed callback/config/extension headers and libraries were inspected with
the corresponding upstream v1.2.2 implementations:

- [ROS publisher implementation](https://github.com/koide3/glim_ros2/blob/v1.2.2/src/glim_ros/rviz_viewer.cpp): existing pose publishers belong to `RvizViewer`; its predicted-frame path also broadcasts odom/base and map/odom transforms. `publish_imu2lidar` controls only the sensor pair. No independent TF-disable setting was found there. Renaming/remapping its TF would still create a broadcaster.
- [Estimator implementation](https://github.com/koide3/glim/blob/v1.2.2/src/glim/odometry/odometry_estimation_imu.cpp): `on_update_new_frame` follows smoothing/state updates and supplies `stamp` and `T_world_imu` for the newest scan. This is a scan-corrected local estimate, distinct from global-map corrections and the earlier IMU-prediction callback.
- [ROS host initialization and destruction](https://github.com/koide3/glim_ros2/blob/v1.2.2/src/glim_ros/glim_ros.cpp): extension `create_subscriptions(Node&)` runs before sensor subscription creation; extensions are cleared during host destruction. A publisher can use that hook without adding a node or subscription.
- [Native extension factory](https://github.com/koide3/glim/blob/v1.2.2/src/glim/util/extension_module.cpp) and [library loader](https://github.com/koide3/glim/blob/v1.2.2/src/glim/util/load_module.cpp): factory symbol is `create_extension_module`; library handles are retained. The installed callback slot has no synchronization, motivating weak callback ownership rather than teardown removal.

Therefore a small project-owned extension is used; installed source is untouched.
It does not use the viewer's trajectory manager or globally corrected publisher.
For exact message/frame/extrinsic/lifetime semantics see the
[component README](../../src/glim_pose_output/README.md).

## Files and isolation

- `src/glim_pose_output/{package.xml,CMakeLists.txt,README.md}`
- `src/glim_pose_output/include/glim_pose_output/pose_output.hpp`
- `src/glim_pose_output/src/{pose_output.cpp,extension.cpp}`
- `src/glim_pose_output/test/{test_pose_output.cpp,check_configuration.py}`
- Portable config: `configs/glim/mode_a_cpu_mapping_pose_output/` (16 JSON files plus README).
- Operational config: `/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output` (16 JSON files).
- Private build/test/safety records: `/home/feras/glim_ws/validation/slam004a_001`.
- Proposed future run: `/home/feras/glim_ws/runs/mode_a_mapping/slam004a_pose_001`.
  **Not created or executed.** The operational logging path points to its `logs/glim`.

The [snapshot README](../../configs/glim/mode_a_cpu_mapping_pose_output/README.md)
records every difference from the validated baseline. Both mapping modules remain
enabled, all three CPU module libraries resolve, and 12/15 baseline files are
byte-identical in each new copy. Only global extension-config reference, extension
list, log destination and the added base-extrinsic file differ. Operational and
portable copies differ only in `config_logging.json`. Portable logging is relative
to the process working directory; runtime must use the operational absolute path.

## Reproduce the build and static checks

These commands build only the new package into the existing private validation
directory, leaving the normal workspace build/install trees alone. They do not
launch GLIM, initialize ROS nodes or install system packages. Use a new validation
directory if preserving a previous build's logs is required.

```bash
cd /home/feras/clearpath_ws
source /opt/ros/jazzy/setup.bash
pose_check=/home/feras/glim_ws/validation/slam004a_001
colcon --log-base "$pose_check/colcon_log" build \
  --base-paths src/glim_pose_output --packages-select glim_pose_output \
  --build-base "$pose_check/build" --install-base "$pose_check/install" \
  --cmake-args -Dglim_DIR=/opt/ros/jazzy/share/glim \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon --log-base "$pose_check/colcon_log" test \
  --base-paths src/glim_pose_output --packages-select glim_pose_output \
  --build-base "$pose_check/build" --install-base "$pose_check/install" \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base "$pose_check/build/glim_pose_output" --verbose
GLIM_POSE_OUTPUT_TEST_CONFIG=/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output \
  "$pose_check/build/glim_pose_output/test_pose_output" \
  --gtest_filter=PoseOutput.LoadsActualSnapshotUsingInstalledGlimConfigApi
python3 -B src/glim_pose_output/test/check_configuration.py \
  --baseline /home/feras/glim_ws/configs/mode_a_cpu_mapping \
  --config configs/glim/mode_a_cpu_mapping_pose_output \
  --config /home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output \
  --library "$pose_check/install/glim_pose_output/lib/libglim_pose_output.so"
```

Results: isolated build PASS; ten GTest cases PASS (colcon reports 11 including
the CTest wrapper), zero failures/errors/skips; actual operational extrinsic read
through installed GLIM Config API PASS. Tests cover exact base offset, noncommuting
rotations/inverse direction, sensor stamps/rounding/range, finite rigid poses,
unchanged frame state, weak lifetime and bad extrinsic configurations. Static
config/link audit PASS for both copies: all JSON and file/module references,
exact sensor settings, native CPU library resolution, extension factory symbol,
no unresolved dependencies, no TF symbols or TF/RViz linked libraries. The audit
opens shared libraries but never calls the factory or constructs an estimator.

The initial configure attempt caught mixed CMake link signatures and an invalid
placeholder maintainer email; both were corrected before the successful build.
No compiler warning or unresolved build issue remains. No ROS publisher/discovery
or end-to-end host loading was exercised by these tests.

## Proposed stationary runtime test — authorization required

Do not execute this section during SLAM-004A. Obtain separate authorization for
simulation/sensor startup and for the GLIM run. Keep the robot stationary; no
command publisher, driving, RViz, switching, RTAB-Map or Nav2 is needed.

1. Recheck `codex-work`, protected hashes, library/config audit, free memory/disk,
   unused output paths and absence of competing processes. Preserve existing data.
2. Prepare every terminal with the existing ROS environment/domain/network settings
   from the validated setup (do not print private settings). The normal workspace
   overlay supplies existing simulation/bridge/adapter packages; add only this
   isolated extension overlay. The runtime run directory is created exclusively,
   so an existing attempt cannot be overwritten:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
source /home/feras/glim_ws/validation/slam004a_001/install/local_setup.bash
export ROS2CLI_DISABLE_DAEMON=1
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="$(python3 -B -c 'import yaml; d=yaml.safe_load(open("/home/feras/clearpath_lidar11b_test/robot.yaml")); print(d["system"]["ros2"]["domain_id"])')"
export pose_run=/home/feras/glim_ws/runs/mode_a_mapping/slam004a_pose_001
mkdir "$pose_run" || exit 1
mkdir -p "$pose_run/logs/ros"
export ROS_LOG_DIR="$pose_run/logs/ros"
cd "$pose_run"
set -o noclobber
```

Create the directory only once; subsequent terminals set the same variables and
source the environments without repeating `mkdir "$pose_run"`.

3. **After simulation authorization**, start the previously validated open-location
   setup in its own terminal and confirm the GUI/pose, zero motion and clock:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_lidar11b_test world:=warehouse \
  generate:=false rviz:=false x:=-4.0 y:=3.0 z:=0.3 yaw:=0.0 \
  > "$pose_run/logs/simulation-console.log" 2>&1
```

After sensor-input approval, run the existing inputs in separate terminals:

```bash
ros2 launch husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py \
  > "$pose_run/logs/helios-console.log" 2>&1
ros2 launch husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py \
  > "$pose_run/logs/adapter-console.log" 2>&1
```

Verify original/adapted clouds near 10 Hz, IMU near 100 Hz, advancing monotonic
simulation stamps, compatible QoS and exact existing sensor TF. Record TF publisher
identities/transform authorities **before GLIM**, including namespaced and root TF:

```bash
ros2 topic info --verbose /a300_00000/tf
ros2 topic info --verbose /a300_00000/tf_static
ros2 topic info --verbose /tf
ros2 topic info --verbose /tf_static
```

4. **After separate GLIM authorization**, invoke the operational configuration:

```bash
test ! -e "$pose_run/glim_dump" || exit 1
timeout --signal=INT --kill-after=60s 60s \
  ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_pose_output_test \
  -p use_sim_time:=true -p debug:=false -p dump_on_unload:=false \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output \
  -p dump_path:="$pose_run/glim_dump" \
  > "$pose_run/logs/glim-console.log" 2>&1
```

The 60 s wall ceiling includes initialization; allow up to another 60 s for
SIGINT queue drain/native saving. Timeout's exit 124 alone is not a GLIM error;
inspect child exit/save completion. Stop earlier after approximately 30 simulation
seconds of initialized output, or on resource pressure, repeated errors, severe
backlog or unstable simulation. Record identified process groups/PIDs. Do not
retry or alter settings. GLIM 1.2.2 may emit the generic external-module/license
notice and `glim_ext package path was not found!!` for this project-owned module;
it uses its own config, not `glim_ext`. Failure to load `libglim_pose_output.so`
or absence of its TF-free startup message is a test failure.

5. In another prepared terminal, inspect the **data** output using compatible QoS:

```bash
ros2 topic info --verbose /a300_00000/slam/glim/pose
timeout --signal=INT 10s ros2 topic echo /a300_00000/slam/glim/pose \
  geometry_msgs/msg/PoseStamped --once --qos-reliability best_effort
timeout --signal=INT 10s ros2 topic hz /a300_00000/slam/glim/pose --use-sim-time
```

Confirm a single intended publisher, correct message type, `glim_odom`, finite
base poses, unit quaternions, estimator/scan stamps (not wall time), increasing
stamps over multiple samples and a rate consistent with processed clouds. Record
CPU/RSS/backlog and compare the pose's sensor-height correction against the exact
static chain. Re-run the four TF publisher inspections and inspect individual
parent/child authorities: no GLIM TF publisher or new `odom → base_link` authority
may appear. Clearpath/EKF remains the authority. The unique reference frame is
intentionally not connected into TF; do not add a transform just to display it.

The installed Jazzy `topic hz` already uses sensor-data best-effort QoS and has
no reliability option; `--use-sim-time` selects the advancing simulation clock.
It measures reception rate. Also use successive message sensor stamps to assess
estimator cadence; do not treat reception rate alone as estimator throughput.

6. Stop GLIM gracefully (SIGINT) while inputs remain available, allow native save
   to complete, then stop only task-owned adapter/bridge/simulation processes.
   Preserve all fresh logs/dumps privately, recheck protected hashes and Git, and
   document observed results separately. No runtime result is claimed here.
