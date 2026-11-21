# Proposed SLAM-003D-B controlled motion procedure

Prepared 2026-11-17 by SLAM-003D-A. **NOT EXECUTED.** Three new, separate user authorizations are required: (1) Gazebo, existing sensor inputs and recording/reference capture; (2) GLIM; (3) movement commands. Earlier stationary-test approvals do not authorize this experiment. Stop after one run; no retry, tuning, Mode B, Nav2 or accuracy evaluation.

## Reservation and preserved settings

Checkpoint: `60c39e30bf50b72b62d2e9c51988482790685c3f` on `codex-work`; parent SLAM-003A checkpoint `136ca1d35404a7d4f0c361985ea2215b76e2d43e`. Protected `main`: `ca1140ef760bd6e28c32229347e75d896bfb38b4`.

- Operational configuration: `/home/feras/glim_ws/configs/mode_a_cpu_mapping_slam003d_001`.
- Private run: `/home/feras/glim_ws/runs/mode_a_mapping/slam003d_001`.
- Module logs: `<run>/logs/glim`; ROS logs: `<run>/logs/ros`; native output: `<run>/glim_dump`; recording: `<run>/bag`; validation captures: `<run>/validation`.
- All 15 files inherited from SLAM-003B; 14 byte-identical, `config_logging.json` changes only `logging.log_dir` from `.../slam003b_001/logs/glim` to `.../slam003d_001/logs/glim`. The portable [snapshot](../../configs/glim/mode_a_cpu_mapping_moving/) instead uses relative `logs/glim` and must not be used by the runtime command.
- CPU odometry/local/global modules, enabled mapping, topics/QoS/time offsets, global-shutter zero point times, exact `T_lidar_imu=[0,0,-0.0475,0,0,0,1]`, extensions `[]`, and `publish_imu2lidar=false` are unchanged. Local optimization stays false, global optimization true. No new TF authority.

## Working control interface and bounded route

Historical live evidence confirms `/a300_00000/cmd_vel` is `geometry_msgs/msg/TwistStamped`, subscribed by the existing `twist_mux`; its output `/a300_00000/platform/cmd_vel` reaches the existing controller. The generated setup selects stamped commands, external priority 1, mux timeout 0.5 s, controller timeout 0.5 s, and controller limits 2 m/s and 2 rad/s. Joy/RC/marker priorities are higher. Never bypass the mux by publishing directly to the controller. The existing Gazebo keyboard bridge also publishes to the mux input; leave all keyboard/joystick controls idle and abort on any unexpected nonzero publisher. A stationary publisher endpoint alone is not proof of interference.

Before movement verify clear space for the robot footprint plus at least 0.5 m clearance along the entire proposed L-shaped path and turning sweep. Clearance has **not** been established statically from the warehouse scene. If it cannot be established at the unchanged spawn pose, do not move or invent an alternate route. Check all three velocity components not used remain zero. Directions are body +X forward / +Z left yaw.

| Stage | Nominal simulation seconds | Linear X (m/s) | Angular Z (rad/s) | Procedure |
|---|---:|---:|---:|---|
| Initial stationary | at least 10 | 0 | 0 | No command required; wait for explicit IMU initialization and stable processing |
| Forward 1 | 8 | 0.15 | 0 | 161 stamped messages at 20 Hz |
| Stop 1 | 2 | 0 | 0 | 41 stamped messages at 20 Hz; verify actual stop |
| Left turn | 8 | 0 | 0.20 | 161 stamped messages at 20 Hz |
| Stop 2 | 2 | 0 | 0 | 41 stamped messages at 20 Hz; verify actual stop |
| Forward 2 | 8 | 0.15 | 0 | 161 stamped messages at 20 Hz |
| Final stationary | 8 | 0 | 0 | 161 stamped messages at 20 Hz; verify actual stop |

Nominal total 46 simulation seconds, plus CLI discovery/process gaps and authorization delay. Ideal command integrals: 2.4 m total forward travel and 1.6 rad (91.7°) left yaw. Acceleration limits, wheel slip and scheduling affect actual motion. This is an open-loop command plan, not a promised geometric route. Each command block is bounded by wall time; the entire movement sequence has a 90 s wall ceiling. An operator/agent must continuously monitor the safety conditions below; the CLI blocks alone are not a watchdog or collision controller.

## Independent reference and frame relationships

The earlier [Gazebo type inspection](../../evidence/command_outputs/2026-11-12-gz-001-gz-topic-info.txt) confirms `/model/a300_00000/robot/pose` publishes `gz.msgs.Pose_V`. The [saved sample](../../evidence/command_outputs/2026-11-12-gz-001-gz-samples.txt) includes `warehouse → a300_00000/robot` at simulation stamp 81.333 s and model-relative link entries, including identity `a300_00000/robot → a300_00000/robot::base_link`. Use the named model pose composed with the same-stamp base pose; do not interpret a wheel/link pose as world pose. Recheck that relationship live rather than assuming identity persists.

Installed PosePublisher configuration enables links, nested models and pose vectors at nominal 50 Hz. [Gazebo 8.15 source](https://github.com/gazebosim/gz-sim/blob/gz-sim8_8.15.0/src/systems/pose_publisher/PosePublisher.cc) reads simulation entity poses with parent/child labels and simulation timestamps. This supplies a reference independent of wheel integration. This is source/history verification; current-run availability remains conditional on live inspection after gate 1. If missing or ambiguous, report reference unavailable and stop for review before movement; do not alter simulation.

Installed ros_gz_bridge 1.0.24 exports `Pose_V → TFMessage`. The [native converter](https://github.com/gazebosim/ros_gz/blob/1.0.24/ros_gz_bridge/src/convert/tf2_msgs.cpp) retains the individual transforms. Its [geometry conversion](https://github.com/gazebosim/ros_gz/blob/1.0.24/ros_gz_bridge/src/convert/geometry_msgs.cpp) preserves per-pose stamps and child IDs, with `::` converted to `/`. The prepared one-way bridge publishes only `/a300_00000/reference/gazebo_pose`. A TFMessage on this dedicated data topic is not broadcast into `/tf`; do not remap it into the live TF tree. Avoid PoseArray, which loses individual frame identities.

Filtered `/a300_00000/platform/odom/filtered` is `nav_msgs/msg/Odometry`, nominal 50 Hz, `odom → base_link`, from the existing wheel-only EKF; raw `/a300_00000/platform/odom` is controller wheel odometry. Neither is independent ground truth. Re-verify headers/rates live.

GLIM text exports are simulation seconds and sensor poses: `odom_lidar` / `traj_lidar` give LiDAR poses, IMU files give IMU poses; odometry/global map origins can differ from `warehouse` and ROS `odom`. For common-base comparison use `T_G_base = T_G_lidar * inverse(T_base_lidar)` (or corresponding IMU expression). Confirm the fixed TF at runtime: base-to-LiDAR Z=0.342259 m, base-to-IMU Z=0.294759 m, identity rotations. Reference base pose is `T_warehouse_model * T_model_base`. Interpolate by sensor header time, record any alignment explicitly and never substitute bag receipt time for pose time. Initial rigid alignment is required before comparing world origins. This preparation defines data relationships, not an accuracy metric/result.

## Gate 1: simulator, inputs and recording

Before launch: verify branch/checkpoints; all protected, operational and original-artifact hashes against the private manifest; no existing task-owned simulation/GLIM/recorder/reference bridge processes; at least 3 GiB available memory and 2 GiB free disk. Refuse existing `glim_dump`, `bag` or the proposed console logs. Do not reuse these outputs after an attempted run. No generator or build.

In every runtime terminal, prepare the existing environment without printing private settings:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS2CLI_DISABLE_DAEMON=1
export ROS_DOMAIN_ID="$(python3 -B -c 'import yaml; d=yaml.safe_load(open("/home/feras/clearpath_lidar11b_test/robot.yaml")); print(d["system"]["ros2"]["domain_id"])')"
export mapping_run=/home/feras/glim_ws/runs/mode_a_mapping/slam003d_001
export ROS_LOG_DIR="$mapping_run/logs/ros"
cd "$mapping_run"
set -o noclobber
```

After authorization, start each in its own terminal/session and capture its new process group/PIDs and unique console file. Shell `noclobber` protects the redirects:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_lidar11b_test generate:=false \
  > "$mapping_run/logs/simulation-console.log" 2>&1
ros2 launch husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py \
  > "$mapping_run/logs/helios-console.log" 2>&1
ros2 launch husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py \
  > "$mapping_run/logs/adapter-console.log" 2>&1
```

Check clock advancing; clouds approximately 10 Hz and IMU approximately 100 Hz; monotonic stamps, compatible QoS, nonempty tightly packed little-endian clouds and exact sensor TF. Recheck mux/controller message types, priorities, timeouts and absence of active competing commands; capture TF publisher sets. Use one bounded observer for safety/resource observations, not additional concurrent `topic hz` processes on the clouds.

Read-only current-run reference inspection, only after gate 1:

```bash
timeout 5s gz topic -i -t /model/a300_00000/robot/pose
timeout 5s gz topic -e -t /model/a300_00000/robot/pose -n 1
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -r __node:=slam003d_reference_bridge -p use_sim_time:=true \
  -p config_file:=/home/feras/clearpath_ws/configs/experiments/slam003d/ground_truth_bridge.yaml \
  > "$mapping_run/logs/reference-console.log" 2>&1
```

Verify dedicated-topic per-transform timestamps/labels and no new TF endpoints. Do not run raw `gz topic -e` continuously. Use one existing rosbag2 recorder with an explicit topic list:

```bash
test ! -e "$mapping_run/bag" || exit 1
ros2 bag record --use-sim-time --disable-keyboard-controls \
  --node-name slam003d_recorder -s mcap \
  --max-cache-size 8388608 -o "$mapping_run/bag" \
  --qos-profile-overrides-path /home/feras/clearpath_ws/configs/experiments/slam003d/recording_qos.yaml \
  --topics /a300_00000/sensors/helios_16p/points \
  /a300_00000/sensors/helios_16p/points_glim /a300_00000/sensors/imu_0/data \
  /a300_00000/platform/odom /a300_00000/platform/odom/filtered /clock \
  /a300_00000/cmd_vel /a300_00000/platform/cmd_vel \
  /a300_00000/reference/gazebo_pose /a300_00000/tf /a300_00000/tf_static \
  > "$mapping_run/logs/recorder-console.log" 2>&1
```

The 11-topic list records both command input and mux output, plus TF for sensor offsets. Recorder cloud/IMU subscriptions use BEST_EFFORT/VOLATILE, compatible with both original and adapted publishers; static TF uses RELIABLE/TRANSIENT_LOCAL. Only the recorder adds a subscription to the original cloud. The safety observer reuses its adapted/IMU/odom/clock inputs. No all-topic recording, compression, image stream or second cloud recorder. Estimated raw cloud payload is 3.92 MB/s, about 180 MB for 46 s or 353 MB for 90 s, before overhead and approval/preflight time; require free space throughout. Cache is 8 MiB per buffer (potentially twice that), not the default 100 MiB. Ensure the bag records all required topics before moving; inspect actual topic/QoS/drop behavior. Record headers and simulation-time bag receipt times without confusing them.

## Gate 2: GLIM mapping

Require separate user authorization after the live checks. Exact operational invocation:

```bash
test ! -e "$mapping_run/glim_dump" || exit 1
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_mode_a_moving_mapping \
  -p use_sim_time:=true -p debug:=false -p dump_on_unload:=false \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_cpu_mapping_slam003d_001 \
  -p dump_path:="$mapping_run/glim_dump" \
  > "$mapping_run/logs/glim-console.log" 2>&1
```

Observe native CPU module loads and successful IMU initialization. Allow at least 10 simulation seconds stationary from GLIM start and at least 3 s of stable initialized processing. Remain stationary while requesting movement authorization. Overall observation ends at 90 simulation seconds from GLIM start or 180 wall seconds, whichever first; reserve enough remaining time for the entire route. If authorization or initialization consumes the budget, stop gracefully without movement. No automatic restart.

## Gate 3: controlled commands

Require separate explicit authorization, clear route, successful initialization, recording and a running safety observer. The following are **proposed commands only**. Define this helper in the supervised movement terminal after that approval; it uses the installed CLI, not a new controller:

```bash
segment() {
  local label="$1" count="$2" vx="$3" wz="$4"
  timeout --signal=INT --kill-after=2s 20s \
    ros2 topic pub --use-sim-time --spin-time 0.5 \
      --rate 20 --times "$count" --print 0 \
      --wait-matching-subscriptions 1 --max-wait-time-secs 2 \
      --qos-reliability reliable --qos-durability volatile \
      --node-name "slam003d_${label}" \
      /a300_00000/cmd_vel geometry_msgs/msg/TwistStamped \
      "{header: {stamp: now, frame_id: base_link}, twist: {linear: {x: $vx}, angular: {z: $wz}}}" \
      > "$mapping_run/logs/command-${label}.log" 2>&1
}
# Run individually, verifying each result and actual stop before proceeding.
segment forward1 161 0.15 0
segment stop1 41 0 0
segment turn 161 0 0.20
segment stop2 41 0 0
segment forward2 161 0.15 0
segment final_stop 161 0 0
```

Do not paste the sequence as an unattended script. Any failed/timeout command or abnormal motion triggers abort; never continue to the next nonzero segment. `ros2topic` uses a ROS-clock timer, refreshes `stamp: now` on every message and spins for discovery; `161` messages span nominally 8 s, `41` span 2 s. Startup/keep-alive gaps add stationary time, not extra nonzero commands. Confirm actual stamps and mux output in the bag. Controller/mux timeout braking at 0.5 simulation seconds is a fallback, not the primary stop.

Emergency stop sequence: interrupt the identified active nonzero publisher first; send freshly stamped zero TwistStamped commands on the same mux input at 20 Hz for at least 2 simulation seconds using `segment emergency_stop 41 0 0` (new log filename). Verify both raw/filtered velocities below 0.02 m/s and 0.03 rad/s for at least 1 simulation second. If the simulation clock stalls, use a separately bounded wall-clock-rate zero publisher as fallback (`ros2 topic pub --rate 20 --times 41 --print 0` with all-zero TwistStamped, no `--use-sim-time`, under `timeout 5s`), explicitly filling its header with the last verified numeric simulation stamp. Do not use `stamp: now` with wall time: that would introduce a future-dated controller command. Mark the fallback in the record. If zeros cannot reach the controller, a higher-priority command is active, or motion continues, stop the identified task-owned simulator process group immediately. Do not resume it or rely on a frozen-clock timeout. Never stop unrelated processes.

## Monitoring, saving, validation and cleanup

Record process CPU/RSS, system available memory/swap, clock progression and sensor arrival/header gaps about once per wall second. Stop motion immediately on unexpected direction, motion during a zero stage, speed >0.20 m/s or yaw rate >0.30 rad/s after transient confirmation, loss of route clearance, conflicting TF authority, any GLIM exception/error/recovery, repeated sensor/processing warnings, recorder failure/drop bursts, or simulation instability. Also abort on cloud header gap >0.5 simulation s, IMU gap >0.1 simulation s, cloud/IMU wall silence >1 s, clock stall >1 wall s or rewind, available memory <1 GiB, GLIM RSS >1 GiB, sustained swap growth, free disk <1 GiB, sustained GLIM queue/backlog warnings or near-saturated CPU with growing delays for 5 s. CPU percentage alone is not proof of backlog; saved retained frames and timestamp coverage are required afterward.

Normal finish: verify final stationary state, then SIGINT the identified GLIM node while inputs/recording stay active. Allow up to 60 wall seconds for queue drain and native `saved`/exit 0. Do not use an observation timeout that kills GLIM before saving. If saving hangs, stop inputs and recorder gracefully; if resource exhaustion persists, terminate only identified task processes and report incomplete saving. Preserve partial outputs and do not retry or count a directory as success. After native save, stop recorder with SIGINT and wait for metadata flush, then reference bridge, adapter, Helios bridge and simulation; inspect all tracked process groups/descendants for leftovers.

Acceptance is based on actual outputs:

- Recorded motion follows both forward stages and left turn with continued clouds/IMU, then verified stop. Report measured travel/yaw versus command integrals; do not claim accurate control.
- GLIM initializes and retains frames covering the moving intervals, with finite ordered poses and nontrivial translation/yaw matching motion direction and timing after explicit frame conversion/alignment. Proposed smoke-test plausibility: >0.5 m displacement during each forward interval and >0.5 rad yaw during the turn; review deviations rather than claim accuracy. Saved coverage should be within 1 simulation second of both motion boundaries; explain frame losses and any non-finite/duplicate data.
- Enumerate actual submap/frame/point counts, compact cloud validity and all file sizes. No required number of submaps or loop closures: unchanged local OVERLAP keyframe selection (15 maximum keyframes, 0.6 overlap threshold) does not justify predicting a count for this short path. End-of-sequence flushing may create the only submap.
- Verify `graph.bin`, `values.bin`, `graph.txt`, saved config, numbered submaps and four trajectory files (`odom_lidar.txt`, `odom_imu.txt`, `traj_lidar.txt`, `traj_imu.txt`). Check finite timestamps/poses, ordering and unit quaternions; treat known IMU-rate segment-join duplicate stamps separately. Verify bag integrity and all required topic counts/timestamp coverage with `ros2 bag info` and an offline reader; recording success is not proven by folder existence.
- Record SHA256 before/after one bounded call to the existing headless native validator. First derive expected counts from saved graph/submap metadata and compact-array sizes, then run `timeout --signal=TERM --kill-after=5s 30s /home/feras/glim_ws/validation/slam003c_b_001/build/glim_dump_reload "$mapping_run/glim_dump" <actual_submaps> <actual_frames> <actual_points>` with `OMP_NUM_THREADS=2` and exclusive captures in `<run>/validation`. Never rerun the stationary validation or overwrite its build/results. Confirm no deserialization errors/recovery and matching counts, no dump changes.
- Recheck the 187 protected hashes and all original SLAM-003B/reload artifact hashes against `preparation_manifest.json`, plus the new config hashes. Verify every task-owned process stopped and branch/main unchanged. Keep all large data private. Update docs/sanitized evidence after reviewed runtime; no commit/push without further instruction.

The prepared validator requires `export_points()` count to equal summed loaded points. The inspected GLIM v1.2.2 implementation concatenates transformed full submap clouds without deduplication, so summed compact-cloud counts are the correct expectation even with multiple submaps. Registration random sampling does not replace those full clouds. Any mismatch is a blocker to report, not permission to change GLIM or validator expectations during the experiment.
