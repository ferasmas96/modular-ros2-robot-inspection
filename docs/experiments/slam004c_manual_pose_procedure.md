# SLAM-004C — Manual live localization experiment

Status: **STOPPED after one user-operated run: region-limit abort with valid
captured-motion pose checks and ATE/RPE.** The final stationary bookend was not
completed. GLIM saved and exited 0; recording and all task processes are closed.
See [results](../../evidence/command_outputs/2026-11-21-slam-004c-manual-live-pose.txt)
and [archived plots](../../evidence/figures/SLAM-004C/README.md). Paths below are
consumed historical experiment paths; do not reuse them for another run.
Sixteen guard/API tests and four trajectory-math tests pass. Setup-helper/API
errors were corrected before arming; their evidence is retained. Actual raw
1.0/0.5 commands were clamped to 0.10/0.10, first verified disconnected and then
against actual robot-path messages during movement.
Separate authorization gates below remain required. Preserve all prior files,
pending Git work and protected Clearpath/GLIM configurations. No commit/push.

## Preparation and control method

Branch `codex-work`, HEAD `f2d9d5b` preserved. Private preflight snapshot:
`/home/feras/glim_ws/validation/slam004c_001/before.json` (223 project files,
561 private/protected contents, including all SLAM-004B results).
Fresh run: `/home/feras/glim_ws/runs/mode_a_mapping/slam004c_manual_001`.
Fresh operational GLIM copy:
`/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output_slam004c_001`.
Only logging.log_dir differs from the successful SLAM-004B operational config;
all CPU mapping, pose extension, sensor extrinsic, timing, QoS and TF settings
remain unchanged. Reuse its validated isolated extension build.

Prior SLAM-003E messages reached ±1.0 m/s / ±0.5 rad/s, despite intended 0.10
settings; two nonzero events were observed before drive approval. The cause of
GUI-setting application was not established. Installed gz-gui is 8.4.0; matching
[Teleop source](https://github.com/gazebosim/gz-gui/blob/gz-gui8_8.4.0/src/plugins/teleop/Teleop.cc)
has those defaults and uses backend maxima for keyboard output. The
[QML controls](https://github.com/gazebosim/gz-gui/blob/gz-gui8_8.4.0/src/plugins/teleop/Teleop.qml)
call setters on editing completion. This does not prove why the earlier settings
were not applied; displayed/intended settings are insufficient evidence.

Use the [hard-capped, default-disarmed manual relay](../../tools/manual_drive_guard/README.md).
Temporarily stop only three task-owned input nodes from this simulation:
`cmd_vel_bridge`, `teleop_twist_joy_node`, `twist_server_node`. The native launch
definitions do not respawn them. No protected file or localization parameter is
changed. Start the replacement one-way raw bridge and guard. The existing mux
and controller remain responsible for command execution; there must be no bypass
publisher or active alternative input. Guard/source code and offline tests are
reviewable in `tools/manual_drive_guard/`.

## Gate 1 — Simulation, inputs, reference and DISARMED control

Obtain user approval before launching. Use working host ROS/Gazebo transport
through the execution approval mechanism; the restricted sandbox previously
prevented getifaddrs/DDS discovery. Do not repeat that known failed transport.
Source the existing ROS/workspace and isolated SLAM-004B extension environments
with the existing private ROS domain/network settings, and set ROS_LOG_DIR to
this new run's logs/ros. Never print private settings or overwrite logs.

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_lidar11b_test world:=warehouse \
  generate:=false rviz:=false x:=-4.0 y:=3.0 z:=0.3 yaw:=0.0
ros2 launch husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py
ros2 launch husky_a300_sensor_extensions pointcloud2_timing_adapter.launch.py
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -r __ns:=/a300_00000 -r __node:=slam004c_reference_bridge \
  -p use_sim_time:=true \
  -p config_file:=/home/feras/clearpath_ws/configs/experiments/slam003d/ground_truth_bridge.yaml
# After task-owned bypass publishers have stopped:
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -r __ns:=/a300_00000 -r __node:=slam004c_manual_input_bridge \
  -p use_sim_time:=true \
  -p config_file:=/home/feras/clearpath_ws/configs/experiments/slam004c/manual_input_bridge.yaml
python3 -B /home/feras/clearpath_ws/tools/manual_drive_guard/manual_guard.py
```

Run each in a separate tracked process group with exclusive console filenames.
Confirm advancing clock, approximately 10 Hz adapted LiDAR / 100 Hz IMU, exact
base-to-IMU/LiDAR transforms, no conflicting TF and stationary wheels. The
ground-truth bridge publishes data on `/a300_00000/reference/gazebo_pose`, never
TF. Inspect same-stamp world/model/base records and compose world-to-base. Abort
before motion if reference identity/timestamps are missing or ambiguous.

## Gate 2 — Operator calibration while DISARMED

Request the user's keyboard exercise after explaining that robot output is
disarmed. User selects the existing Gazebo Teleop Keyboard panel on the existing
Gazebo topic and briefly holds/releases W, then A, then releases all keys.
The assistant never creates nonzero input commands. Record actual raw and
preview messages, robot mux input/output and wheel odometry. Require bounded
nonzero preview values (≤0.10 each), all other components zero, robot commands
zero, stationary wheels, and released final input. If limits/exclusivity cannot
be verified, stop and explain; do not enable driving.

## Gate 3 — GLIM

Obtain separate approval after control/sensor/reference checks. No viewer or TF
publisher. One GLIM process; do not change algorithms or restart on failure.

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -r __ns:=/a300_00000 -r __node:=glim_manual_pose_004c \
  -p use_sim_time:=true -p debug:=false -p dump_on_unload:=false \
  -p config_path:=/home/feras/glim_ws/configs/mode_a_cpu_mapping_pose_output_slam004c_001 \
  -p dump_path:=/home/feras/glim_ws/runs/mode_a_mapping/slam004c_manual_001/glim_dump
```

Verify CPU odometry/local/global mapping and extension loads, IMU initialization,
healthy finite live base poses near 10 Hz and no GLIM TF publisher. Use direct
CLI discovery early; do not let a late CLI query replace actual typed reception
and endpoint inspection. Keep the robot disarmed/stationary until gate 4.

## Gate 4 — Recording and user-operated driving

Obtain explicit manual-driving approval. Start one native MCAP recorder with
simulation time, twelve explicit topics from `recording_qos.yaml`, an 8 MiB
cache per buffer, and no clouds/IMU payloads. Preserve its logs and metadata.
Record at least 10 initial stationary simulation seconds **in the bag**, and
verify all required topics are actually received before arming. Keep the
reference recorder synchronized by message sensor stamps; recording receipt time
alone is not a synchronization method. Capture raw and limited commands to
demonstrate the clamp, and controller commands to rule out mux bypass.

Only after that initial hold, arm the guard via its service under the user's
authorization. Arming discards old inputs; fresh manual key events are required.
The user personally drives the suggested short sequence: forward, stop, left
turn, stop, forward, stop. Initial suggestion: about 5 s forward, 5 s left turn,
5 s forward at maximum 0.10/0.10, with explicit stops and a visibly clear path.
These are operator instructions, not automatic command blocks or a certified
3D route. Keep the unchanged ≥0.5 m obstacle margin; stop if uncertain. Stay near
the clear spawn; retain human visual oversight. Do not run the rejected old route.

The reused world/robot collision analysis gives an orientation-independent
projected bound for the small operator region: all 16 robot collision parts fit
within a 0.541257 m XY radius. With robot center within 1.25 m of (−4,3) and an
additional illustrative 0.15 m allowance, nominal world clearance is bounded
below by **2.261005 m**, limited by the warehouse collision mesh. The unchanged
0.5 m criterion passes this projection. Ground support and sufficiently high roof
triangles are excluded consistently with the prior analysis; two chair models
are nonstatic, so live visual verification remains required. This does not
certify full 3D collision freedom or measured tracking uncertainty.
`tools/slam_route_clearance/check_manual_region.py` reproduces the bound without
runtime; the private result is `validation/static_motion_region.json`.
The independent runtime observer must stop input if the reference center leaves
the 1.25 m region or total measured travel exceeds 1.5 m.

The independent monitor checks actual mux/controller command limits at every
received sample. Check the first user-operated movement before continuing to
turns. Finish within 120 wall seconds of arming; no rearm/retry on a fault. On
finish, disarm and retain ≥10 s final stationary data before graceful save.

Abort/disarm on nonfinite poses, non-increasing stamps, translation jump greater
than `0.10*dt + 0.05 m` or orientation jump greater than `0.10*dt + 0.05 rad`,
pose/cloud/reference gaps >0.5 simulation seconds, repeated processing warnings,
clock/input stalls, TF conflicts, command-limit violation, insufficient clearance,
available RAM <1 GiB, GLIM RSS >1 GiB or unstable recording. If zero commands do
not stop the robot or another publisher wins the mux, stop only the identified
task-owned simulator. No automatic movement or recovery sequence.

Report recorder transport/cache loss messages separately from header-stamp gaps;
the prior SLAM-003D recorder's three losses are a known data-quality limitation.
Do not claim zero loss merely from a successful bag close. Validate counts,
timestamps, QoS and gaps after recording; disclose observer/bag differences.

## Comparison and completion

Primary estimate is the live PoseStamped **already converted to base_link**.
Do not apply the sensor extrinsic twice. Reference is
`T_world_model * T_model_base` from same-stamp native Gazebo poses. Never use
wheel odometry as ground truth. Verify parent/child identities and saved static
extrinsics independently.

Interpolate reference position linearly and orientation with quaternion SLERP at
GLIM header stamps, without extrapolation; require reference brackets ≤0.10 s.
Use one fixed metric SE(3) alignment from the recorded initial stationary hold,
not a fitted scale or whole-route best fit. Compare common base reference points.
Compute translation ATE RMSE/median/p95/max and orientation error. For 1 s RPE,
pair estimate samples within 0.03 s of the intended interval and compute
`E_ij = inverse(inverse(T_ref_i)*T_ref_j) * (inverse(T_est_i)*T_est_j)`.
Report translation/rotation RMSE, pair count, coverage and exclusions. If motion,
reference identity, synchronization, coverage or calibration data is insufficient,
report exactly why valid metrics cannot be calculated.

After final stop, keep sensors active while GLIM drains and saves. Then stop
recorder (flush metadata), guard/raw/reference bridges, adapter/LiDAR bridge and
Gazebo. Inspect native graph/values/submaps/trajectories, bag metadata and all
task-owned processes. Compare all protected/source/config/artifact hashes. Append
results to integration docs/log and add sanitized evidence. If figures are
produced, archive actual exports and plotting source under evidence/figures/SLAM-004C.
No commit/push, new SLAM algorithm, Nav 2, switching or next experiment.
