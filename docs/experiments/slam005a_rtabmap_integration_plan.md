# SLAM-005A — GLIM checkpoint and RTAB-Map integration plan

Date: 2026-11-21. Status: static assessment complete; implementation and runtime
are pending. This document describes proposed interfaces, not available camera
topics or a tested RTAB-Map system.

## Git checkpoint and preservation

Completed GLIM work was reviewed and committed **before RTAB-Map inspection**:

- Commit: `f2bad11511214a858d0e2dcf56b7016e5eb0e0d5`.
- Message: `feat: complete GLIM mapping and live localization validation`.
- Scope: 62 project-owned files, including the pose-output package and tests,
  portable configurations, manual guard, trajectory evaluator, documentation,
  sanitized evidence and small reusable figures. Private settings, native maps,
  bags, binaries and build/log directories were excluded.
- Fresh verification: 10 GLIM C++ tests, 16 manual-guard/API tests and four
  trajectory-math tests passed. Original evidence/configuration bytes were retained.
- Created and switched to **`adding-rtab`**, initially clean, at the same commit.
  `codex-work` remains at that checkpoint. `main` remains
  `ca1140ef760bd6e28c32229347e75d896bfb38b4`; previous checkpoints remain ancestors.

Only this planning document, sanitized inspection evidence, and append-only
integration/development-log entries follow the checkpoint. They remain uncommitted.
No push, installation, generator, simulation, SLAM process or movement occurred.
See [inspection evidence](../../evidence/command_outputs/2026-11-21-slam-005a-assessment.txt)
and its [structured record](../../evidence/command_outputs/2026-11-21-slam-005a-assessment.json).

## Existing camera: absent from the validated setup

Inspected `/home/feras/clearpath_lidar11b_test/robot.yaml`, its existing Xacro,
sensor launch files and an **in-memory** simulation expansion using the sourced
workspace overlay. This did not regenerate or write the robot description.

| Question | Static finding |
|---|---|
| RGB-D camera instantiated? | No. `sensors.camera` is empty. Expanded sensors are `imu_0` (`imu`) and `helios_16p_gpu_lidar` (`gpu_lidar`). |
| Existing camera ROS/Gazebo topics | None defined by this robot setup; no live topic graph was started or queried. |
| RGB, depth, CameraInfo | No configured camera streams. Current LiDAR cannot substitute for RGB-D visual odometry. |
| Camera links / optical TF / mount | No camera links in the expansion; no existing camera-to-base transform or camera mounting pose to validate. |
| Ready for RGB-D visual odometry? | No: camera, calibration, synchronized images and verified extrinsic are prerequisites. |

The installed Clearpath description supplies an `intel_realsense` RGB-D macro
(including D435), and `realsense2_description` is available. These are reusable
assets, not evidence of an active camera or of the physical camera model. The
**workspace overlay** macro passes `use_nominal_extrinsics=$(arg is_sim)`; its
optical joints use roll/yaw −π/2. Inspect the overlay used at launch rather than
assuming its bytes equal `/opt/ros/jazzy`.

The installed Clearpath Gazebo bridge templates use the following conventions.
They are **proposed future names** for a new `camera_0`, not current topics:

| Gazebo transport | Proposed ROS 2 topic | Type |
|---|---|---|
| `/a300_00000/sensors/camera_0/image` | `/a300_00000/sensors/camera_0/color/image` | `sensor_msgs/msg/Image` |
| `/a300_00000/sensors/camera_0/depth_image` | `/a300_00000/sensors/camera_0/depth/image` | `sensor_msgs/msg/Image` |
| `/a300_00000/sensors/camera_0/camera_info` | `/a300_00000/sensors/camera_0/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| Same camera-info source in the template | `/a300_00000/sensors/camera_0/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |

The existing warehouse world already includes the Sensors system with Ogre2.
Reuse it. For simulation, prefer the existing RGB-D sensor and image/CameraInfo
bridges; a hardware RealSense driver is not required. Select the forward-facing
mount in an isolated setup copy in SLAM-005B after checking camera/robot collision
geometry, self-occlusion and its exact `T_base_camera_optical`. No mounting
translation or camera extrinsic is approved here. Retain the unchanged LiDAR/IMU
transforms, including `T_lidar_imu=[0,0,-0.0475,0,0,0,1]`.

Validate actual headers and geometry: optical axes +Z forward, +X right, +Y down;
RGB/depth registration, calibration dimensions/intrinsics, depth units and invalid
pixels. The nominal physical D435 frame model alone does not prove simulated
depth registration. If streams differ, use supported rectification/registration
only after demonstrating the need; do not relabel a frame to hide an offset.

## Software and compatible upstream interfaces

Sourced Jazzy and the existing workspace without starting ROS nodes. Ament,
`dpkg-query` and executable lookup show **no installed RTAB-Map core or ROS
packages/executable**. Already present: `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`,
`cv_bridge`, `image_geometry`, `image_transport`, `message_filters`,
`realsense2_description` and `realsense2_camera_msgs`.
`realsense2_camera`, `image_proc` and `depth_image_proc` are absent; the latter two
are conditional preprocessing dependencies, not assumed requirements for a
registered synthetic RGB-D stream.

The local APT cache offers ROS wrappers **0.23.13** and core **0.23.14**. The
[Jazzy distribution metadata](https://raw.githubusercontent.com/ros/rosdistro/master/jazzy/distribution.yaml)
lists those release versions. No APT refresh or installation was performed;
recheck candidates and the proposed transaction before any authorized install.

| Package / executable | Assessment and recommendation |
|---|---|
| `rtabmap_odom` / `rgbd_odometry` | Required native RGB-D visual odometry; source target verified, absent locally. |
| `rtabmap_slam` / `rtabmap` | Required native RGB-D mapping; source target verified, absent locally. |
| `rtabmap_sync` / `rgbd_sync` | Native synchronizer verified; optional extra process if sharing one synchronized RGBDImage is warranted. Package is also a binary dependency. |
| `rtabmap`, `rtabmap_msgs`, `rtabmap_conversions`, `rtabmap_util` | Missing core/message/support packages; resolve through the approved Jazzy package transaction. |
| `rtabmap_launch`, `rtabmap_ros` | Available in cache, absent locally; convenient launch/meta packages, not essential to the minimum two-node launch. |

Executable names are confirmed by the release's
[odometry CMake](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_odom/CMakeLists.txt),
[mapping CMake](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_slam/CMakeLists.txt)
and [sync CMake](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_sync/CMakeLists.txt),
not by executing uninstalled programs. Eight inspected source files were fetched
read-only and byte-checked against release commit
`cb2117f1ed6674cf7cfc644d7bb4ffc0b567b364`; their hashes are in the evidence.

APT's core package also depends on `ros-jazzy-gtsam` (candidate **4.2.0**),
`ros-jazzy-libg2o` and `ros-jazzy-libpointmatcher`, currently absent. GLIM already
uses `/usr/local/lib/libgtsam.so.4.3a1`. This is an **unresolved library-resolution
check**, not an observed incompatibility. Before launching anything after a future
install, inspect `readelf`/`ldd` for both stacks in their intended environments.
Keep separate processes/overlays as needed; do not replace GLIM's GTSAM, change
global linker paths or assume selecting another optimizer removes the shared
library dependency. Stop and report if binary coexistence cannot be preserved.

## Recommended minimum independent data flow

Use namespace **`/a300_00000/slam/rtabmap`**. The camera is the only estimator
sensor input initially; no GLIM pose, LiDAR, wheel/EKF odometry, TF odometry guess
or ground-truth input. Robot static sensor transforms and `/clock` are shared
infrastructure. Camera failure still makes this method unavailable; independence
from LiDAR does not imply independence from common CPU/power/time failures.

| Stage | Input → output | Proposed semantics |
|---|---|---|
| RGB-D sensor + existing bridges | RGB, registered metric depth, calibrated CameraInfo | Matching simulation acquisition stamps; calibrated optical frame. |
| `rgbd_odometry` | RGB-D → namespace `odom`, `odom_info` | `nav_msgs/msg/Odometry`, parent **`rtabmap_odom`**, child **`base_link`**. |
| `rtabmap` mapper | Same RGB-D + **that VO odometry topic** → native map graph/data and database | Own **`rtabmap_map`**, with map corrections kept internal. |
| Future tiny project adapter | VO `Odometry` → `/a300_00000/slam/rtabmap/pose` | `geometry_msgs/msg/PoseStamped`, base pose in **`rtabmap_odom`**, original stamp, no TF. |

The adapter is **planned only**. It should copy the header and `pose.pose`, check
the expected child/frame, finite values and a valid unit quaternion, and drop
invalid/lost estimates. Never synthesize confidence, restamp old poses as current,
or publish a cached pose as successful tracking. Preserve native odometry and
diagnostics for health analysis. This gives continuous **local visual odometry**,
matching the local character of the existing GLIM `glim_odom` output; it is not a
globally corrected or Gazebo-world pose. No alignment between algorithm frames
is established. The [odometry source](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_odom/src/OdometryROS.cpp)
publishes sensor-stamped base odometry and gates transform sending on `publish_tf`;
the [RGB-D implementation](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_odom/src/nodelets/rgbd_odometry.cpp)
obtains the base-to-camera transform from TF.

For map-corrected localization, retain native namespace `localization_pose`
(`geometry_msgs/msg/PoseWithCovarianceStamped`) separately in `rtabmap_map`.
With `pub_loc_pose_only_when_localizing=false`, it can publish after successful
non-intermediate mapping updates when subscribed, using correction × odometry.
It is not an image-rate guarantee and can jump at graph corrections. A future
PoseStamped copy of it must have a distinct, explicitly documented map-pose
topic. Do not silently change the standardized local-pose topic's meaning.
See [CoreWrapper](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_slam/src/CoreWrapper.cpp).

Keep an **absolute, fresh private** database path, provisionally
`<private-runs>/rtabmap/slam005b_001/rtabmap.db`, with a separate GLIM dump and
recording directory if both later run. This is a naming proposal; no run has been
reserved. Refuse existing outputs; do not use the shared default `~/.ros/rtabmap.db`
or database-deletion flags. Expect native graph/data and a depth-derived colored
3D map when generated/requested; exported point clouds and trajectories require
their own native save/export validation. An empty DB file alone is not success.

## TF and synchronization contract for SLAM-005B

These are proposed settings to implement and validate against the installed
versions later; no RTAB-Map YAML or launch file was created in SLAM-005A.

| Component | Proposed settings / remapping |
|---|---|
| Both nodes | `use_sim_time=true`, `frame_id=base_link`, `publish_tf=false`; remap `/tf` → `/a300_00000/tf` and `/tf_static` → `/a300_00000/tf_static` for the existing robot sensor tree. |
| VO | `odom_frame_id=rtabmap_odom`; empty `guess_frame_id` / `ground_truth_frame_id`; no IMU initialization dependency. Pure visual registration, no automatic external-odom reset (`Odom/ResetCountdown=0`). |
| Mapper | `map_frame_id=rtabmap_map`, **`odom_frame_id=""`**, empty `odom_frame_id_init`, `subscribe_odom=true`, `odom_sensor_sync=false`; `odom` remapped explicitly to its own VO output. Enable depth/RGB input, incremental mapping and depth-derived 3D mapping; disable scan/scan-cloud subscriptions. |
| Adapter | Own local pose topic above; no broadcaster, covariance invention or EKF/controller connection. |

An explicit mapper `odom_frame_id` disables its odometry-topic subscription in
[CommonDataSubscriber](https://github.com/introlab/rtabmap_ros/blob/0.23.13/rtabmap_sync/src/CommonDataSubscriber.cpp).
Leaving it empty is essential when no RTAB-Map odometry TF is broadcast. No
`rtabmap_odom → base_link` or `rtabmap_map → rtabmap_odom` transform is required on
the operational tree. Pose-message frame names need not have a published TF.

One native behavior needs explicit validation: `CoreWrapper::odomUpdate()` still
attempts an odometry TF lookup for a nonzero data stamp, then falls back to the
message pose/stamp when unavailable, with a warning printed once. Disabling
`odom_sensor_sync` avoids additional TF-based sensor motion compensation; it does
not remove that odometry lookup. Match VO/image acquisition stamps, measure the
lookup wait/latency and verify the message fallback with the installed release.
Do not enable an operational TF broadcast to suppress the warning. If the fallback
cannot meet the stationary processing budget, report that blocker before tuning
waits or proposing any alternate frame arrangement.

Clearpath/EKF remains the sole sender of existing `odom → base_link`; the
controller's `enable_odom_tf=false` and existing GLIM viewer/TF isolation remain.
RTAB-Map source constructs broadcaster objects even with sending disabled, so
**idle TF publisher endpoints may appear**. Future checks must inspect actual
transform content/parent/child and sender GIDs, not mistake endpoint counts alone
for competing authority. The new camera's static joints belong to the existing
robot-state publisher, with unique frame names and one parent per link.

Start with raw image transport and the two native nodes' built-in RGB-D
synchronization. Exact synchronization is appropriate only if RGB, depth and
CameraInfo headers actually match. Otherwise measure offsets and choose a small,
justified approximate-sync bound; do not use unrestricted queues to conceal lag.
Optional `rgbd_sync` can later share the same pair via `RGBDImage` to both nodes.
Verify offered/requested image, CameraInfo and odometry QoS separately (reliable
subscribers cannot receive best-effort publishers). Use simulation acquisition
time throughout, including recordings and evaluation; reject time resets/gaps.

Proposed initial camera budget: **640×480 at 15 Hz**, subject to texture/depth
and resource validation. Aim for VO/local poses at accepted image rate; mapping
may run at a lower rate, initially about 1 Hz. Neither is a measured result or a
promise that stationary images produce new graph nodes. Featureless, repetitive,
occluded, out-of-range or badly synchronized views can cause visual tracking loss.

## Target hardware, resources and difficulty

The target **NUC13ANKi7** uses an i7-1360P, with four P cores, eight E cores and
16 threads; its actual installed RAM and sustained thermal/power limits still
need measurement. See the [Intel NUC 13 Pro product brief](https://download.intel.com/newsroom/2023/client-computing/Intel-NUC-13-Pro-Product-Brief.pdf).
This inspection instead sees **AMD Ryzen 7 7800X3D, 16 logical CPUs** in a Microsoft
VM, with 7515 MiB total/4445 MiB available RAM and zero used swap at the sampled
idle check. These host observations do not benchmark the NUC.

SLAM-004C measured GLIM alone alongside simulation: mean CPU 22.95% of one core,
peak RSS 213.5 MiB and minimum available RAM 1717.1 MiB. That short, slow trial
does not bound larger-map growth, RGB-D VO, loop closure or parallel operation.
RGB8 + float depth at 640×480×15 is **32,256,000 bytes/s (30.76 MiB/s)** before
copies, DDS, queues and map data. Queues, duplicate image subscriptions, camera
rendering and feature extraction add RAM/CPU/GPU load. Thread pools from multiple
libraries can oversubscribe cores; map/database growth and loop closure add bursts.

Simultaneous operation is a reasonable hypothesis to test, **not demonstrated**.
Measure camera-only, RTAB-only, then both stationary under separate approval:
per-process CPU/RSS, system available RAM/swap, real-time factor, acquisition-to-
output latency, queue growth, drops/gaps and tracking diagnostics. Use bounded
queues, avoid viewers/dense-map subscribers initially, and profile reduced image
size/rate only if needed. Retain a memory stop floor (at least 1 GiB available)
and stop on stalls, repeated errors or instability. Physical-camera timing/USB
bandwidth and NUC thermals need separate hardware validation.

Integration difficulty: **moderate**. Native VO/mapping nodes already exist;
the missing camera, correct optical geometry/registration, TF-free topic wiring,
dependency coexistence and measured compute headroom are the main work. This
does not include cross-algorithm alignment, switching, Nav2 or fault-tolerance proof.

## Exact next task: SLAM-005B

1. Recheck `adding-rtab`, preserved refs/hashes and pending planning changes.
   Address the library dependency check in a reviewed installation proposal;
   obtain installation approval, inspect the APT transaction and retain GLIM's
   libraries/settings. Verify actual installed packages/executables afterwards.
2. Create a separate project-owned RGB-D description extension and isolated
   setup copy referencing the existing Husky. Reuse the installed camera macro
   and native bridges. Choose/inspect the mount; expand statically, verify all
   base/camera/optical transforms and added collision geometry. Do not modify
   the protected setup, world or existing GLIM configuration or run generators
   against them.
3. Add a small isolated RTAB-Map launch/parameter set with the interfaces above.
   Add the minimal Odometry-to-PoseStamped adapter only; deterministic tests for
   preserved stamp/frame/base pose, invalid/lost input, namespace and absence of TF.
   Inspect resolved libraries, typed parameters and exact ROS remaps before runtime.
4. Request separate permission for a bounded **stationary** camera/simulation test
   at the previously verified open spawn (−4,3,0.3,yaw0), `generate=false`, RViz off.
   Measure images/depth/calibration/QoS/clock/optical geometry before VO startup.
5. Obtain separate RTAB-Map runtime approval. Run VO+mapping independently with
   GLIM/LiDAR feeds disconnected from its estimator inputs. Verify valid local
   base poses, native map/database save and TF sender isolation; record explicit
   loss/gap/latency/resource evidence. Camera frames stationary alone cannot
   establish moving visual-odometry quality or LiDAR-failure robustness.
6. Only after independent success, request a bounded simultaneous **stationary**
   comparison with unchanged GLIM; use fresh private outputs for both. No claim
   of independence under failure until an explicitly reviewed fault test.
7. Preserve results, gracefully stop and clean up; document unresolved items.
   No switching manager, Nav2 integration, frame alignment or automatic follow-on.

**Movement remains blocked.** SLAM-004C crossed its reviewed 1.25 m center region
and had a **1.029 s /0.092405 m shutdown tail**; its final stationary bookend was
not completed. Diagnose and verify control-side stop/disarm behavior, stale-input
handling and stop latency before proposing a new motion envelope. Simulator
shutdown alone is not an established stop mechanism. Keep the 0.5 m clearance
requirement and measured 0.10 m/s /0.10 rad/s command caps. This plan does not fix
that issue or authorize another driving experiment.
