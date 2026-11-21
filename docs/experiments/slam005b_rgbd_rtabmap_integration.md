# SLAM-005B — Independent RGB-D RTAB-Map integration

Date: 2026-11-21. Result: **independent stationary RGB-D odometry/mapping PASS,
with documented warnings and a teardown correction validated statically only**.
Branch `adding-rtab`; GLIM checkpoint `f2bad11511214a858d0e2dcf56b7016e5eb0e0d5`.
Original SLAM-005A planning changes remain untouched and uncommitted.

## Compatibility decision and approved installation

APT simulation for `ros-jazzy-rtabmap-odom` + `ros-jazzy-rtabmap-slam`, without
recommendations: **36 new packages, zero upgrades/removals**. Archive inspection
found an actual ABI hazard: existing GTSAM 4.3a1 and candidate ROS GTSAM 4.2 both
use SONAME `libgtsam.so.4`. Substituting candidate 4.2 into GLIM's loader path gives
**54 unresolved symbols**; normal GLIM resolution gives zero.

The user approved a **private installation** of the reviewed package archives at
`/home/feras/rtabmap_jazzy_isolated/slam005b_001`, instead of system APT/dpkg
installation. Core 0.23.14, ROS wrappers 0.23.13, GTSAM 4.2.0 and their dependencies
are scoped to native RTAB-Map processes by the project runner. No global linker
configuration or GLIM files/commands are changed. ELF relocation checks for the
core and both native executables pass with private 4.2; normal GLIM still uses
local 4.3a1 with zero unresolved symbols. Archive names, versions and SHA256 are
retained privately with the installation. The packages are not registered as
system dpkg installations.

## Camera and isolated simulation

The user approved bottom-screw mount **(0.35, +0.18, 0.35) m in base_link**,
RPY=(0,0,0), with +X forward/+Y left. A 30×30×107.741 mm fixed post supports the
existing Clearpath D435 model. Native Gazebo RGB-D renders at its color frame,
whose optical origin is **(0.3606,0.2125,0.3625) m**. Optical +Z maps to base +X,
optical +X to base −Y, optical +Y to base −Z. Attaching the sensor to the color
frame avoids confusing the physical model's 15 mm depth/color offset with the
single-viewpoint synthetic registered stream.

Nominal checks: camera body is 107.741 mm above all old robot collision parts;
the forward frustum clears them vertically by at least 46.052 mm. Camera/post
contact and post/top-plate support contact are intentional. The current Helios
has no collision body; this does not certify a physical bracket or camera
aperture. Existing collision parts and old link transforms remain identical.

New private setup: `/home/feras/clearpath_rgbd005b_001`. Original setup and warehouse
are untouched. Only the copied robot Xacro and three copied generated launch
files differ: the latter rebase absolute paths so robot-state publisher actually
uses the new copy. All sensor/controller/localization YAML bytes remain unchanged.
No Clearpath generator ran.

Camera: 640×480, 15 Hz, HFOV 1.25 rad, clip 0.3–8 m. Nominal square-pixel
fx=fy≈443.530008 px; actual CameraInfo matched, cx=320/cy=240, zero distortion.
Native floating depth is meters,
with invalid out-of-range samples allowed. Three GZ→ROS sensor-data-QoS streams:

| ROS topic | Type |
|---|---|
| `/a300_00000/sensors/camera_0/color/image` | `sensor_msgs/msg/Image` |
| `/a300_00000/sensors/camera_0/depth/image` | `sensor_msgs/msg/Image` |
| `/a300_00000/sensors/camera_0/color/camera_info` | `sensor_msgs/msg/CameraInfo` |

One CameraInfo describes the common projection. All three topics were observed
live. Optional lazy native point-cloud bridging supports
a short per-pixel XYZ/depth/RGB registration check without continuous cloud load.
The [native Gazebo RGB-D implementation](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc)
uses one depth renderer for color/depth, simulation acquisition stamps and the
configured optical frame. Runtime checks must confirm these properties on this host.

## Pipeline, outputs and static checks

Project package [rtabmap_husky_integration](../../src/rtabmap_husky_integration/package.xml)
contains camera Xacro, bridge/typed node configuration, two launch files and a
minimal pose adapter. [Isolation tools and commands](../../tools/rtabmap_isolated/README.md)
document reproducible provisioning and launch.

Namespace `/a300_00000/slam/rtabmap`; RGB-D `rgbd_odometry` publishes its own
`odom`, parent `rtabmap_odom`, child `base_link`. Mapper consumes that topic with
`odom_frame_id=""`, `subscribe_odom=true`, `odom_sensor_sync=false`. Both disable
TF sending and remap TF listeners to existing namespaced topics. No LiDAR, GLIM,
wheel/EKF odometry, IMU or Gazebo reference input is required. Sensor static TF
and simulation clock are shared. EKF remains responsible for `odom → base_link`.

Native mapper still tries missing odometry TF then falls back to the message
pose/stamp. `wait_for_transform=0.0` avoids blocking each camera callback on that
deliberately missing transform. Core's warning is once; the conversion layer can
also warn on each failed lookup. Record counts/latency; do not suppress the issue
by enabling conflicting TF.

The TF-free adapter publishes valid **local**, sensor-stamped base PoseStamped
messages on `/a300_00000/slam/rtabmap/pose`. It rejects wrong frames, invalid
quaternions/nonfinite values, duplicate/backward timestamps, clock-not-ready,
over-one-second-old or more-than-50-ms-future messages. It preserves native
odometry/diagnostics separately and fabricates no covariance/confidence. Native
map-corrected `localization_pose` remains in separate `rtabmap_map`.

Fresh run: `/home/feras/glim_ws/runs/rtabmap/slam005b_001`;
database `database/rtabmap.db`, scoped `logs/` and `ros_home/`. The reserved run is
now consumed; its saved DB must not be reused for another trial.
Existing DB/sidecars are refused; no delete-on-start or shared default DB.

Static results: isolated package build PASS; initially 11 adapter tests PASS,
finally **12** including duplicate-SIGINT teardown. All 10 existing GLIM C++ tests
passed with their required test-configuration environment (the first direct
invocation omitted it; diagnostic retained). New runtime used the initial adapter;
the cleanup-only correction was not tested in a second runtime.
native executable discovery and three-node launch construction without execution
PASS; database-overwrite rejection PASS. Native SDF conversion PASS, with only
the known custom `gz_frame_id` warnings (also present for old sensors). All 16
old collision parts, 37 old link transforms and both old sensor XML definitions
match; new total is 18 collision parts. GLIM's normal loader still resolves local
4.3a1 with no unresolved symbols; no GLIM simulation/runtime was launched.

## Runtime gates and limitations

The user separately approved private package installation, the mount, camera-only
simulation, then **one** stationary RTAB-Map run. Commands were executed as listed
in the [runner README](../../tools/rtabmap_isolated/README.md), at
(-4,3,0.3,yaw0), generate=false, RViz off, with task-owned teleop input paths
disabled. No GLIM or LiDAR timing pipeline ran. Images stayed active through
native saving; all task-owned processes were then stopped. Original settings,
past maps, recordings and SLAM-005A planning files remain preserved.

**No driving.** SLAM-004C's region abort and 1.029-second stopping tail remain
unresolved; simulator shutdown is not the emergency stopping mechanism. No
simultaneous GLIM test, Nav2 or switching manager is authorized here. Stationary
operation cannot establish moving visual-SLAM accuracy or a useful spatial map.

## Observed stationary results

| Check | Result |
|---|---|
| Camera-only observation | 20.56 wall seconds; 261 RGB /272 depth /292 CameraInfo messages. Observed 13.54 /14.11 /15.15 Hz in simulation time; monotonic headers. |
| Image formats | `rgb8`, `32FC1`, 640×480, little-endian; sampled finite depth fraction 43.76%, 0.923–7.757 m. |
| Sensor TF | Measured translation (0.3606,0.2125,0.3625) m and quaternion (−0.5,+0.5,−0.5,+0.5), as designed. |
| RGB-D registration | Three same-stamp cloud/image checks: RGB agreement 100%; native forward depth exactly matches depth image. See cloud-axis limitation below. |
| Visual odometry | 392 logged updates; valid native camera tracking (logged quality 832), no GLIM/LiDAR/wheel/reference input. |
| Standard pose | **379** PoseStamped messages, stamps **485.037–514.935 s**, observed **12.643 Hz**, `rtabmap_odom`, base pose. Finite positions, quaternion norm exactly one, strictly increasing stamps, zero observed position range. |
| Mapper | 29 logged mapping updates /29 observed MapData messages. Working/optimized graph had one node; 28 additional negative-weight stationary records were stored. |
| Native cloud | **2,411 finite XYZ points**, frame `rtabmap_map`, 77,152 bytes of PointCloud2 data. Predominantly visible floor; single stationary viewpoint. |
| Database | **5,402,624 bytes**, native version 0.23.14; read-only SQLite `integrity_check=ok`. Node/Data/Statistics rows 29 each, Link rows 28, Word rows 1020, Feature rows 24128. Image/depth/calibration blobs present in all 29 sensor records. No binary reload test. |
| Native shutdown | VO and mapping children exited cleanly; native database/long-term-memory save reported done. Launch wrapper exited 0. Adapter shutdown warning below. |
| Stillness | Wheel position ranges below 3×10⁻¹⁸ m, maximum reported linear speed below 1.5×10⁻¹⁸ m/s; no nonzero command was generated. |
| TF isolation | Native per-message sender checks identify only EKF for `odom → base_link` during the observation. Dynamic endpoints 4→6 due to two idle native broadcasters; static endpoints remain 2. No RTAB transforms, root TF transforms or conflicting parents were observed. |
| Cleanup | Camera bridge, optional registration bridge and simulator wrappers exited 0; read-only observers ended; host process inspection empty. |

Rates are **delivered observation rates**, not a no-loss guarantee. Best-effort
images had gaps; the concurrent Python observer received RGB at only 3.55 Hz
during RTAB operation while native VO continued at higher rate. These independent
subscriber counts are not a measurement of the native estimator's own input
loss. No loss-free transport claim is made.

### Resources and timing

| Native process | Mean /peak CPU, % of one core | Peak RSS |
|---|---:|---:|
| RGB-D odometry | 35.68 /43.90 | 237.62 MiB |
| Mapper | 11.14 /25.95 | 356.29 MiB |

Minimum system available RAM **1499.10 MiB** (1.46 GiB), above the 1 GiB stop floor;
peak swap use about **40 KiB**. Native logged VO processing mean/max
22.29/50.13 ms, delay 39.56/69 ms. Mapper processing mean/max 23.57/34.50 ms,
total reported delay 68.48/90 ms. The run wrapper lasted 38.50 wall seconds,
including startup and saving; the bounded observer lasted 32.07 wall seconds.
No watchdog fault, repeated estimator exception or growing pose backlog occurred.
This is the current Ryzen WSL host, not a NUC or simultaneous-SLAM benchmark.

### Warnings, corrections and evidence limits

- Two early Python image-observer attempts stopped on incompatible rclpy message
  metadata assumptions; retained diagnostics. The corrected observer completed;
  the simulation/camera were not restarted. Native C++ probe supplied sender GIDs.
- Gazebo's optional validation point cloud contains rendering-axis XYZ despite
  its optical header. Analytic conversion `[-Y,-Z,X]` was used **only offline** to
  compare with images. Native forward X equals image depth exactly. Remaining XY
  difference was at most 5.48 mm, consistent with a half-pixel rendering convention
  in sampled points. It is not proof of exact subpixel calibration. This cloud
  never entered the estimator or operational TF. RGB/depth share native rendering
  and exact acquisition stamps; sampled colors matched at every pixel tested.
- **278** native TF-lookup warnings occurred for the deliberately unbroadcast
  `rtabmap_odom → base_link`. Zero wait and the documented odometry-message
  fallback allowed continued mapping. Do not enable competing TF to hide them.
- Runtime accepted the deprecated `Grid/FromDepth` alias and chose PNG fallback
  for 32-bit depth compression. After the run, the new config was updated to
  canonical `Grid/Sensor='1'` and explicit `Mem/DepthCompressionFormat='.png'`,
  verified against installed 0.23.14 headers. The as-tested YAML is retained
  privately. These settings and adapter teardown fix have static validation;
  **no second runtime validation was performed**.
- Native VO/map saved cleanly, but the adapter received another SIGINT during
  `destroy_node`, causing KeyboardInterrupt /exit −2. Project cleanup now ignores
  subsequent SIGINT during teardown and handles external context shutdown.
  Its new deterministic duplicate-signal test passes; a future authorized
  stationary test must confirm clean live teardown.
- Late ROS CLI parameter dumps returned `Node not found` and were unavailable.
  Native startup logs confirm base/odometry/map frames, disabled VO TF,
  mapper topic-odometry selection, disabled sensor compensation and zero wait;
  launched typed configuration and native TF measurements provide the remaining
  checks. Live `/proc` library-map capture was not obtained; prelaunch relocation
  checks and the isolated runner selected the approved libraries.
- Known joystick absence, a controller-discovery startup retry, custom SDF
  `gz_frame_id` warnings, and idle command-timeout braking were recorded. The
  controllers activated; no motion test, simulator restart or SLAM retry occurred.

See [sanitized report](../../evidence/command_outputs/2026-11-21-slam-005b-rgbd-rtabmap.txt),
[structured evidence](../../evidence/command_outputs/2026-11-21-slam-005b-rgbd-rtabmap.json)
and [reusable native map preview](../../evidence/figures/SLAM-005B/README.md).
Large database/raw logs and the native cloud remain in the private run directory.

## Next steps, requiring review

1. Review the independent stationary result and its delivered-message losses,
   diagnostic-cloud axis discrepancy and native TF warning volume.
2. With separate approval, verify the teardown fix/canonical config in a fresh
   short stationary RTAB-only run; obtain actual live parameter/library snapshots
   early. Do not reuse the consumed DB or silently retry this experiment.
3. Prepare separate GLIM and RTAB process environments and fresh native outputs
   for a later parallel **stationary** test. Run GLIM in its original environment,
   never in RTAB's private GTSAM path. Recheck resources, TF authority, each
   estimator's input independence, latency and native saves before authorization.
4. Request explicit parallel runtime permission. No accuracy/switching/fault
   robustness claim follows from this stationary single-algorithm result.
5. Keep all moving experiments blocked until the SLAM-004C stopping issue has been
   corrected and verified separately. No automatic follow-on was started.
