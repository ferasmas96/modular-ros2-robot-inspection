# 07. Sensor Integration

**Status:** IMU-001B runtime validation and LiDAR-001A/B design and static integration are complete. LiDAR-001C verified the generic simulated cloud, ROS bridge, TF, IMU continuity, manual movement, and RViz display in the isolated combined setup. A transient wall-clock publication gap and WSL2 performance warnings remain. The model is not claimed to reproduce the physical RS-Helios-16P. The cloud has no sensor-authentic per-point timing; GLIM's inspected TimeKeeper synthesizes pseudo-times when the field is absent. GLIM runtime compatibility remains untested. SLAM-001A adds an explicit zero-time approximation to avoid relying on inferred scan timing. The active setup is unchanged.

## 1. Objective

The SEN-001 investigation inspected the existing Husky A300 sensor configuration and Clearpath/Gazebo sensor-generation path, then recommended a low-risk integration plan for a RoboSense RS-Helios-16P LiDAR, a MicroStrain GX5-25 IMU, and an RGB-D camera. That investigation was read-only. No simulation was launched, no generator was run, and the active setup was not changed. The later isolated IMU generation is documented under Task IMU-001A below.

## 2. System Configuration

Environment baseline: ROS 2 Jazzy, Gazebo Harmonic 8.15.0, and a Clearpath overlay on top of APT packages. Package version numbers are package-specific: active `clearpath_common`, `clearpath_description`, `clearpath_generator_common`, and `clearpath_sensors_description` are 2.9.19, while `clearpath_config` is 2.9.8 and the active `clearpath_generator_gz`, `clearpath_gz`, and `clearpath_simulator` are 2.9.4. The active robot configuration is `/home/feras/clearpath/robot.yaml`; it declares the `top_plate` attachment, but all sensor arrays and custom mount arrays are empty. The generated `/home/feras/clearpath/sensors/launch/sensors-service.launch.py` contains an empty launch description, and no generated sensor parameter files were found.

The Clearpath generator separates sensor description generation (`urdf_enabled`) from sensor launch/bridge generation (`launch_enabled`). Workspace description sources map supported sensor models to Xacro macros; the installed Gazebo generator creates bridge parameter files and sensor launch files for models it recognizes.

| Sensor area | Existing support inspected | Limitation for this project |
|---|---|---|
| 3D LiDAR | Clearpath Xacros include Gazebo `gpu_lidar` implementations for Velodyne, Ouster, Seyond, and Hesai. The installed bridge parameter generator maps Velodyne, Ouster, and Seyond to ROS `LaserScan` and `PointCloud2`. | Velodyne and Ouster have description/bridge/launch mappings. Seyond and Hesai are URDF-only by default; Hesai also lacks an installed bridge parameter mapping. No RoboSense/RS-Helios-16P model or mapping was found. None of these profiles establishes Helios-16P fidelity. |
| IMU | Clearpath has a generic Gazebo IMU macro and Microstrain configuration/bridge support. The generic macro publishes a Gazebo IMU topic at the configured name/frame and update rate. | The simulated macro is not a GX5-25 hardware model and does not establish hardware-specific noise, bias, alignment, or driver behavior. The generated Microstrain bridge map includes `data` and `data_raw`, while the inspected generic macro declares `data`; verify actual publishers before relying on `data_raw`. |
| RGB-D | Intel RealSense and ZED have RGB-D descriptions, and the installed generator contains image, camera-info, depth-image, and point-cloud bridging paths for RealSense. | The exact physical camera has not been selected. The OAK-D description exists in workspace sources, but the installed generator's RGB-D parameter map and depth-image bridge list do not include OAK-D; do not assume it has a complete ROS RGB-D interface. |

### Version and implementation caveat

The installed/source version combination is mixed, but that alone does not demonstrate incompatibility: Clearpath packages have independent version numbers, and the installed APT set itself uses different versions for different packages. The active model-map differences are concrete integration risks for specific models, not proof that all generation is incompatible. Do not run the generator as an inspection shortcut: its code can remove and recreate generated directories.

The installed `ros_gz_bridge` documentation lists conversions for `sensor_msgs/Imu` ↔ `gz.msgs.IMU` and `sensor_msgs/PointCloud2` ↔ `gz.msgs.PointCloudPacked`. This confirms that message conversion is available in principle, not that any sensor topics currently exist or publish at runtime.

## 3. Implementation or Inspection Procedure

Read-only inspection covered:

1. The active `robot.yaml`, generated sensor launch directory, and Clearpath sensor description/configuration sources.
2. The installed Gazebo generator's sensor launch and bridge parameter mappings.
3. Representative 3D LiDAR, IMU, and RGB-D Xacros.
4. The A300 chassis and top-plate mount frame definitions.
5. Official Gazebo Harmonic sensor documentation, Jazzy `ros_gz_bridge` documentation, Clearpath's A300 integration documentation, and GLIM project documentation.

The generated-data path implied by the inspected sources is:

```text
robot.yaml sensor entry
   ├── Clearpath description generator -> robot Xacro -> Gazebo sensor plugin
   └── Clearpath Gazebo generator -> per-sensor bridge parameters and launch
Gazebo sensor topic (gz.msgs.*)
   ├── ros_gz_bridge parameter_bridge -> ROS sensor_msgs/*
   └── ros_gz_image image_bridge -> ROS image topics
ROS PointCloud2 + IMU (after runtime validation) -> GLIM ROS 2 integration
```

The A300 has a `default_mount` frame at the top of the chassis, and the active configuration attaches a top plate. Clearpath's integration guide identifies `base_link` at the bottom of the chassis, with +X forward and +Y to the robot's left, and describes the PACS top plate's M5×0.8 mounting grid at 80 mm spacing. No sensor-specific mount coordinates or clearances are currently configured.

## 4. Results

### Design alternatives

| Approach | Benefits | Risks/effort | Assessment |
|---|---|---|---|
| Clearpath-generated sensors | Reuses existing robot configuration, URDF, TF and supported ROS–Gazebo bridge generation. Best fit for supported camera/IMU models. | Sensor-model coverage is incomplete; current installed/source version skew; the supported LiDAR models are not Helios-16P. | Reuse for the IMU and selected RGB-D model only after reconciling versions and verifying each generated/runtime topic. |
| Project-owned description extension | Keeps changes outside upstream repositories; allows a Gazebo `gpu_lidar` model with scan geometry set from verified RS-Helios-16P specifications and an explicit bridge mapping. | Requires maintaining the Xacro/launch/bridge integration and validating generated-robot inclusion, frames, performance, and data shape. | Recommended for a Helios-specific simulation model if no maintained, compatible third-party model is found. Do not label generic `gpu_lidar` output as hardware-faithful without validation. |
| Third-party sensor model/plugin | Could reduce initial modeling work if a maintained model already supports Gazebo Harmonic and suitable ROS 2 outputs. | Compatibility, licensing, topic/message conventions, update status, and sensor fidelity are unknown. No suitable RS-Helios-16P model was verified in this investigation. | Consider only after a source, license, Gazebo version, and end-to-end ROS topic test are reviewed. |

### Proposed sensor and mount plan

- **LiDAR:** Keep the requested RS-Helios-16P as the target physical device. No RoboSense model mapping was found in the inspected Clearpath sources. If no compatible existing model is identified, use a project-owned Xacro/Gazebo `gpu_lidar` extension with horizontal/vertical scan characteristics, range, update rate, and any modeled noise derived from a manufacturer source—not from a different Clearpath LiDAR's defaults. Prefer a rigid top-plate mount near the robot center, with unobstructed coverage; use the documented 80 mm hole grid to plan a bracket, then finalize coordinates only after checking the real unit, cable routing, and model clearances. The intended ROS interface is a stamped point cloud (`sensor_msgs/PointCloud2`), with a scan topic only if the downstream consumers need it.
- **IMU:** Use the Microstrain configuration only to describe the intended hardware interface. For simulation, use a Gazebo IMU sensor and bridge its `gz.msgs.IMU` output to `sensor_msgs/Imu`; do not claim GX5-25 noise or bias fidelity unless modeled and validated. A provisional position discussed for the IMU is `(x=0.20, y=0, z=0.26) m`. If this is relative to `base_link`, it is forward of the robot centerline and near the chassis upper plane; the intended parent frame, installed top-plate offset, bracket, and physical feasibility still need confirmation. Prefer a rigid, low-vibration location near the chassis center, and document the IMU frame orientation and axis convention.
- **RGB-D:** Select the exact physical camera before implementation. If it is compatible with the intended hardware, an Intel RealSense model is the most directly supported path found in the inspected Clearpath generator. Place it toward the front of the top plate with a clear forward view; derive final pose and downward angle from the selected camera's field of view and mounting hardware. Verify color/depth images, camera info, optical frames, and point-cloud output independently.

GLIM is a range-based mapping framework and its project documentation describes use with 3D LiDAR and RGB-D inputs. Before integrating it, confirm the selected GLIM ROS 2 package's exact topic/frame requirements and configure it against observed ROS topics; do not infer those interfaces from the algorithm overview alone.

### Recommended sequence

1. Confirm physical sensor part numbers and obtain authoritative RS-Helios-16P and GX5-25 specifications, including axes, rates, FOV/range, and mounting constraints; select the RGB-D camera.
2. Reconcile Clearpath 2.9.4 installed generator packages with the 2.9.19 workspace sources, and define a project-owned integration boundary. Back up the external active config and generated outputs before any later approved generation.
3. Add and validate one sensor at a time: begin with the generic IMU, then the selected RGB-D unit, and then the Helios-specific LiDAR description/bridge.
4. With a separately approved implementation/runtime task, inspect Gazebo topic names and message types, ROS topic names/types, publisher/subscriber counts, update rates, timestamps, frame IDs, TF connectivity, and `use_sim_time`.
5. Inspect image dimensions/encoding, point-cloud fields and density, IMU orientation/acceleration/angular-velocity behavior, and sensor response during manually controlled motion. Compare simulated outputs with vendor specifications and document what the simulation does not model.
6. Only after those checks, validate the resulting point-cloud/IMU inputs with GLIM. Keep ROS–Gazebo bridging, sensor simulation, and SLAM validation as separately diagnosable stages.

## 5. Verification

| Check | Result | Evidence/qualification |
|---|---|---|
| Active sensor arrays are empty | PASS | Read-only inspection of the five sensor arrays in the active `robot.yaml`. |
| Existing generated sensor launch is empty | PASS | The generated `sensors-service.launch.py` contains only an empty `LaunchDescription`; no sensor parameter files were present. |
| Clearpath sensor descriptions and bridge mappings inspected | PASS | Workspace Xacro/configuration and installed Gazebo generator sources inspected. |
| RoboSense RS-Helios-16P support in inspected Clearpath mapping | FAIL | No RoboSense model mapping found. This is a support gap, not a sensor runtime failure. |
| Required Gazebo/ROS sensor topics currently available | NOT TESTED in SEN-001 | At that investigation stage no simulation was launched and no sensors were configured; the later isolated IMU runtime test is documented under IMU-001B. |
| Sensor update rates, frame IDs, or data quality | NOT TESTED | Requires sensor implementation and runtime inspection. |
| Sensor mounting fit and exact transforms | NOT TESTED | No sensor geometry/coordinates are configured; physical clearances were not measured. |
| GLIM input compatibility with live sensor data | NOT TESTED | GLIM and sensor streams were not launched or integrated. |
| Vendor-specific model fidelity | NOT TESTED | Manufacturer data sheets and a validated model are still required. |

## 6. Problems and Solutions

- **No sensors are configured:** the active sensor arrays and generated sensor launch are empty. This is the expected current state; nothing was changed.
- **RoboSense is not mapped by Clearpath:** the inspected model list has no RS-Helios-16P entry. A Helios-specific integration must not be represented by substituting another LiDAR's physical model or defaults.
- **Installed/source package versions differ:** the installed Gazebo generator is older than the workspace sensor configuration/description sources, and their supported model mappings differ. Reconcile this before future generation; no generator was run.
- **Simulation fidelity is limited by the model:** Gazebo sensor output should be treated as a controllable synthetic stream, not equivalent to hardware timing, return behavior, noise, calibration, or driver output unless each is explicitly modeled and tested.
- **External product specifications and runtime behavior remain unverified:** this investigation established the local integration path and generic bridge conversions, not the physical sensors' exact parameters or simulated stream behavior.

## 7. Evidence

- [Sanitized static inspection results](../evidence/command_outputs/2026-11-14-sen-001-static-inspection.txt)
- No runtime log or screenshot was created for SEN-001 because that investigation did not launch the simulation or visualization. Runtime evidence from IMU-001B is recorded below.

## 8. Reproducibility

These are read-only inspection commands; they do not run the generator or start the simulator:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 pkg prefix clearpath_gz
ros2 pkg prefix clearpath_generator_gz
grep -nE '^(sensors:|  (camera|gps|imu|lidar2d|lidar3d):)' /home/feras/clearpath/robot.yaml
find /home/feras/clearpath/sensors -maxdepth 3 -type f -print
sed -n '1,80p' /home/feras/clearpath/sensors/launch/sensors-service.launch.py
```

Do not invoke Clearpath generator executables for read-only inspection; they may recreate generated configuration. Runtime topic/type/rate checks belong to a later, approved task after sensors have been configured.

## 9. Thesis Writing Notes

Describe this milestone as an integration feasibility and design investigation, not sensor implementation or SLAM validation. Separate (1) the manufacturer's physical sensor, (2) Clearpath's supported configuration/description, (3) Gazebo's synthetic sensor model, and (4) the ROS message bridge. State explicitly which parameters come from vendor data sheets and which are simulator approximations. Report topic, frame, timing, point-cloud, and IMU tests as measured results only after runtime evidence has been collected. Preserve the observed package-version skew and current absence of sensor topics as reproducibility limitations.

### References

- [Clearpath Husky A300 documentation](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300)
- [Clearpath Husky A300 integration and mounting guide](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/integration_husky)
- [Gazebo Harmonic sensors tutorial](https://gazebosim.org/docs/harmonic/sensors/)
- [ROS 2 Jazzy `ros_gz_bridge` documentation](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/)
- [GLIM documentation](https://koide3.github.io/glim/)
- Local implementation references: `/home/feras/clearpath_ws/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/description/sensors.py`, `/opt/ros/jazzy/lib/python3.12/site-packages/clearpath_generator_gz/param/sensors.py`, `/opt/ros/jazzy/lib/python3.12/site-packages/clearpath_generator_gz/launch/sensors.py`, and `/home/feras/clearpath_ws/src/clearpath_common/clearpath_sensors_description/urdf/`.

## Clearpath Version Compatibility and Sensor Generation Strategy

This subsection records the COMPAT-001 assessment before sensor generation. At that milestone it was read-only: no generator was executed, no ROS package was changed, and the active `robot.yaml` remained untouched. The subsequent isolated generation is documented under Task IMU-001A below.

### Package inventory and resolution

APT version suffixes are omitted in the table; `apt-cache policy` showed the installed version is also the current candidate for each listed package.

| Package | Installed APT version | Workspace source version | Active ROS 2 resolution |
|---|---:|---:|---|
| `clearpath_common` | 2.9.17 | 2.9.19 | `/home/feras/clearpath_ws/install/clearpath_common`, 2.9.19 |
| `clearpath_description` | 2.9.17 | 2.9.19 | `/home/feras/clearpath_ws/install/clearpath_description`, 2.9.19 |
| `clearpath_config` | 2.9.8 | 2.9.8 | `/home/feras/clearpath_ws/install/clearpath_config`, 2.9.8 |
| `clearpath_generator_common` | 2.9.17 | 2.9.19 | `/home/feras/clearpath_ws/install/clearpath_generator_common`, 2.9.19 |
| `clearpath_generator_gz` | 2.9.4 | Not present in workspace source | `/opt/ros/jazzy`, 2.9.4 |
| `clearpath_gz` | 2.9.4 | Not present in workspace source | `/opt/ros/jazzy`, 2.9.4 |
| `clearpath_simulator` | 2.9.4 | Not present in workspace source | `/opt/ros/jazzy`, 2.9.4 |
| `clearpath_sensors_description` | 2.9.17 | 2.9.19 | `/home/feras/clearpath_ws/install/clearpath_sensors_description`, 2.9.19 |

With the ROS Jazzy and workspace setup files sourced, Python resolves:

- `clearpath_config` from `/home/feras/clearpath_ws/build/clearpath_config/clearpath_config/` (source manifest version 2.9.8).
- `clearpath_generator_common` from `/home/feras/clearpath_ws/install/clearpath_generator_common/lib/python3.12/site-packages/` (2.9.19).
- `clearpath_generator_gz` from `/opt/ros/jazzy/lib/python3.12/site-packages/` (2.9.4).

Thus the active Gazebo generator imports the workspace `clearpath_generator_common` and `clearpath_config` modules while its own implementation is the APT-installed 2.9.4 module. The package manifests declare `clearpath_generator_gz` dependencies on `clearpath_config` and `clearpath_generator_common` without version bounds; workspace `clearpath_generator_common` declares `clearpath_config >= 2.9.0`, which the resolved 2.9.8 package satisfies. No declared dependency-version violation was found. Successful imports and mapping introspection are confirmed; end-to-end generation across this combination remains untested.

### Generation pipeline and side effects

`clearpath_gz/launch/simulation.launch.py` includes both `gz_sim.launch.py` and `robot_spawn.launch.py`. The `generate` argument defaults to `true`; therefore, `generate:=true` is not a generation-only command and also starts Gazebo. The robot-spawn launch sequences these executables before spawning the robot:

1. `clearpath_generator_common generate_description`
2. `clearpath_generator_common generate_semantic_description`
3. `clearpath_generator_gz generate_launch`
4. `clearpath_generator_gz generate_param`
5. Include generated platform and sensor launches, then spawn the robot.

Each generator receives `-s <setup_path>`. With the current launch default, the setup path is `/home/feras/clearpath`, and the input is `/home/feras/clearpath/robot.yaml`. `generate:=false` skips all four generators and uses the existing generated description, parameter, and launch files; the launch still parses the configuration to obtain the namespace and spawn settings.

Generated outputs include `robot.urdf.xacro`, semantic description output (`robot.srdf.xacro`/`robot.srdf`), and the `sensors`, `platform`, `platform-extras`, and `manipulators` launch/config directories. The description writer opens `robot.urdf.xacro` for writing. The launch-generator constructor removes and recreates generated launch directories, and the parameter-generator constructor removes and recreates generated parameter directories. A failure partway through generation can therefore leave a partial setup. The generators read `robot.yaml`; no inspected code writes that input file.

### Sensor compatibility findings

| Sensor path | Finding from active code | Compatibility assessment |
|---|---|---|
| Generic Gazebo IMU | The generic `BaseIMU` bridge parameter class exists, but the active workspace description map has no `base` IMU Xacro mapping. The concrete `microstrain_imu` model is mapped by both the workspace description generator and the installed Gazebo bridge/launch generator; its Xacro instantiates a generic Gazebo IMU rather than a GX5-25 hardware model. | **Supported path by static mapping** for the concrete Microstrain model; the generic `base` model is not confirmed end-to-end. Runtime output and generator execution are untested. |
| Generic Gazebo 3D LiDAR | The config model factory offers named models (Velodyne, Ouster, Seyond, and Hesai), not an arbitrary `BaseLidar3D`/RS-Helios model. The active description sources and installed bridge generator overlap for Velodyne/Ouster/Seyond, whose Xacros use generic Gazebo LiDAR simulation. Seyond is URDF-only/launch-disabled by default. Hesai has a description but no installed bridge parameter mapping and is also launch-disabled by default. | **Supported path by static mapping** for Velodyne/Ouster. Seyond has description and parameter mappings but no default launch path. None is a generic model selector or an RS-Helios-16P simulation. |
| Intel RealSense RGB-D | `intel_realsense` is present in the active description, bridge-parameter, and sensor-launch mappings. The generator has color/depth image, camera-info, and point-cloud paths. | **Supported path by static mapping**; actual generated topic names and runtime behavior are not tested. No version-induced conflict was identified for this mapped model. |
| Other workspace models | The workspace description mapping includes Hesai and Luxonis OAK-D; the installed Gazebo generator's model/launch mappings do not provide the same complete bridge path for them. | **Confirmed mapping gap** for these models in the active package combination; avoid selecting them through the native generator without a separate integration. |
| RoboSense RS-Helios-16P | No RS-Helios-16P configuration class, description macro, or bridge entry was found. | **Unsupported by inspected native mappings**, independent of whether the version numbers are aligned. |

These are source/mapping findings, not runtime failures. In particular, the mixed versions do not prevent the statically mapped Microstrain and RealSense paths from being represented by the generator code. No sensor was configured or generated to prove the complete output.

### Compared strategies and recommendation

| Strategy | Complexity | Simulation risk | Compatibility/reproducibility | Hardware transfer |
|---|---|---|---|---|
| **A. Keep the existing package set** | Low for statically mapped Microstrain, RealSense, Velodyne/Ouster | Lowest immediate risk; no package changes. Generation still replaces selected generated directories. | APT candidates are already installed; pin the actual package/source matrix and verify generated output in an isolated setup copy. | Good for supported physical drivers; Gazebo sensor models remain approximations. |
| **B. Align Clearpath packages** | Medium to high; requires selecting and testing a complete package release set | Higher risk to the working simulation; changing overlays or rebuilding can affect previously verified control, odometry, TF, and RViz. | APT itself is version-staggered (2.9.17, 2.9.8, 2.9.4); requiring identical version numbers would be incorrect. No newer APT candidate is available in the inspected package metadata. | May improve reproducibility if a tested release set is selected, but does not add RoboSense support automatically. |
| **C. Project-owned sensor extension** | Medium; own Xacro plus explicit ROS–Gazebo bridge launch | Avoids upstream source edits; still changes the robot description/config and needs careful isolated generation/testing. | `platform.extras.urdf` is an explicit Xacro-include hook. The installed Gazebo launch generator does not wire a `platform.extras.launch` entry into its generated service launch, so use a project-owned launch wrapper/bridge launch rather than assuming that hook works. | Best control over stable ROS topics and frames for the intended physical sensor; fidelity must be validated against vendor data. |

**Recommendation: a hybrid of A and C.** Keep the current package versions for the statically mapped Microstrain IMU and selected RealSense RGB-D path, with generation tested on a copy first. Use a project-owned Gazebo LiDAR description and bridge for the RS-Helios-16P rather than falsely labeling a Velodyne/Ouster profile as that hardware. Do not upgrade or rebuild now. Consider strategy B only if isolated generation demonstrates an API/runtime incompatibility or a later reproducibility requirement justifies testing a different complete release set.

### Safe implementation prerequisites and procedure

Do not modify the active configuration until the exact sensor models, vendor specifications, mounting frames, intended ROS topic names, and a project-owned extension boundary are approved. Preserve the active APT/overlay resolution shown above during the first test; do not change package versions while investigating sensor-generation behavior.

Files and outputs to protect:

- `/home/feras/clearpath/robot.yaml` (active input configuration; do not place a copy in Git or evidence).
- `/home/feras/clearpath/robot.urdf.xacro` and `/home/feras/clearpath/robot.srdf.xacro`/`robot.srdf`.
- `/home/feras/clearpath/sensors/`, `/home/feras/clearpath/platform/`, `/home/feras/clearpath/platform-extras/`, and `/home/feras/clearpath/manipulators/` generated launch/config outputs.
- Any later project-owned sensor Xacro and bridge launch files in the workspace.

For a future implementation, the safer first pass is to work from a private copy of the complete setup directory rather than the active one. At COMPAT-001, the following commands were a plan only and were not run; Task IMU-001A later used separate, clearly named private paths:

```bash
# Choose a new backup name and ensure the destination does not already exist.
cp -a /home/feras/clearpath /home/feras/clearpath.backup-COMPAT-001-20261114
cp -a /home/feras/clearpath /home/feras/clearpath-sensor-test

source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash

# After editing only the test copy's robot.yaml in an approved later task:
ros2 run clearpath_generator_common generate_description -s /home/feras/clearpath-sensor-test
ros2 run clearpath_generator_common generate_semantic_description -s /home/feras/clearpath-sensor-test
ros2 run clearpath_generator_gz generate_launch -s /home/feras/clearpath-sensor-test
ros2 run clearpath_generator_gz generate_param -s /home/feras/clearpath-sensor-test
```

After each generator returns successfully, inspect the test copy's generated Xacro, sensor bridge YAML, and sensor/platform launch files; check that required files are non-empty and that Xacro expands successfully. Only then, with approval to start the simulator, launch the copied setup with `generate:=false` so it cannot regenerate:

```bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath-sensor-test generate:=false
```

Verify the existing robot still spawns, accepts the established manual teleoperation path, and publishes the previously verified command, odometry, and TF interfaces. Separately verify each new sensor's Gazebo and ROS message types, topic endpoints, rates, frame IDs, timestamps, and `use_sim_time`; do not assume generation success proves runtime connectivity.

If any generator fails, stop the sequence and do not launch the partial setup. Keep the original active setup untouched, save the test-copy error output, and restore the active setup from the private snapshot only if it was changed:

```bash
cp -a /home/feras/clearpath.backup-COMPAT-001-20261114/. /home/feras/clearpath/
```

Keep the backup local with restrictive access because `robot.yaml` includes system/network configuration. Never commit it, copy it into evidence, or use an unresolved/wildcard path during rollback.

### Assessment and remaining risks

- The package version difference is **not a demonstrated general incompatibility**; package versions are independently released, the active imports resolve, and declared dependency minimums are met.
- A **real feature-mapping gap** exists for RS-Helios-16P and other unmapped models. A supported Gazebo sensor type does not imply hardware-specific fidelity.
- Generation itself is destructive to generated launch/config directories and overwrites robot descriptions; test only on a copy with a backup.
- During COMPAT-001, no version alignment, upgrade, rebuild, generator run, simulation launch, or sensor runtime validation occurred. IMU-001A later generated only in a private test copy; it did not launch a simulator.
- Next step: perform a separately approved runtime check of the generated `data` topic, without changing the existing EKF; integrate the project-owned LiDAR extension only in a later task.

## Task IMU-001A: Isolated IMU Configuration and Static Validation

**Date:** 2026-11-14

**Status:** PASS for isolated generation and static checks; NOT TESTED at runtime.

### Isolated setup and privacy

- Private pre-generation backup: `/home/feras/clearpath_imu001a_backup`
- Isolated test setup: `/home/feras/clearpath_imu001a_test`
- Both destinations were checked absent before creation. The active setup was copied to both directories before editing; their roots and all descendant directories/files were restricted to owner-only access (`0700` directories, `0600` files).
- The active `robot.yaml` includes host/IP and ROS domain configuration. Those values remain only in the private copies and are intentionally excluded from repository documentation and evidence.
- The test copy was edited and generated outside the Git workspace. The active setup compares byte-for-byte with the private backup after generation; the active `robot.yaml` checksum is unchanged.

### Selected model, schema, mount, and pose

The selected Clearpath model is `microstrain_imu`, from the installed `microstrain_imu.yaml` example and the active Microstrain description/bridge mappings. The example uses `model`, `urdf_enabled`, `launch_enabled`, `parent`, `xyz`, and `rpy`; the test copy uses those fields and does not add a driver-specific hardware configuration. The Microstrain configuration class defaults to the `gx5` family and a 100 Hz IMU data rate. This is a generic Gazebo IMU, not a hardware-faithful 3DM-GX5-25 simulation; no GX5-25 noise, bias, calibration, or hardware driver behavior is claimed.

The A300 description provides `base_link → chassis_link → default_mount → top_plate_link → top_plate_default_mount`. The top plate is attached to `default_mount`, and `top_plate_default_mount` is an available attachment frame. The discussed `(0.20, 0.00, 0.26) m` position was relative to `base_link`; the corresponding physical bracket and transform were not established, so those coordinates were not reused as if they were relative to the plate.

The test copy instead uses this explicitly provisional pose, expressed relative to `top_plate_default_mount`:

```yaml
sensors:
  imu:
    - model: microstrain_imu
      urdf_enabled: true
      launch_enabled: true
      parent: top_plate_default_mount
      xyz: [0.0, 0.0, 0.05]
      rpy: [0.0, 0.0, 0.0]
```

The IMU frame is `imu_0_link`; Clearpath also creates `imu_0_base_link`. The generic macro adds a further 2.5 mm fixed link offset. From the A300 description, the resulting `imu_0_link` origin is approximately 0.2948 m above `base_link`; this is a derived simulation pose, not a measured physical mounting position. No rate field was added to the YAML: the existing Microstrain class default was used and generated as 100 Hz.

Source references inspected:

- `/opt/ros/jazzy/share/clearpath_config/sample/microstrain_imu.yaml`
- `/home/feras/clearpath_ws/src/clearpath_config/clearpath_config/sensors/types/imu.py`
- `/home/feras/clearpath_ws/src/clearpath_common/clearpath_sensors_description/urdf/microstrain_imu.urdf.xacro`
- `/home/feras/clearpath_ws/src/clearpath_common/clearpath_generator_common/clearpath_generator_common/description/sensors.py`
- `/opt/ros/jazzy/lib/python3.12/site-packages/clearpath_generator_gz/param/sensors.py`
- `/opt/ros/jazzy/lib/python3.12/site-packages/clearpath_generator_gz/launch/sensors.py`

### Generation and outputs

Only `/home/feras/clearpath_imu001a_test/robot.yaml` was edited by hand. The generators received the test-copy path and did not target `/home/feras/clearpath`:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 run clearpath_generator_common generate_description -s /home/feras/clearpath_imu001a_test
ros2 run clearpath_generator_common generate_semantic_description -s /home/feras/clearpath_imu001a_test
ros2 run clearpath_generator_gz generate_launch -s /home/feras/clearpath_imu001a_test
ros2 run clearpath_generator_gz generate_param -s /home/feras/clearpath_imu001a_test
```

All four commands completed successfully. The description generator wrote `robot.urdf.xacro`; semantic generation wrote `robot.srdf.xacro` and `robot.srdf`; launch generation recreated generated launch directories; parameter generation recreated generated configuration directories.

Generated or edited files in the private test copy:

- `robot.yaml` — the sole manual configuration edit.
- `robot.urdf.xacro`, `robot.srdf.xacro`, `robot.srdf`.
- `sensors/config/imu_0.yaml`.
- `sensors/launch/imu_0.launch.py`, `sensors/launch/sensors-service.launch.py`.
- `platform/launch/platform-service.launch.py`.
- `platform/config/control.yaml`, `diagnostic_aggregator.yaml`, `diagnostic_updater.yaml`, `foxglove_bridge.yaml`, `imu_filter.yaml`, `localization.yaml`, `teleop_interactive_markers.yaml`, `teleop_joy.yaml`, and `twist_mux.yaml`.
- `manipulators/launch/manipulators-service.launch.py` and `manipulators/config/moveit.yaml`.

The parameter generator automatically added IMU entries to the test copy's `platform/config/localization.yaml`. To preserve the “do not modify the existing EKF” requirement, that one file was restored from the private pre-generation backup. The test-copy localization file now matches both the backup and active file; the IMU is not configured for EKF fusion. Diagnostic configuration files were generated with sensor entries. The generated launch/config directories may also have removed pre-existing Python cache files inside the test copy.

### Generated description and bridge

The generated description includes the Microstrain Xacro call with parent `top_plate_default_mount`, pose `(0, 0, 0.05)`, and `update_rate="100"`. Xacro expansion with namespace `a300_00000` and `is_sim:=true` succeeded; `check_urdf` parsed the robot and showed the connected chain through `top_plate_default_mount`, `imu_0_base_link`, and `imu_0_link`.

The expanded Gazebo sensor block is `type="imu"`, `name="imu_0"`, `always_on=true`, `update_rate=100`, and `gz_frame_id=imu_0_link`. The Xacro declares the Gazebo topic as `a300_00000/sensors/imu_0/data` (without a leading slash in the XML). Runtime Gazebo topic resolution has not been checked.

The generated `sensors/config/imu_0.yaml` contains these Gazebo-to-ROS mappings:

| Gazebo topic in bridge config | Gazebo type | ROS topic (resolved namespace) | ROS type | Static result |
|---|---|---|---|---|
| `/a300_00000/sensors/imu_0/data` | `gz.msgs.IMU` | `/a300_00000/sensors/imu_0/data` | `sensor_msgs/msg/Imu` | Configured; the Xacro declares the corresponding `data` sensor topic. |
| `/a300_00000/sensors/imu_0/data_raw` | `gz.msgs.IMU` | `/a300_00000/sensors/imu_0/data_raw` | `sensor_msgs/msg/Imu` | Bridge mapping generated, but no `data_raw` topic is declared by this Xacro. No publisher is established. |

The generated sensor launch starts `ros_gz_bridge/parameter_bridge` with simulation time and a static transform publisher. It does not start the MicroStrain hardware driver. The ROS `data` topic is therefore the expected simulated IMU interface; neither ROS topic has been observed at runtime.

### Verification and limitations

| Check | Result | Evidence |
|---|---|---|
| Private backup and isolated copy created; permissions restricted | PASS | Both copies exist outside Git with owner-only directory/file access. |
| Supported model and YAML schema | PASS | Installed Clearpath `microstrain_imu.yaml` sample and model class inspected; generation accepted the test configuration. |
| Intended parent frame, provisional pose, and sensor links | PASS | Generated Xacro and expanded URDF contain the parent and connected sensor links. Pose remains provisional. |
| Robot description/Xacro syntax | PASS | Xacro expansion and `check_urdf` succeeded. |
| Gazebo IMU sensor definition and 100 Hz rate | PASS | Expanded description contains `sensor type="imu"`, `imu_0_link`, topic, and update rate. |
| Bridge config and ROS IMU mapping | PASS | Generated config maps `gz.msgs.IMU` to `sensor_msgs/msg/Imu` for `data`; this is static configuration only. |
| `data_raw` publisher | NOT TESTED | The generator adds a bridge entry, but the inspected Xacro declares only `data`. |
| Active setup, active EKF, and Clearpath source repositories unchanged | PASS | Active setup matches the private baseline; active localization matches the backup; nested source repositories are clean. |
| Gazebo sensor topic, ROS message flow/rate/frame/timestamps, and visual pose | NOT TESTED at the end of IMU-001A | Gazebo was not launched in that task; runtime results are documented under IMU-001B. |
| GX5-25 noise, bias, calibration, and hardware fidelity | NOT TESTED | The selected model is a generic Gazebo IMU. |

The semantic-description generator emitted a non-fatal warning that `imu_0_link` has visual geometry but no collision geometry; this matches the inspected generic Xacro. An initial validation grep assumed a different XML attribute order and a leading slash in the Xacro topic; the expanded XML was inspected and the checks were corrected. The generators themselves did not fail. The first shell invocation also stopped before starting a generator because `set -u` conflicted with an unset variable in the ROS setup script; rerunning without nounset succeeded.

**Runtime readiness at the end of IMU-001A:** The isolated setup was ready for a separately approved Gazebo test of the simulated `data` interface. The results of that later test are documented under IMU-001B; do not rely on `data_raw` or EKF fusion.

Sanitized command results are saved in [IMU-001A static validation evidence](../evidence/command_outputs/2026-11-14-imu-001a-static-validation.txt). The evidence omits private host, address, and domain configuration. During IMU-001A no simulator or RViz2 was launched, and no package installation, build, active setup modification, or EKF parameter change was performed.

## IMU-001B: Runtime Validation of Simulated IMU

**Date:** 2026-11-14

**Overall result:** PASS for the generic simulated `data` stream and manual-motion response; NOT TESTED for GX5-25 hardware fidelity or `data_raw`.

### Launch and startup

After checking the isolated files, backup, active setup, test-copy EKF, and process list, the user approved this exact launch. No generator was run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_imu001a_test \
  generate:=false
```

PASS — the `warehouse.sdf` world loaded, Gazebo advertised the robot IMU, the Husky was visible to the user, and the controller manager loaded and activated `joint_state_broadcaster` and `platform_velocity_controller`. The generated bridge node `/a300_00000/sensors/imu_0_gz_bridge` and static-transform node `/a300_00000/imu_0_static_tf` appeared in the ROS graph.

The first ROS graph query ran before discovery had settled and returned no nodes. Subsequent queries using the test setup's configured ROS domain showed the expected nodes/topics. The configured domain value is not recorded here or in evidence.

### Gazebo topic and ROS bridge

| Interface | Runtime observation | Result |
|---|---|---|
| Gazebo IMU topic | `/a300_00000/sensors/imu_0/data` | PASS — listed by `gz topic`, type `gz.msgs.IMU`, one publisher and one bridge subscriber; Gazebo log says the sensor advertised this topic. |
| ROS IMU topic | `/a300_00000/sensors/imu_0/data` | PASS — type `sensor_msgs/msg/Imu`; one publisher, `imu_0_gz_bridge` in `/a300_00000/sensors`. |
| Bridge direction | Gazebo to ROS | PASS — generated parameter bridge connected to the Gazebo IMU and published the ROS message. |
| `data_raw` | `/a300_00000/sensors/imu_0/data_raw` | NOT PUBLISHED — Gazebo reported zero publishers and one subscriber (the bridge). A ROS publisher endpoint exists from the bridge configuration, but no raw messages were established. |

### Message content, time, rate, and TF

ROS messages had the expected `header.frame_id: imu_0_link`. Gazebo and ROS timestamps advanced with the simulation clock. The bounded ROS echo samples had stamps `100.752 s` and `103.761 s`; a later capture contained **4,404** messages over **44.028 simulation seconds**, with strictly increasing stamps and no non-finite numeric values. Its mean simulation-time interval was **10.000 ms**, minimum **9 ms**, and maximum **12 ms**.

The bounded `ros2 topic hz` check reported **99.985 Hz** average over a 505-sample window; reported interval extrema were **5–16 ms**. It initially warned that the topic was not yet published, then returned regular measurements. This is a short WSL2 run, not a long-term benchmark.

Stationary samples at the start and end of the motion capture showed angular velocity effectively zero, linear acceleration approximately `(0, 0, +9.8) m/s²`, and a populated, near-unit orientation quaternion. All three covariance arrays were zero; these values do **not** establish physical sensor accuracy. The generic Xacro sets no alternate ENU/NED orientation conversion; the observed orientation is Gazebo's simulated sensor attitude, not a validated GX5-25 convention.

With explicit namespaced TF remappings, `tf2_echo base_link imu_0_link` resolved a static transform of approximately `(0, 0, 0.295) m` and identity rotation. The first lookups waited for TF data; the connected transform then resolved. `odom → base_link` also resolved on the namespaced TF topics.

### Manual movement and regression

The user confirmed the Husky was visible and manually drove W, then turned with A, and stopped. During a separately captured repeat, the IMU responded:

- `angular_velocity.z` reached approximately **+0.475 rad/s**.
- Horizontal linear acceleration reached approximately **11.63 m/s²** (mostly X); it returned near zero after stopping.
- Orientation changed during the movement. The captured yaw derived from the quaternion changed from about **0.926 rad** at capture start to **−2.437 rad** at capture end; these are observed simulation values, not an accuracy comparison.
- Final raw and filtered odometry agreed at approximately `(2.724, 1.916, 0) m`; final linear and angular velocities were near zero. The user reported the robot stopped after releasing the controls.

Regression checks: PASS — the simulation clock was available; raw and filtered odometry each had a publisher; namespaced dynamic/static TF topics were active; the robot remained manually controllable. The runtime EKF retained `odom0: platform/odom`; its `imu0` parameter was not set, and the test-copy localization file still matches the active baseline. No IMU fusion was enabled. The active setup matched its private backup after the run.

### Warnings, limitations, and shutdown

- Gazebo warned that `gz_frame_id` is not a standard SDF element and copied it as an extension. Despite that warning, the ROS message frame ID was observed as `imu_0_link`.
- The controller warned its 20 ms update period was slower than the 3 ms Gazebo period. No parameters were changed.
- The configured PS4 joystick device was unavailable; the user successfully used the existing keyboard controls. The `ros2 control` CLI extension was unavailable, so controller activation was verified from launch logs and the live ROS graph instead.
- The controller repeatedly logged command-timeout/braking warnings while no command was held. These were consistent with the stationary intervals.
- The launch wrapper exited with status 143 after `SIGTERM` but left child processes. Each remaining process was checked against the isolated test launch and stopped by its specific PID; a final process check found no Gazebo or test nodes running. No unrelated process was signaled.
- No screenshot was captured. The user confirmed the Husky was visible in Gazebo.
- The simulated stream is a generic Gazebo IMU. Noise, bias, calibration, GX5-25 hardware behavior, and longer-duration timing/performance remain unverified.

### Reproduction and evidence

Use the isolated setup only; the required launch arguments are:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_imu001a_test \
  generate:=false
```

- Sensor description: `/home/feras/clearpath_imu001a_test/robot.urdf.xacro`
- Bridge parameters: `/home/feras/clearpath_imu001a_test/sensors/config/imu_0.yaml`
- Generated sensor launch: `/home/feras/clearpath_imu001a_test/sensors/launch/imu_0.launch.py`

Sanitized runtime measurements are in [IMU-001B command evidence](../evidence/command_outputs/2026-11-14-imu-001b-runtime-validation.txt) and [the concise runtime log](../evidence/logs/2026-11-14-imu-001b-runtime.log). Raw network-bearing diagnostics and the full motion capture are not copied into Git.

**GLIM readiness:** The validated ROS IMU stream is a candidate for a later GLIM interface test, but the system is not ready for full GLIM integration: the target RS-Helios-16P LiDAR is not yet integrated, and GLIM frame/topic/convention compatibility remains to be tested.

## LiDAR-001A: RoboSense RS-Helios-16P Simulation Design

**Date:** 2026-11-15
**Status:** Research and plan only. No sensor, configuration, source, or runtime changes were made.

### Manufacturer specifications and verification limits

The official RoboSense downloads page lists a Helios 16 product as a 16-beam LiDAR and lists a Helios 16 User Guide (English, 2023-11-30). The page's download action requires completing RoboSense's contact form; no contact information was submitted and the guide itself was not obtained. RoboSense's own SDK explicitly recognizes the distinct `RSHELIOS_16P` model and its decoder defines 16 laser channels. Therefore the channel count is verified for the target model; the following numeric sensor specifications could not be verified from accessible model-specific manufacturer documentation in this investigation.

| Specification | Verified finding |
|---|---|
| Laser channels | **16** — the RoboSense SDK's `RSHELIOS_16P` decoder sets laser number to 16. The manufacturer's download page also describes Helios 16 as 16-beam. |
| Horizontal field of view | **Not verified** for RS-Helios-16P. The SDK's configurable output start/end angles are not a physical sensor-FOV specification. |
| Vertical field of view and per-channel elevation angles | **Not verified.** Do not assume evenly spaced vertical channels or reuse another Helios model's angles. |
| Scan frequency | **Not verified.** |
| Horizontal angular resolution | **Not verified.** |
| Minimum/maximum detection range | **Not verified.** The SDK decoder's distance decode bounds are implementation limits, not a manufacturer detection-range claim. |
| Point rate | **Not verified.** Do not estimate without confirmed scan rate, angular sampling, and operating mode. |
| Coordinate convention | The RoboSense decoder's code-level spherical conversion is `x = r cos(elevation) cos(azimuth)`, `y = -r cos(elevation) sin(azimuth)`, `z = r sin(elevation)`, followed by channel corrections and optional user-configured transform. The accessible manufacturer material did not provide a named REP-103 convention; axis alignment and handedness must be checked against the exact user guide/device output before fixing the ROS frame orientation. |
| Data format and timing | RoboSense's SDK supports ROS `sensor_msgs/PointCloud2` output, including XYZI and XYZIRT point layouts (XYZ, intensity, ring, timestamp). Its mechanical-LiDAR parameters support selecting the LiDAR clock or host clock and stamping a cloud at its first or last point. These are SDK capabilities, not a promise that every device setting/output has the same timestamp units or semantics; verify them against the exact model manual and driver version. |
| Multiple returns | The RS-Helios-16P SDK decoder recognizes dual return and single-return strongest, last, or nearest modes. The active mode is read from device DIFOP data; no single mode is asserted here. |

These are deliberately conservative findings: only the 16-channel count is treated as a verified physical scan specification. The RoboSense decoder's packet encoding and range-decoding constants are not substituted for the manufacturer's performance specifications. Obtain the exact model/revision manual and confirm all scan-profile values before encoding a numerical simulation profile.

### Reusable Gazebo implementation

The installed Clearpath description has no RoboSense/RS-Helios-16P model or mapping. Its existing Velodyne and Ouster 3D LiDAR descriptions use Gazebo Harmonic `gpu_lidar`, with configurable horizontal/vertical angles and samples, update rate, range, sensor frame, and topic. This is sufficient to create a **generic ray-based 3D point-cloud simulation** without a custom Gazebo plugin.

Gazebo Harmonic's `GpuLidarSensor` implementation publishes a `gz.msgs.PointCloudPacked` topic formed by appending `/points` to the sensor's configured base topic. Its point-cloud fields are XYZ (`FLOAT32`), intensity (`FLOAT32`), and ring (`UINT16`); the implementation does not emit a per-point time field. `ros_gz_bridge` supports conversion from `gz.msgs.PointCloudPacked` to `sensor_msgs/msg/PointCloud2`, and its Jazzy demo uses this mapping for `/lidar/points`.

The Gazebo model can match a verified 16-ray vertical count and approximate the confirmed elevation envelope if RoboSense documentation confirms a uniform pattern. Gazebo's GPU ray grid uses regular angular increments, however, so nonuniform manufacturer channel angles cannot be represented exactly by this built-in sensor. Its rendering-derived intensity, one return per ray, and absent per-point firing timestamps are not hardware-faithful RS-Helios-16P behavior. It is appropriate for ROS/GLIM interface and mapping-pipeline development, not for validating the RoboSense driver, packet format, multi-return behavior, or motion distortion.

### Proposed isolated integration

- **Preserve the known-good IMU setup:** make a new owner-only copy of `/home/feras/clearpath_imu001a_test`, for example `/home/feras/clearpath_lidar11b_test`, and a separate pre-change backup. In Task 11B, first verify both destinations do not exist; never overwrite a prior test or backup. Do not modify the existing IMU test directory or `/home/feras/clearpath`.
- **Description ownership:** add a project-owned Xacro macro (planned package resource: `src/husky_a300_sensor_extensions/urdf/rs_helios_16p.urdf.xacro`) using `gpu_lidar`. Integrate it only into the copied setup's generated robot Xacro, with `generate:=false`; do not add an unsupported Clearpath model to `robot.yaml` or run Clearpath generators.
- **Mount and frame:** parent the fixed mount to the already-used `top_plate_default_mount`; define child frame `helios_16p_link`. Leave xyz/RPY provisional until the exact sensor mechanical drawing, bracket, top-plate clearances, and measured RoboSense frame convention are reviewed. Keep the point-cloud frame ID equal to this link and verify its static TF connection to `base_link`.
- **Interfaces:** set the Gazebo GPU-LiDAR base topic to `/a300_00000/sensors/helios_16p`; expect a `gz.msgs.LaserScan` topic at that base name and `gz.msgs.PointCloudPacked` at `/a300_00000/sensors/helios_16p/points`. Bridge the point-cloud endpoint to ROS `/a300_00000/sensors/helios_16p/points` as `sensor_msgs/msg/PointCloud2`. The LaserScan bridge is optional and not the GLIM input. The ROS cloud must carry `header.frame_id: helios_16p_link`, simulation-time header stamps, and at least valid XYZ fields; inspect the actual fields instead of assuming bridge conversion preserves them.
- **GLIM compatibility:** GLIM's ROS 2 node subscribes to `sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Imu`. Its converter requires XYZ coordinate fields (`FLOAT32` or `FLOAT64`) and accepts optional intensity and ring fields; it recognizes point-time fields named `t`, `time`, `time_stamp`, or `timestamp`. It reads `UINT32` times as nanoseconds and `FLOAT32`/`FLOAT64` as seconds. The ROS 2 callback consumes IMU linear acceleration and angular velocity; it does not use the IMU orientation field. GLIM reads cloud and IMU header stamps, auto-detects message frame IDs when not configured, and requires a correct `T_lidar_imu` transform (defined in its config as transforming IMU-frame points into the LiDAR frame). The ROS 2 callback invokes `TimeKeeper::process` before cloud preprocessing. When the cloud has no recognized point-time field, the inspected TimeKeeper fills pseudo-times using an estimated scan interval and point order rather than rejecting the cloud. The preprocessor then sorts times and establishes scan end from the last offset. With `global_shutter_lidar=true`, it zeros those times and disables deskewing. This is a source-level compatibility path, not runtime validation or a model of real scan timing. Use the observed/static transform between `imu_0_link` and `helios_16p_link`; do not estimate the extrinsic from the provisional mount pose.
- **Point timing limitation:** Gazebo's output has one cloud-level simulation timestamp but no point-level acquisition times. Without a time field, this GLIM revision may infer pseudo-times from frame intervals and point order; those values do not represent sensor firing. For reproducible instantaneous simulation, the project-owned adapter in SLAM-001A supplies explicit zero-valued relative times, and a future GLIM test should enable `global_shutter_lidar` to disable deskewing. The adapter does not reproduce the real spinning scan, point firing times, or motion compensation. The zero-time adapter is an explicit approximation, not a prerequisite proven necessary for GLIM message parsing.
- **ROS–Gazebo bridge:** keep bridge configuration and launch ownership in the project package (planned `config/rs_helios_16p_bridge.yaml` and `launch/rs_helios_16p_bridge.launch.py`) and launch the bridge explicitly alongside the copied Clearpath simulation. Do not change upstream Clearpath files.

### Task 11B files, sequence, and rollback

Planned project-owned files: `src/husky_a300_sensor_extensions/package.xml`, `CMakeLists.txt`, `urdf/rs_helios_16p.urdf.xacro`, `config/rs_helios_16p_bridge.yaml`, and `launch/rs_helios_16p_bridge.launch.py`. The copied setup would receive only the Xacro include/invocation needed to attach the new macro; the existing IMU Xacro, bridge config, launch, localization/EKF, and active setup remain protected. Build or generate changes are not authorized by LiDAR-001A; the later implementation task must specify and approve them separately.

1. Before implementation, obtain the exact Helios-16P revision/manual, confirm channel angles/FOV, scan mode/rate, angular sampling, range, point rate, timing semantics, return mode, and coordinate axes. Do not fill missing simulation parameters with OS1/Velodyne values.
2. Record hashes or backups of the working IMU test, create the new combined copy with owner-only permissions, and confirm the selected project package path and all destinations before writing.
3. Implement the project-owned Xacro/bridge only; use verified manufacturer geometry where available and label any unavoidable approximation. Do not fabricate physical firing times. Before a separately approved GLIM runtime test, choose either the explicit zero-time adapter with global-shutter mode or GLIM's pseudo-time fallback, and document that the latter is not physical timing.
4. Perform static Xacro/URDF validation first. Then, in a separately approved bounded runtime task, launch only the combined copy with `generate:=false`; inspect Gazebo/ROS message types, cloud fields/density/rate/stamps, frame ID/TF, bridge endpoints, IMU continuity, odometry, and manual robot control.
5. Test GLIM only after both streams, fields, stamps, QoS, frame IDs, and measured extrinsic are verified. The inspected GLIM revision can synthesize pseudo-times when a time field is absent; prefer the explicit zero-time adapter for an instantaneous test, and do not treat either pseudo-times or zero offsets as physical scan timing. No GLIM integration, LiDAR generation, or runtime test was performed in LiDAR-001A.
6. Roll back by stopping the isolated test, deleting no pre-existing files, and restoring only the new combined test directory from its verified backup. Reconfirm the original IMU test and active `/home/feras/clearpath` still match their pre-test baselines.

### Assessment and remaining risks

| Item | Result |
|---|---|
| Reuse GPU LiDAR for a generic 3D point cloud without custom plugin | **PASS** — Gazebo Harmonic publishes packed cloud data; Jazzy bridge conversion exists. |
| Exact RS-Helios-16P scan geometry from accessible official numerical documentation | **PARTIAL / BLOCKED** — 16 channels verified; other physical scan numbers are not verified. Do not finalize sensor values until the model-specific manual is reviewed. |
| GLIM message interface | **NOT RUNTIME TESTED** — GLIM subscribes to PointCloud2 + Imu; the inspected TimeKeeper synthesizes pseudo-times if the point-time field is absent. Frames, extrinsic, QoS, and runtime ingestion remain unverified. |
| GLIM motion compensation with stock GPU LiDAR output | **NOT SUPPORTED BY SENSOR DATA** — the cloud has no physical per-point times. GLIM's pseudo-times are inferred, and global-shutter mode disables deskewing. Explicit zero-time input permits an instantaneous interface/mapping test only. |
| Hardware fidelity | **NOT TESTED / NOT CLAIMED** — scan firing, exact channel angles, calibrated intensity/range noise, return behavior, packet timing, and motion distortion are not represented faithfully. |

### Sources

1. RoboSense, [Helios Series downloads](https://www.robosense.ai/en/resources-81) — lists the Helios 16 guide and 16-beam product; the guide download is contact-form gated.
2. RoboSense, [rslidar_sdk README](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/README.md) and [parameter introduction](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/intro/02_parameter_intro.md) — model support, XYZI/XYZIRT outputs, ROS PointCloud2, and SDK timing parameters.
3. RoboSense, [`RSHELIOS_16P` decoder source](https://github.com/RoboSense-LiDAR/rs_driver/blob/897b14d3bdb6186a75df27ba51b65b5bd5557723/src/rs_driver/driver/decoder/decoder_RSHELIOS_16P.hpp) — channel count, coordinate calculation, per-channel timestamps, and return-mode handling.
4. Gazebo Harmonic, [sensor tutorial](https://gazebosim.org/docs/harmonic/sensors/) and [`gz-sensors8` GPU LiDAR implementation](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/GpuLidarSensor.cc) — scan configuration, `/points` topic, packed point-cloud fields, and lack of per-point time.
5. ROS 2 Jazzy, [`ros_gz_bridge` supported conversions](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/) and [GPU LiDAR bridge example](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim_demos/launch/gpu_lidar_bridge.launch.py).
6. GLIM, [`config_sensors.json`](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/config/config_sensors.json), [ROS PointCloud2 converter](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/include/glim/util/ros_cloud_converter.hpp), [TimeKeeper](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/util/time_keeper.cpp), [cloud preprocessor](https://github.com/koide3/glim/blob/1a197c999e4865f5cab7c9cd25474128413ebbca/src/glim/preprocess/cloud_preprocessor.cpp), and [ROS 2 input callbacks](https://github.com/koide3/glim_ros2/blob/4d4ec524ccf1b02aa09b0af2af767ecc54343798/src/glim_ros/glim_ros.cpp); [GTSAM Points `add_times`](https://github.com/koide3/gtsam_points/blob/8e94654558dfbe7ddb0341de300daaf17e25ee02/src/gtsam_points/types/point_cloud_cpu.cpp).

Sanitized research and local implementation inspection are recorded in [LiDAR-001A evidence](../evidence/command_outputs/2026-11-15-lidar-001a-simulation-design.txt).

## LiDAR-001B: Project-Owned LiDAR Extension and Static Validation

**Date:** 2026-11-15
**Status:** Static validation only. Gazebo and GLIM were not launched.

### Package and isolated setup

Created the project-owned ROS 2 `ament_cmake` package `husky_a300_sensor_extensions` under `src/`. Its files are:

- `package.xml`
- `CMakeLists.txt`
- `urdf/rs_helios_16p.urdf.xacro`
- `config/rs_helios_16p_bridge.yaml`
- `launch/rs_helios_16p_bridge.launch.py`

The package is not excluded by the workspace `.gitignore`; no ignore rules or third-party Clearpath repositories were changed. The maintainer contact in `package.xml` is a reserved example address and must be replaced before publication.

Created the private combined setup `/home/feras/clearpath_lidar11b_test` and baseline backup `/home/feras/clearpath_lidar11b_backup`; both are owner-only. The test copy was made from `/home/feras/clearpath_imu001a_test`. Its `robot.yaml` uses Clearpath's supported `platform.extras.urdf` package/path extension hook to include the new package Xacro. `generate_description` was run only on the test copy to regenerate its `robot.urdf.xacro`. The copied platform and IMU launch files contained absolute paths to the previous IMU setup; those paths were retargeted to the new copy so `generate:=false` uses the combined description and its own IMU config. Only paths changed in those launch files; the IMU and EKF parameters are unchanged. The working active setup and validated IMU setup remain unchanged.

### Generic model, mounting, and topics

**The simulated LiDAR is a generic 16-channel Gazebo GPU-LiDAR model. Its scan characteristics have not yet been validated against the physical RoboSense RS-Helios-16P.**

The Xacro macro exposes horizontal sample count and angle limits, vertical angle limits, update rate, and range limits/resolution. The vertical sample count is fixed at the verified 16 channels. The one instantiated static-test profile uses synthetic, non-manufacturer values:

| Parameter | Static-test value | Origin/status |
|---|---:|---|
| Horizontal samples | 360 | Temporary synthetic value |
| Horizontal angular limits | -pi to +pi rad | Temporary synthetic 360-degree span |
| Vertical samples | 16 | Verified channel count; not an angle profile |
| Vertical angular limits | -0.261799 to +0.261799 rad | Temporary synthetic 30-degree span |
| Update rate | 10 Hz | Temporary synthetic value |
| Range limits | 0.1 to 20.0 m | Temporary synthetic values |
| Range resolution | 0.01 m | Temporary synthetic value |
| Mount pose in `top_plate_default_mount` | xyz `[0, 0, 0.10]` m; RPY `[0, 0, 0]` rad | Provisional static-test pose only |

The fixed joint connects parent `top_plate_default_mount` to child `helios_16p_link`. The existing IMU remains on its original `top_plate_default_mount` branch and retains its generated `imu_0_link`.

Expected Gazebo base topic: `/a300_00000/sensors/helios_16p`; expected packed point-cloud topic: `/a300_00000/sensors/helios_16p/points`, `gz.msgs.PointCloudPacked`. The bridge maps that endpoint to ROS `/a300_00000/sensors/helios_16p/points`, `sensor_msgs/msg/PointCloud2`, in the GZ-to-ROS direction. Expected cloud frame ID is `helios_16p_link` (runtime header not verified). The bridge launch defaults `use_sim_time` to true.

### Static validation results

| Check | Result |
|---|---|
| Targeted build of `husky_a300_sensor_extensions` | **PASS** — `colcon build --packages-select husky_a300_sensor_extensions` |
| Isolated Clearpath description generation | **PASS** — `generate_description -s /home/feras/clearpath_lidar11b_test`; no generator was run against either protected setup |
| Xacro expansion using the installed package and test copy | **PASS** |
| Expanded URDF parsed by `check_urdf` | **PASS** — one connected Husky tree containing the LiDAR and original IMU links |
| Parent/child frames, fixed mount, `gpu_lidar`, 16 vertical samples, configured base topic | **PASS** — statically inspected in expanded URDF |
| Bridge YAML syntax and exact ROS/Gazebo message mapping | **PASS** |
| Bridge launch-file parse | **PASS** — `ros2 launch ... --show-args`; this did not start the bridge |
| Copied platform/IMU launch paths | **PASS** — launch files parse and no longer reference the old IMU test directory |
| Active `/home/feras/clearpath` unchanged | **PASS** — content digest matches the pre-task value |
| Validated `/home/feras/clearpath_imu001a_test` unchanged | **PASS** — content digest matches the pre-task value; backup comparison also passes |
| Runtime sensor publication, point-cloud fields/rate, and live bridge | **NOT TESTED** — Gazebo was not launched |
| GLIM ingestion or compatibility | **NOT TESTED** — no GLIM launch or adapter was added |

Files changed inside the isolated test copy are only:

- `robot.yaml` — project extension-hook reference.
- `robot.urdf.xacro` — regenerated description containing the extension include.
- `platform/launch/platform-service.launch.py` — setup path redirected to this test copy.
- `sensors/launch/sensors-service.launch.py` — IMU launch path redirected to this test copy.
- `sensors/launch/imu_0.launch.py` — IMU bridge config path redirected to this test copy.

No IMU sensor, bridge, or EKF settings changed, and no private robot configuration was copied into Git. The generated point cloud has no sensor-authentic per-point times. Later source inspection in SLAM-001A established that this GLIM revision synthesizes pseudo-times when the field is absent, so the missing field alone is not a parser blocker; GLIM runtime compatibility remains untested. The SLAM-001A adapter adds explicit zero offsets for a documented instantaneous approximation.

### Reproduction of static checks

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
cd /home/feras/clearpath_ws
colcon build --packages-select husky_a300_sensor_extensions
ros2 run clearpath_generator_common generate_description \
  -s /home/feras/clearpath_lidar11b_test
tmp_urdf=$(mktemp /tmp/lidar11b.XXXXXX.urdf)
trap 'rm -f "$tmp_urdf"' EXIT
xacro /home/feras/clearpath_lidar11b_test/robot.urdf.xacro > "$tmp_urdf"
check_urdf "$tmp_urdf"
ros2 launch husky_a300_sensor_extensions \
  rs_helios_16p_bridge.launch.py --show-args
```

**At LiDAR-001B completion:** The isolated configuration was ready for a separate, controlled Gazebo runtime test using `generate:=false`; publication, bridge operation, and runtime behavior were not yet verified at that point. The subsequent LiDAR-001C results are documented below.

## LiDAR-001C — Combined LiDAR and IMU Runtime Verification

**Date:** 2026-11-15

**Overall result:** PASS for Gazebo sensor creation, GZ-to-ROS cloud publication, PointCloud2 structure and finite measurements, connected sensor TF, existing IMU/odometry streams, manual movement, and user-confirmed RViz visualization. PARTIAL for wall-clock performance: a transient approximately 3.03 s gap was observed while simulation-time cloud stamps continued at approximately 10 Hz. No physical per-point timing is present; the inspected GLIM revision has a pseudo-time fallback, but GLIM ingestion was not tested. No physical RS-Helios-16P fidelity is claimed.

### Launch and preservation

After pre-launch checks and user approval, the combined private copy was launched with the required explicit arguments. No generator, package build, configuration edit, or sensor change was performed during this runtime task. ROS processes used the domain configured in the private test `robot.yaml`; its value is intentionally omitted from documentation and evidence.

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export ROS_DOMAIN_ID="$(python3 -c 'import yaml; d=yaml.safe_load(open("/home/feras/clearpath_lidar11b_test/robot.yaml")); print(d["system"]["ros2"]["domain_id"])')"
export ROS2CLI_DISABLE_DAEMON=1
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_lidar11b_test \
  generate:=false
```

In another sourced shell using the same private domain, the project bridge and existing Clearpath RViz view were started with:

```bash
ros2 launch husky_a300_sensor_extensions rs_helios_16p_bridge.launch.py
ros2 launch clearpath_viz view_robot.launch.py \
  namespace:=a300_00000 use_sim_time:=true config:=robot.rviz
```

The RViz PointCloud2 display was added interactively and not saved to or used to overwrite the installed configuration. The user confirmed that warehouse points were visible and plausible; `ros2 node info` subsequently confirmed RViz subscribed to the expected cloud topic.

### Gazebo sensor and ROS bridge

| Check | Result | Observation |
|---|---|---|
| Warehouse/Husky startup and controllers | **PASS** | The isolated simulation started; startup logs show the Husky and existing controllers. |
| Gazebo GPU-LiDAR creation | **PASS** | Gazebo logged `GpuLidarSensor` advertising `/a300_00000/sensors/helios_16p/points`. |
| Gazebo publisher before starting the project bridge | **PASS** | `gz topic -i` reported one active `gz.msgs.PointCloudPacked` publisher. Three bounded samples had simulation stamps 165.801, 165.900, and 166.002 s; the frame metadata identified `helios_16p_link`. |
| ROS bridge | **PASS** | `rs_helios_16p_bridge` reported GZ-to-ROS mapping from `gz.msgs.PointCloudPacked` to `sensor_msgs/msg/PointCloud2`. The ROS topic had one bridge publisher; QoS was RELIABLE/VOLATILE. |
| ROS PointCloud2 and RViz endpoint | **PASS** | `/a300_00000/sensors/helios_16p/points`; final graph inspection showed one bridge publisher and one RViz subscription. |
| IMU regression | **PASS** | `/a300_00000/sensors/imu_0/data` remained `sensor_msgs/msg/Imu`, with one publisher from `imu_0_gz_bridge`. |
| `data_raw` | **NOT TESTED** | The previously documented IMU-001B limitation remains applicable; this task did not investigate or create a raw stream. |

### PointCloud2 structure and measurements

The ROS cloud had `header.frame_id: helios_16p_link`, simulation-time stamps, width 360, height 16, little-endian data, `point_step: 32`, `row_step: 11520`, and a 184,320-byte data array. `row_step * height` matched the payload length. `is_dense` was false.

| Field | Offset | Datatype | Count |
|---|---:|---|---:|
| x | 0 | FLOAT32 | 1 |
| y | 4 | FLOAT32 | 1 |
| z | 8 | FLOAT32 | 1 |
| intensity | 16 | FLOAT32 | 1 |
| ring | 24 | UINT16 | 1 |

Every field fit within the declared 32-byte point step. The 5,760-point grid contained 5,620 points with finite X, Y, and Z values and 140 non-finite returns. No finite XYZ vector was all zero. Finite point ranges were approximately 0.377 m minimum, 5.207 m median, and 19.9998 m maximum, within the configured synthetic 0.1–20 m range. These values validate the generic simulation output only.

No `t`, `time`, `time_stamp`, or `timestamp` field was present. The cloud has a scan-level timestamp but no verified point-acquisition times. The inspected GLIM TimeKeeper can infer pseudo-times when no field is present; neither that fallback nor GLIM ingestion was runtime-tested in LiDAR-001C. No adapter or time field was added during LiDAR-001C.

### Timing and performance

| Measurement | Result |
|---|---|
| Gazebo point-cloud stamp progression | PASS — bounded pre-bridge samples advanced by 0.099 and 0.102 s. |
| ROS cloud simulation-time rate during the repeat movement capture | PASS — 287 stamps covered 28.599 s of simulation time; observed intervals were 0.099–0.102 s, approximately 10 Hz. |
| ROS cloud wall-clock rate | PARTIAL — normal rolling windows were approximately 9.4–10.0 Hz; one approximately 3.03 s wall-clock gap temporarily reduced rolling estimates to about 6.1–6.3 Hz before recovery. |
| IMU rate | PASS — a bounded standalone check reported approximately 99.2–100.1 Hz. The repeat movement capture received 2,848 angular-velocity samples in its 30 s wall-clock window. |
| Gazebo real-time factor | One sampled world-statistics value was 0.672; this is a point observation, not a benchmark. |

### TF, IMU, and robot regression

Namespaced TF remappings resolved the required transforms. All reported rotations were identity for the static sensor mounts:

| Transform | Translation (m) | RPY (rad) |
|---|---:|---:|
| `base_link → imu_0_link` | `[0, 0, 0.295]` | `[0, 0, 0]` |
| `base_link → helios_16p_link` | `[0, 0, 0.342]` | `[0, 0, 0]` |
| `imu_0_link → helios_16p_link` | `[0, 0, 0.048]` | `[0, 0, 0]` |

The sensor links resolved in the Husky tree, and the cloud frame matched the LiDAR child frame. The translations are simulated and include the provisional mount; they are not physical mounting measurements. `odom → base_link` also resolved after movement. Multiple existing nodes publish the namespaced TF topics; the required lookups resolved, but no separate exhaustive per-frame duplicate-authority audit was performed.

A stationary IMU sample had `header.frame_id: imu_0_link`, an identity orientation quaternion, near-zero angular velocity, and linear acceleration approximately `(0, 0, +9.8) m/s²`. Its stamp (326.322 s) was within 12 ms of the sampled `/clock` value (326.334 s). Covariance arrays were all zero and do not establish sensor accuracy.

The user manually drove forward and turned using the existing keyboard controls, then released the controls. During the repeat monitored movement, IMU angular velocity reached approximately 0.692 rad/s on Z; filtered odometry reached approximately 1.052 m/s on X and 0.510 rad/s yaw. Cloud timestamps continued advancing, and the post-movement point-cloud payload differed from the stationary baseline. Final raw and filtered odometry agreed at approximately `(1.934, 3.786, 0) m`; TF resolved the same position and approximately 30.5° yaw. Final velocities were near zero, consistent with the user reporting that the robot had stopped.

PASS — `/clock`, raw `/a300_00000/platform/odom`, filtered `/a300_00000/platform/odom/filtered`, namespaced TF, and the keyboard teleoperation interface remained available. The test-copy IMU and EKF configuration files matched the Task 11B private baseline; EKF input settings were not changed and IMU fusion was not enabled.

### Warnings and limitations

- Gazebo warned that `gz_frame_id` is not a standard SDF element and copied it as a sensor child. The actual Gazebo/ROS cloud frame was nevertheless observed as `helios_16p_link`.
- The warehouse SDF gravity element was not defined in the expected SDF location; Gazebo copied it into the simulation. The world still loaded.
- The controller manager reported that IMU `imu_0` was not in `hardware_info` and skipped hardware registration for it; the separate Gazebo IMU sensor and ROS bridge remained active.
- The controller update period (0.02 s) was reported slower than the Gazebo simulation period (0.003 s); controller-remapping deprecation and idle velocity-timeout/braking warnings also appeared.
- The joystick process could not open `/dev/input/ps4`; the user successfully used W/A/D keyboard controls.
- The EKF logged missed-update errors under load, including one 0.408 s update and several 0.027–0.039 s updates. No settings were changed. Together with the sampled real-time factor and wall-clock cloud gap, this is a WSL2 runtime-performance limitation requiring a separate performance investigation if needed.
- The first `ros2 topic hz` check warned that the cloud topic did not appear published during discovery; subsequent bounded measurements received data.
- The first RViz launch did not yet subscribe to the cloud until the user added the display. The final ROS graph showed the expected RViz subscription, and the user confirmed the cloud was visible. No screenshot was captured.
- The generic scan profile and mounting pose remain synthetic/provisional; physical sensor accuracy, return behavior, exact channel angles, and hardware fidelity are not tested.

### Evidence and reproduction

Sanitized results are recorded in [LiDAR-001C command evidence](../evidence/command_outputs/2026-11-15-lidar-001c-runtime-validation.txt) and [the concise runtime log](../evidence/logs/2026-11-15-lidar-001c-runtime.log). No full point-cloud payload, private domain value, or network address was saved. During LiDAR-001C, no GLIM launch, installation, adapter, or per-point timestamp modification was performed.

Relevant project-owned files:

- Xacro: `src/husky_a300_sensor_extensions/urdf/rs_helios_16p.urdf.xacro`
- Bridge configuration: `src/husky_a300_sensor_extensions/config/rs_helios_16p_bridge.yaml`
- Bridge launch: `src/husky_a300_sensor_extensions/launch/rs_helios_16p_bridge.launch.py`
- Isolated combined setup: `/home/feras/clearpath_lidar11b_test`

The original `/home/feras/clearpath` setup and validated `/home/feras/clearpath_imu001a_test` setup matched their protected backups after the run. The isolated test was stopped normally after evidence collection; no commit or push was made.
