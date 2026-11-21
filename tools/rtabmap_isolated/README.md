# Isolated RTAB-Map runtime

SLAM-005B uses reviewed Jazzy Debian binaries in a private prefix, with explicit
user approval. It does **not** install them with APT/dpkg or source their setup
globally. Both GTSAM versions have SONAME `libgtsam.so.4`; normal ROS GTSAM 4.2
installation could break the validated GLIM/GTSAM 4.3a1 resolution.

`run_isolated.py` scopes the archive prefix, Python messages and ROS logs to
**only** the RTAB-Map launch. It checks relocations and the selected GTSAM before
execution. Never source that private library environment into GLIM, Gazebo or a
shared component container. Existing GLIM commands/configuration remain unchanged.
Keep `PACKAGE_ARCHIVES.json` with the private installation for version/hash audit.

Build only the new project package, in an isolated build directory:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
colcon --log-base /home/feras/glim_ws/builds/slam005b_001/log build \
  --base-paths /home/feras/clearpath_ws/src/rtabmap_husky_integration \
  --packages-select rtabmap_husky_integration \
  --build-base /home/feras/glim_ws/builds/slam005b_001/build \
  --install-base /home/feras/glim_ws/builds/slam005b_001/install --symlink-install
source /home/feras/glim_ws/builds/slam005b_001/install/setup.bash
```

`prepare_setup.py --source <validated-setup> --destination <new-private-setup>`
refuses existing destinations. It copies settings, rebases absolute generated
launch references in the copy, and adds the camera Xacro invocation. It does not
run Clearpath generators. Do not copy private robot YAML into Git.

The approved mount is the camera bottom screw at base `(0.35,0.18,0.35)` m,
forward-facing, on a fixed post. The native RGB-D sensor is attached to the
**color frame**, so the renderer and declared color optical origin coincide.
RGB and depth share one 640×480 projection at 15 Hz, with 0.3–8 m clipping;
depth is native floating-point meters. Exact synchronization and sensor-data QoS
are explicit. Actual calibration, registration and rates still need runtime checks.

Read-only ELF check, with no ROS nodes:

```bash
/usr/bin/python3 tools/rtabmap_isolated/run_isolated.py \
  --prefix /home/feras/rtabmap_jazzy_isolated/slam005b_001 --check
```

Runtime commands require **separate user approvals**. The commands below record
the completed SLAM-005B run; its paths are consumed and must never be reused.
Choose new reviewed output/setup paths for a future experiment:

The executed run used its private `runtime/environment.sh` to select the existing
ROS domain, sourced Jazzy/workspace/project overlays, and isolated log directory.
Keep that domain configuration private; run the commands in a consistently
configured ROS session. Never apply RTAB's private library path to the simulation
or GLIM environment.

```bash
# Simulation/camera approval required first; no robot driving.
ros2 launch clearpath_gz simulation.launch.py \
  setup_path:=/home/feras/clearpath_rgbd005b_001 world:=warehouse \
  generate:=false rviz:=false x:=-4 y:=3 z:=0.3 yaw:=0
ros2 launch rtabmap_husky_integration camera_bridge.launch.py

# After camera validation, separate RTAB-Map approval required.
/usr/bin/python3 tools/rtabmap_isolated/run_isolated.py \
  --prefix /home/feras/rtabmap_jazzy_isolated/slam005b_001 \
  --database /home/feras/glim_ws/runs/rtabmap/slam005b_001/database/rtabmap.db
```

The mapping launch refuses existing database/sidecar paths and unscoped native
packages. It starts only `rgbd_odometry`, `rtabmap`, and the local pose adapter.
No TF sending, wheel/GLIM/LiDAR/ground-truth estimator inputs, viewer or controller
connection. The mapper uses topic odometry, not `odom_frame_id` TF lookup.
`wait_for_transform=0` avoids waiting for deliberately absent odometry TF; native
lookup/fallback warnings may still occur and must be recorded. A warning is not a
reason to broadcast a second base transform.

The adapter preserves valid base poses/stamps in `rtabmap_odom`, rejects stale,
duplicate, nonfinite or invalid-quaternion input, and publishes best-effort
PoseStamped on `/a300_00000/slam/rtabmap/pose`. Native corrected map localization
is separate. Shutdown mapping gracefully while images/clock remain active, allow
database flush, then stop bridges/simulation and verify cleanup. An empty DB or
stationary success does not prove moving SLAM quality.
