# Offline route-clearance inspection

This tool inspects existing files only. It creates no ROS node, simulation, recorder, GLIM object or velocity publisher. It uses installed NumPy/SciPy, xacro and native rosbag2/CDR APIs; no packages are installed. It is an inspection utility, not a motion controller or a general collision-certification library.

It evaluates the existing combined robot description **in memory**, resolving all 16 collision parts and fixed transforms. Continuous wheel rotation does not change the cylinder collision shape. STL/OBJ meshes and box/cylinder geometry are read directly. Wheel cylinders use circumscribed 64-sided bounds. Existing Clearpath generators are never invoked and generated/configuration files are never written.

The installed warehouse world references 25 cached models with 49 collision geometries, plus its ground plane. The tool resolves available local assets without downloading anything. Unknown geometry or unresolved assets fail inspection. The two chair models omit `static`; their nominal poses are recorded as unverified dynamic poses. Ground support and sub-floor warehouse triangles are excluded, and irrelevant high roof triangles are filtered. These exclusions and conservative projections are explicitly recorded; they do not constitute a terrain or complete 3D safety certificate.

The complete adapted-cloud stream is inspected through native rosbag2 reading. The SLAM-003D-B bag contains 957 clouds with one unique XYZ geometry and 5,620 finite unique points; it supplies one stationary viewpoint. World collision geometry covers information the visible cloud does not establish.

All A/B/C paths are checked from their initial pose through translation and rotation. Straight-translation sweeps are independently checked using their exact convex swept polygon. Sampled rotations have a Lipschitz displacement allowance. An explicit illustrative uncertainty budget is also reported; it is not a measured controller-performance guarantee. A support-plane calculation against the standing-person collision mesh gives a separate 3D diagnostic. Its gap is recomputed from unit-normal vertex supports rather than accepted from optimizer status alone. It is not a full-world 3D certificate.

The tool is deliberately tied to the verified stationary origin and model/base relationship of `slam003d_001`; it asserts those saved reference conditions. It refuses an existing output directory. Failed preliminary static-analysis outputs remain private rather than being overwritten.

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B tools/slam_route_clearance/analyze_routes.py \
  --source-run /home/feras/glim_ws/runs/mode_a_mapping/slam003d_001 \
  --robot-xacro /home/feras/clearpath_lidar11b_test/robot.urdf.xacro \
  --world /opt/ros/jazzy/share/clearpath_gz/worlds/warehouse.sdf \
  --fuel-root /home/feras/.gz/fuel/fuel.gazebosim.org \
  --output /home/feras/glim_ws/planning/slam003d_r_001/final_results
```

That output directory now exists; any reproduction must use a new planning directory. This command is offline inspection, not authorization for any runtime test. Full input hashes, geometry arrays and detailed results remain outside Git. See the [route review](../../docs/experiments/slam003d_r_route_review.md) and [sanitized evidence](../../evidence/command_outputs/2026-11-17-slam-003d-r-static-route-review.txt).
