# SLAM-005B — Stationary native RGB-D map preview

- **Task/run:** SLAM-005B, `slam005b_001`, one separately approved stationary test.
- **Date/version:** 2026-11-21, recommended v02; first v01 layout preserved.
- **Source:** Native `/a300_00000/slam/rtabmap/cloud_map` PointCloud2, simulation
  stamp 484.245 s, 2,411 finite XYZ/color points. Native binary/database stay in
  the private experiment directory. [Provenance](data/provenance.json) records hashes.
- **Method:** Matplotlib/Agg, [editable script](source/plot_native_map.py) and
  [full small exported cloud CSV](data/native_cloud.csv). No synthetic map geometry.
- **Frame/units:** `rtabmap_map`, metres, initially aligned with the stationary
  base frame by the native estimator. Red cross marks that initial origin.
  No transform to Gazebo world, existing odom or GLIM is established.
- **Review:** PNG visually inspected; all exported XYZ values checked finite.

| Export | Caption |
|---|---|
| [PNG v02](2026-11-21_SLAM-005B_stationary-native-map_v02.png) / [SVG v02](2026-11-21_SLAM-005B_stationary-native-map_v02.svg) | Top and side projections of the actual native colored map cloud. Side-view vertical display is expanded and explicitly labelled; coordinates remain metres. Most points describe the visible floor. This documents stationary depth mapping, not map completeness or moving-SLAM accuracy. |
| [PNG v01](2026-11-21_SLAM-005B_stationary-native-map_v01.png) / [SVG v01](2026-11-21_SLAM-005B_stationary-native-map_v01.svg) | Original equal-axis layout, retained with [its source](source/plot_native_map_v01.py). |

The database has 29 stored sensor rows, including 28 negative-weight stationary
records, while observed working/optimized map messages retained one graph node.
Those counts do not represent 29 separate viewpoints. The sensor's limited field
of view and depth clipping remain visible. Database integrity passed after native
graceful saving; no native binary map reload test was performed in this task.
