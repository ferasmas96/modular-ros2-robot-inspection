# SLAM-004C — Partial manual localization validation figures

- **Task / experiment:** SLAM-004C, `slam004c_manual_001`; aborted when the robot center exceeded the reviewed 1.25 m region. No retry.
- **Date and version:** 2026-11-21, `v01`.
- **Source run / data:** Private native MCAP bag and its offline evaluation. Data and bag SHA256 provenance are recorded in `assets.json` and the structured experiment evidence.
- **Produced by / rendering method:** Matplotlib/Agg; [editable plotting source](source/plot_results.py), processed CSVs and [summary](data/summary.json) included. No AI-generated map or occupancy-map rendering.
- **Frames, units and time:** Both trajectories compare `base_link`. Gazebo uses `warehouse`; GLIM uses its local `glim_odom`, transformed offline by one fixed initial-hold SE(3), without scale fitting or TF publication. Metres, simulation seconds, millimetres and degrees are explicitly labelled.
- **Review status:** All three PNGs visually inspected; CSV data/metrics checked against native bag deserialization. Numerical validity applies to the captured interval; the planned final stationary bookend was not recorded.

## Saved files

| Files | Caption / intended documentation use | Editable source |
|---|---|---|
| [Base trajectories PNG](2026-11-21_SLAM-004C_aligned-base-trajectories_v01.png) / [SVG](2026-11-21_SLAM-004C_aligned-base-trajectories_v01.svg) | Same-time aligned trajectories and reviewed center region. Dotted reference tail/red endpoint show continued simulator movement after the stop request, without matching GLIM estimates. This is a trajectory plot, not a map or 3D clearance certificate. | [plot_results.py](source/plot_results.py), [aligned poses](data/aligned_poses.csv), [reference plotting data](data/reference_for_plot.csv) |
| [Pose errors PNG](2026-11-21_SLAM-004C_synchronized-pose-errors_v01.png) / [SVG](2026-11-21_SLAM-004C_synchronized-pose-errors_v01.svg) | Error traces around actual movement. Moving-interval translation ATE RMSE 16.29 mm; orientation error is a separate angular quantity. Red line marks region stop request. | Same script and aligned CSV |
| [Command limits PNG](2026-11-21_SLAM-004C_actual-command-limits_v01.png) / [SVG](2026-11-21_SLAM-004C_actual-command-limits_v01.svg) | Raw keyboard defaults reached 1.0 m/s and 0.5 rad/s; actual guard/mux and controller commands stayed within ±0.10/±0.10. Multiple bursts and combined forward/yaw inputs are visible. | Same script and [command CSV](data/commands_motion_window.csv) |

## Interpretation and limitations

The bag contains 1,554 valid live poses, stamps 3585.501–3740.802 s, all matched
to reference brackets of at most 0.021 s. Alignment uses 101 initial stationary
pairs, 3585.501–3595.500 s. The error/command plots zoom into the actual command
interval 3723.012–3740.829 s, with a short surrounding context. Primary motion
ATE uses 178 poses; one-second motion RPE uses 168 pairs. These are diagnostic
figures for a short synthetic simulation trial, not a general accuracy benchmark.

The native reference extends to 3741.906 s: its final 1.104 s has no matching
GLIM pose. The region crossing occurred at 3740.877 s; 1.029 s / about 0.092 m
of further reference motion was observed during simulator shutdown. Maximum
center radius was 1.3403 m. The planned final stationary hold is unavailable.
Reference plotting CSV uses every fifth native reference plus start/end and the
first region crossing; metrics use all appropriate native records.

Raw keyboard headers are zero, so raw command traces use simulation-time bag
receipt stamps. Delivered commands use header stamps. The command CSV keeps all
records in the selected simulation-time window, including repeated zero samples
after the simulator clock stopped. The simulation ran longer than the short
recording; neither long stationary prefixes nor shutdown tails demonstrate
accuracy. One GLIM IMU-prediction warning block remains undiagnosed. Raw wheel
odometry has 48 duplicate stamps (zero decreases); it is not the comparison
reference. No recorder loss notices were logged, which does not prove zero DDS loss.

## Provenance and reproduction

Original bag, native dump and raw logs remain private under
`/home/feras/glim_ws/runs/mode_a_mapping/slam004c_manual_001`.
The version-controlled [native-bag evaluator](../../../tools/slam_trajectory_eval/README.md)
describes the frame conversion, synchronization and formulas. No existing figure
or previous version was overwritten.

```bash
python3 -B evidence/figures/SLAM-004C/source/plot_results.py --version v02
```

This writes a fresh version from the included processed data and refuses existing
output files. Re-rendered bytes can vary with Matplotlib versions; preserve
earlier exports and record revisions. Full results and limitations are in the
[experiment report](../../command_outputs/2026-11-21-slam-004c-manual-live-pose.txt).
