# SLAM-003D-R2 — Turn-first static route evaluation

2026-11-17. **Result: FAIL to verify the unchanged 0.5 m clearance requirement.** No route is selected and no new experiment reservation is created. This is a static analysis; no runtime, recording or movement was performed.

## Proposed sequence

All commands below are proposed values only, using the existing `/a300_00000/cmd_vel` mux input, `geometry_msgs/msg/TwistStamped`, refreshed simulation stamps and 20 Hz publication. All other Twist components remain zero.

1. Remain stationary for at least 10 simulation seconds and successful GLIM initialization.
2. Left yaw `+0.10 rad/s` for `15.7 s`: `+1.57 rad`, approximately 89.95 degrees.
3. Zero command for `2 s`.
4. Forward `+0.10 m/s` for `5 s`: nominal `0.5 m` in the new heading.
5. Zero command/final hold for `8 s`.

Nominal total: 40.7 simulation seconds before discovery/approval gaps. Proposed 20 Hz counts are 315 for the turn, 41 for the intermediate stop, 101 for the forward stage and 161 for the final stop. No messages were published.

## Reconstructed geometry and entire sweep

Reused the preserved [SLAM-003D-R geometry routines](../../tools/slam_route_clearance/analyze_routes.py) through a separate [turn-first caller](../../tools/slam_route_clearance/analyze_turn_first.py); the original utility and all earlier work remain unchanged.

The existing launch uses `(x,y,yaw)=(0,0,0)` with initial spawn Z=0.3 m. The saved stationary reference/identity establishes the settled world/base pose `(0,0,0.13596983616624614)` m, yaw 0, and identity model/base relationship. This measured settled pose is used, rather than treating the airborne launch height as the working pose. No new simulation was needed.

The existing robot xacro is inspected in memory, not regenerated on disk. All **16 collision parts** are included: chassis, wheels, suspension, motors, bumpers and top plate. The conservative full footprint is approximately 0.9954×0.6801 m, with radius 0.5413 m. The existing world/cached assets provide **25 included models / 49 collision geometries**, with model/link/collision transforms and scales applied. Intended ground support and the previously documented irrelevant high roof/support triangles are excluded. Current poses of the two nominally dynamic chair models remain unverified; a complete runtime 3D claim is not made.

Rotation is evaluated throughout `[0,+1.57]` rad at **0.001 rad spacing**, with **1,571 poses**, including both endpoints and local minimum refinement. A conservative radial displacement bound of **0.000270629 m** covers any half-step between rotation samples. Every smaller-turn candidate includes its complete rotation prefix and its exact terminal heading.

For each heading, the full 0.5 m translation is evaluated using the **exact convex swept polygon** formed from the start/end full robot footprints; it covers every intermediate position. A scalar clearance witness is independently checked against that exact sweep. Broad-phase exclusions have AABB separation at least 3.4828 m from an enclosure of every tested swept footprint, so they cannot determine the reported sub-meter minima.

After the 1.57 rad turn, the endpoint is world XY `(0.000398163,0.499999841)` m. The small X component is expected because 1.57 rad is slightly less than pi/2.

## Main route result

| Phase | Minimum conservative projected clearance | Where it occurs | Limiting obstacle |
|---|---:|---|---|
| Entire 0–1.57 rad rotation | **0.493693061 m** | Initial pose: XY `(0,0)`, yaw `0` | `Person 1 - Standing/collision` |
| Entire subsequent 0.5 m forward sweep | **0.516212973 m** | Start of translation: XY `(0,0)`, yaw `1.57` | Same standing-person collision |
| Complete route | **0.493693061 m** | Initial pose | Same standing-person collision |

The limiting model remains `MaleVisitorOnPhone` at world `(1,-1,0)`, yaw 1.57 rad. Its collision geometry differs from the visible LiDAR surface, including upper-body overhang in the full-height projection. The rotation has a continuous projected lower bound of 0.493422432 m after its sampling allowance. The existing explicit illustrative position/yaw/geometry/tilt budget is retained at 0.080824961 m, giving a whole-route adjusted lower bound of 0.412597471 m. This budget is not a measured controller guarantee.

The forward phase alone clears 0.5 m **nominally in the projected model**, but is not independently certified after uncertainty and 3D limitations. It does not erase the initial-pose condition of the complete route. **Overall result: FAIL to verify clearance.**

## Smaller left turns followed by the same 0.5 m forward stage

| Left turn | Minimum over entire rotation | Minimum over subsequent forward stage | Whole-route minimum | Result |
|---:|---:|---:|---:|---|
| 5 degrees | 0.493693 m | 0.385854 m | **0.385854 m** | FAIL |
| 10 degrees | 0.493693 m | 0.447377 m | **0.447377 m** | FAIL |
| 15 degrees | 0.493693 m | 0.502908 m | **0.493693 m** | FAIL |
| 30 degrees | 0.493693 m | 0.629564 m | **0.493693 m** | FAIL |
| 45 degrees | 0.493693 m | 0.690142 m | **0.493693 m** | FAIL |
| 60 degrees | 0.493693 m | 0.680514 m | **0.493693 m** | FAIL |

All are limited by the standing-person collision projection. At 5 degrees the translation minimum occurs about 0.346381 m along the stage, world XY `(0.345063,0.030189)` m. At 10 degrees it occurs at the 0.5 m endpoint, XY `(0.492404,0.086824)` m. Larger tested turns retain the initial-pose minimum over the complete route.

There is also an angle-independent conclusion: **every smaller left-turn route includes the same initial pose**, whose retained conservative projected clearance is already below 0.5 m. Therefore no smaller positive angle can satisfy that criterion over its entire route. This argument covers angles between the listed examples; no margin reduction or exclusion of the initial pose is introduced.

## Projected versus actual 3D clearance

The projected results are conservative. Full robot convexification fills gaps, wheel cylinders use circumscribed bounds, and the person projection includes higher body parts. A failed projected bound is **not proof of a collision or of actual physical 3D separation below 0.5 m**.

A separate diagnostic reused the existing convex 3D support-plane routine for the robot envelope versus the standing person only. At rotation spacing 0.01 rad and forward spacing 0.005 m, its smallest sampled separation lower bounds were:

- Rotation: approximately **0.583320 m**, at yaw 0 / the initial pose.
- Forward phase: approximately **0.585337 m**, at yaw 1.57 / the start of translation.

These are **person-only sampled convex-envelope diagnostics**, not actual full-world continuous 3D clearance measurements or certificates. They are not used to declare this route or any smaller-turn route safe. Actual full-world continuous 3D clearance remains **UNVERIFIED**, including tracking/tilt uncertainty, current dynamic geometry and ground/part assumptions. The 0.5 m requirement is not relaxed or replaced with an incomplete diagnostic.

## Reproduction, preservation and stop

The following command performed offline file inspection only:

```bash
source /opt/ros/jazzy/setup.bash
source /home/feras/clearpath_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B tools/slam_route_clearance/analyze_turn_first.py \
  --source-run /home/feras/glim_ws/runs/mode_a_mapping/slam003d_001 \
  --robot-xacro /home/feras/clearpath_lidar11b_test/robot.urdf.xacro \
  --world /opt/ros/jazzy/share/clearpath_gz/worlds/warehouse.sdf \
  --fuel-root /home/feras/.gz/fuel/fuel.gazebosim.org \
  --output /home/feras/glim_ws/planning/slam003d_r2_001/results
```

That planning output now exists; reproduction must use a fresh output directory. Full source hashes, rotation samples, route witnesses and integrity snapshots remain private there. Planning output is not an experiment/run reservation. No `slam003d_002` directory or new operational GLIM configuration was created. Existing reference bridge, recorder approach and its three previously reported transport losses remain unchanged.

All prior project files and uncommitted modifications are preserved byte-for-byte, including existing documentation and the original geometry utility. Existing datasets, configurations, protected files, checkpoints and `main` are preserved. Branch remains `codex-work`, index empty; no commit/push/reset/stash/clean. No Gazebo, GLIM, recorder, GUI, publisher or movement. [Sanitized evidence](../../evidence/command_outputs/2026-11-17-slam-003d-r2-turn-first.txt). Stop after static analysis; no runtime authorization is requested.
