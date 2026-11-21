# Offline live-pose comparison

`evaluate_bag.py` reads a closed MCAP bag through native `rosbag2_py` and ROS CDR
deserialization. It creates no node, TF, command publisher or SLAM instance.
Installed NumPy/SciPy supply SE(3) operations and quaternion interpolation.

The live PoseStamped is already the GLIM **base_link** pose. Reference poses
compose same-stamp `warehouse → a300_00000/robot → .../base_link` records from
the dedicated native Gazebo bridge. Sensor extrinsics are not applied twice.
Reference translation is interpolated linearly and orientation with SLERP at
GLIM header stamps, without extrapolation or brackets above 0.10 s.

One fixed metric SE(3) alignment uses the initial recorded ten-second stationary
hold, including pose orientations. No scale or whole-route best fit is used.
Translation ATE and orientation errors are reported for both the complete paired
window and the interval between first/last actual nonzero controller commands.
One-second RPE accepts sample pairs within 0.03 s and uses
`inverse(inverse(T_ref_i)*T_ref_j) * (inverse(T_est_i)*T_est_j)`.
Wheel odometry validates lack of initial motion; it is not ground truth.

The tool reports matched coverage, message counts versus native metadata,
timestamp gaps, actual command maxima, finite/unit poses, jump checks and file
hashes. An experiment's safety abort/completion status must be reported separately
from numerical validity. Reference data after the last estimate is not evaluated.
Raw GUI command headers can be zero; plotting uses bag receipt simulation time
for those raw messages and header times for delivered commands.

```bash
source /opt/ros/jazzy/setup.bash
python3 -B -m unittest discover -s tools/slam_trajectory_eval -p 'test_*.py' -v
python3 -B tools/slam_trajectory_eval/evaluate_bag.py \
  --bag /home/feras/glim_ws/runs/mode_a_mapping/slam004c_manual_001/bag \
  --output /tmp/slam004c_analysis_review_001
```

The output directory must not exist; existing results are never overwritten.
Four analytic tests check noncommuting frame offsets, SLERP/no extrapolation,
known linear drift/RPE, and preservation of a scale error.

SLAM-004C's [results](../../evidence/command_outputs/2026-11-21-slam-004c-manual-live-pose.txt)
and [figure archive](../../evidence/figures/SLAM-004C/README.md) describe the aborted
run and the valid captured interval. These measurements do not establish general
SLAM accuracy or successful completion of the planned manual sequence.
