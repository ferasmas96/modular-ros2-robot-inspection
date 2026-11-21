# SLAM-004C manual live-pose experiment configuration

`manual_input_bridge.yaml` uses the installed native Gazebo Twist-to-ROS
TwistStamped converter, publishing only the isolated manual raw-input topic.
`recording_qos.yaml` lists twelve small measurement topics; no clouds or IMU
payloads are included in this bag. Raw/limited/mux/controller commands are recorded
separately so speed enforcement can be checked from messages.

The existing [reference bridge](../slam003d/ground_truth_bridge.yaml) is reused
unchanged, and its TFMessage data topic must never be remapped into operational
TF. All reference composition/alignment uses message sensor timestamps.

The GLIM settings reuse [the portable live-pose configuration](../../glim/mode_a_cpu_mapping_pose_output/README.md).
The private operational experiment copy differs from the successful SLAM-004B
configuration **only in logging.log_dir**, to protect its previous logs. The
portable setting remains `logs/glim`, resolved from the private run directory.
No second duplicate baseline or private paths are needed in this portable record.

See the [procedure and authorization gates](../../../docs/experiments/slam004c_manual_pose_procedure.md)
and [manual guard](../../../tools/manual_drive_guard/README.md). Preparation alone
does not verify live speed enforcement or authorize movement.

**Recorded outcome:** One user-operated run was stopped by the reviewed 1.25 m
region guard. Actual command limiting passed; captured-motion pose comparison
is valid, with final stationary verification unavailable. All processes are
cleaned up. See [results](../../../evidence/command_outputs/2026-11-21-slam-004c-manual-live-pose.txt).
The original configurations and previous run outputs remain unchanged; no retry.
