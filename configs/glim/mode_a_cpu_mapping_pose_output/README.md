# SLAM-004A isolated pose-output snapshot

Portable review copy only: 16 JSON files, no private paths or runtime logs.
Use the separate operational directory for an authorized runtime, as described in
the [SLAM-004A procedure](../../../docs/experiments/slam004a_pose_output.md).

Compared with the unchanged operational `mode_a_cpu_mapping` baseline:

| File | Exact change |
|---|---|
| `config.json` | Add `global.config_pose_output = "config_pose_output.json"` |
| `config_ros.json` | `extension_modules`: `[]` → `["libglim_pose_output.so"]` |
| `config_logging.json` | Private baseline log directory → portable `logs/glim` |
| `config_pose_output.json` | New `pose_output.T_base_imu = [0,0,0.294759,0,0,0,1]` |

The remaining **12** baseline files are byte-identical. Relative to the existing
portable baseline, the logging value is already `logs/glim`; only the extension
reference/list and the added extrinsic file change semantically. The operational
copy instead uses a unique absolute log directory; both new copies otherwise
match byte-for-byte.

CPU odometry, local/global mapping, input topics, QoS, time settings and exact
`T_lidar_imu` remain unchanged. `publish_imu2lidar=false`; no viewer is enabled.
The new output uses `glim_odom` without broadcasting TF. Existing `odom`/`map`
strings in the retained ROS configuration do not control this extension's frame
and do not cause transforms to be published while viewer extensions are disabled.

The relative logging directory resolves against the process working directory,
not this configuration directory. Do not execute this portable copy directly.
